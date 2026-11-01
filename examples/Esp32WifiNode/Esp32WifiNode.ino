// MicroRIO node on an ESP32 over WiFi.
//
// The same node as Esp32PoeNode, on the radio instead of the cable. The
// library only ever sees an Arduino UDP, so nothing below the network setup
// changes.
//
// The network name and password are NOT in this file. They come from
// wifi_secrets.h beside it, which is never committed:
//
//     #define WIFI_SSID "your network"
//     #define WIFI_PASS "your password"
//
// arduino/flash-wifi-node.sh writes that file into a temporary copy of this
// sketch, builds, uploads and deletes the copy, so the password never lands
// in the project at all.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <MicroRIO.h>

#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#else
#error "No wifi_secrets.h. Build with arduino/flash-wifi-node.sh, or create one (see the top of this file)."
#endif

// A fixed address, so the station's profile can name it. Pick one outside the
// router's DHCP range, or reserve this one for the board's MAC.
IPAddress ip(192, 168, 0, 51);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 0, 1);

WiFiUDP udp;

// telemetry: 2 bool, 3 i32, 5 f32, 1 str | control: 2 bool, 1 i32, 2 f32, 1 cmd
MicroRIO<2, 3, 5, 1, 2, 1, 2, 0, 1> rio(udp);

MicroRIO_TelemBool t_heartbeat, t_limit;
MicroRIO_TelemI32 t_a0, t_uptime, t_mode_echo;
MicroRIO_TelemF32 t_rssi, t_loop_pct, t_sine, t_follow, t_supply;
MicroRIO_TelemStr t_status;

MicroRIO_CtrlBool c_led, c_intake;
MicroRIO_CtrlI32 c_mode;
MicroRIO_CtrlF32 c_drive_x, c_amplitude;
MicroRIO_CtrlCmd c_zero;

// Say when the radio joins and drops, and why it is not up. A node that is
// silent on the network is far easier to diagnose when it says what it sees.
void reportLink() {
    static int8_t was = -2;
    wl_status_t st = WiFi.status();
    int8_t now = (int8_t)st;
    if (now == was) return;
    was = now;
    Serial.print(F("wifi "));
    switch (st) {
        case WL_CONNECTED:
            Serial.print(F("up "));
            Serial.print(WiFi.localIP());
            Serial.print(F(" rssi "));
            Serial.println(WiFi.RSSI());
            break;
        case WL_NO_SSID_AVAIL:
            Serial.println(F("network not found (name wrong, out of range, or 5 GHz only)"));
            break;
        case WL_CONNECT_FAILED:
            Serial.println(F("join refused (password wrong?)"));
            break;
        case WL_CONNECTION_LOST:
            Serial.println(F("lost; rejoining"));
            break;
        default:
            Serial.print(F("down, status "));
            Serial.println((int)st);
    }
}

void setup() {
    Serial.begin(115200);

    WiFi.mode(WIFI_STA);
    // Power saving parks the radio between beacons, which turns a 2 ms round
    // trip into 100 ms and makes the watchdog trip on a healthy link.
    WiFi.setSleep(false);
    WiFi.config(ip, gateway, subnet, dns);
    WiFi.setAutoReconnect(true);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    udp.begin(5005);

    rio.node(2, F("esp32-wifi"));
    rio.rate(50);
    rio.watchdog(200);
    rio.maxFrame(1400);  // plenty of room on this board

    t_heartbeat = rio.telemetry_bool(F("heartbeat"));
    t_limit     = rio.telemetry_bool(F("limit_top"));
    t_a0        = rio.telemetry_i32(F("a0"));
    t_uptime    = rio.telemetry_i32(F("uptime_ms"));
    t_mode_echo = rio.telemetry_i32(F("mode_echo"));
    t_rssi      = rio.telemetry_f32(F("rssi_dbm"));
    t_loop_pct  = rio.telemetry_f32(F("loop_pct"));
    // A waveform of its own, so two boards can be watched against each other.
    t_sine      = rio.telemetry_f32(F("sine"));
    // Whatever drive_x is being told to do, measured back out. With a
    // cross-node link driving drive_x, this is how the far board's reading
    // shows up here - the round trip, end to end.
    t_follow    = rio.telemetry_f32(F("follow"));
    t_supply    = rio.telemetry_f32(F("supply_v"));
    t_status    = rio.telemetry_str(F("status"));

    c_led       = rio.control_bool(F("led"));
    c_intake    = rio.control_bool(F("intake"));
    c_mode      = rio.control_i32(F("mode"));
    c_drive_x   = rio.control_f32(F("drive_x"));
    c_amplitude = rio.control_f32(F("amplitude"));
    c_zero      = rio.control_cmd(F("zero_encoders"));

    // What each control reads as while the station is not being heard from.
    rio.failstate(c_led, false);
    rio.failstate(c_intake, false);
    rio.failstate(c_drive_x, 0.0f);
    rio.failstate(c_mode, 0);

    rio.begin();
}

void loop() {
    rio.loop();
    reportLink();

    const uint32_t now = millis();
    rio.write(t_heartbeat, (now / 500) % 2);
    rio.write(t_a0, analogRead(36));
    rio.write(t_uptime, (int32_t)now);
    rio.write(t_rssi, (float)WiFi.RSSI());
    rio.write(t_status, rio.linked() ? (rio.enabled() ? "enabled" : "linked") : "waiting");

    // Simulated instrument. 0.25 Hz so a five-minute chart shows many cycles.
    float amp = rio.read(c_amplitude);
    if (amp <= 0.0f) amp = 1.0f;
    rio.write(t_sine, amp * sinf((float)now * 0.0005f * TWO_PI));
    // Read the commanded output back, as a real board would read an encoder.
    rio.write(t_follow, rio.read(c_drive_x));
    rio.write(t_mode_echo, rio.read(c_mode));
    // A pack voltage that sags while something is being driven.
    rio.write(t_supply, 12.6f - fabsf(rio.read(c_drive_x)) * 0.8f);
    rio.write(t_limit, rio.read(c_drive_x) > 0.9f);

    if (rio.enabled()) {
        // drive outputs here
    }
    if (rio.fired(c_zero)) {
        // one-shot
    }
}
