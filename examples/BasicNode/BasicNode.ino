// MicroRIO node on an Ethernet shield.
//
// Telemetry is what the GUI displays. Control is what the GUI populates --
// read() returns whatever the station last sent, not anything this board
// measured.

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <MicroRIO.h>

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 0, 50);

EthernetUDP udp;

// Capacity, per direction: telemetry bools/i32/f32/str, control bools/i32/f32/str/cmd.
MicroRIO<2, 2, 2, 1, 2, 0, 1, 0, 1> rio(udp);

MicroRIO_TelemBool t_heartbeat, t_limit;
MicroRIO_TelemI32 t_a0, t_uptime;
MicroRIO_TelemF32 t_volts, t_loop_pct;
MicroRIO_TelemStr t_status;

MicroRIO_CtrlBool c_led, c_intake;
MicroRIO_CtrlF32 c_drive_x;
MicroRIO_CtrlCmd c_zero;

void setup() {
    Ethernet.begin(mac, ip);
    // The W5100 blocks the sketch while it resolves an address, and by
    // default it retries for over a second and a half before giving up. A
    // station that is slow to answer ARP — a laptop asleep on WiFi — would
    // stall this loop every time we sent to it. Give up quickly instead.
    Ethernet.setRetransmissionTimeout(50);
    Ethernet.setRetransmissionCount(1);
    udp.begin(5005);

    rio.node(1, F("drivebase"));
    rio.rate(50);
    rio.watchdog(200);

    t_heartbeat = rio.telemetry_bool(F("heartbeat"));
    t_limit     = rio.telemetry_bool(F("limit_top"));
    t_a0        = rio.telemetry_i32(F("a0"));
    t_uptime    = rio.telemetry_i32(F("uptime_ms"));
    t_volts     = rio.telemetry_f32(F("battery_v"));
    t_loop_pct  = rio.telemetry_f32(F("loop_pct"));
    t_status    = rio.telemetry_str(F("status"));

    c_led       = rio.control_bool(F("led"));
    c_intake    = rio.control_bool(F("intake"));
    c_drive_x   = rio.control_f32(F("drive_x"));
    c_zero      = rio.control_cmd(F("zero_encoders"));

    rio.begin();

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    rio.loop();

    rio.write(t_heartbeat, (millis() / 500) % 2);
    rio.write(t_limit, digitalRead(2) == LOW);
    rio.write(t_a0, analogRead(A0));
    rio.write(t_uptime, (int32_t)millis());
    rio.write(t_volts, analogRead(A1) * (5.0f / 1023.0f));
    rio.write(t_status, rio.linked() ? "linked" : "waiting");

    // Outputs are gated on enabled(); the library has already neutralised if
    // the watchdog tripped or the operator's UI went away.
    if (rio.enabled()) {
        digitalWrite(LED_BUILTIN, rio.read(c_led));
    } else {
        digitalWrite(LED_BUILTIN, LOW);
    }

    if (rio.fired(c_zero)) {
        // one-shot: fires once per station-side increment, even over a lossy link
    }
}
