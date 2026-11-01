// MicroRIO node on an Olimex ESP32-POE-ISO.
//
// Same library, same protocol, different transport: the sketch owns the
// network, the library only ever sees an Arduino UDP.

#include <ETH.h>
#include <WiFiUdp.h>
#include <MicroRIO.h>

// Olimex ESP32-POE / POE-ISO RMII wiring.
#define OLIMEX_PHY_TYPE  ETH_PHY_LAN8720
#define OLIMEX_PHY_ADDR  0
#define OLIMEX_PHY_MDC   23
#define OLIMEX_PHY_MDIO  18
#define OLIMEX_PHY_POWER 12
#define OLIMEX_CLK_MODE  ETH_CLOCK_GPIO17_OUT

IPAddress ip(192, 168, 0, 51);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(192, 168, 0, 1);

WiFiUDP udp;

MicroRIO<2, 2, 2, 1, 2, 0, 1, 0, 1> rio(udp);

MicroRIO_TelemBool t_heartbeat, t_limit;
MicroRIO_TelemI32 t_a0, t_uptime;
MicroRIO_TelemF32 t_volts, t_loop_pct;
MicroRIO_TelemStr t_status;

MicroRIO_CtrlBool c_led, c_intake;
MicroRIO_CtrlF32 c_drive_x;
MicroRIO_CtrlCmd c_zero;

void setup() {
    Serial.begin(115200);

    ETH.begin(OLIMEX_PHY_TYPE, OLIMEX_PHY_ADDR, OLIMEX_PHY_MDC, OLIMEX_PHY_MDIO,
              OLIMEX_PHY_POWER, OLIMEX_CLK_MODE);
    ETH.config(ip, gateway, subnet, dns);
    udp.begin(5005);

    rio.node(2, F("esp32-poe"));
    rio.rate(50);
    rio.watchdog(200);
    rio.maxFrame(1400);  // plenty of room on this board

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
}

// Say when the cable comes and goes. A board that is silent on the network
// is far easier to diagnose when it says whether it ever had a link.
void reportLink() {
    static int8_t was = -1;
    int8_t up = ETH.linkUp() ? 1 : 0;
    if (up == was) return;
    was = up;
    Serial.print(F("eth "));
    if (up) {
        Serial.print(F("up "));
        Serial.print(ETH.localIP());
        Serial.print(' ');
        Serial.print(ETH.linkSpeed());
        Serial.println(F(" Mbps"));
    } else {
        Serial.println(F("down (no cable, or no switch at the other end)"));
    }
}

void loop() {
    rio.loop();
    reportLink();

    rio.write(t_heartbeat, (millis() / 500) % 2);
    rio.write(t_a0, analogRead(36));
    rio.write(t_uptime, (int32_t)millis());
    rio.write(t_status, rio.linked() ? "linked" : "waiting");

    if (rio.enabled()) {
        // drive outputs here
    }
    if (rio.fired(c_zero)) {
        // one-shot
    }
}
