// MicroRIO conformance harness.
//
// Runs the real protocol against the real library, but over USB serial instead
// of UDP, so the C++ encoder/decoder can be cross-checked against the Rust
// codec with no network involved. Drive it with:
//
//     cargo xtask conformance <port>
//
// Line protocol:
//     ->  FROM <a.b.c.d> <port>   which station the next packets come from
//     ->  STALL <ms>    make every send cost this long, as an unreachable
//                       address does on a real shield
//     ->  RX <hex>      inject a packet as if it arrived over UDP
//     <-  TX <hex>      a packet the library tried to send
//     <-  HASH <hex>    schema hash computed by the firmware
//     <-  VAL <k>=<v>   a decoded control value, for checking the C++ decoder
//     <-  STATE <n> <linked>
//     ->  DUMP          ask for VAL/STATE lines
//     ->  TELEM         force one telemetry frame

#include <Arduino.h>

// This harness's transport is the serial port, so every "send" costs
// milliseconds of printing. Raise the bar the library uses to spot a
// destination that is costing it real time, or the harness itself would
// look like one. The logic under test is the same; only the number moves.
#define MICRORIO_SLOW_SEND_US 60000

#include <MicroRIO.h>

static const uint16_t BUF = 512;

// A UDP that talks to the serial port. Proves the transport abstraction too:
// the library cannot tell this from an Ethernet shield.
class HarnessUDP : public UDP {
   public:
    uint8_t begin(uint16_t) override { return 1; }
    void stop() override {}

    int beginPacket(IPAddress, uint16_t) override { _txLen = 0; return 1; }
    int beginPacket(const char *, uint16_t) override { _txLen = 0; return 1; }
    int endPacket() override {
        if (_stallMs) delay(_stallMs);
        Serial.print(F("TX "));
        for (uint16_t i = 0; i < _txLen; i++) printHex(_tx[i]);
        Serial.println();
        return 1;
    }

    size_t write(uint8_t b) override {
        if (_txLen < BUF) _tx[_txLen++] = b;
        return 1;
    }
    size_t write(const uint8_t *b, size_t n) override {
        for (size_t i = 0; i < n; i++) write(b[i]);
        return n;
    }

    int parsePacket() override {
        if (!_rxPending) return 0;
        _rxPending = false;
        _rxPos = 0;
        return (int)_rxLen;
    }
    int available() override { return (int)(_rxLen - _rxPos); }
    int read() override { return _rxPos < _rxLen ? _rx[_rxPos++] : -1; }
    int read(unsigned char *buf, size_t len) override {
        size_t n = 0;
        while (n < len && _rxPos < _rxLen) buf[n++] = _rx[_rxPos++];
        return (int)n;
    }
    int read(char *buf, size_t len) override { return read((unsigned char *)buf, len); }
    int peek() override { return _rxPos < _rxLen ? _rx[_rxPos] : -1; }
    void flush() override {}
    IPAddress remoteIP() override { return _from; }
    uint16_t remotePort() override { return _fromPort; }

    // Who the next injected packet appears to come from. Two stations on one
    // node is the thing this lets the harness reproduce.
    void from(IPAddress ip, uint16_t port) {
        _from = ip;
        _fromPort = port;
    }

    // What an address the chip cannot resolve costs: the real shield blocks
    // the sketch while it retries, and that is the thing worth testing.
    void stall(uint32_t ms) { _stallMs = ms; }

    void inject(const uint8_t *data, uint16_t n) {
        _rxLen = n > BUF ? BUF : n;
        for (uint16_t i = 0; i < _rxLen; i++) _rx[i] = data[i];
        _rxPos = 0;
        _rxPending = true;
    }

   private:
    static void printHex(uint8_t b) {
        static const char *digits = "0123456789abcdef";
        Serial.print(digits[b >> 4]);
        Serial.print(digits[b & 0x0F]);
    }
    uint32_t _stallMs = 0;
    IPAddress _from{192, 168, 0, 148};
    uint16_t _fromPort = 5006;
    uint8_t _tx[BUF];
    uint16_t _txLen = 0;
    uint8_t _rx[BUF];
    uint16_t _rxLen = 0, _rxPos = 0;
    bool _rxPending = false;
};

HarnessUDP udp;

// Every channel type, both directions.
MicroRIO<3, 2, 2, 1, 2, 1, 2, 1, 2> rio(udp);

MicroRIO_TelemBool tb0, tb1, tb2;
MicroRIO_TelemI32 ti0, ti1;
MicroRIO_TelemF32 tf0, tf1;
MicroRIO_TelemStr ts0;
MicroRIO_CtrlBool cb0, cb1;
MicroRIO_CtrlI32 ci0;
MicroRIO_CtrlF32 cf0, cf1;
MicroRIO_CtrlStr cs0;
MicroRIO_CtrlCmd cc0, cc1;

static char line[1200];
static uint16_t lineLen = 0;
static uint8_t packet[BUF];

static int8_t hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    rio.node(7, F("conformance"));
    rio.rate(50);
    rio.watchdog(200);
    rio.maxFrame(192);  // small on purpose, so the schema must chunk

    tb0 = rio.telemetry_bool(F("hb"));
    tb1 = rio.telemetry_bool(F("limit_top"));
    tb2 = rio.telemetry_bool(F("limit_bottom"));
    ti0 = rio.telemetry_i32(F("a0"));
    ti1 = rio.telemetry_i32(F("uptime_ms"));
    tf0 = rio.telemetry_f32(F("battery_v"));
    tf1 = rio.telemetry_f32(F("loop_pct"));
    ts0 = rio.telemetry_str(F("status"));

    cb0 = rio.control_bool(F("led"));
    cb1 = rio.control_bool(F("intake"));
    ci0 = rio.control_i32(F("target_ticks"));
    cf0 = rio.control_f32(F("drive_x"));
    cf1 = rio.control_f32(F("drive_y"));
    cs0 = rio.control_str(F("mode"));
    cc0 = rio.control_cmd(F("zero_encoders"));
    cc1 = rio.control_cmd(F("reset_fault"));

    // led and drive_x have failstates; drive_y deliberately does not, so the
    // harness can see both behaviours once the watchdog trips.
    rio.failstate(cb0, false);
    rio.failstate(cf0, -0.5f);

    rio.begin();

    // Fixed telemetry values so the Rust side can assert on them.
    rio.write(tb0, true);
    rio.write(tb1, false);
    rio.write(tb2, true);
    rio.write(ti0, 1234);
    rio.write(ti1, -98765);
    rio.write(tf0, 12.5f);
    rio.write(tf1, -0.375f);
    rio.write(ts0, "hello harness");

    Serial.print(F("HASH "));
    Serial.println(rio.schemaHash(), HEX);
    Serial.println(F("READY"));
}

void handleLine() {
    if (lineLen >= 3 && strncmp(line, "RX ", 3) == 0) {
        uint16_t n = 0;
        for (uint16_t i = 3; i + 1 < lineLen && n < BUF; i += 2) {
            int8_t hi = hexVal(line[i]), lo = hexVal(line[i + 1]);
            if (hi < 0 || lo < 0) break;
            packet[n++] = (uint8_t)((hi << 4) | lo);
        }
        udp.inject(packet, n);
        rio.loop();
        Serial.println(F("OK"));
        return;
    }

    if (lineLen >= 5 && strncmp(line, "FROM ", 5) == 0) {
        int a, b, c, d, port;
        if (sscanf(line + 5, "%d.%d.%d.%d %d", &a, &b, &c, &d, &port) == 5) {
            udp.from(IPAddress(a, b, c, d), (uint16_t)port);
            Serial.println(F("OK"));
        } else {
            Serial.println(F("ERR"));
        }
        return;
    }

    if (lineLen >= 6 && strncmp(line, "STALL ", 6) == 0) {
        udp.stall((uint32_t)atol(line + 6));
        Serial.println(F("OK"));
        return;
    }

    if (lineLen >= 4 && strncmp(line, "DUMP", 4) == 0) {
        Serial.print(F("VAL led=")); Serial.println(rio.read(cb0) ? 1 : 0);
        Serial.print(F("VAL intake=")); Serial.println(rio.read(cb1) ? 1 : 0);
        Serial.print(F("VAL target_ticks=")); Serial.println(rio.read(ci0));
        Serial.print(F("VAL drive_x=")); Serial.println(rio.read(cf0), 6);
        Serial.print(F("VAL drive_y=")); Serial.println(rio.read(cf1), 6);
        Serial.print(F("VAL mode=")); Serial.println(rio.read(cs0));
        Serial.print(F("VAL fired_zero=")); Serial.println(rio.fired(cc0) ? 1 : 0);
        Serial.print(F("VAL fired_reset=")); Serial.println(rio.fired(cc1) ? 1 : 0);
        Serial.print(F("STATE ")); Serial.print(rio.state());
        Serial.print(' '); Serial.println(rio.linked() ? 1 : 0);
        Serial.println(F("OK"));
        return;
    }

    if (lineLen >= 5 && strncmp(line, "TELEM", 5) == 0) {
        forceTelemetry();
        Serial.println(F("OK"));
        return;
    }

    Serial.println(F("ERR"));
}

// The library sends on its own schedule; for a deterministic test we want one
// frame exactly when asked.
void forceTelemetry() { rio.sendNow(); }

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            line[lineLen] = 0;
            if (lineLen) handleLine();
            lineLen = 0;
        } else if (c != '\r' && lineLen < sizeof(line) - 1) {
            line[lineLen++] = c;
        }
    }
}
