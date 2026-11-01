// MicroRIO simulation node for the Mega.
//
// Generates synthetic waveforms so the driver station has something worth
// graphing, alongside real analog reads and a DS3231 RTC on the Mega's I2C
// pins (20 = SDA, 21 = SCL).
//
// Time handling:
//   unix_time / rtc_time   telemetry, straight off the RTC
//   set_unix + sync_time   control: the station puts a unix timestamp in
//                          set_unix, then bumps the sync_time counter. The
//                          counter is why this survives a dropped packet --
//                          a boolean pulse would simply be lost.

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Wire.h>
#include <RTClib.h>
#include <MicroRIO.h>

byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 0, 50);

EthernetUDP udp;
RTC_DS3231 rtc;
bool rtcPresent = false;

// telemetry: 3 bool, 5 i32, 6 f32, 2 str | control: 2 bool, 2 i32, 3 f32, 2 cmd
MicroRIO<3, 5, 6, 2, 2, 2, 3, 0, 2> rio(udp);

MicroRIO_TelemBool t_hb, t_rtc_ok, t_limit;
MicroRIO_TelemI32 t_a0, t_uptime_s, t_unix, t_loop_us, t_mode_echo;
MicroRIO_TelemF32 t_sine, t_ramp, t_square, t_noise, t_temp_c, t_battery;
MicroRIO_TelemStr t_time, t_status;

MicroRIO_CtrlBool c_led, c_sim_hold;
MicroRIO_CtrlI32 c_set_unix, c_mode;
MicroRIO_CtrlF32 c_freq_hz, c_amplitude, c_setpoint;
MicroRIO_CtrlCmd c_sync_time, c_reset_ramp;

const int LED_PIN = 13;
unsigned long rampStartMs = 0;
uint32_t noiseSeed = 0x1234567uL;

// Cheap deterministic noise: no rand(), no heap, same sequence every boot.
float nextNoise() {
    noiseSeed ^= noiseSeed << 13;
    noiseSeed ^= noiseSeed >> 17;
    noiseSeed ^= noiseSeed << 5;
    return ((int32_t)(noiseSeed & 0xFFFF) - 32768) / 32768.0f;
}

void setup() {
    pinMode(LED_PIN, OUTPUT);

    // Unused analog inputs are left as inputs on purpose: A0 is deliberately
    // left floating so the example still shows a live ADC read. Tie any pin
    // you are not using to GND if you would rather it read a solid zero.


    Wire.begin();              // Mega: SDA = 20, SCL = 21
    rtcPresent = rtc.begin();
    if (rtcPresent && rtc.lostPower()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    Ethernet.begin(mac, ip);
    // The W5100 blocks the sketch while it resolves an address, and by
    // default it retries for over a second and a half before giving up. A
    // station that is slow to answer ARP — a laptop asleep on WiFi — would
    // stall this loop every time we sent to it. Give up quickly instead.
    Ethernet.setRetransmissionTimeout(50);
    Ethernet.setRetransmissionCount(1);
    udp.begin(5005);

    rio.node(1, F("mega-sim"));
    rio.rate(50);
    rio.watchdog(200);

    t_hb       = rio.telemetry_bool(F("heartbeat"));
    t_rtc_ok   = rio.telemetry_bool(F("rtc_ok"));
    t_limit    = rio.telemetry_bool(F("limit_top"));

    t_a0       = rio.telemetry_i32(F("a0"));
    t_uptime_s = rio.telemetry_i32(F("uptime_s"));
    t_unix     = rio.telemetry_i32(F("unix_time"));
    t_loop_us  = rio.telemetry_i32(F("loop_us"));
    // A plain integer the station can give names to: set it from a dropdown,
    // watch the board report it back as a label.
    t_mode_echo = rio.telemetry_i32(F("mode_echo"));

    t_sine     = rio.telemetry_f32(F("sine"));
    t_ramp     = rio.telemetry_f32(F("ramp"));
    t_square   = rio.telemetry_f32(F("square"));
    t_noise    = rio.telemetry_f32(F("noise"));
    t_temp_c   = rio.telemetry_f32(F("rtc_temp_c"));
    t_battery  = rio.telemetry_f32(F("battery_v"));

    t_time     = rio.telemetry_str(F("rtc_time"));
    t_status   = rio.telemetry_str(F("status"));

    c_led        = rio.control_bool(F("led"));
    c_sim_hold   = rio.control_bool(F("sim_hold"));
    c_set_unix   = rio.control_i32(F("set_unix"));
    c_mode       = rio.control_i32(F("mode"));
    c_freq_hz    = rio.control_f32(F("freq_hz"));
    c_amplitude  = rio.control_f32(F("amplitude"));
    c_setpoint   = rio.control_f32(F("setpoint"));
    c_sync_time  = rio.control_cmd(F("sync_time"));
    c_reset_ramp = rio.control_cmd(F("reset_ramp"));

    rio.begin();
    rampStartMs = millis();
}

void loop() {
    unsigned long loopStart = micros();
    rio.loop();

    const unsigned long now = millis();

    // --- waveform parameters come from the station -------------------------
    // Defaults are deliberate: an untouched control sits at zero, which would
    // give a flat line, so fall back to something visible.
    float freq = rio.read(c_freq_hz);
    if (freq <= 0.0f) freq = 0.2f;
    float amp = rio.read(c_amplitude);
    if (amp == 0.0f) amp = 1.0f;

    // Untouched controls sit at zero, so zero has to mean "running" --
    // otherwise a fresh station shows flat lines and looks broken.
    const bool running = !rio.read(c_sim_hold);

    // Advance by real elapsed time, not per iteration. loop() free-runs at
    // well over a kilohertz here, so assuming a 20 ms tick ran the waveforms
    // about 27x too fast.
    static unsigned long lastPhaseUs = 0;
    const unsigned long nowUs = micros();
    float dt = lastPhaseUs ? (nowUs - lastPhaseUs) / 1000000.0f : 0.0f;
    lastPhaseUs = nowUs;
    if (dt < 0.0f || dt > 0.5f) dt = 0.0f;   // first pass, or a micros() wrap

    static float phase = 0.0f;
    if (running) {
        phase += 2.0f * PI * freq * dt;
        while (phase > 2.0f * PI) phase -= 2.0f * PI;
    }

    rio.write(t_sine, amp * sin(phase));
    rio.write(t_square, amp * (sin(phase) >= 0 ? 1.0f : -1.0f));
    rio.write(t_noise, amp * nextNoise() * 0.15f);

    float ramp = ((now - rampStartMs) % 4000UL) / 4000.0f;   // 0..1 over 4 s
    rio.write(t_ramp, amp * (ramp * 2.0f - 1.0f));

    // --- real hardware -----------------------------------------------------
    // A0 is a genuine ADC read. With nothing wired to it the pin is FLOATING,
    // and a floating CMOS input has no defined voltage: it drifts to wherever
    // leakage and stray capacitance leave it, typically somewhere mid-rail.
    // Measured on this board it sits near 580 counts and barely moves, which
    // looks convincingly like a real signal and is not one. Wire something to
    // A0, or tie it to GND, before believing this number.
    rio.write(t_a0, analogRead(A0));
    rio.write(t_limit, digitalRead(2) == LOW);

    // battery_v is SIMULATED, not measured. It used to read A1, which has
    // nothing connected either -- two floating pins settle to nearly the same
    // voltage because they share a package and an environment, so it looked
    // like a real reading that agreed with A0. A simulated pack is honest
    // about what it is and more useful to watch on the chart.
    {
        const float nominal = 12.6f;
        const float sag = 0.8f * (0.5f + 0.5f * sin(phase));   // load following the sim
        rio.write(t_battery, nominal - sag + nextNoise() * 0.02f);
    }
    rio.write(t_hb, (now / 500) % 2);
    rio.write(t_uptime_s, (int32_t)(now / 1000));

    // --- RTC ---------------------------------------------------------------
    static unsigned long lastRtcMs = 0;
    static char timeBuf[20];
    if (rtcPresent && now - lastRtcMs >= 250) {
        lastRtcMs = now;
        DateTime dt = rtc.now();
        snprintf(timeBuf, sizeof(timeBuf), "%04d-%02d-%02d %02d:%02d:%02d",
                 dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
        rio.write(t_time, timeBuf);
        rio.write(t_unix, (int32_t)dt.unixtime());
        rio.write(t_temp_c, rtc.getTemperature());
    } else if (!rtcPresent) {
        rio.write(t_time, "no rtc");
        rio.write(t_unix, 0);
    }
    rio.write(t_rtc_ok, rtcPresent);

    // Operator time sync. Fires once per station-side increment.
    if (rio.fired(c_sync_time)) {
        int32_t want = rio.read(c_set_unix);
        if (rtcPresent && want > 1000000000L) {   // sanity: after 2001
            rtc.adjust(DateTime((uint32_t)want));
        }
    }
    if (rio.fired(c_reset_ramp)) {
        rampStartMs = now;
        phase = 0.0f;
    }

    rio.write(t_status, rio.linked() ? (rio.enabled() ? "enabled" : "linked") : "waiting");

    // --- outputs are gated, always ----------------------------------------
    digitalWrite(LED_PIN, rio.enabled() && rio.read(c_led));

    rio.write(t_mode_echo, rio.read(c_mode));
    rio.write(t_loop_us, (int32_t)(micros() - loopStart));
}
