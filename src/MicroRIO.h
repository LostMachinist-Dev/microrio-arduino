// MicroRIO - named telemetry and control over UDP.
//
// Declare channels by name once; the station negotiates the schema at connect
// and both sides then exchange fixed binary frames addressed by slot. Names
// never appear in a control or telemetry frame.
//
//   Telemetry  node -> station.  The GUI displays it.
//   Control    station -> node.  The GUI populates it.
//
// "read" on this side means read what the station sent. Control values do not
// come from the board's own hardware; something in the GUI has to drive them.
//
// Capacity is fixed at compile time so you pay only for what you declare.
// Nothing is allocated, no String is used, and on an Uno frames stream straight
// into the socket rather than being staged in RAM - which is what makes this
// fit a 2 KB board.
//
// Every telemetry sample is numbered. Boards with RAM to spare (Mega, ESP32)
// keep the last few and resend any the station has not acknowledged, so a
// dropped frame leaves no hole in the log. Control is never resent: it is
// full state every tick, and a late command is worse than a lost one.
//
// A node answers to one station at a time. The first to send control owns it;
// another station's control is refused, and told so, until the owner falls
// silent for longer than the watchdog. Anyone may ask for the schema.
//
// Other stations watch through the owner's station, which relays to them, so
// the board never sends to an address it had no reason to talk to. A watcher
// may still stop the machine: that is listened for, and listening is free.

#ifndef MICRORIO_H
#define MICRORIO_H

#include <Arduino.h>
#include <Udp.h>

#include "MicroRIO_protocol.h"

#if defined(__AVR__)
#include <avr/pgmspace.h>
#define MICRORIO_DEFAULT_FRAME 512
#else
#include <pgmspace.h>
#define MICRORIO_DEFAULT_FRAME MICRORIO_DEFAULT_MAX_FRAME
#endif

// A send that takes longer than this is not a working link. Sending to an
// address the chip cannot resolve blocks the whole sketch while it retries
// ARP - on a W5100 that is 200 ms eight times over, so better than a second
// and a half with nothing else running. Any destination that costs this much
// is dropped rather than paid for again.
//
// Twenty milliseconds sits well clear of a healthy send - under a
// millisecond on a wired shield, a couple on WiFi - and well under what a
// failed address costs even with the retries turned down. Override it for a
// transport that is slow by nature, such as a test harness on a serial port.
#ifndef MICRORIO_SLOW_SEND_US
#define MICRORIO_SLOW_SEND_US 20000
#endif
// How long a destination that did that is left alone for.
#ifndef MICRORIO_QUIET_MS
#define MICRORIO_QUIET_MS 5000
#endif

// How many telemetry records to keep for resending. A 2 KB board keeps none:
// it streams each frame and gets loss measurement only. Override by passing
// the tenth template argument.
#if defined(__AVR__) && defined(RAMEND) && (RAMEND < 0x1000)
#define MICRORIO_SMALL_RAM 1
#else
#define MICRORIO_SMALL_RAM 0
#endif

// ---------------------------------------------------------------- handles
//
// Declaring a channel returns a typed handle. Using handles instead of names
// at runtime keeps the hot path O(1), and avoids the trap where a bare string
// literal lands in SRAM on AVR.

struct MicroRIO_TelemBool { uint8_t slot; };
struct MicroRIO_TelemI32  { uint8_t slot; };
struct MicroRIO_TelemF32  { uint8_t slot; };
struct MicroRIO_TelemStr  { uint8_t slot; };
struct MicroRIO_CtrlBool  { uint8_t slot; };
struct MicroRIO_CtrlI32   { uint8_t slot; };
struct MicroRIO_CtrlF32   { uint8_t slot; };
struct MicroRIO_CtrlStr   { uint8_t slot; };
struct MicroRIO_CtrlCmd   { uint8_t slot; };

namespace microrio_detail {

inline uint8_t flashByte(const char *p) { return pgm_read_byte(p); }

inline uint8_t flashLen(const __FlashStringHelper *s) {
    const char *p = reinterpret_cast<const char *>(s);
    uint8_t n = 0;
    while (flashByte(p + n) != 0 && n < MICRORIO_MAX_NAME_LEN) n++;
    return n;
}

inline uint32_t fnvFeed(uint32_t h, uint8_t b) {
    h ^= (uint32_t)b;
    return h * MICRORIO_FNV_PRIME;
}

// Width of one stored telemetry record at full declared capacity.
constexpr uint16_t recordBytes(uint8_t tb, uint8_t ti, uint8_t tf, uint8_t ts) {
    return MICRORIO_RECORD_FIXED + (tb + 7) / 8 + ti * 4 + tf * 4 + ts * MICRORIO_STR_LEN;
}

// Default history: as many records as fit one default-sized frame, capped at
// the spec default. Keeping more than one frame can carry would be RAM spent
// on records that can never be sent.
constexpr uint8_t defaultHistory(uint8_t tb, uint8_t ti, uint8_t tf, uint8_t ts) {
    return MICRORIO_SMALL_RAM ? 1
         : (MICRORIO_DEFAULT_FRAME - MICRORIO_HEADER_LEN - MICRORIO_TELEM_FIXED) /
                       recordBytes(tb, ti, tf, ts) >= MICRORIO_DEFAULT_HISTORY
             ? MICRORIO_DEFAULT_HISTORY
         : (MICRORIO_DEFAULT_FRAME - MICRORIO_HEADER_LEN - MICRORIO_TELEM_FIXED) /
                       recordBytes(tb, ti, tf, ts) < 1
             ? 1
             : (MICRORIO_DEFAULT_FRAME - MICRORIO_HEADER_LEN - MICRORIO_TELEM_FIXED) /
                   recordBytes(tb, ti, tf, ts);
}

}  // namespace microrio_detail

// ------------------------------------------------------------------ class
//
// Template parameters are the per-direction capacities:
//   TB,TI,TF,TS  telemetry bools, i32, f32, strings
//   CB,CI,CF,CS,CC  control bools, i32, f32, strings, commands
//   HIST  telemetry records kept for resending; 1 = none. Optional.
//
// Canonical channel order - which fixes both the schema and the payload
// layout - is exactly that listing order. The station reproduces it from the
// schema it receives, so the two sides cannot disagree.

template <uint8_t TB, uint8_t TI, uint8_t TF, uint8_t TS,
          uint8_t CB, uint8_t CI, uint8_t CF, uint8_t CS, uint8_t CC,
          uint8_t HIST = microrio_detail::defaultHistory(TB, TI, TF, TS)>
class MicroRIO {
    static_assert(HIST >= 1, "HIST must be at least 1");

   public:
    explicit MicroRIO(UDP &udp) : _udp(udp) { _nodeName = F("node"); }

    // ---------------------------------------------------------- setup

    void node(uint16_t id, const __FlashStringHelper *name) {
        _nodeId = id;
        _nodeName = name;
    }
    /// The sketch's telemetry rate. The station may ask for a different one
    /// at runtime; this is what the node falls back to when it does not.
    void rate(uint16_t hz) { _sketchPeriodMs = _periodMs = periodFor(hz); }
    void watchdog(uint16_t ms) { _watchdogMs = ms; }
    void maxFrame(uint16_t bytes) {
        _maxFrame = bytes < MICRORIO_MIN_MAX_FRAME ? MICRORIO_MIN_MAX_FRAME : bytes;
    }

    // Declarations. Wrap every name in F() so it stays in flash.
    MicroRIO_TelemBool telemetry_bool(const __FlashStringHelper *n) { return {push(_tbName, _nTB, TB, n)}; }
    MicroRIO_TelemI32  telemetry_i32 (const __FlashStringHelper *n) { return {push(_tiName, _nTI, TI, n)}; }
    MicroRIO_TelemF32  telemetry_f32 (const __FlashStringHelper *n) { return {push(_tfName, _nTF, TF, n)}; }
    MicroRIO_TelemStr  telemetry_str (const __FlashStringHelper *n) { return {push(_tsName, _nTS, TS, n)}; }
    MicroRIO_CtrlBool  control_bool  (const __FlashStringHelper *n) { return {push(_cbName, _nCB, CB, n)}; }
    MicroRIO_CtrlI32   control_i32   (const __FlashStringHelper *n) { return {push(_ciName, _nCI, CI, n)}; }
    MicroRIO_CtrlF32   control_f32   (const __FlashStringHelper *n) { return {push(_cfName, _nCF, CF, n)}; }
    MicroRIO_CtrlStr   control_str   (const __FlashStringHelper *n) { return {push(_csName, _nCS, CS, n)}; }
    MicroRIO_CtrlCmd   control_cmd   (const __FlashStringHelper *n) { return {push(_ccName, _nCC, CC, n)}; }

    /// Freeze the schema and compute its hash. Call after all declarations.
    void begin() {
        _schemaHash = computeHash();
        // Never claim more history than one frame can carry back.
        uint16_t room = _maxFrame - MICRORIO_HEADER_LEN - MICRORIO_TELEM_FIXED;
        uint16_t fit = room / (MICRORIO_RECORD_FIXED + telemPayloadLen());
        _history = fit < 1 ? 1 : (fit < HIST ? (uint8_t)fit : HIST);
        _lastControlMs = millis();
        _everLinked = false;
    }

    uint32_t schemaHash() const { return _schemaHash; }

    // ----------------------------------------------------------- runtime

    /// Service the link: drain inbound packets, run the watchdog, send when due.
    void loop() {
        receive();

        const uint32_t now = millis();
        // Three expensive sends in a row and the station is not really there:
        // drop to five a second until one of them is cheap again.
        const uint16_t period = _slowOwner >= 3 ? 200u : _periodMs;
        if (now - _lastSendMs >= period) {
            _lastSendMs = now;
            if (_haveStation) sendTelemetry();
        }
    }

    /// Send one telemetry frame now, outside the schedule. The record counts
    /// like any other.
    void sendNow() {
        if (_haveStation) sendTelemetry();
    }

    // Telemetry writers.
    void write(MicroRIO_TelemBool h, bool v)  { if (h.slot < TB) _tb[h.slot] = v; }
    void write(MicroRIO_TelemI32 h, int32_t v){ if (h.slot < TI) _ti[h.slot] = v; }
    void write(MicroRIO_TelemF32 h, float v)  { if (h.slot < TF) _tf[h.slot] = v; }
    void write(MicroRIO_TelemStr h, const char *v) {
        if (h.slot >= TS || !v) return;
        uint8_t i = 0;
        for (; i < MICRORIO_STR_LEN - 1 && v[i]; i++) _ts[h.slot][i] = v[i];
        _ts[h.slot][i] = 0;
    }

    // Control readers. These return what the station last sent - or, while
    // the station is not being heard from, the channel's failstate if it has one.
    bool read(MicroRIO_CtrlBool h) const {
        if (h.slot >= CB) return false;
        return (_cbHasFail[h.slot] && !linked()) ? _cbFail[h.slot] : _cb[h.slot];
    }
    int32_t read(MicroRIO_CtrlI32 h) const {
        if (h.slot >= CI) return 0;
        return (_ciHasFail[h.slot] && !linked()) ? _ciFail[h.slot] : _ci[h.slot];
    }
    float read(MicroRIO_CtrlF32 h) const {
        if (h.slot >= CF) return 0.0f;
        return (_cfHasFail[h.slot] && !linked()) ? _cfFail[h.slot] : _cf[h.slot];
    }

    /// What a control reads as while the station's watchdog is tripped, or
    /// before the station has ever been heard. Set them together in setup()
    /// and every safe value is in one place. A control with no failstate keeps
    /// reading the last value the station sent; nothing is decided for you.
    void failstate(MicroRIO_CtrlBool h, bool v) {
        if (h.slot < CB) { _cbFail[h.slot] = v; _cbHasFail[h.slot] = true; }
    }
    void failstate(MicroRIO_CtrlI32 h, int32_t v) {
        if (h.slot < CI) { _ciFail[h.slot] = v; _ciHasFail[h.slot] = true; }
    }
    void failstate(MicroRIO_CtrlF32 h, float v) {
        if (h.slot < CF) { _cfFail[h.slot] = v; _cfHasFail[h.slot] = true; }
    }
    const char *read(MicroRIO_CtrlStr h) const { return h.slot < CS ? _cs[h.slot] : ""; }

    /// True once per station-side increment. Idempotent and loss tolerant:
    /// a dropped packet delays the event, it does not lose it.
    bool fired(MicroRIO_CtrlCmd h) {
        if (h.slot >= CC) return false;
        if (_cc[h.slot] == _ccSeen[h.slot]) return false;
        _ccSeen[h.slot] = _cc[h.slot];
        return true;
    }

    // ------------------------------------------------------------ state

    /// True when no control frame has arrived inside the watchdog window.
    ///
    /// Evaluated live rather than cached, so the answer cannot go stale between
    /// calls to loop(). A safety gate that depends on being serviced recently
    /// is not a safety gate.
    bool watchdogExpired() const {
        return _everLinked && (uint32_t)(millis() - _lastControlMs) > _watchdogMs;
    }

    /// Effective state. The watchdog overrides whatever was commanded, and a
    /// watcher's estop overrides everything: it is held for two seconds and
    /// refreshed while that station keeps asking, so letting go of it hands
    /// the machine back rather than leaving it stopped for ever.
    uint8_t state() const {
        if (_viewerEstopMs && (uint32_t)(millis() - _viewerEstopMs) < 2000u) {
            return MICRORIO_STATE_ESTOP;
        }
        if (!_everLinked || watchdogExpired()) return MICRORIO_STATE_FAULT;
        return _commanded;
    }
    /// The only gate user code needs before driving an output. Safe to call at
    /// any point in the sketch, serviced or not.
    bool enabled() const { return state() == MICRORIO_STATE_ENABLED; }
    bool linked() const { return _everLinked && !watchdogExpired(); }
    /// The station-to-node watchdog: true while control frames keep arriving.
    /// False means lost comms or a station that has stopped - go safe.
    bool watchdogOK() const { return linked(); }
    /// Milliseconds since the last valid control frame.
    uint32_t controlAgeMs() const { return millis() - _lastControlMs; }

    /// True while the node is holding back because sending costs too much.
    /// Something between here and the station is not answering.
    bool slowLink() const { return _slowOwner >= 3; }

    /// True when another station has recently tried to control this node.
    /// It was refused; this is so a sketch can say so on a local display.
    bool stationConflict() const {
        return _conflictMs && (uint32_t)(millis() - _conflictMs) < 2000u;
    }

    // ------------------------------------------------------------ link

    /// Number of the newest telemetry record. Counts up from 1 at boot.
    uint32_t record() const { return _record; }
    /// Records this node can resend, newest included. 1 means no backfill.
    uint8_t history() const { return _history; }
    /// The telemetry rate in force: the station's request, else the sketch's.
    uint16_t telemetryRateHz() const { return _periodMs ? (uint16_t)(1000u / _periodMs) : 0; }
    /// Percentage of the station's control frames that never arrived, from
    /// gaps in their sequence numbers. 0 until there is enough to judge.
    uint8_t linkLossPct() const {
        if (_ctrlExpected < 20) return 0;
        uint16_t got = _ctrlGot > _ctrlExpected ? _ctrlExpected : _ctrlGot;
        return (uint8_t)(100u - (uint32_t)got * 100u / _ctrlExpected);
    }
    /// Milliseconds from a telemetry record leaving to the station saying it
    /// has it. 0 until measured.
    uint16_t linkLatencyMs() const { return _latencyMs; }

   private:
    // ------------------------------------------------------- declaration

    static uint8_t push(const __FlashStringHelper **arr, uint8_t &n, uint8_t cap,
                        const __FlashStringHelper *name) {
        if (n >= cap) return 0xFF;  // over capacity: handle is inert
        arr[n] = name;
        return n++;
    }

    /// FNV-1a over dir, type, slot and name of every channel in canonical order.
    /// Must match microrio-proto's Schema::hash() byte for byte.
    uint32_t computeHash() const {
        uint32_t h = MICRORIO_FNV_OFFSET;
        h = hashGroup(h, _tbName, _nTB, MICRORIO_DIR_TELEMETRY, MICRORIO_TYPE_BOOL);
        h = hashGroup(h, _tiName, _nTI, MICRORIO_DIR_TELEMETRY, MICRORIO_TYPE_I32);
        h = hashGroup(h, _tfName, _nTF, MICRORIO_DIR_TELEMETRY, MICRORIO_TYPE_F32);
        h = hashGroup(h, _tsName, _nTS, MICRORIO_DIR_TELEMETRY, MICRORIO_TYPE_STR);
        h = hashGroup(h, _cbName, _nCB, MICRORIO_DIR_CONTROL, MICRORIO_TYPE_BOOL);
        h = hashGroup(h, _ciName, _nCI, MICRORIO_DIR_CONTROL, MICRORIO_TYPE_I32);
        h = hashGroup(h, _cfName, _nCF, MICRORIO_DIR_CONTROL, MICRORIO_TYPE_F32);
        h = hashGroup(h, _csName, _nCS, MICRORIO_DIR_CONTROL, MICRORIO_TYPE_STR);
        h = hashGroup(h, _ccName, _nCC, MICRORIO_DIR_CONTROL, MICRORIO_TYPE_CMD);
        return h;
    }

    static uint32_t hashGroup(uint32_t h, const __FlashStringHelper *const *arr, uint8_t n,
                              uint8_t dir, uint8_t ctype) {
        using namespace microrio_detail;
        for (uint8_t i = 0; i < n; i++) {
            h = fnvFeed(h, dir);
            h = fnvFeed(h, ctype);
            h = fnvFeed(h, (uint8_t)(i & 0xFF));         // slot, low byte
            h = fnvFeed(h, (uint8_t)((i >> 8) & 0xFF));  // slot, high byte
            const char *p = reinterpret_cast<const char *>(arr[i]);
            for (uint8_t k = 0;; k++) {
                uint8_t c = flashByte(p + k);
                if (!c || k >= MICRORIO_MAX_NAME_LEN) break;
                h = fnvFeed(h, c);
            }
        }
        return h;
    }

    static uint16_t periodFor(uint16_t hz) {
        if (!hz) return 1000u / MICRORIO_DEFAULT_RATE_HZ;
        uint16_t p = 1000u / hz;
        return p ? p : 1;
    }

    // ------------------------------------------------------------ counts

    static uint8_t boolBytes(uint8_t n) { return (uint8_t)((n + 7) / 8); }
    uint16_t telemPayloadLen() const {
        return boolBytes(_nTB) + (uint16_t)_nTI * 4 + (uint16_t)_nTF * 4 +
               (uint16_t)_nTS * MICRORIO_STR_LEN;
    }
    uint16_t ctrlPayloadLen() const {
        return boolBytes(_nCB) + (uint16_t)_nCI * 4 + (uint16_t)_nCF * 4 +
               (uint16_t)_nCS * MICRORIO_STR_LEN + (uint16_t)_nCC * 2;
    }
    uint16_t totalChannels() const {
        return (uint16_t)_nTB + _nTI + _nTF + _nTS + _nCB + _nCI + _nCF + _nCS + _nCC;
    }

    // ------------------------------------------------------------ writing

    void putHeader(uint8_t msg) {
        uint8_t h[MICRORIO_HEADER_LEN];
        h[0] = (uint8_t)(MICRORIO_MAGIC & 0xFF);
        h[1] = (uint8_t)(MICRORIO_MAGIC >> 8);
        h[2] = MICRORIO_PROTO_VERSION;
        h[3] = msg;
        h[4] = (uint8_t)(_nodeId & 0xFF);
        h[5] = (uint8_t)(_nodeId >> 8);
        h[6] = (uint8_t)(_schemaHash & 0xFF);
        h[7] = (uint8_t)((_schemaHash >> 8) & 0xFF);
        h[8] = (uint8_t)((_schemaHash >> 16) & 0xFF);
        h[9] = (uint8_t)((_schemaHash >> 24) & 0xFF);
        h[10] = (uint8_t)(_seq & 0xFF);
        h[11] = (uint8_t)(_seq >> 8);
        _udp.write(h, MICRORIO_HEADER_LEN);
        _seq++;
    }

    void putU16(uint16_t v) {
        uint8_t b[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
        _udp.write(b, 2);
    }
    void putU32(uint32_t v) {
        uint8_t b[4] = {(uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF),
                        (uint8_t)((v >> 16) & 0xFF), (uint8_t)((v >> 24) & 0xFF)};
        _udp.write(b, 4);
    }
    void putF32(float v) {
        uint32_t bits;
        memcpy(&bits, &v, 4);
        putU32(bits);
    }
    void putFlashName(const __FlashStringHelper *n) {
        using namespace microrio_detail;
        const char *p = reinterpret_cast<const char *>(n);
        uint8_t len = flashLen(n);
        _udp.write(&len, 1);
        for (uint8_t i = 0; i < len; i++) {
            uint8_t c = flashByte(p + i);
            _udp.write(&c, 1);
        }
    }

    void sendTelemetry() {
        const uint32_t rec = ++_record;
        const uint32_t nowUs = (uint32_t)micros();
        if (HIST > 1) snapshot(rec, nowUs);

        // Resend whatever the station has not acknowledged, while it is still
        // held. An ack of 0, or one from the future (a station that outlived
        // our reboot), asks for nothing but the newest.
        uint8_t count = 1;
        if (HIST > 1 && _ack != 0 && _ack < rec) {
            uint32_t behind = rec - _ack;
            count = behind < _history ? (uint8_t)behind : _history;
        }

        // Latency probe: note when one record leaves, and time the ack for it.
        if (_probeRec == 0) {
            _probeRec = rec;
            _probeMs = (uint16_t)millis();
        }

        const uint32_t sendStart = micros();
        _udp.beginPacket(_stationIp, _stationPort);
        putHeader(MICRORIO_MSG_TELEM);

        putU32(_echoHostTime);
        uint8_t st = state();
        _udp.write(&st, 1);
        uint8_t flags = 0;
        if (watchdogExpired()) flags |= MICRORIO_FLAG_WATCHDOG_TRIPPED;
        if (!_everLinked) flags |= MICRORIO_FLAG_NEVER_LINKED;
        // Held for a couple of seconds so the owner sees it too: being fought
        // over is something the operator driving the machine should know.
        if (_conflictMs && (uint32_t)(millis() - _conflictMs) < 2000u) {
            flags |= MICRORIO_FLAG_STATION_CONFLICT;
        }
        _udp.write(&flags, 1);
        putU16(telemetryRateHz());
        putU32(rec);
        _udp.write(&count, 1);

        if (HIST > 1) {
            const uint16_t len = MICRORIO_RECORD_FIXED + telemPayloadLen();
            for (uint8_t i = 0; i < count; i++) _udp.write(_ring[(rec - i) % HIST], len);
        } else {
            putU32(nowUs);
            streamPayload();
        }
        _udp.endPacket();

        // Even the owner can become unreachable — a laptop that sleeps, a
        // cable pulled. Sending to it then blocks for as long as the chip
        // takes to give up, every tick. Back the rate right off instead, so
        // control keeps being read and the sketch keeps running.
        if ((uint32_t)(micros() - sendStart) > MICRORIO_SLOW_SEND_US) {
            if (_slowOwner < 255) _slowOwner++;
        } else {
            _slowOwner = 0;
        }
    }

    /// Bools bitpacked LSB first, then i32, f32 and str in slot order, written
    /// straight to the socket. This is the Uno path: nothing is staged.
    void streamPayload() {
        for (uint8_t byte = 0; byte < boolBytes(_nTB); byte++) {
            uint8_t packed = packBools(byte);
            _udp.write(&packed, 1);
        }
        for (uint8_t i = 0; i < _nTI; i++) putU32((uint32_t)_ti[i]);
        for (uint8_t i = 0; i < _nTF; i++) putF32(_tf[i]);
        for (uint8_t i = 0; i < _nTS; i++) {
            uint8_t field[MICRORIO_STR_LEN];
            strField(i, field);
            _udp.write(field, MICRORIO_STR_LEN);
        }
    }

    /// The same bytes as streamPayload(), kept for resending.
    void snapshot(uint32_t rec, uint32_t nodeUs) {
        uint8_t *p = _ring[rec % HIST];
        p = stash32(p, nodeUs);
        for (uint8_t byte = 0; byte < boolBytes(_nTB); byte++) *p++ = packBools(byte);
        for (uint8_t i = 0; i < _nTI; i++) p = stash32(p, (uint32_t)_ti[i]);
        for (uint8_t i = 0; i < _nTF; i++) {
            uint32_t bits;
            memcpy(&bits, &_tf[i], 4);
            p = stash32(p, bits);
        }
        for (uint8_t i = 0; i < _nTS; i++) {
            strField(i, p);
            p += MICRORIO_STR_LEN;
        }
    }

    static uint8_t *stash32(uint8_t *p, uint32_t v) {
        p[0] = (uint8_t)(v & 0xFF);
        p[1] = (uint8_t)((v >> 8) & 0xFF);
        p[2] = (uint8_t)((v >> 16) & 0xFF);
        p[3] = (uint8_t)((v >> 24) & 0xFF);
        return p + 4;
    }

    uint8_t packBools(uint8_t byte) const {
        uint8_t packed = 0;
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint8_t idx = byte * 8 + bit;
            if (idx < _nTB && _tb[idx]) packed |= (uint8_t)(1 << bit);
        }
        return packed;
    }

    void strField(uint8_t i, uint8_t *field) const {
        memset(field, 0, MICRORIO_STR_LEN);
        for (uint8_t k = 0; k < MICRORIO_STR_LEN && _ts[i][k]; k++) field[k] = _ts[i][k];
    }

    /// Whether it is worth sending anything to a station that is not the
    /// owner right now.
    bool strangersWelcome() const {
        return !_quietUntil || (int32_t)(millis() - _quietUntil) >= 0;
    }

    /// Note how long a send to a stranger took, and stop talking to
    /// strangers for a while if it cost real time.
    void chargeStranger(uint32_t us) {
        if (us > MICRORIO_SLOW_SEND_US) _quietUntil = millis() + MICRORIO_QUIET_MS;
    }

    /// Tell a station that is not the owner that its control was refused.
    ///
    /// A telemetry frame carrying the conflict flag and nothing else that
    /// matters: one record, the live values, so the station can decode it
    /// with the schema it already has and show what is wrong.
    void sendRefusal(IPAddress to, uint16_t toPort) {
        _udp.beginPacket(to, toPort);
        putHeader(MICRORIO_MSG_TELEM);
        putU32(0);  // not this station's clock; it must not be timed against
        uint8_t st = state();
        _udp.write(&st, 1);
        uint8_t flags = MICRORIO_FLAG_STATION_CONFLICT;
        _udp.write(&flags, 1);
        putU16(telemetryRateHz());
        putU32(_record);
        uint8_t count = 1;
        _udp.write(&count, 1);
        putU32((uint32_t)micros());
        streamPayload();
        _udp.endPacket();
    }

    /// Emit one schema chunk. Chunk 0 additionally carries the node info block.
    void sendSchemaChunk(uint16_t want, IPAddress to, uint16_t toPort) {
        uint16_t count = chunkCount();
        if (want >= count) return;

        uint16_t first, n;
        chunkRange(want, first, n);

        _udp.beginPacket(to, toPort);
        putHeader(MICRORIO_MSG_SCHEMA);
        putU16(want);
        putU16(count);
        putU16(n);
        if (want == 0) {
            putU16(_sketchPeriodMs ? (uint16_t)(1000u / _sketchPeriodMs) : 0);
            putU16(_maxFrame);
            putU16(totalChannels());
            _udp.write(&_history, 1);
            putFlashName(_nodeName);
        }
        for (uint16_t i = 0; i < n; i++) putEntry(first + i);
        _udp.endPacket();
    }

    /// Describe channel `idx` in canonical order.
    void channelAt(uint16_t idx, uint8_t &dir, uint8_t &ctype, uint8_t &slot,
                   const __FlashStringHelper *&name) const {
        struct Group {
            const __FlashStringHelper *const *arr;
            uint8_t n, dir, ctype;
        };
        const Group groups[9] = {
            {_tbName, _nTB, MICRORIO_DIR_TELEMETRY, MICRORIO_TYPE_BOOL},
            {_tiName, _nTI, MICRORIO_DIR_TELEMETRY, MICRORIO_TYPE_I32},
            {_tfName, _nTF, MICRORIO_DIR_TELEMETRY, MICRORIO_TYPE_F32},
            {_tsName, _nTS, MICRORIO_DIR_TELEMETRY, MICRORIO_TYPE_STR},
            {_cbName, _nCB, MICRORIO_DIR_CONTROL, MICRORIO_TYPE_BOOL},
            {_ciName, _nCI, MICRORIO_DIR_CONTROL, MICRORIO_TYPE_I32},
            {_cfName, _nCF, MICRORIO_DIR_CONTROL, MICRORIO_TYPE_F32},
            {_csName, _nCS, MICRORIO_DIR_CONTROL, MICRORIO_TYPE_STR},
            {_ccName, _nCC, MICRORIO_DIR_CONTROL, MICRORIO_TYPE_CMD},
        };
        uint16_t seen = 0;
        for (uint8_t g = 0; g < 9; g++) {
            if (idx < seen + groups[g].n) {
                slot = (uint8_t)(idx - seen);
                dir = groups[g].dir;
                ctype = groups[g].ctype;
                name = groups[g].arr[slot];
                return;
            }
            seen += groups[g].n;
        }
        dir = ctype = slot = 0;
        name = F("");
    }

    void putEntry(uint16_t idx) {
        uint8_t dir, ctype, slot;
        const __FlashStringHelper *name;
        channelAt(idx, dir, ctype, slot, name);
        _udp.write(&dir, 1);
        _udp.write(&ctype, 1);
        putU16(slot);
        putFlashName(name);
    }

    uint16_t entryLen(uint16_t idx) const {
        uint8_t dir, ctype, slot;
        const __FlashStringHelper *name;
        channelAt(idx, dir, ctype, slot, name);
        return 1 + 1 + 2 + 1 + microrio_detail::flashLen(name);
    }

    uint16_t preambleLen(bool first) const {
        uint16_t base = 2 + 2 + 2;
        if (first) base += 2 + 2 + 2 + 1 + 1 + microrio_detail::flashLen(_nodeName);
        return base;
    }

    /// Walk the canonical order, splitting wherever the budget runs out.
    /// Mirrors Schema::encode_chunks() on the Rust side.
    void chunkWalk(uint16_t target, uint16_t &outFirst, uint16_t &outN, uint16_t &outCount) const {
        const uint16_t total = totalChannels();
        uint16_t chunk = 0, first = 0, n = 0;
        uint16_t used = MICRORIO_HEADER_LEN + preambleLen(true);
        outFirst = 0;
        outN = 0;

        for (uint16_t i = 0; i < total; i++) {
            uint16_t e = entryLen(i);
            if (n > 0 && used + e > _maxFrame) {
                if (chunk == target) { outFirst = first; outN = n; }
                chunk++;
                first = i;
                n = 0;
                used = MICRORIO_HEADER_LEN + preambleLen(false);
            }
            used += e;
            n++;
        }
        if (chunk == target) { outFirst = first; outN = n; }
        outCount = (uint16_t)(chunk + 1);
    }

    uint16_t chunkCount() const {
        uint16_t f, n, c;
        chunkWalk(0xFFFF, f, n, c);
        return c;
    }
    void chunkRange(uint16_t want, uint16_t &first, uint16_t &n) const {
        uint16_t c;
        chunkWalk(want, first, n, c);
    }

    // ------------------------------------------------------------ reading

    void receive() {
        int size = _udp.parsePacket();
        while (size > 0) {
            handlePacket((uint16_t)size);
            size = _udp.parsePacket();
        }
    }

    void handlePacket(uint16_t size) {
        if (size < MICRORIO_HEADER_LEN) { flush(size); return; }

        uint8_t h[MICRORIO_HEADER_LEN];
        _udp.read(h, MICRORIO_HEADER_LEN);
        uint16_t remaining = size - MICRORIO_HEADER_LEN;

        uint16_t magic = (uint16_t)h[0] | ((uint16_t)h[1] << 8);
        if (magic != MICRORIO_MAGIC || h[2] != MICRORIO_PROTO_VERSION) { flush(remaining); return; }

        uint16_t nodeId = (uint16_t)h[4] | ((uint16_t)h[5] << 8);
        if (nodeId != _nodeId) { flush(remaining); return; }

        IPAddress from = _udp.remoteIP();
        uint16_t fromPort = _udp.remotePort();

        // Anyone may ask what this node has: a schema request is a question,
        // not a command, and answering it costs nothing. The answer goes
        // straight back to whoever asked without disturbing the owner, so a
        // second station can look at a board it is not allowed to drive.
        if (h[3] == MICRORIO_MSG_SCHEMA_REQ) {
            uint16_t want = 0;
            if (remaining >= 2) {
                uint8_t b[2];
                _udp.read(b, 2);
                want = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
                remaining -= 2;
            }
            flush(remaining);
            bool stranger = _haveStation && (from != _stationIp || fromPort != _stationPort);
            if (stranger && !strangersWelcome()) return;
            uint32_t t0 = micros();
            sendSchemaChunk(want, from, fromPort);
            if (stranger) chargeStranger(micros() - t0);
            return;
        }

        if (h[3] != MICRORIO_MSG_CONTROL) { flush(remaining); return; }

        // A watcher: never the owner, never a source of values. Answered
        // with telemetry so it can see, and listened to for one thing only.
        if (remaining >= MICRORIO_CONTROL_FIXED) {
            uint8_t peek[MICRORIO_CONTROL_FIXED];
            // Read the fixed head, then decide. The payload is flushed either
            // way, so nothing is left in the socket for the next packet.
            _udp.read(peek, MICRORIO_CONTROL_FIXED);
            uint8_t cflags = peek[5];
            if (cflags & MICRORIO_CFLAG_VIEW_ONLY) {
                // A station that is only watching is not answered at all —
                // not even to say no. Replying is what costs: this chip
                // resolves the destination before every packet, and a laptop
                // on WiFi takes a few hundred milliseconds to answer that,
                // every single time. The owner's station relays to watchers
                // instead; a computer has an address cache and a board does
                // not.
                //
                // One thing is still listened for. Anyone watching may stop
                // the machine, and hearing that costs nothing.
                if (peek[4] == MICRORIO_STATE_ESTOP) _viewerEstopMs = millis();
                flush(remaining - MICRORIO_CONTROL_FIXED);
                return;
            }
            memcpy(_pending, peek, MICRORIO_CONTROL_FIXED);
            _havePending = true;
        }

        // Ownership. The first station to send control takes the node, and
        // keeps it until it goes quiet for longer than the watchdog - by
        // which point its outputs have gone to their failstates anyway, so
        // there is nothing to take away from it.
        //
        // Without this, a node followed whoever spoke last: two stations
        // would each get half the telemetry, both would command it, and the
        // board would do as it was told by whichever packet landed most
        // recently. That is a real hazard, not just a confusing display.
        const uint32_t nowMs = millis();
        if (!_haveStation || (uint32_t)(nowMs - _lastControlMs) > _watchdogMs) {
            _stationIp = from;
            _stationPort = fromPort;
            _haveStation = true;
        } else if (from != _stationIp || fromPort != _stationPort) {
            flush(remaining);
            _conflictMs = nowMs;
            // Tell the intruder rather than ignoring it silently; it has no
            // other way to find out, because the telemetry it is waiting for
            // goes to the owner. Rate limited: one answer per second is
            // enough to say "not yours", and a flood of refusals is just a
            // second way to drown the link.
            if (strangersWelcome() && (!_refusedMs || (uint32_t)(nowMs - _refusedMs) > 1000u)) {
                _refusedMs = nowMs;
                uint32_t t0 = micros();
                sendRefusal(from, fromPort);
                chargeStranger(micros() - t0);
            }
            return;
        }

        uint32_t hash = (uint32_t)h[6] | ((uint32_t)h[7] << 8) | ((uint32_t)h[8] << 16) |
                        ((uint32_t)h[9] << 24);
        // Wrong schema means the station is talking about a different firmware.
        // Discard rather than guess at the layout.
        if (hash != _schemaHash) { flush(remaining); return; }

        const uint16_t need = MICRORIO_CONTROL_FIXED + ctrlPayloadLen();
        if (!_havePending || remaining < need) { flush(remaining); return; }
        remaining -= MICRORIO_CONTROL_FIXED;
        _havePending = false;

        _echoHostTime = (uint32_t)_pending[0] | ((uint32_t)_pending[1] << 8) |
                        ((uint32_t)_pending[2] << 16) | ((uint32_t)_pending[3] << 24);
        uint8_t st = _pending[4];
        uint16_t hz = (uint16_t)_pending[6] | ((uint16_t)_pending[7] << 8);
        uint32_t ack = (uint32_t)_pending[8] | ((uint32_t)_pending[9] << 8) |
                       ((uint32_t)_pending[10] << 16) | ((uint32_t)_pending[11] << 24);

        if (st <= MICRORIO_STATE_FAULT) _commanded = st;
        // Rate governs telemetry, and so logging. It never touches the sketch's loop.
        _periodMs = hz ? periodFor(hz) : _sketchPeriodMs;
        noteAck(ack);
        noteControlSeq((uint16_t)h[10] | ((uint16_t)h[11] << 8));

        for (uint8_t byte = 0; byte < boolBytes(_nCB); byte++) {
            uint8_t packed = getU8();
            for (uint8_t bit = 0; bit < 8; bit++) {
                uint8_t idx = byte * 8 + bit;
                if (idx < _nCB) _cb[idx] = (packed & (1 << bit)) != 0;
            }
        }
        for (uint8_t i = 0; i < _nCI; i++) _ci[i] = (int32_t)getU32();
        for (uint8_t i = 0; i < _nCF; i++) {
            uint32_t bits = getU32();
            memcpy(&_cf[i], &bits, 4);
        }
        for (uint8_t i = 0; i < _nCS; i++) {
            uint8_t field[MICRORIO_STR_LEN];
            _udp.read(field, MICRORIO_STR_LEN);
            uint8_t k = 0;
            for (; k < MICRORIO_STR_LEN - 1 && field[k]; k++) _cs[i][k] = (char)field[k];
            _cs[i][k] = 0;
        }
        for (uint8_t i = 0; i < _nCC; i++) _cc[i] = getU16();

        flush(remaining - ctrlPayloadLen());

        _lastControlMs = millis();
        _everLinked = true;
    }

    /// Remember a watcher, or refresh one we already know.
    void noteAck(uint32_t ack) {
        _ack = ack;
        if (_probeRec && ack >= _probeRec && ack <= _record) {
            uint16_t d = (uint16_t)millis() - _probeMs;
            // Light smoothing; one late frame should not swing the figure.
            _latencyMs = _latencyMs ? (uint16_t)((_latencyMs * 3u + d) / 4u) : d;
            _probeRec = 0;
        } else if (_probeRec && (ack > _record || _record - _probeRec > 250)) {
            _probeRec = 0;  // that record is never going to be acked; try again
        }
    }

    void noteControlSeq(uint16_t seq) {
        if (_haveCtrlSeq) {
            uint16_t gap = seq - _lastCtrlSeq;
            // A huge jump is a new station, not loss.
            if (gap > 0 && gap < 1000) {
                _ctrlExpected += gap;
                _ctrlGot++;
            }
            if (_ctrlExpected > 400) {
                _ctrlExpected /= 2;
                _ctrlGot /= 2;
            }
        }
        _lastCtrlSeq = seq;
        _haveCtrlSeq = true;
    }

    uint8_t getU8() {
        uint8_t b = 0;
        _udp.read(&b, 1);
        return b;
    }
    uint16_t getU16() {
        uint8_t b[2];
        _udp.read(b, 2);
        return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
    }
    uint32_t getU32() {
        uint8_t b[4];
        _udp.read(b, 4);
        return (uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) |
               ((uint32_t)b[3] << 24);
    }
    void flush(uint16_t n) {
        uint8_t sink[16];
        while (n) {
            uint16_t take = n > sizeof(sink) ? sizeof(sink) : n;
            _udp.read(sink, take);
            n -= take;
        }
    }

    // ------------------------------------------------------------- state

    UDP &_udp;
    IPAddress _stationIp;
    uint16_t _stationPort = 0;
    bool _haveStation = false;

    uint16_t _nodeId = 1;
    const __FlashStringHelper *_nodeName;  // set in the ctor
    uint16_t _periodMs = 1000u / MICRORIO_DEFAULT_RATE_HZ;
    uint16_t _sketchPeriodMs = 1000u / MICRORIO_DEFAULT_RATE_HZ;
    uint16_t _watchdogMs = MICRORIO_DEFAULT_WATCHDOG_MS;
    uint16_t _maxFrame = MICRORIO_DEFAULT_FRAME;

    uint32_t _schemaHash = 0;
    uint16_t _seq = 0;
    uint32_t _lastSendMs = 0;
    uint32_t _lastControlMs = 0;
    uint32_t _echoHostTime = 0;
    uint8_t _commanded = MICRORIO_STATE_DISABLED;
    bool _everLinked = false;

    uint32_t _record = 0;
    uint32_t _ack = 0;
    uint8_t _history = 1;
    uint32_t _probeRec = 0;
    uint16_t _probeMs = 0;
    uint16_t _latencyMs = 0;
    uint32_t _conflictMs = 0;
    uint32_t _viewerEstopMs = 0;
    // The fixed head of a control frame, read before we know whether the
    // sender is allowed to send one.
    uint8_t _pending[MICRORIO_CONTROL_FIXED];
    bool _havePending = false;
    uint32_t _quietUntil = 0;
    uint8_t _slowOwner = 0;
    uint32_t _refusedMs = 0;
    uint16_t _lastCtrlSeq = 0;
    bool _haveCtrlSeq = false;
    uint16_t _ctrlExpected = 0, _ctrlGot = 0;
    // One byte when HIST is 1, so an Uno pays nothing for a buffer it never uses.
    uint8_t _ring[HIST > 1 ? HIST : 1][HIST > 1 ? microrio_detail::recordBytes(TB, TI, TF, TS) : 1];

    const __FlashStringHelper *_tbName[TB ? TB : 1];
    const __FlashStringHelper *_tiName[TI ? TI : 1];
    const __FlashStringHelper *_tfName[TF ? TF : 1];
    const __FlashStringHelper *_tsName[TS ? TS : 1];
    const __FlashStringHelper *_cbName[CB ? CB : 1];
    const __FlashStringHelper *_ciName[CI ? CI : 1];
    const __FlashStringHelper *_cfName[CF ? CF : 1];
    const __FlashStringHelper *_csName[CS ? CS : 1];
    const __FlashStringHelper *_ccName[CC ? CC : 1];
    uint8_t _nTB = 0, _nTI = 0, _nTF = 0, _nTS = 0;
    uint8_t _nCB = 0, _nCI = 0, _nCF = 0, _nCS = 0, _nCC = 0;

    bool _tb[TB ? TB : 1];
    int32_t _ti[TI ? TI : 1];
    float _tf[TF ? TF : 1];
    char _ts[TS ? TS : 1][MICRORIO_STR_LEN];

    bool _cb[CB ? CB : 1];
    int32_t _ci[CI ? CI : 1];
    float _cf[CF ? CF : 1];
    char _cs[CS ? CS : 1][MICRORIO_STR_LEN];
    uint16_t _cc[CC ? CC : 1];

    bool _cbFail[CB ? CB : 1] = {};
    int32_t _ciFail[CI ? CI : 1] = {};
    float _cfFail[CF ? CF : 1] = {};
    bool _cbHasFail[CB ? CB : 1] = {};
    bool _ciHasFail[CI ? CI : 1] = {};
    bool _cfHasFail[CF ? CF : 1] = {};
    uint16_t _ccSeen[CC ? CC : 1];
};

#endif  // MICRORIO_H
