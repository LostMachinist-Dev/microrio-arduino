# MicroRIO

Named telemetry and control over UDP, for a MicroRIO driver station.

Declare channels by name once. The station negotiates the schema when it
connects, and from then on both sides exchange fixed binary frames addressed by
slot — names never appear in a telemetry or control frame.

## Install

Extract the zip into your Arduino `libraries` folder, so you end up with:

    libraries/MicroRIO/library.properties
    libraries/MicroRIO/src/MicroRIO.h
    libraries/MicroRIO/examples/...

Restart the IDE and MicroRIO appears under *Sketch → Include Library*, with the
examples under *File → Examples*.

## Direction is the thing people get wrong

| Direction | You declare | You call | The GUI |
|---|---|---|---|
| **Telemetry** — board to station | `telemetry_f32("battery_v")` | `write(h, v)` | displays it |
| **Control** — station to board | `control_f32("drive_x")` | `read(h)` | populates it |

`read()` on this side means **read what the station sent**. Control values do
not come from the board's own hardware; something in the GUI — a slider, a
checkbox, a joystick mapping — has to drive them. A control channel with
nothing driving it sits at its default forever. That is not a fault.

## Minimal sketch

```cpp
#include <Ethernet.h>
#include <MicroRIO.h>

EthernetUDP udp;
// capacity per direction: telemetry bool,i32,f32,str | control bool,i32,f32,str,cmd
MicroRIO<1, 1, 1, 0,  1, 0, 1, 0, 1> rio(udp);

MicroRIO_TelemBool t_hb;
MicroRIO_TelemI32  t_a0;
MicroRIO_TelemF32  t_volts;
MicroRIO_CtrlBool  c_led;
MicroRIO_CtrlF32   c_drive;
MicroRIO_CtrlCmd   c_zero;

void setup() {
    Ethernet.begin(mac, ip);     // your network, your business
    udp.begin(5005);

    rio.node(1, F("drivebase"));
    rio.rate(50);          // telemetry rate, until the station asks for another
    rio.watchdog(200);     // no control frame for this long and the link is down

    t_hb    = rio.telemetry_bool(F("heartbeat"));
    t_a0    = rio.telemetry_i32 (F("a0"));
    t_volts = rio.telemetry_f32 (F("battery_v"));
    c_led   = rio.control_bool  (F("led"));
    c_drive = rio.control_f32   (F("drive_x"));
    c_zero  = rio.control_cmd   (F("zero_encoders"));

    // What the controls read as while the station is not being heard from.
    // Every safe value for this board, in one place.
    rio.failstate(c_led, false);
    rio.failstate(c_drive, 0.0f);

    rio.begin();
}

void loop() {
    rio.loop();

    rio.write(t_a0, analogRead(A0));
    rio.write(t_hb, (millis() / 500) % 2);

    if (rio.enabled()) {
        digitalWrite(LED_BUILTIN, rio.read(c_led));
    }
    if (rio.fired(c_zero)) {
        // one-shot, fires once per station-side increment
    }
}
```

## Things worth knowing

**Wrap every name in `F()`.** That keeps it in flash. A bare string literal
lands in SRAM on AVR, which is the expensive place to put it.

**Use the handles, not names, at runtime.** Declaring returns a typed handle;
`write(handle, v)` is O(1) and type-checked at compile time, so you cannot write
an int into a float slot.

**Capacity is a compile-time template parameter.** You pay for the channels you
declare and nothing else. A fixed one-size library would cost about 1.4 KB of
SRAM on an Uno before your sketch did anything.

**Transport is injected.** The library takes an Arduino `UDP&` and never calls
`Ethernet.begin()` or `WiFi.begin()` itself — you own the network. An Ethernet
shield, WiFi, AP mode and the ESP32's `ETH` are all the same library code.

**On a W5100 shield, shorten the retries.** Sending to an address the chip
cannot resolve blocks your sketch while it retries ARP — by default for over a
second and a half, every time. Two lines after `Ethernet.begin()` cap it:

```cpp
Ethernet.setRetransmissionTimeout(50);
Ethernet.setRetransmissionCount(1);
```

The library protects itself as well: any station that is not the owner is
dropped if sending to it costs real time, and the node backs its own rate off
rather than stalling if the owner stops answering. But the two lines above are
what turn a second and a half into a tenth of one.

**There are no pin APIs here.** Only named variables. Where a value came from,
and what it drives, is your sketch's business.

**`enabled()` is the gate.** It is evaluated live, so it can never return a
stale answer, and the library neutralises on its own if the watchdog expires —
you do not have to remember to check.

**Failstates put every safe value in one place.** `rio.failstate(handle, v)`
says what a control reads as while the watchdog is tripped. A channel with no
failstate keeps the last value it was sent; nothing is decided for you.
`rio.watchdogOK()` is the same answer as a plain bool, for code that wants to
do more than fall back to a value.

**Telemetry is numbered, and resent if it goes missing.** Every sample carries
a record number; the station says what it has and the library resends what it
still holds. A board with RAM to spare keeps a few records, and a 2 KB board
keeps none and streams exactly as before — it still gets true loss
measurement. Override the depth with the tenth template parameter if you want
to spend the memory differently:

```cpp
MicroRIO<1, 1, 1, 0,  1, 0, 1, 0, 1,  16> rio(udp);   // keep 16 records
```

**The station can set the telemetry rate.** It governs how often the node
*reports* — and so how many rows a second the log gets — never how fast your
loop runs. A board can compute at 1 kHz and report at 50 Hz.

**One station at a time, enforced.** The first station to send control owns
the node; another station's control is refused and told so, until the owner
goes quiet for longer than the watchdog. Anyone may ask for the schema at any
time. `rio.stationConflict()` is true for two seconds after someone was
refused, if you want to show it on a local display.

**Other stations watch through the driver station, not through you.** A
station that marks its frames view-only is never answered — the station that
owns the node forwards telemetry to it instead. That is deliberate: replying
to a station you are not already talking to is what costs a board its timing.
On a W5100 a watcher on a sleepy WiFi laptop cost 377 ms *per frame*, because
the chip resolves the address before every packet.

The one thing a watcher is listened to for is the estop, and the node obeys
it over its owner. Listening is free; answering is not. `rio.stationConflict()`
still reports a station that tried to *drive* and was refused.

## Memory

Measured against the real toolchain. Capacity is per direction.

| Config | bools | i32 | f32 | cmd | str | Uno SRAM | Mega SRAM |
|---|---|---|---|---|---|---|---|
| baseline (Ethernet + UDP only) | — | — | — | — | — | 199 B / 9% | 227 B / 2% |
| small | 16 | 8 | 8 | 8 | 0 | 383 B / 18% | 411 B / 5% |
| medium | 32 | 16 | 16 | 8 | 2 | 575 B / 28% | 603 B / 7% |
| large | 64 | 32 | 32 | 16 | 4 | 927 B / 45% | 955 B / 11% |

It fits an Uno because frames stream straight into the socket rather than being
staged in RAM, and channel names live in flash. Numbering the records costs an
Uno 27 bytes and no buffer; a board that keeps records pays for them, which is
why a 2 KB board does not.

## Examples

- **BasicNode** — Ethernet shield, the smallest useful node.
- **SimNode** — sine, ramp, square and noise plus a DS3231 RTC and clock sync.
  This is the one to flash if you want something worth graphing.
- **Esp32PoeNode** — same library over an Olimex ESP32-POE-ISO's Ethernet.
- **Esp32WifiNode** — the same node over WiFi, with its credentials kept out
  of the sketch. Flash it with `arduino/flash-wifi-node.sh`, which writes them
  into a temporary copy and deletes it afterwards.
- **Conformance** — runs the protocol over USB serial instead of UDP, for
  cross-checking the firmware against the station's codec with no network.

## A note on the two headers

`MicroRIO.h` is the library. `MicroRIO_protocol.h` is **generated** from
`spec/protocol.toml` in the project, which also generates the Rust side — so the
firmware and the driver station cannot drift apart. Do not hand-edit it; change
the spec and regenerate.

## Where the rest of it is

This repository is the board half, packaged on its own so the Arduino IDE can
install it. The driver station it talks to, the protocol spec both sides are
generated from, the design notes and the development history all live in
[LostMachinist-Dev/microrio](https://github.com/LostMachinist-Dev/microrio).

A note on what the Conformance example is for: it speaks the protocol over USB
serial and never opens a socket, so a board holding it is invisible on the
network. Flash a node sketch back when you are finished testing.

## Licence

MIT or Apache-2.0, at your option. It drives machinery and comes with no
warranty; whatever you connect it to is yours to make safe.
