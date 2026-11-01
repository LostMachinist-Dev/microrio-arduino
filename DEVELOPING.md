# Working on the library

`README.md` is for someone using the library in a sketch. This is for
changing the library itself.

## Where it lives, and the one rule

The library is written in `arduino/MicroRIO/` inside the **station**
repository, and pushed from there to its own repository by
`arduino/publish-library.sh`. The published copy is overwritten every time
that runs.

**Never edit the published copy.** A library that exists twice gets fixed in
the wrong one. To work on it:

```sh
git clone https://github.com/LostMachinist-Dev/microrio.git
cd microrio/arduino/MicroRIO      # this is the library
```

and when it is ready, from the station repo's root:

```sh
arduino/publish-library.sh        # copies this tree over the published repo
```

It says `nothing to publish` when the two already match.

## What you can change alone, and what you cannot

Everything in `MicroRIO.h` is yours: the API, the capacities, the timing
rules, the diagnostics, the examples. The library can grow without the
station knowing.

`src/MicroRIO_protocol.h` is **generated** from `spec/protocol.toml` in the
station repo, which generates the Rust side from the same file. Do not
hand-edit it. If a change needs a new field, a new message type or a
different frame layout, that is a protocol change: edit the spec, run
`cargo xtask gen-header`, and the station changes with it. That part is not
solo work.

The dividing line is the wire. Above it, you are alone. On it, there are two
sides.

## How a node actually runs

1. **Declare.** `telemetry_f32(F("battery_v"))` and friends return a typed
   handle. The declaration order is the canonical channel order: it fixes the
   schema and the payload layout, and the station rebuilds the same order from
   the schema it receives, so the two cannot disagree.
2. **`begin()`** computes the schema hash from the names and types.
3. **`loop()`** answers what arrives and sends telemetry when the period is
   up. Nothing blocks waiting for the station.
4. A station asks for the schema once. Names are sent then and never again;
   everything after that is numbered slots. That is the whole reason a 2 KB
   board can take part.
5. Telemetry records are **numbered**. The station acknowledges the highest
   record it holds with nothing missing below it, and the node resends what is
   absent from the records it still keeps. `HIST` is how many it keeps.
6. Control is **full state, last write wins, never resent**. A late command is
   worse than a lost one.
7. If no control frame arrives within `watchdog(ms)`, every control reads its
   **failstate** instead of its last value. A channel with no failstate holds.
8. The node answers **one station**. The first to send control owns it; others
   are refused, and ownership lapses one watchdog period after the owner goes
   quiet. A frame flagged view-only is not answered at all — except that an
   e-stop in one is obeyed over the owner. Listening is free; replying is what
   costs a board its timing.

## The API

| Setup, before `begin()` | |
|---|---|
| `node(id, F("name"))` | Node id and the name the station shows |
| `rate(hz)` | The sketch's telemetry rate; the station may ask for another |
| `watchdog(ms)` | Silence after which controls fall to their failstates |
| `maxFrame(bytes)` | Largest frame this board can take |
| `telemetry_bool/i32/f32/str(F("name"))` | Declare a reading |
| `control_bool/i32/f32/str/cmd(F("name"))` | Declare something the station drives |
| `failstate(handle, value)` | What a control reads while the watchdog is tripped |
| `begin()` | Freeze the schema |

| Running | |
|---|---|
| `loop()` | Call every pass. Receives, sends, keeps time |
| `write(handle, value)` | Set a telemetry value |
| `read(handle)` | What the station is commanding, or the failstate |
| `fired(cmd)` | True once per press of a command channel |
| `sendNow()` | Send a telemetry frame on the next `loop()`, off-schedule |

| State the sketch should act on | |
|---|---|
| `linked()` / `watchdogOK()` | A station has been heard from and has not gone quiet |
| `enabled()` | The operator has enabled this node |
| `state()` | The raw state: disabled, enabled, estop, fault |
| `watchdogExpired()`, `controlAgeMs()` | How the silence is measured |

| Diagnosis, for telemetry about the link itself | |
|---|---|
| `record()`, `history()` | The newest record number, and how many are kept |
| `telemetryRateHz()` | The rate actually being sent at |
| `linkLossPct()`, `linkLatencyMs()` | What the node sees of the link |
| `slowLink()` | The owner is costing this board time and has been backed off |
| `stationConflict()` | Another station has been told no, recently |
| `schemaHash()` | The hash the station must agree with |

`SimNode` and `Esp32WifiNode` report several of the diagnosis values as
ordinary telemetry channels, which is how they end up on screen.

## Capacity is a template parameter

```cpp
MicroRIO<2, 3, 5, 1,      // telemetry: bools, i32, f32, strings
         2, 1, 2, 0, 1>   // control:  bools, i32, f32, strings, commands
    rio(udp);
```

You pay SRAM for the channels you declare and nothing else. A tenth
parameter sets `HIST` by hand; left out, it defaults to as many records as
one default-sized frame can carry, capped at the spec's 8 — keeping more
than a frame can hold would be RAM spent on records that can never be sent.

On a board with less than 4 KB of SRAM (`MICRORIO_SMALL_RAM`), `HIST`
defaults to 1: an Uno streams each frame and gets loss measurement only, no
backfill. That is deliberate. Do not "fix" it by raising the default.

## Compile-time knobs

| Macro | Default | Why it exists |
|---|---|---|
| `MICRORIO_SLOW_SEND_US` | `20000` | A send that takes longer than this is not a working link. On a W5100, sending to an address the chip cannot resolve blocks the sketch for over a second while it retries ARP. Any destination that costs this much is dropped rather than paid for again. Raise it for a transport that is slow by nature — the `Conformance` example sets 60000 because serial is |
| `MICRORIO_QUIET_MS` | `5000` | How long such a destination is left alone |
| `MICRORIO_DEFAULT_FRAME` | 512 on AVR, 1400 elsewhere | What `HIST` is sized against |
| `MICRORIO_SMALL_RAM` | set from `RAMEND` | Below 4 KB of SRAM, no backfill |

## Memory, measured

Per direction, against the real `arduino:avr` toolchain and `Ethernet` 2.0.2:

| Config | bools | i32 | f32 | cmd | str | Uno SRAM | Mega SRAM |
|---|---|---|---|---|---|---|---|
| baseline (Ethernet + UDP only) | — | — | — | — | — | 199 B / 9% | 227 B / 2% |
| small | 16 | 8 | 8 | 8 | 0 | 383 B / 18% | 411 B / 5% |
| medium | 32 | 16 | 16 | 8 | 2 | 575 B / 28% | 603 B / 7% |
| large | 64 | 32 | 32 | 16 | 4 | 927 B / 45% | 955 B / 11% |

`BasicNode` as it stands, compiled for an Uno: **15844 B flash (49%), 466 B
SRAM (22%)**, leaving 1582 B for local variables. Numbering the records costs
an Uno 27 bytes and no buffer.

It fits because frames are **streamed**: values go straight into the socket
buffer through `Udp.write()` and are parsed straight back out of it, field by
field. A frame is never staged in SRAM. Channel names live in flash, which is
why every name is wrapped in `F()`.

Keep it that way. The first thing that would break the Uno is a buffer.

## Building and testing

Compile for the boards that matter before anything else:

```sh
arduino-cli compile -b arduino:avr:uno    --library arduino/MicroRIO arduino/MicroRIO/examples/BasicNode
arduino-cli compile -b arduino:avr:mega   --library arduino/MicroRIO arduino/MicroRIO/examples/SimNode
arduino-cli compile -b esp32:esp32:esp32-poe-iso --library arduino/MicroRIO arduino/MicroRIO/examples/Esp32PoeNode
```

An Uno is the real test of a change's cost: it is the board with no room.

### Conformance, over USB

The strongest check. It runs this C++ codec against the station's Rust codec
over a serial port, with no network in the way, and covers 50 cases:
schema chunking and hashing, every channel type, record numbering, backfill
by acknowledgement, the watchdog, failstates, ownership and refusals,
view-only frames, and frames that should be ignored.

```sh
arduino-cli compile -b arduino:avr:mega --library arduino/MicroRIO -u -p /dev/ttyACM0 \
    arduino/MicroRIO/examples/Conformance
cargo xtask conformance /dev/ttyACM0
```

Expect `50 checks passed.` Any protocol-level change to the library should
come with a new check here.

**The board is then holding `Conformance.ino`, which never opens a socket.**
It is invisible on the network until a node sketch is flashed back, and that
looks exactly like a broken station. The command prints the reflash lines
when it finishes; run them.

### On the network

```sh
cargo xtask probe 192.168.0.50 1        # rate, loss, backfill, watchdog, clock
arduino/flash-wifi-node.sh /dev/ttyUSB0 # ESP32 over WiFi, credentials kept out of the sketch
```

Stop the station first: the probe is itself a station, and a node answers to
one at a time.

## What is proven, and what is not

**Proven on real boards.** An Arduino Mega over Ethernet and an Olimex
ESP32-POE-ISO over WiFi: 50 of 50 conformance checks on each, telemetry at
50 Hz with 0.0% loss, deliberate frame drops recovered by backfill, one
board's reading driving another's output, failstates on a tripped watchdog,
clock sync, and a board that stayed under 3.4 ms of worst-case loop while two
stations watched it.

**Not proven.**

- **An Uno on the network.** Compile-only. Its memory use is measured and its
  no-backfill path is covered by conformance, but no Uno has been plugged in.
- **Ethernet on the ESP32** (`Esp32PoeNode`). It compiles and passes
  conformance over USB; no cable has been connected.
- **Three or more nodes at once.** Two have run together.
- **Any board other than these.** Nothing in the library is specific to a
  make — it needs an Arduino `UDP` and nothing else — but that is a claim, not
  a measurement.

## Worth doing

- An Uno on the network, with a real sensor, to confirm the no-backfill path
  behaves in the field as it does in conformance.
- The PoE sketch on a cable.
- A `str` control is declared and read but thin on examples; nothing exercises
  one end to end.
- Submission to the Arduino Library Manager needs a tagged release and the
  library at a repository root, which it now is.
- `keywords.txt` covers the API as it stands; it needs a line adding whenever
  a public method does.

## Picking this up fresh

> Working on the MicroRIO Arduino library, in `arduino/MicroRIO/` of the
> microrio repo. Read `arduino/MicroRIO/DEVELOPING.md` first. The library is
> written here and pushed to its own repo by `arduino/publish-library.sh` —
> never edit the published copy. `src/MicroRIO_protocol.h` is generated from
> `spec/protocol.toml`; a wire change means changing the station too. Test
> with `arduino-cli compile` for uno/mega/esp32, then conformance over USB
> (`cargo xtask conformance <port>`), and reflash a node sketch afterwards or
> the board is invisible on the network.
