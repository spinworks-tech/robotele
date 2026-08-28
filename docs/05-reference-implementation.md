# Reference Implementation

The protocol spec (`DESIGN.md`, `REQUIREMENTS.md`) is implemented as a Cargo
workspace of Rust crates plus a small Python bridge, exercised end-to-end against a
real XGO-Lite V2 quadruped robot. This is a **v0 reference implementation**, not a
claim of full spec coverage — where it deviates from the spec (e.g. certificate
signature algorithm, timestamp encoding), that's called out explicitly rather than
left silent.

## Workspace layout

```text
[workspace]
members = [
    "crates/roboprotocol-proto",
    "crates/roboprotocol-core",
    "crates/roboprotocol-recording",
    "crates/robot-edge",
    "crates/operator-console",
    "tools/dev-certs",
    "tools/replay-decode",
]
```

```mermaid
flowchart LR
    subgraph Robot["Robot (Raspberry Pi CM4)"]
        RE[robot-edge<br/>QUIC server]
        XB[xgo_bridge.py<br/>JSON-lines subprocess]
        RE -- stdin/stdout JSON --> XB
        XB -- serial --> XGO[XGO-Lite V2<br/>vendor xgolib SDK]
    end
    subgraph Operator["Operator workstation"]
        OC[operator-console<br/>QUIC client + TUI]
    end
    OC <-- QUIC + mTLS<br/>Channel A/B/C --> RE

    subgraph Core["Shared crates"]
        Core[roboprotocol-core<br/>safety, HELLO, sizing, timestamps]
        Proto[roboprotocol-proto<br/>FlatBuffers bindings]
        Rec[roboprotocol-recording<br/>black-box logging]
    end
    RE -.depends on.-> Core
    RE -.depends on.-> Proto
    RE -.depends on.-> Rec
    OC -.depends on.-> Core
    OC -.depends on.-> Proto
    OC -.depends on.-> Rec
```

## Crates

### `roboprotocol-core` (library)

Pure protocol logic — deliberately **no networking, tokio, or `quiche` dependency**,
so it stays unit-testable and reusable by anything that needs the same rules
(`robot-edge`, `operator-console`, and eventually a ROS 2 bridge) without dragging the
transport in.

| Module | Responsibility |
| --- | --- |
| `hello.rs` | HELLO capability handshake negotiation (`DESIGN.md` §1.2) |
| `profile.rs` | Robot/media session descriptor types + deterministic field-layout derivation (`DESIGN.md` §1.3) |
| `sizing.rs` | Channel B payload sizing, quantization tiers, body-region splitting — a 1:1 port of `simulator/roboprotocol_sim/protocol/sizing.py` |
| `datagram.rs` | 1-byte channel discriminator tagging for unreliable datagrams |
| `estop.rs` | E-Stop transport framing (reliable stream + redundant 1 kHz datagram, 10-byte encoding) |
| `interpolation.rs` | Timestamp-based motion reconstruction (`DESIGN.md` §2.2.2) |
| `action_trigger.rs` | Discrete one-shot command triggers |
| `camera_control.rs` | Discrete "set-and-hold" camera image controls (brightness/contrast/EV/shutter) |
| `timestamp.rs` | 64-bit datagram timestamps + RTT/OWD math — **note:** v0 uses µs-since-epoch, not the spec's NTP 32.32 fixed-point format |
| `video.rs` | Channel A chunk framing and Annex-B NAL reassembly, kept unwrapped (not FlatBuffers) |
| `recording.rs` | Record framing + rotation/retention decision logic (I/O itself lives in `roboprotocol-recording`) |
| `safety/` | `arbitration.rs` (control-source arbitration), `state_machine.rs` (safety tier state machine), `tiers.rs` (Task Class B–E thresholds), `watchdog.rs` |

### `roboprotocol-proto` (library)

FlatBuffers schema + generated Rust bindings. `build.rs` invokes `flatc` against
`schemas/roboprotocol.fbs`; `src/lib.rs` is a thin `include!()` wrapper re-exporting
the generated module. See [§ Schema](#the-schema-roboprotocolfbs) below.

### `roboprotocol-recording` (library)

The actual thread/file-I/O plumbing for local black-box recording (`DESIGN.md` §10),
implementing FR-9's bounded-queue, drop-oldest, never-block design. Plain
`std::thread`/`std::sync` — no async runtime, so it's usable identically from both
tokio-based binaries.

| Module | Responsibility |
| --- | --- |
| `ring.rs` | Bounded per-category ring buffer / `CategoryState` |
| `segment.rs` | Segment file creation, listing, size/time-based rotation |
| `writer.rs` | Writer thread loop — periodic buffered flush, drop-oldest-on-full |

Public entry point: `Recorder::start(RecorderConfig)` spawns one writer thread per
configured `Category` (`VideoA`, `ChannelBCommand`, `ChannelBTelemetry`,
`ChannelBHaptic`, `ActionTriggerC`, `KeyPress`); `enqueue`/`set_active`/`stats` are all
non-blocking, matching NFR-6.1's ≤1 ms enqueue bound. See
[Recording & Replay](07-recording-and-replay.md).

### `robot-edge` (binary)

The QUIC server that runs on the robot. v0 serves a single peer connection at a time
and loops back to accept the next one after a disconnect — it doesn't require a
restart between operator reconnects, only after the host itself reboots.

| Module | Responsibility |
| --- | --- |
| `main.rs` | CLI parsing: `--listen`, `--cert/--key/--ca`, `--robot-id`, `--task-class`, `--tick-hz`, `--bridge-script`, `--stub-bridge`, `--serial-port`, `--watchdog-ms`, `--camera`, `--record-dir`/`--record ...` |
| `quic_server.rs` | Connection setup + main driving loop (v0: no Retry token / connection-ID-based routing) |
| `bridge/process.rs` | `BridgeSupervisor` — spawns and supervises the `xgo_bridge.py` subprocess, restarts it on failure |
| `bridge/protocol.rs` | The JSON-lines IPC protocol spoken to the bridge |
| `channel_b.rs` | Encode/decode `ChannelBFrame` |
| `hello_handler.rs`, `session_handler.rs`, `action_trigger_handler.rs`, `camera_control_handler.rs` | FlatBuffers ↔ core-type marshaling (decodes commands/triggers off the wire, encodes HELLO/SessionDescribe onto it) |
| `safety_task.rs` | Wires `roboprotocol-core`'s safety state machine, watchdog, and arbitration into a per-tick decision |
| `xgo_profile.rs` | Hardcoded `RobotProfile`/`CameraDescriptor` for the XGO-Lite V2 — the only robot v0 ships a profile for |
| `video/capture.rs`, `video/channel_a.rs` | `libcamera-vid`/`rpicam-vid` capture subprocess management + Channel A chunking |

### `operator-console` (binary)

The QUIC client + terminal UI that runs on the operator's workstation.

| Module | Responsibility |
| --- | --- |
| `main.rs` | CLI: `--connect`, `--bind`, `--server-name`, cert flags, `--video`/`--video-backend ffplay\|native`, `--video-overlay`, `--headless`, `--record-dir`/`--record-extra` |
| `quic_client.rs` | Connection setup + main loop, mirrors `quic_server.rs` |
| `ui.rs` | `ratatui` full-screen HUD (interactive), or a single greppable status line under `--headless` (what `scripts/smoke_test.sh` checks) |
| `input.rs` | Raw-terminal keyboard teleop input |
| `session_cache.rs` | Per-server-address 0-RTT session ticket persistence, plus the `(robot_id, profile_hash)` cache backing an honest `SESSION_ACCEPT.cached` |
| `channel_b.rs`, `hello_handler.rs`, `session_handler.rs`, `action_trigger_handler.rs`, `camera_control_handler.rs` | Reverse-role marshaling vs. `robot-edge` |
| `video/playback.rs` | Pipes decoded H.264 into an `ffplay` subprocess |
| `video/native_playback.rs` | Experimental alternative: in-process `openh264` decode + `minifb` display, no external `ffplay` dependency |

## The schema: `roboprotocol.fbs`

A single 194-line FlatBuffers file (`crates/roboprotocol-proto/schemas/roboprotocol.fbs`),
namespace `roboprotocol`, deliberately not split across files (untested `flatc`
cross-file merge risk). No `root_type` declared — six independent top-level types,
each endpoint calls `flatbuffers::get_root::<T>()` directly for whichever one it
expects on a given stream/datagram.

| Type | Carries |
| --- | --- |
| `TLV` | Generic tag/value extension record |
| `RoboProtocolHello` | `protocol_version`, `capability_bitmask`, supported task classes/quantization tiers, `max_control_rate_hz`, `extensions` |
| `Codec` (enum) | `H265`, `AV1`, **`H264`** — v0 uses H264 since it's what the CM4's hardware encoder actually supports |
| `JointDescriptor`, `CommandShape` (enum: `Kinematic`/`VelocityAttitude`/`CartesianEndEffector`), `BodyRegionDescriptor`, `BaseType` (enum) | Robot morphology description |
| `RobotProfile` | The full robot description sent in `SESSION_DESCRIBE` |
| `CameraDescriptor`, `SessionDescribe`, `FieldQuantization`, `SessionAccept` | Session negotiation |
| `ChannelBCategory` (enum: `Command`/`Telemetry`/`Haptic`), `ChannelBFrame` | `timestamp`, `seq`, `tick_id`, `category`, `region_id`, and an **opaque packed `fields:[ubyte]`** — byte offsets are derived from the negotiated `RobotProfile`, not self-describing, for datagram-budget reasons (see [Protocol Architecture](02-protocol-architecture.md#session_describe--session_accept--robot--media-profile)) |
| `ActionTrigger` | `action_id`, `trigger_seq` — Channel C stream 4 |
| `CameraControl` | `brightness`, `contrast`, `ev`, `shutter_us`, `control_seq` — Channel C stream 8, maps directly to `libcamera-vid` args |

## Tools

### `dev-certs` (binary)

Dev-only mTLS certificate generator built on `rcgen`: a self-signed CA plus robot and
operator leaf certificates. **Issues ECDSA P-256, not Ed25519** — a documented
deviation from `DESIGN.md` §4, because `quiche` 0.22.0's vendored BoringSSL fails to
load Ed25519 PKCS8 keys. `.key` files are written with `0600` permissions on Unix.

```bash
cargo run -p dev-certs -- --out-dir certs --robot-san <hostname-or-ip> [--no-overwrite]
```

### `replay-decode` (binary)

Decodes a recorded segment directory into CSV, reusing the same wire-format decode
logic as the live binaries rather than re-deriving it:

```bash
replay-decode <segment-dir> [--category command|telemetry|haptic|key-press|action-trigger]
```

`ActionTrigger` and `CameraControl` share the same `action-trigger-c` recording
category; `replay-decode` disambiguates them structurally via FlatBuffers vtable
byte-size (`>10` bytes ⇒ `CameraControl`), not by value-range heuristics. See
[Recording & Replay](07-recording-and-replay.md).

## Integration test: `scripts/smoke_test.sh`

A two-process localhost integration test with no real hardware required:

1. `cargo build --workspace --quiet`
2. Generates fresh dev certs into a temp directory via `cargo run -p dev-certs`
3. Starts `robot-edge --stub-bridge --robot-id smoke_test` on `127.0.0.1:14433`
   (configurable via `SMOKE_TEST_PORT`)
4. Starts `operator-console --headless` connected to it
5. Greps both logs for `HELLO negotiated successfully`, then for
   `SESSION_ACCEPT received` / `SESSION_DESCRIBE received`
6. Kills `operator-console`, then asserts `robot-edge`'s log shows `E-Stop latched`
   within 3 seconds (the Class D watchdog threshold is 400 ms; the test allows
   generous scheduling margin)
7. Exits non-zero and dumps both logs on any failed check

```bash
./scripts/smoke_test.sh
```

This is the fastest way to confirm a change hasn't broken the handshake → session
setup → Channel B → watchdog chain, without touching real hardware.

## Simulator (`simulator/`)

A separate, Python discrete-event **packet-level network/protocol simulator** —
not a robot-dynamics simulator. It validates the spec's own NFR/SR claims (RTT,
payload sizing, quantization, FEC, safety tiers, watchdog, a TDPA passivity proxy)
against injected network failures, independent of the Rust reference implementation.
Explicitly out of scope: WBC/ZMP balance, real video codecs, ROS 2/DDS, kinematic
retargeting.

```bash
python -m roboprotocol_sim run --scenario dual_arm --failure blackout \
  --network cellular_5g --duration 30 --out simulator/output/my-run
python -m roboprotocol_sim sweep --network home_broadband_wifi6 --out simulator/output
python -m roboprotocol_sim list-scenarios
python -m roboprotocol_sim list-failures
```

- **Scenarios:** `simple_arm`, `arm_wrist_cam`, `arm_wrist_fixed_cam`, `dual_arm`.
- **Failure presets:** `bandwidth_drop`, `blackout`, `packet_loss_20`/`packet_loss_40`,
  `jitter_spike`, `latency_ramp`, `latency_spike_recover`, `handover` (implements
  `DESIGN.md` §6's reactive path-validation switch), `compound_congestion`, `none`.
- **Network profiles:** `home_broadband_wifi6` (12 ms OWD, 3 ms jitter, 0.1% loss,
  20/20 Mbps), `cellular_5g` (20 ms OWD, 8 ms jitter, 0.5% loss, 30/15 Mbps).
- **`--compare-naive`** replays identical conditions against a modeled naive
  WebSocket/TCP baseline (one reliable ordered stream, full float32, fixed bitrate,
  no FEC/watchdog). Headline finding: under `blackout`, RoboProtocol E-Stops within
  200 ms while the naive baseline's ping/pong liveness check never fires at all.
- Each run emits `summary.md` (PASS/FAIL against seven `REQUIREMENTS.md` clauses),
  plots (`rtt_and_tier.png`, `bandwidth.png`, `video_bitrate.png`,
  `tdpa_energy.png`), and raw CSVs; `sweep` additionally emits `comparison.md`.
- 38 pytest tests cover sizing/quantization math, safety-tier boundaries, the TDPA
  proxy, FlexFEC's recovery boundary, naive-baseline behavior, and end-to-end
  scenario×failure smoke runs.

See `simulator/README.md` and `simulator/SPEC.md` for the full design.

## Building everything

```bash
cargo build --workspace --release
```

For running the reference implementation against the real XGO-Lite V2 hardware, see
the [XGO-Lite V2 Guide](06-xgo-lite-guide.md).
