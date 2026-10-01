# DimOS integration spike (discussion #6)

Findings from reading DimOS (shallow clone of `main`, Sept 2026) against this
repo. dimTELE / WebRTC is out of scope by decision.

## What DimOS gives us

- **Transport contract** (`dimos/core/stream.py`): `Transport` needs `start/stop`,
  `broadcast(stream, msg)` and `subscribe(callback) -> unsubscribe`. Its own TODO in
  `dimos/core/transport.py` says the intended end state is just `send(msg)` and
  `receive(callback)`. Bytes only; no notion of safety state or session.
- **Backends today**: `TransportBackend = Literal["lcm", "zenoh"]`, with Zenoh
  the default (`docs/usage/transports/index.md`). There is no runtime plugin
  registry for transports, but a blueprint can pin a specific transport per
  stream: `Blueprint.transports({(name, type): Cls.spec(...)})`.
- **Plugin seam for whole robots**: the `dimos.blueprints` entry-point group
  (`dimos/robot/external_blueprints.py`) lets an out-of-tree package register
  blueprints. A `roboprotocol` package can live outside DimOS.
- **XGO-Lite**: no support in DimOS (no source hits, web search finds none).
  Supported hardware includes Go2, TRON1, xArm/Lite 6, R1 Lite, M20, drones.

## What we already have

`robot-edge` already runs a Zenoh sidecar (`crates/robot-edge/src/zenoh_bridge.rs`,
`BABYROS.md`) on keys `robotele/<id>/{telemetry,command,autonomy_goal}`. The
autonomy goal is freshness-gated and ranked below `FullTeleoperation` by the
arbitration ladder, so a remote autonomy source can never preempt teleop.

## Answer to the open question: where is the seam?

Two different seams, and they should not be conflated:

1. **Autonomy in (DimOS -> robot)**: already exists. A DimOS module publishing
   on `robotele/<id>/autonomy_goal` is arbitrated by the safety ladder. No new
   transport is required.
2. **Teleop control (operator -> robot)**: must not be a DimOS `Transport`. The
   safety state machine (TDPA, deadman, WBC override) is coupled to QUIC
   datagram timing and the session lifecycle; a bytes-only `broadcast/subscribe`
   interface would discard that. Keep it out-of-band: DimOS observes and
   feeds, RoboProtocol controls.

Recommendation: do **not** implement RoboProtocol as a DimOS transport backend.
Integrate at the Zenoh boundary instead.

## Proposed adapter (lives here, not in DimOS)

A Python package (e.g. `dimos-roboprotocol/`) exposing, via the
`dimos.blueprints` entry point:

- A `RoboteleBridge` module: DimOS streams <-> `robotele/<id>/*` Zenoh keys
  (telemetry -> DimOS odometry/joint state; DimOS goal -> `autonomy_goal`).
- An `xgo-lite` blueprint wrapping the above, so DimOS agents/skills can
  drive the XGO through robot-edge's arbitration rather than `xgolib` directly.

## Open items / next steps

1. Check the telemetry payload format (`BABYROS.md`, "Telemetry payload") maps
   onto DimOS message types (likely `sensor_msgs.JointState`/`Imu`); DimOS uses
   LCM-encoded typed messages on `dimos/<name>/<msg_name>` keys, so a
   translation layer is needed either way.
2. `autonomy_goal` now carries an optional `vx, vy, turn` velocity in robot-native
   units, clamped by `robot-edge` to the operator console's limits and dispatched
   only while arbitration selects `SemiAutonomous` (see `BABYROS.md`). The adapter
   maps a DimOS `Twist` (m/s, rad/s) onto it via `max_linear_mps` /
   `max_angular_rps`, which are **uncalibrated placeholders**: the XGO's real
   m/s per xgolib unit has not been measured. Calibrate before trusting odometry
   or nav planners.
3. Decide whether to propose the adapter upstream after the DimOS-side
   discussion (dimensionalOS/dimos#3751) responds.
