# BabyROS / Zenoh integration: setup and testing

`robot-edge` has an optional **Channel C sidecar** that talks to the outside
world over [Zenoh](https://zenoh.io/), the pub/sub bus that
[BabyROS](https://github.com/telekinesis-ai/babyros) is built on. Design
discussion: [#5](https://github.com/spinworks-tech/robotele/discussions/5).

Ships in the **[v0.1.2 release](https://github.com/spinworks-tech/robotele/releases/tag/v0.1.2)**
(`robot-edge-v0.1.2-babyros-aarch64-unknown-linux-gnu.tar.gz`), separate from the
plain `robot-edge-v0.1.1-*` build on `main`.

## What it does

| Direction | Zenoh key | What |
| --- | --- | --- |
| robot → world | `robotele/<robot-id>/telemetry` | Robot telemetry, ~1 Hz (the bridge is polled every 50 ticks) |
| world → robot | `robotele/<robot-id>/autonomy_goal` | "A semi-autonomy source wants control" |

`<robot-id>` is robot-edge's `--robot-id` (e.g. `xgo_real`).

**Teleoperation over QUIC is untouched.** All operator control still goes over
RoboProtocol's QUIC transport; Zenoh is bolted on the side, never in series with it.

- Telemetry is published fire-and-forget through a depth-1 channel. A slow or
  absent Zenoh peer just drops samples; it can never stall a control tick.
- The autonomy goal is a single freshness-gated flag: **any** sample on the
  topic (payload ignored) counts as "asserted" for **500 ms** (`AUTONOMY_GOAL_TTL`).
  A stalled/crashed autonomy node stops asserting on its own; nothing latches.
- The flag feeds `arbitrate()`'s `autonomy_goal_asserted`. The ladder is
  `EStop > EmergencySafeParking > ActiveImpedanceHold > FullTeleoperation > SemiAutonomous`,
  so an autonomy goal only wins when the robot is not E-Stopped/suspended **and**
  teleop isn't ready (no deadman / stale command). Teleop always preempts it.
- If Zenoh fails to open, robot-edge logs a warning and runs without the sidecar.

Code: `crates/robot-edge/src/zenoh_bridge.rs`, wired in `quic_server.rs` and
`safety_task.rs`.

### Telemetry payload

Not JSON, and not BabyROS's own codec: it is robot-edge's compact Channel B
telemetry encoding (`TelemetryData::pack`), big-endian:

| Bytes | Field |
| --- | --- |
| 0 | battery, `u8`, percent |
| 1–2, 3–4, 5–6 | roll, pitch, yaw, `i16`, 0.01°/LSB |
| 7… | per-joint angles, `i16`, 0.01°/LSB, 2 bytes each (15 joints with the arm = 37 bytes total) |

```python
import struct
def unpack(b):
    battery = b[0]
    roll, pitch, yaw = (v / 100 for v in struct.unpack(">3h", b[1:7]))
    motors = [v / 100 for v in struct.unpack(f">{(len(b) - 7) // 2}h", b[7:])]
    return battery, roll, pitch, yaw, motors
```

## Setup

### 1. Robot (XGO-Lite V2 Pi CM4)

Needs the binary from v0.1.2 (built against glibc ≤ 2.31 to match the stock
Debian 11 image; check with `strings robot-edge | grep '^GLIBC_' | sort -V | tail -1`,
which must print `GLIBC_2.30` or lower).

```bash
gh release download v0.1.2 --repo spinworks-tech/robotele
sha256sum -c SHA256SUMS
tar xzf robot-edge-v0.1.2-babyros-aarch64-unknown-linux-gnu.tar.gz
scp -r robot-edge-v0.1.2-babyros-aarch64-unknown-linux-gnu pi@<robot-ip>:~/
```

Run it exactly as in the [XGO-Lite V2 guide](docs/06-xgo-lite-guide.md) (certs,
serial port, etc.). A start-up line like this means the sidecar is up:

```
zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/192.168.2.19:40109
```

**The Zenoh port is random on every start.** See [Known limitations](#known-limitations).

### 2. Something to talk to it

**Recommended for now: plain `eclipse-zenoh`.** It works on any Python ≥ 3.8
(the aarch64 wheel is `cp38-abi3`) and needs no patches:

```bash
pip install eclipse-zenoh
```

**BabyROS proper** (`pip install babyros`) needs Python ≥ 3.11 upstream, and
its subscriber won't decode robot-edge's telemetry, since it isn't in BabyROS's
codec (see limitations). If you use it, subscribe with `decode=False`:

```python
from babyros.node import Subscriber, Publisher
sub = Subscriber("robotele/xgo_real/telemetry", callback=lambda m: print(m), decode=False)
```

The Pi itself runs Python 3.9.2, which upstream BabyROS rejects. A working
3.9 port exists (`robomotic/babyros@py39-xgo-compat`) but its dependency
`telekinesis-datatypes` is proprietary and the patched copy isn't published, so
for on-robot consumers prefer plain `eclipse-zenoh`. Pi gotchas from that
port: `pyarrow` 21 crashes with SIGILL on the CM4's Cortex-A72 (pin
`pyarrow==18.0.0`), and the SD card is nearly full (see [SDCOPY.md](SDCOPY.md)).

### 3. Building from source

Don't build on the Pi (the SD card can't hold the build) or on Windows (the
quiche/BoringSSL link step fails). Use an Ubuntu box with Docker + `cross`:

```bash
docker build -f docker/cross-aarch64-glibc2.31.Dockerfile -t robotele/cross-aarch64-glibc2.31:latest .
cross build --release --target aarch64-unknown-linux-gnu -p robot-edge
strings target/aarch64-unknown-linux-gnu/release/robot-edge | grep '^GLIBC_' | sort -V | tail -1
```

`Cross.toml` points at that image (Ubuntu 20.04 base → glibc 2.31, plus flatc
v1.11.0). The stock `cross` image links glibc 2.39 and will not start on the robot.

## Testing

### Safe smoke test (no motors)

Run robot-edge with `--stub-bridge` (no serial port; the bridge just prints
`[stub] ...` instead of moving anything) on a spare port so it can't collide
with a real instance on 4433:

```bash
RUST_LOG=info ./robot-edge --stub-bridge --robot-id xgo_real --listen 0.0.0.0:4444 \
  --cert certs/robot/robot.crt --key certs/robot/robot.key --ca certs/dev-ca/ca.crt
```

Note the Zenoh port from the log (`tcp/<ip>:<port>`). **Certs:** the robot and
operator must chain to the same dev CA. The Pi's `RoboProtocol/certs` had a
different CA from the repo's `certs/`, so use one set on both sides.

### Zenoh from a PC

Multicast scouting often doesn't cross Windows firewalls or WiFi APs, so
connect to the endpoint directly:

```python
import time, zenoh
c = zenoh.Config()
c.insert_json5("connect/endpoints", '["tcp/192.168.2.19:40109"]')   # from the log
s = zenoh.open(c)

# The robot's autonomy_goal subscriber is visible:
pub = s.declare_publisher("robotele/xgo_real/autonomy_goal")
time.sleep(1.5)
print(pub.matching_status)            # MatchingStatus { matching: true }

# Telemetry arrives once a QUIC session is open (see next step):
got = []
s.declare_subscriber("robotele/xgo_real/telemetry", lambda smp: got.append(bytes(smp.payload)))
time.sleep(12)
print(len(got), "samples")            # ~1 per second
```

### Full loop with `operator-console`

Telemetry is only produced inside a QUIC session, so start the console too:

```bash
./operator-console --connect <robot-ip>:4444 --server-name robot-edge \
  --cert certs/operator/operator.crt --key certs/operator/operator.key --ca certs/dev-ca/ca.crt
```

The robot log should show `HELLO negotiated successfully`, `E-Stop cleared`,
`SESSION_ACCEPT received`. Then run the Zenoh snippet above and expect telemetry.

### Checking that the autonomy goal is honored

The log doesn't show the arbitrated control source, but the recordings do.
Start robot-edge with `--record-dir rec --record telemetry,command`, drive the
test, stop it, then read `rec/channel-b-telemetry/*.rec`. Each record is
`u32 payload_len | u64 capture_us | u8 control_source | payload` (big-endian):

```python
import struct, collections
d = open("00001789972891001153.rec", "rb").read(); i = 0; out = []
while i + 13 <= len(d):
    n, ts, cs = struct.unpack(">IQB", d[i:i + 13]); out.append((ts, cs)); i += 13 + n
names = {0: "EStop", 1: "SafeParking", 2: "ActiveImpedanceHold", 3: "FullTeleop", 4: "SemiAutonomous"}
print(collections.Counter(names[c] for _, c in out))
```

Expected behavior while a goal is being published at ~10 Hz:

- **E-Stopped or teleop active:** source stays `EStop` / `FullTeleop` (goal ignored).
- **Connected, E-Stop cleared, deadman not held:** source becomes `SemiAutonomous`,
  and returns to `ActiveImpedanceHold` ~500 ms after the goals stop.

## Status of testing

Verified on a real XGO-Lite V2 Pi CM4 with v0.1.2 (stub bridge, real
`operator-console` over QUIC): the binary starts and stays up, the Zenoh
session opens, the `autonomy_goal` subscriber matches from a PC, telemetry is
received at ~1 Hz, and no panics or errors are logged.

**Not yet verified:** that a goal actually flips the source to `SemiAutonomous`.
In the one recorded run the robot was either E-Stopped or in `FullTeleop`
whenever a goal was published, so the goal was correctly outranked but never won.
Doing this needs someone at the console to clear the E-Stop and not hold the
deadman. Also untested: the sidecar with the real (non-stub) bridge and motors.

## Known limitations

- **Random Zenoh port.** Clients must read it from the log or you must pin one
  (robot-edge doesn't yet set `listen/endpoints`).
- **Multicast scouting** may not reach across subnets/APs; use `connect/endpoints`.
- **Raw payloads vs BabyROS's codec.** BabyROS tags every payload with a 4-byte
  attachment (`JSON`, `NDAR`, …) and its decoder raises on an untagged one.
  robot-edge publishes raw bytes with no attachment, so BabyROS needs
  `decode=False` (or plain Zenoh). Conversely, the autonomy goal accepts anything,
  so a BabyROS `Publisher` works fine there. A shared schema is a follow-up.
- **Single writer for goals.** Any sample on the topic asserts the goal; there is
  no goal content, priority or authentication. Treat the Zenoh network as trusted.
- **Nothing is in the plain build.** `robot-edge-v0.1.1-*` from `main` has no
  Zenoh; use v0.1.2 for this feature.
