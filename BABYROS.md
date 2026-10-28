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
| robot → world | `robotele/<robot-id>/command` | The operator's command and the control source it was arbitrated to. **Not in the v0.1.2 binary**: needs a build from a later commit (see below) |
| world → robot | `robotele/<robot-id>/autonomy_goal` | "A semi-autonomy source wants control", optionally with a velocity (see below) |

`<robot-id>` is robot-edge's `--robot-id` (e.g. `xgo_real`).

**Teleoperation over QUIC is untouched.** All operator control still goes over
RoboProtocol's QUIC transport; Zenoh is bolted on the side, never in series with it.

- Telemetry is published fire-and-forget through a depth-1 channel. A slow or
  absent Zenoh peer just drops samples; it can never stall a control tick.
- The autonomy goal is a freshness-gated flag: **any** sample on the
  topic counts as "asserted" for **500 ms** (`AUTONOMY_GOAL_TTL`).
  A stalled/crashed autonomy node stops asserting on its own; nothing latches.
- The payload may carry a velocity: 12 bytes, big-endian `f32` `vx, vy, turn` in
  robot-native (xgolib) units, the same units the operator console sends. It is
  clamped to the console's own limits (`|vx|<=15`, `|vy|<=12`, `|turn|<=60`), and
  any other length (e.g. a 1-byte flag), NaN or infinity means zero velocity. While
  arbitration picks `SemiAutonomous`, `robot-edge` sends that velocity to the bridge
  every tick, and it decays to zero with the 500 ms TTL. This only runs while a QUIC
  session is up (the tick loop is per-session).
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

### Command payload

`robotele/<id>/command` is the operator's *requested* command plus the source the
arbitration ladder picked for it, published from the same place commands are
dispatched to the robot. It is still published while E-Stopped (you see what the
operator asked for even when it isn't applied). Big-endian, 30 bytes:

| Bytes | Field |
| --- | --- |
| 0 | control source `u8`: 0 EStop, 1 EmergencySafeParking, 2 ActiveImpedanceHold, 3 FullTeleoperation, 4 SemiAutonomous |
| 1–4, 5–8, 9–12 | vx, vy, turn, `f32` |
| 13–16, 17–20, 21–24 | attitude roll, pitch, yaw, `f32` |
| 25–26, 27–28 | arm x, arm z, `i16` |
| 29 | claw, `u8` |

To keep 50 Hz operator input from becoming 50 Hz of Zenoh traffic on the Pi, a
sample is sent only when the command or its source changes, plus a repeat every
250 ms while unchanged. Like telemetry it is fire-and-forget and can never stall
a control tick. Watching this topic while publishing to `autonomy_goal` shows the
source flip to `SemiAutonomous` (only when not E-Stopped and nobody is driving).

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
zenoh::net::runtime::orchestrator: Zenoh can be reached at: tcp/192.168.2.19:7447
```

The Zenoh listen port defaults to **7447** and no longer changes on every start (fixed since
this was written — see `--zenoh-port` below); override it with `--zenoh-port N` if 7447 is taken,
blocked, or you'd rather pin a different one for a firewall rule / a BabyROS `connect` endpoint.

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

## Starting and stopping

**There is no separate Zenoh service.** The sidecar runs inside the `robot-edge`
process: it opens its Zenoh session when `robot-edge` starts and closes it when
`robot-edge` exits. Starting/stopping the robot side means starting/stopping
`robot-edge`. There is also no flag to turn Zenoh off (only `--zenoh-port` to
move it); to run without it, use the plain `robot-edge-v0.1.1-*` build.

### Robot side

The launch tooling on the Pi runs the binary at
`/home/pi/RoboProtocol/target/release/robot-edge` (falling back to `target/debug/`).
To run the BabyROS build with that tooling, put it in that spot (keeping the
plain build so you can switch back; the SD card is tight, so check `df -h /` first):

```bash
ssh pi@<robot-ip>
cd /home/pi/RoboProtocol
cp target/release/robot-edge target/release/robot-edge.plain          # one-time backup
cp ~/robot-edge-v0.1.2-babyros-aarch64-unknown-linux-gnu/robot-edge target/release/robot-edge
chmod +x target/release/robot-edge
```

**Start** (any one of these; only one `robot-edge` can hold the QUIC port and
the serial port at a time):

```bash
# From your PC: detached from SSH, timestamped log, stops any running robot-edge first
xgo_bridge/scripts/run-detached.sh pi@<robot-ip> --robot-id xgo_real --camera
# Optional: --zenoh-port 7447 (default), --record-dir ... --record command,telemetry

# Or on the Pi by hand (dies when the SSH session closes; use for quick tests)
cd /home/pi/RoboProtocol && ./target/release/robot-edge --robot-id xgo_real --camera
```

**LCD panel** (`xgo-wifi-panel.service` → `wifi_edge_panel.py`): the ROBOT-EDGE
view starts/stops `robot-edge` with A and shows a `build:` line (plain / babyros)
and a `zenoh: :7447 UP/off` line. To use it without swapping files, install the
BabyROS binary next to the plain one instead of over it:

```bash
cp ~/robot-edge-v0.1.2-babyros-aarch64-unknown-linux-gnu/robot-edge \
   /home/pi/RoboProtocol/target/release/robot-edge.babyros
chmod +x /home/pi/RoboProtocol/target/release/robot-edge.babyros
```

When that file exists and `robot-edge` is stopped, button **C** switches the next
start between `plain` and `babyros`; the panel defaults to `plain` after every
panel restart, so it never runs the Zenoh build unless you pick it. If you move
Zenoh off 7447, set `ZENOH_PORT` at the top of the script (it then passes
`--zenoh-port`). After editing the script in the repo, copy it to the Pi
(`/home/pi/xgo_wifi_edge_panel.py`) and `sudo systemctl restart xgo-wifi-panel`.
`run-detached.sh` still only runs `target/release/robot-edge`, so use the
copy-over method above with that script.

**Check that Zenoh is up:**

```bash
grep "Zenoh can be reached" /home/pi/RoboProtocol/robot-edge-*.log | tail -1
ss -ltn | grep 7447          # listening on the Zenoh port
```

**Stop:**

```bash
pgrep -x robot-edge          # exact name; avoid `pkill -f`, see xgo_bridge/RUNME.md
kill <pid>                   # SIGINT/SIGTERM is fine; `kill -9 <pid>` if it hangs
```

(`run-detached.sh` and the panel do this for you when they start/stop it.)
Stopping `robot-edge` ends the QUIC session, so any connected `operator-console`
loses its connection, and closes Zenoh: telemetry stops and the autonomy goal
stops being honored immediately (nothing latches).

**Switch back to the plain build:**

```bash
cd /home/pi/RoboProtocol && cp target/release/robot-edge.plain target/release/robot-edge
```

**Running the side-by-side BabyROS binary by hand.** Always start from the
folder that contains `certs/`, because the default `--cert/--key/--ca` paths are
relative (`certs/robot/robot.crt`, `certs/robot/robot.key`, `certs/dev-ca/ca.crt`):

```bash
cd /home/pi/RoboProtocol
./target/release/robot-edge.babyros --robot-id xgo_real --camera
# add --stub-bridge to run without the serial port (no motors move)
```

The panel and `run-detached.sh` already start from `/home/pi/RoboProtocol`.

**Troubleshooting: `Error: loading robot cert chain` / `TlsFail`.** You started
it from a directory with no `certs/` folder (typically `/home/pi`). quiche reports
a missing certificate file with the same generic `TlsFail` as an invalid one, so it
looks like a bad cert. `cd /home/pi/RoboProtocol` first, or pass absolute paths:
`--cert /home/pi/RoboProtocol/certs/robot/robot.crt --key .../robot.key --ca .../dev-ca/ca.crt`.

### Consumer side (BabyROS / Zenoh node)

Your BabyROS or plain-Zenoh client is its own process on whatever machine runs
it; start it with `python your_node.py` and stop it with Ctrl+C. **Order doesn't
matter**: either side can start first and Zenoh connects them once both are up
(use `connect/endpoints` to the robot's `tcp/<robot-ip>:7447` if multicast
discovery doesn't find it). If the consumer stops, `robot-edge` keeps running
unaffected: telemetry just has no listener and any autonomy goal expires within
500 ms.

## Testing

### Safe smoke test (no motors)

Run robot-edge with `--stub-bridge` (no serial port; the bridge just prints
`[stub] ...` instead of moving anything) on a spare port so it can't collide
with a real instance on 4433:

```bash
RUST_LOG=info ./robot-edge --stub-bridge --robot-id xgo_real --listen 0.0.0.0:4444 \
  --cert certs/robot/robot.crt --key certs/robot/robot.key --ca certs/dev-ca/ca.crt
```

Zenoh listens on the fixed `--zenoh-port` (default 7447; the log still confirms the address). **Certs:** the robot and
operator must chain to the same dev CA. The Pi's `RoboProtocol/certs` had a
different CA from the repo's `certs/`, so use one set on both sides.

### Zenoh from a PC

Multicast scouting often doesn't cross Windows firewalls or WiFi APs, so
connect to the endpoint directly:

```python
import time, zenoh
c = zenoh.Config()
c.insert_json5("connect/endpoints", '["tcp/192.168.2.19:7447"]')   # default --zenoh-port
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

### Live monitor (`zenoh-monitor`)

A small Rust tool that subscribes to `robotele/<robot-id>/**`, decodes the
telemetry and the operator commands, and shows the data flowing: one line per
telemetry sample, command lines like `cmd [FullTeleop] vx=+0.40 turn=+0.00 ...`
(at most ~5/s, but always immediately when the control source or moving/idle
state changes), plus a line every 5 s with connected peers and per-topic rates.
Other topics (e.g. `autonomy_goal`, ~10 Hz) are counted rather than printed unless
you pass `--all`. Command lines need a `robot-edge` build that includes the
`command` topic; against v0.1.2 you'll just see telemetry.
Source: `tools/zenoh-monitor`. The Windows build is in the demo folder next to the
operator console (`C:\Users\Public\RoboProtocol-Demo\bin\zenoh-monitor.exe`).

```powershell
.\bin\zenoh-monitor.exe                    # robot at 192.168.2.19:7447, id xgo_real
.\bin\zenoh-monitor.exe --connect tcp/192.168.2.19:7447 --robot-id xgo_real --all
.\bin\zenoh-monitor.exe --only commands    # just the operator's commands (and autonomy goals)
.\bin\zenoh-monitor.exe --only status      # just telemetry + the 5 s summary
.\bin\zenoh-monitor.exe --only commands --changes   # print a command only when it changes
```

`--changes` (alias `--diff`) skips a command that is identical to the last one
printed. `robot-edge` re-sends an unchanged command every 250 ms, so without it a
held key repeats on screen; with it you get one line per real change (key pressed,
released, source changed). Skipped commands are still counted in the summary.

`--only commands` hides telemetry and the summary line (the summary is still
printed if there are no peers, so "not connected" can't be mistaken for "no
commands"); `--only status` hides the command lines but the summary still counts them.

Telemetry only flows while an `operator-console` session is open. `peers=0`
means it can't reach the robot's Zenoh port: check the IP, that `robot-edge`
is the BabyROS build, and that the panel shows `zenoh: ... UP`. Build it with
`cargo build --release -p zenoh-monitor` (PowerShell on Windows). Multicast
discovery on the same LAN also works, but `--connect` is more reliable.

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

## Forwarding topics

Discussion: [#36](https://github.com/spinworks-tech/robotele/discussions/36).
BabyROS nodes on the operator's PC and on the robot can talk to each other
without changing any code: each host keeps its own Zenoh, and only the link
between them goes over RoboProtocol's QUIC session.

```
 operator PC                                            robot (CM4)
 BabyROS nodes <-> Zenoh <-> operator-console ==QUIC==> robot-edge <-> Zenoh <-> BabyROS nodes
                   :7448                        Wi-Fi                 :7447
```

Each side subscribes to the key expressions it was told to forward, sends the
samples across, and the other side republishes them on its own Zenoh under the
same key. The payload and the attachment travel byte for byte, so BabyROS's
codec works unchanged on the far side.

| Mode | Carried as | Use for |
| --- | --- | --- |
| `latest` | one QUIC datagram per sample; newest per key while waiting; may be lost | small state that only matters while fresh (pose, status). A sample has to fit in one datagram (about 1.4 KB including its key); larger ones are dropped and counted |
| `reliable` | one QUIC stream per direction, shared by all reliable topics; in order, nothing lost | goals, maps, anything that must arrive or is larger than a datagram |

Forwarded traffic never gets ahead of control. On the robot, latest-mode
datagrams take turns with video and sensor data while both are waiting (one
each), and the reliable stream is below everything but bulk objects. Under
an uplink overload the reliable stream gets little, because QUIC packets
carry datagrams first: see "Measured on the CM4" below.

### Running it

Robot:

```bash
./target/release/robot-edge ... --zenoh-local-only \
    --zenoh-forward 'latest:robot/pose' --zenoh-forward 'reliable:robot/map/**' \
    --zenoh-accept 'planner/**'
```

Operator:

```bash
./target/release/operator-console --connect <robot-ip>:4433 --server-name robot-edge \
    --zenoh-forward 'reliable:planner/**' --zenoh-accept 'robot/**'
```

- `--zenoh-forward MODE:KEY_EXPR` (repeatable) is what *this* side sends.
- `--zenoh-accept KEY_EXPR` (repeatable) is the allowlist for what the other
  side sends: a forwarded sample is republished only if one of these includes
  its key (Zenoh matching, so `planner/**` also covers `planner`). **With no
  `--zenoh-accept`, nothing is republished.** A rejected key is logged once and
  counted (`republish_rejected` in the session-end log line).
- `--zenoh-topics FILE` reads the same rules from a file, alongside any flags:

  ```text
  # operator side
  forward reliable planner/**
  accept robot/**
  ```

- `--zenoh-max-mbps N` caps what *this* side forwards (latest and reliable
  together); default **2 Mbps**, `0` for no cap, `max-mbps N` in a topics file.
  While the cap binds, latest samples wait and newer ones replace them, and
  reliable samples wait in a backlog that holds about 2 s at the capped rate
  (at least 1 MB, so a sample up to that size always fits; 4 MB with no cap).
  A reliable sample that doesn't fit is dropped and counted. It exists because forwarded
  volume can delay commands on a weak link (see "Measured on the CM4").
- Any of these turns forwarding on; `--zenoh-bridge` turns it on with no rules.
- The console opens its own Zenoh session, by default listening on
  `tcp/127.0.0.1:7448` only, with multicast scouting off (`--zenoh-listen` to
  change the endpoint, `--zenoh-config FILE` for a full Zenoh config). The HUD
  shows a `C (zenoh in)` row.
- `--zenoh-local-only` makes robot-edge's Zenoh listen on loopback only, with
  multicast scouting off (see below for why).

BabyROS nodes then connect to their own host's endpoint. Replacing BabyROS's
config drops the timestamping it turns on by default, so set it again:

```python
import zenoh
from babyros.node import SessionManager

cfg = zenoh.Config()
cfg.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7448"]')  # 7447 on the robot
cfg.insert_json5("scouting/multicast/enabled", "false")
cfg.insert_json5("timestamping/enabled", "true")
SessionManager.set_session_config(cfg)  # before creating any node
```

### Older builds and plain teleop

Forwarding is negotiated in HELLO (capability bit `TOPIC_FORWARDING`): each side
sends topic samples only when both have forwarding on. Every combination keeps
teleoperation working; checked on localhost with the `main` build:

| operator-console | robot-edge | Result |
| --- | --- | --- |
| with forwarding | older build, or no forwarding | session and teleop as usual; console logs that the robot doesn't forward topics; nothing sent |
| older build, or no forwarding | with forwarding | session and teleop as usual; robot logs that the operator doesn't forward topics; nothing sent |
| with forwarding | with forwarding | topics forwarded both ways |

The older console logs an `InvalidStreamState` warning per bulk object whether
or not forwarding is involved; that's fixed in this build.

### Keep the two Zenoh domains apart

If the PC's Zenoh can also reach the robot's directly (for example a node with
default multicast scouting on the same Wi-Fi), every forwarded sample would
arrive twice and loop between the two sides. robot-edge declares a liveliness
token `robotele/forwarder/<robot-id>`; while the console can see it on its own
Zenoh, it **pauses forwarding in both directions** and logs a warning (the HUD
row shows `PAUSED`). Fix it by running robot-edge with `--zenoh-local-only` and
turning multicast scouting off on the PC's nodes, as above.

### Rules and limits

- Keys under `robotele/` are never forwarded or republished: they are
  robot-edge's own, and `robotele/<id>/autonomy_goal` asserts semi-autonomy.
- Only samples from other Zenoh sessions are forwarded, so a republished sample
  isn't sent back. Deletes aren't forwarded.
- Samples captured before a session is up are discarded when it starts, rather
  than delivered stale.
- A reliable sample that would overflow the waiting backlog is dropped
  (counted in the session-end log line); see `--zenoh-max-mbps` for its size. One large sample delays the other
  reliable topics behind it, since they share a stream.
- Each sample carries its key (no per-session topic ids yet), so a long key
  costs bytes in every latest-mode datagram.
- Not carried: BabyROS `Server`/`Client` (queries), `list_topics` (liveliness),
  `TRANSIENT_LOCAL` history for late joiners, and Zenoh timestamps (the robot's
  clock can be days off anyway).
- Not yet tested with real BabyROS (Python) nodes; `zenoh_probe` stands in for them.

### Measured on the CM4

2026-10-28, XGO-Lite CM4 at -34 dBm on 2.4 GHz channel 6, laptop also on
Wi-Fi, robot-edge with `--stub-bridge` (`benchmark/topic_forwarding_wifi.sh`).
Round trips go laptop -> bridge -> a Zenoh echo on the robot -> bridge ->
laptop, timed on the laptop only. "Link" is ICMP ping during the run.

| Run | Link avg / max | latest 32 B: back, p50 / p90 | reliable 32 B: back, p50 / p90 | reliable 100 KB at 5 Hz: back, p50 | 300 KB maps to operator | E-Stop latched |
| --- | --- | --- | --- | --- | --- | --- |
| idle, good link | 40 / 216 ms | 280/300, 23 / 156 ms | 300/300, 8 / 107 ms | 50/50, 193 ms | 10/10 | no |
| sim lidar, before the fix below | 104 / 344 ms | **0/300** | 300/300, 164 / 334 ms | 18/50, 6.1 s | 1/10 | yes |
| sim lidar, no forwarding traffic (control) | – | – | – | – | – | yes, 19 s in |
| sim lidar, after the fix | 335 / 1,539 ms | 63/300, 489 / 2,185 ms | 294/300, 1.2 / 3.0 s | 8/50, 8.3 s | 0/10 | yes |
| sim nav cloud | 357 / 1,756 ms | 115/300, 1.2 / 2.2 s | 300/300, 317 ms / 2.0 s | 17/50, 5.7 s | 0/10 | yes |
| idle again, link now degraded | 137 / 1,035 ms | 283/300, 65 / 318 ms | 300/300, 143 / 475 ms | 4/50, 9.0 s | 10/10 | yes, in the 100 KB phase |

What it shows:

- **On a good link it works:** every reliable sample arrived, small ones in
  8 ms median, and latest mode lost about 7%, as datagrams on this Wi-Fi do.
- **Latest mode was starved under sensor load** (0 of 300), because topic
  datagrams went only when video and sensors had nothing waiting, which with
  lidar is never. Now they take turns; the robot then sent 67 instead of 1 (the
  link had also got much worse by then, so the delivered count isn't comparable).
- **Sim lidar alone latches the watchdog** (control row), as doc 12 found for
  overload: that isn't forwarding.
- **Forwarded volume can hurt control on a weak link.** With the link degraded
  and no sensors, 100 KB reliable samples 5 times a second each way (about
  4 Mbps per direction) coincided with a watchdog latch; the same phase on the
  good link didn't. Commands go first into each packet, but they still queue
  behind forwarded packets in the Wi-Fi driver and the access point. That run
  was uncapped; `--zenoh-max-mbps` (default 2) now limits this.
- The link swung from 40 ms to over 300 ms average within 20 minutes, so the
  rows after the first two are not a clean comparison.

**The cap, A/B.** Later the same day, three alternating pairs with no cap and
the default 2 Mbps cap (maps every 2 s, 0.5 Hz). Each test phase's start is
given from the session start: ICMP only 0 s, latest 23 s, reliable 32 B 41 s,
100 KB at 5 Hz (4 Mbps offered each way) 58 s, maps 71 s.

| Run | Link avg / max | reliable 32 B p50 / p90 | 100 KB back | maps | Watchdog latched at |
| --- | --- | --- | --- | --- | --- |
| 1, no cap | 162 / 1,215 ms | – (the laptop lost Wi-Fi) | 0/50 | 0/10 | +25 s |
| 1, cap 2 | 86 / 452 ms | 48 / 199 ms | 3/50 | 9/10 | +61 s (100 KB) |
| 2, no cap | 78 / 822 ms | 246 / 1,308 ms | 50/50 | 10/10 | +54 s (32 B) |
| 2, cap 2 | 69 / 439 ms | 43 / 138 ms | 2/50 | 3/10 | +62 s (100 KB) |
| 3, no cap | 30 / 332 ms | 50 / 141 ms | 32/50 | 10/10 | +63 s (100 KB) |
| 3, cap 2 | 108 / 751 ms | 39 / 137 ms | 9/50 | 10/10 | +15 s (ICMP only) |

- The cap holds forwarded traffic to its rate: with 4 Mbps offered, few 100 KB
  samples came back, as designed (the backlog is 2 s at the cap, 1 MB here).
- **Inconclusive on control.** That evening the link latched the watchdog with
  no forwarded traffic at all (run 3 capped, +15 s), and capped and uncapped
  runs both latched in the 100 KB phase. Showing whether the cap protects
  commands needs the doc 12 control benchmark (command latency, not one
  latch flag) on a quieter link.
- A first capped A/B, before the backlog was sized from the cap, had 0 of 10
  maps arrive: the fixed 4 MB backlog was 16 s of queue at 2 Mbps.

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

- **Multicast scouting** may not reach across subnets/APs; use `connect/endpoints`
  (pointed at the fixed `--zenoh-port`, default 7447, rather than a port read from the log).
- **Raw payloads vs BabyROS's codec.** BabyROS tags every payload with a 4-byte
  attachment (`JSON`, `NDAR`, …) and its decoder raises on an untagged one.
  robot-edge publishes raw bytes with no attachment, so BabyROS needs
  `decode=False` (or plain Zenoh). Conversely, the autonomy goal accepts anything,
  so a BabyROS `Publisher` works fine there. A shared schema is a follow-up.
- **Single writer for goals.** Any sample on the topic asserts the goal; there is
  no goal content, priority or authentication. Treat the Zenoh network as trusted.
- **Nothing is in the plain build.** `robot-edge-v0.1.1-*` from `main` has no
  Zenoh; use v0.1.2 for this feature.
