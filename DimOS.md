# DimOS integration: setup and testing

[DimOS](https://github.com/dimensionalOS/dimos) is a Python "agentic operating system" for
robots. This repo connects to it through `robot-edge`'s Zenoh sidecar (the same one described
in [BABYROS.md](BABYROS.md)), using the adapter package in [dimos-roboprotocol/](dimos-roboprotocol/).
Background and the design decision are in
[docs/10-dimos-integration-spike.md](docs/10-dimos-integration-spike.md)
(discussion [#6](https://github.com/spinworks-tech/robotele/discussions/6)).

**Teleoperation over QUIC is untouched.** DimOS is never in the control path. It observes
telemetry and can assert a semi-autonomy goal, which the arbitration ladder ranks below
`FullTeleoperation` (`EStop > EmergencySafeParking > ActiveImpedanceHold > FullTeleoperation >
SemiAutonomous`). RoboProtocol is deliberately *not* a DimOS transport backend: a DimOS
`Transport` is bytes-only and cannot carry the safety state machine.

## What works today

| Direction | Zenoh key | DimOS stream | Notes |
| --- | --- | --- | --- |
| robot → DimOS | `robotele/<id>/telemetry` | `joint_state` (`JointState`) | 15 joints, degrees converted to radians |
| robot → DimOS | `robotele/<id>/telemetry` | `battery_percent` (`Float32`) | 0-100 |
| robot → DimOS | `robotele/<id>/telemetry` | `imu` (`Imu`) | orientation only, see below |
| robot → DimOS | `robotele/<id>/video` | `color_image` (`Image`, RGB) | H.264, only while subscribed, see "Video" |
| DimOS → robot | `robotele/<id>/autonomy_goal` | `cmd_vel` (`Twist`, input) | see "Driving the robot" |

`<id>` is `robot-edge`'s `--robot-id` (default in the adapter: `xgo_real`).

### IMU

Telemetry carries roll/pitch/yaw only: no gyro and no accelerometer. The `imu` stream therefore
has a valid `orientation` (built from the Euler angles) and marks `angular_velocity` and
`linear_acceleration` as not provided with covariance `[0] = -1`, the ROS convention, so
consumers don't mistake the zeros for measurements. Yaw is whatever the robot reports (0-360),
but a quaternion cannot hold that, so anything reading `orientation` back gets it wrapped to
±180° (327.67° becomes -32.33°). The Rerun "Attitude (deg)" panel shows the wrapped values.

### Joint names

The robot sends no joint names; these are ours, from the wire order in
`crates/robot-edge/src/xgo_profile.rs`:

| Index | Name |
| --- | --- |
| 0-2 | `front_left_{lower,middle,upper}` |
| 3-5 | `front_right_{lower,middle,upper}` |
| 6-8 | `rear_right_{lower,middle,upper}` |
| 9-11 | `rear_left_{lower,middle,upper}` |
| 12-14 | `arm_1`, `arm_2`, `arm_3` |

The arm values are servo readback of the optional accessory. The vendor SDK documents no
per-joint control or limits for it (it is commanded in Cartesian x/z plus a claw value), so
treat `arm_*` as raw readback, not calibrated joint angles. A 12-value layout (base kit
without the arm) gets the leg names only.

## Setup

DimOS needs **Python 3.10-3.12 on Ubuntu 22.04+ or macOS**. It does not build on native Windows
(its LCM dependency needs GLib), and the XGO's Raspberry Pi is too small for it (Python 3.9 on
Debian 11, 1.8 GB RAM, and a full root disk when this was written). Run DimOS on a separate
machine and reach the robot over Zenoh; the Pi only runs `robot-edge`, as usual.

On Windows this was done in WSL2 (Ubuntu 20.04, below DimOS's tested minimum, but it worked):

```bash
# uv provides Python 3.12 without touching the system Python
curl -LsSf https://astral.sh/uv/install.sh | sh
uv venv --python 3.12 ~/dimos-venv
cd dimos-roboprotocol
VIRTUAL_ENV=~/dimos-venv uv pip install -e . "dimos[web,visualization]"
```

The `web` and `visualization` extras are needed for the viewer. The web dashboard also needs
`git-lfs` on `PATH`; without sudo, drop the static binary from the
[git-lfs releases](https://github.com/git-lfs/git-lfs/releases) into `~/.local/bin`.

`Image.to_rerun()` (used to show `color_image` in Rerun, see "Video" below) needs
`libturbojpeg`, a system library `pip install av` does not provide. Without it every
`color_image` sample throws inside DimOS's Rerun bridge (logged as "Error in subscribe_all
callback", with no traceback reaching the log because it happens in a worker process) — decode
and the Zenoh publish still work, only the Rerun display fails. Without sudo, extract the
shared library instead of installing it:

```bash
mkdir -p ~/.local/lib-extract && cd ~/.local/lib-extract
apt-get download libturbojpeg   # fetches the .deb, no root needed
dpkg-deb -x libturbojpeg_*.deb extracted
mkdir -p ~/.local/lib && cp extracted/usr/lib/*/libturbojpeg.so.0 ~/.local/lib/
export LD_LIBRARY_PATH=~/.local/lib:$LD_LIBRARY_PATH   # needed every run, before `dimos`
```

`turbojpeg.py`'s library search checks `LD_LIBRARY_PATH` for the exact filename
`libturbojpeg.so.0`, so this works without touching the system library path.

Run the tests for the pure-Python codec (no DimOS needed):

```bash
cd dimos-roboprotocol && PYTHONPATH=src python -m pytest tests -q
```

## Running

`robot-edge` must be running on the robot (see [BABYROS.md](BABYROS.md)), and the DimOS machine
must reach its Zenoh port (default `7447`). Telemetry is polled from the control tick, so it
only flows while a QUIC session is up.

```bash
source ~/dimos-venv/bin/activate
export ROBOTELE_ZENOH_CONNECT=tcp/<robot-ip>:7447
dimos --rerun-open web run dimos-roboprotocol.xgo-lite
```

`ROBOTELE_ZENOH_CONNECT` is read by the bridge module (or set `zenoh_connect` in its config).
DimOS's own `ZENOH_CONNECT` does not apply: the bridge opens its own Zenoh session.

Check the wire without DimOS:

```python
import time, zenoh
from dimos_roboprotocol.codec import decode_telemetry
c = zenoh.Config(); c.insert_json5("connect/endpoints", '["tcp/<robot-ip>:7447"]')
s = zenoh.open(c)
s.declare_subscriber("robotele/*/telemetry", lambda x: print(decode_telemetry(bytes(x.payload))))
time.sleep(5)
```

## Viewing

DimOS uses [Rerun](https://rerun.io/). With `--rerun-open web` the dashboard is at
`http://localhost:7779/` and the Rerun web viewer at port `9878`. The blueprint adds three
time-series panels (`Battery %`, `Attitude (deg)`, `Joint angles (deg)`) under `telemetry/`.

Two problems seen with the released `dimos` 0.0.10.post2:

- **Blank Rerun pane on the dashboard.** The dashboard template
  (`dimos/web/templates/rerun_dashboard.html`) points the iframe at ports `9090`/`9876`, but the
  package serves the viewer on `9878` and gRPC on `9877` (`dimos/visualization/rerun/constants.py`).
  Patch those two numbers in the installed template, or open the viewer directly:
  `http://localhost:9878/?url=rerun%2Bhttp%3A%2F%2Flocalhost%3A9877%2Fproxy`.
- **"Rerun has crashed" / `createBuffer failed, size (33554432) is too large`.** The browser's
  WebGPU rejects a buffer for the 3D view. Add `&renderer=webgl` to the viewer URL.

Why a custom Rerun config at all: DimOS's Rerun bridge only draws messages that have a
`.to_rerun()` method, and neither `Float32` nor `JointState` does, so they are silently
skipped. `dimos-roboprotocol/src/dimos_roboprotocol/rerun_views.py` sideloads converters via
`visual_override` and supplies the panel layout.

## Video

`robot-edge` captures H.264 with `libcamera-vid` and sends it to the operator console as QUIC
datagrams (Channel A). It also taps the same NAL units onto Zenoh, `robotele/<id>/video`, **but
only while at least one Zenoh subscriber matches**: with nobody subscribed the tap is a no-op
(no copy, no queue), so an idle robot pays nothing. The bridge module subscribes (config
`video: true`, the default) and decodes with PyAV into `color_image`.

- **Payload:** one NAL unit per Zenoh sample, Annex-B start code prepended, in stream order.
  Concatenated they are a valid H.264 stream.
- **Delivery mirrors Channel A:** SPS/PPS/IDR are queued and never dropped by congestion
  control (queue bounded at 32 so a wedged peer cannot grow memory); all other NALs are
  latest-wins and dropped under congestion. The publisher runs in its own task, so a slow
  Zenoh peer cannot stall the QUIC video path or a control tick.
- **Joining mid-stream:** the latest SPS/PPS are replayed when a subscriber appears; the picture
  starts at the next IDR (about a second with `--intra 30` at 30 fps).
- **Session-bound:** capture is per QUIC session, so video needs the operator console (or any
  QUIC client) connected with `--camera`, like telemetry.
- **Cost:** decoding needs PyAV (`pip install av`, or `pip install -e ".[video]"`), and showing
  frames in Rerun needs `libturbojpeg` (see "Setup"). The viewer is throttled to 10 Hz (`max_hz`
  in `rerun_views.py`) because frames are logged as raw RGB. Watching adds roughly the camera
  bitrate (about 4 Mbps by default) of Zenoh traffic and some CM4 CPU; whether that disturbs the
  control loop on the real robot has not been measured yet.
- **Verified against the real robot** (robot-edge v0.1.3, 2026-09-22): a stale `libcamera-vid`
  process left over from an earlier `robot-edge` run held the camera and made a fresh session's
  capture fail every time until it was killed (capture gives up permanently per session on
  repeated failure, see `capture.rs`; reconnecting the operator console after the kill got a
  clean capture). Once that and the `libturbojpeg` gap were cleared, `color_image` streamed at
  around 16 fps end to end from the real camera with no further errors.

## Driving the robot (autonomy goal)

A DimOS planner or agent publishing `cmd_vel` (`Twist`) makes the bridge put a sample on
`robotele/<id>/autonomy_goal`. The payload is 12 bytes, big-endian `f32` `vx, vy, turn` in
**robot-native (xgolib) units**, the same units the operator console sends. `robot-edge`:

- clamps to the console's own limits (`|vx|<=15`, `|vy|<=12`, `|turn|<=60`);
- treats any other payload length (e.g. a legacy 1-byte flag), NaN or infinity as zero velocity;
- honors a sample for 500 ms (`AUTONOMY_GOAL_TTL`), so a stalled sender stops the robot;
- sends the velocity to the bridge each tick only while arbitration selects `SemiAutonomous`,
  i.e. not E-Stopped/suspended and no ready teleop. Teleop always preempts it.

Limitations:

- **Uncalibrated units.** The adapter maps `Twist` (m/s, rad/s) to robot-native units with
  `max_linear_mps=0.25` and `max_angular_rps=1.0`. These are placeholders: the XGO's real m/s
  per xgolib unit has not been measured. Calibrate before trusting odometry or a nav planner.
- **Session-bound.** The tick loop is per QUIC session, so autonomy only acts while a session is
  up. It is not a headless autonomy mode.
- **Untested on hardware.** The Rust unit tests pass, but nothing has driven the real robot from
  DimOS yet. (`cargo test` does not link on native Windows because of quiche's BoringSSL; run it
  in Linux/WSL with `flatc` v1.12.0 on `PATH`, e.g. `CARGO_TARGET_DIR=~/robotele-target cargo test -p robot-edge`.)

## Known gaps

- **Video works, verified against the real robot** (see "Video" above). The autonomy-goal
  velocity path is still untested on hardware.
- **No real IMU data.** No angular velocity or acceleration reaches DimOS, because the robot
  edge does not send any; adding it means extending the telemetry format.
- **XGO-Lite is not a DimOS-supported robot.** No upstream module or hardware support exists;
  this adapter is out-of-tree, registered through the `dimos.blueprints` entry-point group.
