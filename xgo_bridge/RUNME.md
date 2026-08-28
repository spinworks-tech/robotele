# Running against the real XGO-Lite V2

Three pieces: `robot-edge` (this repo's Rust QUIC server) + `xgo_bridge.py` (this
directory — spawned by `robot-edge` as a subprocess, talks to the vendor `xgolib` SDK
over serial) run **on the Pi** attached to the robot; `operator-console` runs on
**your machine**.

## Quick start (scripts)

`xgo_bridge/scripts/` has one script per step below, all taking `<pi-user@host>` as
their first argument (e.g. `pi@192.168.68.64` — see [Find the Pi](#1-find-the-pi) for
why that address changes). Run from the repo root:

```bash
xgo_bridge/scripts/free-serial.sh   pi@<ip>                        # clear the serial port
xgo_bridge/scripts/deploy.sh        pi@<ip> [--release]             # sync + build robot-edge
xgo_bridge/scripts/run-detached.sh  pi@<ip> --robot-id xgo_real --camera  # run, logged, survives logout
xgo_bridge/scripts/clean.sh         pi@<ip> [recordings-dir]         # clear old logs/recordings
```

`run-detached.sh` forwards everything after `<pi-user@host>` straight to `robot-edge`,
so any flags from [Run robot-edge on the Pi](#4-run-robot-edge-on-the-pi) or
[Recording](#recording) below go there too. The sections below explain what each script
does and why, for troubleshooting or if you'd rather run the steps by hand.

## Certs: nothing to set up

Dev-only mTLS certs are committed at `certs/` in the repo root (see `certs/README.md`)
at the exact paths both binaries default to. **Don't pass `--cert`/`--key`/`--ca` —
just don't.** They already work for both loopback stub testing and the real robot
over the LAN, regardless of the Pi's current IP: verification is by the `robot-edge`
DNS name in the cert's SAN (via `--server-name robot-edge`, robot-edge's own default),
not by IP, so the Pi's address changing on every reboot (see below) doesn't need new
certs. If you ever do need to regenerate: `cargo run -p dev-certs -- --out-dir certs`.

## 1. Find the Pi

**The Pi's LAN IP changes on every reboot** (DHCP) — check your router or `arp -a`
for the current one rather than assuming a previously-used address still works. A
`Connection refused` or `No route to host` on an address that worked last time almost
always just means the Pi rebooted, not that something broke.

First SSH to a never-before-seen IP will hit `Host key verification failed` under
non-interactive/batch SSH (no `known_hosts` entry yet) — use
`ssh -o StrictHostKeyChecking=accept-new pi@<ip>` for that first connection.

## 2. Clear the serial port

XGO's vendor auto-start services grab `/dev/ttyAMA0` on every boot and will block
`robot-edge`/`xgo_bridge.py` from opening it. `xgo_bridge/scripts/free-serial.sh pi@<ip>`
does the below; by hand:

```bash
ssh pi@<ip> 'sudo lsof /dev/ttyAMA0'
```

Two seen so far, both safe to kill, both auto-starting on every boot (so you'll do
this again after every Pi reboot — on this unit they've also been observed to respawn
mid-session, not just at boot, so if the port gets held again later, just re-run this):

```bash
ssh pi@<ip> 'pgrep -f "python3 main.py"'       # find the pid, then:
ssh pi@<ip> 'sudo kill -9 <pid>'
```

**Use `pgrep`+`kill -9 <pid>`, not `pkill -f` directly** — on this Pi, killing either
vendor process (by either method) reliably drops the *current* SSH session as a side
effect (looks like a brief network interruption on this vendor image), but `pkill -f`'s
broader matching risks catching more than the intended target while it's at it. Either
way the kill itself always lands; just reconnect and verify with a fresh
`ssh pi@<ip> 'sudo lsof /dev/ttyAMA0'` rather than continuing in the same session — see
`free-serial.sh` for the scripted version of that dance.

If a *different* process shows up holding the port, don't assume it's safe to kill —
check what it is first.

## 3. Sync + build on the Pi

`/home/pi/RoboProtocol` on the Pi is a plain directory, not a git clone — it does not
auto-update. `xgo_bridge/scripts/deploy.sh pi@<ip> [--release]` does the below; by hand,
after any local change to `crates/*` or `xgo_bridge/xgo_bridge.py`:

```bash
rsync -az --exclude target --exclude .git crates/ pi@<ip>:/home/pi/RoboProtocol/crates/
scp xgo_bridge/xgo_bridge.py pi@<ip>:/home/pi/RoboProtocol/xgo_bridge/xgo_bridge.py
ssh pi@<ip> 'source ~/.cargo/env && cd /home/pi/RoboProtocol && cargo build -p robot-edge'
```

Sync the **whole** `crates/` directory, not just `robot-edge/src/` — it depends on
`roboprotocol-core` and `roboprotocol-recording` too, and a stale copy of either fails
the build on the Pi with confusing "no such item" errors that have nothing to do with
whatever you actually changed.

(`cargo`/`rustup` are at `~/.cargo/bin` on the Pi but not on the non-interactive SSH
`PATH`, hence sourcing `~/.cargo/env` first. Native aarch64 build, no cross-compiling.)

## 4. Run robot-edge on the Pi

`xgo_bridge/scripts/run-detached.sh pi@<ip> --robot-id xgo_real --camera` runs this
**detached from the SSH session** (survives you logging off) with output saved to a
timestamped, line-buffered log file, and stops any already-running `robot-edge` first.
By hand, in a session you're willing to leave open:

```bash
ssh pi@<ip>
cd /home/pi/RoboProtocol
./target/debug/robot-edge --robot-id xgo_real --camera
```

- No `--stub-bridge` — that's for hardware-free dev/CI testing only; omitting it uses
  the real serial backend (`--serial-port` defaults to `/dev/ttyAMA0`, already correct).
- `--camera` enables the onboard `ov5647` sensor via `libcamera-vid` (already installed
  on this Pi) for the video feed.
- `robot-edge` serves one connection at a time but survives a connection ending (crash,
  clean quit, network drop) and loops back to accept the next one — you don't need to
  restart it between operator-console reconnects, only after a Pi reboot.
- If you do run it by hand over plain SSH rather than `run-detached.sh`, closing that
  SSH session normally kills it (no `nohup`) — plan to leave the session open, or use
  the script.

## 5. Run operator-console on your machine

```bash
cd /path/to/RoboProtocol
./target/debug/operator-console --connect <pi-ip>:4433 --server-name robot-edge --video
```

`--video` opens the camera feed in its own `ffplay` window (separate from the terminal
HUD — video is deliberately not composited into the TUI).

Add `--video-overlay` to also burn the current brightness/contrast/EV/shutter values
directly into that video window (top-left corner, semi-transparent black box behind
white text), updated live every time you nudge one with `b`/`f`/`v`/`h`/`9` — useful
since the TUI's own HUD and the `ffplay` window are separate OS windows you can't
always see at once. This works by pointing `ffplay`'s own `-vf drawtext=...:reload=1`
filter at a small text file `operator-console` rewrites on every change — no periodic
polling, no custom rendering, `ffplay` just re-reads the file every frame. **Requires
your `ffplay`'s `ffmpeg` build to have `libfreetype` (and `libfontconfig`, so the plain
`font=monospace` name resolves without an explicit font file path)** — check with
`ffmpeg -hide_banner -buildconf | grep -E 'freetype|fontconfig'`. If your build lacks
it, `ffplay` will fail to start entirely with this flag on rather than silently skip
the overlay — that's why it's opt-in, not the `--video` default. `--video-overlay`
without `--video` is a startup error, not a silent no-op.

## Keybindings

Every key below is read straight off the terminal in raw mode — no Enter needed, and
holding a key relies on your terminal's own key-repeat, not a custom timer (see
`input.rs`'s module doc for why: the more robust Kitty keyboard protocol that would give
real press/release events misbehaves badly enough on some terminals to break input
parsing entirely).

### Movement & turning

| Keys | Does |
|---|---|
| `w`/`a`/`s`/`d` | Move forward/left/back/right (held velocity, auto-stops ~400ms after the key stops repeating — see `--move-stale-ms`) |
| `left`/`right` | Turn in place (same held-velocity behavior) — a **walking** rotation, the feet actually step; continuous, no fixed range |
| `space` | Explicit stop (zeroes move + turn immediately) |
| `1` / `2` | Stand / sit (canned gaits) |
| `e` / `c` | E-Stop (latches, refuses all motion commands) / clear |

### Whole-body attitude (roll / pitch / yaw)

Tilts the robot's body — and the head-mounted camera rigidly fixed to it — **without
walking**. Uses `xgolib`'s `attitude(axis, deg)`: all four feet stay planted, and the
vendor SDK's own inverse kinematics redistributes the tilt across all 12 leg joints
(hip/knee/ankle-equivalent on every leg) to hold that pose — it is not a knee-only
trick, and not something you'd derive from leg geometry by hand. This is the
answer to "how do I point the camera up/down for a grasping task" — you don't need a
manual IK computation, the vendor SDK already does it.

| Keys | Does | Range |
|---|---|---|
| `up`/`down` (or `=`/`-`) | Camera pitch (tilt up/down) | +/-10 deg |
| `[` / `]` | Roll (lean left/right) **(sign unverified — see caveats)** | +/-20 deg |
| `,` / `.` | Yaw twist — pan in place, feet planted, distinct from `left`/`right`'s walking turn above **(sign unverified — see caveats)** | +/-12 deg |
| `0` | Reset roll/pitch/yaw to level (0 deg) in one shot | — |

Ranges are XGO-Lite's real vendor limits (`ATTITUDE_LIMIT` in `xgolib`'s source on the
real Pi — the xgomini variant's are wider, e.g. +/-22 deg pitch, don't assume they
transfer). These are **held positions, not velocities** — they don't auto-stop, and
each keypress is a small nudge, not continuous motion, same as arm/claw below. Pitch's
sign was confirmed inverted from the intuitive on-screen direction (`up` sends a
*negative* delta internally) after real-hardware testing; roll/yaw are an educated
guess, not yet confirmed the same way — swap the corresponding deltas in
`crates/operator-console/src/input.rs` if either turns out backwards.

### Camera image controls (brightness / contrast / exposure / shutter)

Tunes how the video feed itself looks — separate from attitude above, and mechanically
very different. These are plain `libcamera-vid` CLI flags
(`--brightness`/`--contrast`/`--ev`/`--shutter`), not an XGO SDK call: no robot motion
is involved, and there's no vendor-confirmed "real" hardware range the way attitude has
— the ranges below are a chosen sane sub-range of what `libcamera-vid` technically
accepts, picked for what's actually useful on this sensor, not a hardware limit.
**Focus is not adjustable at all** — the camera is a fixed-focus OV5647 sensor with no
autofocus motor; if the image is out of focus, that's a physical twist of the lens
barrel on the module, nothing here can fix it.

| Keys | Does | Range |
|---|---|---|
| `b` / `B` | **Brightness** down/up — shifts the whole image darker/lighter | -1.0..1.0 (default 0.0) |
| `f` / `F` | **Contrast** down/up — how much difference between dark and light areas; low = flat/washed-out, high = punchy | 0.0..2.0 (default 1.0) |
| `v` / `V` | **Exposure compensation (EV)** down/up — the simplest overall "too dark/too bright" fix, biases the camera's own auto-exposure without taking manual control of it | -4.0..4.0 stops (default 0.0) |
| `h` / `H` | **Manual shutter speed** down/up — how long the sensor collects light per frame, in microseconds; longer = brighter but more motion blur, shorter = crisper on fast motion but darker. `H` (up) makes it *longer* | 0 (auto-exposure) .. 100000 us (default 0/auto) |
| `9` | Reset all four to defaults in one shot | — |

Want to see these values burned directly into the video window instead of just the
TUI's command panel? Pass `--video-overlay` alongside `--video` — see
[step 5](#5-run-operator-console-on-your-machine).

Two things that make these behave differently from every other key in this table:

1. **No live-tuning interface exists in `libcamera-vid`**, so applying any one of these
   means `robot-edge` kills and restarts the capture subprocess on the Pi with new
   args. Expect a brief (roughly half a second to a second) video freeze/reconnect on
   *every single keypress* — that's the tool, not a bug, and it's why these aren't
   designed for smooth continuous nudging the way attitude is.
2. **Held state, sent as its own message, not a per-tick field.** Wire-protocol-wise
   this rides a new `CameraControl` type on its own Channel C reliable stream
   (`roboprotocol_core::camera_control`), the same "discrete event, not continuous
   Channel B state" shape as `ActionTrigger`'s canned stand/sit gaits — sending it every
   tick like `vx`/`vy` would restart the camera process constantly.

Shutter's "auto" (0) and manual value are on the same linear scale as everything else
here (no log/multiplicative stepping), which means covering a wide brightness range
manually takes many presses of `H` — that's a known coarseness tradeoff for
implementation simplicity, easy to retune in `input.rs`'s `SHUTTER_STEP_US` if it
proves too slow in practice.

### Arm & claw

| Keys | Does |
|---|---|
| `i`/`k` | Arm up/down (Z axis) |
| `j`/`l` | Arm back/forward (X axis) |
| `u`/`o` | Claw open/close **(direction unverified — flip in `input.rs` if backwards; see caveats)** |

Arm/claw are also **held positions, not velocities** (`xgolib`'s `arm(x, z)`/
`claw(pos)` hold wherever last commanded) — same nudge-per-keypress behavior as
attitude above, real ranges `x: -80..155mm`, `z: -95..155mm`.

### System

| Keys | Does |
|---|---|
| `r` | Start/stop local recording (video, commands, telemetry, key presses) — see [Recording](#recording) below |
| `q` | Quit |

### What's on screen

The telemetry panel's leg-joint readout is a 2x2 grid — `FL`/`FR`/`RR`/`RL`, one small
block per leg, each showing that leg's lower/middle/upper joint angles on one line —
rather than a flat 12-15 line list, so it fits alongside the rest of the HUD without
scrolling. If the arm is attached (15 motors instead of 12), its 3 joints get one extra
summary line below the grid. The command panel separately shows currently-held
attitude (`roll`/`pitch`/`yaw`, next to their real ranges) and camera control values, so
you can always see the actual state you've nudged to, not just the delta you last sent.

## Recording

Both sides can independently log their own traffic to local rotating files (FR-9,
[DESIGN.md §10](file:///home/robomotic/DevOps/RoboProtocol/DESIGN.md)) — video, commands,
telemetry, `ActionTrigger`, and (operator-console only) every key you press. It's off by
default on both ends; nothing is recorded unless you ask for it.

### Turn it on

**robot-edge** — config-only, set at launch, no runtime toggle (it has no keyboard):

```bash
./target/debug/robot-edge --robot-id xgo_real --camera \
  --record-dir /home/pi/recordings --record video,command,telemetry
```

**operator-console** — `--record-dir` makes recording *available*; the `'r'` key then
starts/stops the default set (video, your own commands, telemetry, key presses) at
runtime, so a normal session isn't recorded unless you press it:

```bash
./target/debug/operator-console --connect <pi-ip>:4433 --server-name robot-edge --video \
  --record-dir ./recordings
```

Press `r` once you're `operating` to start; press it again to stop. `--record-extra
haptic,action` adds those two categories from launch instead (they have no natural
on/off point of their own during a session, so they're not part of the `'r'` toggle).

### Settings

Both binaries take the same rotation/retention flags — `--help` on either shows current
defaults (robot-edge's are smaller, sized for the Pi's SD card; operator-console's are
larger, sized for a workstation):

| Flag | Controls |
|---|---|
| `--record-max-segment-mb` | Rotate to a new file once the current one hits this size |
| `--record-max-segment-secs` | ...or this age, whichever comes first |
| `--record-budget-mb` | Total on-disk cap per category (video excluded) — oldest files get deleted first once exceeded |
| `--record-video-budget-mb` | Same, but for video specifically (usually much smaller than the others — video is orders of magnitude bigger per second than command/telemetry) |
| `--record-flush-secs` | How often buffered writes get flushed to disk |

### Confirm it's actually recording

- **On screen**: the header shows `● REC 12s` (red) while active, with a dropped-record
  count appended if the writer's falling behind (`● REC 12s (3 dropped)`) — recording is
  designed to degrade under pressure rather than ever block your controls, so a few
  drops during a burst isn't a bug, just the writer catching up. `rec: off ('r' to
  start)` (grey) means it's not running.
- **On disk**: each category gets its own subdirectory under `--record-dir`, named after
  the segment's start time so `ls -la` sorts chronologically:

  ```bash
  find ./recordings -type f -exec ls -la {} \;
  # ./recordings/video-a/00001787217814008577.rec
  # ./recordings/channel-b-command/00001787217813005434.rec
  # ./recordings/key-press/00001787217864890016.rec
  ```

  A file that exists but stays at 0 bytes for a while is normal — writes are buffered
  and flushed periodically (`--record-flush-secs`, default 2s on robot-edge / 5s on
  operator-console), not per record. If a category's directory never appears at all,
  double check it was actually named in `--record`/`--record-extra`, or toggled on with
  `'r'`.

### Convert recordings for review

`.rec` segments aren't directly watchable/readable — `tools/replay/convert_recordings.sh
<recordings-dir> [output-dir]` turns every category it finds into something that is,
output defaulting to `<recordings-dir>/converted`:

```bash
cargo build --release -p replay-decode   # once
tools/replay/convert_recordings.sh recordings
```

| Category directory | Becomes |
|---|---|
| `video-a` | `video-a.mp4` — re-encoded with a crosshair, live fps/kbps, and each frame's own `capture_us` burned in; also the arm x/z/claw state as of each frame if `channel-b-command` is present too |
| `channel-b-command` | `channel-b-command.csv` — `capture_us,arm_x,arm_z,claw` |
| `channel-b-telemetry` | `channel-b-telemetry.csv` — `capture_us,seq,tick_id,region_id,battery,roll,pitch,yaw,motors` (motor angles `;`-joined, since the count varies by robot profile) |
| `key-press` | `key-press.csv` — `capture_us,input` (the operator's own keypresses) |
| `action-trigger-c` | `action-trigger-c.csv` — `capture_us,kind,...`; shared between `ActionTrigger` and `CameraControl` sends, `kind` says which |
| `channel-b-haptic` | `channel-b-haptic.csv` — raw hex; nothing in this codebase sends haptic frames yet, so there's no real layout to decode against |

A category directory that doesn't exist, or exists but is empty (not every session
records everything — see "Turn it on" above), is silently skipped. Each conversion is
also available standalone: `target/release/replay-decode <dir> --category <name>` for
just the CSV, `tools/replay/recording_to_mp4.py --help` for the video's own overlay
flags individually.

Recordings (and old `robot-edge-*.log` files) accumulate fast and the Pi's SD card is
small — `xgo_bridge/scripts/clean.sh pi@<ip> [recordings-dir]` clears both; run it
occasionally or when `df` on the Pi starts looking tight.

## Known caveats

- **Claw direction unverified.** `u`=open/`o`=close in `input.rs` is a best guess;
  `xgolib`'s exact `claw(pos)` polarity hasn't been confirmed against real hardware.
  If it's backwards, swap the two `ClawNudge` deltas in `crates/operator-console/src/input.rs`.
- **Test arm movement in a clear space first**, especially near the edges of its
  range (`x: -80..155mm`, `z: -95..155mm`).
- **`robot-edge` only serves one connection at a time.** If a handshake stalls at
  "awaiting HELLO" for more than a few seconds, it's usually real network flakiness on
  the WiFi link, not a code bug — quit and retry.
- **Two distinct connectivity failure modes seen on real hardware, don't confuse them:**
  1. *Pi actually reboots* (power loss, not a software crash) — `ping`/`ssh` both fail
     with `No route to host`/`Destination Host Unreachable`, `robot-edge`'s log just
     stops mid-line with no panic/traceback, and `journalctl --list-boots` on the Pi
     afterward shows a new boot with no clean shutdown logged before it. Confirmed this
     is really power loss, not a kernel panic: no under-voltage warnings anywhere in
     `journalctl -k`, and `vcgencmd get_throttled` reads clean once back up, so it looks
     like a loose/marginal power connector rather than a sagging supply — check the
     cable/connector physically. `robot-edge` has no supervisor (no systemd unit), so it
     does **not** come back on its own after this — always re-run
     `run-detached.sh`/step 4 once the Pi is reachable again.
  2. *Pi never reboots, just a transient WiFi drop* — `robot-edge`'s own log shows a
     clean `connection closed ... awaiting next connection` and stays running
     (`ps`/uptime confirm the same process, same boot, the whole time); nothing to
     restart, just reconnect `operator-console`. A likely contributor on this unit: the
     Broadcom WiFi chip (`brcmfmac`, BCM4345/6) enables power-save at boot
     (`brcmf_cfg80211_set_power_mgmt: power save enabled` in `dmesg`), which can make a
     mesh AP (e.g. TP-Link Deco) flag the Pi as briefly "offline" even though it's
     actually just dozing between packets and answers fine once polled. Fixed on this
     Pi with a udev rule so it's disabled on every boot, not just once by hand:
     ```bash
     echo 'ACTION=="add", SUBSYSTEM=="net", KERNEL=="wlan0", RUN+="/sbin/iw dev wlan0 set power_save off"' \
       | sudo tee /etc/udev/rules.d/70-wifi-powersave-off.rules
     sudo udevadm control --reload-rules
     sudo /sbin/iw dev wlan0 set power_save off   # apply immediately, don't wait for next boot
     ```
     This is a **Pi OS-level config, not part of this repo** — `/home/pi/RoboProtocol`
     doesn't carry it, so it needs re-doing if the Pi's SD card is ever re-flashed.
