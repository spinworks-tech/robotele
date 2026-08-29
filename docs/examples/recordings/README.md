# Example recorded session

A real ~2-minute session captured with `operator-console --record-dir` against a
live XGO-Lite V2 (see [Recording & Replay](../07-recording-and-replay.md)) —
included so you can see the on-disk `.rec` format and its converted output without
running the reference implementation yourself first.

## Layout

```text
recordings/
├── video-a/                    Raw Channel A segment(s) -- H.264 Annex-B, framed per record
├── channel-b-command/          Raw Channel B command segment(s) -- FlatBuffers TeleopCommand
├── channel-b-telemetry/        Raw Channel B telemetry segment(s) -- FlatBuffers Telemetry
├── key-press/                  Raw operator key-press segment(s)
└── converted/                  Human-usable output, from tools/replay/convert_recordings.sh
    ├── video-a.mp4              Re-encoded with crosshair, live fps/kbps, capture_us and arm-state overlays
    ├── channel-b-command.csv    capture_us,arm_x,arm_z,claw
    ├── channel-b-telemetry.csv  capture_us,seq,tick_id,region_id,battery,roll,pitch,yaw,motors
    └── key-press.csv            capture_us,input
```

This session recorded no haptic or `ActionTrigger`/`CameraControl` traffic, so
`convert_recordings.sh` silently skipped those two categories — normal, not an
error (see [Recording & Replay § Converting recordings for review](../07-recording-and-replay.md#converting-recordings-for-review)).

## Reproducing this conversion

```bash
cargo build --release -p replay-decode
tools/replay/convert_recordings.sh docs/examples/recordings /tmp/converted-example
```
