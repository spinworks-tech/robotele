#!/usr/bin/env bash
# Starts robot-edge on the Pi detached from this SSH session (survives
# logout, keeps running after you disconnect) with output saved to a
# timestamped log file. Line-buffers that output explicitly via `stdbuf`
# -- without it, output redirected to a file (rather than a TTY) sits
# unflushed in the process's block buffer for a long time, so `tail -f`
# on the log shows nothing for minutes even though the process is fine.
# Stops any already-running robot-edge first, since only one can hold the
# listen port (and the serial port) at a time.
#
# Usage: xgo_bridge/scripts/run-detached.sh <pi-user@host> [robot-edge args...]
# Example:
#   xgo_bridge/scripts/run-detached.sh pi@192.168.68.64 --robot-id xgo_real --camera
#   xgo_bridge/scripts/run-detached.sh pi@192.168.68.64 --robot-id xgo_real --camera \
#     --record-dir /home/pi/recordings --record command,telemetry
set -euo pipefail

PI="${1:?usage: run-detached.sh <pi-user@host> [robot-edge args...]}"
shift

ssh -o StrictHostKeyChecking=accept-new "$PI" bash -s -- "$@" <<'REMOTE'
set -e
cd /home/pi/RoboProtocol

# -x: exact process-name match, not `-f` (full command line) -- avoids
# false positives against anything else that happens to mention
# "robot-edge" in its own arguments (e.g. a deploy/verification command).
existing="$(pgrep -x robot-edge || true)"
if [ -n "$existing" ]; then
    echo "stopping existing robot-edge (pid: $(echo "$existing" | tr '\n' ' '))"
    kill -9 $existing
    sleep 1
fi

BIN=target/release/robot-edge
[ -x "$BIN" ] || BIN=target/debug/robot-edge

LOG="robot-edge-$(date +%Y%m%d-%H%M%S).log"
nohup stdbuf -oL -eL "$BIN" "$@" > "$LOG" 2>&1 < /dev/null &
disown
sleep 1

echo "started, log: $LOG"
pgrep -af robot-edge
REMOTE
