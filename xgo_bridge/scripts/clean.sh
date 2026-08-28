#!/usr/bin/env bash
# Removes robot-edge-*.log crash/session logs and (optionally) old
# recording segments on the Pi. Both accumulate fast across dev sessions
# and the SD card is small -- run this occasionally, or when df shows
# disk pressure.
#
# Safe to run while robot-edge is live: deleting its currently-open log
# file doesn't disrupt the running process (it keeps writing fine via the
# file descriptor it already holds) -- the file just becomes invisible to
# `ls`/`find` immediately, and the disk space isn't actually reclaimed
# until that process exits. If you want to keep watching the current
# session's log, note its name before running this.
#
# Usage: xgo_bridge/scripts/clean.sh <pi-user@host> [recordings-dir]
set -euo pipefail

PI="${1:?usage: clean.sh <pi-user@host> [recordings-dir]}"
RECORDINGS_DIR="${2:-}"
SSH=(ssh -o StrictHostKeyChecking=accept-new "$PI")

echo "== removing /home/pi/RoboProtocol/robot-edge*.log =="
"${SSH[@]}" 'rm -fv /home/pi/RoboProtocol/robot-edge*.log 2>/dev/null || echo "(none found)"'

if [[ -n "$RECORDINGS_DIR" ]]; then
    echo "== removing recordings under $RECORDINGS_DIR =="
    "${SSH[@]}" "rm -rfv '$RECORDINGS_DIR'"
else
    echo "(no recordings dir given -- pass one as \$2 to also clear it, e.g. /home/pi/recordings)"
fi

echo "== disk space now =="
"${SSH[@]}" 'df -h / | tail -1'

echo "== reclaiming cargo build cache if disk is still tight (safe -- just rebuild artifacts) =="
"${SSH[@]}" 'df --output=avail / | tail -1' | awk '{if ($1 < 1000000) print "low disk space -- consider: ssh '"$PI"' \"source ~/.cargo/env && cd /home/pi/RoboProtocol && cargo clean\""}'
