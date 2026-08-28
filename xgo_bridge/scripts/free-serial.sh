#!/usr/bin/env bash
# Kills the known XGO vendor auto-start processes holding /dev/ttyAMA0 on
# the Pi, so robot-edge/xgo_bridge.py can open it. These auto-start on
# every boot, so re-run this after every Pi reboot.
#
# Usage: xgo_bridge/scripts/free-serial.sh <pi-user@host>
set -euo pipefail

PI="${1:?usage: free-serial.sh <pi-user@host>}"
SSH=(ssh -o StrictHostKeyChecking=accept-new "$PI")

echo "== who's holding /dev/ttyAMA0 =="
"${SSH[@]}" 'sudo lsof /dev/ttyAMA0 2>/dev/null || echo "(nothing holding it)"'

echo "== killing known vendor processes (main.py, app_dogzilla.py) =="
# If a *different*, unrecognized process shows up holding the port above,
# don't assume it's safe to kill -- check what it is first, then extend
# this script rather than widening the pattern blindly.
#
# pgrep-then-kill-by-pid, not `pkill -f` directly, to avoid matching more
# than intended. Either way, killing `main.py` on this robot reliably
# drops the *current* SSH session as a side effect (confirmed repeatedly
# -- looks like it briefly disrupts networking on this specific vendor
# image) even though the kill itself always lands successfully. So: this
# step is expected to disconnect and report a non-zero exit here -- that
# is not a failure, it's why verification below reconnects fresh instead
# of continuing in the same session.
"${SSH[@]}" '
    pids="$(pgrep -f "python3 main.py") $(pgrep -f "app/app_dogzilla.py")"
    for pid in $pids; do
        echo "killing pid $pid"
        sudo kill -9 "$pid" 2>/dev/null || true
    done
' || echo "(SSH session dropped -- expected, see comment above; verifying with a fresh connection)"

sleep 2
echo "== after (fresh connection) =="
"${SSH[@]}" 'sudo lsof /dev/ttyAMA0 2>/dev/null || echo "(clear)"'
