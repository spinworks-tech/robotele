#!/usr/bin/env bash
# Syncs local crates/ (robot-edge and everything it depends on --
# roboprotocol-core, roboprotocol-recording, etc.) + xgo_bridge.py to the
# Pi, then builds robot-edge there. `/home/pi/RoboProtocol` is a plain
# directory, not a git clone -- it never auto-updates on its own.
#
# Usage: xgo_bridge/scripts/deploy.sh <pi-user@host> [--release]
set -euo pipefail

PI="${1:?usage: deploy.sh <pi-user@host> [--release]}"
# cargo's own dev-profile output directory is named "debug", not "dev" --
# only the release profile's directory name matches its flag.
OUT_DIR="debug"
CARGO_FLAGS=()
if [[ "${2:-}" == "--release" ]]; then
    OUT_DIR="release"
    CARGO_FLAGS=(--release)
fi

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$REPO_ROOT"

echo "== syncing crates/ =="
rsync -az --exclude target --exclude .git \
    -e "ssh -o StrictHostKeyChecking=accept-new" \
    crates/ "$PI:/home/pi/RoboProtocol/crates/"

echo "== syncing xgo_bridge.py =="
scp -o StrictHostKeyChecking=accept-new \
    xgo_bridge/xgo_bridge.py "$PI:/home/pi/RoboProtocol/xgo_bridge/xgo_bridge.py"

echo "== building robot-edge on the Pi ($OUT_DIR) =="
# shellcheck disable=SC2029  # deliberately expanded client-side, this is a client-side variable
ssh -o StrictHostKeyChecking=accept-new "$PI" \
    "source ~/.cargo/env && cd /home/pi/RoboProtocol && cargo build -p robot-edge ${CARGO_FLAGS[*]}"

echo "== done -- binary at /home/pi/RoboProtocol/target/$OUT_DIR/robot-edge =="
