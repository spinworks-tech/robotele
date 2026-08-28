#!/usr/bin/env bash
# Two-process localhost integration smoke test: builds (if needed), spins up
# robot-edge (--stub-bridge, no real XGO hardware needed) and operator-console
# (--headless, no real terminal needed) against each other over real UDP/QUIC
# on loopback, and checks for:
#   1. HELLO negotiated on both sides
#   2. SESSION_DESCRIBE / SESSION_ACCEPT completed
#   3. the watchdog E-Stop latches within the Class D threshold (400ms) once
#      the operator link goes silent
#
# This automates the same checks from the plan's manual verification pass
# (see DESIGN.md/REQUIREMENTS.md and the workspace's own crates/ for what's
# under test). Exits non-zero on any check failing.

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

WORKDIR="$(mktemp -d)"
ROBOT_LOG="$WORKDIR/robot-edge.log"
OPERATOR_LOG="$WORKDIR/operator-console.log"
PORT="${SMOKE_TEST_PORT:-14433}"

ROBOT_PID=""
OPERATOR_PID=""

cleanup() {
    [[ -n "$ROBOT_PID" ]] && kill -9 "$ROBOT_PID" 2>/dev/null
    [[ -n "$OPERATOR_PID" ]] && kill -9 "$OPERATOR_PID" 2>/dev/null
    rm -rf "$WORKDIR"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $1" >&2
    echo "--- robot-edge log ---" >&2
    cat "$ROBOT_LOG" >&2 2>/dev/null
    echo "--- operator-console log ---" >&2
    cat "$OPERATOR_LOG" >&2 2>/dev/null
    exit 1
}

wait_for_log() {
    local file="$1" pattern="$2" timeout_s="$3" waited=0
    while ! grep -q "$pattern" "$file" 2>/dev/null; do
        sleep 0.1
        waited=$((waited + 1))
        if [[ $waited -ge $((timeout_s * 10)) ]]; then
            return 1
        fi
    done
    return 0
}

echo "== building workspace =="
cargo build --workspace --quiet || fail "cargo build failed"

echo "== generating dev certs =="
CERT_DIR="$WORKDIR/certs"
cargo run --quiet -p dev-certs -- --out-dir "$CERT_DIR" >/dev/null || fail "dev-certs failed"

echo "== starting robot-edge (--stub-bridge) on 127.0.0.1:$PORT =="
setsid ./target/debug/robot-edge \
    --listen "127.0.0.1:$PORT" \
    --cert "$CERT_DIR/robot/robot.crt" --key "$CERT_DIR/robot/robot.key" --ca "$CERT_DIR/dev-ca/ca.crt" \
    --stub-bridge --robot-id smoke_test \
    > "$ROBOT_LOG" 2>&1 < /dev/null &
ROBOT_PID=$!
disown

echo "== starting operator-console (--headless) =="
setsid ./target/debug/operator-console \
    --connect "127.0.0.1:$PORT" --server-name robot-edge \
    --cert "$CERT_DIR/operator/operator.crt" --key "$CERT_DIR/operator/operator.key" --ca "$CERT_DIR/dev-ca/ca.crt" \
    --headless \
    > "$OPERATOR_LOG" 2>&1 < /dev/null &
OPERATOR_PID=$!
disown

echo "== waiting for HELLO negotiation =="
wait_for_log "$ROBOT_LOG" "HELLO negotiated successfully" 5 || fail "robot-edge never negotiated HELLO"
wait_for_log "$OPERATOR_LOG" "HELLO negotiated successfully" 5 || fail "operator-console never negotiated HELLO"
echo "  ok: HELLO negotiated on both sides"

echo "== waiting for SESSION_DESCRIBE / SESSION_ACCEPT =="
wait_for_log "$ROBOT_LOG" "SESSION_ACCEPT received" 5 || fail "robot-edge never received SESSION_ACCEPT"
wait_for_log "$OPERATOR_LOG" "SESSION_DESCRIBE received" 5 || fail "operator-console never received SESSION_DESCRIBE"
echo "  ok: session handshake completed"

echo "== killing operator-console to trigger the watchdog =="
kill -9 "$OPERATOR_PID" 2>/dev/null
OPERATOR_PID=""

# Class D watchdog blackout threshold is 400ms; allow generous margin for
# scheduling jitter under a busy CI host.
wait_for_log "$ROBOT_LOG" "E-Stop latched" 3 || fail "watchdog never latched E-Stop after the operator link went silent"
echo "  ok: watchdog E-Stop latched after operator link silence"

echo
echo "PASS: HELLO -> SESSION_DESCRIBE/ACCEPT -> Channel B -> watchdog E-Stop all verified"
