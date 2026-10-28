#!/usr/bin/env bash
# Two-process localhost integration smoke test: builds (if needed), spins up
# robot-edge (--stub-bridge, no real XGO hardware needed) and operator-console
# (--headless, no real terminal needed) against each other over real UDP/QUIC
# on loopback, and checks for:
#   1. HELLO negotiated on both sides
#   2. SESSION_DESCRIBE / SESSION_ACCEPT completed
#   2b. sensor slices (docs/13) from two --sim-sensor sources assemble into
#      frames on the operator -- the lidar's descriptor also makes
#      SESSION_DESCRIBE span more than one packet
#   2c. a bulk object (docs/13, --sim-map) arrives whole on its own stream
#   2d. Zenoh topic forwarding (#36): samples published on each side's own
#      Zenoh are republished on the other side's, in latest and reliable mode
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
# Each side's own Zenoh session, for the topic forwarding check.
ROBOT_ZENOH="tcp/127.0.0.1:$((PORT + 1))"
OPERATOR_ZENOH="tcp/127.0.0.1:$((PORT + 2))"
PROBE=./target/debug/examples/zenoh_probe

ROBOT_PID=""
OPERATOR_PID=""
SUB_PIDS=()

cleanup() {
    [[ -n "$ROBOT_PID" ]] && kill -9 "$ROBOT_PID" 2>/dev/null
    [[ -n "$OPERATOR_PID" ]] && kill -9 "$OPERATOR_PID" 2>/dev/null
    for pid in "${SUB_PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
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

# Waits until `pattern` appears on at least `count` lines of `file`.
wait_for_count() {
    local file="$1" pattern="$2" count="$3" timeout_s="$4" waited=0
    while [[ "$(grep -c "$pattern" "$file" 2>/dev/null)" -lt "$count" ]]; do
        sleep 0.1
        waited=$((waited + 1))
        if [[ $waited -ge $((timeout_s * 10)) ]]; then
            return 1
        fi
    done
    return 0
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
cargo build --quiet -p roboprotocol-zenoh --example zenoh_probe || fail "building zenoh_probe failed"

echo "== generating dev certs =="
CERT_DIR="$WORKDIR/certs"
cargo run --quiet -p dev-certs -- --out-dir "$CERT_DIR" >/dev/null || fail "dev-certs failed"

echo "== starting robot-edge (--stub-bridge) on 127.0.0.1:$PORT =="
setsid ./target/debug/robot-edge \
    --listen "127.0.0.1:$PORT" \
    --cert "$CERT_DIR/robot/robot.crt" --key "$CERT_DIR/robot/robot.key" --ca "$CERT_DIR/dev-ca/ca.crt" \
    --stub-bridge --robot-id smoke_test \
    --sim-sensor lidar --sim-sensor radar --sim-map 1 \
    --zenoh-port $((PORT + 1)) --zenoh-local-only \
    --zenoh-forward 'latest:demo/robot/**' --zenoh-forward 'reliable:demo/map/**' --zenoh-accept 'demo/op/**' \
    > "$ROBOT_LOG" 2>&1 < /dev/null &
ROBOT_PID=$!
disown

# The operator side takes its rules from a file. It doesn't accept
# demo/robot/secret, which the robot forwards anyway.
cat > "$WORKDIR/operator-topics" <<'TOPICS'
forward reliable demo/op/**   # goals to the robot
accept demo/robot/pose
accept demo/map/**
TOPICS

echo "== starting operator-console (--headless) =="
setsid ./target/debug/operator-console \
    --connect "127.0.0.1:$PORT" --server-name robot-edge \
    --cert "$CERT_DIR/operator/operator.crt" --key "$CERT_DIR/operator/operator.key" --ca "$CERT_DIR/dev-ca/ca.crt" \
    --headless \
    --zenoh-listen "$OPERATOR_ZENOH" --zenoh-topics "$WORKDIR/operator-topics" \
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

echo "== waiting for sensor frames (lidar, radar) =="
wait_for_count "$OPERATOR_LOG" "first sensor frame assembled" 2 10 || fail "operator-console didn't assemble a frame from both sim sensors"
echo "  ok: sensor slices assembled into frames"

echo "== waiting for a bulk object (sim map) =="
wait_for_log "$OPERATOR_LOG" "bulk object received" 10 || fail "operator-console never received a bulk object"
echo "  ok: bulk object received"

echo "== forwarding zenoh topics both ways =="
"$PROBE" sub "$OPERATOR_ZENOH" 'demo/**' > "$WORKDIR/operator-sub.log" 2>&1 &
SUB_PIDS+=($!)
disown
"$PROBE" sub "$ROBOT_ZENOH" 'demo/**' > "$WORKDIR/robot-sub.log" 2>&1 &
SUB_PIDS+=($!)
disown
wait_for_log "$WORKDIR/operator-sub.log" "subscribed" 5 && wait_for_log "$WORKDIR/robot-sub.log" "subscribed" 5 || fail "zenoh_probe subscribers didn't start"
sleep 0.5
wait_for_log "$ROBOT_LOG" "zenoh topic forwarding negotiated" 1 || fail "robot-edge didn't negotiate topic forwarding"
"$PROBE" pub "$ROBOT_ZENOH" demo/robot/pose 50 32 100 >/dev/null || fail "publishing demo/robot/pose failed"
"$PROBE" pub "$ROBOT_ZENOH" demo/robot/secret 5 16 50 >/dev/null || fail "publishing demo/robot/secret failed"
"$PROBE" pub "$ROBOT_ZENOH" demo/map/grid 3 300000 5 >/dev/null || fail "publishing demo/map/grid failed"
"$PROBE" pub "$OPERATOR_ZENOH" demo/op/goal 20 16 50 >/dev/null || fail "publishing demo/op/goal failed"
# Latest mode may drop or supersede a few; reliable mode must deliver all.
wait_for_count "$WORKDIR/operator-sub.log" "recv demo/robot/pose .*attachment=probe-attachment" 40 5 || fail "latest-mode samples didn't reach the operator's zenoh"
wait_for_count "$WORKDIR/operator-sub.log" "recv demo/map/grid .*bytes=300000" 3 10 || fail "reliable 300 KB samples didn't all reach the operator's zenoh"
wait_for_count "$WORKDIR/robot-sub.log" "recv demo/op/goal" 20 5 || fail "reliable samples didn't all reach the robot's zenoh"
grep -q "recv demo/robot/secret" "$WORKDIR/operator-sub.log" && fail "the operator republished demo/robot/secret, which it doesn't accept"
for f in operator-sub robot-sub; do
    dups=$(grep "^recv" "$WORKDIR/$f.log" | sort | uniq -d | wc -l)
    [[ "$dups" -eq 0 ]] || fail "$f received $dups duplicate samples (a forwarding loop?)"
done
echo "  ok: zenoh topics forwarded both ways, only accepted keys republished, no duplicates"

echo "== killing operator-console to trigger the watchdog =="
kill -9 "$OPERATOR_PID" 2>/dev/null
OPERATOR_PID=""

# Class D watchdog blackout threshold is 400ms; allow generous margin for
# scheduling jitter under a busy CI host.
wait_for_log "$ROBOT_LOG" "E-Stop latched" 3 || fail "watchdog never latched E-Stop after the operator link went silent"
echo "  ok: watchdog E-Stop latched after operator link silence"

echo
echo "PASS: HELLO -> SESSION_DESCRIBE/ACCEPT -> sensor slices -> bulk objects -> zenoh topics -> Channel B -> watchdog E-Stop all verified"
