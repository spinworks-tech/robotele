#!/usr/bin/env bash
# Sonny OS (mock mode) behind robot-edge's Zenoh topic forwarder, on localhost
# (docs/16). Sonny runs with no config, so it simulates its hardware and never
# opens a serial port. Its telemetry, status and watchdog topics cross the QUIC
# session to operator-console, and its alpha/cmd/** topics are not forwarded.
#
# Sonny is AGPL-3.0 and not vendored here: build it yourself and pass the binary.
#   git clone https://github.com/JackTrainer/Sonny && cd Sonny
#   cargo build --release --bin SONNY   # at 8ba9ad0, drop `pub mod chaos;` from src/lib.rs first
#
# Usage: scripts/sonny_mock_forwarding.sh /path/to/SONNY [DURATION_S]
# Exits non-zero if nothing arrives on the operator side, or if any of Sonny's
# command topics leak across.

set -uo pipefail

SONNY="${1:?usage: $0 /path/to/SONNY [DURATION_S]}"
DURATION="${2:-15}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

WORKDIR="$(mktemp -d)"
PORT="${SONNY_TEST_PORT:-17433}"
ROBOT_ZENOH="tcp/127.0.0.1:$((PORT + 1))"
OPERATOR_ZENOH="tcp/127.0.0.1:$((PORT + 2))"
PROBE=./target/debug/examples/zenoh_probe
PIDS=()

cleanup() {
    for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
    sleep 0.5
    for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
    echo "logs: $WORKDIR"
}
trap cleanup EXIT

fail() {
    echo "FAIL: $1" >&2
    exit 1
}

cargo build --quiet -p robot-edge -p operator-console || fail "build failed"
cargo build --quiet -p roboprotocol-zenoh --example zenoh_probe || fail "building zenoh_probe failed"
cargo run --quiet -p dev-certs -- --out-dir "$WORKDIR/certs" >/dev/null || fail "dev-certs failed"
CERTS="$WORKDIR/certs"

# No --zenoh-local-only: Sonny opens Zenoh with its default config (multicast
# scouting, no way to set endpoints), so it only finds robot-edge by scouting.
setsid ./target/debug/robot-edge \
    --listen "127.0.0.1:$PORT" \
    --cert "$CERTS/robot/robot.crt" --key "$CERTS/robot/robot.key" --ca "$CERTS/dev-ca/ca.crt" \
    --stub-bridge --robot-id sonny_test --zenoh-port $((PORT + 1)) \
    --zenoh-forward 'latest:alpha/telemetry/**' \
    --zenoh-forward 'latest:alpha/status/**' \
    --zenoh-forward 'latest:alpha/watchdog/**' \
    > "$WORKDIR/robot-edge.log" 2>&1 < /dev/null &
PIDS+=($!)

cat > "$WORKDIR/operator-topics" <<'TOPICS'
accept alpha/telemetry/**
accept alpha/status/**
accept alpha/watchdog/**
TOPICS

setsid ./target/debug/operator-console \
    --connect "127.0.0.1:$PORT" --server-name robot-edge \
    --cert "$CERTS/operator/operator.crt" --key "$CERTS/operator/operator.key" --ca "$CERTS/dev-ca/ca.crt" \
    --headless --zenoh-listen "$OPERATOR_ZENOH" --zenoh-topics "$WORKDIR/operator-topics" \
    > "$WORKDIR/operator.log" 2>&1 < /dev/null &
PIDS+=($!)
sleep 2

"$PROBE" sub "$ROBOT_ZENOH" 'alpha/**' > "$WORKDIR/robot-sub.log" 2>&1 &
PIDS+=($!)
"$PROBE" sub "$OPERATOR_ZENOH" 'alpha/**' > "$WORKDIR/operator-sub.log" 2>&1 &
PIDS+=($!)
sleep 1

# Sonny writes its watchdog and jitter logs to stdout; keep them for docs/16.
setsid env -C "$WORKDIR" "$SONNY" > "$WORKDIR/sonny.log" 2>&1 < /dev/null &
PIDS+=($!)
sleep "$DURATION"

for side in robot operator; do
    echo "== samples on the $side's Zenoh =="
    grep -o "recv [^ ]*" "$WORKDIR/$side-sub.log" | sort | uniq -c
done
echo "== Sonny's own watchdog =="
echo "trips: $(grep -c "HARD-WATCHDOG\] TRIP" "$WORKDIR/sonny.log")"
echo "jitter warnings: $(grep -c "Frame out of window" "$WORKDIR/sonny.log")"

grep -q "recv alpha/telemetry/" "$WORKDIR/operator-sub.log" || fail "no Sonny telemetry reached the operator's zenoh"
grep -q "recv alpha/cmd/" "$WORKDIR/operator-sub.log" && fail "Sonny command topics crossed the link"
echo "PASS: Sonny telemetry forwarded to the operator, commands kept on the robot"
