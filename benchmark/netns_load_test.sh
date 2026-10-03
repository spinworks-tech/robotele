#!/usr/bin/env bash
# Channel B round trips while sim sensors saturate the robot's uplink, on one
# machine (docs/12-control-under-load-benchmark.md, "First results, on a
# namespace link").
#
# robot-edge and operator-console run in two network namespaces joined by a
# veth pair, each direction shaped with netem. Everything runs inside an
# unprivileged user + network namespace (`unshare -rn`), so no root is needed
# and the host's network, including loopback, is never touched.
#
# Usage:
#   cargo build --release -p robot-edge -p operator-console
#   benchmark/netns_load_test.sh [ROBOT-EDGE ARGS...]
#
# e.g.  benchmark/netns_load_test.sh --sim-sensor lidar --sim-sensor depth --sim-sensor cloud --cc bbr2
#
# Environment: BIN (default target/release), COUNT (pings, default 300),
# RATE (default 20mbit), DELAY (one way, default 10ms), LIMIT (netem queue in
# packets, default 100).
#
# Prints the ping-pong summary, the robot's uplink use over the run, and how
# much lossy data robot-edge dropped before sending it.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${BIN:-$ROOT/target/release}"
COUNT="${COUNT:-300}"
RATE="${RATE:-20mbit}"; DELAY="${DELAY:-10ms}"; LIMIT="${LIMIT:-100}"
for b in robot-edge operator-console; do
    [[ -x "$BIN/$b" ]] || { echo "missing $BIN/$b -- build with: cargo build --release -p robot-edge -p operator-console" >&2; exit 1; }
done
command -v bc >/dev/null || { echo "needs bc" >&2; exit 1; }

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT
(cd "$ROOT" && cargo run --quiet -p dev-certs -- --out-dir "$WORK/certs" >/dev/null)

exec unshare -rn bash -s -- "$BIN" "$WORK" "$COUNT" "$RATE" "$DELAY" "$LIMIT" "$*" <<'INNER'
set -u
BIN="$1"; WORK="$2"; COUNT="$3"; RATE="$4"; DELAY="$5"; LIMIT="$6"; ROBOT_ARGS="$7"
CERTS="$WORK/certs"
ip link set lo up
# The operator's namespace: a child of this one, so we can configure it.
unshare -n sleep 3600 & OP_NS=$!
sleep 0.3
ip link add va type veth peer name vb
ip link set vb netns "$OP_NS"
ip addr add 10.9.0.1/24 dev va && ip link set va up
tc qdisc add dev va root netem delay "$DELAY" rate "$RATE" limit "$LIMIT"
nsenter -t "$OP_NS" -n sh -c "ip link set lo up; ip addr add 10.9.0.2/24 dev vb; ip link set vb up; \
    tc qdisc add dev vb root netem delay $DELAY rate $RATE limit $LIMIT"

# shellcheck disable=SC2086  # ROBOT_ARGS is a word list on purpose
RUST_LOG=info "$BIN/robot-edge" --listen 10.9.0.1:14600 \
    --cert "$CERTS/robot/robot.crt" --key "$CERTS/robot/robot.key" --ca "$CERTS/dev-ca/ca.crt" \
    --stub-bridge --bench echo --robot-id netns-load --zenoh-port 17600 $ROBOT_ARGS > "$WORK/robot.log" 2>&1 &
ROBOT=$!
sleep 1.5

sent_bytes() { tc -s qdisc show dev va | grep -o 'Sent [0-9]*' | cut -d' ' -f2; }
B0=$(sent_bytes); T0=$(date +%s.%N)
nsenter -t "$OP_NS" -n "$BIN/operator-console" --connect 10.9.0.1:14600 --server-name robot-edge \
    --cert "$CERTS/operator/operator.crt" --key "$CERTS/operator/operator.key" --ca "$CERTS/dev-ca/ca.crt" \
    --headless --bench pingpong --bench-count "$COUNT" 2>&1 | grep -aE '^n=|lost'
B1=$(sent_bytes); T1=$(date +%s.%N)
echo "uplink: $(echo "($B1 - $B0) * 8 / ($T1 - $T0) / 1000000" | bc -l | cut -c1-5) Mbit/s over $(echo "$T1 - $T0" | bc | cut -c1-4) s"

sleep 1.5  # let robot-edge log the end of the session
kill "$ROBOT" "$OP_NS" 2>/dev/null
sed 's/\x1b\[[0-9;]*m//g' "$WORK/robot.log" | grep -a 'lossy datagrams' | tail -1 | sed 's/.*lossy/robot: lossy/'
INNER
