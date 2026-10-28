#!/usr/bin/env bash
# Topic forwarding (#36) over Wi-Fi against a real robot: round trips
# through the bridge in latest and reliable mode (a zenoh_probe echo on the
# robot sends each sample back, timed on this host only), 300 KB reliable
# maps robot -> operator, an ICMP baseline, and E-Stop latches.
#
# Needs robot-edge and the zenoh_probe example built in release on the robot
# (xgo_bridge/scripts/deploy.sh, then `cargo build --release -p
# roboprotocol-zenoh --example zenoh_probe` there) and here. Runs with
# --stub-bridge: no motors.
#
# usage: benchmark/topic_forwarding_wifi.sh PI_HOST NAME ["EXTRA ROBOT-EDGE ARGS"]
#   e.g. benchmark/topic_forwarding_wifi.sh pi@192.168.2.26 lidar "--sim-sensor lidar"
set -u
cd "$(dirname "${BASH_SOURCE[0]}")/.."

PI=${1:?usage: topic_forwarding_wifi.sh PI_HOST NAME [EXTRA]}; NAME=${2:?NAME}; EXTRA=${3:-}; IP=${PI#*@}
W=benchmark/results/topic-forwarding/$NAME; rm -rf "$W"; mkdir -p "$W"
PROBE=./target/release/examples/zenoh_probe
OZ=tcp/127.0.0.1:7448
CONSOLE=""; SUBP=""

stop_robot() {
    ssh $PI bash -s <<'R'
pkill -INT -x robot-edge
for i in $(seq 1 30); do pgrep -x robot-edge >/dev/null || break; sleep 0.1; done
pkill -KILL -x robot-edge
pkill -KILL -x zenoh_probe
true
R
}
finish() {
    stop_robot
    [ -n "$CONSOLE" ] && kill -9 $CONSOLE 2>/dev/null
    [ -n "$SUBP" ] && kill -9 $SUBP 2>/dev/null
    scp -q $PI:/tmp/fwd-test/*.log $W/ 2>/dev/null
}
trap finish EXIT

echo "== $NAME: starting robot-edge on the CM4 ($EXTRA) =="
ssh $PI bash -s <<R
rm -rf /tmp/fwd-test; mkdir -p /tmp/fwd-test
cd /home/pi/RoboProtocol
nohup ./target/release/robot-edge --stub-bridge --listen 0.0.0.0:4433 --zenoh-local-only \
  --zenoh-forward 'latest:demo/robot/**' --zenoh-forward 'reliable:demo/map/**' \
  --zenoh-accept 'demo/op/**' --zenoh-accept 'demo/fast/**' $EXTRA > /tmp/fwd-test/robot.log 2>&1 < /dev/null &
sleep 2
nohup ./target/release/examples/zenoh_probe echo tcp/127.0.0.1:7447 demo/fast/ping demo/robot/pong > /tmp/fwd-test/echo-latest.log 2>&1 < /dev/null &
nohup ./target/release/examples/zenoh_probe echo tcp/127.0.0.1:7447 demo/op/ping demo/map/pong > /tmp/fwd-test/echo-reliable.log 2>&1 < /dev/null &
R
sleep 1
./target/release/operator-console --connect $IP:4433 --server-name robot-edge --headless \
  --zenoh-listen $OZ --zenoh-forward 'latest:demo/fast/**' --zenoh-forward 'reliable:demo/op/**' \
  --zenoh-accept 'demo/robot/**' --zenoh-accept 'demo/map/**' > $W/console.log 2>&1 < /dev/null &
CONSOLE=$!
for i in $(seq 1 100); do grep -q "SESSION_DESCRIBE received" $W/console.log && break; sleep 0.1; done
grep -q "zenoh topic forwarding negotiated" $W/console.log || { echo "no session/negotiation"; tail -5 $W/console.log; exit 1; }
sleep 3   # let the echo nodes' subscriptions settle

echo "-- ICMP baseline"; ping -c 100 -i 0.2 -q $IP | tail -1
echo "-- bridge round trips (laptop -> robot echo -> laptop)"
$PROBE ping $OZ demo/fast/ping demo/robot/pong 300 32 20 | tail -1
$PROBE ping $OZ demo/op/ping demo/map/pong 300 32 20 | tail -1
$PROBE ping $OZ demo/op/ping demo/map/pong 50 100000 5 | tail -1
echo "-- 300 KB reliable maps robot -> laptop"
$PROBE sub $OZ 'demo/map/grid' > $W/maps.log 2>&1 &
SUBP=$!
sleep 1
ssh $PI 'cd /home/pi/RoboProtocol
./target/release/examples/zenoh_probe pub tcp/127.0.0.1:7447 demo/map/grid 10 300000 2' >/dev/null
sleep 3
echo "maps received: $(grep -c 'bytes=300000' $W/maps.log)/10"
echo "-- control: E-Stop latches while connected (want 0)"
ssh $PI 'grep -c "E-Stop latched" /tmp/fwd-test/robot.log'
