#!/usr/bin/env bash
# Drops and restores a network interface at a logged wall-clock time, so the
# outage window can be lined up against a concurrent tshark capture and the
# robot-edge/operator-console log lines for E-Stop latch/clear. See
# BENCHMARK.md part (c). Requires sudo.
#
# Usage: wifi_drop_helper.sh <iface> <outage_seconds>
set -euo pipefail

IFACE="${1:?usage: wifi_drop_helper.sh <iface> <outage_seconds>}"
OUTAGE_S="${2:?usage: wifi_drop_helper.sh <iface> <outage_seconds>}"

ts() { date -u +"%Y-%m-%dT%H:%M:%S.%3NZ"; }

echo "$(ts) dropping $IFACE"
sudo ip link set "$IFACE" down
sleep "$OUTAGE_S"
echo "$(ts) restoring $IFACE"
sudo ip link set "$IFACE" up

# Some wifi drivers/NetworkManager setups don't fully de-associate on
# `ip link down` -- if the capture shows no gap, try instead:
#   sudo nmcli radio wifi off  /  sudo nmcli radio wifi on
# or, for a hard radio kill (needs a `rfkill list` id first):
#   sudo rfkill block wifi     /  sudo rfkill unblock wifi
