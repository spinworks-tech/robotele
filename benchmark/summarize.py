#!/usr/bin/env python3
"""Print the max-sustained-rate table from one or more run dirs' results.json."""
import json
import sys
ORDER = ["mqtt", "zenoh", "udp", "mqtt-tls", "zenoh-tls", "webrtc",
         "rs-udp", "rs-mqtt", "rs-zenoh", "rs-mqtt-tls", "rs-zenoh-tls", "rs-webrtc",
         "channel-b-raw", "channel-b"]
rows = {}
for d in sys.argv[1:]:
    for k, v in json.load(open(f"{d}/results.json"))["throughput"].items():
        rows[k] = v
protos = sorted({k.split("/")[0] for k in rows}, key=ORDER.index)
sizes = sorted({int(k.split("/")[1]) for k in rows})
print("| payload | " + " | ".join(protos) + " |")
print("|---|" + "---|" * len(protos))
for sz in sizes:
    cells = []
    for p in protos:
        b = rows.get(f"{p}/{sz}", {}).get("best")
        if not b:
            cells.append("< 1k msg/s")
            continue
        hz = b["recv_hz"]
        rate = f"{hz/1000:.0f}k" if hz >= 1000 else f"{hz:.0f}"
        mbps = b["mbps"]
        bw = f"{mbps/1000:.1f} Gbps" if mbps >= 1000 else f"{mbps:.0f} Mbps"
        cells.append(f"{rate} msg/s, {bw}" + (" †" if b.get("sender_limited") else ""))
    label = f"{sz//1024} KB" if sz >= 1024 else f"{sz} B"
    print(f"| {label} | " + " | ".join(cells) + " |")
