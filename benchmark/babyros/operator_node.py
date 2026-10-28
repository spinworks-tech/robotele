"""Operator-side BabyROS node for the topic forwarding test (#36); see
robot_node.py for the topics. Round trips are timed on this host only.

usage: operator_node.py ZENOH_ENDPOINT SECONDS
"""
import json
import sys
import threading
import time

import numpy as np

import babyros
from babyros.node import Publisher, Subscriber
from telekinesis import datatypes

endpoint, seconds = sys.argv[1], float(sys.argv[2])
babyros.configure(connect_endpoints=[endpoint], scouting_multicast_enabled=False)
# configure()'s timestamping_enabled_* map to `timestamping/enabled/<mode>`,
# which Zenoh 1.10 rejects ("unknown key"), so set it on the stored config.
babyros.node.SessionManager.get_session_config().insert_json5("timestamping/enabled", "true")

lock = threading.Lock()
got = {"pose": 0, "pose_last_seq": -1, "vel": 0, "vel_ok": 0, "grid": 0, "grid_ok": 0, "cloud": 0, "cloud_ok": 0}
sent = {"latest": {}, "reliable": {}}
rtt = {"latest": [], "reliable": []}


def on_pose(d):
    with lock:
        got["pose"] += 1
        got["pose_last_seq"] = max(got["pose_last_seq"], d["seq"])


def on_vel(d):
    with lock:
        got["vel"] += 1
        got["vel_ok"] += isinstance(d, datatypes.Vector3D)


def on_grid(d):
    with lock:
        got["grid"] += 1
        got["grid_ok"] += isinstance(d, np.ndarray) and d.shape == (500, 600) and d.dtype == np.uint8 and len(np.unique(d)) == 1


def on_cloud(d):
    with lock:
        got["cloud"] += 1
        got["cloud_ok"] += isinstance(d, datatypes.PointCloud)


def on_pong(kind):
    def callback(d):
        t = sent[kind].pop(d["seq"], None)
        if t is not None:
            with lock:
                rtt[kind].append((time.monotonic() - t) * 1e3)
    return callback


subs = [
    Subscriber("demo/robot/pose", on_pose),
    Subscriber("demo/robot/vel", on_vel, history="keep_all", depth=256),
    Subscriber("demo/map/grid", on_grid, history="keep_all", depth=16),
    Subscriber("demo/map/cloud", on_cloud, history="keep_all", depth=16),
    Subscriber("demo/robot/pong", on_pong("latest"), history="keep_all", depth=256),
    Subscriber("demo/map/pong", on_pong("reliable"), history="keep_all", depth=256),
]
ping = {"latest": Publisher("demo/fast/ping", compression=None), "reliable": Publisher("demo/op/ping", compression=None)}
goal = Publisher("demo/op/goal")
time.sleep(1.0)

start = time.monotonic()
i = 0
while time.monotonic() - start < seconds:
    for kind, pub in ping.items():
        sent[kind][i] = time.monotonic()
        pub.publish({"seq": i, "pad": "x" * 32})
    if i % 4 == 0:
        goal.publish({"seq": i // 4, "path": np.full((50, 2), float(i // 4))})
    i += 1
    time.sleep(0.05)
time.sleep(3.0)


def pct(xs, q):
    xs = sorted(xs)
    return round(xs[min(int(len(xs) * q), len(xs) - 1)], 1) if xs else None


with lock:
    out = dict(got, pings_sent=i, goals_sent=(i + 3) // 4)
    for kind in rtt:
        out[f"rtt_{kind}"] = {"back": len(rtt[kind]), "p50": pct(rtt[kind], 0.5), "p90": pct(rtt[kind], 0.9), "max": pct(rtt[kind], 1.0)}
print("OPERATOR " + json.dumps(out), flush=True)
