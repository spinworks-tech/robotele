"""Robot-side BabyROS node for the topic forwarding test (#36).

Publishes on the robot's own Zenoh and echoes the operator's pings, all
through BabyROS's Publisher/Subscriber and codec:
  demo/robot/pose   dict (JSON), 20 Hz            -> latest
  demo/robot/vel    datatypes.Vector3D, 20 Hz     -> latest
  demo/map/grid     uint8 ndarray 500x600 (NDAR), every 2 s -> reliable
  demo/map/cloud    datatypes.PointCloud, 2,000 pts, every 2 s -> reliable
  demo/fast/ping -> demo/robot/pong (latest), demo/op/ping -> demo/map/pong (reliable)
  demo/op/goal      dict with an ndarray (NDDC), counted and checked

usage: robot_node.py ZENOH_ENDPOINT SECONDS
"""
import json
import sys
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

stats = {"goal": 0, "goal_ok": 0, "echo_latest": 0, "echo_reliable": 0}
pong_latest = Publisher("demo/robot/pong", compression=None)
pong_reliable = Publisher("demo/map/pong", compression=None)


def echo(publisher, key):
    def callback(data):
        stats[key] += 1
        publisher.publish(data)
    return callback


def on_goal(data):
    stats["goal"] += 1
    path = data.get("path") if isinstance(data, dict) else None
    if isinstance(path, np.ndarray) and path.shape == (50, 2) and float(path[0, 0]) == float(data["seq"]):
        stats["goal_ok"] += 1


subs = [
    Subscriber("demo/fast/ping", echo(pong_latest, "echo_latest")),
    Subscriber("demo/op/ping", echo(pong_reliable, "echo_reliable"), history="keep_all", depth=256),
    Subscriber("demo/op/goal", on_goal, history="keep_all", depth=256),
]
pose = Publisher("demo/robot/pose", compression=None)
vel = Publisher("demo/robot/vel")
grid = Publisher("demo/map/grid", compression=None)
cloud = Publisher("demo/map/cloud")

rng = np.random.default_rng(0)
start = time.monotonic()
i = maps = 0
while time.monotonic() - start < seconds:
    pose.publish({"seq": i, "x": i * 0.01, "y": 0.0, "yaw": 0.1})
    vel.publish(datatypes.Vector3D([float(i), 0.0, 0.0]))
    if i % 40 == 0:
        g = np.full((500, 600), maps % 256, dtype=np.uint8)
        grid.publish(g)
        cloud.publish(datatypes.PointCloud(positions=rng.random((2000, 3), dtype=np.float32)))
        maps += 1
    i += 1
    time.sleep(0.05)
stats.update(pose_sent=i, maps_sent=maps)
print("ROBOT " + json.dumps(stats), flush=True)
