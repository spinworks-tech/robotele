"""DimOS module bridging robot-edge's Zenoh sidecar.

Teleoperation stays on RoboProtocol's QUIC path. This module only observes
(telemetry, arbitrated command) and asserts the autonomy-goal flag, which
robot-edge's arbitration ladder ranks below FullTeleoperation.
"""
from __future__ import annotations

import math
import time

import zenoh

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.sensor_msgs.JointState import JointState

from .codec import decode_telemetry, encode_autonomy_goal

# robot-edge honors an autonomy goal for 500 ms (AUTONOMY_GOAL_TTL); refresh well inside it.
_GOAL_REFRESH_S = 0.2


class RoboteleBridgeConfig(ModuleConfig):
    robot_id: str = "xgo_real"
    zenoh_connect: str | None = None  # e.g. "tcp/192.168.2.19:7447"; None = default scouting
    frame_id: str = "base_link"


class RoboteleBridge(Module):
    config: RoboteleBridgeConfig
    cmd_vel: In[Twist]
    joint_state: Out[JointState]

    _session = None
    _last_goal = 0.0

    @rpc
    def start(self) -> None:
        conf = zenoh.Config()
        if self.config.zenoh_connect:
            conf.insert_json5("connect/endpoints", f'["{self.config.zenoh_connect}"]')
        self._session = zenoh.open(conf)
        prefix = f"robotele/{self.config.robot_id}"
        self._telemetry_sub = self._session.declare_subscriber(
            f"{prefix}/telemetry", self._on_telemetry
        )
        self._goal_pub = self._session.declare_publisher(f"{prefix}/autonomy_goal")
        self.register_disposable(self.cmd_vel.observable().subscribe(self._on_cmd_vel))

    @rpc
    def stop(self) -> None:
        if self._session is not None:
            self._session.close()
            self._session = None

    def _on_telemetry(self, sample) -> None:
        t = decode_telemetry(bytes(sample.payload))
        self.joint_state.publish(
            JointState(
                ts=time.time(),
                frame_id=self.config.frame_id,
                name=[f"joint_{i}" for i in range(len(t.joints))],
                position=[math.radians(d) for d in t.joints],
            )
        )

    def _on_cmd_vel(self, _twist: Twist) -> None:
        # NOTE: the velocity itself is not transmitted yet -- robot-edge only
        # sees "an autonomy source wants control". See docs/10-dimos-integration-spike.md.
        now = time.monotonic()
        if now - self._last_goal >= _GOAL_REFRESH_S:
            self._last_goal = now
            self._goal_pub.put(encode_autonomy_goal())
