"""DimOS module bridging robot-edge's Zenoh sidecar.

Teleoperation stays on RoboProtocol's QUIC path. This module only observes
(telemetry, arbitrated command) and asserts the autonomy-goal flag, which
robot-edge's arbitration ladder ranks below FullTeleoperation.
"""
from __future__ import annotations

import math
import os
import threading
import time

import zenoh

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.Float32 import Float32

from .codec import MAX_TURN, MAX_VX, MAX_VY, decode_telemetry, encode_autonomy_goal, joint_names

# robot-edge honors an autonomy goal for 500 ms (AUTONOMY_GOAL_TTL); refresh well inside it.
_GOAL_REFRESH_S = 0.2


class RoboteleBridgeConfig(ModuleConfig):
    robot_id: str = "xgo_real"
    zenoh_connect: str | None = None  # e.g. "tcp/192.168.2.19:7447"; None = default scouting
    frame_id: str = "base_link"
    imu_frame_id: str = "imu_link"
    # Subscribe to robotele/<id>/video and publish decoded frames. robot-edge
    # only publishes video while someone subscribes, so this is what turns the
    # camera tap (and its CM4 CPU cost) on.
    video: bool = True
    # Twist magnitude that maps to full robot-native deflection (the operator
    # console's own max: vx 15, vy 12, turn 60). UNCALIBRATED placeholders: the
    # XGO's real m/s per xgolib unit has not been measured.
    max_linear_mps: float = 0.25
    max_angular_rps: float = 1.0


class RoboteleBridge(Module):
    config: RoboteleBridgeConfig
    cmd_vel: In[Twist]
    joint_state: Out[JointState]
    battery_percent: Out[Float32]
    imu: Out[Imu]
    color_image: Out[Image]

    _session = None
    _last_goal = 0.0

    @rpc
    def start(self) -> None:
        conf = zenoh.Config()
        connect = self.config.zenoh_connect or os.environ.get("ROBOTELE_ZENOH_CONNECT")
        if connect:
            conf.insert_json5("connect/endpoints", f'["{connect}"]')
        self._session = zenoh.open(conf)
        prefix = f"robotele/{self.config.robot_id}"
        self._telemetry_sub = self._session.declare_subscriber(
            f"{prefix}/telemetry", self._on_telemetry
        )
        self._goal_pub = self._session.declare_publisher(f"{prefix}/autonomy_goal")
        if self.config.video:
            from .video import H264Decoder

            self._decoder = H264Decoder()
            self._decode_lock = threading.Lock()
            self._video_sub = self._session.declare_subscriber(f"{prefix}/video", self._on_video)
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
                name=joint_names(len(t.joints)),
                position=[math.radians(d) for d in t.joints],
            )
        )
        self.battery_percent.publish(Float32(float(t.battery)))
        # Orientation only: telemetry carries roll/pitch/yaw, no gyro or
        # accelerometer. Covariance[0] = -1 is the ROS convention for "this
        # field is not provided", so consumers don't read the zeros as data.
        self.imu.publish(
            Imu(
                orientation=Quaternion.from_euler(
                    Vector3(math.radians(t.roll), math.radians(t.pitch), math.radians(t.yaw))
                ),
                angular_velocity_covariance=[-1.0] + [0.0] * 8,
                linear_acceleration_covariance=[-1.0] + [0.0] * 8,
                frame_id=self.config.imu_frame_id,
            )
        )

    def _on_video(self, sample) -> None:
        with self._decode_lock:
            frames = self._decoder.decode(bytes(sample.payload))
        for rgb in frames:
            self.color_image.publish(Image.from_numpy(rgb, format=ImageFormat.RGB, ts=time.time()))

    def _on_cmd_vel(self, twist: Twist) -> None:
        # Publish the latest velocity at most every _GOAL_REFRESH_S. A stopped
        # planner stops publishing, so robot-edge's 500 ms TTL halts the robot.
        now = time.monotonic()
        if now - self._last_goal < _GOAL_REFRESH_S:
            return
        self._last_goal = now
        lin, ang = self.config.max_linear_mps, self.config.max_angular_rps
        self._goal_pub.put(
            encode_autonomy_goal(
                vx=twist.linear.x / lin * MAX_VX,
                vy=twist.linear.y / lin * MAX_VY,
                turn=twist.angular.z / ang * MAX_TURN,
            )
        )
