"""The four progressive teleoperation scenarios requested for this
simulator, built on a shared arm-DoF assumption documented here (and in
simulator/SPEC.md) since REQUIREMENTS.md does not pin an exact DoF
count for "an arm":

  * One arm = 7-DoF anthropomorphic arm + 1-DoF parallel gripper = 8
    actuated DoF.
  * Channel B "command" fields = DoF x 3 (position, velocity, torque).
  * Channel B "telemetry" fields = one 6-axis body/wrist IMU (3 accel +
    3 gyro) + DoF x 3 (position, velocity, torque) + battery
    voltage/temperature (2 fields). No foot force-torque sensors: these
    are seated/stationary arm-manipulation scenarios, not legged
    locomotion, so Task Class B (Fine Tool & Utensil Manipulation) is
    the default active class for all four (configurable).
  * Channel B "haptic" fields = wrist wrench (Fx,Fy,Fz,Tx,Ty,Tz = 6) +
    2 simplified per-gripper contact-force scalars = 8 fields per arm.

Scenarios are a strict progression, each adding one axis of complexity
on top of the previous (per the user's "then X, then Y, then Z, then
W" framing): scenario 4 (dual arm) therefore also carries both wrist
cameras plus the fixed head camera introduced in scenarios 2-3, not
just the second arm in isolation. This is a documented assumption --
flip DUAL_ARM_INCLUDES_CAMERAS below (or pass a custom ScenarioConfig)
if a control-only dual-arm comparison point is wanted instead.
"""
from __future__ import annotations

from dataclasses import dataclass, field

from ..protocol.channel_a import VideoStreamConfig
from ..protocol.safety_state_machine import TaskClass
from ..protocol.sizing import QuantizationTier

DUAL_ARM_INCLUDES_CAMERAS = True

ARM_DOF = 8  # 7-DoF arm + 1-DoF gripper
BODY_IMU_FIELDS = 6
BATTERY_THERMAL_FIELDS = 2
HAPTIC_FIELDS_PER_ARM = 8  # 6 wrench + 2 gripper contact-force scalars

WRIST_CAM = VideoStreamConfig(name="wrist_cam", fps=30, initial_bps=2_500_000, min_bps=2_000_000, max_bps=4_000_000)
HEAD_STEREO_CAM = VideoStreamConfig(name="head_stereo_cam", fps=30, initial_bps=8_000_000, min_bps=3_000_000, max_bps=16_000_000)


def _wrist_cam(name: str = "wrist_cam") -> VideoStreamConfig:
    return VideoStreamConfig(
        name=name, fps=WRIST_CAM.fps, initial_bps=WRIST_CAM.initial_bps,
        min_bps=WRIST_CAM.min_bps, max_bps=WRIST_CAM.max_bps,
    )


def _head_cam() -> VideoStreamConfig:
    return VideoStreamConfig(
        name=HEAD_STEREO_CAM.name, fps=HEAD_STEREO_CAM.fps, initial_bps=HEAD_STEREO_CAM.initial_bps,
        min_bps=HEAD_STEREO_CAM.min_bps, max_bps=HEAD_STEREO_CAM.max_bps,
    )


@dataclass
class ScenarioConfig:
    key: str
    label: str
    task_class: TaskClass = TaskClass.B
    control_hz: float = 250.0
    n_arms: int = 1
    command_tier: QuantizationTier = QuantizationTier.FULL
    telemetry_tier: QuantizationTier = QuantizationTier.STANDARD
    haptic_tier: QuantizationTier = QuantizationTier.STANDARD
    video_streams: list[VideoStreamConfig] = field(default_factory=list)

    @property
    def command_fields(self) -> int:
        return self.n_arms * ARM_DOF * 3

    @property
    def telemetry_fields(self) -> int:
        return BODY_IMU_FIELDS + self.n_arms * ARM_DOF * 3 + BATTERY_THERMAL_FIELDS

    @property
    def haptic_fields(self) -> int:
        return self.n_arms * HAPTIC_FIELDS_PER_ARM


SCENARIOS: dict[str, ScenarioConfig] = {
    "simple_arm": ScenarioConfig(
        key="simple_arm", label="1. Simple single-arm control (no video)",
        n_arms=1, video_streams=[],
    ),
    "arm_wrist_cam": ScenarioConfig(
        key="arm_wrist_cam", label="2. Single arm + wrist camera",
        n_arms=1, video_streams=[_wrist_cam()],
    ),
    "arm_wrist_fixed_cam": ScenarioConfig(
        key="arm_wrist_fixed_cam", label="3. Single arm + wrist camera + fixed head camera",
        n_arms=1, video_streams=[_wrist_cam(), _head_cam()],
    ),
    "dual_arm": ScenarioConfig(
        key="dual_arm", label="4. Dual-arm control" + (" + both cameras" if DUAL_ARM_INCLUDES_CAMERAS else ""),
        n_arms=2,
        video_streams=[_wrist_cam("wrist_cam_left"), _wrist_cam("wrist_cam_right"), _head_cam()]
        if DUAL_ARM_INCLUDES_CAMERAS else [],
    ),
}


def get_scenario(key: str) -> ScenarioConfig:
    try:
        return SCENARIOS[key]
    except KeyError as exc:
        raise ValueError(f"unknown scenario {key!r}; choices: {sorted(SCENARIOS)}") from exc
