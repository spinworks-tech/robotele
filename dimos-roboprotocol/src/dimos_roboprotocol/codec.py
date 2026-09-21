"""Wire codecs for the robot-edge Zenoh sidecar (see BABYROS.md).

Pure Python, no DimOS dependency, so it can be tested and reused standalone.
All fields are big-endian.
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

CONTROL_SOURCES = {
    0: "EStop",
    1: "EmergencySafeParking",
    2: "ActiveImpedanceHold",
    3: "FullTeleoperation",
    4: "SemiAutonomous",
}

# Joint order on the wire (crates/robot-edge/src/xgo_profile.rs): four legs
# (front-left, front-right, rear-right, rear-left), each lower/middle/upper,
# then the optional arm accessory. The robot sends no names, these are ours.
# The arm's per-joint meaning is undocumented by the vendor SDK.
_LEGS = ("front_left", "front_right", "rear_right", "rear_left")
XGO_JOINT_NAMES = [f"{leg}_{pos}" for leg in _LEGS for pos in ("lower", "middle", "upper")] + [
    "arm_1",
    "arm_2",
    "arm_3",
]


def joint_names(count: int) -> list[str]:
    """Names for `count` telemetry joints; generic names if the layout is unknown."""
    if count == len(XGO_JOINT_NAMES):
        return list(XGO_JOINT_NAMES)
    if count == 12:  # base kit without the arm
        return XGO_JOINT_NAMES[:12]
    return [f"joint_{i}" for i in range(count)]


_TELEMETRY_HEADER = 7  # battery u8 + roll/pitch/yaw i16
_COMMAND_LEN = 30


@dataclass(frozen=True)
class Telemetry:
    battery: int  # percent
    roll: float  # degrees
    pitch: float
    yaw: float
    joints: list[float]  # degrees


@dataclass(frozen=True)
class Command:
    source: int  # key of CONTROL_SOURCES
    vx: float
    vy: float
    turn: float
    roll: float
    pitch: float
    yaw: float
    arm_x: int
    arm_z: int
    claw: int

    @property
    def source_name(self) -> str:
        return CONTROL_SOURCES.get(self.source, f"Unknown({self.source})")


def decode_telemetry(b: bytes) -> Telemetry:
    if len(b) < _TELEMETRY_HEADER or (len(b) - _TELEMETRY_HEADER) % 2:
        raise ValueError(f"bad telemetry length {len(b)}")
    roll, pitch, yaw = (v / 100 for v in struct.unpack(">3h", b[1:7]))
    n = (len(b) - _TELEMETRY_HEADER) // 2
    joints = [v / 100 for v in struct.unpack(f">{n}h", b[7:])]
    return Telemetry(b[0], roll, pitch, yaw, joints)


def decode_command(b: bytes) -> Command:
    if len(b) != _COMMAND_LEN:
        raise ValueError(f"bad command length {len(b)}, expected {_COMMAND_LEN}")
    src, vx, vy, turn, r, p, y, ax, az, claw = struct.unpack(">B6f2hB", b)
    return Command(src, vx, vy, turn, r, p, y, ax, az, claw)


# Robot-native per-axis limits; robot-edge clamps to the same values
# (zenoh_bridge.rs AUTONOMY_MAX_*), matching what the operator console can send.
MAX_VX = 15.0
MAX_VY = 12.0
MAX_TURN = 60.0


def _clamp(v: float, limit: float) -> float:
    return max(-limit, min(limit, v))


def encode_autonomy_goal(vx: float = 0.0, vy: float = 0.0, turn: float = 0.0) -> bytes:
    """`vx, vy, turn` as big-endian f32 in robot-native (xgolib) units.

    Any fresh sample asserts the goal for 500 ms; the velocity is honored only
    while it is fresh, so a stalled sender stops the robot. Values are clamped
    here too, but robot-edge is the authority.
    """
    return struct.pack(">3f", _clamp(vx, MAX_VX), _clamp(vy, MAX_VY), _clamp(turn, MAX_TURN))
