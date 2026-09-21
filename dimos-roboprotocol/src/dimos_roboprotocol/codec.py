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


def encode_autonomy_goal() -> bytes:
    """robot-edge ignores the payload: any fresh sample asserts the goal for 500 ms."""
    return b"\x01"
