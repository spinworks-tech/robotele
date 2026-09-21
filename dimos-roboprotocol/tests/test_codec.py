import struct

import pytest

from dimos_roboprotocol.codec import decode_command, decode_telemetry, encode_autonomy_goal, joint_names


def test_telemetry_roundtrip():
    b = bytes([87]) + struct.pack(">3h", 150, -250, 9000) + struct.pack(">3h", 100, -100, 4500)
    t = decode_telemetry(b)
    assert (t.battery, t.roll, t.pitch, t.yaw) == (87, 1.5, -2.5, 90.0)
    assert t.joints == [1.0, -1.0, 45.0]


def test_telemetry_bad_length():
    with pytest.raises(ValueError):
        decode_telemetry(b"\x00" * 8)


def test_command_decode():
    b = struct.pack(">B6f2hB", 3, 0.5, 0.0, -0.25, 1.0, 2.0, 3.0, -80, 155, 200)
    c = decode_command(b)
    assert c.source_name == "FullTeleoperation"
    assert (c.vx, c.turn, c.arm_x, c.arm_z, c.claw) == (0.5, -0.25, -80, 155, 200)
    with pytest.raises(ValueError):
        decode_command(b[:-1])


def test_autonomy_goal_encodes_and_clamps():
    assert struct.unpack(">3f", encode_autonomy_goal(5.0, -3.0, 20.0)) == (5.0, -3.0, 20.0)
    assert struct.unpack(">3f", encode_autonomy_goal(99, -99, 99)) == (15.0, -12.0, 60.0)
    assert len(encode_autonomy_goal()) == 12


def test_joint_names():
    n = joint_names(15)
    assert len(n) == 15 and len(set(n)) == 15
    assert n[0] == "front_left_lower" and n[8] == "rear_right_upper" and n[12] == "arm_1"
    assert joint_names(12) == n[:12]
    assert joint_names(4) == ["joint_0", "joint_1", "joint_2", "joint_3"]
