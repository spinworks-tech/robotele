import pytest

from roboprotocol_sim.protocol import sizing


def test_single_arm_command_fits_in_one_datagram():
    # 8 DoF * 3 fields (pos/vel/torque) = 24 fields, Full tier.
    plans = sizing.plan_datagrams(24, sizing.QuantizationTier.FULL, carries_haptic=False)
    assert len(plans) == 1
    assert plans[0].app_bytes <= sizing.SINGLE_DATAGRAM_BUDGET_BYTES


def test_dual_arm_haptic_fits_in_one_datagram_at_standard_tier():
    # 2 arms * 8 haptic fields = 16 fields, Standard tier (the haptic floor).
    plans = sizing.plan_datagrams(16, sizing.QuantizationTier.STANDARD, carries_haptic=True)
    assert len(plans) == 1
    assert plans[0].app_bytes <= sizing.SINGLE_DATAGRAM_BUDGET_BYTES


def test_high_dof_stress_triggers_body_region_split():
    # A synthetic full-body-suit-scale field count at Full tier should overflow
    # the 1200B budget and be split into multiple datagrams.
    num_fields = 400
    plans = sizing.plan_datagrams(num_fields, sizing.QuantizationTier.FULL, carries_haptic=False)
    assert len(plans) > 1
    assert sum(p.num_fields for p in plans) == num_fields
    for p in plans:
        assert p.app_bytes <= sizing.SINGLE_DATAGRAM_BUDGET_BYTES


def test_haptic_compact_tier_is_rejected():
    with pytest.raises(sizing.HapticTierViolation):
        sizing.validate_haptic_tier(sizing.QuantizationTier.COMPACT)
    with pytest.raises(sizing.HapticTierViolation):
        sizing.plan_datagrams(8, sizing.QuantizationTier.COMPACT, carries_haptic=True)


def test_haptic_standard_and_full_tiers_are_allowed():
    sizing.validate_haptic_tier(sizing.QuantizationTier.STANDARD)
    sizing.validate_haptic_tier(sizing.QuantizationTier.FULL)


def test_wire_bytes_adds_fixed_overhead():
    app = 500
    assert sizing.wire_bytes(app) == app + sizing.WIRE_OVERHEAD_BYTES
