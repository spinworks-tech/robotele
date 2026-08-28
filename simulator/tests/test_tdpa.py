from roboprotocol_sim.network.failures import FailureTimeline
from roboprotocol_sim.network.profiles import NetworkProfile
from roboprotocol_sim.protocol.tdpa import PassivityObserver


def _profile(owd_ms: float) -> NetworkProfile:
    return NetworkProfile(
        name="test",
        base_owd_ms=owd_ms,
        jitter_std_ms=0.0,
        baseline_loss=0.0,
        robot_upload_mbps=100.0,
        operator_upload_mbps=100.0,
    )


def test_zero_delay_is_always_passive():
    timeline = FailureTimeline(_profile(0.0))
    obs = PassivityObserver(timeline, damping_gain=5.0, velocity_amplitude=0.3, velocity_freq_hz=1.5)
    t = 0.0
    dt = 0.001
    for _ in range(3000):
        obs.sample(t)
        t += dt
    assert obs.violations == []
    assert obs._energy <= 1e-9


def test_large_delay_produces_passivity_violations():
    timeline = FailureTimeline(_profile(200.0))  # 200ms delay vs a 1.5Hz (~667ms period) signal
    obs = PassivityObserver(timeline, damping_gain=5.0, velocity_amplitude=0.3, velocity_freq_hz=1.5)
    t = 0.0
    dt = 0.001
    for _ in range(3000):
        obs.sample(t)
        t += dt
    assert len(obs.violations) > 0


def test_sub_floor_quantization_worsens_violations():
    timeline = FailureTimeline(_profile(150.0))
    obs_ok = PassivityObserver(timeline, damping_gain=5.0, velocity_amplitude=0.3, velocity_freq_hz=1.5)
    obs_bad = PassivityObserver(timeline, damping_gain=5.0, velocity_amplitude=0.3, velocity_freq_hz=1.5)
    t = 0.0
    dt = 0.001
    for _ in range(3000):
        obs_ok.sample(t, haptic_tier_ok=True)
        obs_bad.sample(t, haptic_tier_ok=False)
        t += dt
    assert len(obs_bad.violations) >= len(obs_ok.violations)
