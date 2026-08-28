import pytest

from roboprotocol_sim.scenarios.definitions import SCENARIOS
from roboprotocol_sim.scenarios.failure_presets import FAILURE_PRESETS
from roboprotocol_sim.scenarios.runner import run_simulation


@pytest.mark.parametrize("scenario_key", sorted(SCENARIOS))
def test_each_scenario_runs_end_to_end(scenario_key):
    run = run_simulation(scenario_key, "none", duration=2.0, seed=1)
    dfs = run.collector.to_dataframes()
    assert not dfs["datagrams"].empty
    assert not dfs["rtt"].empty
    if run.scenario.video_streams:
        assert not dfs["video_bitrate"].empty
        assert not dfs["fec"].empty


@pytest.mark.parametrize("failure_key", sorted(FAILURE_PRESETS))
def test_each_failure_preset_runs_on_dual_arm(failure_key):
    run = run_simulation("dual_arm", failure_key, duration=6.0, seed=2)
    dfs = run.collector.to_dataframes()
    assert not dfs["datagrams"].empty


def test_blackout_triggers_watchdog():
    run = run_simulation("simple_arm", "blackout", duration=10.0, seed=3)
    assert len(run.watchdog.trigger_events) >= 1
    threshold_ms = 200.0  # Class B default
    triggered = run.watchdog.trigger_events[0]
    assert abs(triggered.blackout_ms - threshold_ms) < 5.0


def test_haptic_floor_is_enforced_by_default_scenarios():
    from roboprotocol_sim.protocol.sizing import HAPTIC_MIN_TIER

    for cfg in SCENARIOS.values():
        assert cfg.haptic_tier.value >= HAPTIC_MIN_TIER.value
