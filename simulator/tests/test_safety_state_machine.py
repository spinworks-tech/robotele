from roboprotocol_sim.core.scheduler import Scheduler
from roboprotocol_sim.protocol.safety_state_machine import (
    RESUME_STABILITY_WINDOW_S,
    TaskClass,
    SafetyStateMachine,
    Watchdog,
    tier_for_latency,
)


def test_tier_boundaries_class_b():
    assert tier_for_latency(TaskClass.B, 10) == 0
    assert tier_for_latency(TaskClass.B, 39.9) == 0
    assert tier_for_latency(TaskClass.B, 40) == 1
    assert tier_for_latency(TaskClass.B, 79.9) == 1
    assert tier_for_latency(TaskClass.B, 80) == 2
    assert tier_for_latency(TaskClass.B, 119.9) == 2
    assert tier_for_latency(TaskClass.B, 120) == 3
    assert tier_for_latency(TaskClass.B, 149.9) == 3
    assert tier_for_latency(TaskClass.B, 150) == 4
    assert tier_for_latency(TaskClass.B, 5000) == 4


def test_tier_boundaries_class_e():
    assert tier_for_latency(TaskClass.E, 199.9) == 0
    assert tier_for_latency(TaskClass.E, 200) == 1
    assert tier_for_latency(TaskClass.E, 999.9) == 3
    assert tier_for_latency(TaskClass.E, 1000) == 4


def test_suspend_on_tier4_and_no_auto_resume_without_deadman():
    sched = Scheduler()
    sm = SafetyStateMachine(TaskClass.B, sched)
    sm.on_rtt_sample(200.0)  # well past Class B's 150ms SUSPENDED threshold
    assert sm.suspended is True
    assert sm.tier == 4

    # Stable, well below nominal, but no deadman reset requested yet.
    sched.now = 5.0
    sm.on_rtt_sample(5.0)
    assert sm.suspended is True


def test_resume_requires_stability_window_and_deadman_reset():
    sched = Scheduler()
    sm = SafetyStateMachine(TaskClass.B, sched)
    sm.on_rtt_sample(200.0)
    assert sm.suspended is True

    sm.request_deadman_reset()
    sched.now = 1.0
    sm.on_rtt_sample(5.0)  # stability window starts now
    assert sm.suspended is True  # not yet 2s

    sched.now = 1.0 + RESUME_STABILITY_WINDOW_S - 0.01
    sm.on_rtt_sample(5.0)
    assert sm.suspended is True  # still short of the window

    sched.now = 1.0 + RESUME_STABILITY_WINDOW_S + 0.01
    sm.on_rtt_sample(5.0)
    assert sm.suspended is False
    assert sm.tier == 0


def test_watchdog_fires_at_class_specific_blackout_threshold():
    sched = Scheduler()
    fired = []
    wd = Watchdog(TaskClass.D, sched, on_estop=lambda t, ms: fired.append((t, ms)))
    sched.run(until=1.0)  # no heartbeats at all -> should fire at 400ms for Class D
    assert wd.triggered is True
    assert len(fired) == 1
    t, blackout_ms = fired[0]
    assert abs(t - 0.4) < 1e-6
    assert abs(blackout_ms - 400.0) < 1e-6


def test_watchdog_heartbeat_prevents_trigger():
    sched = Scheduler()
    fired = []
    wd = Watchdog(TaskClass.B, sched, on_estop=lambda t, ms: fired.append((t, ms)))

    def heartbeat_loop():
        wd.heartbeat()
        if sched.now < 1.0:
            sched.schedule(0.05, heartbeat_loop)

    sched.schedule(0.0, heartbeat_loop)
    sched.run(until=1.0)
    assert wd.triggered is False
    assert fired == []
