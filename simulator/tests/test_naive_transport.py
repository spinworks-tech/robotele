
from roboprotocol_sim.core.scheduler import Scheduler
from roboprotocol_sim.network.failures import FailureTimeline
from roboprotocol_sim.network.naive_link import NaiveLink, NaiveSegment
from roboprotocol_sim.network.profiles import HOME_BROADBAND_WIFI6
from roboprotocol_sim.protocol.naive_session import LIVENESS_CHECK_INTERVAL_S, LIVENESS_TIMEOUT_S, NaiveWebSocketSession
from roboprotocol_sim.scenarios.definitions import SCENARIOS
from roboprotocol_sim.scenarios.runner import run_naive_simulation, run_simulation


def test_head_of_line_blocking_delays_segments_queued_behind_a_loss():
    """A segment lost during a blackout must not just be individually
    delayed: everything enqueued behind it on the same ordered stream has
    to wait too, unlike QUIC's independent per-datagram delivery."""
    scheduler = Scheduler()
    timeline = FailureTimeline(HOME_BROADBAND_WIFI6)
    timeline.add_blackout(0.0, 0.5)

    delivered = []

    def on_deliver(seg, arrival):
        delivered.append((seg.seq, arrival))

    link = NaiveLink(scheduler, timeline, seed=0)
    link.set_deliver_callback(on_deliver)

    for i in range(3):
        seg = NaiveSegment(send_time=0.0, size_wire_bytes=200, category="command", seq=i, direction="operator_to_robot")
        link.send(seg)

    scheduler.run(until=3.0)

    assert [seq for seq, _ in delivered] == [0, 1, 2]  # strict FIFO order preserved
    for _, arrival in delivered:
        assert arrival >= 0.5  # nothing could get through until the blackout window closed


def test_naive_liveness_check_does_not_fire_for_a_short_blackout():
    """RoboProtocol's blackout preset (2.5s) is well past the Class B
    hardware-watchdog threshold (200ms, SR-4.1) but well short of a
    naive WS client's coarse ping/pong reconnect policy -- so a naive
    implementation doesn't even register anything is wrong. That silent
    degradation (stale commands keep flowing once the link recovers,
    with no safety trigger at all) is the point of the comparison."""
    run = run_simulation("simple_arm", "blackout", duration=10.0, seed=3)
    naive_run = run_naive_simulation("simple_arm", "blackout", duration=10.0, seed=3)

    assert len(run.watchdog.trigger_events) >= 1
    assert naive_run.session.dead_detected_at is None


def test_naive_liveness_check_fires_after_its_own_timeout():
    """For a blackout long enough to cross LIVENESS_TIMEOUT_S, the coarse
    ping/pong-style check does eventually fire -- multiple orders of
    magnitude slower than RoboProtocol's watchdog, but not never."""
    scheduler = Scheduler()
    timeline = FailureTimeline(HOME_BROADBAND_WIFI6)
    blackout_start = 1.0
    timeline.add_blackout(blackout_start, blackout_start + LIVENESS_TIMEOUT_S + 3.0)

    link = NaiveLink(scheduler, timeline, seed=0)
    scenario = SCENARIOS["simple_arm"]
    from roboprotocol_sim.metrics.naive_collector import NaiveMetricsCollector

    metrics = NaiveMetricsCollector()
    session = NaiveWebSocketSession(scenario, scheduler, link, metrics, seed=0)
    link.set_deliver_callback(session.on_delivered)

    scheduler.run(until=blackout_start + LIVENESS_TIMEOUT_S + LIVENESS_CHECK_INTERVAL_S + 2.0)

    assert session.dead_detected_at is not None
    detection_latency_s = session.dead_detected_at - blackout_start
    assert LIVENESS_TIMEOUT_S <= detection_latency_s <= LIVENESS_TIMEOUT_S + LIVENESS_CHECK_INTERVAL_S + 1.0


def test_naive_command_delay_is_not_lower_than_quic_under_loss():
    """Under a sustained loss burst, RoboProtocol just drops stale Channel
    B datagrams (dead-reckoning covers the gap) instead of retrying them,
    so *delivered* command delay stays near baseline OWD. The naive
    stream instead blocks and retries in order, so its delivered-command
    delay should not be better than RoboProtocol's, and is typically far
    worse once retries stack up."""
    run = run_simulation("simple_arm", "packet_loss_40", duration=8.0, seed=5)
    naive_run = run_naive_simulation("simple_arm", "packet_loss_40", duration=8.0, seed=5)

    dgram_df = run.collector.to_dataframes()["datagrams"]
    quic_cmd = dgram_df[(dgram_df["channel"] == "B") & (dgram_df["category"] == "command") & dgram_df["delay_ms"].notna()]
    naive_seg = naive_run.collector.to_dataframes()["segments"]
    naive_cmd = naive_seg[(naive_seg["category"] == "command") & naive_seg["delay_ms"].notna()]

    assert not quic_cmd.empty
    assert not naive_cmd.empty
    assert naive_cmd["delay_ms"].max() >= quic_cmd["delay_ms"].max()
