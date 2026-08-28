"""Wires a ScenarioConfig + failure preset + network profile into a full
discrete-event simulation run and returns the collected results.
"""
from __future__ import annotations

from dataclasses import dataclass

from ..core.scheduler import Scheduler
from ..metrics.collector import MetricsCollector
from ..metrics.naive_collector import NaiveMetricsCollector
from ..network.failures import FailureTimeline
from ..network.link import Link
from ..network.naive_link import NaiveLink
from ..network.profiles import get_profile
from ..protocol.channel_a import VideoStream
from ..protocol.channel_b import ChannelB, ChannelBConfig
from ..protocol.naive_session import NAIVE_RECONNECT_COST_S, NaiveWebSocketSession
from ..protocol.safety_state_machine import SafetyStateMachine, Watchdog
from ..protocol.tdpa import PassivityObserver
from .definitions import ScenarioConfig, get_scenario
from .failure_presets import get_failure_preset


@dataclass
class RunResult:
    scenario: ScenarioConfig
    failure_name: str
    network_profile_name: str
    duration: float
    collector: MetricsCollector
    safety_sm: SafetyStateMachine
    watchdog: Watchdog
    tdpa: PassivityObserver
    timeline: FailureTimeline
    video_streams: list[VideoStream]


def run_simulation(
    scenario_key: str,
    failure_name: str,
    network_profile_name: str = "home_broadband_wifi6",
    duration: float = 20.0,
    seed: int = 0,
) -> RunResult:
    scenario = get_scenario(scenario_key)
    scheduler = Scheduler()

    base_profile = get_profile(network_profile_name)
    timeline = FailureTimeline(base_profile)
    get_failure_preset(failure_name)(timeline, duration)

    link = Link(scheduler, timeline, seed=seed)
    metrics = MetricsCollector()
    safety_sm = SafetyStateMachine(scenario.task_class, scheduler)
    watchdog = Watchdog(scenario.task_class, scheduler, on_estop=lambda t, ms: None)
    tdpa = PassivityObserver(timeline)

    channel_b_config = ChannelBConfig(
        control_hz=scenario.control_hz,
        task_class=scenario.task_class,
        command_fields=scenario.command_fields,
        command_tier=scenario.command_tier,
        telemetry_fields=scenario.telemetry_fields,
        telemetry_tier=scenario.telemetry_tier,
        haptic_fields=scenario.haptic_fields,
        haptic_tier=scenario.haptic_tier,
    )
    channel_b = ChannelB(channel_b_config, scheduler, link, timeline, safety_sm, watchdog, tdpa, metrics, seed=seed)

    video_streams = [
        VideoStream(vs_config, scheduler, link, timeline, metrics, seed=seed + i + 1)
        for i, vs_config in enumerate(scenario.video_streams)
    ]
    stream_by_name = {vs.config.name: vs for vs in video_streams}

    def on_deliver(dgram, delivered, arrival_time):
        metrics.log_datagram_delivered(dgram, delivered, arrival_time)
        if dgram.channel == "B":
            channel_b.on_delivery_result(dgram, delivered)
        elif dgram.channel == "A":
            vs = stream_by_name.get(dgram.meta.get("stream"))
            if vs is not None:
                vs.on_delivery_result(dgram, delivered)

    link.set_deliver_callback(on_deliver)

    scheduler.run(until=duration)

    return RunResult(
        scenario=scenario,
        failure_name=failure_name,
        network_profile_name=network_profile_name,
        duration=duration,
        collector=metrics,
        safety_sm=safety_sm,
        watchdog=watchdog,
        tdpa=tdpa,
        timeline=timeline,
        video_streams=video_streams,
    )


@dataclass
class NaiveRunResult:
    scenario: ScenarioConfig
    failure_name: str
    network_profile_name: str
    duration: float
    collector: NaiveMetricsCollector
    session: NaiveWebSocketSession
    timeline: FailureTimeline


# Same fractional offset preset_handover() uses for its own `at` -- kept in
# sync so the naive extra-reconnect window lands right after the QUIC gap.
_HANDOVER_AT_FRACTION = 0.50


def run_naive_simulation(
    scenario_key: str,
    failure_name: str,
    network_profile_name: str = "home_broadband_wifi6",
    duration: float = 20.0,
    seed: int = 0,
) -> NaiveRunResult:
    """Runs the same scenario+failure combo through the naive WebSocket/TCP
    baseline instead of RoboProtocol's QUIC-based channels, for a
    like-for-like comparison. Uses its own Scheduler and FailureTimeline
    instances (built from the identical failure preset) so the two runs
    are statistically comparable without being lockstep-coupled to
    RoboProtocol's own RNG consumption pattern.
    """
    scenario = get_scenario(scenario_key)
    scheduler = Scheduler()

    base_profile = get_profile(network_profile_name)
    timeline = FailureTimeline(base_profile)
    get_failure_preset(failure_name)(timeline, duration)
    if failure_name == "handover":
        # A naive WS/TCP client has no QUIC Connection Migration: it must
        # detect the dead socket, then pay a fresh TCP handshake + WS
        # upgrade, on top of whatever gap QUIC's own migration needs.
        at = duration * _HANDOVER_AT_FRACTION
        timeline.add_blackout(at, at + NAIVE_RECONNECT_COST_S)

    link = NaiveLink(scheduler, timeline, seed=seed)
    metrics = NaiveMetricsCollector()
    session = NaiveWebSocketSession(scenario, scheduler, link, metrics, seed=seed)

    link.set_deliver_callback(session.on_delivered)

    scheduler.run(until=duration)

    return NaiveRunResult(
        scenario=scenario,
        failure_name=failure_name,
        network_profile_name=network_profile_name,
        duration=duration,
        collector=metrics,
        session=session,
        timeline=timeline,
    )
