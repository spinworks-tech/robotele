"""Named failure-injection presets, mapped to REQUIREMENTS.md Sec.2's
"Typical Operational Problems" table and the specific network failure
modes requested for this simulator. Each preset places its window(s)
relative to the run duration so the same preset works at any --duration.
"""
from __future__ import annotations

from typing import Callable

from ..network.failures import FailureTimeline
from ..network.profiles import CELLULAR_5G, HOME_BROADBAND_WIFI6

FailurePresetFn = Callable[[FailureTimeline, float], None]


def preset_none(timeline: FailureTimeline, duration: float) -> None:
    """Baseline: no injected failures, just the profile's steady-state noise."""


def preset_bandwidth_drop(timeline: FailureTimeline, duration: float) -> None:
    """Sustained congestion: uplink capacity cut ~75% for a third of the run."""
    timeline.add_bandwidth_drop(duration * 0.40, duration * 0.70, reduction_fraction=0.75)


def preset_blackout(timeline: FailureTimeline, duration: float) -> None:
    """Sudden total connection drop -- exercises the SR-4.1 watchdog and SR-3.3 resume."""
    start = duration * 0.45
    timeline.add_blackout(start, start + 2.5)


def preset_packet_loss_20(timeline: FailureTimeline, duration: float) -> None:
    """20% burst loss -- within FlexFEC's claimed "up to 30%" recovery envelope (NFR-3.1)."""
    timeline.add_packet_loss_burst(duration * 0.40, duration * 0.70, loss_prob=0.20)


def preset_packet_loss_40(timeline: FailureTimeline, duration: float) -> None:
    """40% burst loss -- deliberately past NFR-3.1's 30% boundary, to show FEC failing."""
    timeline.add_packet_loss_burst(duration * 0.40, duration * 0.70, loss_prob=0.40)


def preset_jitter_spike(timeline: FailureTimeline, duration: float) -> None:
    """Jitter blown out past NFR-1.3's 15ms tolerance."""
    timeline.add_jitter_spike(duration * 0.40, duration * 0.70, factor=6.0)


def preset_latency_ramp(timeline: FailureTimeline, duration: float) -> None:
    """Gradual congestion buildup, held at peak -- walks Class B through Tiers 0 to 4 (SR-3.2)."""
    timeline.add_latency_ramp(duration * 0.30, duration * 0.60, peak_owd_ms=300.0)


def preset_latency_spike_recover(timeline: FailureTimeline, duration: float) -> None:
    """Latency spikes past SUSPENDED then recovers -- the only preset that
    exercises SR-3.3's resume path (a held ramp never comes back down, and
    a blackout gives the RTT-driven state machine no signal at all)."""
    timeline.add_latency_spike(duration * 0.30, duration * 0.40, duration * 0.55, peak_owd_ms=300.0)


def preset_handover(timeline: FailureTimeline, duration: float) -> None:
    """Wi-Fi<->5G path handover mid-session (DESIGN.md Sec.6 QUIC Connection Migration)."""
    other = CELLULAR_5G if timeline.base_profile.name != CELLULAR_5G.name else HOME_BROADBAND_WIFI6
    timeline.add_handover(duration * 0.50, other)


def preset_compound_congestion(timeline: FailureTimeline, duration: float) -> None:
    """Realistic congested-cellular scenario: bandwidth drop + loss burst together."""
    start, end = duration * 0.40, duration * 0.75
    timeline.add_bandwidth_drop(start, end, reduction_fraction=0.60)
    timeline.add_packet_loss_burst(start, end, loss_prob=0.15)


FAILURE_PRESETS: dict[str, FailurePresetFn] = {
    "none": preset_none,
    "bandwidth_drop": preset_bandwidth_drop,
    "blackout": preset_blackout,
    "packet_loss_20": preset_packet_loss_20,
    "packet_loss_40": preset_packet_loss_40,
    "jitter_spike": preset_jitter_spike,
    "latency_ramp": preset_latency_ramp,
    "latency_spike_recover": preset_latency_spike_recover,
    "handover": preset_handover,
    "compound_congestion": preset_compound_congestion,
}


def get_failure_preset(name: str) -> FailurePresetFn:
    try:
        return FAILURE_PRESETS[name]
    except KeyError as exc:
        raise ValueError(f"unknown failure preset {name!r}; choices: {sorted(FAILURE_PRESETS)}") from exc
