"""Time-windowed failure injection over a NetworkProfile.

Each failure is a modifier active during [start, end) (simulation
seconds). Multiple windows can overlap (e.g. a "compound congestion"
scenario stacks bandwidth_drop + packet_loss_burst); overlapping
windows combine multiplicatively for bandwidth/jitter and via
independent-probability combination for loss, so stacking two 50%
loss windows does not naively sum to 100%.

A "handover" is modelled as DESIGN.md Sec.6 actually specifies it:
reactive QUIC Connection Migration, not concurrent path bonding
(NFR-3.2's literal "dual-interface bonding" wording is a known,
previously-flagged spec ambiguity -- this simulator implements the
concretely-specified Sec.6 mechanism). A handover = a short blackout
covering (detection delay + one path-validation RTT on the new path),
followed by a hard switch to the new NetworkProfile.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .profiles import NetworkProfile


@dataclass
class _Window:
    kind: str
    start: float
    end: float
    bandwidth_mult: float = 1.0
    loss_prob: float = 0.0
    jitter_mult: float = 1.0
    extra_owd_ms: float = 0.0

    def active_at(self, t: float) -> bool:
        return self.start <= t < self.end


@dataclass
class _ProfileSwitch:
    at: float
    profile: NetworkProfile


@dataclass
class _LatencyRamp:
    start: float
    end: float
    peak_extra_owd_ms: float

    def extra_owd_at(self, t: float) -> float:
        if t < self.start:
            return 0.0
        if t >= self.end:
            return self.peak_extra_owd_ms
        frac = (t - self.start) / (self.end - self.start)
        return frac * self.peak_extra_owd_ms


@dataclass
class _TriangleLatencySpike:
    """Ramps 0 -> peak -> 0, unlike _LatencyRamp which holds at peak.
    Used to exercise SR-3.3's SUSPENDED->resume path, which a
    permanently-held ramp or a total blackout cannot: the RTT-driven
    Tier 0-4 state machine needs a *recovering* latency signal, and a
    blackout produces no RTT samples at all (that is what the separate
    Tier 5 watchdog is for).
    """

    start: float
    peak_at: float
    end: float
    peak_extra_owd_ms: float

    def extra_owd_at(self, t: float) -> float:
        if t < self.start or t >= self.end:
            return 0.0
        if t < self.peak_at:
            frac = (t - self.start) / (self.peak_at - self.start)
        else:
            frac = 1.0 - (t - self.peak_at) / (self.end - self.peak_at)
        return frac * self.peak_extra_owd_ms


@dataclass
class EffectiveParams:
    owd_ms: float
    jitter_std_ms: float
    loss_prob: float

    @property
    def blackout(self) -> bool:
        return self.loss_prob >= 1.0


class FailureTimeline:
    """Combines a base NetworkProfile with a set of failure windows."""

    def __init__(self, base_profile: NetworkProfile) -> None:
        self.base_profile = base_profile
        self._windows: list[_Window] = []
        self._profile_switches: list[_ProfileSwitch] = [
            _ProfileSwitch(at=0.0, profile=base_profile)
        ]
        self._ramps: list[_LatencyRamp] = []
        self._spikes: list[_TriangleLatencySpike] = []
        self.events: list[dict] = []  # human-readable log for reports

    # -- failure builders -------------------------------------------------
    def add_bandwidth_drop(self, start: float, end: float, reduction_fraction: float = 0.7) -> None:
        self._windows.append(
            _Window("bandwidth_drop", start, end, bandwidth_mult=1.0 - reduction_fraction)
        )
        self.events.append({"t": start, "kind": "bandwidth_drop_start", "detail": f"-{reduction_fraction:.0%}"})
        self.events.append({"t": end, "kind": "bandwidth_drop_end", "detail": ""})

    def add_blackout(self, start: float, end: float) -> None:
        self._windows.append(_Window("blackout", start, end, loss_prob=1.0))
        self.events.append({"t": start, "kind": "blackout_start", "detail": f"{(end - start) * 1000:.0f}ms"})
        self.events.append({"t": end, "kind": "blackout_end", "detail": ""})

    def add_packet_loss_burst(self, start: float, end: float, loss_prob: float) -> None:
        self._windows.append(_Window("packet_loss_burst", start, end, loss_prob=loss_prob))
        self.events.append({"t": start, "kind": "packet_loss_burst_start", "detail": f"{loss_prob:.0%}"})
        self.events.append({"t": end, "kind": "packet_loss_burst_end", "detail": ""})

    def add_jitter_spike(self, start: float, end: float, factor: float = 5.0) -> None:
        self._windows.append(_Window("jitter_spike", start, end, jitter_mult=factor))
        self.events.append({"t": start, "kind": "jitter_spike_start", "detail": f"x{factor}"})
        self.events.append({"t": end, "kind": "jitter_spike_end", "detail": ""})

    def add_latency_ramp(self, start: float, end: float, peak_owd_ms: float) -> None:
        base = self.base_profile.base_owd_ms
        self._ramps.append(_LatencyRamp(start, end, max(0.0, peak_owd_ms - base)))
        self.events.append({"t": start, "kind": "latency_ramp_start", "detail": f"-> {peak_owd_ms:.0f}ms"})

    def add_latency_spike(self, start: float, peak_at: float, end: float, peak_owd_ms: float) -> None:
        """Triangular ramp 0->peak->0 (unlike add_latency_ramp, which holds
        at peak). This is the only failure that gives the RTT-driven Tier
        0-4 state machine a recovering signal, so it is what exercises
        SR-3.3's SUSPENDED->resume path."""
        base = self.base_profile.base_owd_ms
        self._spikes.append(_TriangleLatencySpike(start, peak_at, end, max(0.0, peak_owd_ms - base)))
        self.events.append({"t": start, "kind": "latency_spike_start", "detail": f"-> {peak_owd_ms:.0f}ms -> baseline"})
        self.events.append({"t": end, "kind": "latency_spike_end", "detail": ""})

    def add_handover(
        self,
        at: float,
        new_profile: NetworkProfile,
        detection_delay_s: float = 0.15,
        validation_rtt_s: Optional[float] = None,
    ) -> None:
        """Wi-Fi<->5G style reactive migration (DESIGN.md Sec.6)."""
        if validation_rtt_s is None:
            validation_rtt_s = 2 * new_profile.base_owd_ms / 1000.0
        gap_end = at + detection_delay_s + validation_rtt_s
        self._windows.append(_Window("handover_gap", at, gap_end, loss_prob=1.0))
        self._profile_switches.append(_ProfileSwitch(at=gap_end, profile=new_profile))
        self.events.append(
            {"t": at, "kind": "handover_start", "detail": f"-> {new_profile.name}"}
        )
        self.events.append(
            {"t": gap_end, "kind": "handover_complete", "detail": f"gap={( gap_end - at) * 1000:.0f}ms"}
        )

    # -- query --------------------------------------------------------------
    def current_profile(self, t: float) -> NetworkProfile:
        active = self._profile_switches[0].profile
        for sw in self._profile_switches:
            if sw.at <= t:
                active = sw.profile
            else:
                break
        return active

    def effective_params(self, t: float) -> EffectiveParams:
        profile = self.current_profile(t)
        jitter_mult = 1.0
        loss_keep_prob = 1.0  # product of (1 - loss_i)
        for w in self._windows:
            if w.active_at(t):
                jitter_mult *= w.jitter_mult
                loss_keep_prob *= (1.0 - w.loss_prob)

        extra_owd_ms = sum(r.extra_owd_at(t) for r in self._ramps) + sum(
            s.extra_owd_at(t) for s in self._spikes
        )
        combined_loss = 1.0 - (1.0 - profile.baseline_loss) * loss_keep_prob

        return EffectiveParams(
            owd_ms=profile.base_owd_ms + extra_owd_ms,
            jitter_std_ms=profile.jitter_std_ms * jitter_mult,
            loss_prob=min(1.0, combined_loss),
        )

    def effective_bandwidth_mbps(self, t: float, direction: str) -> float:
        profile = self.current_profile(t)
        base = profile.robot_upload_mbps if direction == "robot_to_operator" else profile.operator_upload_mbps
        mult = 1.0
        for w in self._windows:
            if w.active_at(t):
                mult *= w.bandwidth_mult
        return max(0.0, base * mult)
