"""Mechanical pass/fail checks against specific REQUIREMENTS.md IDs.

REQUIREMENTS.md Sec.7's own Requirement Verification Matrix is sparse
(a documented gap from a prior review pass); this module is a concrete
down-payment on it for the subset of NFR/SR clauses a network/protocol
simulation can actually adjudicate mechanically. Each check reports
PASS / FAIL / N/A ("not exercised by this run") rather than forcing a
boolean, since e.g. a watchdog-timing check is meaningless on a run
that never triggered the watchdog.

Nominal-condition checks (NFR-1.2, NFR-1.3) are evaluated only up to
the first injected-failure event, since those NFRs are claims about
steady-state operation -- a deliberately-injected blackout is SUPPOSED
to blow through the RTT target, that is the point of running it, not a
protocol defect.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import pandas as pd

from ..protocol.safety_state_machine import (
    RESUME_STABILITY_WINDOW_S,
    TASK_CLASS_THRESHOLDS,
    WATCHDOG_BLACKOUT_MS,
)
from ..protocol.sizing import HAPTIC_MIN_TIER
from .collector import MetricsCollector


@dataclass
class NfrCheck:
    id: str
    description: str
    status: str  # "PASS" | "FAIL" | "N/A"
    detail: str


def _nominal_window_end(run) -> float:
    starts = [e["t"] for e in run.timeline.events if e["kind"].endswith("_start")]
    return min(starts) if starts else run.duration


def check_control_rtt(run, dfs: dict[str, pd.DataFrame]) -> NfrCheck:
    threshold_ms = 30.0 if run.network_profile_name == "home_broadband_wifi6" else 60.0
    window_end = _nominal_window_end(run)
    rtt_df = dfs["rtt"]
    nominal = rtt_df[rtt_df["t"] < window_end]
    if nominal.empty:
        return NfrCheck("NFR-1.2", "Control RTT under nominal conditions", "N/A", "no RTT samples before first failure window")
    p95 = float(nominal["rtt_ms"].quantile(0.95))
    passed = p95 <= threshold_ms
    return NfrCheck(
        "NFR-1.2", "Control RTT under nominal conditions",
        "PASS" if passed else "FAIL",
        f"p95 RTT = {p95:.1f}ms vs {threshold_ms:.0f}ms target ({run.network_profile_name})",
    )


def check_jitter_tolerance(run, dfs: dict[str, pd.DataFrame]) -> NfrCheck:
    """NFR-1.3 is about *packet arrival jitter*, so this measures the
    stddev of observed one-way delay on delivered Channel B datagrams
    directly, rather than a derived RTT-successive-difference stat
    (which combines two independent hops and a differencing transform,
    inflating the number by ~sqrt(2) relative to the per-hop jitter the
    spec is actually describing)."""
    window_end = _nominal_window_end(run)
    dgram_df = dfs["datagrams"]
    nominal = dgram_df[
        (dgram_df["channel"] == "B") & (dgram_df["send_time"] < window_end) & (dgram_df["delivered"] == True)  # noqa: E712
    ]
    if len(nominal) < 5:
        return NfrCheck("NFR-1.3", "Jitter within 15ms absorbable budget", "N/A", "insufficient nominal-window Channel B deliveries")
    jitter_std = float(nominal["delay_ms"].std())
    passed = jitter_std <= 15.0
    return NfrCheck(
        "NFR-1.3", "Jitter within 15ms absorbable budget",
        "PASS" if passed else "FAIL",
        f"stddev of Channel B one-way delay = {jitter_std:.1f}ms vs 15ms budget",
    )


def check_video_bitrate_envelope(run, dfs: dict[str, pd.DataFrame]) -> NfrCheck:
    video_df = dfs["video_bitrate"]
    if video_df.empty:
        return NfrCheck("NFR-2.1", "Video bitrate stays within 2-20Mbps", "N/A", "no video streams in this scenario")
    within = video_df["target_bps"].between(2_000_000 - 1, 20_000_000 + 1)
    passed = bool(within.all())
    return NfrCheck(
        "NFR-2.1", "Video bitrate stays within 2-20Mbps",
        "PASS" if passed else "FAIL",
        f"min={video_df['target_bps'].min()/1e6:.2f}Mbps max={video_df['target_bps'].max()/1e6:.2f}Mbps",
    )


# loss_prob actually injected by each uniform burst-loss preset (see failure_presets.py)
_PRESET_INJECTED_LOSS = {"packet_loss_20": 0.20, "packet_loss_40": 0.40}


def check_flexfec_recovery_boundary(run, dfs: dict[str, pd.DataFrame]) -> NfrCheck:
    """NFR-3.1's "up to 30% loss" claim is about a uniform burst-loss
    process, so this check only issues a PASS/FAIL verdict for the two
    presets that actually inject one (packet_loss_20/40); other
    presets get a descriptive N/A. It also deliberately uses the
    preset's *known* injected loss_prob rather than reconstructing an
    average from the FEC block log: a run-wide average dilutes the
    failure window against the clean before/after periods (e.g.
    packet_loss_40's true 40% in-window loss averages out to ~13%
    across a 20s run), which would silently mislabel a real
    above-boundary loss rate as within-boundary.
    """
    fec_df = dfs["fec"]
    if fec_df.empty:
        return NfrCheck("NFR-3.1", "FlexFEC recovers up to 30% loss", "N/A", "no FEC blocks formed (no video streams, or run too short)")
    recovered_fraction = float(fec_df["recovered"].mean())
    if run.failure_name not in _PRESET_INJECTED_LOSS:
        return NfrCheck(
            "NFR-3.1", "FlexFEC recovers up to 30% loss", "N/A",
            f"not a uniform burst-loss preset (recovered {recovered_fraction:.0%} of blocks descriptively; "
            f"see packet_loss_20 / packet_loss_40 for the boundary check)",
        )
    injected_loss = _PRESET_INJECTED_LOSS[run.failure_name]
    expect_recoverable = injected_loss <= 0.30
    consistent = (expect_recoverable and recovered_fraction >= 0.9) or (
        not expect_recoverable and recovered_fraction < 0.9
    )
    return NfrCheck(
        "NFR-3.1", "FlexFEC recovers up to 30% loss",
        "PASS" if consistent else "FAIL",
        f"injected burst loss={injected_loss:.0%}, recovered={recovered_fraction:.0%} of blocks (run-wide, incl. outside the failure window)",
    )


def check_watchdog_timing(run, dfs: dict[str, pd.DataFrame]) -> NfrCheck:
    if not run.watchdog.trigger_events:
        return NfrCheck("SR-4.1", "Watchdog fires at the spec'd blackout threshold", "N/A", "watchdog never triggered in this run")
    threshold_ms = WATCHDOG_BLACKOUT_MS[run.scenario.task_class]
    worst_err = max(abs(ev.blackout_ms - threshold_ms) for ev in run.watchdog.trigger_events)
    passed = worst_err < 5.0
    return NfrCheck(
        "SR-4.1", "Watchdog fires at the spec'd blackout threshold",
        "PASS" if passed else "FAIL",
        f"{len(run.watchdog.trigger_events)} trigger(s), worst timing error={worst_err:.2f}ms vs {threshold_ms:.0f}ms threshold",
    )


def check_resume_hysteresis(run, dfs: dict[str, pd.DataFrame]) -> NfrCheck:
    resumes = [t for t in run.safety_sm.transitions if t.trigger == "resume"]
    if not resumes:
        return NfrCheck("SR-3.3", "Resume requires >=2s continuous stability", "N/A", "no SUSPENDED->resume transition occurred")
    suspends = [t for t in run.safety_sm.transitions if t.new_tier == 4]
    ok = True
    detail_parts = []
    for r in resumes:
        prior_suspend = max((s.t for s in suspends if s.t <= r.t), default=None)
        if prior_suspend is None:
            continue
        gap = r.t - prior_suspend
        ok = ok and (gap >= RESUME_STABILITY_WINDOW_S)
        detail_parts.append(f"{gap:.2f}s")
    return NfrCheck(
        "SR-3.3", "Resume requires >=2s continuous stability",
        "PASS" if ok else "FAIL",
        f"suspend->resume gaps: {', '.join(detail_parts)} (>= {RESUME_STABILITY_WINDOW_S:.0f}s required)",
    )


def check_haptic_floor(run, dfs: dict[str, pd.DataFrame]) -> NfrCheck:
    tier = run.scenario.haptic_tier
    passed = tier.value >= HAPTIC_MIN_TIER.value
    return NfrCheck(
        "FR-3.1", "Haptic quantization never below Standard tier",
        "PASS" if passed else "FAIL",
        f"configured haptic tier={tier.name} ({tier.value}B/field), floor={HAPTIC_MIN_TIER.name} ({HAPTIC_MIN_TIER.value}B/field)",
    )


ALL_CHECKS = [
    check_control_rtt,
    check_jitter_tolerance,
    check_video_bitrate_envelope,
    check_flexfec_recovery_boundary,
    check_watchdog_timing,
    check_resume_hysteresis,
    check_haptic_floor,
]


def run_all_checks(run, dfs: dict[str, pd.DataFrame]) -> list[NfrCheck]:
    return [check(run, dfs) for check in ALL_CHECKS]
