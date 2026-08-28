"""Simplified Time-Domain Passivity Approach (TDPA) proxy (DESIGN.md Sec.3.4,
REQUIREMENTS.md FR-3.1/FR-3.2).

This is NOT a research-grade TDPA implementation. It is a small,
deterministic model chosen because it reproduces the textbook
mechanism by which network delay destabilizes force feedback, using
only the network delay this simulator already tracks (no need to
thread real force/torque payload bytes through the whole event
pipeline for this to be illustrative):

* The operator's hand velocity is a synthetic sinusoid v(t).
* A robot-side virtual damper would, with zero delay, always return
  F_ideal(t) = -k * v(t), i.e. force opposing velocity, so
  instantaneous power P(t) = F*v = -k*v(t)^2 <= 0 (always dissipative
  -- passive) at every instant.
* Because of network delay d(t) (sampled from the same NetworkProfile
  / FailureTimeline the rest of the simulator uses), the operator only
  has F_received(t) = -k * v(t - d(t)) available. Once d(t) is
  comparable to the motion's own period, F_received(t) and v(t) can
  fall out of phase enough that their product goes *positive* --
  exactly the "virtual energy injected into the loop" failure mode
  Sec.3.4 describes, and exactly why FR-3.2 requires an energy-integral
  passivity observer in the first place.
* Cumulative energy E(t) = integral of P(t) dt is tracked; a real
  passivity CONTROLLER would clamp/attenuate F once E(t) trends
  positive. This module only implements the OBSERVER half (detection),
  logging a violation whenever E(t) exceeds a small tolerance -- which
  is what the simulator's reports care about (how often/how badly does
  the link's delay profile threaten passivity).
* Haptic quantization below the FR-3.1 Standard-tier floor is modelled
  as an added force-error bias, so a (deliberately misconfigured,
  should-be-impossible) sub-floor run visibly produces more/worse
  violations -- demonstrating why FR-3.1 hard-enforces the floor rather
  than leaving it as a recommendation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from ..network.failures import FailureTimeline

QUANTIZATION_NOISE_BIAS = 0.15  # fractional force error injected when floor is violated


@dataclass
class PassivityViolation:
    t: float
    energy: float
    detail: str


class PassivityObserver:
    def __init__(
        self,
        timeline: FailureTimeline,
        damping_gain: float = 5.0,
        velocity_amplitude: float = 0.3,
        velocity_freq_hz: float = 1.5,
        violation_tolerance: float = 1e-3,
    ) -> None:
        self.timeline = timeline
        self.k = damping_gain
        self.v0 = velocity_amplitude
        self.f = velocity_freq_hz
        self.tolerance = violation_tolerance
        self._energy = 0.0
        self._last_t = 0.0
        self.violations: list[PassivityViolation] = []
        self.energy_log: list[tuple[float, float]] = []

    def _velocity(self, t: float) -> float:
        return self.v0 * math.sin(2.0 * math.pi * self.f * t)

    def sample(self, t: float, haptic_tier_ok: bool = True) -> float:
        """Advance the observer to time t; call once per haptic tick.

        Returns the instantaneous power (for diagnostics/plots).
        """
        owd_s = self.timeline.effective_params(t).owd_ms / 1000.0
        delayed_t = max(0.0, t - owd_s)
        v_now = self._velocity(t)
        f_received = -self.k * self._velocity(delayed_t)
        if not haptic_tier_ok:
            f_received *= 1.0 + QUANTIZATION_NOISE_BIAS
        power = f_received * v_now

        dt = max(0.0, t - self._last_t)
        self._energy += power * dt
        self._last_t = t
        self.energy_log.append((t, self._energy))

        if self._energy > self.tolerance:
            self.violations.append(
                PassivityViolation(t, self._energy, "virtual energy injected (E(t) > 0)")
            )
        return power
