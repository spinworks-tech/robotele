"""Task-adaptive safety tier state machine (REQUIREMENTS.md Sec.5, SR-2/SR-3)
plus the independent hardware watchdog / E-Stop trigger (SR-4.1, SR-4.2).

Two axes, evaluated concurrently and independently, matching the spec:

1. Tier 0-4, driven by measured RTT (SR-3.1). Crossing into Tier 4
   (SUSPENDED) decouples operator input and holds the robot in active
   impedance/gravity compensation -- software-recoverable via the
   SR-3.3 handshake (explicit deadman reset + a continuous stability
   window below the nominal threshold, with hysteresis).
2. Tier 5 E-Stop, driven by *blackout duration* (time since the last
   received heartbeat datagram), not by latency. This is the local
   hardware watchdog (SR-4.1): it can fire even while Tier 0-4 has no
   fresh RTT sample to react to, e.g. mid-blackout. It is a harder
   stop than SUSPENDED and is not modelled as auto-recoverable here --
   real E-Stop recovery is an out-of-band operational procedure, not a
   network-driven state transition.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

from ..core.scheduler import EventHandle, Scheduler


class TaskClass(str, Enum):
    B = "B"  # Fine Tool & Utensil Manipulation
    C = "C"  # Coarse Object Free-Hand Pick & Place
    D = "D"  # Indoor Humanoid Locomotion
    E = "E"  # Outdoor Gross Locomotion


@dataclass(frozen=True)
class TierThresholdsMs:
    tier0_max: float  # Tier 0 NOMINAL upper bound (exclusive)
    tier1_max: float  # Tier 1 DEGRADED upper bound
    tier2_max: float  # Tier 2 CAUTIONARY upper bound
    tier3_max: float  # Tier 3 CRITICAL upper bound; >= this is Tier 4 SUSPENDED


TASK_CLASS_THRESHOLDS: dict[TaskClass, TierThresholdsMs] = {
    TaskClass.B: TierThresholdsMs(40, 80, 120, 150),
    TaskClass.C: TierThresholdsMs(80, 150, 220, 300),
    TaskClass.D: TierThresholdsMs(100, 250, 400, 500),
    TaskClass.E: TierThresholdsMs(200, 400, 700, 1000),
}

# Tier 5 watchdog / E-Stop blackout thresholds (SR-4.1, ms of heartbeat silence).
WATCHDOG_BLACKOUT_MS: dict[TaskClass, float] = {
    TaskClass.B: 200.0,
    TaskClass.C: 300.0,
    TaskClass.D: 400.0,
    TaskClass.E: 500.0,
}

RESUME_HYSTERESIS_MS = 20.0
RESUME_STABILITY_WINDOW_S = 2.0

TIER_NAMES = {0: "NOMINAL", 1: "DEGRADED", 2: "CAUTIONARY", 3: "CRITICAL", 4: "SUSPENDED"}


def tier_for_latency(task_class: TaskClass, latency_ms: float) -> int:
    th = TASK_CLASS_THRESHOLDS[task_class]
    if latency_ms < th.tier0_max:
        return 0
    if latency_ms < th.tier1_max:
        return 1
    if latency_ms < th.tier2_max:
        return 2
    if latency_ms < th.tier3_max:
        return 3
    return 4


@dataclass
class TierTransition:
    t: float
    task_class: TaskClass
    old_tier: int
    new_tier: int
    trigger: str  # "latency" | "resume"


class SafetyStateMachine:
    """Tier 0-4 RTT-driven state machine with SR-3.3 resume hysteresis."""

    def __init__(self, task_class: TaskClass, scheduler: Scheduler) -> None:
        self.task_class = task_class
        self.scheduler = scheduler
        self.tier = 0
        self.suspended = False
        self.deadman_reset_requested = False
        self._stable_since: Optional[float] = None
        self.transitions: list[TierTransition] = [
            TierTransition(0.0, task_class, 0, 0, "init")
        ]

    def request_deadman_reset(self) -> None:
        self.deadman_reset_requested = True

    def on_rtt_sample(self, rtt_ms: float) -> None:
        if self.suspended:
            self._evaluate_resume(rtt_ms)
            return
        new_tier = tier_for_latency(self.task_class, rtt_ms)
        if new_tier != self.tier:
            self._transition(new_tier, "latency")
        if new_tier == 4:
            self.suspended = True
            self._stable_since = None

    def _evaluate_resume(self, rtt_ms: float) -> None:
        th = TASK_CLASS_THRESHOLDS[self.task_class]
        resume_ceiling = th.tier0_max - RESUME_HYSTERESIS_MS
        now = self.scheduler.now
        if rtt_ms >= resume_ceiling:
            self._stable_since = None
            return
        if self._stable_since is None:
            self._stable_since = now
        stable_for = now - self._stable_since
        if stable_for >= RESUME_STABILITY_WINDOW_S and self.deadman_reset_requested:
            self.suspended = False
            self.deadman_reset_requested = False
            self._stable_since = None
            self._transition(0, "resume")

    def _transition(self, new_tier: int, trigger: str) -> None:
        old = self.tier
        self.tier = new_tier
        self.transitions.append(
            TierTransition(self.scheduler.now, self.task_class, old, new_tier, trigger)
        )


@dataclass
class WatchdogTrigger:
    t: float
    blackout_ms: float


class Watchdog:
    """Tier 5 hardware watchdog: fires E-Stop on heartbeat silence (SR-4.1/4.2)."""

    def __init__(
        self,
        task_class: TaskClass,
        scheduler: Scheduler,
        on_estop: Callable[[float, float], None],
    ) -> None:
        self.task_class = task_class
        self.scheduler = scheduler
        self.on_estop = on_estop
        self.threshold_ms = WATCHDOG_BLACKOUT_MS[task_class]
        self._last_heartbeat = 0.0
        self._timer: Optional[EventHandle] = None
        self.triggered = False
        self.trigger_events: list[WatchdogTrigger] = []
        self._arm()

    def _arm(self) -> None:
        self._timer = self.scheduler.schedule(self.threshold_ms / 1000.0, self._on_timeout)

    def heartbeat(self) -> None:
        self._last_heartbeat = self.scheduler.now
        if self._timer is not None:
            self._timer.cancel()
        if not self.triggered:
            self._arm()

    def _on_timeout(self) -> None:
        if self.triggered:
            return
        self.triggered = True
        blackout_ms = (self.scheduler.now - self._last_heartbeat) * 1000.0
        self.trigger_events.append(WatchdogTrigger(self.scheduler.now, blackout_ms))
        self.on_estop(self.scheduler.now, blackout_ms)
