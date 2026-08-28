"""Channel B: bidirectional command / telemetry / haptic datagram stream
(DESIGN.md Sec.2.2, REQUIREMENTS.md FR-1.x/FR-2.2/FR-3.x, SR-3.1).

Each control tick sends three logical messages:
  * command   (operator -> robot)
  * telemetry (robot -> operator)
  * haptic    (robot -> operator, floored at Standard quantization)

RTT sampling (SR-3.1) is modelled as: the command's one-way delay
(operator->robot) plus the same tick's telemetry one-way delay
(robot->operator) approximates a round trip, without needing to
implement an explicit timestamp-echo sub-protocol -- this is what an
echoed-timestamp RTT measurement converges to in expectation, and only
requires both legs of the same tick to have been *delivered* (if
either is lost, that tick contributes no RTT sample, which is exactly
why the Tier 5 watchdog below is deliberately a separate,
blackout-duration-driven mechanism rather than depending on fresh RTT
data).

The hardware watchdog (SR-4.1) is heartbeat-fed by any successfully
delivered operator->robot Channel B datagram. This stands in for the
SR-4.2 dedicated redundant 1kHz E-Stop datagram stream, since both
exist purely to detect loss of contact with the operator at the robot
edge controller; modelling them as a literally separate datagram flow
would not change any observable behaviour here.
"""
from __future__ import annotations

import itertools
import random
from dataclasses import dataclass
from typing import Optional

from ..core.scheduler import Scheduler
from ..network.failures import FailureTimeline
from ..network.link import Datagram, Link, Priority
from . import sizing
from .safety_state_machine import SafetyStateMachine, TaskClass, Watchdog
from .tdpa import PassivityObserver


@dataclass
class ChannelBConfig:
    control_hz: float
    task_class: TaskClass
    command_fields: int
    command_tier: sizing.QuantizationTier
    telemetry_fields: int
    telemetry_tier: sizing.QuantizationTier
    haptic_fields: int
    haptic_tier: sizing.QuantizationTier


class ChannelB:
    def __init__(
        self,
        config: ChannelBConfig,
        scheduler: Scheduler,
        link: Link,
        timeline: FailureTimeline,
        safety_sm: SafetyStateMachine,
        watchdog: Watchdog,
        tdpa_observer: PassivityObserver,
        metrics,
        seed: int = 0,
    ) -> None:
        self.config = config
        self.scheduler = scheduler
        self.link = link
        self.timeline = timeline
        self.safety_sm = safety_sm
        self.watchdog = watchdog
        self.tdpa = tdpa_observer
        self.metrics = metrics
        self._rng = random.Random(seed)
        self._seq = itertools.count()
        self._tick_id = itertools.count()
        self._pending_rtt: dict[int, dict] = {}
        self._haptic_tier_ok = config.haptic_tier.value >= sizing.HAPTIC_MIN_TIER.value

        self._schedule_tick()

    def _schedule_tick(self) -> None:
        self.scheduler.schedule(1.0 / self.config.control_hz, self._tick)

    def _tick(self) -> None:
        tick_id = next(self._tick_id)
        self._pending_rtt[tick_id] = {}

        # Model an attentive operator who is already watching for recovery
        # and holds the deadman-reset control down the instant SUSPENDED
        # allows it, rather than a human reaction-time delay -- isolates
        # SR-3.3's resume gate to the network-recovery timing this
        # simulator is actually measuring. Harmless no-op outside SUSPENDED.
        self.safety_sm.request_deadman_reset()

        self._send_category(
            "operator_to_robot", "command", self.config.command_fields,
            self.config.command_tier, carries_haptic=False, tick_id=tick_id, rtt_key="cmd",
        )
        self._send_category(
            "robot_to_operator", "telemetry", self.config.telemetry_fields,
            self.config.telemetry_tier, carries_haptic=False, tick_id=tick_id, rtt_key="tel",
        )
        self._send_category(
            "robot_to_operator", "haptic", self.config.haptic_fields,
            self.config.haptic_tier, carries_haptic=True, tick_id=tick_id, rtt_key=None,
        )

        self.tdpa.sample(self.scheduler.now, haptic_tier_ok=self._haptic_tier_ok)
        self._schedule_tick()

    def _send_category(
        self,
        direction: str,
        category: str,
        num_fields: int,
        tier: sizing.QuantizationTier,
        carries_haptic: bool,
        tick_id: int,
        rtt_key: Optional[str],
    ) -> None:
        plans = sizing.plan_datagrams(num_fields, tier, carries_haptic=carries_haptic)
        for i, plan in enumerate(plans):
            dgram = Datagram(
                send_time=self.scheduler.now,
                size_wire_bytes=plan.wire_bytes,
                size_app_bytes=plan.app_bytes,
                priority=Priority.CONTROL,
                channel="B",
                category=category,
                seq=next(self._seq),
                direction=direction,
                meta={"tick_id": tick_id, "rtt_key": rtt_key if i == 0 else None},
            )
            self.metrics.log_datagram_sent(dgram)
            self.link.send(dgram)

    def on_delivery_result(self, datagram: Datagram, delivered: bool) -> None:
        if datagram.channel != "B":
            return

        if delivered and datagram.direction == "operator_to_robot":
            self.watchdog.heartbeat()

        tick_id = datagram.meta.get("tick_id")
        rtt_key = datagram.meta.get("rtt_key")
        if rtt_key is None or tick_id not in self._pending_rtt:
            return
        if not delivered:
            self._pending_rtt.pop(tick_id, None)
            return

        owd_ms = (self.scheduler.now - datagram.send_time) * 1000.0
        entry = self._pending_rtt[tick_id]
        entry[rtt_key] = owd_ms
        if "cmd" in entry and "tel" in entry:
            rtt_ms = entry["cmd"] + entry["tel"]
            self.metrics.log_rtt_sample(self.scheduler.now, rtt_ms)
            self.safety_sm.on_rtt_sample(rtt_ms)
            del self._pending_rtt[tick_id]
