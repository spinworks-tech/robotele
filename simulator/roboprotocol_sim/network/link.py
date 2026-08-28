"""Packet-level directional link: priority queue + single deterministic
server (the access-link bottleneck) + propagation/jitter/loss.

Two priority classes mirror NFR-2.2's DSCP tagging: CONTROL (Channel B
commands/telemetry/haptic + Channel C E-Stop) gets DSCP-EF-equivalent
strict priority over MEDIA (Channel A video, DSCP AF41). Under a
bandwidth-constrained failure window this is what makes video degrade
before control traffic does -- the behaviour NFR-2.2 exists to
guarantee, and this simulator can show it happening rather than just
asserting it.

Simplification (documented): available bandwidth for a packet's
serialization time is sampled once, at the moment it starts service,
even if a failure window's bandwidth changes mid-transmission. For the
datagram sizes and window durations used by this simulator that
approximation is immaterial; it keeps the queueing model a simple
single-server-with-priority-preemption-at-arrival design instead of a
piecewise-service-rate integral.
"""
from __future__ import annotations

import heapq
import itertools
import random
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Optional

from ..core.scheduler import Scheduler
from .failures import FailureTimeline

MIN_BANDWIDTH_MBPS = 0.01  # floor to avoid an infinite/zero service time during a near-total outage


class Priority(IntEnum):
    CONTROL = 0  # Channel B (command/telemetry/haptic) + Channel C E-Stop -- DSCP EF equivalent
    MEDIA = 1    # Channel A video -- DSCP AF41 equivalent


@dataclass
class Datagram:
    send_time: float
    size_wire_bytes: int
    size_app_bytes: int
    priority: Priority
    channel: str       # "A" | "B" | "C"
    category: str      # "command" | "telemetry" | "haptic" | "video" | "fec_parity" | "estop" | "rpc"
    seq: int
    direction: str      # "robot_to_operator" | "operator_to_robot"
    meta: dict = field(default_factory=dict)


DeliverCallback = Callable[[Datagram, bool, Optional[float]], None]


class DirectionalLink:
    """FIFO-with-priority server for one direction of the link."""

    def __init__(
        self,
        scheduler: Scheduler,
        timeline: FailureTimeline,
        direction: str,
        rng: random.Random,
        on_deliver: DeliverCallback,
    ) -> None:
        self.scheduler = scheduler
        self.timeline = timeline
        self.direction = direction
        self.rng = rng
        self.on_deliver = on_deliver
        self._queue: list[tuple[int, int, Datagram]] = []
        self._seq = itertools.count()
        self._draining = False
        self.queue_depth_samples: list[tuple[float, int]] = []

    def enqueue(self, datagram: Datagram) -> None:
        heapq.heappush(self._queue, (int(datagram.priority), next(self._seq), datagram))
        self.queue_depth_samples.append((self.scheduler.now, len(self._queue)))
        if not self._draining:
            self._drain()

    def _drain(self) -> None:
        if not self._queue:
            self._draining = False
            return
        self._draining = True
        now = self.scheduler.now
        bw_mbps = max(
            self.timeline.effective_bandwidth_mbps(now, self.direction), MIN_BANDWIDTH_MBPS
        )
        _, _, datagram = heapq.heappop(self._queue)
        self.queue_depth_samples.append((now, len(self._queue)))
        service_time = (datagram.size_wire_bytes * 8) / (bw_mbps * 1e6)
        self.scheduler.schedule(service_time, lambda: self._finish_service(datagram))

    def _finish_service(self, datagram: Datagram) -> None:
        self._apply_propagation_and_loss(datagram)
        self._drain()

    def _apply_propagation_and_loss(self, datagram: Datagram) -> None:
        now = self.scheduler.now
        params = self.timeline.effective_params(now)
        if self.rng.random() < params.loss_prob:
            self.on_deliver(datagram, False, None)
            return
        jitter_ms = self.rng.gauss(0.0, params.jitter_std_ms)
        owd_ms = max(0.1, params.owd_ms + jitter_ms)
        arrival = now + owd_ms / 1000.0
        self.scheduler.schedule_at(arrival, lambda: self.on_deliver(datagram, True, arrival))


class Link:
    """Both directions of the operator<->robot path, sharing one FailureTimeline."""

    def __init__(self, scheduler: Scheduler, timeline: FailureTimeline, seed: int = 0) -> None:
        self.scheduler = scheduler
        self.timeline = timeline
        self._rng = random.Random(seed)
        self._deliver_cb: Optional[DeliverCallback] = None
        self.robot_to_operator = DirectionalLink(
            scheduler, timeline, "robot_to_operator", self._rng, self._dispatch
        )
        self.operator_to_robot = DirectionalLink(
            scheduler, timeline, "operator_to_robot", self._rng, self._dispatch
        )

    def set_deliver_callback(self, cb: DeliverCallback) -> None:
        self._deliver_cb = cb

    def _dispatch(self, datagram: Datagram, delivered: bool, arrival_time: Optional[float]) -> None:
        if self._deliver_cb is not None:
            self._deliver_cb(datagram, delivered, arrival_time)

    def send(self, datagram: Datagram) -> None:
        link = (
            self.robot_to_operator
            if datagram.direction == "robot_to_operator"
            else self.operator_to_robot
        )
        link.enqueue(datagram)
