"""Baseline "how would a standard WebSocket-over-TCP implementation fare"
comparison transport: one ordered, reliable, unprioritized stream per
direction carrying every message type together -- the way a typical
`ws://`/`wss://` teleop demo is actually built, with none of
RoboProtocol's QUIC-specific structure: no independent per-message
datagrams, no priority scheduling, no adaptive bitrate, no dedicated
liveness/E-Stop path.

Modeled TCP behaviour (simplified, not a full congestion-control
implementation, but the qualitatively important parts):

* Transmission is pipelined: consecutive segments are put on the wire
  back-to-back at the bandwidth-limited service rate, same as real TCP
  under a congestion/flow-control window -- a segment's own
  propagation delay does not stall the next segment's *send*.
* Delivery to the application is strictly in order. If a segment is
  lost, it retransmits independently (see below); segments queued
  behind it that *do* arrive are buffered but withheld from the
  application until the lost one is recovered -- true head-of-line
  blocking, exactly like a TCP receive buffer. QUIC's independent
  per-datagram framing on Channel A/B is specifically what avoids
  this; a single ordered stream cannot.
* Retransmission after a computed RTO: 4x the currently observed RTT,
  floored at 200ms, doubling on each consecutive loss of the *same*
  segment and capped at 3s -- the same qualitative backoff shape real
  TCP stacks use.
* No priority: control, telemetry, haptic, and video segments share
  one queue in send order, so a large video segment ahead of a control
  command delays the command by the video segment's full serialization
  time. There is no DSCP-EF-equivalent preemption.
* No adaptive bitrate: video is generated at a fixed rate regardless of
  link conditions (see naive_session.py), so a bandwidth-constrained
  failure grows an unbounded send queue (bufferbloat) instead of
  backing off.
"""
from __future__ import annotations

import random
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Optional

from ..core.scheduler import Scheduler
from .failures import FailureTimeline

MIN_BANDWIDTH_MBPS = 0.01
RTO_FLOOR_S = 0.2
RTO_RTT_MULTIPLIER = 4.0
RTO_CAP_S = 3.0


@dataclass
class NaiveSegment:
    send_time: float
    size_wire_bytes: int
    category: str  # "command" | "telemetry" | "haptic" | "video"
    seq: int
    direction: str  # "robot_to_operator" | "operator_to_robot"
    meta: dict = field(default_factory=dict)


DeliverCallback = Callable[[NaiveSegment, float], None]


class NaiveReliableLink:
    """One direction of a single ordered, reliable, unprioritized stream.

    Two independent stages, as in real TCP:
      1. Transmission -- bandwidth-limited, pipelined, one segment's
         service time after another regardless of loss.
      2. In-order delivery -- a segment only reaches `on_deliver` once
         it *and every segment queued ahead of it* have completed
         (including any retransmission), so a lost segment holds back
         everything behind it even though those later segments may
         individually have already crossed the wire.
    """

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
        self._to_transmit: deque[tuple[int, NaiveSegment]] = deque()
        self._next_local_idx = 0
        self._transmitting = False
        self._completed: dict[int, tuple[NaiveSegment, float]] = {}
        self._next_deliver_idx = 0
        self._last_delivered_time = 0.0
        self.queue_depth_samples: list[tuple[float, int]] = []

    def enqueue(self, seg: NaiveSegment) -> None:
        local_idx = self._next_local_idx
        self._next_local_idx += 1
        self._to_transmit.append((local_idx, seg))
        self.queue_depth_samples.append((self.scheduler.now, len(self._to_transmit)))
        if not self._transmitting:
            self._pump_transmit()

    def _pump_transmit(self) -> None:
        if not self._to_transmit:
            self._transmitting = False
            return
        self._transmitting = True
        local_idx, seg = self._to_transmit.popleft()
        self.queue_depth_samples.append((self.scheduler.now, len(self._to_transmit)))
        now = self.scheduler.now
        bw_mbps = max(
            self.timeline.effective_bandwidth_mbps(now, self.direction), MIN_BANDWIDTH_MBPS
        )
        service_time = (seg.size_wire_bytes * 8) / (bw_mbps * 1e6)
        self.scheduler.schedule(service_time, lambda: self._on_wire(local_idx, seg))

    def _on_wire(self, local_idx: int, seg: NaiveSegment) -> None:
        self._pump_transmit()  # next segment's send is not gated on this one's propagation/retries
        self._attempt_delivery(local_idx, seg, retry_count=0)

    def _attempt_delivery(self, local_idx: int, seg: NaiveSegment, retry_count: int) -> None:
        now = self.scheduler.now
        params = self.timeline.effective_params(now)
        if self.rng.random() < params.loss_prob:
            rtt_estimate_s = 2.0 * params.owd_ms / 1000.0
            rto = min(RTO_CAP_S, max(RTO_FLOOR_S, RTO_RTT_MULTIPLIER * rtt_estimate_s) * (2 ** retry_count))
            self.scheduler.schedule(rto, lambda: self._attempt_delivery(local_idx, seg, retry_count + 1))
            return
        jitter_ms = self.rng.gauss(0.0, params.jitter_std_ms)
        owd_ms = max(0.1, params.owd_ms + jitter_ms)
        arrival = now + owd_ms / 1000.0
        self.scheduler.schedule_at(arrival, lambda: self._on_arrived(local_idx, seg, arrival))

    def _on_arrived(self, local_idx: int, seg: NaiveSegment, arrival_time: float) -> None:
        self._completed[local_idx] = (seg, arrival_time)
        self._release_in_order()

    def _release_in_order(self) -> None:
        while self._next_deliver_idx in self._completed:
            seg, completion_time = self._completed.pop(self._next_deliver_idx)
            deliver_time = max(completion_time, self._last_delivered_time)
            self._last_delivered_time = deliver_time
            self._next_deliver_idx += 1
            self.on_deliver(seg, deliver_time)


class NaiveLink:
    """Both directions of the naive baseline path, sharing one FailureTimeline
    so it can be compared apples-to-apples against the QUIC-based Link
    driven by the same failure preset."""

    def __init__(self, scheduler: Scheduler, timeline: FailureTimeline, seed: int = 0) -> None:
        self.scheduler = scheduler
        self.timeline = timeline
        self._rng = random.Random(seed)
        self._deliver_cb: Optional[DeliverCallback] = None
        self.robot_to_operator = NaiveReliableLink(scheduler, timeline, "robot_to_operator", self._rng, self._dispatch)
        self.operator_to_robot = NaiveReliableLink(scheduler, timeline, "operator_to_robot", self._rng, self._dispatch)

    def set_deliver_callback(self, cb: DeliverCallback) -> None:
        self._deliver_cb = cb

    def _dispatch(self, seg: NaiveSegment, arrival: float) -> None:
        if self._deliver_cb is not None:
            self._deliver_cb(seg, arrival)

    def send(self, seg: NaiveSegment) -> None:
        link = self.robot_to_operator if seg.direction == "robot_to_operator" else self.operator_to_robot
        link.enqueue(seg)
