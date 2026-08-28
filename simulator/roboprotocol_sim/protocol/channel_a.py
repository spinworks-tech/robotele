"""Channel A: video feed datagram generation with PIR-style near-constant
bitrate, a simplified GCC-like adaptive bitrate controller (NFR-2.1), and
inline FlexFEC (NFR-3.1, RFC 8627 proxy in fec.py).

The bitrate controller's delay-gradient feedback is driven by the
*actually observed* end-to-end delay of delivered datagrams (arrival
time minus send time), not the FailureTimeline's propagation-only OWD
parameter. This matters: propagation OWD alone never moves when a
bandwidth-cap failure hits -- what balloons is *queueing* delay at the
link's server (network/link.py), which only shows up in a packet's
real observed transit time. A controller keyed off propagation OWD
would never back off under pure bandwidth pressure, which would be
both wrong and would defeat the entire point of testing that failure
mode.
"""
from __future__ import annotations

import itertools
import random
from collections import deque
from dataclasses import dataclass

from ..core.scheduler import Scheduler
from ..network.failures import FailureTimeline
from ..network.link import Datagram, Link, Priority
from . import sizing
from .fec import DEFAULT_BLOCK_SIZE, DEFAULT_REDUNDANCY_RATIO, FlexFecTracker

MIN_BITRATE_BPS = 2_000_000
MAX_BITRATE_BPS = 20_000_000
GCC_UPDATE_INTERVAL_S = 0.2
GCC_DELAY_GRADIENT_THRESHOLD_MS_PER_S = 5.0
GCC_INCREASE_STEP_BPS = 250_000.0
GCC_DECREASE_FACTOR = 0.85

VIDEO_DATAGRAM_APP_BYTES = sizing.SINGLE_DATAGRAM_BUDGET_BYTES

# Fixed encode/decode/jitter-buffer latency budget added on top of network
# OWD when estimating glass-to-glass delay (NFR-1.1). Not modelled pixel-
# accurately -- see SPEC.md.
FIXED_ENCODE_DECODE_BUDGET_MS = 12.0


class GccLikeController:
    """Simplified additive-increase/multiplicative-decrease bitrate controller."""

    def __init__(
        self,
        initial_bps: float,
        min_bps: float = MIN_BITRATE_BPS,
        max_bps: float = MAX_BITRATE_BPS,
    ) -> None:
        self.min_bps = min_bps
        self.max_bps = max_bps
        self.target_bps = max(min_bps, min(max_bps, initial_bps))
        self._owd_history: deque[tuple[float, float]] = deque(maxlen=6)

    def update(self, t: float, owd_ms: float) -> float:
        self._owd_history.append((t, owd_ms))
        trend = self._delay_gradient_ms_per_s()
        if trend > GCC_DELAY_GRADIENT_THRESHOLD_MS_PER_S:
            self.target_bps = max(self.min_bps, self.target_bps * GCC_DECREASE_FACTOR)
        else:
            self.target_bps = min(self.max_bps, self.target_bps + GCC_INCREASE_STEP_BPS)
        return self.target_bps

    def _delay_gradient_ms_per_s(self) -> float:
        if len(self._owd_history) < 2:
            return 0.0
        t0, o0 = self._owd_history[0]
        t1, o1 = self._owd_history[-1]
        if t1 <= t0:
            return 0.0
        return (o1 - o0) / (t1 - t0)


@dataclass
class VideoStreamConfig:
    name: str
    fps: int
    initial_bps: float
    min_bps: float = MIN_BITRATE_BPS
    max_bps: float = MAX_BITRATE_BPS
    fec_block_size: int = DEFAULT_BLOCK_SIZE
    fec_redundancy_ratio: float = DEFAULT_REDUNDANCY_RATIO


class VideoStream:
    """One camera feed (wrist or head/fixed) on Channel A, robot -> operator."""

    def __init__(
        self,
        config: VideoStreamConfig,
        scheduler: Scheduler,
        link: Link,
        timeline: FailureTimeline,
        metrics,
        seed: int = 0,
    ) -> None:
        self.config = config
        self.scheduler = scheduler
        self.link = link
        self.timeline = timeline
        self.metrics = metrics
        self.direction = "robot_to_operator"

        self.controller = GccLikeController(config.initial_bps, config.min_bps, config.max_bps)
        self.fec_tracker = FlexFecTracker()
        self._n_redundant = max(1, round(config.fec_block_size * config.fec_redundancy_ratio))
        self._rng = random.Random(seed)
        self._seq = itertools.count()
        self._block_id = 0
        self._block_sent = 0
        self._observed_delay_ms: deque[float] = deque(maxlen=20)

        self._schedule_frame()
        self._schedule_gcc_update()

    # -- sending --------------------------------------------------------
    def _schedule_frame(self) -> None:
        self.scheduler.schedule(1.0 / self.config.fps, self._send_frame)

    def _send_frame(self) -> None:
        target_bps = self.controller.target_bps
        # target_bps is the GCC-like controller's cap on total *wire*
        # throughput for this stream (what NFR-2.1's 2-20Mbps envelope
        # is bounding). FlexFEC parity datagrams add ~fec_redundancy_ratio
        # on top of the raw frame data at packetization time (see
        # _send_datagram below), so the encoded-frame data rate is
        # backed off accordingly to keep data+parity within target_bps
        # -- otherwise the controller would silently let actual link
        # usage run ~30% over its own adapted cap.
        data_rate_bps = target_bps / (1.0 + self.config.fec_redundancy_ratio)
        frame_bytes = max(
            1.0, (data_rate_bps / 8.0 / self.config.fps) * (1.0 + self._rng.gauss(0.0, 0.05))
        )
        self.metrics.log_video_bitrate(self.scheduler.now, self.config.name, target_bps)
        remaining = frame_bytes
        while remaining > 0:
            chunk = min(remaining, VIDEO_DATAGRAM_APP_BYTES)
            self._send_datagram(chunk, "video")
            remaining -= chunk
        self._schedule_frame()

    def _send_datagram(self, app_bytes: float, category: str) -> None:
        app_bytes_i = int(max(1, round(app_bytes)))
        dgram = Datagram(
            send_time=self.scheduler.now,
            size_wire_bytes=sizing.wire_bytes(app_bytes_i),
            size_app_bytes=app_bytes_i,
            priority=Priority.MEDIA,
            channel="A",
            category=category,
            seq=next(self._seq),
            direction=self.direction,
            meta={
                "stream": self.config.name,
                "block_id": self._block_id,
                "block_size": self.config.fec_block_size,
                "n_redundant": self._n_redundant,
            },
        )
        self.metrics.log_datagram_sent(dgram)
        self.link.send(dgram)
        if category == "video":
            self._block_sent += 1
            if self._block_sent >= self.config.fec_block_size:
                for _ in range(self._n_redundant):
                    self._send_parity(app_bytes_i)
                self._block_id += 1
                self._block_sent = 0

    def _send_parity(self, app_bytes_i: int) -> None:
        dgram = Datagram(
            send_time=self.scheduler.now,
            size_wire_bytes=sizing.wire_bytes(app_bytes_i),
            size_app_bytes=app_bytes_i,
            priority=Priority.MEDIA,
            channel="A",
            category="fec_parity",
            seq=next(self._seq),
            direction=self.direction,
            meta={
                "stream": self.config.name,
                "block_id": self._block_id,
                "block_size": self.config.fec_block_size,
                "n_redundant": self._n_redundant,
            },
        )
        self.metrics.log_datagram_sent(dgram)
        self.link.send(dgram)

    # -- adaptation -------------------------------------------------------
    def _schedule_gcc_update(self) -> None:
        self.scheduler.schedule(GCC_UPDATE_INTERVAL_S, self._gcc_update)

    def _gcc_update(self) -> None:
        if self._observed_delay_ms:
            # Mean of recent real observed delays (queueing + propagation +
            # jitter), not just the profile's propagation-only OWD.
            owd_ms = sum(self._observed_delay_ms) / len(self._observed_delay_ms)
        else:
            # No deliveries yet to learn from -- fall back to the profile's
            # propagation OWD as a startup estimate only.
            owd_ms = self.timeline.effective_params(self.scheduler.now).owd_ms
        self.controller.update(self.scheduler.now, owd_ms)
        self._schedule_gcc_update()

    # -- delivery feedback --------------------------------------------------
    def on_delivery_result(self, datagram: Datagram, delivered: bool) -> None:
        if datagram.meta.get("stream") != self.config.name:
            return
        result = self.fec_tracker.record(
            block_id=datagram.meta["block_id"],
            block_size=datagram.meta["block_size"],
            n_redundant=datagram.meta["n_redundant"],
            lost=not delivered,
            t=self.scheduler.now,
        )
        if result is not None:
            self.metrics.log_fec_block(self.config.name, result)

        if delivered:
            owd_ms = (self.scheduler.now - datagram.send_time) * 1000.0
            self._observed_delay_ms.append(owd_ms)
            glass_to_glass_ms = owd_ms + FIXED_ENCODE_DECODE_BUDGET_MS
            self.metrics.log_glass_to_glass(self.config.name, self.scheduler.now, glass_to_glass_ms)
