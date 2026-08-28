"""Traffic generator for the naive WebSocket-over-TCP baseline.

Mirrors the same command/telemetry/haptic/video traffic the RoboProtocol
ChannelB/ChannelA generators produce for a given ScenarioConfig, but the
way a typical from-scratch teleop demo actually builds it:

* Full float32 fields, no quantization tiers, no per-category budget or
  body-region splitting -- a naive implementation just serializes
  whatever the message is (e.g. JSON or a simple struct) and sends it.
* No anti-replay/authentication framing (no NTP timestamp, sequence
  number, or MAC) -- just a minimal length-prefixed frame.
* Video at a fixed bitrate, no FlexFEC, no GCC-like adaptation.
* Liveness is a coarse periodic check (a typical WS ping/pong interval),
  not a dedicated hardware watchdog: detecting a dead connection can
  take multiple seconds, versus RoboProtocol's 200-500ms watchdog
  (SR-4.1).
"""
from __future__ import annotations

import itertools
import random
from typing import Optional

from ..core.scheduler import Scheduler
from ..network.naive_link import NaiveSegment

NAIVE_APP_FRAME_OVERHEAD_BYTES = 16  # minimal length-prefixed struct framing, no auth/anti-replay
NAIVE_WIRE_OVERHEAD_BYTES = 46  # TCP(20) + IPv4(20) + WebSocket frame header(~6)
FLOAT32_BYTES = 4

# Typical WS client heartbeat interval and "declare dead after N missed
# heartbeats" reconnect policy -- orders of magnitude coarser than
# RoboProtocol's dedicated hardware watchdog.
LIVENESS_CHECK_INTERVAL_S = 5.0
LIVENESS_TIMEOUT_S = 10.0

# Extra one-time cost a naive client pays on a network handover that
# RoboProtocol's QUIC Connection Migration avoids: detecting the dead
# TCP socket, then a fresh TCP handshake + WebSocket upgrade.
NAIVE_RECONNECT_COST_S = 1.5


def naive_wire_bytes(num_float_fields: int) -> int:
    return num_float_fields * FLOAT32_BYTES + NAIVE_APP_FRAME_OVERHEAD_BYTES + NAIVE_WIRE_OVERHEAD_BYTES


class NaiveWebSocketSession:
    def __init__(self, scenario, scheduler: Scheduler, link, metrics, seed: int = 0) -> None:
        self.scenario = scenario
        self.scheduler = scheduler
        self.link = link
        self.metrics = metrics
        self._rng = random.Random(seed)
        self._seq = itertools.count()

        self._last_received_from_operator = 0.0
        self.connection_declared_dead = False
        self.dead_detected_at: Optional[float] = None
        self.recovered_at: Optional[float] = None

        self._schedule_tick()
        for vs_config in scenario.video_streams:
            self._schedule_video(vs_config)
        self._schedule_liveness_check()

    # -- Channel-B-equivalent traffic (one shared, unprioritized stream) ----
    def _schedule_tick(self) -> None:
        self.scheduler.schedule(1.0 / self.scenario.control_hz, self._tick)

    def _tick(self) -> None:
        self._send("operator_to_robot", "command", naive_wire_bytes(self.scenario.command_fields))
        self._send("robot_to_operator", "telemetry", naive_wire_bytes(self.scenario.telemetry_fields))
        self._send("robot_to_operator", "haptic", naive_wire_bytes(self.scenario.haptic_fields))
        self._schedule_tick()

    # -- Channel-A-equivalent traffic: fixed bitrate, no FEC, no adaptation --
    def _schedule_video(self, vs_config) -> None:
        self.scheduler.schedule(1.0 / vs_config.fps, lambda: self._send_video_frame(vs_config))

    def _send_video_frame(self, vs_config) -> None:
        frame_bytes = max(1.0, (vs_config.initial_bps / 8.0 / vs_config.fps) * (1.0 + self._rng.gauss(0.0, 0.05)))
        self._send(
            "robot_to_operator", "video",
            int(frame_bytes) + NAIVE_WIRE_OVERHEAD_BYTES,
            meta={"stream": vs_config.name},
        )
        self.scheduler.schedule(1.0 / vs_config.fps, lambda: self._send_video_frame(vs_config))

    def _send(self, direction: str, category: str, size_wire_bytes: int, meta: Optional[dict] = None) -> None:
        seg = NaiveSegment(
            send_time=self.scheduler.now,
            size_wire_bytes=size_wire_bytes,
            category=category,
            seq=next(self._seq),
            direction=direction,
            meta=meta or {},
        )
        self.metrics.log_naive_sent(seg, direction)
        self.link.send(seg)

    # -- delivery feedback / coarse liveness detection ------------------------
    def on_delivered(self, seg: NaiveSegment, arrival_time: float) -> None:
        self.metrics.log_naive_delivered(seg, seg.direction, arrival_time)
        if seg.direction == "operator_to_robot":
            self._last_received_from_operator = arrival_time
            if self.connection_declared_dead:
                self.connection_declared_dead = False
                self.recovered_at = arrival_time

    def _schedule_liveness_check(self) -> None:
        self.scheduler.schedule(LIVENESS_CHECK_INTERVAL_S, self._liveness_check)

    def _liveness_check(self) -> None:
        if not self.connection_declared_dead:
            silence_s = self.scheduler.now - self._last_received_from_operator
            if silence_s > LIVENESS_TIMEOUT_S:
                self.connection_declared_dead = True
                self.dead_detected_at = self.scheduler.now
                self.metrics.log_naive_dead_detected(self.scheduler.now, silence_s * 1000.0)
        self._schedule_liveness_check()
