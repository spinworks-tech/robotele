"""Metrics for the naive WebSocket-over-TCP baseline session."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import pandas as pd

from ..network.naive_link import NaiveSegment


@dataclass
class NaiveSegmentRecord:
    seq: int
    send_time: float
    category: str
    direction: str
    size_wire_bytes: int
    delivered_at: Optional[float] = None
    delay_ms: Optional[float] = None


class NaiveMetricsCollector:
    def __init__(self) -> None:
        self._records: list[NaiveSegmentRecord] = []
        self._pending: dict[int, NaiveSegmentRecord] = {}
        self.liveness_events: list[tuple[float, float]] = []  # (t, silence_ms)

    def log_naive_sent(self, seg: NaiveSegment, direction: str) -> None:
        rec = NaiveSegmentRecord(
            seq=seg.seq, send_time=seg.send_time, category=seg.category,
            direction=direction, size_wire_bytes=seg.size_wire_bytes,
        )
        self._records.append(rec)
        self._pending[id(seg)] = rec

    def log_naive_delivered(self, seg: NaiveSegment, direction: str, arrival_time: float) -> None:
        rec = self._pending.pop(id(seg), None)
        if rec is None:
            return
        rec.delivered_at = arrival_time
        rec.delay_ms = (arrival_time - rec.send_time) * 1000.0

    def log_naive_dead_detected(self, t: float, silence_ms: float) -> None:
        self.liveness_events.append((t, silence_ms))

    def to_dataframes(self) -> dict[str, pd.DataFrame]:
        seg_df = pd.DataFrame(
            [
                dict(
                    seq=r.seq, send_time=r.send_time, category=r.category, direction=r.direction,
                    size_wire_bytes=r.size_wire_bytes, delivered_at=r.delivered_at, delay_ms=r.delay_ms,
                )
                for r in self._records
            ]
        )
        liveness_df = pd.DataFrame(self.liveness_events, columns=["t", "silence_ms"])
        return {"segments": seg_df, "liveness": liveness_df}
