"""Per-run metrics collection: every datagram send/delivery outcome, RTT
samples, video bitrate decisions, and FlexFEC block verdicts, converted
to pandas DataFrames at the end of a run for reporting/plotting.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import pandas as pd

from ..network.link import Datagram
from ..protocol.fec import FecBlockResult


@dataclass
class DatagramRecord:
    seq: int
    send_time: float
    channel: str
    category: str
    direction: str
    size_app_bytes: int
    size_wire_bytes: int
    delivered: Optional[bool] = None
    arrival_time: Optional[float] = None
    delay_ms: Optional[float] = None


class MetricsCollector:
    def __init__(self) -> None:
        self._datagram_records: list[DatagramRecord] = []
        self._pending: dict[int, DatagramRecord] = {}
        self.rtt_samples: list[tuple[float, float]] = []
        self.video_bitrate_log: list[tuple[float, str, float]] = []
        self.fec_results: list[tuple[str, FecBlockResult]] = []
        self.glass_to_glass_log: list[tuple[str, float, float]] = []

    # -- writers ------------------------------------------------------------
    def log_datagram_sent(self, dgram: Datagram) -> None:
        rec = DatagramRecord(
            seq=dgram.seq,
            send_time=dgram.send_time,
            channel=dgram.channel,
            category=dgram.category,
            direction=dgram.direction,
            size_app_bytes=dgram.size_app_bytes,
            size_wire_bytes=dgram.size_wire_bytes,
        )
        self._datagram_records.append(rec)
        self._pending[id(dgram)] = rec

    def log_datagram_delivered(self, dgram: Datagram, delivered: bool, arrival_time: Optional[float]) -> None:
        rec = self._pending.pop(id(dgram), None)
        if rec is None:
            return
        rec.delivered = delivered
        rec.arrival_time = arrival_time
        if delivered and arrival_time is not None:
            rec.delay_ms = (arrival_time - rec.send_time) * 1000.0

    def log_rtt_sample(self, t: float, rtt_ms: float) -> None:
        self.rtt_samples.append((t, rtt_ms))

    def log_video_bitrate(self, t: float, stream: str, target_bps: float) -> None:
        self.video_bitrate_log.append((t, stream, target_bps))

    def log_fec_block(self, stream: str, result: FecBlockResult) -> None:
        self.fec_results.append((stream, result))

    def log_glass_to_glass(self, stream: str, t: float, glass_to_glass_ms: float) -> None:
        self.glass_to_glass_log.append((stream, t, glass_to_glass_ms))

    # -- readers --------------------------------------------------------------
    def to_dataframes(self) -> dict[str, pd.DataFrame]:
        dgram_df = pd.DataFrame(
            [
                dict(
                    seq=r.seq, send_time=r.send_time, channel=r.channel, category=r.category,
                    direction=r.direction, size_app_bytes=r.size_app_bytes,
                    size_wire_bytes=r.size_wire_bytes, delivered=r.delivered,
                    arrival_time=r.arrival_time, delay_ms=r.delay_ms,
                )
                for r in self._datagram_records
            ]
        )
        rtt_df = pd.DataFrame(self.rtt_samples, columns=["t", "rtt_ms"])
        video_df = pd.DataFrame(self.video_bitrate_log, columns=["t", "stream", "target_bps"])
        fec_df = pd.DataFrame(
            [
                dict(
                    stream=s, t=r.t, block_id=r.block_id, block_size=r.block_size,
                    n_redundant=r.n_redundant, n_lost=r.n_lost, recovered=r.recovered,
                )
                for s, r in self.fec_results
            ]
        )
        g2g_df = pd.DataFrame(self.glass_to_glass_log, columns=["stream", "t", "glass_to_glass_ms"])
        return {
            "datagrams": dgram_df,
            "rtt": rtt_df,
            "video_bitrate": video_df,
            "fec": fec_df,
            "glass_to_glass": g2g_df,
        }
