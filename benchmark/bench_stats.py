"""Shared latency/throughput reporting helpers for the Part 4 protocol
comparison scripts (mqtt_bench.py, zenoh_bench.py) -- see BENCHMARK.md.

Kept dependency-free (stdlib `statistics` only) so it matches the rest of
`benchmark/*.py`, which intentionally avoids a pip install just to run a
capture/analysis pass.
"""
from __future__ import annotations

import statistics
from dataclasses import dataclass


@dataclass
class LatencyReport:
    n: int
    n_trimmed: int
    median_rtt_us: float
    median_one_way_us: float
    p95_rtt_us: float
    p99_rtt_us: float
    min_rtt_us: float
    max_rtt_us: float

    def __str__(self) -> str:
        return (
            f"n={self.n} (trimmed to {self.n_trimmed} after 1st/99th pct cut)  "
            f"median RTT={self.median_rtt_us:.1f}us "
            f"(one-way={self.median_one_way_us:.1f}us)  "
            f"p95={self.p95_rtt_us:.1f}us  p99={self.p99_rtt_us:.1f}us  "
            f"min={self.min_rtt_us:.1f}us  max={self.max_rtt_us:.1f}us"
        )


def summarize_latency(rtt_seconds: list[float]) -> LatencyReport:
    """Trim to the 1st/99th percentile band and report median/p95/p99,
    matching the methodology in the Zenoh vs MQTT/Kafka/DDS blog post this
    suite is meant to be comparable with: half the median RTT is the
    reported one-way latency, outliers outside the 1st/99th percentile are
    excluded before computing statistics.
    """
    if not rtt_seconds:
        raise ValueError("no samples collected -- check the responder is running "
                          "and reachable")
    ordered = sorted(rtt_seconds)
    n = len(ordered)
    lo = ordered[int(n * 0.01)]
    hi = ordered[min(n - 1, int(n * 0.99))]
    trimmed = [s for s in ordered if lo <= s <= hi] or ordered
    us = [s * 1e6 for s in trimmed]
    median = statistics.median(us)
    return LatencyReport(
        n=n,
        n_trimmed=len(trimmed),
        median_rtt_us=median,
        median_one_way_us=median / 2.0,
        p95_rtt_us=_percentile(us, 0.95),
        p99_rtt_us=_percentile(us, 0.99),
        min_rtt_us=min(us),
        max_rtt_us=max(us),
    )


def _percentile(sorted_or_not: list[float], pct: float) -> float:
    data = sorted(sorted_or_not)
    if len(data) == 1:
        return data[0]
    idx = pct * (len(data) - 1)
    lo_idx = int(idx)
    hi_idx = min(lo_idx + 1, len(data) - 1)
    frac = idx - lo_idx
    return data[lo_idx] + (data[hi_idx] - data[lo_idx]) * frac


@dataclass
class ThroughputReport:
    messages: int
    bytes_total: int
    elapsed_s: float

    @property
    def msg_per_s(self) -> float:
        return self.messages / self.elapsed_s if self.elapsed_s > 0 else 0.0

    @property
    def bits_per_s(self) -> float:
        return (self.bytes_total * 8) / self.elapsed_s if self.elapsed_s > 0 else 0.0

    def __str__(self) -> str:
        return (
            f"{self.messages} msgs, {self.bytes_total} bytes in "
            f"{self.elapsed_s:.2f}s -> {self.msg_per_s:.1f} msg/s, "
            f"{self.bits_per_s / 1e6:.2f} Mbps"
        )
