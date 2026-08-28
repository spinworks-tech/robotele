"""Simplified FlexFEC (RFC 8627) recovery model for Channel A (NFR-3.1).

NFR-3.1: "inline FlexFEC capable of recovering up to 30% packet loss
without retransmission requests." This module implements a
Reed-Solomon-style erasure-code proxy: a block of `block_size` video
datagrams gets `n_redundant` parity datagrams appended; the whole
block is recoverable iff the number of lost datagrams within it is
<= n_redundant. This is not a bit-accurate FEC implementation, just
enough to demonstrate the claimed "recovers up to 30% loss" boundary
under simulated packet loss.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

DEFAULT_BLOCK_SIZE = 10
DEFAULT_REDUNDANCY_RATIO = 0.30  # covers "up to 30% loss" per NFR-3.1


@dataclass
class FecBlockResult:
    t: float
    block_id: int
    block_size: int
    n_redundant: int
    n_lost: int
    recovered: bool


class FlexFecTracker:
    """Tallies per-block delivery outcomes and finalizes recovery verdicts.

    Datagrams may be reported out of order (loss is known immediately;
    successful delivery is known only once the propagation delay
    elapses), so blocks are finalized once every datagram belonging to
    them has reported exactly once, regardless of order.
    """

    def __init__(self) -> None:
        self._blocks: dict[int, dict] = {}

    def record(
        self, block_id: int, block_size: int, n_redundant: int, lost: bool, t: float
    ) -> Optional[FecBlockResult]:
        state = self._blocks.setdefault(block_id, {"sent": 0, "lost": 0})
        state["sent"] += 1
        if lost:
            state["lost"] += 1
        total = block_size + n_redundant
        if state["sent"] >= total:
            del self._blocks[block_id]
            recovered = state["lost"] <= n_redundant
            return FecBlockResult(t, block_id, block_size, n_redundant, state["lost"], recovered)
        return None
