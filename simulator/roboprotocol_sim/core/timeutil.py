"""NTP-style timestamp / RTT / OWD helpers (REQUIREMENTS.md SR-3.1).

Every Channel B datagram carries a 64-bit send timestamp. The receiver
echoes it back so both ends can compute a sliding-window RTT/OWD. Over
a WAN path, RTT is preferred over OWD to avoid clock-drift error
between operator and robot clocks (SR-3.1) -- the simulator gives both
ends the same simulation clock, so OWD is exact here, but RTT is still
what the safety-tier state machine keys off, matching the spec.
"""
from __future__ import annotations

from collections import deque


class SlidingRTT:
    """Sliding-window RTT/OWD tracker fed by (send_time, recv_time) samples."""

    def __init__(self, window: int = 32) -> None:
        self._rtt_samples: deque[float] = deque(maxlen=window)
        self._owd_samples: deque[float] = deque(maxlen=window)

    def add_owd_sample(self, owd_seconds: float) -> None:
        self._owd_samples.append(owd_seconds)

    def add_rtt_sample(self, rtt_seconds: float) -> None:
        self._rtt_samples.append(rtt_seconds)

    @property
    def rtt_ms(self) -> float | None:
        if not self._rtt_samples:
            return None
        return 1000.0 * sum(self._rtt_samples) / len(self._rtt_samples)

    @property
    def owd_ms(self) -> float | None:
        if not self._owd_samples:
            return None
        return 1000.0 * sum(self._owd_samples) / len(self._owd_samples)
