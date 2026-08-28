"""Minimal heapq-based discrete-event scheduler.

No external dependency (no simpy) so the simulator runs with only the
stdlib plus numpy/pandas/matplotlib for analysis/reporting.
"""
from __future__ import annotations

import heapq
import itertools
from dataclasses import dataclass, field
from typing import Callable


@dataclass(order=True)
class _Event:
    time: float
    seq: int
    callback: Callable[[], None] = field(compare=False)
    cancelled: bool = field(compare=False, default=False)


class EventHandle:
    """Opaque handle returned by schedule(); allows cancellation."""

    __slots__ = ("_event",)

    def __init__(self, event: _Event):
        self._event = event

    def cancel(self) -> None:
        self._event.cancelled = True


class Scheduler:
    """A simple, deterministic discrete-event simulation clock.

    Callbacks are invoked in strictly increasing time order; ties are
    broken by insertion order (seq) so a fixed random seed makes runs
    fully reproducible.
    """

    def __init__(self) -> None:
        self._heap: list[_Event] = []
        self._seq = itertools.count()
        self.now: float = 0.0

    def schedule_at(self, time: float, callback: Callable[[], None]) -> EventHandle:
        if time < self.now:
            raise ValueError(f"cannot schedule in the past: {time} < {self.now}")
        ev = _Event(time, next(self._seq), callback)
        heapq.heappush(self._heap, ev)
        return EventHandle(ev)

    def schedule(self, delay: float, callback: Callable[[], None]) -> EventHandle:
        if delay < 0:
            raise ValueError("delay must be >= 0")
        return self.schedule_at(self.now + delay, callback)

    def run(self, until: float) -> None:
        while self._heap and self._heap[0].time <= until:
            ev = heapq.heappop(self._heap)
            if ev.cancelled:
                continue
            self.now = ev.time
            ev.callback()
        self.now = max(self.now, until)
