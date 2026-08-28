//! Bounded, drop-oldest-under-pressure ring buffer shared between the
//! real-time enqueue path and a category's dedicated writer thread
//! (DESIGN.md §10.3). Plain `std::sync` -- no async runtime -- so the
//! enqueue side is a cheap, uncontended-fast-path push regardless of
//! which runtime (or none) the caller is running under.

use std::collections::VecDeque;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Condvar, Mutex};
use std::time::Duration;

use crate::Record;

pub struct CategoryState {
    ring: Mutex<VecDeque<Record>>,
    cond: Condvar,
    capacity: usize,
    pub active: AtomicBool,
    dropped: AtomicU64,
    bytes_enqueued: AtomicU64,
    pub shutdown: AtomicBool,
}

impl CategoryState {
    pub fn new(capacity: usize, start_active: bool) -> Self {
        Self {
            ring: Mutex::new(VecDeque::with_capacity(capacity.min(1024))),
            cond: Condvar::new(),
            capacity: capacity.max(1),
            active: AtomicBool::new(start_active),
            dropped: AtomicU64::new(0),
            bytes_enqueued: AtomicU64::new(0),
            shutdown: AtomicBool::new(false),
        }
    }

    /// Non-blocking push. Drops the oldest queued record if the ring is
    /// already at capacity -- never applies backpressure to the caller
    /// (FR-9.3).
    pub fn push(&self, record: Record) {
        self.bytes_enqueued.fetch_add(record.payload.len() as u64, Ordering::Relaxed);
        let mut ring = self.ring.lock().unwrap();
        if ring.len() >= self.capacity {
            ring.pop_front();
            self.dropped.fetch_add(1, Ordering::Relaxed);
        }
        ring.push_back(record);
        drop(ring);
        self.cond.notify_one();
    }

    /// Blocks (bounded by `timeout`, so periodic flush/rotation checks
    /// still happen during a lull) until something is queued or shutdown
    /// is requested, then hands back everything currently queued in one
    /// shot. The lock is held only long enough to swap the deque out,
    /// never across any I/O the caller does with the result.
    pub fn drain(&self, timeout: Duration) -> VecDeque<Record> {
        let ring = self.ring.lock().unwrap();
        let (mut ring, _timed_out) = self
            .cond
            .wait_timeout_while(ring, timeout, |r| r.is_empty() && !self.shutdown.load(Ordering::Relaxed))
            .unwrap();
        std::mem::take(&mut ring)
    }

    /// `(bytes_enqueued, records_dropped, active)`.
    pub fn stats(&self) -> (u64, u64, bool) {
        (self.bytes_enqueued.load(Ordering::Relaxed), self.dropped.load(Ordering::Relaxed), self.active.load(Ordering::Relaxed))
    }

    pub fn wake(&self) {
        self.cond.notify_one();
    }
}
