//! The QUIC side of topic forwarding: one connection's worth of state, used
//! the same way by `robot-edge` and `operator-console`.
//!
//! Latest-mode samples leave as datagrams through `pop_datagram`, which the
//! caller drains after its control datagrams, taking turns with video and
//! sensor data on the robot (see robot-edge's `feed_lossy`). Reliable
//! samples go out on one
//! unidirectional stream at `TOPIC_URGENCY`, below every other stream but
//! bulk objects. Both share one `RateCap` (`--zenoh-max-mbps`), so forwarded
//! traffic can't crowd out commands on a weak link: while it binds, latest
//! samples wait (and newer ones replace them) and reliable bytes wait in the
//! outbox's backlog, which holds about 2 s at the capped rate.

use std::collections::HashMap;
use std::time::Instant;

use roboprotocol_core::topic::{reliable_backlog, OutboxStats, RateCap, TopicError, TopicOutbox, TopicSample, TopicStreamReader};

use crate::Outgoing;

/// quiche stream urgency (lower first): E-Stop is 0, bulk objects 7.
pub const TOPIC_URGENCY: u8 = 6;

/// Cap tokens the reliable stream leaves while latest datagrams are
/// waiting: one full datagram's worth. Without it the stream took every
/// token as soon as any refilled, and a latest datagram (which needs room
/// for a whole one) never got through while a large reliable sample
/// drained: with real BabyROS nodes on localhost, 661 latest samples were
/// superseded during 300 KB grids.
const LATEST_RESERVE: usize = 1500;

/// Bytes handed to one `stream_send` call at most.
const CHUNK: usize = 16 * 1024;

pub struct TopicLink {
    outbox: TopicOutbox,
    /// `None`: no cap.
    cap: Option<RateCap>,
    send_stream: Option<u64>,
    priority_set: bool,
    /// Incoming reliable topic streams (normally just the peer's one).
    readers: HashMap<u64, TopicStreamReader>,
}

impl TopicLink {
    /// `max_mbps <= 0` means no cap. The reliable backlog is sized from the
    /// cap (`reliable_backlog`).
    pub fn new(max_mbps: f64) -> Self {
        Self { outbox: TopicOutbox::with_backlog(reliable_backlog(max_mbps)), cap: RateCap::new(max_mbps, Instant::now()), send_stream: None, priority_set: false, readers: HashMap::new() }
    }

    pub fn stats(&self) -> OutboxStats {
        self.outbox.stats()
    }

    pub fn push(&mut self, out: &Outgoing) {
        self.outbox.push(out.mode, &out.sample);
    }

    pub fn has_datagram(&self) -> bool {
        self.outbox.has_datagram()
    }

    /// The next latest-mode datagram, tagged, if one fits in `max_len` and
    /// the cap has room for a datagram that size.
    pub fn pop_datagram(&mut self, max_len: usize) -> Option<Vec<u8>> {
        if !self.outbox.has_datagram() {
            return None;
        }
        if let Some(cap) = &mut self.cap {
            if cap.available(Instant::now()) < max_len {
                return None;
            }
        }
        let d = self.outbox.pop_datagram(max_len)?;
        if let Some(cap) = &mut self.cap {
            cap.consume(d.len());
        }
        Some(d)
    }

    /// Writes waiting reliable bytes, at most `budget` if given, opening the
    /// stream with `allocate` the first time. Returns the bytes quiche took.
    pub fn pump_reliable(&mut self, conn: &mut quiche::Connection, allocate: impl FnOnce() -> u64, budget: Option<usize>) -> usize {
        if self.outbox.reliable_pending().is_empty() {
            return 0;
        }
        let reserve = if self.outbox.has_datagram() { LATEST_RESERVE } else { 0 };
        let capped = self.cap.as_mut().map(|c| c.available(Instant::now()).saturating_sub(reserve));
        let mut left = budget.unwrap_or(usize::MAX).min(capped.unwrap_or(usize::MAX));
        if left == 0 {
            return 0;
        }
        let stream_id = *self.send_stream.get_or_insert_with(allocate);
        let mut written = 0;
        while left > 0 {
            let pending = self.outbox.reliable_pending();
            if pending.is_empty() {
                break;
            }
            let end = pending.len().min(CHUNK).min(left);
            match conn.stream_send(stream_id, &pending[..end], false) {
                Ok(n) => {
                    if n > 0 && !self.priority_set {
                        // Only takes effect once the stream exists, i.e.
                        // after its first write.
                        let _ = conn.stream_priority(stream_id, TOPIC_URGENCY, false);
                        self.priority_set = true;
                    }
                    self.outbox.consume_reliable(n);
                    written += n;
                    left -= n;
                    if n < end {
                        break; // flow control: the rest waits for credit
                    }
                }
                Err(quiche::Error::Done) | Err(quiche::Error::StreamLimit) => break,
                Err(e) => {
                    tracing::warn!(error = ?e, stream_id, "topic stream_send failed; dropping the waiting reliable samples");
                    self.outbox.consume_reliable(usize::MAX);
                    break;
                }
            }
        }
        if let Some(cap) = &mut self.cap {
            cap.consume(written);
        }
        written
    }

    /// Whether `stream_id` is an incoming reliable topic stream this link is
    /// already reading.
    pub fn is_reading(&self, stream_id: u64) -> bool {
        self.readers.contains_key(&stream_id)
    }

    /// Feeds bytes read from an incoming reliable topic stream.
    pub fn on_stream_data(&mut self, stream_id: u64, data: &[u8], fin: bool) -> (Vec<TopicSample>, Vec<TopicError>) {
        let out = self.readers.entry(stream_id).or_default().push(data);
        if fin {
            self.readers.remove(&stream_id);
        }
        out
    }

    pub fn on_stream_reset(&mut self, stream_id: u64) {
        self.readers.remove(&stream_id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roboprotocol_core::topic::TopicMode;

    fn latest(key: &str, bytes: usize) -> Outgoing {
        Outgoing { mode: TopicMode::Latest, sample: TopicSample { key: key.into(), attachment: vec![], payload: vec![0; bytes] } }
    }

    #[test]
    fn the_cap_holds_back_latest_datagrams_until_it_refills() {
        // 0.1 Mbps: 12.5 KB/s, a 1,500-byte burst.
        let mut capped = TopicLink::new(0.1);
        let mut free = TopicLink::new(0.0);
        for i in 0..10 {
            capped.push(&latest(&format!("k/{i}"), 500));
            free.push(&latest(&format!("k/{i}"), 500));
        }
        let drain = |l: &mut TopicLink| std::iter::from_fn(|| l.pop_datagram(1200)).count();
        assert_eq!(drain(&mut capped), 1, "one datagram fits the burst, then the cap needs room for a full one");
        assert!(capped.has_datagram(), "the rest wait rather than being dropped");
        assert_eq!(drain(&mut free), 10);
        std::thread::sleep(std::time::Duration::from_millis(120)); // refills to the 1,500-byte burst
        assert_eq!(drain(&mut capped), 1);
    }
}
