//! The QUIC side of topic forwarding: one connection's worth of state, used
//! the same way by `robot-edge` and `operator-console`.
//!
//! Latest-mode samples leave as datagrams through `pop_datagram`, which the
//! caller drains only after its own control and media datagrams, so they
//! never queue ahead of those. Reliable samples go out on one
//! unidirectional stream at `TOPIC_URGENCY`, below every other stream but
//! bulk objects.

use std::collections::HashMap;

use roboprotocol_core::topic::{OutboxStats, TopicError, TopicOutbox, TopicSample, TopicStreamReader};

use crate::Outgoing;

/// quiche stream urgency (lower first): E-Stop is 0, bulk objects 7.
pub const TOPIC_URGENCY: u8 = 6;

/// Bytes handed to one `stream_send` call at most.
const CHUNK: usize = 16 * 1024;

#[derive(Default)]
pub struct TopicLink {
    outbox: TopicOutbox,
    send_stream: Option<u64>,
    priority_set: bool,
    /// Incoming reliable topic streams (normally just the peer's one).
    readers: HashMap<u64, TopicStreamReader>,
}

impl TopicLink {
    pub fn new() -> Self {
        Self::default()
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

    /// The next latest-mode datagram, tagged, if one fits in `max_len`.
    pub fn pop_datagram(&mut self, max_len: usize) -> Option<Vec<u8>> {
        self.outbox.pop_datagram(max_len)
    }

    /// Writes waiting reliable bytes, at most `budget` if given, opening the
    /// stream with `allocate` the first time. Returns the bytes quiche took.
    pub fn pump_reliable(&mut self, conn: &mut quiche::Connection, allocate: impl FnOnce() -> u64, budget: Option<usize>) -> usize {
        if self.outbox.reliable_pending().is_empty() {
            return 0;
        }
        let stream_id = *self.send_stream.get_or_insert_with(allocate);
        let mut left = budget.unwrap_or(usize::MAX);
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
