//! Sends bulk objects (`roboprotocol_core::bulk`, docs/13 "Bulk objects"):
//! one server-initiated unidirectional stream per object, at the lowest
//! stream priority.
//!
//! A version that has started sending always finishes; a newer version waits
//! behind it, and a still-newer one replaces the waiting version (which never
//! started). Doc 13 first proposed resetting the in-flight stream whenever a
//! newer version appears, but that starves: with 600 KB maps every 0.5 s on a
//! 5 Mbps link, all 30 versions were reset in flight and none arrived. This
//! way every delivered object is at most about one transfer time old, and
//! something always gets through.
//!
//! Writing is paced by `pump`, called on every flush: quiche accepts stream
//! data only up to the peer's flow-control window, so a large object goes
//! out over many calls. quiche builds each packet with datagrams before
//! stream data, so bulk bytes never sit in front of a control datagram in
//! the same packet; when `--lossy-rate-control` is on, `pump` is also given
//! a byte budget so bulk shares (and comes last in) the rate cap.

use std::collections::BTreeMap;

use roboprotocol_core::bulk::{BulkHeader, BulkKind};

/// quiche stream urgency 0..=7, lower first (RFC 9218). E-Stop is 0; bulk is
/// the least urgent thing on the connection. Non-incremental: quiche sends
/// one bulk stream at a time, oldest first, so each object completes before
/// the next starts. Incremental (round-robin) let a dozen maps progress in
/// parallel on a 5 Mbps link, and none finished in 15 s.
pub const BULK_URGENCY: u8 = 7;

/// Bytes handed to one `stream_send` call at most.
const CHUNK: usize = 16 * 1024;

#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct BulkSendStats {
    pub published: u64,
    pub completed: u64,
    /// Versions replaced while waiting, never sent.
    pub superseded: u64,
}

struct Outgoing {
    stream_id: u64,
    data: Vec<u8>,
    offset: usize,
}

pub struct BulkSender {
    /// Next server-initiated unidirectional stream: 3, 7, 11, ...
    next_stream_id: u64,
    /// Per (kind, source): the version being sent, if any.
    active: BTreeMap<(u8, u8), Outgoing>,
    /// Per (kind, source): the newest version waiting for the one in flight.
    waiting: BTreeMap<(u8, u8), Vec<u8>>,
    stats: BulkSendStats,
}

impl Default for BulkSender {
    fn default() -> Self {
        Self { next_stream_id: 3, active: BTreeMap::new(), waiting: BTreeMap::new(), stats: BulkSendStats::default() }
    }
}

impl BulkSender {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stats(&self) -> BulkSendStats {
        self.stats
    }

    /// Bytes still waiting to be written, across all objects.
    pub fn pending_bytes(&self) -> usize {
        self.active.values().map(|o| o.data.len() - o.offset).sum::<usize>() + self.waiting.values().map(Vec::len).sum::<usize>()
    }

    /// Queues a new version of an object: sent at once if nothing of the
    /// same (kind, source) is in flight, otherwise kept as the next version
    /// to send, replacing any older waiting one.
    pub fn publish(&mut self, conn: &mut quiche::Connection, header: BulkHeader, payload: &[u8]) {
        let key = (header.kind.to_u8(), header.source_id);
        let mut data = Vec::with_capacity(header.encode().len() + payload.len());
        data.extend_from_slice(&header.encode());
        data.extend_from_slice(payload);
        self.stats.published += 1;
        if self.active.contains_key(&key) {
            if self.waiting.insert(key, data).is_some() {
                self.stats.superseded += 1;
            }
        } else {
            self.start(conn, key, data);
        }
    }

    fn start(&mut self, conn: &mut quiche::Connection, key: (u8, u8), data: Vec<u8>) {
        let stream_id = self.next_stream_id;
        self.next_stream_id += 4;
        // Takes effect once the stream exists, i.e. after the first write;
        // set again then (see `pump`).
        let _ = conn.stream_priority(stream_id, BULK_URGENCY, false);
        self.active.insert(key, Outgoing { stream_id, data, offset: 0 });
    }

    /// Writes waiting object data, at most `budget` bytes if given. Returns
    /// the bytes quiche accepted. Stops on an object at quiche's flow-control
    /// or stream limit and moves on to the next.
    pub fn pump(&mut self, conn: &mut quiche::Connection, budget: Option<usize>) -> usize {
        let mut left = budget.unwrap_or(usize::MAX);
        let mut written = 0;
        let mut finished = Vec::new();
        for (key, o) in self.active.iter_mut() {
            while left > 0 {
                let start = o.offset;
                let end = (start + CHUNK.min(left)).min(o.data.len());
                let fin = end == o.data.len();
                match conn.stream_send(o.stream_id, &o.data[start..end], fin) {
                    Ok(n) => {
                        if start == 0 && n > 0 {
                            let _ = conn.stream_priority(o.stream_id, BULK_URGENCY, false);
                        }
                        o.offset += n;
                        written += n;
                        left -= n;
                        if fin && o.offset == o.data.len() {
                            finished.push(*key);
                            break;
                        }
                        if n < end - start {
                            break; // flow control: the rest waits for credit
                        }
                    }
                    Err(quiche::Error::Done) | Err(quiche::Error::StreamLimit) => break,
                    Err(e) => {
                        tracing::warn!(error = ?e, stream_id = o.stream_id, "bulk stream_send failed; dropping the object");
                        finished.push(*key);
                        break;
                    }
                }
            }
            if left == 0 {
                break;
            }
        }
        for key in finished {
            if self.active.remove(&key).is_some_and(|o| o.offset == o.data.len()) {
                self.stats.completed += 1;
            }
            if let Some(next) = self.waiting.remove(&key) {
                self.start(conn, key, next);
            }
        }
        written
    }
}

/// The header for the next version of an object.
pub fn header(kind: BulkKind, source_id: u8, encoding: u8, version: u32, payload: &[u8]) -> BulkHeader {
    BulkHeader {
        kind,
        source_id,
        encoding,
        version,
        length: payload.len() as u64,
        capture_time_us: roboprotocol_core::timestamp::now_micros(),
    }
}
