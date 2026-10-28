//! Forwarded pub/sub topics (spinworks-tech/robotele#36): samples from a
//! local Zenoh bus on one end of the session, republished on the other end's
//! own bus under the same key. Zenoh stays the bus on each host; only the
//! operator <-> robot link goes over RoboProtocol.
//!
//! A sample is a key, an attachment and a payload, all carried as opaque
//! bytes, so a BabyROS codec (Arrow IPC in the payload, codec info in the
//! attachment) passes through unchanged:
//!
//! ```text
//! key_len:u16 | key (UTF-8) | attachment_len:u32 | attachment | payload
//! ```
//!
//! Each forwarded topic has a mode:
//!   - `Latest`: one datagram per sample (tag `DATAGRAM_TAG_TOPIC`), lossy,
//!     and latest-wins per key while waiting to be sent. For state that is
//!     only useful while fresh. A sample larger than one datagram is
//!     dropped (and counted); use `Reliable` for those.
//!   - `Reliable`: in order, nothing lost, on one unidirectional stream per
//!     direction that every reliable topic shares. It starts with
//!     `TOPIC_STREAM_MAGIC` and `TOPIC_STREAM_VERSION`, then carries
//!     `len:u32 | sample` frames, and is never finished. Sharing one stream
//!     means a large sample on one topic delays the others behind it; the
//!     alternative, a stream per topic, would use up the peer's 16-stream
//!     limit.
//!
//! The key is sent with every sample rather than as an id agreed per
//! session: no handshake, at the cost of the key's length in each datagram.
//!
//! Transport-free like the rest of this crate: `TopicOutbox` decides what to
//! send next and `TopicStreamReader` reassembles the reliable stream; the
//! binaries move the bytes through quiche.

use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};

/// First byte of a reliable topic stream. Bulk object streams start with
/// `bulk::BULK_FORMAT` (1) instead, which is how a receiver tells the two
/// kinds of server-initiated unidirectional stream apart.
pub const TOPIC_STREAM_MAGIC: u8 = 0x54;
pub const TOPIC_STREAM_VERSION: u8 = 1;
/// Largest sample either side accepts, so a peer can't make it buffer
/// without bound.
pub const MAX_TOPIC_SAMPLE: usize = 16 * 1024 * 1024;
/// Reliable bytes a sender keeps waiting for flow-control credit before it
/// drops new samples instead, when forwarding isn't capped.
pub const DEFAULT_RELIABLE_BACKLOG: usize = 4 * 1024 * 1024;
/// With a `RateCap`, the reliable backlog holds this long at the capped rate:
/// a fixed 4 MB at 2 Mbps was 16 s of queue, so on the CM4 300 KB maps sat
/// behind stale samples and none arrived.
pub const CAPPED_BACKLOG_SECS: f64 = 2.0;
/// ...but never less than this, so a sample up to this size always fits.
pub const MIN_RELIABLE_BACKLOG: usize = 1024 * 1024;

/// The reliable backlog for a cap of `max_mbps` (0: no cap).
pub fn reliable_backlog(max_mbps: f64) -> usize {
    if max_mbps.is_nan() || max_mbps <= 0.0 {
        return DEFAULT_RELIABLE_BACKLOG;
    }
    ((max_mbps * 1e6 / 8.0 * CAPPED_BACKLOG_SECS) as usize).max(MIN_RELIABLE_BACKLOG)
}
/// Default cap on forwarded bytes per direction, in megabits per second.
/// On the CM4's 2.4 GHz Wi-Fi about 4 Mbps of forwarded traffic each way,
/// on a degraded link, coincided with a watchdog latch (BABYROS.md,
/// "Measured on the CM4"); half that leaves room for commands and video.
pub const DEFAULT_MAX_MBPS: f64 = 2.0;
/// How much a `RateCap` lets through at once after an idle spell.
const CAP_BURST: Duration = Duration::from_millis(50);
/// A burst always admits at least one full datagram.
const CAP_MIN_BURST_BYTES: f64 = 1500.0;

/// A token bucket on forwarded bytes: refills at the configured rate, holds
/// at most `CAP_BURST` worth.
#[derive(Debug, Clone)]
pub struct RateCap {
    bytes_per_sec: f64,
    burst: f64,
    tokens: f64,
    last: Instant,
}

impl RateCap {
    /// `None` for `mbps <= 0` (no cap).
    pub fn new(mbps: f64, now: Instant) -> Option<Self> {
        if mbps.is_nan() || mbps <= 0.0 {
            return None;
        }
        let bytes_per_sec = mbps * 1e6 / 8.0;
        let burst = (bytes_per_sec * CAP_BURST.as_secs_f64()).max(CAP_MIN_BURST_BYTES);
        Some(Self { bytes_per_sec, burst, tokens: burst, last: now })
    }

    fn refill(&mut self, now: Instant) {
        let dt = now.saturating_duration_since(self.last).as_secs_f64();
        self.tokens = (self.tokens + dt * self.bytes_per_sec).min(self.burst);
        self.last = now;
    }

    /// Bytes that may be sent now.
    pub fn available(&mut self, now: Instant) -> usize {
        self.refill(now);
        self.tokens.max(0.0) as usize
    }

    pub fn consume(&mut self, n: usize) {
        self.tokens -= n as f64;
    }
}

/// Keys under this prefix belong to robot-edge's own Zenoh sidecar
/// (`robotele/<id>/autonomy_goal` and so on), so a forwarded sample may not
/// use one: otherwise the operator's side could assert an autonomy goal
/// just by forwarding a topic.
pub const RESERVED_PREFIX: &str = "robotele/";

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TopicMode {
    Latest,
    Reliable,
}

impl TopicMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "latest" => Some(Self::Latest),
            "reliable" => Some(Self::Reliable),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Latest => "latest",
            Self::Reliable => "reliable",
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TopicSample {
    pub key: String,
    pub attachment: Vec<u8>,
    pub payload: Vec<u8>,
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum TopicError {
    #[error("sample ended early")]
    Short,
    #[error("key is not UTF-8")]
    KeyNotUtf8,
    #[error("key {0:?} is not a concrete key: it is empty, starts or ends with '/', or has a wildcard")]
    BadKey(String),
    #[error("key {0:?} is reserved for robot-edge's own topics")]
    ReservedKey(String),
    #[error("sample of {0} bytes is over the {MAX_TOPIC_SAMPLE}-byte limit")]
    TooLarge(usize),
    #[error("stream does not start with the topic stream header")]
    BadStreamHeader,
}

/// Whether `key` can be republished as is: a concrete Zenoh key (no
/// wildcards or empty chunks), and not under `RESERVED_PREFIX`.
pub fn check_key(key: &str) -> Result<(), TopicError> {
    let concrete = !key.is_empty()
        && key.len() <= u16::MAX as usize
        && !key.starts_with('/')
        && !key.ends_with('/')
        && !key.contains("//")
        && !key.contains(['*', '$', '?', '#']);
    if !concrete {
        return Err(TopicError::BadKey(key.to_string()));
    }
    if key.starts_with(RESERVED_PREFIX) {
        return Err(TopicError::ReservedKey(key.to_string()));
    }
    Ok(())
}

impl TopicSample {
    pub fn encoded_len(&self) -> usize {
        2 + self.key.len() + 4 + self.attachment.len() + self.payload.len()
    }

    pub fn encode_into(&self, out: &mut Vec<u8>) {
        out.reserve(self.encoded_len());
        out.extend_from_slice(&(self.key.len() as u16).to_be_bytes());
        out.extend_from_slice(self.key.as_bytes());
        out.extend_from_slice(&(self.attachment.len() as u32).to_be_bytes());
        out.extend_from_slice(&self.attachment);
        out.extend_from_slice(&self.payload);
    }

    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::new();
        self.encode_into(&mut out);
        out
    }

    /// Decodes a sample and checks its key with `check_key`.
    pub fn decode(bytes: &[u8]) -> Result<Self, TopicError> {
        if bytes.len() > MAX_TOPIC_SAMPLE {
            return Err(TopicError::TooLarge(bytes.len()));
        }
        let key_len = u16::from_be_bytes(bytes.get(..2).ok_or(TopicError::Short)?.try_into().unwrap()) as usize;
        let key = bytes.get(2..2 + key_len).ok_or(TopicError::Short)?;
        let key = std::str::from_utf8(key).map_err(|_| TopicError::KeyNotUtf8)?.to_string();
        check_key(&key)?;
        let rest = &bytes[2 + key_len..];
        let att_len = u32::from_be_bytes(rest.get(..4).ok_or(TopicError::Short)?.try_into().unwrap()) as usize;
        let attachment = rest.get(4..4 + att_len).ok_or(TopicError::Short)?.to_vec();
        let payload = rest[4 + att_len..].to_vec();
        Ok(Self { key, attachment, payload })
    }
}

/// What a `TopicOutbox` did with the samples it was given, for logs and the
/// HUD.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct OutboxStats {
    pub latest_sent: u64,
    /// Replaced by a newer sample of the same key before it was sent.
    pub latest_superseded: u64,
    /// Too large for one datagram on this connection.
    pub latest_oversize: u64,
    pub reliable_queued: u64,
    /// Dropped because the reliable backlog was full.
    pub reliable_dropped: u64,
}

/// Outgoing forwarded samples for one connection.
pub struct TopicOutbox {
    /// Tagged datagrams waiting to be sent, newest per key, oldest key first.
    latest: HashMap<String, Vec<u8>>,
    order: VecDeque<String>,
    /// Reliable stream bytes not yet accepted by quiche, starting with the
    /// stream header on a fresh connection.
    reliable: Vec<u8>,
    reliable_started: bool,
    reliable_backlog: usize,
    stats: OutboxStats,
}

impl Default for TopicOutbox {
    fn default() -> Self {
        Self::with_backlog(DEFAULT_RELIABLE_BACKLOG)
    }
}

impl TopicOutbox {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_backlog(reliable_backlog: usize) -> Self {
        Self {
            latest: HashMap::new(),
            order: VecDeque::new(),
            reliable: Vec::new(),
            reliable_started: false,
            reliable_backlog,
            stats: OutboxStats::default(),
        }
    }

    pub fn stats(&self) -> OutboxStats {
        self.stats
    }

    pub fn push(&mut self, mode: TopicMode, sample: &TopicSample) {
        match mode {
            TopicMode::Latest => self.push_latest(sample),
            TopicMode::Reliable => self.push_reliable(sample),
        }
    }

    fn push_latest(&mut self, sample: &TopicSample) {
        let mut d = Vec::with_capacity(1 + sample.encoded_len());
        d.push(crate::datagram::DATAGRAM_TAG_TOPIC);
        sample.encode_into(&mut d);
        if self.latest.insert(sample.key.clone(), d).is_some() {
            self.stats.latest_superseded += 1;
        } else {
            self.order.push_back(sample.key.clone());
        }
    }

    fn push_reliable(&mut self, sample: &TopicSample) {
        let len = sample.encoded_len();
        if len > MAX_TOPIC_SAMPLE || self.reliable.len() + 4 + len > self.reliable_backlog {
            self.stats.reliable_dropped += 1;
            return;
        }
        if !self.reliable_started {
            self.reliable.extend_from_slice(&[TOPIC_STREAM_MAGIC, TOPIC_STREAM_VERSION]);
            self.reliable_started = true;
        }
        self.reliable.extend_from_slice(&(len as u32).to_be_bytes());
        sample.encode_into(&mut self.reliable);
        self.stats.reliable_queued += 1;
    }

    pub fn has_datagram(&self) -> bool {
        !self.order.is_empty()
    }

    /// The next latest-mode datagram that fits in `max_len` bytes. Ones that
    /// don't fit are dropped and counted.
    pub fn pop_datagram(&mut self, max_len: usize) -> Option<Vec<u8>> {
        while let Some(key) = self.order.pop_front() {
            let d = self.latest.remove(&key).expect("every queued key has a datagram");
            if d.len() <= max_len {
                self.stats.latest_sent += 1;
                return Some(d);
            }
            self.stats.latest_oversize += 1;
        }
        None
    }

    /// Reliable stream bytes waiting to be written.
    pub fn reliable_pending(&self) -> &[u8] {
        &self.reliable
    }

    /// `n` bytes of `reliable_pending` were accepted by the transport.
    pub fn consume_reliable(&mut self, n: usize) {
        self.reliable.drain(..n.min(self.reliable.len()));
    }
}

/// Reassembles samples from a reliable topic stream's bytes.
#[derive(Default)]
pub struct TopicStreamReader {
    buf: Vec<u8>,
    header_seen: bool,
    failed: bool,
}

impl TopicStreamReader {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feeds bytes read from the stream and returns the samples they
    /// complete, in order. A sample with a bad key is skipped (its error is
    /// returned alongside, so the caller can log it); a malformed stream
    /// header or frame length fails the stream, and everything after it is
    /// ignored.
    pub fn push(&mut self, data: &[u8]) -> (Vec<TopicSample>, Vec<TopicError>) {
        let (mut samples, mut errors) = (Vec::new(), Vec::new());
        if self.failed {
            return (samples, errors);
        }
        self.buf.extend_from_slice(data);
        if !self.header_seen {
            if self.buf.len() < 2 {
                return (samples, errors);
            }
            if self.buf[..2] != [TOPIC_STREAM_MAGIC, TOPIC_STREAM_VERSION] {
                self.fail();
                errors.push(TopicError::BadStreamHeader);
                return (samples, errors);
            }
            self.buf.drain(..2);
            self.header_seen = true;
        }
        let mut at = 0;
        while let Some(len_bytes) = self.buf.get(at..at + 4) {
            let len = u32::from_be_bytes(len_bytes.try_into().unwrap()) as usize;
            if len > MAX_TOPIC_SAMPLE {
                self.fail();
                errors.push(TopicError::TooLarge(len));
                return (samples, errors);
            }
            let Some(frame) = self.buf.get(at + 4..at + 4 + len) else { break };
            match TopicSample::decode(frame) {
                Ok(s) => samples.push(s),
                Err(e) => errors.push(e),
            }
            at += 4 + len;
        }
        self.buf.drain(..at);
        (samples, errors)
    }

    fn fail(&mut self) {
        self.failed = true;
        self.buf = Vec::new();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::datagram::{untag, DATAGRAM_TAG_TOPIC};

    fn sample(key: &str, payload: &[u8]) -> TopicSample {
        TopicSample { key: key.into(), attachment: b"codec".to_vec(), payload: payload.to_vec() }
    }

    #[test]
    fn sample_round_trips_with_attachment_and_empty_parts() {
        for s in [sample("robot/camera", b"\x00\x01\x02"), TopicSample { key: "a".into(), attachment: vec![], payload: vec![] }] {
            let bytes = s.encode();
            assert_eq!(bytes.len(), s.encoded_len());
            assert_eq!(TopicSample::decode(&bytes).unwrap(), s);
        }
    }

    #[test]
    fn truncated_samples_fail_cleanly() {
        let bytes = sample("robot/camera", b"xyz").encode();
        for cut in 0..2 + "robot/camera".len() + 4 {
            assert_eq!(TopicSample::decode(&bytes[..cut]), Err(TopicError::Short), "cut at {cut}");
        }
    }

    #[test]
    fn keys_must_be_concrete_and_not_reserved() {
        for good in ["a", "robot/camera", "planner/goal/0", "robotele"] {
            assert_eq!(check_key(good), Ok(()), "{good}");
        }
        for bad in ["", "/a", "a/", "a//b", "a/*", "a/**", "$*", "a?b", "a#b"] {
            assert!(matches!(check_key(bad), Err(TopicError::BadKey(_))), "{bad}");
        }
        assert!(matches!(check_key("robotele/xgo/autonomy_goal"), Err(TopicError::ReservedKey(_))));
        assert!(matches!(TopicSample::decode(&sample("robotele/x/command", b"").encode()), Err(TopicError::ReservedKey(_))));
    }

    #[test]
    fn latest_is_newest_per_key_and_oldest_key_first() {
        let mut o = TopicOutbox::new();
        o.push(TopicMode::Latest, &sample("a", b"1"));
        o.push(TopicMode::Latest, &sample("b", b"1"));
        o.push(TopicMode::Latest, &sample("a", b"2"));
        let (tag, d) = untag(&o.pop_datagram(1200).unwrap()).unwrap();
        assert_eq!(tag, DATAGRAM_TAG_TOPIC);
        assert_eq!(TopicSample::decode(&d).unwrap(), sample("a", b"2"));
        assert_eq!(TopicSample::decode(&untag(&o.pop_datagram(1200).unwrap()).unwrap().1).unwrap().key, "b");
        assert_eq!(o.pop_datagram(1200), None);
        assert_eq!(o.stats().latest_superseded, 1);
        assert_eq!(o.stats().latest_sent, 2);
    }

    #[test]
    fn oversize_latest_samples_are_dropped_not_sent() {
        let mut o = TopicOutbox::new();
        o.push(TopicMode::Latest, &sample("big", &[0; 2000]));
        o.push(TopicMode::Latest, &sample("small", b"x"));
        let d = o.pop_datagram(1200).unwrap();
        assert_eq!(TopicSample::decode(&d[1..]).unwrap().key, "small");
        assert!(!o.has_datagram());
        assert_eq!(o.stats().latest_oversize, 1);
    }

    #[test]
    fn reliable_stream_round_trips_in_any_chunking() {
        let mut o = TopicOutbox::new();
        let samples: Vec<_> = (0..20u8).map(|i| sample(&format!("t/{}", i % 3), &vec![i; i as usize * 37])).collect();
        for s in &samples {
            o.push(TopicMode::Reliable, s);
        }
        let bytes = o.reliable_pending().to_vec();
        assert_eq!(bytes[..2], [TOPIC_STREAM_MAGIC, TOPIC_STREAM_VERSION]);
        for chunk in [1, 2, 7, 100, bytes.len()] {
            let mut r = TopicStreamReader::new();
            let mut got = Vec::new();
            for c in bytes.chunks(chunk) {
                let (s, e) = r.push(c);
                assert!(e.is_empty());
                got.extend(s);
            }
            assert_eq!(got, samples, "chunk size {chunk}");
        }
    }

    #[test]
    fn consumed_reliable_bytes_are_not_resent_and_the_header_goes_once() {
        let mut o = TopicOutbox::new();
        o.push(TopicMode::Reliable, &sample("a", b"1"));
        let first = o.reliable_pending().len();
        o.consume_reliable(first);
        assert!(o.reliable_pending().is_empty());
        o.push(TopicMode::Reliable, &sample("a", b"2"));
        assert_eq!(o.reliable_pending()[..4], (sample("a", b"2").encoded_len() as u32).to_be_bytes());
    }

    #[test]
    fn a_full_reliable_backlog_drops_new_samples() {
        let mut o = TopicOutbox::with_backlog(100);
        o.push(TopicMode::Reliable, &sample("a", &[0; 40]));
        o.push(TopicMode::Reliable, &sample("a", &[0; 40]));
        assert_eq!(o.stats().reliable_queued, 1);
        assert_eq!(o.stats().reliable_dropped, 1);
        o.consume_reliable(usize::MAX);
        o.push(TopicMode::Reliable, &sample("a", &[0; 40]));
        assert_eq!(o.stats().reliable_queued, 2);
    }

    #[test]
    fn the_reliable_backlog_follows_the_cap() {
        assert_eq!(reliable_backlog(0.0), DEFAULT_RELIABLE_BACKLOG);
        assert_eq!(reliable_backlog(2.0), MIN_RELIABLE_BACKLOG, "2 s at 2 Mbps is 500 KB, under the floor");
        assert_eq!(reliable_backlog(20.0), 5_000_000);
    }

    #[test]
    fn rate_cap_holds_the_rate_after_its_burst() {
        let t0 = Instant::now();
        assert!(RateCap::new(0.0, t0).is_none(), "0 means no cap");
        let mut cap = RateCap::new(2.0, t0).unwrap(); // 250,000 B/s, 12,500 B burst
        assert_eq!(cap.available(t0), 12_500);
        cap.consume(12_500);
        assert_eq!(cap.available(t0), 0);
        assert_eq!(cap.available(t0 + Duration::from_millis(20)), 5_000);
        // An idle spell refills only up to the burst.
        assert_eq!(cap.available(t0 + Duration::from_secs(10)), 12_500);
        // A tiny rate still admits one full datagram per burst.
        assert_eq!(RateCap::new(0.01, t0).unwrap().available(t0), 1_500);
    }

    #[test]
    fn reader_skips_bad_keys_but_fails_on_a_bad_header_or_length() {
        let mut o = TopicOutbox::new();
        o.push(TopicMode::Reliable, &sample("ok/1", b"1"));
        let mut bytes = o.reliable_pending().to_vec();
        let bad = TopicSample { key: "robotele/x/autonomy_goal".into(), attachment: vec![], payload: vec![1] }.encode();
        bytes.extend_from_slice(&(bad.len() as u32).to_be_bytes());
        bytes.extend_from_slice(&bad);
        let ok2 = sample("ok/2", b"2").encode();
        bytes.extend_from_slice(&(ok2.len() as u32).to_be_bytes());
        bytes.extend_from_slice(&ok2);
        let (s, e) = TopicStreamReader::new().push(&bytes);
        assert_eq!(s.iter().map(|s| s.key.as_str()).collect::<Vec<_>>(), ["ok/1", "ok/2"]);
        assert!(matches!(e[..], [TopicError::ReservedKey(_)]));

        let (s, e) = TopicStreamReader::new().push(&[crate::bulk::BULK_FORMAT, 1, 0, 0]);
        assert!(s.is_empty());
        assert_eq!(e, [TopicError::BadStreamHeader]);

        let mut r = TopicStreamReader::new();
        let (_, e) = r.push(&[TOPIC_STREAM_MAGIC, TOPIC_STREAM_VERSION, 0xFF, 0xFF, 0xFF, 0xFF]);
        assert!(matches!(e[..], [TopicError::TooLarge(_)]));
        assert_eq!(r.push(&bytes), (vec![], vec![]), "a failed stream stays failed");
    }
}
