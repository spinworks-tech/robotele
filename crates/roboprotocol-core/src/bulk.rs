//! Bulk objects (docs/13-large-sensor-payloads.md, "Bulk objects"): large
//! data that must arrive whole and is updated rarely -- an occupancy grid or
//! map, an accumulated keyframe cloud, a full-resolution snapshot.
//!
//! Each object travels on its own server-initiated unidirectional QUIC
//! stream (the Media over QUIC pattern): a fixed `BulkHeader`, then exactly
//! `length` bytes, then FIN. Streams are reliable, so an object arrives
//! complete or not at all. When a newer version of the same object (same
//! `kind` and `source_id`) starts, the sender resets the older stream if it
//! hasn't finished, and the receiver drops any partial copy: the latest
//! version wins, as for sensor frames, but nothing is ever delivered torn.
//!
//! Transport-free like the rest of this crate: `robot-edge` and
//! `operator-console` share the header format and the receiver's rules.

use std::collections::{HashMap, HashSet};

/// Header format version, the first byte of every bulk stream.
pub const BULK_FORMAT: u8 = 1;
/// format:u8, kind:u8, source_id:u8, encoding:u8, version:u32,
/// length:u64, capture_time_us:u64 -- big-endian, like the other headers.
pub const BULK_HEADER_LEN: usize = 1 + 1 + 1 + 1 + 4 + 8 + 8;
/// Largest object a receiver will accept, so a peer can't make it buffer
/// without bound: a 4,000 x 4,000 occupancy grid is 16 MB.
pub const MAX_BULK_OBJECT: u64 = 64 * 1024 * 1024;
/// Application error code a sender uses when it resets a stream because a
/// newer version of the same object superseded it.
pub const BULK_SUPERSEDED: u64 = 0x5B;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BulkKind {
    OccupancyGrid,
    KeyframeCloud,
    DepthSnapshot,
    /// Opaque bytes, for benchmarks and anything not listed yet.
    Blob,
}

impl BulkKind {
    pub fn to_u8(self) -> u8 {
        match self {
            Self::OccupancyGrid => 1,
            Self::KeyframeCloud => 2,
            Self::DepthSnapshot => 3,
            Self::Blob => 255,
        }
    }

    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            1 => Some(Self::OccupancyGrid),
            2 => Some(Self::KeyframeCloud),
            3 => Some(Self::DepthSnapshot),
            255 => Some(Self::Blob),
            _ => None,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::OccupancyGrid => "occupancy-grid",
            Self::KeyframeCloud => "keyframe-cloud",
            Self::DepthSnapshot => "depth-snapshot",
            Self::Blob => "blob",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BulkHeader {
    pub kind: BulkKind,
    pub source_id: u8,
    /// Kind-specific payload format; see `OccupancyGrid` for kind 1.
    pub encoding: u8,
    /// Increases with every new version of this (kind, source_id).
    pub version: u32,
    pub length: u64,
    /// Robot-clock time the object's contents describe.
    pub capture_time_us: u64,
}

impl BulkHeader {
    pub fn encode(&self) -> [u8; BULK_HEADER_LEN] {
        let mut out = [0u8; BULK_HEADER_LEN];
        out[0] = BULK_FORMAT;
        out[1] = self.kind.to_u8();
        out[2] = self.source_id;
        out[3] = self.encoding;
        out[4..8].copy_from_slice(&self.version.to_be_bytes());
        out[8..16].copy_from_slice(&self.length.to_be_bytes());
        out[16..24].copy_from_slice(&self.capture_time_us.to_be_bytes());
        out
    }

    pub fn decode(bytes: &[u8]) -> Result<Self, BulkError> {
        if bytes.len() < BULK_HEADER_LEN {
            return Err(BulkError::ShortHeader);
        }
        if bytes[0] != BULK_FORMAT {
            return Err(BulkError::UnknownFormat(bytes[0]));
        }
        let kind = BulkKind::from_u8(bytes[1]).ok_or(BulkError::UnknownKind(bytes[1]))?;
        let length = u64::from_be_bytes(bytes[8..16].try_into().unwrap());
        if length > MAX_BULK_OBJECT {
            return Err(BulkError::TooLarge(length));
        }
        Ok(Self {
            kind,
            source_id: bytes[2],
            encoding: bytes[3],
            version: u32::from_be_bytes(bytes[4..8].try_into().unwrap()),
            length,
            capture_time_us: u64::from_be_bytes(bytes[16..24].try_into().unwrap()),
        })
    }

    pub fn key(&self) -> (BulkKind, u8) {
        (self.kind, self.source_id)
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum BulkError {
    #[error("stream ended inside the header")]
    ShortHeader,
    #[error("unknown bulk header format {0}")]
    UnknownFormat(u8),
    #[error("unknown bulk object kind {0}")]
    UnknownKind(u8),
    #[error("object of {0} bytes is over the {MAX_BULK_OBJECT}-byte limit")]
    TooLarge(u64),
    #[error("stream carried more bytes than its header declared")]
    TooLong,
    #[error("stream ended after {got} of {expected} bytes")]
    Truncated { got: u64, expected: u64 },
}

/// Server-initiated unidirectional streams are the ones numbered 3 mod 4
/// (RFC 9000 §2.1): the only streams bulk objects use.
pub fn is_bulk_stream(stream_id: u64) -> bool {
    stream_id % 4 == 3
}

/// A complete object, ready for the application.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BulkObject {
    pub header: BulkHeader,
    pub bytes: Vec<u8>,
}

/// What happened to the objects a `BulkReceiver` has seen, for the HUD.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BulkStats {
    pub completed: u64,
    /// Partial objects dropped because the sender reset the stream or a
    /// newer version arrived first.
    pub superseded: u64,
    /// Streams dropped for a malformed header or a length mismatch.
    pub rejected: u64,
}

struct Partial {
    header: Option<BulkHeader>,
    bytes: Vec<u8>,
}

/// Reassembles bulk objects from their streams' bytes.
#[derive(Default)]
pub struct BulkReceiver {
    streams: HashMap<u64, Partial>,
    /// Highest version delivered (or started) per (kind, source).
    latest: HashMap<(BulkKind, u8), u32>,
    /// Streams dropped before they ended (superseded or malformed): the rest
    /// of their bytes is ignored until FIN, rather than misread as a new
    /// object's header.
    dropped: HashSet<u64>,
    stats: BulkStats,
}

impl BulkReceiver {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stats(&self) -> BulkStats {
        self.stats
    }

    /// Feeds bytes read from `stream_id`. Returns the object when `fin`
    /// completes it. An error means the stream was dropped (and counted);
    /// the caller should stop reading it.
    pub fn on_data(&mut self, stream_id: u64, data: &[u8], fin: bool) -> Result<Option<BulkObject>, BulkError> {
        if self.dropped.contains(&stream_id) {
            if fin {
                self.dropped.remove(&stream_id);
            }
            return Ok(None);
        }
        let partial = self.streams.entry(stream_id).or_insert_with(|| Partial { header: None, bytes: Vec::new() });
        partial.bytes.extend_from_slice(data);
        if partial.header.is_none() && partial.bytes.len() >= BULK_HEADER_LEN {
            match BulkHeader::decode(&partial.bytes) {
                Ok(h) => {
                    partial.bytes.drain(..BULK_HEADER_LEN);
                    partial.bytes.reserve(h.length as usize);
                    partial.header = Some(h);
                    self.on_header(stream_id, h);
                }
                Err(e) => return Err(self.reject(stream_id, e)),
            }
        }
        let Some(partial) = self.streams.get(&stream_id) else { return Ok(None) }; // superseded just now
        let header = partial.header;
        let have = partial.bytes.len() as u64;
        if let Some(h) = header {
            if have > h.length {
                return Err(self.reject(stream_id, BulkError::TooLong));
            }
        }
        if !fin {
            return Ok(None);
        }
        let partial = self.streams.remove(&stream_id).expect("present above");
        let Some(header) = partial.header else {
            self.stats.rejected += 1;
            return Err(BulkError::ShortHeader);
        };
        if have != header.length {
            self.stats.rejected += 1;
            return Err(BulkError::Truncated { got: have, expected: header.length });
        }
        if self.latest.get(&header.key()).is_some_and(|&v| v > header.version) {
            self.stats.superseded += 1;
            return Ok(None);
        }
        self.stats.completed += 1;
        Ok(Some(BulkObject { header, bytes: partial.bytes }))
    }

    /// The sender reset `stream_id` (or it was otherwise abandoned): drop it.
    pub fn on_reset(&mut self, stream_id: u64) {
        self.dropped.remove(&stream_id);
        if self.streams.remove(&stream_id).is_some() {
            self.stats.superseded += 1;
        }
    }

    fn drop_stream(&mut self, stream_id: u64) {
        if self.streams.remove(&stream_id).is_some() {
            self.dropped.insert(stream_id);
        }
    }

    /// A newer version makes any partial copy of an older one pointless.
    fn on_header(&mut self, stream_id: u64, h: BulkHeader) {
        let latest = self.latest.entry(h.key()).or_insert(h.version);
        *latest = (*latest).max(h.version);
        let stale: Vec<u64> = self
            .streams
            .iter()
            .filter(|(&id, p)| id != stream_id && p.header.is_some_and(|o| o.key() == h.key() && o.version < h.version))
            .map(|(&id, _)| id)
            .collect();
        for id in stale {
            self.drop_stream(id);
            self.stats.superseded += 1;
        }
        if h.version < *self.latest.get(&h.key()).unwrap() {
            self.drop_stream(stream_id);
            self.stats.superseded += 1;
        }
    }

    fn reject(&mut self, stream_id: u64, e: BulkError) -> BulkError {
        self.drop_stream(stream_id);
        self.stats.rejected += 1;
        e
    }
}

/// Encoding 1 of `BulkKind::OccupancyGrid`: width:u32, height:u32,
/// resolution_mm:u32, origin_x_mm:i32, origin_y_mm:i32, then width x height
/// cells row-major, one byte each: 0 free, 100 occupied, 255 unknown (the
/// ROS `OccupancyGrid` convention, as unsigned bytes).
pub const OCCUPANCY_GRID_ENCODING: u8 = 1;
pub const OCCUPANCY_GRID_HEADER_LEN: usize = 20;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OccupancyGrid {
    pub width: u32,
    pub height: u32,
    pub resolution_mm: u32,
    pub origin_x_mm: i32,
    pub origin_y_mm: i32,
    pub cells: Vec<u8>,
}

impl OccupancyGrid {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(OCCUPANCY_GRID_HEADER_LEN + self.cells.len());
        out.extend_from_slice(&self.width.to_be_bytes());
        out.extend_from_slice(&self.height.to_be_bytes());
        out.extend_from_slice(&self.resolution_mm.to_be_bytes());
        out.extend_from_slice(&self.origin_x_mm.to_be_bytes());
        out.extend_from_slice(&self.origin_y_mm.to_be_bytes());
        out.extend_from_slice(&self.cells);
        out
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        let h = bytes.get(..OCCUPANCY_GRID_HEADER_LEN)?;
        let u = |i: usize| u32::from_be_bytes(h[i..i + 4].try_into().unwrap());
        let (width, height) = (u(0), u(4));
        let cells = bytes[OCCUPANCY_GRID_HEADER_LEN..].to_vec();
        (cells.len() as u64 == width as u64 * height as u64).then(|| Self {
            width,
            height,
            resolution_mm: u(8),
            origin_x_mm: u(12) as i32,
            origin_y_mm: u(16) as i32,
            cells,
        })
    }

    /// The grid as a binary PGM image (free white, occupied black, unknown
    /// grey), viewable in any image viewer. Row 0 is printed last, so +y
    /// points up as on a map.
    pub fn to_pgm(&self) -> Vec<u8> {
        let mut out = format!("P5\n{} {}\n255\n", self.width, self.height).into_bytes();
        for row in self.cells.chunks(self.width.max(1) as usize).rev() {
            out.extend(row.iter().map(|&c| match c {
                0 => 255,
                255 => 128,
                occupied => 255u8.saturating_sub((occupied as u16 * 255 / 100) as u8),
            }));
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(version: u32, length: u64) -> BulkHeader {
        BulkHeader { kind: BulkKind::OccupancyGrid, source_id: 0, encoding: 1, version, length, capture_time_us: 42 }
    }

    fn stream_bytes(h: BulkHeader, payload: &[u8]) -> Vec<u8> {
        let mut v = h.encode().to_vec();
        v.extend_from_slice(payload);
        v
    }

    #[test]
    fn header_round_trips() {
        let h = header(7, 1234);
        assert_eq!(BulkHeader::decode(&h.encode()), Ok(h));
    }

    #[test]
    fn header_rejects_unknown_format_kind_and_oversize() {
        let mut b = header(1, 10).encode();
        b[0] = 9;
        assert_eq!(BulkHeader::decode(&b), Err(BulkError::UnknownFormat(9)));
        let mut b = header(1, 10).encode();
        b[1] = 77;
        assert_eq!(BulkHeader::decode(&b), Err(BulkError::UnknownKind(77)));
        let b = header(1, MAX_BULK_OBJECT + 1).encode();
        assert_eq!(BulkHeader::decode(&b), Err(BulkError::TooLarge(MAX_BULK_OBJECT + 1)));
    }

    #[test]
    fn bulk_streams_are_server_initiated_unidirectional() {
        assert!(is_bulk_stream(3) && is_bulk_stream(7));
        assert!(!is_bulk_stream(0) && !is_bulk_stream(1) && !is_bulk_stream(2) && !is_bulk_stream(5));
    }

    #[test]
    fn an_object_split_across_reads_completes_at_fin() {
        let payload: Vec<u8> = (0..5_000u32).map(|i| i as u8).collect();
        let bytes = stream_bytes(header(1, payload.len() as u64), &payload);
        let mut r = BulkReceiver::new();
        let (a, b) = bytes.split_at(10); // splits the header too
        assert_eq!(r.on_data(3, a, false), Ok(None));
        assert_eq!(r.on_data(3, &b[..1_000], false), Ok(None));
        let obj = r.on_data(3, &b[1_000..], true).unwrap().unwrap();
        assert_eq!((obj.header.version, obj.bytes), (1, payload));
        assert_eq!(r.stats().completed, 1);
    }

    #[test]
    fn a_truncated_or_overlong_stream_is_rejected() {
        let mut r = BulkReceiver::new();
        let bytes = stream_bytes(header(1, 100), &[0; 50]);
        assert_eq!(r.on_data(3, &bytes, true), Err(BulkError::Truncated { got: 50, expected: 100 }));
        let bytes = stream_bytes(header(2, 10), &[0; 20]);
        assert_eq!(r.on_data(7, &bytes, false), Err(BulkError::TooLong));
        assert_eq!(r.stats().rejected, 2);
    }

    #[test]
    fn a_newer_version_drops_an_older_partial_copy() {
        let mut r = BulkReceiver::new();
        r.on_data(3, &stream_bytes(header(1, 100), &[1; 40]), false).unwrap();
        let v2 = stream_bytes(header(2, 10), &[2; 10]);
        let obj = r.on_data(7, &v2, true).unwrap().unwrap();
        assert_eq!(obj.header.version, 2);
        // The rest of version 1 arriving later is ignored, not delivered.
        assert_eq!(r.on_data(3, &[1; 60], true), Ok(None));
        assert_eq!(r.stats(), BulkStats { completed: 1, superseded: 1, rejected: 0 });
    }

    #[test]
    fn an_older_version_starting_late_is_dropped() {
        let mut r = BulkReceiver::new();
        r.on_data(7, &stream_bytes(header(2, 10), &[2; 10]), true).unwrap();
        assert_eq!(r.on_data(3, &stream_bytes(header(1, 10), &[1; 10]), true), Ok(None));
        assert_eq!(r.stats().superseded, 1);
    }

    #[test]
    fn a_reset_stream_is_dropped() {
        let mut r = BulkReceiver::new();
        r.on_data(3, &stream_bytes(header(1, 100), &[1; 40]), false).unwrap();
        r.on_reset(3);
        assert_eq!(r.stats().superseded, 1);
        assert_eq!(r.on_data(3, &[], true), Err(BulkError::ShortHeader), "a reset stream is gone; nothing is delivered");
    }

    #[test]
    fn different_objects_do_not_supersede_each_other() {
        let mut r = BulkReceiver::new();
        let mut other = header(1, 10);
        other.source_id = 1;
        r.on_data(3, &stream_bytes(header(5, 100), &[0; 40]), false).unwrap();
        assert!(r.on_data(7, &stream_bytes(other, &[0; 10]), true).unwrap().is_some());
        assert!(r.on_data(3, &[0; 60], true).unwrap().is_some(), "source 0's object still completes");
    }

    #[test]
    fn occupancy_grid_round_trips_and_renders() {
        let g = OccupancyGrid { width: 3, height: 2, resolution_mm: 50, origin_x_mm: -100, origin_y_mm: 200, cells: vec![0, 100, 255, 0, 0, 50] };
        let back = OccupancyGrid::decode(&g.encode()).unwrap();
        assert_eq!(back, g);
        let pgm = g.to_pgm();
        assert!(pgm.starts_with(b"P5\n3 2\n255\n"));
        // Row 1 (printed first): free, free, 50% -> 255, 255, 128.
        assert_eq!(&pgm[pgm.len() - 6..pgm.len() - 3], &[255, 255, 128]);
        assert!(OccupancyGrid::decode(&g.encode()[..20 + 5]).is_none(), "cell count must match");
    }
}
