//! Local recording (black-box logging) record framing (DESIGN.md §10.2,
//! REQUIREMENTS.md FR-9) and the pure rotation/retention decisions that
//! drive it. Not FlatBuffers-wrapped -- deliberately the inverse call from
//! Channel B/C's own wire format: a segment file is read by exactly one
//! thing (a replay/analysis tool in this codebase), so a vtable per record
//! would be pure overhead with no cross-language/schema-evolution benefit
//! to justify it (§10.2).
//!
//! The writer pipeline itself (bounded ring, dedicated thread, segment
//! files, actual disk I/O) lives in the separate `roboprotocol-recording`
//! crate, which needs OS threads/file I/O this crate deliberately avoids
//! (see `lib.rs`'s module doc) -- this module is only the pure, trivially
//! testable parts: record framing and the rotation/retention math.
//!
//! `capture_us` in every record is this *recording* endpoint's own local
//! wall-clock time, always -- not a copy of some remote sender's embedded
//! timestamp, even for records that started life elsewhere (e.g.
//! robot-edge recording an operator-sent command). See the recording
//! implementation plan's "Cross-endpoint time synchronization" section for
//! why: the remote sender's own timestamp already lives inside the
//! recorded Channel B payload (FR-9.2 requires the wire bytes verbatim,
//! and `ChannelBFrameData` carries `timestamp`/`seq`), so nothing is lost
//! by keeping `capture_us` purely local -- and a purely local clock is the
//! only thing that stays meaningful without assuming clock sync this
//! system doesn't have (there is no PTP between endpoints today; see
//! `timestamp.rs`'s module doc). `estimate_clock_offset_us` in
//! `timestamp.rs` is how a replay tool later reconciles two endpoints'
//! independently-local-clocked logs against each other.

use std::time::{Duration, Instant};

use crate::safety::ControlSource;

/// Sentinel `control_source` byte for records that have no arbitrated
/// control source at all -- Channel A (video), Channel C (`ActionTrigger`),
/// and key-press records, plus *every* record operator-console ever
/// writes (arbitration is robot-edge-local only; there is no
/// `control_source` field anywhere in the wire format).
pub const CONTROL_SOURCE_SENTINEL: u8 = 0xFF;

/// Maps an arbitrated `ControlSource` to its recorded byte (FR-9.8).
/// Never collides with `CONTROL_SOURCE_SENTINEL`.
pub fn control_source_byte(source: ControlSource) -> u8 {
    match source {
        ControlSource::EStop => 0,
        ControlSource::EmergencySafeParking => 1,
        ControlSource::ActiveImpedanceHold => 2,
        ControlSource::FullTeleoperation => 3,
        ControlSource::SemiAutonomous => 4,
    }
}

/// `record_len:u32 + capture_us:u64 + control_source:u8`, directly
/// prefixing `payload` -- see §10.2's record layout.
const HEADER_LEN: usize = 4 + 8 + 1;

/// One record's fixed header. `record_len` itself isn't a field here --
/// `encode_record` derives it from the actual payload length it's given,
/// so there's no way for a caller to encode a header/payload-length
/// mismatch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecordHeader {
    pub capture_us: u64,
    pub control_source: u8,
}

/// Appends one framed record (header + payload) to `out`.
pub fn encode_record(header: RecordHeader, payload: &[u8], out: &mut Vec<u8>) {
    out.extend_from_slice(&(payload.len() as u32).to_be_bytes());
    out.extend_from_slice(&header.capture_us.to_be_bytes());
    out.push(header.control_source);
    out.extend_from_slice(payload);
}

/// Self-delimiting iterator over concatenated records (FR-9.6): stops
/// cleanly, without error, the moment it hits a truncated final record --
/// e.g. a segment file cut off mid-write by a power loss -- so every
/// complete record before that point is still readable.
pub fn decode_records(buf: &[u8]) -> impl Iterator<Item = (RecordHeader, &[u8])> {
    RecordIter { buf }
}

struct RecordIter<'a> {
    buf: &'a [u8],
}

impl<'a> Iterator for RecordIter<'a> {
    type Item = (RecordHeader, &'a [u8]);

    fn next(&mut self) -> Option<Self::Item> {
        if self.buf.len() < HEADER_LEN {
            return None;
        }
        let record_len = u32::from_be_bytes(self.buf[0..4].try_into().ok()?) as usize;
        let capture_us = u64::from_be_bytes(self.buf[4..12].try_into().ok()?);
        let control_source = self.buf[12];
        let total = HEADER_LEN + record_len;
        if self.buf.len() < total {
            // Truncated final record -- stop here, don't error. Nothing
            // before this point is affected.
            self.buf = &[];
            return None;
        }
        let payload = &self.buf[HEADER_LEN..total];
        self.buf = &self.buf[total..];
        Some((RecordHeader { capture_us, control_source }, payload))
    }
}

/// Size/duration rotation thresholds for one category's segments
/// (FR-9.4) -- a segment rotates at whichever is hit first.
#[derive(Debug, Clone, Copy)]
pub struct RotationThresholds {
    pub max_segment_bytes: u64,
    pub max_segment_duration: Duration,
}

/// True if the currently-open segment should be closed and a fresh one
/// started, given its size and how long it's been open.
pub fn should_rotate(current_size: u64, opened_at: Instant, now: Instant, thresholds: &RotationThresholds) -> bool {
    current_size >= thresholds.max_segment_bytes || now.saturating_duration_since(opened_at) >= thresholds.max_segment_duration
}

/// One existing segment's size, for retention accounting.
#[derive(Debug, Clone, Copy)]
pub struct SegmentMeta {
    pub size_bytes: u64,
}

/// Indices into `existing` (must be oldest-first, e.g. by the
/// timestamp-prefixed segment filenames the writer crate uses) to delete
/// so total size plus `incoming_bytes` fits under `budget_bytes` --
/// FR-9.4's "delete the oldest segment(s) first, never the newest."
/// Never selects the last (newest) element of `existing`, even if
/// deleting everything else still leaves the total over budget: the
/// segment currently being appended to is never a deletion target here,
/// only history is.
pub fn segments_to_delete_for_budget(existing: &[SegmentMeta], incoming_bytes: u64, budget_bytes: u64) -> Vec<usize> {
    let mut total: u64 = existing.iter().map(|s| s.size_bytes).sum::<u64>() + incoming_bytes;
    let mut to_delete = Vec::new();
    for (i, seg) in existing.iter().enumerate() {
        if i + 1 == existing.len() {
            break; // never delete the newest
        }
        if total <= budget_bytes {
            break;
        }
        total = total.saturating_sub(seg.size_bytes);
        to_delete.push(i);
    }
    to_delete
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn record_round_trips_single() {
        let mut buf = Vec::new();
        encode_record(RecordHeader { capture_us: 1_000_000, control_source: 3 }, b"hello", &mut buf);
        let records: Vec<_> = decode_records(&buf).collect();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].0, RecordHeader { capture_us: 1_000_000, control_source: 3 });
        assert_eq!(records[0].1, b"hello");
    }

    #[test]
    fn record_round_trips_multiple_concatenated() {
        let mut buf = Vec::new();
        encode_record(RecordHeader { capture_us: 1, control_source: 0 }, b"a", &mut buf);
        encode_record(RecordHeader { capture_us: 2, control_source: 0xFF }, b"bb", &mut buf);
        encode_record(RecordHeader { capture_us: 3, control_source: 4 }, b"", &mut buf);
        let records: Vec<_> = decode_records(&buf).collect();
        assert_eq!(records.len(), 3);
        assert_eq!(records[0].1, b"a");
        assert_eq!(records[1].1, b"bb");
        assert_eq!(records[2].1, b"" as &[u8]);
    }

    #[test]
    fn truncated_final_record_does_not_prevent_reading_earlier_ones() {
        let mut buf = Vec::new();
        encode_record(RecordHeader { capture_us: 1, control_source: 0 }, b"complete", &mut buf);
        let good_len = buf.len();
        encode_record(RecordHeader { capture_us: 2, control_source: 0 }, b"this one gets cut off", &mut buf);
        buf.truncate(good_len + 10); // chop the second record mid-payload

        let records: Vec<_> = decode_records(&buf).collect();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].1, b"complete");
    }

    #[test]
    fn truncated_header_alone_yields_nothing_but_does_not_panic() {
        let buf = vec![0u8, 1, 2]; // shorter than HEADER_LEN
        assert_eq!(decode_records(&buf).count(), 0);
    }

    #[test]
    fn should_rotate_at_size_threshold() {
        let t = RotationThresholds { max_segment_bytes: 100, max_segment_duration: Duration::from_secs(3600) };
        let now = Instant::now();
        assert!(should_rotate(100, now, now, &t));
        assert!(should_rotate(150, now, now, &t));
        assert!(!should_rotate(99, now, now, &t));
    }

    #[test]
    fn should_rotate_at_duration_threshold() {
        let t = RotationThresholds { max_segment_bytes: u64::MAX, max_segment_duration: Duration::from_secs(60) };
        let opened_at = Instant::now();
        let now = opened_at + Duration::from_secs(61);
        assert!(should_rotate(0, opened_at, now, &t));
        let now_early = opened_at + Duration::from_secs(30);
        assert!(!should_rotate(0, opened_at, now_early, &t));
    }

    #[test]
    fn segments_to_delete_deletes_oldest_first_and_stops_under_budget() {
        let existing = [SegmentMeta { size_bytes: 100 }, SegmentMeta { size_bytes: 100 }, SegmentMeta { size_bytes: 100 }];
        let to_delete = segments_to_delete_for_budget(&existing, 50, 200);
        // total = 350, budget 200: delete index 0 (-100 -> 250), delete index 1 (-100 -> 150 <= 200), stop.
        assert_eq!(to_delete, vec![0, 1]);
    }

    #[test]
    fn segments_to_delete_is_empty_when_already_under_budget() {
        let existing = [SegmentMeta { size_bytes: 10 }, SegmentMeta { size_bytes: 10 }];
        assert!(segments_to_delete_for_budget(&existing, 5, 1000).is_empty());
    }

    #[test]
    fn segments_to_delete_never_selects_the_newest() {
        // Even wildly over budget with only one segment, it's never a
        // deletion target -- it's the one currently being written to.
        let existing = [SegmentMeta { size_bytes: 1_000_000 }];
        assert!(segments_to_delete_for_budget(&existing, 0, 1).is_empty());

        // With several segments, the last index must never appear.
        let existing = [SegmentMeta { size_bytes: 500 }, SegmentMeta { size_bytes: 500 }, SegmentMeta { size_bytes: 500 }];
        let to_delete = segments_to_delete_for_budget(&existing, 0, 1);
        assert!(!to_delete.contains(&2));
    }

    #[test]
    fn control_source_byte_never_produces_the_sentinel() {
        for source in [
            ControlSource::EStop,
            ControlSource::EmergencySafeParking,
            ControlSource::ActiveImpedanceHold,
            ControlSource::FullTeleoperation,
            ControlSource::SemiAutonomous,
        ] {
            assert_ne!(control_source_byte(source), CONTROL_SOURCE_SENTINEL);
        }
    }
}
