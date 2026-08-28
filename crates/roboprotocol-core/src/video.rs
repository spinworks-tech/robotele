//! Channel A (video) chunk framing, Annex-B NAL splitting, and
//! reassembly. Not FlatBuffers-wrapped: DESIGN.md's layered-protocol
//! table specifies FlatBuffers for the Control application layer, not
//! Video ("H.265 / AV1 (PIR Mode) ... over QUIC Datagrams / MoQ") -- so
//! this is a small fixed binary header directly prefixing each chunk's
//! raw bytes, kept as simple/cheap as Channel B's own per-datagram
//! overhead philosophy.
//!
//! v0 known gap (documented, not silent): no FlexFEC, no cross-NAL
//! reordering/resequencing. A lost or reordered datagram just
//! glitches/drops that NAL; the decoder recovers cleanly at the next
//! I-frame (`rpicam-vid --intra N` on the capture side).

use std::collections::HashMap;

pub const CHUNK_HEADER_LEN: usize = 4 + 2 + 1; // nal_id:u32, chunk_index:u16, is_last:u8
pub const MAX_CHUNK_PAYLOAD: usize = 1100;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ChunkHeader {
    pub nal_id: u32,
    pub chunk_index: u16,
    pub is_last: bool,
}

impl ChunkHeader {
    pub fn encode(&self, out: &mut Vec<u8>) {
        out.extend_from_slice(&self.nal_id.to_be_bytes());
        out.extend_from_slice(&self.chunk_index.to_be_bytes());
        out.push(self.is_last as u8);
    }

    pub fn decode(bytes: &[u8]) -> Option<(Self, &[u8])> {
        if bytes.len() < CHUNK_HEADER_LEN {
            return None;
        }
        let nal_id = u32::from_be_bytes(bytes[0..4].try_into().ok()?);
        let chunk_index = u16::from_be_bytes(bytes[4..6].try_into().ok()?);
        let is_last = bytes[6] != 0;
        Some((Self { nal_id, chunk_index, is_last }, &bytes[CHUNK_HEADER_LEN..]))
    }
}

/// True for NAL types the decoder cannot recover from losing: SPS(7),
/// PPS(8), IDR slice(5). Everything else (P/B slices, SEI, AUDs, ...) is
/// safe to drop when superseded -- losing an SPS/PPS/IDR instead breaks
/// decoding until the next IDR (or, if the encoder doesn't repeat
/// SPS/PPS per GOP, for the rest of the session). `nal` must be the raw
/// NAL bytes *without* an Annex-B start code prefix.
pub fn nal_is_critical(nal: &[u8]) -> bool {
    matches!(nal.first().map(|b| b & 0x1F), Some(5 | 7 | 8))
}

/// Splits one Annex-B NAL unit (without its start-code prefix) into
/// datagram-sized chunks, each self-describing via `ChunkHeader`.
pub fn chunk_nal(nal_id: u32, nal: &[u8]) -> Vec<Vec<u8>> {
    if nal.is_empty() {
        return Vec::new();
    }
    let n_chunks = nal.len().div_ceil(MAX_CHUNK_PAYLOAD);
    let mut out = Vec::with_capacity(n_chunks);
    for (i, piece) in nal.chunks(MAX_CHUNK_PAYLOAD).enumerate() {
        let mut buf = Vec::with_capacity(CHUNK_HEADER_LEN + piece.len());
        ChunkHeader { nal_id, chunk_index: i as u16, is_last: i + 1 == n_chunks }.encode(&mut buf);
        buf.extend_from_slice(piece);
        out.push(buf);
    }
    out
}

/// Finds complete Annex-B NAL units (0x000001 / 0x00000001 start codes)
/// in a growing byte stream fed incrementally via `push`, handing back
/// each NAL's bytes (without its start code) once the *next* start code
/// confirms it's complete. Bytes before the first start code are
/// discarded (partial NAL from mid-stream attach).
#[derive(Default)]
pub struct AnnexBSplitter {
    buf: Vec<u8>,
}

impl AnnexBSplitter {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, data: &[u8]) -> Vec<Vec<u8>> {
        self.buf.extend_from_slice(data);
        let starts = find_start_codes(&self.buf);
        if starts.len() < 2 {
            return Vec::new();
        }
        let mut nals = Vec::with_capacity(starts.len() - 1);
        for w in starts.windows(2) {
            let (start, next_start) = (w[0], w[1]);
            nals.push(self.buf[start.1..next_start.0].to_vec());
        }
        // Keep from the last confirmed start code onward -- it may still
        // be an incomplete NAL awaiting more bytes/the next start code.
        let keep_from = starts.last().unwrap().0;
        self.buf.drain(0..keep_from);
        nals
    }
}

/// Returns `(start_code_offset, nal_data_offset)` pairs for every Annex-B
/// start code (3- or 4-byte form) found in `buf`.
fn find_start_codes(buf: &[u8]) -> Vec<(usize, usize)> {
    let mut out = Vec::new();
    let mut i = 0;
    while i + 3 <= buf.len() {
        if buf[i] == 0 && buf[i + 1] == 0 {
            if buf[i + 2] == 1 {
                out.push((i, i + 3));
                i += 3;
                continue;
            }
            if i + 4 <= buf.len() && buf[i + 2] == 0 && buf[i + 3] == 1 {
                out.push((i, i + 4));
                i += 4;
                continue;
            }
        }
        i += 1;
    }
    out
}

struct PendingNal {
    chunks: Vec<Option<Vec<u8>>>,
    received: usize,
    total: Option<usize>,
}

/// Reassembles NAL units from received chunks, tolerant of loss/reorder
/// per this module's documented v0 gap: a NAL only ever completes if
/// every one of its chunks arrives; under sustained loss, the oldest
/// still-incomplete NAL id is evicted once `max_pending` is exceeded,
/// bounding memory rather than growing unbounded.
pub struct NalReassembler {
    pending: HashMap<u32, PendingNal>,
    max_pending: usize,
}

impl NalReassembler {
    pub fn new(max_pending: usize) -> Self {
        Self { pending: HashMap::new(), max_pending }
    }

    /// Feed one received chunk (post `ChunkHeader::decode`). Returns the
    /// fully reassembled NAL's bytes once its last chunk arrives.
    pub fn on_chunk(&mut self, header: ChunkHeader, payload: &[u8]) -> Option<Vec<u8>> {
        let entry = self.pending.entry(header.nal_id).or_insert_with(|| PendingNal {
            chunks: Vec::new(),
            received: 0,
            total: None,
        });

        let idx = header.chunk_index as usize;
        if entry.chunks.len() <= idx {
            entry.chunks.resize(idx + 1, None);
        }
        if entry.chunks[idx].is_none() {
            entry.chunks[idx] = Some(payload.to_vec());
            entry.received += 1;
        }
        if header.is_last {
            entry.total = Some(idx + 1);
        }

        let complete = entry.total == Some(entry.received) && entry.chunks.iter().all(Option::is_some);
        let result = if complete {
            let entry = self.pending.remove(&header.nal_id).unwrap();
            Some(entry.chunks.into_iter().flatten().flatten().collect())
        } else {
            None
        };

        if self.pending.len() > self.max_pending {
            if let Some(&oldest) = self.pending.keys().min() {
                self.pending.remove(&oldest);
            }
        }

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nal_is_critical_identifies_sps_pps_idr_only() {
        assert!(nal_is_critical(&[0x67])); // SPS (type 7)
        assert!(nal_is_critical(&[0x68])); // PPS (type 8)
        assert!(nal_is_critical(&[0x65])); // IDR slice (type 5)
        assert!(!nal_is_critical(&[0x61])); // non-IDR slice (type 1)
        assert!(!nal_is_critical(&[0x06])); // SEI (type 6)
        assert!(!nal_is_critical(&[]));
    }

    #[test]
    fn chunk_header_round_trips() {
        let h = ChunkHeader { nal_id: 0xDEADBEEF, chunk_index: 7, is_last: true };
        let mut buf = Vec::new();
        h.encode(&mut buf);
        buf.extend_from_slice(b"payload");
        let (decoded, rest) = ChunkHeader::decode(&buf).unwrap();
        assert_eq!(decoded, h);
        assert_eq!(rest, b"payload");
    }

    #[test]
    fn small_nal_produces_one_chunk() {
        let nal = vec![0xAB; 50];
        let chunks = chunk_nal(1, &nal);
        assert_eq!(chunks.len(), 1);
        let (h, payload) = ChunkHeader::decode(&chunks[0]).unwrap();
        assert_eq!(h, ChunkHeader { nal_id: 1, chunk_index: 0, is_last: true });
        assert_eq!(payload, &nal[..]);
    }

    #[test]
    fn large_nal_splits_across_multiple_chunks_in_order() {
        let nal: Vec<u8> = (0..3000u32).map(|i| (i % 256) as u8).collect();
        let chunks = chunk_nal(9, &nal);
        assert!(chunks.len() > 1);

        let mut reassembler = NalReassembler::new(16);
        let mut result = None;
        for chunk in &chunks {
            let (h, payload) = ChunkHeader::decode(chunk).unwrap();
            assert_eq!(h.nal_id, 9);
            result = reassembler.on_chunk(h, payload).or(result);
        }
        assert_eq!(result.unwrap(), nal);
    }

    #[test]
    fn splitter_finds_nal_units_delimited_by_3_and_4_byte_start_codes() {
        let mut stream = Vec::new();
        stream.extend_from_slice(&[0, 0, 0, 1]); // 4-byte start code
        stream.extend_from_slice(b"SPS_DATA");
        stream.extend_from_slice(&[0, 0, 1]); // 3-byte start code
        stream.extend_from_slice(b"PPS");
        stream.extend_from_slice(&[0, 0, 0, 1]);
        stream.extend_from_slice(b"SLICE");

        let mut splitter = AnnexBSplitter::new();
        // Feed incrementally in two pushes to exercise cross-call buffering.
        let mid = stream.len() / 2;
        let mut nals = splitter.push(&stream[..mid]);
        nals.extend(splitter.push(&stream[mid..]));

        assert_eq!(nals, vec![b"SPS_DATA".to_vec(), b"PPS".to_vec()]);
        // "SLICE" is the last NAL in the stream with no following start
        // code yet -- correctly not yet emitted (could still be growing).
    }

    #[test]
    fn reassembler_evicts_oldest_incomplete_nal_once_over_capacity() {
        let mut reassembler = NalReassembler::new(2);
        // Three NALs, each missing their final chunk -- none ever complete.
        for nal_id in 0..3u32 {
            let h = ChunkHeader { nal_id, chunk_index: 0, is_last: false };
            assert!(reassembler.on_chunk(h, b"partial").is_none());
        }
        assert!(reassembler.pending.len() <= 2, "must bound memory under sustained loss");
    }
}
