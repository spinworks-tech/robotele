//! Sensor slices (docs/13-large-sensor-payloads.md): carrying sensor frames
//! larger than one QUIC datagram -- point clouds, lidar, radar, depth maps --
//! as datagrams that each decode on their own.
//!
//! Unlike Channel A's `video::chunk_nal`, a frame here is never
//! fragmented-and-reassembled. It is a sequence of fixed-size *elements* (a
//! point, a lidar column, a depth-row segment), and slice `k` of `n` carries
//! elements `k, k + n, k + 2n, ...`. Losing a slice therefore costs `1/n` of
//! the elements spread evenly across the frame (lower density), rather than
//! a contiguous hole that could hide an obstacle, and rather than the whole
//! frame (fragmentation's all-or-nothing failure, see doc 13's loss table).
//!
//! Like `video::ChunkHeader`, the slice header is a small fixed binary
//! header, not FlatBuffers. All multi-byte fields, in the header and in the
//! encodings, are big-endian, matching the video chunk header.
//!
//! This module is transport-free: it builds and parses datagram bytes and
//! takes the current time as an argument, so `robot-edge` and
//! `operator-console` share the exact same rules.

use std::collections::BTreeMap;
use std::f32::consts::TAU;

use crate::datagram::DATAGRAM_TAG_SENSOR_SLICE;

/// sensor_id:u8, frame_seq:u32, capture_time_us:u64, slice_index:u16,
/// slice_count:u16. Doc 13's "18 bytes" includes the 1-byte datagram tag.
pub const SLICE_HEADER_LEN: usize = 1 + 4 + 8 + 2 + 2;

/// Slice payload size the sizing in doc 13 assumes: the same as Channel A's
/// video chunks, leaving room under Channel B's 1,200-byte datagram budget
/// (doc 02) for the tag, the header and future per-slice compression framing.
/// A sender may use more when `dgram_max_writable_len` allows.
pub const DEFAULT_SLICE_PAYLOAD: usize = crate::video::MAX_CHUNK_PAYLOAD;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SliceHeader {
    pub sensor_id: u8,
    /// Per-sensor frame counter. Not wrap-aware: at 30 Hz a `u32` lasts
    /// about 4.5 years, and every session starts a fresh `FrameAssembler`.
    pub frame_seq: u32,
    /// Robot-clock capture time (`timestamp::now_micros` on the robot). The
    /// operator maps it onto its own clock with `estimate_clock_offset_us`.
    pub capture_time_us: u64,
    pub slice_index: u16,
    pub slice_count: u16,
}

impl SliceHeader {
    pub fn encode(&self, out: &mut Vec<u8>) {
        out.push(self.sensor_id);
        out.extend_from_slice(&self.frame_seq.to_be_bytes());
        out.extend_from_slice(&self.capture_time_us.to_be_bytes());
        out.extend_from_slice(&self.slice_index.to_be_bytes());
        out.extend_from_slice(&self.slice_count.to_be_bytes());
    }

    /// Parses a slice (the datagram *after* its tag byte). Rejects a header
    /// whose `slice_index` isn't below a non-zero `slice_count`, so the
    /// assembler can index by it without further checks.
    pub fn decode(bytes: &[u8]) -> Option<(Self, &[u8])> {
        if bytes.len() < SLICE_HEADER_LEN {
            return None;
        }
        let header = Self {
            sensor_id: bytes[0],
            frame_seq: u32::from_be_bytes(bytes[1..5].try_into().ok()?),
            capture_time_us: u64::from_be_bytes(bytes[5..13].try_into().ok()?),
            slice_index: u16::from_be_bytes(bytes[13..15].try_into().ok()?),
            slice_count: u16::from_be_bytes(bytes[15..17].try_into().ok()?),
        };
        if header.slice_index >= header.slice_count {
            return None;
        }
        Some((header, &bytes[SLICE_HEADER_LEN..]))
    }
}

#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum SliceError {
    #[error("element size must be non-zero")]
    ZeroElementSize,
    #[error("a {element_size} B element doesn't fit a {max_payload} B slice")]
    ElementTooLarge { element_size: usize, max_payload: usize },
    #[error("{len} B isn't a whole number of {element_size} B elements")]
    PartialElement { len: usize, element_size: usize },
    #[error("frame needs {needed} slices, over the u16 limit")]
    TooManySlices { needed: usize },
}

/// How many slices `element_count` elements of `element_size` bytes need
/// when each slice carries at most `max_payload` bytes. An empty frame
/// (e.g. a point cloud with nothing in range) still takes one, empty, slice
/// so the operator sees the frame happened.
pub fn slice_count(element_count: usize, element_size: usize, max_payload: usize) -> Result<u16, SliceError> {
    if element_size == 0 {
        return Err(SliceError::ZeroElementSize);
    }
    let per_slice = max_payload / element_size;
    if per_slice == 0 {
        return Err(SliceError::ElementTooLarge { element_size, max_payload });
    }
    let needed = element_count.div_ceil(per_slice).max(1);
    u16::try_from(needed).map_err(|_| SliceError::TooManySlices { needed })
}

/// Splits a frame's element bytes into interleaved slice payloads: slice
/// `k` of `n` gets elements `k, k + n, k + 2n, ...`.
pub fn interleave(elements: &[u8], element_size: usize, max_payload: usize) -> Result<Vec<Vec<u8>>, SliceError> {
    if element_size == 0 {
        return Err(SliceError::ZeroElementSize);
    }
    if !elements.len().is_multiple_of(element_size) {
        return Err(SliceError::PartialElement { len: elements.len(), element_size });
    }
    let element_count = elements.len() / element_size;
    let n = slice_count(element_count, element_size, max_payload)? as usize;
    let mut slices: Vec<Vec<u8>> = (0..n).map(|k| Vec::with_capacity(element_count.saturating_sub(k).div_ceil(n) * element_size)).collect();
    for (i, element) in elements.chunks_exact(element_size).enumerate() {
        slices[i % n].extend_from_slice(element);
    }
    Ok(slices)
}

/// Builds the complete, tagged datagrams for one frame.
pub fn slice_frame(
    sensor_id: u8,
    frame_seq: u32,
    capture_time_us: u64,
    elements: &[u8],
    element_size: usize,
    max_payload: usize,
) -> Result<Vec<Vec<u8>>, SliceError> {
    let payloads = interleave(elements, element_size, max_payload)?;
    let slice_count = payloads.len() as u16; // bounded by `slice_count` above
    Ok(payloads
        .into_iter()
        .enumerate()
        .map(|(k, payload)| {
            let mut datagram = Vec::with_capacity(1 + SLICE_HEADER_LEN + payload.len());
            datagram.push(DATAGRAM_TAG_SENSOR_SLICE);
            SliceHeader { sensor_id, frame_seq, capture_time_us, slice_index: k as u16, slice_count }.encode(&mut datagram);
            datagram.extend_from_slice(&payload);
            datagram
        })
        .collect())
}

/// A frame as the operator receives it: complete, or partial once its
/// deadline passed. Missing slices are `None`.
#[derive(Debug, Clone, PartialEq)]
pub struct AssembledFrame {
    pub sensor_id: u8,
    pub frame_seq: u32,
    pub capture_time_us: u64,
    pub slices: Vec<Option<Vec<u8>>>,
}

/// A frame's elements back in their original order. Elements carried by a
/// missing slice are zero-filled and marked absent; zero is already "no
/// return" for range images and "no depth" for depth maps.
#[derive(Debug, Clone, PartialEq)]
pub struct Deinterleaved {
    pub bytes: Vec<u8>,
    pub present: Vec<bool>,
}

impl AssembledFrame {
    pub fn slices_received(&self) -> usize {
        self.slices.iter().filter(|s| s.is_some()).count()
    }

    pub fn is_complete(&self) -> bool {
        self.slices.iter().all(Option::is_some)
    }

    /// Fraction of slices received, 0.0..=1.0. Because slices interleave,
    /// this is also (to within one element per slice) the fraction of the
    /// frame's elements that arrived.
    pub fn completeness(&self) -> f32 {
        self.slices_received() as f32 / self.slices.len() as f32
    }

    /// Restores element order for a fixed-geometry frame (range image,
    /// depth map) of `element_count` elements. Bytes beyond `element_count`
    /// or a trailing partial element in a slice are ignored rather than
    /// trusted, since slice payloads come off the network.
    pub fn deinterleave(&self, element_size: usize, element_count: usize) -> Deinterleaved {
        let n = self.slices.len();
        let mut bytes = vec![0u8; element_count * element_size];
        let mut present = vec![false; element_count];
        if element_size == 0 {
            return Deinterleaved { bytes, present };
        }
        for (k, slice) in self.slices.iter().enumerate() {
            let Some(slice) = slice else { continue };
            for (j, element) in slice.chunks_exact(element_size).enumerate() {
                let idx = k + j * n;
                if idx >= element_count {
                    break;
                }
                bytes[idx * element_size..(idx + 1) * element_size].copy_from_slice(element);
                present[idx] = true;
            }
        }
        Deinterleaved { bytes, present }
    }

    /// Every received element, in slice order -- for point sets, where
    /// order doesn't matter and the element count varies per frame.
    pub fn received_elements(&self, element_size: usize) -> Vec<u8> {
        if element_size == 0 {
            return Vec::new();
        }
        let mut out = Vec::new();
        for slice in self.slices.iter().flatten() {
            let whole = slice.len() - slice.len() % element_size;
            out.extend_from_slice(&slice[..whole]);
        }
        out
    }
}

/// Counters for one sensor's `FrameAssembler`, for the operator's display.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AssemblyStats {
    pub frames_complete: u64,
    pub frames_partial: u64,
    /// Frames dropped unseen because a newer frame was emitted first.
    pub frames_superseded: u64,
    /// Slices for a frame already emitted or superseded.
    pub slices_late: u64,
    pub slices_duplicate: u64,
    /// Slices whose `slice_count` disagrees with earlier slices of their frame.
    pub slices_inconsistent: u64,
}

struct Pending {
    frame: AssembledFrame,
    first_seen_us: u64,
}

/// Assembles one sensor's frames from received slices.
///
/// A frame is emitted when all of its slices arrive, or -- partially -- once
/// `deadline_us` has passed since its first slice (`poll`), or when more
/// than `max_pending` frames are in flight (oldest first). Frames are only
/// ever emitted in increasing `frame_seq` order: once a frame is emitted,
/// anything older still pending is dropped as superseded, and late slices
/// for it are discarded. This is the same "latest wins" rule as Channel B
/// and the video sender -- a display has no use for an older frame.
pub struct FrameAssembler {
    deadline_us: u64,
    max_pending: usize,
    pending: BTreeMap<u32, Pending>,
    last_emitted: Option<u32>,
    stats: AssemblyStats,
}

impl FrameAssembler {
    pub fn new(deadline_us: u64, max_pending: usize) -> Self {
        Self { deadline_us, max_pending: max_pending.max(1), pending: BTreeMap::new(), last_emitted: None, stats: AssemblyStats::default() }
    }

    /// Doc 13's default deadline: 1.5 frame periods at `rate_hz`.
    pub fn deadline_for_rate(rate_hz: f32) -> u64 {
        (1.5e6 / rate_hz.max(0.1)) as u64
    }

    pub fn stats(&self) -> AssemblyStats {
        self.stats
    }

    /// Feeds one slice (post `SliceHeader::decode`). Returns frames ready
    /// to display, oldest first: the frame this slice completed, and any
    /// partial frame evicted to stay within `max_pending`.
    pub fn on_slice(&mut self, header: SliceHeader, payload: &[u8], now_us: u64) -> Vec<AssembledFrame> {
        if self.last_emitted.is_some_and(|last| header.frame_seq <= last) {
            self.stats.slices_late += 1;
            return Vec::new();
        }
        let entry = self.pending.entry(header.frame_seq).or_insert_with(|| Pending {
            frame: AssembledFrame {
                sensor_id: header.sensor_id,
                frame_seq: header.frame_seq,
                capture_time_us: header.capture_time_us,
                slices: vec![None; header.slice_count as usize],
            },
            first_seen_us: now_us,
        });
        if entry.frame.slices.len() != header.slice_count as usize {
            self.stats.slices_inconsistent += 1;
            return Vec::new();
        }
        let slot = &mut entry.frame.slices[header.slice_index as usize];
        if slot.is_some() {
            self.stats.slices_duplicate += 1;
            return Vec::new();
        }
        *slot = Some(payload.to_vec());

        let mut ready = Vec::new();
        if entry.frame.is_complete() {
            ready.push(self.emit(header.frame_seq));
        } else if self.pending.len() > self.max_pending {
            let oldest = *self.pending.keys().next().expect("pending is non-empty");
            ready.push(self.emit(oldest));
        }
        ready
    }

    /// Emits, partially, every pending frame whose deadline has passed.
    pub fn poll(&mut self, now_us: u64) -> Vec<AssembledFrame> {
        let expired: Vec<u32> = self
            .pending
            .iter()
            .filter(|(_, p)| now_us.saturating_sub(p.first_seen_us) >= self.deadline_us)
            .map(|(&seq, _)| seq)
            .collect();
        let mut ready = Vec::new();
        for seq in expired {
            // An earlier emit in this loop may already have superseded it.
            if self.pending.contains_key(&seq) {
                ready.push(self.emit(seq));
            }
        }
        ready
    }

    fn emit(&mut self, seq: u32) -> AssembledFrame {
        let frame = self.pending.remove(&seq).expect("emit called for a pending frame").frame;
        if frame.is_complete() {
            self.stats.frames_complete += 1;
        } else {
            self.stats.frames_partial += 1;
        }
        let older: Vec<u32> = self.pending.range(..seq).map(|(&s, _)| s).collect();
        for s in older {
            self.pending.remove(&s);
            self.stats.frames_superseded += 1;
        }
        self.last_emitted = Some(seq);
        frame
    }
}

// ---------------------------------------------------------------------
// Encodings. Each turns sensor data into a frame of fixed-size elements
// for `slice_frame`, and back.
// ---------------------------------------------------------------------

/// Quantizes `value / scale` to an i16, or `None` if it's out of range or
/// not finite. Out-of-range values are dropped, never clamped: clamping
/// would draw phantom points on the edge of the representable range.
fn quantize_i16(value: f32, scale: f32) -> Option<i16> {
    let q = (value / scale).round();
    (q.is_finite() && q >= i16::MIN as f32 && q <= i16::MAX as f32).then_some(q as i16)
}

/// Point sets: navigation clouds, filtered 3D lidar, radar detections.
/// Element = one point: int16 x, y, z in the sensor frame, then the
/// optional fields in this order.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PointFormat {
    /// Metres per unit: 0.01 covers ±327 m, 0.001 covers ±32 m.
    pub scale_m: f32,
    pub intensity: bool,
    /// Radar radial velocity, metres/second per unit, if carried.
    pub doppler_scale_mps: Option<f32>,
    pub snr: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Point {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub intensity: u8,
    pub doppler_mps: f32,
    pub snr: u8,
}

impl PointFormat {
    pub fn element_size(&self) -> usize {
        6 + self.intensity as usize + if self.doppler_scale_mps.is_some() { 2 } else { 0 } + self.snr as usize
    }

    /// Encodes `points` as elements. Returns the bytes and how many points
    /// were dropped for being out of range (or NaN).
    pub fn encode(&self, points: &[Point]) -> (Vec<u8>, usize) {
        let mut out = Vec::with_capacity(points.len() * self.element_size());
        let mut dropped = 0;
        for p in points {
            let xyz = (quantize_i16(p.x, self.scale_m), quantize_i16(p.y, self.scale_m), quantize_i16(p.z, self.scale_m));
            let doppler = match self.doppler_scale_mps {
                Some(scale) => quantize_i16(p.doppler_mps, scale).map(Some),
                None => Some(None),
            };
            let ((Some(x), Some(y), Some(z)), Some(doppler)) = (xyz, doppler) else {
                dropped += 1;
                continue;
            };
            for v in [x, y, z] {
                out.extend_from_slice(&v.to_be_bytes());
            }
            if self.intensity {
                out.push(p.intensity);
            }
            if let Some(d) = doppler {
                out.extend_from_slice(&d.to_be_bytes());
            }
            if self.snr {
                out.push(p.snr);
            }
        }
        (out, dropped)
    }

    /// Decodes elements (e.g. `AssembledFrame::received_elements`). A
    /// trailing partial element is ignored.
    pub fn decode(&self, bytes: &[u8]) -> Vec<Point> {
        let es = self.element_size();
        bytes
            .chunks_exact(es)
            .map(|e| {
                let i16_at = |o: usize| i16::from_be_bytes([e[o], e[o + 1]]) as f32;
                let mut p = Point { x: i16_at(0) * self.scale_m, y: i16_at(2) * self.scale_m, z: i16_at(4) * self.scale_m, ..Point::default() };
                let mut o = 6;
                if self.intensity {
                    p.intensity = e[o];
                    o += 1;
                }
                if let Some(scale) = self.doppler_scale_mps {
                    p.doppler_mps = i16_at(o) * scale;
                    o += 2;
                }
                if self.snr {
                    p.snr = e[o];
                }
                p
            })
            .collect()
    }
}

/// Organized lidar (3D, or 2D with `beams == 1`): a range per (beam,
/// column). Element = one column (one azimuth): `beams` uint16 ranges, then
/// `beams` uint8 intensities if carried. Interleaving by column means a
/// lost slice lowers angular resolution evenly instead of losing a wedge.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RangeImageFormat {
    pub beams: u16,
    pub columns: u16,
    /// Metres per unit: 0.001 reaches 65.5 m, 0.002 reaches 131 m.
    pub range_scale_m: f32,
    pub intensity: bool,
}

/// `ranges_m` and `intensity` are beam-major (`beam * columns + column`).
/// A range of 0.0 means no return, which is also what a lost column decodes to.
#[derive(Debug, Clone, PartialEq)]
pub struct RangeImage {
    pub beams: u16,
    pub columns: u16,
    pub ranges_m: Vec<f32>,
    pub intensity: Vec<u8>,
    pub columns_present: Vec<bool>,
}

impl RangeImageFormat {
    pub fn element_size(&self) -> usize {
        self.beams as usize * (2 + self.intensity as usize)
    }

    pub fn element_count(&self) -> usize {
        self.columns as usize
    }

    fn len(&self) -> usize {
        self.beams as usize * self.columns as usize
    }

    /// Encodes a beam-major range image. Ranges that don't fit the format
    /// (negative, NaN, or beyond its maximum) become "no return". Returns
    /// `None` if the inputs don't match the format's dimensions.
    pub fn encode(&self, ranges_m: &[f32], intensity: Option<&[u8]>) -> Option<Vec<u8>> {
        if ranges_m.len() != self.len() || (self.intensity && intensity.map(<[u8]>::len) != Some(self.len())) {
            return None;
        }
        let (beams, columns) = (self.beams as usize, self.columns as usize);
        let mut out = Vec::with_capacity(columns * self.element_size());
        for c in 0..columns {
            for b in 0..beams {
                let q = (ranges_m[b * columns + c] / self.range_scale_m).round();
                let q = if q.is_finite() && q > 0.0 && q <= u16::MAX as f32 { q as u16 } else { 0 };
                out.extend_from_slice(&q.to_be_bytes());
            }
            if let (true, Some(intensity)) = (self.intensity, intensity) {
                out.extend((0..beams).map(|b| intensity[b * columns + c]));
            }
        }
        Some(out)
    }

    pub fn decode(&self, frame: &Deinterleaved) -> RangeImage {
        let (beams, columns) = (self.beams as usize, self.columns as usize);
        let mut ranges_m = vec![0.0; self.len()];
        let mut intensity = vec![0u8; self.len()];
        for (c, e) in frame.bytes.chunks_exact(self.element_size().max(1)).enumerate().take(columns) {
            for b in 0..beams {
                ranges_m[b * columns + c] = u16::from_be_bytes([e[2 * b], e[2 * b + 1]]) as f32 * self.range_scale_m;
                if self.intensity {
                    intensity[b * columns + c] = e[2 * beams + b];
                }
            }
        }
        RangeImage { beams: self.beams, columns: self.columns, ranges_m, intensity, columns_present: frame.present.clone() }
    }
}

impl RangeImage {
    /// Converts returns to sensor-frame points. Column `c` is at azimuth
    /// `TAU * c / columns`; `beam_elevations_rad[b]` is beam `b`'s elevation.
    pub fn to_points(&self, beam_elevations_rad: &[f32]) -> Vec<Point> {
        let (beams, columns) = (self.beams as usize, self.columns as usize);
        let mut out = Vec::new();
        for (b, elevation) in beam_elevations_rad.iter().enumerate().take(beams) {
            let (sin_el, cos_el) = elevation.sin_cos();
            for c in 0..columns {
                let r = self.ranges_m[b * columns + c];
                if r <= 0.0 {
                    continue;
                }
                let (sin_az, cos_az) = (TAU * c as f32 / columns as f32).sin_cos();
                out.push(Point {
                    x: r * cos_el * cos_az,
                    y: r * cos_el * sin_az,
                    z: r * sin_el,
                    intensity: self.intensity.get(b * columns + c).copied().unwrap_or(0),
                    ..Point::default()
                });
            }
        }
        out
    }
}

/// Depth maps: uint16 millimetres per pixel, 0 = no depth. Element = a
/// `segment_width`-pixel run of one row, so wide rows still fit a slice and
/// slices pack efficiently; the last segment of a row is zero-padded when
/// `segment_width` doesn't divide `width`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DepthFormat {
    pub width: u16,
    pub height: u16,
    pub segment_width: u16,
}

/// Default depth segment: 32 pixels (64 B), 17 to a 1,100-byte slice.
pub const DEFAULT_DEPTH_SEGMENT_WIDTH: u16 = 32;

#[derive(Debug, Clone, PartialEq)]
pub struct DepthImage {
    pub width: u16,
    pub height: u16,
    /// Row-major, 0 = no depth (including pixels from lost slices).
    pub depth_mm: Vec<u16>,
}

impl DepthFormat {
    pub fn new(width: u16, height: u16) -> Self {
        Self { width, height, segment_width: DEFAULT_DEPTH_SEGMENT_WIDTH.min(width.max(1)) }
    }

    fn segments_per_row(&self) -> usize {
        (self.width as usize).div_ceil(self.segment_width.max(1) as usize)
    }

    pub fn element_size(&self) -> usize {
        self.segment_width as usize * 2
    }

    pub fn element_count(&self) -> usize {
        self.height as usize * self.segments_per_row()
    }

    /// Encodes a row-major depth map. Returns `None` if `depth_mm` doesn't
    /// match the format's dimensions.
    pub fn encode(&self, depth_mm: &[u16]) -> Option<Vec<u8>> {
        let (w, sw) = (self.width as usize, self.segment_width as usize);
        if depth_mm.len() != w * self.height as usize || sw == 0 {
            return None;
        }
        let mut out = Vec::with_capacity(self.element_count() * self.element_size());
        for row in depth_mm.chunks_exact(w) {
            for seg in 0..self.segments_per_row() {
                for x in seg * sw..(seg + 1) * sw {
                    out.extend_from_slice(&row.get(x).copied().unwrap_or(0).to_be_bytes());
                }
            }
        }
        Some(out)
    }

    pub fn decode(&self, frame: &Deinterleaved) -> DepthImage {
        let (w, sw, spr) = (self.width as usize, self.segment_width as usize, self.segments_per_row());
        let mut depth_mm = vec![0u16; w * self.height as usize];
        for (i, e) in frame.bytes.chunks_exact(self.element_size().max(1)).enumerate().take(self.element_count()) {
            let (y, seg) = (i / spr, i % spr);
            for (k, px) in e.chunks_exact(2).enumerate() {
                let x = seg * sw + k;
                if x < w {
                    depth_mm[y * w + x] = u16::from_be_bytes([px[0], px[1]]);
                }
            }
        }
        DepthImage { width: self.width, height: self.height, depth_mm }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn header(frame_seq: u32, slice_index: u16, slice_count: u16) -> SliceHeader {
        SliceHeader { sensor_id: 3, frame_seq, capture_time_us: 1_000 + frame_seq as u64, slice_index, slice_count }
    }

    /// Splits `slice_frame` output back into (header, payload) pairs.
    fn decoded(datagrams: &[Vec<u8>]) -> Vec<(SliceHeader, Vec<u8>)> {
        datagrams
            .iter()
            .map(|d| {
                assert_eq!(d[0], DATAGRAM_TAG_SENSOR_SLICE);
                let (h, p) = SliceHeader::decode(&d[1..]).unwrap();
                (h, p.to_vec())
            })
            .collect()
    }

    #[test]
    fn header_round_trips_and_is_18_bytes_with_the_tag() {
        let h = SliceHeader { sensor_id: 7, frame_seq: 0xDEAD_BEEF, capture_time_us: 0x0102_0304_0506_0708, slice_index: 4, slice_count: 9 };
        let mut buf = Vec::new();
        h.encode(&mut buf);
        assert_eq!(1 + buf.len(), 18);
        buf.extend_from_slice(b"rest");
        assert_eq!(SliceHeader::decode(&buf), Some((h, &b"rest"[..])));
    }

    #[test]
    fn header_decode_rejects_short_input_and_out_of_range_index() {
        let mut buf = Vec::new();
        header(1, 0, 2).encode(&mut buf);
        assert!(SliceHeader::decode(&buf[..SLICE_HEADER_LEN - 1]).is_none());
        for (index, count) in [(2, 2), (0, 0)] {
            let mut bad = Vec::new();
            header(1, index, count).encode(&mut bad);
            assert!(SliceHeader::decode(&bad).is_none(), "index {index} of {count} must be rejected");
        }
    }

    #[test]
    fn slice_counts_match_doc_13() {
        let p = DEFAULT_SLICE_PAYLOAD;
        let nav = PointFormat { scale_m: 0.01, intensity: true, doppler_scale_mps: None, snr: false };
        assert_eq!(slice_count(10_000, nav.element_size(), p), Ok(64));
        let lidar3d = RangeImageFormat { beams: 64, columns: 1024, range_scale_m: 0.002, intensity: true };
        assert_eq!(slice_count(lidar3d.element_count(), lidar3d.element_size(), p), Ok(205));
        let lidar2d = RangeImageFormat { beams: 1, columns: 720, range_scale_m: 0.001, intensity: true };
        assert_eq!(slice_count(lidar2d.element_count(), lidar2d.element_size(), p), Ok(2));
        let radar = PointFormat { scale_m: 0.01, intensity: false, doppler_scale_mps: Some(0.01), snr: true };
        assert_eq!(radar.element_size(), 9);
        assert_eq!(slice_count(300, radar.element_size(), p), Ok(3));
        let depth = DepthFormat::new(320, 240);
        assert_eq!(slice_count(depth.element_count(), depth.element_size(), p), Ok(142));
        let depth_vga = DepthFormat::new(640, 480);
        assert_eq!(slice_count(depth_vga.element_count(), depth_vga.element_size(), p), Ok(565));
    }

    #[test]
    fn slice_count_rejects_bad_sizes() {
        assert_eq!(slice_count(10, 0, 100), Err(SliceError::ZeroElementSize));
        assert_eq!(slice_count(10, 101, 100), Err(SliceError::ElementTooLarge { element_size: 101, max_payload: 100 }));
        assert_eq!(slice_count(70_000, 1, 1), Err(SliceError::TooManySlices { needed: 70_000 }));
        assert_eq!(slice_count(0, 4, 100), Ok(1), "an empty frame still sends one slice");
    }

    #[test]
    fn interleave_spreads_consecutive_elements_across_slices() {
        let elements: Vec<u8> = (0..10).collect();
        let slices = interleave(&elements, 1, 4).unwrap();
        assert_eq!(slices, vec![vec![0, 3, 6, 9], vec![1, 4, 7], vec![2, 5, 8]]);
        assert!(slices.iter().all(|s| s.len() <= 4));
        assert_eq!(interleave(&elements, 3, 9), Err(SliceError::PartialElement { len: 10, element_size: 3 }));
    }

    #[test]
    fn every_datagram_fits_the_channel_b_budget() {
        let elements = vec![0xAB; 9_600 * 64];
        for d in slice_frame(0, 0, 0, &elements, 64, DEFAULT_SLICE_PAYLOAD).unwrap() {
            assert!(d.len() <= 1200, "{} B datagram", d.len());
        }
    }

    #[test]
    fn complete_frame_round_trips_through_the_assembler_out_of_order() {
        let elements: Vec<u8> = (0..5_000u32).flat_map(|i| (i as u16).to_be_bytes()).collect();
        let mut slices = decoded(&slice_frame(3, 42, 9_999, &elements, 2, 300).unwrap());
        slices.reverse();
        let mut asm = FrameAssembler::new(100_000, 4);
        let mut ready = Vec::new();
        for (h, p) in &slices {
            ready.extend(asm.on_slice(*h, p, 0));
        }
        assert_eq!(ready.len(), 1);
        let frame = &ready[0];
        assert!(frame.is_complete());
        assert_eq!((frame.sensor_id, frame.frame_seq, frame.capture_time_us), (3, 42, 9_999));
        let d = frame.deinterleave(2, 5_000);
        assert_eq!(d.bytes, elements);
        assert!(d.present.iter().all(|&p| p));
        assert_eq!(asm.stats().frames_complete, 1);
    }

    #[test]
    fn a_lost_slice_thins_the_frame_evenly_instead_of_leaving_a_hole() {
        let elements: Vec<u8> = (0..100).collect();
        let slices = decoded(&slice_frame(0, 1, 0, &elements, 1, 10).unwrap());
        assert_eq!(slices.len(), 10);
        let mut asm = FrameAssembler::new(1_000, 4);
        for (h, p) in slices.iter().filter(|(h, _)| h.slice_index != 3) {
            assert!(asm.on_slice(*h, p, 0).is_empty());
        }
        let frame = asm.poll(1_000).pop().expect("deadline passed");
        assert_eq!(frame.slices_received(), 9);
        assert!((frame.completeness() - 0.9).abs() < 1e-6);
        let d = frame.deinterleave(1, 100);
        let missing: Vec<usize> = (0..100).filter(|&i| !d.present[i]).collect();
        assert_eq!(missing, vec![3, 13, 23, 33, 43, 53, 63, 73, 83, 93]);
        assert_eq!(frame.received_elements(1).len(), 90);
        assert_eq!(asm.stats().frames_partial, 1);
    }

    #[test]
    fn deadline_only_expires_frames_that_are_old_enough() {
        let mut asm = FrameAssembler::new(1_000, 8);
        asm.on_slice(header(1, 0, 2), b"a", 0);
        asm.on_slice(header(2, 0, 2), b"a", 600);
        assert!(asm.poll(999).is_empty());
        let ready = asm.poll(1_000);
        assert_eq!(ready.iter().map(|f| f.frame_seq).collect::<Vec<_>>(), vec![1]);
        assert_eq!(asm.poll(1_600).iter().map(|f| f.frame_seq).collect::<Vec<_>>(), vec![2]);
    }

    #[test]
    fn newer_frame_supersedes_older_pending_ones_and_late_slices_are_dropped() {
        let mut asm = FrameAssembler::new(1_000_000, 8);
        asm.on_slice(header(1, 0, 2), b"a", 0);
        asm.on_slice(header(2, 0, 2), b"a", 0);
        let ready = asm.on_slice(header(3, 0, 1), b"a", 0);
        assert_eq!(ready.iter().map(|f| f.frame_seq).collect::<Vec<_>>(), vec![3]);
        assert_eq!(asm.stats().frames_superseded, 2);
        assert!(asm.on_slice(header(1, 1, 2), b"b", 0).is_empty());
        assert!(asm.on_slice(header(3, 0, 1), b"a", 0).is_empty());
        assert_eq!(asm.stats().slices_late, 2);
        assert!(asm.poll(u64::MAX).is_empty(), "nothing older may be emitted after frame 3");
    }

    #[test]
    fn pending_frames_are_bounded_under_sustained_loss() {
        let mut asm = FrameAssembler::new(u64::MAX, 2);
        let mut emitted = Vec::new();
        for seq in 0..10 {
            emitted.extend(asm.on_slice(header(seq, 0, 2), b"a", 0).into_iter().map(|f| f.frame_seq));
            assert!(asm.pending.len() <= 2);
        }
        assert_eq!(emitted, (0..8).collect::<Vec<_>>(), "evicted oldest-first as partial frames");
    }

    #[test]
    fn duplicate_and_inconsistent_slices_are_counted_not_applied() {
        let mut asm = FrameAssembler::new(1_000, 4);
        asm.on_slice(header(1, 0, 3), b"a", 0);
        asm.on_slice(header(1, 0, 3), b"x", 0);
        asm.on_slice(header(1, 1, 4), b"y", 0);
        let frame = asm.poll(1_000).pop().unwrap();
        assert_eq!(frame.slices, vec![Some(b"a".to_vec()), None, None]);
        let stats = asm.stats();
        assert_eq!((stats.slices_duplicate, stats.slices_inconsistent), (1, 1));
    }

    #[test]
    fn deinterleave_ignores_oversized_slices_from_the_network() {
        let frame = AssembledFrame { sensor_id: 0, frame_seq: 0, capture_time_us: 0, slices: vec![Some(vec![1, 2, 3, 4, 5]), Some(vec![6])] };
        let d = frame.deinterleave(2, 3);
        assert_eq!(d.bytes, vec![1, 2, 0, 0, 3, 4]);
        assert_eq!(d.present, vec![true, false, true]);
        assert_eq!(frame.received_elements(2), vec![1, 2, 3, 4]);
    }

    #[test]
    fn points_round_trip_within_quantization_and_drop_out_of_range() {
        let fmt = PointFormat { scale_m: 0.01, intensity: true, doppler_scale_mps: Some(0.01), snr: true };
        let points = [
            Point { x: 1.234, y: -5.678, z: 0.5, intensity: 200, doppler_mps: -3.21, snr: 17 },
            Point { x: 400.0, ..Point::default() },
            Point { x: f32::NAN, ..Point::default() },
            Point { doppler_mps: 1_000.0, ..Point::default() },
        ];
        let (bytes, dropped) = fmt.encode(&points);
        assert_eq!(dropped, 3);
        assert_eq!(bytes.len(), fmt.element_size());
        let p = fmt.decode(&bytes)[0];
        for (got, want) in [(p.x, 1.23), (p.y, -5.68), (p.z, 0.5), (p.doppler_mps, -3.21)] {
            assert!((got - want).abs() < 1e-4, "{got} vs {want}");
        }
        assert_eq!((p.intensity, p.snr), (200, 17));
    }

    #[test]
    fn range_image_round_trips_and_lost_columns_read_as_no_return() {
        let fmt = RangeImageFormat { beams: 2, columns: 4, range_scale_m: 0.01, intensity: true };
        let ranges = [1.0, 2.0, 3.0, 700.0, 0.5, 0.0, -1.0, 4.0];
        let intensity = [10, 20, 30, 40, 50, 60, 70, 80];
        let bytes = fmt.encode(&ranges, Some(&intensity)).unwrap();
        assert_eq!(bytes.len(), fmt.element_count() * fmt.element_size());

        let slices = decoded(&slice_frame(0, 0, 0, &bytes, fmt.element_size(), fmt.element_size() * 2).unwrap());
        let mut frame = AssembledFrame { sensor_id: 0, frame_seq: 0, capture_time_us: 0, slices: slices.into_iter().map(|(_, p)| Some(p)).collect() };
        let img = fmt.decode(&frame.deinterleave(fmt.element_size(), fmt.element_count()));
        assert_eq!(img.ranges_m, vec![1.0, 2.0, 3.0, 0.0, 0.5, 0.0, 0.0, 4.0], "700 m and -1 m don't fit: no return");
        assert_eq!(img.intensity, intensity);

        frame.slices[1] = None; // columns 1 and 3
        let img = fmt.decode(&frame.deinterleave(fmt.element_size(), fmt.element_count()));
        assert_eq!(img.columns_present, vec![true, false, true, false]);
        assert_eq!(img.ranges_m, vec![1.0, 0.0, 3.0, 0.0, 0.5, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn range_image_rejects_mismatched_dimensions() {
        let fmt = RangeImageFormat { beams: 2, columns: 4, range_scale_m: 0.01, intensity: true };
        assert!(fmt.encode(&[1.0; 7], Some(&[0; 8])).is_none());
        assert!(fmt.encode(&[1.0; 8], None).is_none(), "format carries intensity");
    }

    #[test]
    fn range_image_to_points_places_returns_by_azimuth_and_elevation() {
        let img = RangeImage { beams: 1, columns: 4, ranges_m: vec![2.0, 0.0, 3.0, 0.0], intensity: vec![9, 0, 8, 0], columns_present: vec![true; 4] };
        let pts = img.to_points(&[0.0]);
        assert_eq!(pts.len(), 2, "no-return columns produce no points");
        assert!((pts[0].x - 2.0).abs() < 1e-5 && pts[0].y.abs() < 1e-5);
        assert!((pts[1].x + 3.0).abs() < 1e-5 && pts[1].y.abs() < 1e-4, "column 2 of 4 faces backwards");
        assert_eq!(pts[0].intensity, 9);
    }

    #[test]
    fn depth_round_trips_with_padded_last_segment() {
        let fmt = DepthFormat { width: 5, height: 2, segment_width: 2 };
        let depth: Vec<u16> = (1..=10).collect();
        let bytes = fmt.encode(&depth).unwrap();
        assert_eq!(bytes.len(), 6 * 4, "3 segments per row, the last one padded");
        let frame = AssembledFrame { sensor_id: 0, frame_seq: 0, capture_time_us: 0, slices: vec![Some(bytes)] };
        let img = fmt.decode(&frame.deinterleave(fmt.element_size(), fmt.element_count()));
        assert_eq!(img.depth_mm, depth);
        assert!(fmt.encode(&depth[..9]).is_none());
    }

    #[test]
    fn depth_default_segment_never_exceeds_the_width() {
        assert_eq!(DepthFormat::new(16, 4).segment_width, 16);
        assert_eq!(DepthFormat::new(640, 480).segment_width, 32);
    }
}
