//! Priority for the robot-to-operator datagram direction (docs/12 "Known
//! gap in Channel B", docs/13 "Rate control and priority").
//!
//! quiche has one first-in, first-out datagram send queue per connection,
//! with no priority. Video chunks and sensor slices handed straight to
//! `dgram_send` used to pile up there whenever the link couldn't carry them,
//! and every Channel B telemetry frame, heartbeat echo and E-Stop datagram
//! queued behind that backlog -- up to seconds of it on a slow uplink.
//!
//! So lossy datagrams wait here instead, and `Session::flush` tops quiche's
//! queue up from this one only while quiche holds fewer than
//! `QUICHE_LOSSY_LIMIT` datagrams. Control traffic still goes straight to
//! `dgram_send`, so it waits behind at most that many lossy datagrams,
//! whatever the video or sensor load.
//!
//! Waiting here rather than in quiche is also what lets stale data be
//! dropped before it is ever sent:
//!
//! - **Video:** a delta NAL is dropped only once it has waited longer than
//!   `VIDEO_MAX_WAIT`, and then every waiting delta goes, and later deltas
//!   are skipped until the next IDR. In H.264 each delta references the one
//!   before it, so after one is lost the rest of the group of pictures would
//!   only decode as smeared garbage: skipping them freezes the picture
//!   cleanly and spends the link on something useful instead. A new IDR
//!   drops everything older except the latest SPS and PPS, which the decoder
//!   can't do without, and ends any skip.
//!
//!   An earlier version dropped a waiting delta as soon as a newer one
//!   arrived. On the CM4 over Wi-Fi, with sensor load, that dropped 16% of
//!   deltas -- nearly all right after IDRs, whose 40-odd chunks briefly hold
//!   up the deltas behind them for one or two frame times -- and smeared
//!   almost every group of pictures.
//! - **Sensors:** a new frame replaces the unsent slices of that sensor's
//!   previous frame. Slices already sent still decode on their own, so the
//!   operator sees a thinner frame, not a broken one.
//!
//! When both are waiting, video and sensors take turns one datagram at a
//! time, and sensors take turns among themselves, so neither class starves
//! the other. (Doc 13 leaves the choice of which to favour to the operator;
//! until that's negotiated, equal shares.)

use std::collections::{BTreeMap, VecDeque};
use std::time::{Duration, Instant};

use roboprotocol_core::datagram::{self, DATAGRAM_TAG_CHANNEL_A};
use roboprotocol_core::video::chunk_nal;

/// Lossy datagrams allowed in quiche's send queue at once. Eight 1.1 KB
/// datagrams are about 4 ms at 20 Mbps -- the most a control datagram can
/// wait behind lossy data inside quiche. Kept above 1 so a packet is ready
/// whenever the congestion window opens.
pub const QUICHE_LOSSY_LIMIT: usize = 8;

/// How long a delta NAL may wait here before video skips to the next IDR.
/// Long enough to ride out an IDR burst (two or three frame times), short
/// enough that the operator never watches video more than this far behind.
pub const VIDEO_MAX_WAIT: Duration = Duration::from_millis(150);

const NAL_IDR: u8 = 5;
const NAL_SPS: u8 = 7;
const NAL_PPS: u8 = 8;

struct PendingNal {
    nal_type: u8,
    queued_at: Instant,
    /// Tagged Channel A datagrams not yet handed to quiche.
    chunks: VecDeque<Vec<u8>>,
}

/// What was dropped before sending, for the end-of-session log.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct LossyStats {
    pub video_nals_dropped: u64,
    /// `Cut`: frames whose unsent slices a newer frame replaced.
    pub sensor_frames_cut: u64,
    /// `Finish`: frames dropped whole, before any slice was sent.
    pub sensor_frames_skipped: u64,
    pub sensor_slices_dropped: u64,
}

/// What a sensor's queue does when a new frame arrives while the previous
/// one isn't fully handed to quiche yet (`robot-edge --sensor-frame-policy`).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub enum SensorFramePolicy {
    /// Replace whatever of the old frame is unsent: always the freshest data,
    /// but a link stall longer than a frame period cuts frames short.
    #[default]
    Cut,
    /// Let a frame that has started sending finish; keep only the newest of
    /// the frames waiting behind it. Frames arrive whole, at most about one
    /// frame period older.
    Finish,
}

impl SensorFramePolicy {
    pub fn parse(name: &str) -> Option<Self> {
        match name {
            "cut" => Some(Self::Cut),
            "finish" => Some(Self::Finish),
            _ => None,
        }
    }
}

/// One sensor's slices: the frame being sent, and under `Finish` at most
/// one whole frame waiting behind it.
#[derive(Default)]
struct SensorQueue {
    current: VecDeque<Vec<u8>>,
    /// Some of `current` has gone to quiche already.
    started: bool,
    waiting: Option<Vec<Vec<u8>>>,
}

impl SensorQueue {
    fn is_empty(&self) -> bool {
        self.current.is_empty() && self.waiting.is_none()
    }
}

#[derive(Default)]
pub struct LossyQueue {
    video: VecDeque<PendingNal>,
    sensors: BTreeMap<u8, SensorQueue>,
    sensor_policy: SensorFramePolicy,
    /// Set when a stale delta forced a drop: deltas are discarded until the
    /// next IDR, since they'd only decode as corrupted pictures.
    skip_deltas_until_idr: bool,
    /// Which class `pop` tries first next time.
    video_turn: bool,
    last_sensor: Option<u8>,
    stats: LossyStats,
}

impl LossyQueue {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_sensor_policy(sensor_policy: SensorFramePolicy) -> Self {
        Self { sensor_policy, ..Self::default() }
    }

    pub fn stats(&self) -> LossyStats {
        self.stats
    }

    /// Queues one NAL unit (without its start code), applying the drop
    /// rules in the module docs.
    pub fn push_video_nal(&mut self, nal_id: u32, nal: &[u8], now: Instant) {
        let nal_type = nal.first().map_or(0, |b| b & 0x1F);
        let before = self.video.len();
        match nal_type {
            NAL_SPS | NAL_PPS => self.video.retain(|p| p.nal_type != nal_type),
            NAL_IDR => {
                self.video.retain(|p| matches!(p.nal_type, NAL_SPS | NAL_PPS));
                self.skip_deltas_until_idr = false;
            }
            _ if self.skip_deltas_until_idr => {
                self.stats.video_nals_dropped += 1;
                return;
            }
            _ => {}
        }
        self.stats.video_nals_dropped += (before - self.video.len()) as u64;
        let chunks = chunk_nal(nal_id, nal).iter().map(|c| datagram::tag(DATAGRAM_TAG_CHANNEL_A, c)).collect();
        self.video.push_back(PendingNal { nal_type, queued_at: now, chunks });
    }

    /// If any waiting delta is older than `VIDEO_MAX_WAIT`, drops every
    /// waiting delta and skips deltas until the next IDR.
    fn expire_stale_video(&mut self, now: Instant) {
        let is_delta = |p: &PendingNal| !matches!(p.nal_type, NAL_IDR | NAL_SPS | NAL_PPS);
        let stale = self.video.iter().any(|p| is_delta(p) && now.saturating_duration_since(p.queued_at) > VIDEO_MAX_WAIT);
        if stale {
            let before = self.video.len();
            self.video.retain(|p| !is_delta(p));
            self.stats.video_nals_dropped += (before - self.video.len()) as u64;
            self.skip_deltas_until_idr = true;
        }
    }

    /// Queues one sensor frame's tagged slice datagrams, following the
    /// queue's `SensorFramePolicy`.
    pub fn push_sensor_frame(&mut self, sensor_id: u8, datagrams: Vec<Vec<u8>>) {
        let q = self.sensors.entry(sensor_id).or_default();
        let stats = &mut self.stats;
        match self.sensor_policy {
            SensorFramePolicy::Cut => {
                if !q.current.is_empty() {
                    stats.sensor_frames_cut += 1;
                    stats.sensor_slices_dropped += q.current.len() as u64;
                }
                q.current = datagrams.into();
                q.started = false;
            }
            SensorFramePolicy::Finish if q.current.is_empty() || !q.started => {
                if !q.current.is_empty() {
                    stats.sensor_frames_skipped += 1;
                    stats.sensor_slices_dropped += q.current.len() as u64;
                }
                q.current = datagrams.into();
                q.started = false;
            }
            SensorFramePolicy::Finish => {
                if let Some(old) = q.waiting.replace(datagrams) {
                    stats.sensor_frames_skipped += 1;
                    stats.sensor_slices_dropped += old.len() as u64;
                }
            }
        }
    }

    /// The next datagram to hand to quiche, alternating between video and
    /// sensors while both have something waiting.
    pub fn pop(&mut self, now: Instant) -> Option<Vec<u8>> {
        self.expire_stale_video(now);
        let first_video = self.video_turn;
        self.video_turn = !self.video_turn;
        if first_video {
            self.pop_video().or_else(|| self.pop_sensor())
        } else {
            self.pop_sensor().or_else(|| self.pop_video())
        }
    }

    fn pop_video(&mut self) -> Option<Vec<u8>> {
        let front = self.video.front_mut()?;
        let chunk = front.chunks.pop_front();
        if front.chunks.is_empty() {
            self.video.pop_front();
        }
        chunk
    }

    /// Round-robin across sensors with slices waiting.
    fn pop_sensor(&mut self) -> Option<Vec<u8>> {
        let after = self.last_sensor;
        let next = self
            .sensors
            .iter()
            .filter(|(_, q)| !q.is_empty())
            .map(|(&id, _)| id)
            .find(|&id| after.is_none_or(|last| id > last))
            .or_else(|| self.sensors.iter().find(|(_, q)| !q.is_empty()).map(|(&id, _)| id))?;
        self.last_sensor = Some(next);
        let q = self.sensors.get_mut(&next)?;
        if q.current.is_empty() {
            q.current = q.waiting.take()?.into();
            q.started = false;
        }
        let slice = q.current.pop_front();
        q.started = true;
        if q.current.is_empty() {
            if let Some(w) = q.waiting.take() {
                q.current = w.into();
                q.started = false;
            }
        }
        slice
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roboprotocol_core::video::ChunkHeader;

    /// NAL type of each NAL still waiting, in order.
    fn pending_types(q: &LossyQueue) -> Vec<u8> {
        q.video.iter().map(|p| p.nal_type).collect()
    }

    fn nal(nal_type: u8, len: usize) -> Vec<u8> {
        let mut n = vec![0xAA; len];
        n[0] = 0x60 | nal_type;
        n
    }

    const MS: Duration = Duration::from_millis(1);

    #[test]
    fn fresh_deltas_wait_behind_an_idr_instead_of_being_dropped() {
        let t0 = Instant::now();
        let mut q = LossyQueue::new();
        for (id, t) in [(0, NAL_SPS), (1, NAL_PPS), (2, NAL_IDR), (3, 1), (4, 1)] {
            q.push_video_nal(id, &nal(t, 50), t0 + id * 33 * MS);
        }
        assert!(q.pop(t0 + 140 * MS).is_some(), "oldest delta waited 41 ms: nothing is stale yet");
        assert_eq!(pending_types(&q), vec![NAL_PPS, NAL_IDR, 1, 1]);
        assert_eq!(q.stats().video_nals_dropped, 0);
    }

    #[test]
    fn a_stale_delta_drops_waiting_deltas_and_skips_to_the_next_idr() {
        let t0 = Instant::now();
        let mut q = LossyQueue::new();
        q.push_video_nal(0, &nal(NAL_IDR, 50), t0);
        q.push_video_nal(1, &nal(1, 50), t0);
        q.push_video_nal(2, &nal(1, 50), t0 + 33 * MS);
        // 151 ms later the first delta is stale: both deltas go, the IDR stays.
        assert!(q.pop(t0 + 151 * MS).is_some());
        assert_eq!(pending_types(&q), Vec::<u8>::new(), "the IDR was the one popped");
        assert_eq!(q.stats().video_nals_dropped, 2);
        // Later deltas in the same group of pictures are skipped...
        q.push_video_nal(3, &nal(1, 50), t0 + 160 * MS);
        assert_eq!(q.stats().video_nals_dropped, 3);
        assert!(pending_types(&q).is_empty());
        // ...until the next IDR, after which deltas flow again.
        q.push_video_nal(4, &nal(NAL_IDR, 50), t0 + 200 * MS);
        q.push_video_nal(5, &nal(1, 50), t0 + 233 * MS);
        assert_eq!(pending_types(&q), vec![NAL_IDR, 1]);
    }

    #[test]
    fn a_waiting_idr_never_goes_stale() {
        let t0 = Instant::now();
        let mut q = LossyQueue::new();
        q.push_video_nal(0, &nal(NAL_IDR, 3_000), t0); // three chunks
        assert!(q.pop(t0 + 10_000 * MS).is_some());
        assert_eq!(pending_types(&q), vec![NAL_IDR]);
        assert_eq!(q.stats().video_nals_dropped, 0);
    }

    #[test]
    fn a_new_idr_drops_everything_older_except_the_latest_parameter_sets() {
        let t0 = Instant::now();
        let mut q = LossyQueue::new();
        for (id, t) in [(0, NAL_SPS), (1, NAL_PPS), (2, NAL_IDR), (3, 1), (4, NAL_SPS), (5, NAL_IDR)] {
            q.push_video_nal(id, &nal(t, 50), t0);
        }
        assert_eq!(pending_types(&q), vec![NAL_PPS, NAL_SPS, NAL_IDR]);
        assert_eq!(q.stats().video_nals_dropped, 3, "older SPS, older IDR and the delta");
    }

    #[test]
    fn video_chunks_come_out_tagged_and_in_order() {
        let t0 = Instant::now();
        let mut q = LossyQueue::new();
        q.push_video_nal(9, &nal(NAL_IDR, 3_000), t0);
        let mut chunks = Vec::new();
        while let Some(d) = q.pop(t0) {
            assert_eq!(d[0], DATAGRAM_TAG_CHANNEL_A);
            let (h, _) = ChunkHeader::decode(&d[1..]).unwrap();
            chunks.push(h.chunk_index);
        }
        assert_eq!(chunks, vec![0, 1, 2]);
    }

    #[test]
    fn a_new_sensor_frame_replaces_the_unsent_rest_of_the_previous_one() {
        let t0 = Instant::now();
        let mut q = LossyQueue::new();
        q.push_sensor_frame(1, vec![vec![1], vec![2], vec![3]]);
        assert_eq!(q.pop(t0), Some(vec![1]));
        q.push_sensor_frame(1, vec![vec![4], vec![5]]);
        assert_eq!(q.stats(), LossyStats { sensor_frames_cut: 1, sensor_slices_dropped: 2, ..LossyStats::default() });
        assert_eq!((q.pop(t0), q.pop(t0), q.pop(t0)), (Some(vec![4]), Some(vec![5]), None));
    }

    #[test]
    fn finish_lets_a_started_frame_complete_and_keeps_only_the_newest_waiting() {
        let t0 = Instant::now();
        let mut q = LossyQueue::with_sensor_policy(SensorFramePolicy::Finish);
        q.push_sensor_frame(1, vec![vec![1], vec![2], vec![3]]);
        assert_eq!(q.pop(t0), Some(vec![1])); // frame 1 has started
        q.push_sensor_frame(1, vec![vec![4], vec![5]]); // waits
        q.push_sensor_frame(1, vec![vec![6], vec![7]]); // replaces the waiting one
        let rest: Vec<_> = std::iter::from_fn(|| q.pop(t0)).collect();
        assert_eq!(rest, vec![vec![2], vec![3], vec![6], vec![7]], "frame 1 finishes, then the newest frame");
        assert_eq!(q.stats(), LossyStats { sensor_frames_skipped: 1, sensor_slices_dropped: 2, ..LossyStats::default() });
    }

    #[test]
    fn finish_replaces_a_frame_that_has_not_started() {
        let t0 = Instant::now();
        let mut q = LossyQueue::with_sensor_policy(SensorFramePolicy::Finish);
        q.push_sensor_frame(1, vec![vec![1], vec![2]]);
        q.push_sensor_frame(1, vec![vec![3]]);
        assert_eq!((q.pop(t0), q.pop(t0)), (Some(vec![3]), None));
        assert_eq!(q.stats().sensor_frames_skipped, 1);
    }

    #[test]
    fn video_and_sensors_alternate_and_sensors_round_robin() {
        let t0 = Instant::now();
        let mut q = LossyQueue::new();
        q.push_video_nal(0, &nal(1, 3_000), t0); // three chunks
        q.push_sensor_frame(2, vec![vec![0x04, 2], vec![0x04, 2]]);
        q.push_sensor_frame(7, vec![vec![0x04, 7], vec![0x04, 7]]);
        let order: Vec<String> = std::iter::from_fn(|| q.pop(t0))
            .map(|d| if d[0] == DATAGRAM_TAG_CHANNEL_A { "v".to_string() } else { format!("s{}", d[1]) })
            .collect();
        assert_eq!(order, vec!["s2", "v", "s7", "v", "s2", "v", "s7"]);
    }
}
