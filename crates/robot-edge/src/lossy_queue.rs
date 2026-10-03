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
//! - **Video:** a new delta NAL drops older deltas still waiting; a new IDR
//!   drops everything older except the latest SPS and PPS, which the decoder
//!   can't do without. A dropped delta glitches the picture until the next
//!   IDR, the same trade `video::channel_a` already makes upstream.
//! - **Sensors:** a new frame replaces the unsent slices of that sensor's
//!   previous frame. Slices already sent still decode on their own, so the
//!   operator sees a thinner frame, not a broken one.
//!
//! When both are waiting, video and sensors take turns one datagram at a
//! time, and sensors take turns among themselves, so neither class starves
//! the other. (Doc 13 leaves the choice of which to favour to the operator;
//! until that's negotiated, equal shares.)

use std::collections::{BTreeMap, VecDeque};

use roboprotocol_core::datagram::{self, DATAGRAM_TAG_CHANNEL_A};
use roboprotocol_core::video::chunk_nal;

/// Lossy datagrams allowed in quiche's send queue at once. Eight 1.1 KB
/// datagrams are about 4 ms at 20 Mbps -- the most a control datagram can
/// wait behind lossy data inside quiche. Kept above 1 so a packet is ready
/// whenever the congestion window opens.
pub const QUICHE_LOSSY_LIMIT: usize = 8;

const NAL_IDR: u8 = 5;
const NAL_SPS: u8 = 7;
const NAL_PPS: u8 = 8;

struct PendingNal {
    nal_type: u8,
    /// Tagged Channel A datagrams not yet handed to quiche.
    chunks: VecDeque<Vec<u8>>,
}

/// What was dropped before sending, for the end-of-session log.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct LossyStats {
    pub video_nals_dropped: u64,
    pub sensor_frames_cut: u64,
    pub sensor_slices_dropped: u64,
}

#[derive(Default)]
pub struct LossyQueue {
    video: VecDeque<PendingNal>,
    sensors: BTreeMap<u8, VecDeque<Vec<u8>>>,
    /// Which class `pop` tries first next time.
    video_turn: bool,
    last_sensor: Option<u8>,
    stats: LossyStats,
}

impl LossyQueue {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stats(&self) -> LossyStats {
        self.stats
    }

    /// Queues one NAL unit (without its start code), applying the drop
    /// rules in the module docs.
    pub fn push_video_nal(&mut self, nal_id: u32, nal: &[u8]) {
        let nal_type = nal.first().map_or(0, |b| b & 0x1F);
        let before = self.video.len();
        match nal_type {
            NAL_SPS | NAL_PPS => self.video.retain(|p| p.nal_type != nal_type),
            NAL_IDR => self.video.retain(|p| matches!(p.nal_type, NAL_SPS | NAL_PPS)),
            _ => self.video.retain(|p| matches!(p.nal_type, NAL_IDR | NAL_SPS | NAL_PPS)),
        }
        self.stats.video_nals_dropped += (before - self.video.len()) as u64;
        let chunks = chunk_nal(nal_id, nal).iter().map(|c| datagram::tag(DATAGRAM_TAG_CHANNEL_A, c)).collect();
        self.video.push_back(PendingNal { nal_type, chunks });
    }

    /// Queues one sensor frame's tagged slice datagrams, replacing whatever
    /// is still unsent of that sensor's previous frame.
    pub fn push_sensor_frame(&mut self, sensor_id: u8, datagrams: Vec<Vec<u8>>) {
        let pending = self.sensors.entry(sensor_id).or_default();
        if !pending.is_empty() {
            self.stats.sensor_frames_cut += 1;
            self.stats.sensor_slices_dropped += pending.len() as u64;
        }
        *pending = datagrams.into();
    }

    /// The next datagram to hand to quiche, alternating between video and
    /// sensors while both have something waiting.
    pub fn pop(&mut self) -> Option<Vec<u8>> {
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
        self.sensors.get_mut(&next)?.pop_front()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use roboprotocol_core::video::ChunkHeader;

    /// NAL type of the first chunk of each NAL left in the queue, in order.
    fn pending_types(q: &LossyQueue) -> Vec<u8> {
        q.video.iter().map(|p| p.nal_type).collect()
    }

    fn nal(nal_type: u8, len: usize) -> Vec<u8> {
        let mut n = vec![0xAA; len];
        n[0] = 0x60 | nal_type;
        n
    }

    #[test]
    fn a_new_delta_drops_waiting_deltas_but_keeps_critical_nals() {
        let mut q = LossyQueue::new();
        for (id, t) in [(0, NAL_SPS), (1, NAL_PPS), (2, NAL_IDR), (3, 1), (4, 1)] {
            q.push_video_nal(id, &nal(t, 50));
        }
        assert_eq!(pending_types(&q), vec![NAL_SPS, NAL_PPS, NAL_IDR, 1]);
        assert_eq!(q.stats().video_nals_dropped, 1);
    }

    #[test]
    fn a_new_idr_drops_everything_older_except_the_latest_parameter_sets() {
        let mut q = LossyQueue::new();
        for (id, t) in [(0, NAL_SPS), (1, NAL_PPS), (2, NAL_IDR), (3, 1), (4, NAL_SPS), (5, NAL_IDR)] {
            q.push_video_nal(id, &nal(t, 50));
        }
        assert_eq!(pending_types(&q), vec![NAL_PPS, NAL_SPS, NAL_IDR]);
        assert_eq!(q.stats().video_nals_dropped, 3, "older SPS, older IDR and the delta");
    }

    #[test]
    fn video_chunks_come_out_tagged_and_in_order() {
        let mut q = LossyQueue::new();
        let big = nal(NAL_IDR, 3_000);
        q.push_video_nal(9, &big);
        let mut chunks = Vec::new();
        while let Some(d) = q.pop() {
            assert_eq!(d[0], DATAGRAM_TAG_CHANNEL_A);
            let (h, _) = ChunkHeader::decode(&d[1..]).unwrap();
            chunks.push(h.chunk_index);
        }
        assert_eq!(chunks, vec![0, 1, 2]);
    }

    #[test]
    fn a_new_sensor_frame_replaces_the_unsent_rest_of_the_previous_one() {
        let mut q = LossyQueue::new();
        q.push_sensor_frame(1, vec![vec![1], vec![2], vec![3]]);
        assert_eq!(q.pop(), Some(vec![1]));
        q.push_sensor_frame(1, vec![vec![4], vec![5]]);
        assert_eq!(q.stats(), LossyStats { video_nals_dropped: 0, sensor_frames_cut: 1, sensor_slices_dropped: 2 });
        assert_eq!((q.pop(), q.pop(), q.pop()), (Some(vec![4]), Some(vec![5]), None));
    }

    #[test]
    fn video_and_sensors_alternate_and_sensors_round_robin() {
        let mut q = LossyQueue::new();
        q.push_video_nal(0, &nal(1, 3_000)); // three chunks
        q.push_sensor_frame(2, vec![vec![0x04, 2], vec![0x04, 2]]);
        q.push_sensor_frame(7, vec![vec![0x04, 7], vec![0x04, 7]]);
        let order: Vec<String> = std::iter::from_fn(|| q.pop())
            .map(|d| if d[0] == DATAGRAM_TAG_CHANNEL_A { "v".to_string() } else { format!("s{}", d[1]) })
            .collect();
        assert_eq!(order, vec!["s2", "v", "s7", "v", "s2", "v", "s7"]);
    }
}
