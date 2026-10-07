//! Freeze, don't smear: which reassembled NALs reach the decoder.
//!
//! In H.264 every delta frame references the one before it, so once a delta
//! is missing, every later delta in its group of pictures decodes as smeared
//! garbage until the next IDR. Deltas go missing in three places: in the air
//! (one lost chunk loses its NAL), on the robot (`robot-edge`'s lossy queue
//! drops stale deltas and skips to the next IDR), and here, when playback
//! can't keep up. `DeltaGate` sees the robot's sequential `nal_id`s, so any
//! gap is visible: after one, it holds deltas back until the next IDR and
//! the picture freezes cleanly instead. Measured on the CM4 over Wi-Fi
//! (docs/12, "Results on the CM4 over Wi-Fi"), deltas decoding as smear were
//! a large share of all deltas under load.
//!
//! `DeltaSlot` is the hand-off to a playback backend: a short queue of
//! deltas. It never silently drops a delta the decoder hasn't taken -- that
//! is exactly a gap -- so when the decoder falls behind by more than
//! `DELTA_QUEUE_CAP` deltas it discards them all and tells the gate. The cap
//! isn't 1: deltas routinely arrive in bursts (after an IDR drains on the
//! robot, or several in one batch of datagrams) faster than the playback
//! task gets scheduled, and a one-delta slot froze the picture on every
//! burst (measured on the CM4: 551 frames shown in 30 s instead of ~800).

use std::collections::VecDeque;
use std::sync::Mutex;

use tokio::sync::Notify;

/// A `nal_id` this far below the expected one isn't a late NAL but a new
/// stream (the robot restarted its encoder): start over from it.
const RESTART_GAP: u32 = 1_000;

const NAL_IDR: u8 = 5;
const NAL_SPS: u8 = 7;
const NAL_PPS: u8 = 8;

/// Counts for the HUD / logs.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct GateStats {
    /// Times a gap (or a playback overflow) started a freeze.
    pub freezes: u64,
    /// Deltas held back while frozen.
    pub deltas_held: u64,
}

#[derive(Default)]
pub struct DeltaGate {
    /// The `nal_id` expected next, once one has been seen.
    next_id: Option<u32>,
    frozen: bool,
    stats: GateStats,
}

impl DeltaGate {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn stats(&self) -> GateStats {
        self.stats
    }

    /// Whether this NAL (`nal_type` = `nal[0] & 0x1F`, without start code)
    /// should go to the decoder. SPS/PPS always do; an IDR always does and
    /// ends a freeze; a delta only while not frozen. A NAL older than one
    /// already seen arrived late, after its successors, and is dropped.
    pub fn admit(&mut self, nal_id: u32, nal_type: u8) -> bool {
        if self.next_id.is_some_and(|expected| nal_id.saturating_add(RESTART_GAP) < expected) {
            *self = Self { stats: self.stats, ..Self::default() };
        }
        if let Some(expected) = self.next_id {
            if nal_id < expected {
                return matches!(nal_type, NAL_SPS | NAL_PPS);
            }
            if nal_id > expected {
                self.freeze();
            }
        }
        self.next_id = Some(nal_id.wrapping_add(1));
        match nal_type {
            NAL_SPS | NAL_PPS => true,
            NAL_IDR => {
                self.frozen = false;
                true
            }
            _ if self.frozen => {
                self.stats.deltas_held += 1;
                false
            }
            _ => true,
        }
    }

    /// Playback had to discard deltas (see `DeltaSlot::offer`): hold deltas
    /// until the next IDR.
    pub fn playback_overflowed(&mut self) {
        self.freeze();
    }

    fn freeze(&mut self) {
        if !self.frozen {
            self.frozen = true;
            self.stats.freezes += 1;
        }
    }
}

/// Deltas the playback backend may hold before it counts as falling
/// behind: about a quarter of a second at 30 fps.
pub const DELTA_QUEUE_CAP: usize = 8;

/// Deltas waiting for a playback backend's decoder.
#[derive(Default)]
pub struct DeltaSlot {
    queue: Mutex<VecDeque<Vec<u8>>>,
    /// Wakes an async consumer (the ffplay feeder); the native render
    /// thread polls `take` instead.
    pub ready: Notify,
}

impl DeltaSlot {
    pub fn new() -> Self {
        Self::default()
    }

    /// Hands a delta over. Returns `false` if `DELTA_QUEUE_CAP` deltas were
    /// already waiting: the decoder has fallen behind, so every waiting delta
    /// and this one are discarded (none of them could follow cleanly from
    /// what's left) and the caller should freeze until the next IDR.
    pub fn offer(&self, bytes: Vec<u8>) -> bool {
        let mut queue = self.queue.lock().unwrap();
        if queue.len() >= DELTA_QUEUE_CAP {
            queue.clear();
            return false;
        }
        queue.push_back(bytes);
        drop(queue);
        self.ready.notify_one();
        true
    }

    /// A critical NAL (SPS/PPS/IDR) is about to go to the decoder: deltas
    /// still waiting belong to the previous group of pictures and would
    /// reach the decoder after the new IDR, so drop them.
    pub fn clear(&self) {
        self.queue.lock().unwrap().clear();
    }

    pub fn take(&self) -> Option<Vec<u8>> {
        self.queue.lock().unwrap().pop_front()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const DELTA: u8 = 1;

    #[test]
    fn deltas_flow_while_ids_are_consecutive() {
        let mut g = DeltaGate::new();
        for (id, t) in [(0, NAL_SPS), (1, NAL_PPS), (2, NAL_IDR), (3, DELTA), (4, DELTA)] {
            assert!(g.admit(id, t), "NAL {id}");
        }
        assert_eq!(g.stats(), GateStats::default());
    }

    #[test]
    fn a_gap_freezes_deltas_until_the_next_idr() {
        let mut g = DeltaGate::new();
        assert!(g.admit(10, NAL_IDR));
        assert!(g.admit(11, DELTA));
        assert!(!g.admit(13, DELTA), "12 is missing: 13 would smear");
        assert!(!g.admit(14, DELTA));
        assert!(g.admit(15, NAL_SPS), "parameter sets always pass");
        assert!(g.admit(16, NAL_IDR), "an IDR ends the freeze");
        assert!(g.admit(17, DELTA));
        assert_eq!(g.stats(), GateStats { freezes: 1, deltas_held: 2 });
    }

    #[test]
    fn a_missing_delta_right_before_an_idr_costs_nothing() {
        let mut g = DeltaGate::new();
        assert!(g.admit(0, NAL_IDR));
        assert!(g.admit(2, NAL_IDR), "the gap freezes, the IDR unfreezes at once");
        assert!(g.admit(3, DELTA));
        assert_eq!(g.stats().deltas_held, 0);
    }

    #[test]
    fn a_late_nal_is_dropped_unless_it_is_a_parameter_set() {
        let mut g = DeltaGate::new();
        assert!(g.admit(5, NAL_IDR));
        assert!(!g.admit(7, DELTA)); // 6 missing so far: freeze
        assert!(!g.admit(6, DELTA), "arrived after 7: too late to show");
        assert!(g.admit(6, NAL_PPS));
    }

    #[test]
    fn ids_restarting_from_zero_start_a_new_stream() {
        let mut g = DeltaGate::new();
        assert!(g.admit(5_000, NAL_IDR));
        assert!(g.admit(5_001, DELTA));
        assert!(g.admit(0, NAL_SPS));
        assert!(g.admit(1, NAL_PPS));
        assert!(g.admit(2, NAL_IDR), "a restarted stream's IDR must get through");
        assert!(g.admit(3, DELTA));
    }

    #[test]
    fn playback_overflow_freezes_until_the_next_idr() {
        let mut g = DeltaGate::new();
        assert!(g.admit(0, NAL_IDR));
        g.playback_overflowed();
        assert!(!g.admit(1, DELTA));
        assert!(g.admit(2, NAL_IDR));
        assert!(g.admit(3, DELTA));
    }

    #[test]
    fn a_burst_of_deltas_queues_in_order() {
        let slot = DeltaSlot::new();
        for i in 0..DELTA_QUEUE_CAP as u8 {
            assert!(slot.offer(vec![i]));
        }
        assert_eq!(slot.take(), Some(vec![0]));
        assert_eq!(slot.take(), Some(vec![1]));
    }

    #[test]
    fn falling_behind_discards_every_waiting_delta() {
        let slot = DeltaSlot::new();
        for i in 0..DELTA_QUEUE_CAP as u8 {
            slot.offer(vec![i]);
        }
        assert!(!slot.offer(vec![99]), "one more than the decoder can be behind");
        assert_eq!(slot.take(), None, "none of them can be shown cleanly");
        assert!(slot.offer(vec![100]));
        assert_eq!(slot.take(), Some(vec![100]));
    }

    #[test]
    fn clear_drops_deltas_left_over_from_the_previous_picture_group() {
        let slot = DeltaSlot::new();
        slot.offer(vec![1]);
        slot.clear();
        assert_eq!(slot.take(), None);
    }
}
