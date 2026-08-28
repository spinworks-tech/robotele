//! Discrete one-shot command triggers (FR-1.8, DESIGN.md §2.2/§2.3).
//!
//! Moved off Channel B (where a discrete trigger used to share a
//! sequence-staleness gate with the continuous fields it rode alongside,
//! risking a genuine new trigger being silently dropped with a stale
//! continuous update) onto its own Channel C reliable stream, which gives
//! it the delivery + ordering guarantees a one-shot event actually wants.

/// Client(operator)-initiated bidi stream, next in the `0, 4, 8, ...`
/// category after `HELLO`'s stream 0 (`DESIGN.md` §1.3.5's numbering
/// rule). Opened once per session; every trigger for the session's
/// lifetime is a small write on this same stream, not a fresh stream per
/// trigger.
pub const ACTION_TRIGGER_STREAM_ID: u64 = 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ActionTrigger {
    pub action_id: u8,
    /// Independent monotonic identity -- not the same sequence space as
    /// `roboprotocol_proto::ChannelBFrame.seq`.
    pub trigger_seq: u64,
}

/// Tracks the highest `trigger_seq` applied so far, so a duplicate (e.g. an
/// app-level retry racing the reliable stream's own delivery) isn't fired
/// twice. Unlike Channel B's staleness gate (FR-1.7), this is about
/// at-most-once application on a reliable, ordered stream, not about
/// discarding superseded continuous state.
#[derive(Debug, Default)]
pub struct TriggerDedup {
    last_applied: Option<u64>,
}

impl TriggerDedup {
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns `true` if `trigger` is new and should be applied, and
    /// records it as applied. Returns `false` for a duplicate or
    /// already-superseded `trigger_seq`.
    pub fn accept(&mut self, trigger: ActionTrigger) -> bool {
        let is_new = match self.last_applied {
            Some(last) => trigger.trigger_seq > last,
            None => true,
        };
        if is_new {
            self.last_applied = Some(trigger.trigger_seq);
        }
        is_new
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_trigger_is_always_accepted() {
        let mut dedup = TriggerDedup::new();
        assert!(dedup.accept(ActionTrigger { action_id: 2, trigger_seq: 1 }));
    }

    #[test]
    fn duplicate_trigger_seq_is_rejected() {
        let mut dedup = TriggerDedup::new();
        assert!(dedup.accept(ActionTrigger { action_id: 2, trigger_seq: 5 }));
        assert!(!dedup.accept(ActionTrigger { action_id: 2, trigger_seq: 5 }), "must not fire the same trigger twice");
    }

    #[test]
    fn out_of_order_older_trigger_seq_is_rejected() {
        let mut dedup = TriggerDedup::new();
        assert!(dedup.accept(ActionTrigger { action_id: 12, trigger_seq: 9 }));
        assert!(!dedup.accept(ActionTrigger { action_id: 2, trigger_seq: 3 }), "an older trigger_seq must not re-fire after a newer one applied");
    }

    #[test]
    fn distinct_increasing_triggers_all_fire() {
        let mut dedup = TriggerDedup::new();
        assert!(dedup.accept(ActionTrigger { action_id: 2, trigger_seq: 1 }));
        assert!(dedup.accept(ActionTrigger { action_id: 12, trigger_seq: 2 }));
        assert!(dedup.accept(ActionTrigger { action_id: 2, trigger_seq: 3 }), "same action_id repeated with a fresh trigger_seq is still a new event");
    }
}
