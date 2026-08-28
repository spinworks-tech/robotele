//! Discrete "set and hold" camera image-quality controls (brightness/
//! contrast/exposure/shutter). Channel C, mirrors `action_trigger`'s
//! reliable-stream + independent-seq-dedup pattern -- applying a change
//! means restarting the underlying `libcamera-vid` subprocess (it has no
//! live-parameter-update interface), so this is emphatically not a
//! per-tick continuous Channel B field, same reasoning as `ActionTrigger`.

/// Client(operator)-initiated bidi stream, next in the `0, 4, 8, ...`
/// category after `ActionTrigger`'s stream 4 (`DESIGN.md` §1.3.5's
/// numbering rule). Opened once per session; every control change for the
/// session's lifetime is a small write on this same stream, not a fresh
/// stream per change.
pub const CAMERA_CONTROL_STREAM_ID: u64 = 8;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraControl {
    pub brightness: f32,
    pub contrast: f32,
    pub ev: f32,
    /// Manual exposure time in microseconds; 0 = auto-exposure.
    pub shutter_us: u32,
    /// Independent monotonic identity -- not the same sequence space as
    /// `roboprotocol_proto::ChannelBFrame.seq` or `ActionTrigger.trigger_seq`.
    pub control_seq: u64,
}

/// Tracks the highest `control_seq` applied so far, so a duplicate (e.g. an
/// app-level retry racing the reliable stream's own delivery) isn't applied
/// twice -- same at-most-once reasoning as `action_trigger::TriggerDedup`,
/// kept as a separate type rather than a shared generic since the two
/// command shapes (discrete gait trigger vs. held image-control state)
/// aren't otherwise related.
#[derive(Debug, Default)]
pub struct CameraControlDedup {
    last_applied: Option<u64>,
}

impl CameraControlDedup {
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns `true` if `control` is new and should be applied, and
    /// records it as applied. Returns `false` for a duplicate or
    /// already-superseded `control_seq`.
    pub fn accept(&mut self, control: CameraControl) -> bool {
        let is_new = match self.last_applied {
            Some(last) => control.control_seq > last,
            None => true,
        };
        if is_new {
            self.last_applied = Some(control.control_seq);
        }
        is_new
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn control(control_seq: u64) -> CameraControl {
        CameraControl { brightness: 0.0, contrast: 1.0, ev: 0.0, shutter_us: 0, control_seq }
    }

    #[test]
    fn first_control_is_always_accepted() {
        let mut dedup = CameraControlDedup::new();
        assert!(dedup.accept(control(1)));
    }

    #[test]
    fn duplicate_control_seq_is_rejected() {
        let mut dedup = CameraControlDedup::new();
        assert!(dedup.accept(control(5)));
        assert!(!dedup.accept(control(5)), "must not re-apply the same control_seq twice");
    }

    #[test]
    fn out_of_order_older_control_seq_is_rejected() {
        let mut dedup = CameraControlDedup::new();
        assert!(dedup.accept(control(9)));
        assert!(!dedup.accept(control(3)), "an older control_seq must not re-apply after a newer one applied");
    }
}
