//! Timestamp-based motion reconstruction (FR-1.9, DESIGN.md §2.2.2).
//!
//! Pure math over the two most recently received timestamped samples of a
//! continuous field vector (e.g. `VelocityAttitudeCommand`'s 6 floats,
//! `CartesianCommand`'s 3 position floats, or N per-joint angles --
//! anything already decoded to `&[f32]`). Reconstructs the value at a
//! query time that lags "now" by a small bounded interpolation delay,
//! instead of stepping directly to each newly-arrived sample at its
//! *arrival* time -- which is what makes evenly-paced operator motion
//! look like it's accelerating/decelerating under jittery arrival timing.
//!
//! Sequence-gating (FR-1.7, `datagram`/robot-edge's dispatch path) has
//! already run by the time a sample reaches this module -- `MotionBuffer`
//! is never handed a stale sample to begin with, so it has no ordering
//! logic of its own.
//!
//! **Not used for the Haptic category** (FR-1.9.3): TDPA's passivity
//! observer needs real-time force data, not delayed/smoothed data.
//!
//! This module is the reconstruction *algorithm* only. Wiring it into a
//! live, periodic per-tick control loop in `robot-edge` (as opposed to
//! today's immediate-dispatch-on-datagram-arrival model) is a separate
//! integration task this module does not attempt.

use crate::timestamp::Timestamp64;

#[derive(Debug, Clone, PartialEq)]
pub struct TimestampedSample {
    /// Sender capture time (`roboprotocol_core::timestamp`), not arrival time.
    pub capture_us: Timestamp64,
    /// The decoded continuous field vector at that capture time. Callers
    /// own the meaning of each index (e.g. `VelocityAttitudeCommand`'s
    /// vx/vy/turn/roll/pitch/yaw order) -- this module only interpolates.
    pub values: Vec<f32>,
}

/// Holds the most recent two samples for one continuous field group (one
/// per (region, category) pair, mirroring FR-1.7's own tracking
/// granularity) and answers "what should this be at time T" queries.
#[derive(Debug, Default)]
pub struct MotionBuffer {
    previous: Option<TimestampedSample>,
    latest: Option<TimestampedSample>,
}

impl MotionBuffer {
    pub fn new() -> Self {
        Self::default()
    }

    /// Feed a newly-decoded, already sequence-gated sample.
    pub fn push(&mut self, sample: TimestampedSample) {
        self.previous = self.latest.take();
        self.latest = Some(sample);
    }

    /// Reconstruct the field vector at `query_us`. Returns `None` when
    /// there is nothing new enough to bracket against at all -- the
    /// sender has actually stopped producing samples, not just delivered
    /// them unevenly (DESIGN.md §2.2.2's "falls back to decay" case; that
    /// fallback is the caller's responsibility, not this module's).
    ///
    /// `max_extrapolation_us` bounds how far past the newest sample a
    /// query is allowed to reach before giving up (rather than
    /// extrapolating indefinitely along a trajectory that may no longer
    /// be true).
    pub fn reconstruct_at(&self, query_us: Timestamp64, max_extrapolation_us: u64) -> Option<Vec<f32>> {
        match (&self.previous, &self.latest) {
            (Some(prev), Some(latest)) if prev.capture_us < latest.capture_us => {
                let span = (latest.capture_us - prev.capture_us) as f64;
                let t = if query_us <= latest.capture_us {
                    // Between (or at/before) the two samples: interpolate,
                    // clamped so a query older than `prev` just holds `prev`
                    // rather than extrapolating backward.
                    (query_us.saturating_sub(prev.capture_us) as f64 / span).clamp(0.0, 1.0)
                } else {
                    // Query is newer than the newest sample (the normal
                    // case: query_us = now - interpolation_delay, and
                    // interpolation_delay is usually chosen larger than
                    // one-way network latency) -- extrapolate along the
                    // same trajectory, bounded.
                    let ahead_us = query_us - latest.capture_us;
                    if ahead_us > max_extrapolation_us {
                        return None;
                    }
                    1.0 + ahead_us as f64 / span
                };
                Some(lerp(&prev.values, &latest.values, t))
            }
            (_, Some(latest)) => {
                // Only one sample ever received (or `previous`/`latest`
                // share a capture time, e.g. a duplicate) -- can't
                // interpolate a trajectory, so hold it if the query isn't
                // unreasonably far past it.
                let ahead_us = query_us.saturating_sub(latest.capture_us);
                (query_us >= latest.capture_us && ahead_us <= max_extrapolation_us || query_us < latest.capture_us)
                    .then(|| latest.values.clone())
            }
            (_, None) => None,
        }
    }
}

fn lerp(a: &[f32], b: &[f32], t: f64) -> Vec<f32> {
    a.iter().zip(b.iter()).map(|(&x, &y)| (x as f64 + (y as f64 - x as f64) * t) as f32).collect()
}

/// FR-1.9.1's default: roughly 1.5 expected sample periods at
/// `control_rate_hz`, matching the "1-2 control periods" scaling already
/// used for the dead-reckoning decay window (DESIGN.md §2.2) so the two
/// mechanisms share one mental model.
pub fn default_interpolation_delay_us(control_rate_hz: u16) -> u64 {
    if control_rate_hz == 0 {
        return 0;
    }
    (1_500_000 / control_rate_hz as u64).max(1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn interpolates_linearly_between_two_bracketing_samples() {
        let mut buf = MotionBuffer::new();
        buf.push(TimestampedSample { capture_us: 0, values: vec![0.0, 10.0] });
        buf.push(TimestampedSample { capture_us: 1000, values: vec![10.0, 20.0] });

        let mid = buf.reconstruct_at(500, 1000).unwrap();
        assert_eq!(mid, vec![5.0, 15.0], "halfway between the two samples in time");

        let quarter = buf.reconstruct_at(250, 1000).unwrap();
        assert_eq!(quarter, vec![2.5, 12.5]);
    }

    #[test]
    fn query_before_first_sample_holds_the_first_sample() {
        let mut buf = MotionBuffer::new();
        buf.push(TimestampedSample { capture_us: 1000, values: vec![5.0] });
        buf.push(TimestampedSample { capture_us: 2000, values: vec![15.0] });

        assert_eq!(buf.reconstruct_at(0, 1000).unwrap(), vec![5.0], "must not extrapolate backward past the oldest sample");
    }

    #[test]
    fn query_past_newest_sample_extrapolates_within_bound() {
        let mut buf = MotionBuffer::new();
        buf.push(TimestampedSample { capture_us: 0, values: vec![0.0] });
        buf.push(TimestampedSample { capture_us: 1000, values: vec![10.0] });

        // Same trajectory (10 units / 1000us) continued 200us past the
        // newest sample.
        let extrapolated = buf.reconstruct_at(1200, 1000).unwrap();
        assert!((extrapolated[0] - 12.0).abs() < 1e-6);
    }

    #[test]
    fn query_too_far_past_newest_sample_returns_none_for_decay_fallback() {
        let mut buf = MotionBuffer::new();
        buf.push(TimestampedSample { capture_us: 0, values: vec![0.0] });
        buf.push(TimestampedSample { capture_us: 1000, values: vec![10.0] });

        assert!(buf.reconstruct_at(5000, 1000).is_none(), "sender has actually stopped producing samples -- caller should fall back to dead-reckoning decay");
    }

    #[test]
    fn single_sample_holds_until_the_extrapolation_bound() {
        let mut buf = MotionBuffer::new();
        buf.push(TimestampedSample { capture_us: 1000, values: vec![7.0] });

        assert_eq!(buf.reconstruct_at(1000, 500).unwrap(), vec![7.0]);
        assert_eq!(buf.reconstruct_at(1400, 500).unwrap(), vec![7.0], "held within bound, no second sample to interpolate against");
        assert!(buf.reconstruct_at(2000, 500).is_none());
    }

    #[test]
    fn empty_buffer_has_nothing_to_reconstruct() {
        let buf = MotionBuffer::new();
        assert!(buf.reconstruct_at(1000, 1000).is_none());
    }

    #[test]
    fn default_delay_scales_inversely_with_rate_like_the_decay_window() {
        // 100Hz -> 10ms period -> ~15ms delay (1.5 periods); matches the
        // DESIGN.md §2.2 decay window's own 100Hz->10-20ms figure in order
        // of magnitude.
        assert_eq!(default_interpolation_delay_us(100), 15_000);
        // 10Hz -> 100ms period -> widens proportionally, same as decay.
        assert_eq!(default_interpolation_delay_us(10), 150_000);
        assert_eq!(default_interpolation_delay_us(0), 0);
    }
}
