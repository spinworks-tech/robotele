//! Tier 5 hardware watchdog: E-Stop on heartbeat silence (SR-4.1/SR-4.2).
//!
//! Adapted from `simulator/roboprotocol_sim/protocol/safety_state_machine.py`'s
//! `Watchdog`. The simulator uses a virtual-time scheduler with a
//! self-arming timer callback; this crate has no async runtime, so the
//! caller (robot-edge's periodic safety tick) polls `check(now)` instead of
//! registering a callback. Same trigger semantics: blackout duration since
//! the last heartbeat, independent of and distinct from the RTT-driven
//! Tier 0-4 state machine.

use std::time::Instant;

use super::tiers::TaskClass;

#[derive(Debug, Clone, Copy)]
pub struct WatchdogTrigger {
    pub at: Instant,
    pub blackout_ms: f64,
}

pub struct Watchdog {
    threshold_ms: f64,
    last_heartbeat: Instant,
    triggered: bool,
    pub trigger_events: Vec<WatchdogTrigger>,
}

impl Watchdog {
    pub fn new(task_class: TaskClass, now: Instant) -> Self {
        Self {
            threshold_ms: task_class.watchdog_blackout_ms(),
            last_heartbeat: now,
            triggered: false,
            trigger_events: Vec::new(),
        }
    }

    pub fn threshold_ms(&self) -> f64 {
        self.threshold_ms
    }

    pub fn is_triggered(&self) -> bool {
        self.triggered
    }

    /// Call on every received Channel B heartbeat/command datagram.
    pub fn heartbeat(&mut self, now: Instant) {
        self.last_heartbeat = now;
    }

    /// Poll periodically (e.g. from the safety tick). Returns `true` the
    /// instant this call newly trips the watchdog (edge, not level) so the
    /// caller can react exactly once per blackout, e.g. to fire the E-Stop
    /// side effects (latch arbitration, notify the xgo_bridge, log).
    pub fn check(&mut self, now: Instant) -> bool {
        if self.triggered {
            return false;
        }
        let blackout_ms = now.duration_since(self.last_heartbeat).as_secs_f64() * 1000.0;
        if blackout_ms >= self.threshold_ms {
            self.triggered = true;
            self.trigger_events.push(WatchdogTrigger { at: now, blackout_ms });
            return true;
        }
        false
    }

    /// Explicit manual clear (out-of-band operational procedure -- SR-4
    /// E-Stop is not auto-recoverable from network state alone).
    pub fn clear(&mut self, now: Instant) {
        self.triggered = false;
        self.last_heartbeat = now;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn trips_exactly_at_class_blackout_threshold() {
        let t0 = Instant::now();
        let mut wd = Watchdog::new(TaskClass::D, t0); // 400ms threshold
        assert!(!wd.check(t0 + Duration::from_millis(399)));
        assert!(!wd.is_triggered());
        assert!(wd.check(t0 + Duration::from_millis(400)));
        assert!(wd.is_triggered());
    }

    #[test]
    fn heartbeat_resets_the_blackout_timer() {
        let t0 = Instant::now();
        let mut wd = Watchdog::new(TaskClass::B, t0); // 200ms threshold
        wd.heartbeat(t0 + Duration::from_millis(150));
        assert!(!wd.check(t0 + Duration::from_millis(340)), "only 190ms since last heartbeat");
        wd.heartbeat(t0 + Duration::from_millis(340));
        assert!(!wd.check(t0 + Duration::from_millis(530)));
        assert!(wd.check(t0 + Duration::from_millis(541)));
    }

    #[test]
    fn trigger_is_edge_not_level_and_clear_rearms() {
        let t0 = Instant::now();
        let mut wd = Watchdog::new(TaskClass::E, t0); // 500ms threshold
        assert!(wd.check(t0 + Duration::from_millis(600)));
        assert!(!wd.check(t0 + Duration::from_millis(700)), "already triggered -- no repeat edge");
        assert_eq!(wd.trigger_events.len(), 1);

        wd.clear(t0 + Duration::from_millis(700));
        assert!(!wd.is_triggered());
        assert!(!wd.check(t0 + Duration::from_millis(1000)));
        assert!(wd.check(t0 + Duration::from_millis(1201)));
        assert_eq!(wd.trigger_events.len(), 2);
    }
}
