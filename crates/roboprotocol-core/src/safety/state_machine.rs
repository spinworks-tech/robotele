//! Tier 0-4 RTT-driven safety state machine with SR-3.3 resume hysteresis.
//!
//! Port of `simulator/roboprotocol_sim/protocol/safety_state_machine.py`'s
//! `SafetyStateMachine`, adapted from the simulator's virtual-time
//! `Scheduler` to a real wall clock: callers pass `Instant::now()` in
//! explicitly rather than the state machine owning a clock, which keeps
//! this testable without sleeping.

use std::time::Instant;

use super::tiers::{tier_for_latency, TaskClass, RESUME_HYSTERESIS_MS, RESUME_STABILITY_WINDOW_S};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionTrigger {
    Init,
    Latency,
    Resume,
}

#[derive(Debug, Clone, Copy)]
pub struct TierTransition {
    pub at: Instant,
    pub old_tier: u8,
    pub new_tier: u8,
    pub trigger: TransitionTrigger,
}

pub struct SafetyStateMachine {
    task_class: TaskClass,
    tier: u8,
    suspended: bool,
    deadman_reset_requested: bool,
    stable_since: Option<Instant>,
    pub transitions: Vec<TierTransition>,
}

impl SafetyStateMachine {
    pub fn new(task_class: TaskClass, now: Instant) -> Self {
        Self {
            task_class,
            tier: 0,
            suspended: false,
            deadman_reset_requested: false,
            stable_since: None,
            transitions: vec![TierTransition {
                at: now,
                old_tier: 0,
                new_tier: 0,
                trigger: TransitionTrigger::Init,
            }],
        }
    }

    pub fn task_class(&self) -> TaskClass {
        self.task_class
    }

    pub fn tier(&self) -> u8 {
        self.tier
    }

    pub fn is_suspended(&self) -> bool {
        self.suspended
    }

    /// SR-3.3: operator dual-deadman-switch reset request. Only takes effect
    /// once the RTT stability window below is also satisfied.
    pub fn request_deadman_reset(&mut self) {
        self.deadman_reset_requested = true;
    }

    pub fn on_rtt_sample(&mut self, rtt_ms: f64, now: Instant) {
        if self.suspended {
            self.evaluate_resume(rtt_ms, now);
            return;
        }
        let new_tier = tier_for_latency(self.task_class, rtt_ms);
        if new_tier != self.tier {
            self.transition(new_tier, TransitionTrigger::Latency, now);
        }
        if new_tier == 4 {
            self.suspended = true;
            self.stable_since = None;
        }
    }

    fn evaluate_resume(&mut self, rtt_ms: f64, now: Instant) {
        let th = self.task_class.thresholds();
        let resume_ceiling = th.tier0_max - RESUME_HYSTERESIS_MS;
        if rtt_ms >= resume_ceiling {
            self.stable_since = None;
            return;
        }
        let stable_since = *self.stable_since.get_or_insert(now);
        let stable_for = now.duration_since(stable_since).as_secs_f64();
        if stable_for >= RESUME_STABILITY_WINDOW_S && self.deadman_reset_requested {
            self.suspended = false;
            self.deadman_reset_requested = false;
            self.stable_since = None;
            self.transition(0, TransitionTrigger::Resume, now);
        }
    }

    fn transition(&mut self, new_tier: u8, trigger: TransitionTrigger, now: Instant) {
        let old = self.tier;
        self.tier = new_tier;
        self.transitions.push(TierTransition { at: now, old_tier: old, new_tier, trigger });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn escalates_to_suspended_and_requires_full_resume_handshake() {
        let t0 = Instant::now();
        let mut sm = SafetyStateMachine::new(TaskClass::D, t0);
        assert_eq!(sm.tier(), 0);

        // Class D SUSPENDED starts at 500ms.
        sm.on_rtt_sample(600.0, t0);
        assert_eq!(sm.tier(), 4);
        assert!(sm.is_suspended());

        // Latency recovers (stability clock starts here) but no deadman
        // reset requested yet -- must not resume no matter how long it waits.
        sm.on_rtt_sample(10.0, t0 + Duration::from_millis(100));
        sm.on_rtt_sample(10.0, t0 + Duration::from_secs(5));
        assert!(sm.is_suspended(), "must not auto-resume without an explicit deadman reset");

        // Reset requested late (well past 2s of accrued stability) --
        // resume interlock is satisfied on the next stable sample.
        sm.request_deadman_reset();
        sm.on_rtt_sample(10.0, t0 + Duration::from_millis(5001));
        assert!(!sm.is_suspended(), "2s+ of already-accrued stability plus the reset request should resume");
        assert_eq!(sm.tier(), 0);
    }

    #[test]
    fn resume_stability_window_is_measured_from_when_latency_first_recovers() {
        let t0 = Instant::now();
        let mut sm = SafetyStateMachine::new(TaskClass::D, t0);
        sm.on_rtt_sample(600.0, t0); // suspend

        sm.request_deadman_reset();
        sm.on_rtt_sample(10.0, t0 + Duration::from_millis(100)); // stability clock starts
        assert!(sm.is_suspended());

        sm.on_rtt_sample(10.0, t0 + Duration::from_millis(1900)); // ~1.8s stable, not enough
        assert!(sm.is_suspended(), "1.8s of stability is not enough");

        sm.on_rtt_sample(10.0, t0 + Duration::from_millis(2200)); // ~2.1s stable
        assert!(!sm.is_suspended(), "2s+ of stability since recovery plus the reset request should resume");
        assert_eq!(sm.tier(), 0);
    }

    #[test]
    fn hysteresis_resets_stability_window_if_latency_spikes_again() {
        let t0 = Instant::now();
        let mut sm = SafetyStateMachine::new(TaskClass::D, t0);
        sm.on_rtt_sample(600.0, t0);
        sm.request_deadman_reset();

        sm.on_rtt_sample(10.0, t0 + Duration::from_millis(100));
        // Spike back above the resume ceiling (tier0_max - hysteresis = 80ms) restarts the clock.
        sm.on_rtt_sample(90.0, t0 + Duration::from_millis(1500));
        sm.on_rtt_sample(10.0, t0 + Duration::from_millis(3400)); // only ~1.9s stable since the spike
        assert!(sm.is_suspended());
    }
}
