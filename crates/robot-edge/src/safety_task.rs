//! Glues `roboprotocol_core`'s safety state machine, watchdog, and
//! arbitration ladder into one per-tick decision for `robot-edge`.
//!
//! v0 simplification (documented, not silent): `safe_parking_requested`
//! and `resume_dwell_exceeded` are always `false` -- there's no operator
//! control wired up yet to request Emergency Safe Parking, and no
//! configurable dwell timer for auto-escalating out of Active Impedance
//! Hold. `autonomy_goal_asserted` is always `false` -- v0 has no
//! semi-autonomy source. The arbitration ladder itself (in
//! `roboprotocol-core`) is fully implemented; only these v0-specific
//! input sources are stubbed.

use std::time::Instant;

use roboprotocol_core::safety::{arbitrate, ArbitrationInputs, ControlSource, SafetyStateMachine, TaskClass, Watchdog};

pub struct SafetyTask {
    pub state_machine: SafetyStateMachine,
    pub watchdog: Watchdog,
    explicit_estop: bool,
    pub deadman_held: bool,
    pub command_fresh: bool,
}

impl SafetyTask {
    pub fn new(task_class: TaskClass, now: Instant) -> Self {
        Self {
            state_machine: SafetyStateMachine::new(task_class, now),
            watchdog: Watchdog::new(task_class, now),
            explicit_estop: false,
            deadman_held: false,
            command_fresh: false,
        }
    }

    /// Call on every received Channel B datagram (command from the
    /// operator, or any heartbeat) -- refreshes the watchdog, and if an
    /// RTT sample is available, feeds the Tier 0-4 state machine.
    pub fn on_channel_b_activity(&mut self, rtt_ms: Option<f64>, now: Instant) {
        self.watchdog.heartbeat(now);
        if let Some(rtt) = rtt_ms {
            self.state_machine.on_rtt_sample(rtt, now);
        }
    }

    /// Operator console's explicit E-Stop key/RPC, or the bridge
    /// supervisor giving up on `xgo_bridge.py` restarts.
    pub fn trigger_explicit_estop(&mut self) {
        self.explicit_estop = true;
    }

    /// SR-4: E-Stop is not auto-recoverable from network state alone --
    /// only an explicit out-of-band clear (operator RPC).
    pub fn clear_explicit_estop(&mut self, now: Instant) {
        self.explicit_estop = false;
        self.watchdog.clear(now);
    }

    pub fn is_estopped(&self) -> bool {
        self.explicit_estop || self.watchdog.is_triggered()
    }

    /// Poll periodically (the safety tick). Also polls the watchdog for a
    /// blackout trip. Returns the arbitrated control source for this tick.
    pub fn tick(&mut self, now: Instant) -> ControlSource {
        self.watchdog.check(now);

        let inputs = ArbitrationInputs {
            estop_latched: self.is_estopped(),
            safe_parking_requested: false,
            suspended: self.state_machine.is_suspended(),
            resume_dwell_exceeded: false,
            teleop_ready: self.deadman_held
                && self.command_fresh
                && !self.state_machine.is_suspended()
                && self.state_machine.tier() <= 3,
            autonomy_goal_asserted: false,
        };
        arbitrate(inputs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn watchdog_blackout_forces_estop_source_regardless_of_teleop_state() {
        let t0 = Instant::now();
        let mut task = SafetyTask::new(TaskClass::D, t0); // 400ms watchdog
        task.deadman_held = true;
        task.command_fresh = true;

        assert_eq!(task.tick(t0), ControlSource::FullTeleoperation);
        assert_eq!(task.tick(t0 + Duration::from_millis(401)), ControlSource::EStop);
    }

    #[test]
    fn explicit_estop_latches_until_cleared() {
        let t0 = Instant::now();
        let mut task = SafetyTask::new(TaskClass::D, t0);
        task.deadman_held = true;
        task.command_fresh = true;
        task.on_channel_b_activity(Some(10.0), t0);

        task.trigger_explicit_estop();
        assert_eq!(task.tick(t0 + Duration::from_millis(1)), ControlSource::EStop);
        task.on_channel_b_activity(Some(10.0), t0 + Duration::from_millis(2));
        assert_eq!(task.tick(t0 + Duration::from_millis(2)), ControlSource::EStop, "must not clear itself on fresh activity");

        task.clear_explicit_estop(t0 + Duration::from_millis(3));
        task.on_channel_b_activity(Some(10.0), t0 + Duration::from_millis(3));
        assert_eq!(task.tick(t0 + Duration::from_millis(3)), ControlSource::FullTeleoperation);
    }

    #[test]
    fn suspended_tier_holds_instead_of_teleop_even_with_deadman_and_fresh_command() {
        let t0 = Instant::now();
        let mut task = SafetyTask::new(TaskClass::D, t0);
        task.deadman_held = true;
        task.command_fresh = true;
        task.on_channel_b_activity(Some(600.0), t0); // Class D SUSPENDED at 500ms+
        assert_eq!(task.tick(t0), ControlSource::ActiveImpedanceHold);
    }
}
