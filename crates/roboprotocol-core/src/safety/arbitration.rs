//! Control Source Arbitration Ladder (DESIGN.md §3.0, REQUIREMENTS.md FR-4.2).
//!
//! Resolves, once per control tick, exactly one control source authorized
//! to command the actuators -- a single fixed-priority decision function
//! evaluated top-down. Never blended, never fall-through.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlSource {
    /// Priority 1 -- SR-4. Watchdog blackout or explicit E-Stop signal.
    /// Latched, level-triggered; cleared only by explicit human act.
    EStop,
    /// Priority 2 -- FR-4.1.4. Explicit request, or the SR-3.3 resume
    /// interlock hasn't completed within a configurable dwell time after
    /// entering SUSPENDED.
    EmergencySafeParking,
    /// Priority 3 -- FR-4.1.3. Tier 4 SUSPENDED, until the SR-3.3 operator
    /// interlock + stability window completes.
    ActiveImpedanceHold,
    /// Priority 4 -- FR-4.1.1. Deadman held, latest command fresh, latency
    /// within Tier 0-3 for the active Task Class.
    FullTeleoperation,
    /// Priority 5 -- FR-4.1.2. Full Teleoperation not engaged, an autonomy
    /// goal asserted via Channel C RPC.
    SemiAutonomous,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct ArbitrationInputs {
    pub estop_latched: bool,
    pub safe_parking_requested: bool,
    /// Tier 4 SUSPENDED, i.e. `SafetyStateMachine::is_suspended()`.
    pub suspended: bool,
    /// SR-3.3 resume interlock has not completed within the configurable
    /// dwell time since entering SUSPENDED. Only meaningful when `suspended`.
    pub resume_dwell_exceeded: bool,
    /// Operator deadman held, latest command fresh, latency within Tier 0-3.
    pub teleop_ready: bool,
    pub autonomy_goal_asserted: bool,
}

/// `DESIGN.md` §3.0's `arbitrate(...)`, evaluated top-down. Exactly one
/// source is returned per call; no source is ever a competing rung the WBC
/// override could outrank -- that's a separate, cross-cutting layer (§3.4)
/// applied to whichever source wins here.
pub fn arbitrate(inputs: ArbitrationInputs) -> ControlSource {
    if inputs.estop_latched {
        return ControlSource::EStop;
    }
    if inputs.safe_parking_requested || (inputs.suspended && inputs.resume_dwell_exceeded) {
        return ControlSource::EmergencySafeParking;
    }
    if inputs.suspended {
        return ControlSource::ActiveImpedanceHold;
    }
    if inputs.teleop_ready {
        return ControlSource::FullTeleoperation;
    }
    if inputs.autonomy_goal_asserted {
        return ControlSource::SemiAutonomous;
    }
    // Fail-safe default: nothing else admitted this tick -- hold, don't drift.
    ControlSource::ActiveImpedanceHold
}

#[cfg(test)]
mod tests {
    use super::*;

    /// FR-4.2's own verification method: an exhaustive truth table over the
    /// arbitration inputs, asserting the ladder's priority ordering holds
    /// for every combination rather than a handful of hand-picked cases.
    #[test]
    fn ladder_priority_holds_over_every_input_combination() {
        for bits in 0u8..64 {
            let inputs = ArbitrationInputs {
                estop_latched: bits & 0b00_0001 != 0,
                safe_parking_requested: bits & 0b00_0010 != 0,
                suspended: bits & 0b00_0100 != 0,
                resume_dwell_exceeded: bits & 0b00_1000 != 0,
                teleop_ready: bits & 0b01_0000 != 0,
                autonomy_goal_asserted: bits & 0b10_0000 != 0,
            };
            let got = arbitrate(inputs);

            if inputs.estop_latched {
                assert_eq!(got, ControlSource::EStop, "{inputs:?}");
                continue;
            }
            if inputs.safe_parking_requested || (inputs.suspended && inputs.resume_dwell_exceeded) {
                assert_eq!(got, ControlSource::EmergencySafeParking, "{inputs:?}");
                continue;
            }
            if inputs.suspended {
                assert_eq!(got, ControlSource::ActiveImpedanceHold, "{inputs:?}");
                continue;
            }
            if inputs.teleop_ready {
                assert_eq!(got, ControlSource::FullTeleoperation, "{inputs:?}");
                continue;
            }
            if inputs.autonomy_goal_asserted {
                assert_eq!(got, ControlSource::SemiAutonomous, "{inputs:?}");
                continue;
            }
            assert_eq!(got, ControlSource::ActiveImpedanceHold, "{inputs:?}");
        }
    }

    #[test]
    fn estop_always_wins_regardless_of_everything_else() {
        let inputs = ArbitrationInputs {
            estop_latched: true,
            safe_parking_requested: true,
            suspended: true,
            resume_dwell_exceeded: true,
            teleop_ready: true,
            autonomy_goal_asserted: true,
        };
        assert_eq!(arbitrate(inputs), ControlSource::EStop);
    }

    #[test]
    fn teleop_ready_outranks_autonomy_goal() {
        let inputs = ArbitrationInputs {
            teleop_ready: true,
            autonomy_goal_asserted: true,
            ..Default::default()
        };
        assert_eq!(arbitrate(inputs), ControlSource::FullTeleoperation);
    }
}
