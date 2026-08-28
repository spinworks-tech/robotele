//! Task-adaptive safety tier thresholds (REQUIREMENTS.md §5.2, SR-2/SR-3).
//!
//! 1:1 port of `simulator/roboprotocol_sim/protocol/safety_state_machine.py`'s
//! `TASK_CLASS_THRESHOLDS` / `WATCHDOG_BLACKOUT_MS` constants.

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum TaskClass {
    /// Fine Tool & Utensil Manipulation
    B,
    /// Coarse Object Free-Hand Pick & Place
    C,
    /// Indoor Mobile-Base Locomotion (renamed from "Indoor Humanoid
    /// Locomotion" -- FR-1.6, DESIGN.md §1.3.6). This is the XGO-Lite V2
    /// quadruped's Task Class, chosen for its indoor-tight-space latency
    /// profile; it is no longer a morphology mismatch now that balance
    /// applicability is gated by `profile::BaseType` instead of implied by
    /// the class name -- `QuadrupedLegs` correctly makes SR-1 a no-op here.
    D,
    /// Outdoor Gross Locomotion
    E,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TierThresholdsMs {
    pub tier0_max: f64, // Tier 0 NOMINAL upper bound (exclusive)
    pub tier1_max: f64, // Tier 1 DEGRADED upper bound
    pub tier2_max: f64, // Tier 2 CAUTIONARY upper bound
    pub tier3_max: f64, // Tier 3 CRITICAL upper bound; >= this is Tier 4 SUSPENDED
}

impl TaskClass {
    pub const fn thresholds(self) -> TierThresholdsMs {
        match self {
            TaskClass::B => TierThresholdsMs { tier0_max: 40.0, tier1_max: 80.0, tier2_max: 120.0, tier3_max: 150.0 },
            TaskClass::C => TierThresholdsMs { tier0_max: 80.0, tier1_max: 150.0, tier2_max: 220.0, tier3_max: 300.0 },
            TaskClass::D => TierThresholdsMs { tier0_max: 100.0, tier1_max: 250.0, tier2_max: 400.0, tier3_max: 500.0 },
            TaskClass::E => TierThresholdsMs { tier0_max: 200.0, tier1_max: 400.0, tier2_max: 700.0, tier3_max: 1000.0 },
        }
    }

    /// Tier 5 watchdog / E-Stop blackout threshold (SR-4.1, ms of heartbeat silence).
    pub const fn watchdog_blackout_ms(self) -> f64 {
        match self {
            TaskClass::B => 200.0,
            TaskClass::C => 300.0,
            TaskClass::D => 400.0,
            TaskClass::E => 500.0,
        }
    }
}

pub const RESUME_HYSTERESIS_MS: f64 = 20.0;
pub const RESUME_STABILITY_WINDOW_S: f64 = 2.0;

pub const TIER_NAMES: [&str; 5] = ["NOMINAL", "DEGRADED", "CAUTIONARY", "CRITICAL", "SUSPENDED"];

pub fn tier_for_latency(task_class: TaskClass, latency_ms: f64) -> u8 {
    let th = task_class.thresholds();
    if latency_ms < th.tier0_max {
        0
    } else if latency_ms < th.tier1_max {
        1
    } else if latency_ms < th.tier2_max {
        2
    } else if latency_ms < th.tier3_max {
        3
    } else {
        4
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn class_d_boundary_values_match_requirements_table() {
        // REQUIREMENTS.md Sec.5.2 Class D row: <100 / 100-250 / 250-400 / 400-500 / >=500
        assert_eq!(tier_for_latency(TaskClass::D, 0.0), 0);
        assert_eq!(tier_for_latency(TaskClass::D, 99.999), 0);
        assert_eq!(tier_for_latency(TaskClass::D, 100.0), 1);
        assert_eq!(tier_for_latency(TaskClass::D, 249.999), 1);
        assert_eq!(tier_for_latency(TaskClass::D, 250.0), 2);
        assert_eq!(tier_for_latency(TaskClass::D, 399.999), 2);
        assert_eq!(tier_for_latency(TaskClass::D, 400.0), 3);
        assert_eq!(tier_for_latency(TaskClass::D, 499.999), 3);
        assert_eq!(tier_for_latency(TaskClass::D, 500.0), 4);
        assert_eq!(tier_for_latency(TaskClass::D, 10_000.0), 4);
    }

    #[test]
    fn all_classes_watchdog_blackout_matches_requirements_table() {
        assert_eq!(TaskClass::B.watchdog_blackout_ms(), 200.0);
        assert_eq!(TaskClass::C.watchdog_blackout_ms(), 300.0);
        assert_eq!(TaskClass::D.watchdog_blackout_ms(), 400.0);
        assert_eq!(TaskClass::E.watchdog_blackout_ms(), 500.0);
    }

    #[test]
    fn tier_boundaries_are_monotonic_for_every_class() {
        for class in [TaskClass::B, TaskClass::C, TaskClass::D, TaskClass::E] {
            let th = class.thresholds();
            assert!(th.tier0_max < th.tier1_max);
            assert!(th.tier1_max < th.tier2_max);
            assert!(th.tier2_max < th.tier3_max);
        }
    }
}
