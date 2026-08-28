//! Hardcoded `RobotProfile`/`CameraDescriptor` for the XGO-Lite V2.
//!
//! v0 has exactly one supported robot, so this is a constant rather than
//! something loaded from a URDF/config file (FR-7.2's build-time-authoring
//! convenience is deferred; a later phase can add real URDF ingestion).
//!
//! Joint numbering follows the vendor `xgolib` `motor_id` convention:
//! first digit = leg (1-4 = left-front, right-front, right-rear,
//! left-rear per the vendor SDK docs), second digit = position
//! bottom-to-top (1=lower, 2=middle, 3=upper). Wire order here is
//! leg-major/position-minor, i.e. index 0..2 = leg 1's lower/middle/upper,
//! matching `motor_id` order -- both endpoints must agree on this order
//! since it's what `roboprotocol_core::profile::derive_layout` walks.
//!
//! **This unit has the optional arm/gripper accessory attached**,
//! confirmed by reading real hardware over SSH: `read_motor()` on the
//! physical robot returns 15 values, not the base kit's 12 -- 12 leg
//! angles that jitter with live sensor noise, plus 3 stable arm-joint
//! values sitting in an idle Cartesian-commanded pose. The arm is
//! appended as joint indices 12-14 / region 4.
//!
//! **Command shape (FR-1.5, DESIGN.md §1.3.2):** this robot is a hybrid --
//! the 4 leg regions are `VelocityAttitude` (xgolib's `move_x/y`, `turn`,
//! `attitude`; body velocity isn't a per-leg quantity, so all 4 collapse
//! into one shared command block per `derive_command_layout`) and the arm
//! region is `CartesianEndEffector` (xgolib's `arm(x, z)` + `claw(pos)`,
//! independently targetable). This used to be an undeclared convention the
//! operator console had to know out-of-band; it is now a wire fact in
//! `RobotProfile`, the same as everything else in this file.

use roboprotocol_core::profile::{
    BaseType, BodyRegionDescriptor, CameraDescriptor, Codec, CommandShape, JointDescriptor, RobotProfile,
};

/// Servo spec (vendor product-parameters page): 0-300 deg mechanical range,
/// bus servo, "0.1s per 60 deg" => ~600 deg/s max slew.
const MAX_VELOCITY_DEG_S: f32 = 600.0;

// Per-position angle ranges from the vendor xgolib docs (`motor()`):
// lower [-65,73], middle [-66,93], upper [-31,31] degrees.
const LOWER_RANGE: (f32, f32) = (-65.0, 73.0);
const MIDDLE_RANGE: (f32, f32) = (-66.0, 93.0);
const UPPER_RANGE: (f32, f32) = (-31.0, 31.0);

const LEG_NAMES: [&str; 4] = ["leg_front_left", "leg_front_right", "leg_rear_right", "leg_rear_left"];
const ARM_REGION_ID: u8 = 4;

pub fn xgo_lite_v2_profile() -> RobotProfile {
    let mut joints = Vec::with_capacity(15);
    for region_id in 0u8..4 {
        for (min_limit, max_limit) in [LOWER_RANGE, MIDDLE_RANGE, UPPER_RANGE] {
            joints.push(JointDescriptor {
                min_limit,
                max_limit,
                max_velocity: MAX_VELOCITY_DEG_S,
                // Bus servos report angle only via `read_motor()`; no
                // per-joint torque/current sensing is exposed by the SDK.
                has_torque_sensing: false,
                region_id,
            });
        }
    }

    // Arm joints (indices 12-14): the vendor SDK does not expose direct
    // per-joint angle control or documented angle limits for the arm --
    // it's commanded in Cartesian space via `arm(x, z)` (x:[-80,155],
    // z:[-95,155] mm) plus `claw(pos)` for the gripper, not `motor()`.
    // `min_limit`/`max_limit` here are therefore the observed idle-pose
    // readback range, not a servo angle spec; real angle limits aren't
    // documented and aren't needed since nothing commands these joints
    // by angle. See `bridge::protocol::BridgeCommand::Arm`/`Claw`.
    for _ in 0..3 {
        joints.push(JointDescriptor {
            min_limit: -90.0,
            max_limit: 90.0,
            max_velocity: MAX_VELOCITY_DEG_S,
            has_torque_sensing: false,
            region_id: ARM_REGION_ID,
        });
    }

    let mut regions: Vec<BodyRegionDescriptor> = (0u8..4)
        .map(|region_id| BodyRegionDescriptor {
            region_id,
            name: LEG_NAMES[region_id as usize].to_string(),
            joint_start: region_id as u16 * 3,
            joint_count: 3,
            // No foot force-torque sensor on this kit.
            has_force_torque_sensor: false,
            command_shape: CommandShape::VelocityAttitude,
        })
        .collect();
    regions.push(BodyRegionDescriptor {
        region_id: ARM_REGION_ID,
        name: "arm".to_string(),
        joint_start: 12,
        joint_count: 3,
        has_force_torque_sensor: false,
        command_shape: CommandShape::CartesianEndEffector,
    });

    // FR-1.6: statically stable on 4 legs, so SR-1's WBC/ZMP override is a
    // documented no-op for this robot (roboprotocol_core::profile::
    // BaseType::requires_dynamic_balance_override).
    RobotProfile { dof_count: 15, joints, regions, base_type: BaseType::QuadrupedLegs }
}

/// OV5647 5MP camera on the CM4's CSI port (vendor product-parameters
/// page), streamed as v0's basic H.264 Channel A path (see DESIGN.md's
/// documented v0 codec deviation from H.265/AV1 -- roboprotocol_core::
/// profile::Codec::H264).
pub fn xgo_lite_v2_camera() -> CameraDescriptor {
    CameraDescriptor {
        camera_id: 0,
        label: "front".to_string(),
        codec: Codec::H264,
        resolution_w: 640,
        resolution_h: 480,
        max_fps: 30,
        // Conservative bounds for 640x480@30 H.264 on a software/HW-M2M
        // encode path; not measured yet -- see the plan's verification
        // checklist item to measure real achievable rates on hardware.
        min_bitrate_kbps: 500,
        max_bitrate_kbps: 4000,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn profile_has_fifteen_joints_across_four_leg_regions_plus_arm() {
        let profile = xgo_lite_v2_profile();
        assert_eq!(profile.dof_count, 15);
        assert_eq!(profile.joints.len(), 15);
        assert_eq!(profile.regions.len(), 5);
        for region in 0..4u8 {
            let count = profile.joints.iter().filter(|j| j.region_id == region).count();
            assert_eq!(count, 3, "leg region {region} should have exactly 3 joints (lower/middle/upper)");
        }
        let arm_count = profile.joints.iter().filter(|j| j.region_id == ARM_REGION_ID).count();
        assert_eq!(arm_count, 3, "arm region should have exactly 3 joints");
    }

    #[test]
    fn no_haptic_or_torque_sensing_on_this_hardware() {
        let profile = xgo_lite_v2_profile();
        assert!(profile.joints.iter().all(|j| !j.has_torque_sensing));
        assert!(profile.regions.iter().all(|r| !r.has_force_torque_sensor));
    }

    #[test]
    fn legs_are_velocity_attitude_and_arm_is_cartesian() {
        let profile = xgo_lite_v2_profile();
        for region in 0..4u8 {
            let r = profile.regions.iter().find(|r| r.region_id == region).unwrap();
            assert_eq!(r.command_shape, roboprotocol_core::profile::CommandShape::VelocityAttitude, "leg region {region}");
        }
        let arm = profile.regions.iter().find(|r| r.region_id == ARM_REGION_ID).unwrap();
        assert_eq!(arm.command_shape, roboprotocol_core::profile::CommandShape::CartesianEndEffector);
    }

    #[test]
    fn base_type_is_quadruped_so_sr1_is_a_documented_no_op() {
        let profile = xgo_lite_v2_profile();
        assert_eq!(profile.base_type, BaseType::QuadrupedLegs);
        assert!(!profile.base_type.requires_dynamic_balance_override());
        assert!(profile.base_type.lateral_velocity_meaningful(), "a quadruped can strafe, unlike WheeledStandard");
    }
}
