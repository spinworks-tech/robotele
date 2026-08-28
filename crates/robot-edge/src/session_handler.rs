//! Encode `SessionDescribe` / decode `SessionAccept` FlatBuffers <->
//! `roboprotocol_core::profile` types. Robot-edge only ever encodes the
//! describe side and decodes the accept side (the operator console does
//! the reverse in its own `session_handler.rs`).

use flatbuffers::FlatBufferBuilder;
use roboprotocol_core::profile::{BaseType, CameraDescriptor, Codec, CommandShape, RobotProfile};
use roboprotocol_proto::{
    BaseType as FbBaseType, BodyRegionDescriptor as FbBodyRegion, BodyRegionDescriptorArgs,
    CameraDescriptor as FbCamera, CameraDescriptorArgs, Codec as FbCodec, CommandShape as FbCommandShape,
    JointDescriptor as FbJoint, JointDescriptorArgs, RobotProfile as FbRobotProfile, RobotProfileArgs,
    SessionAccept, SessionDescribe, SessionDescribeArgs,
};

fn to_fb_codec(codec: Codec) -> FbCodec {
    match codec {
        Codec::H265 => FbCodec::H265,
        Codec::Av1 => FbCodec::AV1,
        Codec::H264 => FbCodec::H264,
    }
}

fn to_fb_command_shape(shape: CommandShape) -> FbCommandShape {
    match shape {
        CommandShape::Kinematic => FbCommandShape::Kinematic,
        CommandShape::VelocityAttitude => FbCommandShape::VelocityAttitude,
        CommandShape::CartesianEndEffector => FbCommandShape::CartesianEndEffector,
    }
}

fn to_fb_base_type(base_type: BaseType) -> FbBaseType {
    match base_type {
        BaseType::Stationary => FbBaseType::Stationary,
        BaseType::WheeledStandard => FbBaseType::WheeledStandard,
        BaseType::WheeledHolonomic => FbBaseType::WheeledHolonomic,
        BaseType::BipedLegs => FbBaseType::BipedLegs,
        BaseType::QuadrupedLegs => FbBaseType::QuadrupedLegs,
        BaseType::Other => FbBaseType::Other,
    }
}

pub fn encode_session_describe(robot_id: &str, profile: &RobotProfile, cameras: &[CameraDescriptor]) -> Vec<u8> {
    let mut b = FlatBufferBuilder::new();

    let joint_offsets: Vec<_> = profile
        .joints
        .iter()
        .map(|j| {
            FbJoint::create(
                &mut b,
                &JointDescriptorArgs {
                    min_limit: j.min_limit,
                    max_limit: j.max_limit,
                    max_velocity: j.max_velocity,
                    has_torque_sensing: j.has_torque_sensing,
                    region_id: j.region_id,
                },
            )
        })
        .collect();
    let joints_vec = b.create_vector(&joint_offsets);

    let region_offsets: Vec<_> = profile
        .regions
        .iter()
        .map(|r| {
            let name = b.create_string(&r.name);
            FbBodyRegion::create(
                &mut b,
                &BodyRegionDescriptorArgs {
                    region_id: r.region_id,
                    name: Some(name),
                    joint_start: r.joint_start,
                    joint_count: r.joint_count,
                    has_force_torque_sensor: r.has_force_torque_sensor,
                    command_shape: to_fb_command_shape(r.command_shape),
                },
            )
        })
        .collect();
    let regions_vec = b.create_vector(&region_offsets);

    let robot_profile = FbRobotProfile::create(
        &mut b,
        &RobotProfileArgs {
            dof_count: profile.dof_count,
            joints: Some(joints_vec),
            regions: Some(regions_vec),
            base_type: to_fb_base_type(profile.base_type),
        },
    );

    let camera_offsets: Vec<_> = cameras
        .iter()
        .map(|c| {
            let label = b.create_string(&c.label);
            FbCamera::create(
                &mut b,
                &CameraDescriptorArgs {
                    camera_id: c.camera_id,
                    label: Some(label),
                    codec: to_fb_codec(c.codec),
                    resolution_w: c.resolution_w,
                    resolution_h: c.resolution_h,
                    max_fps: c.max_fps,
                    min_bitrate_kbps: c.min_bitrate_kbps,
                    max_bitrate_kbps: c.max_bitrate_kbps,
                },
            )
        })
        .collect();
    let cameras_vec = b.create_vector(&camera_offsets);

    let robot_id_str = b.create_string(robot_id);

    let describe = SessionDescribe::create(
        &mut b,
        &SessionDescribeArgs {
            robot_id: Some(robot_id_str),
            profile_hash: profile.profile_hash(),
            robot_profile: Some(robot_profile),
            cameras: Some(cameras_vec),
        },
    );
    b.finish(describe, None);
    b.finished_data().to_vec()
}

pub struct SessionAcceptInfo {
    pub profile_hash: u64,
    pub cached: bool,
    pub selected_regions: Vec<u8>,
    pub selected_cameras: Vec<u8>,
}

pub fn decode_session_accept(buf: &[u8]) -> anyhow::Result<SessionAcceptInfo> {
    let accept = flatbuffers::get_root::<SessionAccept>(buf);
    Ok(SessionAcceptInfo {
        profile_hash: accept.profile_hash(),
        cached: accept.cached(),
        selected_regions: accept.selected_regions().map(|v| v.to_vec()).unwrap_or_default(),
        selected_cameras: accept.selected_cameras().map(|v| v.to_vec()).unwrap_or_default(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xgo_profile::{xgo_lite_v2_camera, xgo_lite_v2_profile};

    #[test]
    fn session_describe_round_trips_profile_shape() {
        let profile = xgo_lite_v2_profile();
        let cameras = vec![xgo_lite_v2_camera()];
        let bytes = encode_session_describe("xgo_lite_v2", &profile, &cameras);

        let describe = flatbuffers::get_root::<roboprotocol_proto::SessionDescribe>(&bytes);
        assert_eq!(describe.robot_id(), Some("xgo_lite_v2"));
        assert_eq!(describe.profile_hash(), profile.profile_hash());
        let fb_profile = describe.robot_profile().unwrap();
        assert_eq!(fb_profile.dof_count(), 15);
        assert_eq!(fb_profile.joints().unwrap().len(), 15);
        assert_eq!(fb_profile.regions().unwrap().len(), 5);
        assert_eq!(describe.cameras().unwrap().len(), 1);

        // FR-1.5: the 4 leg regions cross the wire as VelocityAttitude,
        // the arm region as CartesianEndEffector -- not silently dropped
        // or defaulted to Kinematic.
        let regions = fb_profile.regions().unwrap();
        for region in regions.iter().take(4) {
            assert_eq!(region.command_shape(), roboprotocol_proto::CommandShape::VelocityAttitude);
        }
        assert_eq!(regions.get(4).command_shape(), roboprotocol_proto::CommandShape::CartesianEndEffector);

        // FR-1.6: base_type crosses the wire too, not silently dropped or
        // defaulted to Stationary -- this is what makes SR-1's no-op
        // applicability for this robot a wire fact, not an assumption.
        assert_eq!(fb_profile.base_type(), roboprotocol_proto::BaseType::QuadrupedLegs);
    }
}
