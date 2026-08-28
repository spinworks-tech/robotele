//! Decode `SessionDescribe` / encode `SessionAccept` FlatBuffers <->
//! `roboprotocol_core::profile` types. Mirrors robot-edge's
//! `session_handler.rs`, reversed roles: the operator console decodes
//! the describe side and encodes the accept side.

use flatbuffers::FlatBufferBuilder;
use roboprotocol_core::profile::{
    BaseType, BodyRegionDescriptor, CameraDescriptor, Codec, CommandShape, JointDescriptor, RobotProfile,
};
use roboprotocol_proto::{
    BaseType as FbBaseType, Codec as FbCodec, CommandShape as FbCommandShape,
    FieldQuantization as FbFieldQuantization, FieldQuantizationArgs, SessionAccept, SessionAcceptArgs,
    SessionDescribe,
};

fn from_fb_codec(codec: FbCodec) -> Codec {
    match codec {
        FbCodec::AV1 => Codec::Av1,
        FbCodec::H264 => Codec::H264,
        _ => Codec::H265,
    }
}

fn from_fb_command_shape(shape: FbCommandShape) -> CommandShape {
    match shape {
        FbCommandShape::VelocityAttitude => CommandShape::VelocityAttitude,
        FbCommandShape::CartesianEndEffector => CommandShape::CartesianEndEffector,
        _ => CommandShape::Kinematic,
    }
}

fn from_fb_base_type(base_type: FbBaseType) -> BaseType {
    match base_type {
        FbBaseType::WheeledStandard => BaseType::WheeledStandard,
        FbBaseType::WheeledHolonomic => BaseType::WheeledHolonomic,
        FbBaseType::BipedLegs => BaseType::BipedLegs,
        FbBaseType::QuadrupedLegs => BaseType::QuadrupedLegs,
        FbBaseType::Other => BaseType::Other,
        _ => BaseType::Stationary,
    }
}

pub struct SessionDescribeInfo {
    pub robot_id: String,
    pub profile_hash: u64,
    pub profile: RobotProfile,
    pub cameras: Vec<CameraDescriptor>,
}

pub fn decode_session_describe(buf: &[u8]) -> anyhow::Result<SessionDescribeInfo> {
    let describe = flatbuffers::get_root::<SessionDescribe>(buf);

    let fb_profile = describe.robot_profile().ok_or_else(|| anyhow::anyhow!("SESSION_DESCRIBE missing robot_profile"))?;
    let joints = fb_profile
        .joints()
        .map(|v| {
            v.iter()
                .map(|j| JointDescriptor {
                    min_limit: j.min_limit(),
                    max_limit: j.max_limit(),
                    max_velocity: j.max_velocity(),
                    has_torque_sensing: j.has_torque_sensing(),
                    region_id: j.region_id(),
                })
                .collect()
        })
        .unwrap_or_default();
    let regions = fb_profile
        .regions()
        .map(|v| {
            v.iter()
                .map(|r| BodyRegionDescriptor {
                    region_id: r.region_id(),
                    name: r.name().unwrap_or_default().to_string(),
                    joint_start: r.joint_start(),
                    joint_count: r.joint_count(),
                    has_force_torque_sensor: r.has_force_torque_sensor(),
                    command_shape: from_fb_command_shape(r.command_shape()),
                })
                .collect()
        })
        .unwrap_or_default();

    let cameras = describe
        .cameras()
        .map(|v| {
            v.iter()
                .map(|c| CameraDescriptor {
                    camera_id: c.camera_id(),
                    label: c.label().unwrap_or_default().to_string(),
                    codec: from_fb_codec(c.codec()),
                    resolution_w: c.resolution_w(),
                    resolution_h: c.resolution_h(),
                    max_fps: c.max_fps(),
                    min_bitrate_kbps: c.min_bitrate_kbps(),
                    max_bitrate_kbps: c.max_bitrate_kbps(),
                })
                .collect()
        })
        .unwrap_or_default();

    Ok(SessionDescribeInfo {
        robot_id: describe.robot_id().unwrap_or_default().to_string(),
        profile_hash: describe.profile_hash(),
        profile: RobotProfile {
            dof_count: fb_profile.dof_count(),
            joints,
            regions,
            base_type: from_fb_base_type(fb_profile.base_type()),
        },
        cameras,
    })
}

/// v0: auto-accepts the full advertised profile (all regions, all
/// cameras, Standard quantization tier) -- no operator UI for narrowing
/// the selection yet (that's NFR-2.3's fuller console, a later phase).
pub fn encode_session_accept_full(info: &SessionDescribeInfo, cached: bool) -> Vec<u8> {
    let mut b = FlatBufferBuilder::new();

    let selected_regions: Vec<u8> = info.profile.regions.iter().map(|r| r.region_id).collect();
    let regions_vec = b.create_vector(&selected_regions);

    let quant_offsets: Vec<_> = [0u8, 1, 2] // command, telemetry, haptic
        .iter()
        .map(|&category| {
            FbFieldQuantization::create(
                &mut b,
                &FieldQuantizationArgs { category, tier: roboprotocol_core::sizing::QuantizationTier::Standard.bytes_per_field() as u8 },
            )
        })
        .collect();
    let quant_vec = b.create_vector(&quant_offsets);

    let selected_cameras: Vec<u8> = info.cameras.iter().map(|c| c.camera_id).collect();
    let cameras_vec = b.create_vector(&selected_cameras);

    let accept = SessionAccept::create(
        &mut b,
        &SessionAcceptArgs {
            profile_hash: info.profile_hash,
            cached,
            selected_regions: Some(regions_vec),
            quantization: Some(quant_vec),
            selected_cameras: Some(cameras_vec),
        },
    );
    b.finish(accept, None);
    b.finished_data().to_vec()
}
