//! Decode `SessionDescribe` / encode `SessionAccept` FlatBuffers <->
//! `roboprotocol_core::profile` types. Mirrors robot-edge's
//! `session_handler.rs`, reversed roles: the operator console decodes
//! the describe side and encodes the accept side.

use flatbuffers::FlatBufferBuilder;
use roboprotocol_core::profile::{
    BaseType, BodyRegionDescriptor, CameraDescriptor, Codec, CommandShape, JointDescriptor, RobotProfile,
};
use roboprotocol_core::sensor::{DepthFormat, DepthIntrinsics, PointFormat, RangeImageFormat, SensorDescriptor, SensorEncoding};
use roboprotocol_proto::{
    BaseType as FbBaseType, Codec as FbCodec, CommandShape as FbCommandShape,
    FieldQuantization as FbFieldQuantization, FieldQuantizationArgs, SensorDescriptor as FbSensor, SensorKind,
    SessionAccept, SessionAcceptArgs, SessionDescribe,
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
    /// Only descriptors that passed `SensorDescriptor::validate`; invalid
    /// ones are logged and left out, so they're never selected either.
    pub sensors: Vec<SensorDescriptor>,
}

fn f32_vec(v: Option<flatbuffers::Vector<'_, f32>>) -> Vec<f32> {
    v.map(|v| (0..v.len()).map(|i| v.get(i)).collect()).unwrap_or_default()
}

/// `None` when the kind's encoding table is missing or the descriptor
/// fails validation.
fn decode_sensor(s: FbSensor<'_>) -> Option<SensorDescriptor> {
    let encoding = match s.kind() {
        SensorKind::PointCloud => {
            let p = s.points()?;
            SensorEncoding::Points(PointFormat {
                scale_m: p.scale_m(),
                intensity: p.intensity(),
                doppler_scale_mps: (p.doppler_scale_mps() != 0.0).then_some(p.doppler_scale_mps()),
                snr: p.snr(),
            })
        }
        SensorKind::RangeImage => {
            let r = s.range_image()?;
            SensorEncoding::RangeImage {
                format: RangeImageFormat { beams: r.beams(), columns: r.columns(), range_scale_m: r.range_scale_m(), intensity: r.intensity() },
                beam_elevations_rad: f32_vec(r.beam_elevations_rad()),
            }
        }
        SensorKind::Depth => {
            let d = s.depth()?;
            SensorEncoding::Depth {
                format: DepthFormat { width: d.width(), height: d.height(), segment_width: d.segment_width() },
                intrinsics: DepthIntrinsics { fx: d.fx(), fy: d.fy(), cx: d.cx(), cy: d.cy() },
            }
        }
    };
    let position = f32_vec(s.mount_position_m());
    let orientation = f32_vec(s.mount_orientation_xyzw());
    let descriptor = SensorDescriptor {
        sensor_id: s.sensor_id(),
        label: s.label().unwrap_or_default().to_string(),
        encoding,
        max_hz: s.max_hz(),
        max_bitrate_kbps: s.max_bitrate_kbps(),
        mount_position_m: position.try_into().unwrap_or([0.0; 3]),
        mount_orientation_xyzw: orientation.try_into().unwrap_or([0.0, 0.0, 0.0, 1.0]),
    };
    match descriptor.validate() {
        Ok(()) => Some(descriptor),
        Err(e) => {
            tracing::warn!(sensor_id = descriptor.sensor_id, label = %descriptor.label, error = %e, "ignoring invalid sensor descriptor");
            None
        }
    }
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

    let sensors = describe.sensors().map(|v| v.iter().filter_map(decode_sensor).collect()).unwrap_or_default();

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
        sensors,
    })
}

/// v0: auto-accepts the full advertised profile (all regions, all
/// cameras, all valid sensors, Standard quantization tier) -- no operator UI for narrowing
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

    let selected_sensors: Vec<u8> = info.sensors.iter().map(|s| s.sensor_id).collect();
    let sensors_vec = b.create_vector(&selected_sensors);

    let accept = SessionAccept::create(
        &mut b,
        &SessionAcceptArgs {
            profile_hash: info.profile_hash,
            cached,
            selected_regions: Some(regions_vec),
            quantization: Some(quant_vec),
            selected_cameras: Some(cameras_vec),
            selected_sensors: Some(sensors_vec),
        },
    );
    b.finish(accept, None);
    b.finished_data().to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;
    use roboprotocol_proto::{
        RangeImageEncoding, RangeImageEncodingArgs, RobotProfile as FbRobotProfile, RobotProfileArgs, SensorDescriptorArgs,
        SessionDescribeArgs,
    };

    /// A SESSION_DESCRIBE with one range-image sensor; `elevations` lets a
    /// test make it invalid (the count must match `beams`).
    fn describe_with_lidar(elevations: &[f32]) -> Vec<u8> {
        let mut b = FlatBufferBuilder::new();
        let profile = FbRobotProfile::create(&mut b, &RobotProfileArgs { dof_count: 0, ..Default::default() });
        let elevations = b.create_vector(elevations);
        let range_image = RangeImageEncoding::create(
            &mut b,
            &RangeImageEncodingArgs { beams: 2, columns: 8, range_scale_m: 0.002, intensity: true, beam_elevations_rad: Some(elevations) },
        );
        let label = b.create_string("lidar");
        let position = b.create_vector(&[0.1f32, 0.0, 0.5]);
        let sensor = FbSensor::create(
            &mut b,
            &SensorDescriptorArgs {
                sensor_id: 4,
                label: Some(label),
                kind: SensorKind::RangeImage,
                max_hz: 10,
                max_bitrate_kbps: 2_000,
                mount_position_m: Some(position),
                range_image: Some(range_image),
                ..Default::default()
            },
        );
        let sensors = b.create_vector(&[sensor]);
        let describe = SessionDescribe::create(&mut b, &SessionDescribeArgs { robot_profile: Some(profile), sensors: Some(sensors), ..Default::default() });
        b.finish(describe, None);
        b.finished_data().to_vec()
    }

    #[test]
    fn decodes_a_sensor_descriptor_and_selects_it() {
        let info = decode_session_describe(&describe_with_lidar(&[-0.1, 0.1])).unwrap();
        assert_eq!(info.sensors.len(), 1);
        let s = &info.sensors[0];
        assert_eq!((s.sensor_id, s.label.as_str(), s.max_hz, s.max_bitrate_kbps), (4, "lidar", 10, 2_000));
        assert_eq!(s.mount_position_m, [0.1, 0.0, 0.5]);
        assert_eq!(s.mount_orientation_xyzw, [0.0, 0.0, 0.0, 1.0], "a missing orientation defaults to identity");
        assert_eq!(
            s.encoding,
            SensorEncoding::RangeImage {
                format: RangeImageFormat { beams: 2, columns: 8, range_scale_m: 0.002, intensity: true },
                beam_elevations_rad: vec![-0.1, 0.1],
            }
        );

        let accept = encode_session_accept_full(&info, false);
        let accept = flatbuffers::get_root::<SessionAccept>(&accept);
        assert_eq!(accept.selected_sensors(), Some(&[4u8][..]));
    }

    #[test]
    fn an_invalid_sensor_descriptor_is_dropped_not_selected() {
        let info = decode_session_describe(&describe_with_lidar(&[0.0])).unwrap();
        assert!(info.sensors.is_empty());
    }
}
