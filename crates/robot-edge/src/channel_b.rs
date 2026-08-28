//! Encode/decode `ChannelBFrame` FlatBuffers (DESIGN.md §2.2, §1.3.3).
//!
//! This robot is FR-1.5's hybrid case: `xgo_profile.rs` declares the 4 leg
//! regions `CommandShape::VelocityAttitude` (one shared block for the
//! robot, not per-leg -- body velocity isn't a per-leg quantity) and the
//! arm region `CommandShape::CartesianEndEffector` (its own independent
//! block). `TeleopCommand` below is exactly those two blocks concatenated
//! in that order -- the same order `roboprotocol_core::profile::
//! derive_command_layout` derives for this robot's profile. The actual
//! byte packing for each shape lives in `roboprotocol_core::profile`
//! (`VelocityAttitudeCommand`, `CartesianCommand`), not duplicated here by
//! hand -- this file used to hand-roll its own i16/u8-mixed layout
//! independently of `RobotProfile`, which is exactly the gap FR-1.5 closes:
//! any other integrator using these same two command shapes reuses that
//! packing rather than re-deriving it. Telemetry packs the 12 joint angles
//! (from `derive_layout`) plus roll/pitch/yaw and battery, unaffected by
//! command shape, per FR-1.5's telemetry-is-always-per-joint rule.

use flatbuffers::FlatBufferBuilder;
use roboprotocol_core::profile::{CartesianCommand, VelocityAttitudeCommand};
use roboprotocol_proto::{ChannelBCategory as FbCategory, ChannelBFrame, ChannelBFrameArgs};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelBCategory {
    Command,
    Telemetry,
    Haptic,
}

fn to_fb_category(c: ChannelBCategory) -> FbCategory {
    match c {
        ChannelBCategory::Command => FbCategory::Command,
        ChannelBCategory::Telemetry => FbCategory::Telemetry,
        ChannelBCategory::Haptic => FbCategory::Haptic,
    }
}

fn from_fb_category(c: FbCategory) -> ChannelBCategory {
    match c {
        FbCategory::Telemetry => ChannelBCategory::Telemetry,
        FbCategory::Haptic => ChannelBCategory::Haptic,
        _ => ChannelBCategory::Command,
    }
}

pub struct ChannelBFrameData {
    pub timestamp: u64,
    pub seq: u64,
    pub tick_id: u32,
    pub category: ChannelBCategory,
    /// 0xFF = all regions (this datagram is not a body-region split).
    pub region_id: u8,
    pub fields: Vec<u8>,
}

pub const ALL_REGIONS: u8 = 0xFF;

pub fn encode_channel_b_frame(frame: &ChannelBFrameData) -> Vec<u8> {
    let mut b = FlatBufferBuilder::new();
    let fields_vec = b.create_vector(&frame.fields);
    let offset = ChannelBFrame::create(
        &mut b,
        &ChannelBFrameArgs {
            timestamp: frame.timestamp,
            seq: frame.seq,
            tick_id: frame.tick_id,
            category: to_fb_category(frame.category),
            region_id: frame.region_id,
            fields: Some(fields_vec),
        },
    );
    b.finish(offset, None);
    b.finished_data().to_vec()
}

pub fn decode_channel_b_frame(buf: &[u8]) -> anyhow::Result<ChannelBFrameData> {
    let f = flatbuffers::get_root::<ChannelBFrame>(buf);
    Ok(ChannelBFrameData {
        timestamp: f.timestamp(),
        seq: f.seq(),
        tick_id: f.tick_id(),
        category: from_fb_category(f.category()),
        region_id: f.region_id(),
        fields: f.fields().map(|v| v.to_vec()).unwrap_or_default(),
    })
}

/// Command payload = `VelocityAttitudeCommand::pack_standard()` (12B) ++
/// `CartesianCommand::pack_standard()` (8B) = 20B total, per this robot's
/// declared region shapes (module docs). `arm_x`/`arm_z`/`claw` are this
/// file's names for the arm's `CartesianCommand.x`/`.z`/`.gripper` --
/// `y` is unused (always 0), xgolib's `arm()` only has 2 controllable axes
/// (`DESIGN.md` §1.3.2). Flat field names kept here (rather than nested
/// `cmd.velocity_attitude.vx` at every call site) purely for call-site
/// ergonomics in `quic_server.rs`; the wire encoding is 100% the two
/// standardized shapes, not a bespoke layout.
///
/// **Does not carry `action_id` any more** (FR-1.8, `REQUIREMENTS.md`):
/// discrete one-shot triggers moved to a dedicated Channel C RPC
/// (`roboprotocol_core::action_trigger`, this crate's `action_trigger.rs`)
/// so a genuine new trigger can never be silently dropped along with a
/// stale continuous update it happened to be batched with.
pub struct TeleopCommand {
    pub vx: f32,
    pub vy: f32,
    pub turn: f32,
    pub attitude_r: f32,
    pub attitude_p: f32,
    pub attitude_y: f32,
    pub arm_x: i16,
    pub arm_z: i16,
    pub claw: u8,
}

const VELOCITY_ATTITUDE_STANDARD_LEN: usize = 12;
const CARTESIAN_STANDARD_LEN: usize = 8;

impl TeleopCommand {
    fn velocity_attitude(&self) -> VelocityAttitudeCommand {
        VelocityAttitudeCommand {
            vx: self.vx,
            vy: self.vy,
            turn: self.turn,
            roll: self.attitude_r,
            pitch: self.attitude_p,
            yaw: self.attitude_y,
        }
    }

    fn arm(&self) -> CartesianCommand {
        CartesianCommand { x: self.arm_x as f32, y: 0.0, z: self.arm_z as f32, gripper: self.claw }
    }

    pub fn pack(&self) -> Vec<u8> {
        let mut out = self.velocity_attitude().pack_standard();
        out.extend(self.arm().pack_standard());
        out
    }

    pub fn unpack(bytes: &[u8]) -> Option<Self> {
        let va = VelocityAttitudeCommand::unpack_standard(bytes.get(0..VELOCITY_ATTITUDE_STANDARD_LEN)?)?;
        let arm = CartesianCommand::unpack_standard(bytes.get(VELOCITY_ATTITUDE_STANDARD_LEN..VELOCITY_ATTITUDE_STANDARD_LEN + CARTESIAN_STANDARD_LEN)?)?;
        Some(Self {
            vx: va.vx,
            vy: va.vy,
            turn: va.turn,
            attitude_r: va.roll,
            attitude_p: va.pitch,
            attitude_y: va.yaw,
            arm_x: arm.x as i16,
            arm_z: arm.z as i16,
            claw: arm.gripper,
        })
    }
}

/// Telemetry field layout, Standard (int16) tier: battery (u8, %) + roll/
/// pitch/yaw (i16, 0.01 deg/LSB) + a trailing variable-length run of
/// per-joint angles (i16, 0.01 deg/LSB) -- length isn't fixed at 12 or 15
/// so this works for either the base kit or one with the arm attached
/// (see xgo_profile.rs's dof_count), derived from the remaining byte
/// count rather than hardcoded.
pub struct TelemetryData {
    pub battery: u8,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub motors: Vec<f32>,
}

const TELEMETRY_HEADER_LEN: usize = 7; // battery(1) + roll/pitch/yaw(2 each)

impl TelemetryData {
    pub fn pack(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(TELEMETRY_HEADER_LEN + self.motors.len() * 2);
        out.push(self.battery);
        out.extend_from_slice(&((self.roll * 100.0).round() as i16).to_be_bytes());
        out.extend_from_slice(&((self.pitch * 100.0).round() as i16).to_be_bytes());
        out.extend_from_slice(&((self.yaw * 100.0).round() as i16).to_be_bytes());
        for m in &self.motors {
            out.extend_from_slice(&((m * 100.0).round() as i16).to_be_bytes());
        }
        out
    }

    pub fn unpack(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < TELEMETRY_HEADER_LEN || (bytes.len() - TELEMETRY_HEADER_LEN) % 2 != 0 {
            return None;
        }
        let i16_at = |i: usize| i16::from_be_bytes([bytes[i], bytes[i + 1]]);
        let motors = bytes[TELEMETRY_HEADER_LEN..].chunks_exact(2).map(|c| i16::from_be_bytes([c[0], c[1]]) as f32 / 100.0).collect();
        Some(Self {
            battery: bytes[0],
            roll: i16_at(1) as f32 / 100.0,
            pitch: i16_at(3) as f32 / 100.0,
            yaw: i16_at(5) as f32 / 100.0,
            motors,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn channel_b_frame_round_trips_through_flatbuffers() {
        let cmd = TeleopCommand {
            vx: 12.0,
            vy: -3.0,
            turn: 20.0,
            attitude_r: 1.5,
            attitude_p: -2.25,
            attitude_y: 0.0,
            arm_x: 40,
            arm_z: -10,
            claw: 200,
        };
        let frame = ChannelBFrameData {
            timestamp: 123456789,
            seq: 42,
            tick_id: 7,
            category: ChannelBCategory::Command,
            region_id: ALL_REGIONS,
            fields: cmd.pack(),
        };
        let bytes = encode_channel_b_frame(&frame);
        let decoded = decode_channel_b_frame(&bytes).unwrap();
        assert_eq!(decoded.timestamp, 123456789);
        assert_eq!(decoded.seq, 42);
        assert_eq!(decoded.tick_id, 7);
        assert_eq!(decoded.category, ChannelBCategory::Command);
        assert_eq!(decoded.region_id, ALL_REGIONS);

        let round_tripped = TeleopCommand::unpack(&decoded.fields).unwrap();
        assert_eq!(round_tripped.vx, 12.0);
        assert_eq!(round_tripped.vy, -3.0);
        assert_eq!(round_tripped.turn, 20.0);
        assert_eq!(round_tripped.attitude_r, 1.5);
        assert_eq!(round_tripped.attitude_p, -2.25);
        assert_eq!(round_tripped.arm_x, 40);
        assert_eq!(round_tripped.arm_z, -10);
        assert_eq!(round_tripped.claw, 200);
    }

    #[test]
    fn channel_b_frame_stays_well_under_the_single_datagram_budget() {
        let cmd = TeleopCommand {
            vx: 0.0,
            vy: 0.0,
            turn: 0.0,
            attitude_r: 0.0,
            attitude_p: 0.0,
            attitude_y: 0.0,
            arm_x: 0,
            arm_z: 0,
            claw: 0,
        };
        let frame = ChannelBFrameData {
            timestamp: 0,
            seq: 0,
            tick_id: 0,
            category: ChannelBCategory::Command,
            region_id: ALL_REGIONS,
            fields: cmd.pack(),
        };
        let bytes = encode_channel_b_frame(&frame);
        assert!(bytes.len() < roboprotocol_core::sizing::SINGLE_DATAGRAM_BUDGET_BYTES as usize);
    }

    #[test]
    fn telemetry_data_round_trips_for_the_xgo_lite_15_dof_case() {
        let motors: Vec<f32> = (0..15).map(|i| i as f32 * 1.5 - 10.0).collect();
        let telemetry = TelemetryData { battery: 73, roll: -0.58, pitch: 2.6, yaw: 14.69, motors: motors.clone() };
        let bytes = telemetry.pack();
        let decoded = TelemetryData::unpack(&bytes).unwrap();
        assert_eq!(decoded.battery, 73);
        assert_eq!(decoded.roll, -0.58);
        assert_eq!(decoded.pitch, 2.6);
        assert_eq!(decoded.yaw, 14.69);
        assert_eq!(decoded.motors.len(), 15);
        for (a, b) in decoded.motors.iter().zip(motors.iter()) {
            assert!((a - b).abs() < 0.01);
        }
    }
}
