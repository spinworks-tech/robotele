//! Encode/decode `ChannelBFrame` FlatBuffers. Mirrors robot-edge's
//! `channel_b.rs` -- see that file's docs for the FR-1.5 command-shape
//! rationale; `TeleopCommand` here must stay byte-layout-identical to it.

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

/// Mirrors robot-edge's `TeleopCommand` -- must stay byte-layout-identical.
/// Wire encoding is `VelocityAttitudeCommand` (12B) ++ `CartesianCommand`
/// (8B) from `roboprotocol_core::profile`, not a locally hand-rolled
/// layout -- see robot-edge's `channel_b.rs` module docs (FR-1.5).
///
/// **Does not carry `action_id` any more** (FR-1.8, `REQUIREMENTS.md`):
/// discrete one-shot triggers moved to a dedicated Channel C RPC
/// (`roboprotocol_core::action_trigger`, this crate's
/// `action_trigger_handler.rs`) so a genuine new trigger can never be
/// silently dropped along with a stale continuous update it happened to
/// be batched with.
pub struct TeleopCommand {
    pub vx: f32,
    pub vy: f32,
    pub turn: f32,
    pub attitude_r: f32,
    pub attitude_p: f32,
    pub attitude_y: f32,
    /// Arm position (mm), Cartesian-commanded (xgolib `arm(x, z)`), not a
    /// velocity -- unlike vx/vy/turn this is a held absolute position with
    /// no auto-stop/staleness handling; it holds wherever last sent.
    pub arm_x: i16,
    pub arm_z: i16,
    /// Gripper position, 0-255, xgolib `claw(pos)`.
    pub claw: u8,
}

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
        // 12B VelocityAttitudeCommand ++ 8B CartesianCommand, Standard tier
        // (see robot-edge's `channel_b.rs`, which uses these same lengths
        // as named constants -- kept as literals here since this crate's
        // `unpack` is test-only, unlike robot-edge's live receive path).
        let va = VelocityAttitudeCommand::unpack_standard(bytes.get(0..12)?)?;
        let arm = CartesianCommand::unpack_standard(bytes.get(12..20)?)?;
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

/// Mirrors robot-edge's `TelemetryData` -- must stay byte-layout-identical.
pub struct TelemetryData {
    pub battery: u8,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub motors: Vec<f32>,
}

const TELEMETRY_HEADER_LEN: usize = 7;

impl TelemetryData {
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
    fn teleop_command_round_trips_and_matches_robot_edges_byte_layout() {
        // Same values as robot-edge's channel_b.rs equivalent test --
        // both crates must produce byte-identical wire output for the same
        // command (FR-1.5's shape-derived layout, not an ad hoc one).
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
        let bytes = cmd.pack();
        assert_eq!(bytes.len(), 20, "VelocityAttitudeCommand (12B) + CartesianCommand (8B)");

        let round_tripped = TeleopCommand::unpack(&bytes).unwrap();
        assert_eq!(round_tripped.vx, 12.0);
        assert_eq!(round_tripped.vy, -3.0);
        assert_eq!(round_tripped.turn, 20.0);
        assert_eq!(round_tripped.attitude_r, 1.5);
        assert_eq!(round_tripped.attitude_p, -2.25);
        assert_eq!(round_tripped.arm_x, 40);
        assert_eq!(round_tripped.arm_z, -10);
        assert_eq!(round_tripped.claw, 200);
    }
}
