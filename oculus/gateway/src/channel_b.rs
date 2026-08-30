//! Decode-only `ChannelBFrame` FlatBuffers, telemetry data only. Mirrors
//! operator-console's `channel_b.rs`, trimmed to what a read-only monitor
//! needs -- no `TeleopCommand`/pack side, since this gateway never sends
//! a Channel B Command frame (see `quic_client.rs`'s module doc for why
//! that's intentional, not an oversight).

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChannelBCategory {
    Command,
    Telemetry,
    Haptic,
}

fn from_fb_category(c: roboprotocol_proto::ChannelBCategory) -> ChannelBCategory {
    match c {
        roboprotocol_proto::ChannelBCategory::Telemetry => ChannelBCategory::Telemetry,
        roboprotocol_proto::ChannelBCategory::Haptic => ChannelBCategory::Haptic,
        _ => ChannelBCategory::Command,
    }
}

pub struct ChannelBFrameData {
    pub timestamp: u64,
    pub seq: u64,
    pub category: ChannelBCategory,
    pub fields: Vec<u8>,
}

pub fn decode_channel_b_frame(buf: &[u8]) -> anyhow::Result<ChannelBFrameData> {
    let f = flatbuffers::get_root::<roboprotocol_proto::ChannelBFrame>(buf);
    Ok(ChannelBFrameData {
        timestamp: f.timestamp(),
        seq: f.seq(),
        category: from_fb_category(f.category()),
        fields: f.fields().map(|v| v.to_vec()).unwrap_or_default(),
    })
}

/// Byte-layout-identical to operator-console's/robot-edge's `TelemetryData`
/// -- battery(1B) + roll/pitch/yaw(2B each, centidegrees) + N motor angles
/// (2B each, centidegrees).
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
