//! E-Stop transport framing (DESIGN.md §2.3): a high-priority reliable
//! QUIC stream plus redundant 1kHz unreliable datagrams.
//!
//! The datagram format here is a minimal fixed 10-byte raw encoding, not
//! FlatBuffers -- this is the single sub-5ms-critical path in the whole
//! protocol (SR-4.2), and skipping FlatBuffers parsing overhead on it is
//! a deliberate, documented choice distinct from Channel A/B's own
//! framing decisions.

pub const ESTOP_STREAM_ID: u64 = 5; // next server-initiated bidi stream after SESSION_DESCRIBE (1)
pub const ESTOP_STREAM_URGENCY: u8 = 0; // quiche: lower value = higher priority
pub const ESTOP_DATAGRAM_MAGIC: u8 = 0xE5;
pub const ESTOP_REDUNDANT_HZ: u32 = 1000;
pub const ESTOP_DATAGRAM_LEN: usize = 10;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EstopDatagram {
    pub latched: bool,
    pub seq: u64,
}

impl EstopDatagram {
    pub fn encode(&self) -> [u8; ESTOP_DATAGRAM_LEN] {
        let mut out = [0u8; ESTOP_DATAGRAM_LEN];
        out[0] = ESTOP_DATAGRAM_MAGIC;
        out[1] = self.latched as u8;
        out[2..10].copy_from_slice(&self.seq.to_be_bytes());
        out
    }

    pub fn decode(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < ESTOP_DATAGRAM_LEN || bytes[0] != ESTOP_DATAGRAM_MAGIC {
            return None;
        }
        Some(Self { latched: bytes[1] != 0, seq: u64::from_be_bytes(bytes[2..10].try_into().ok()?) })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn estop_datagram_round_trips() {
        let d = EstopDatagram { latched: true, seq: 99 };
        assert_eq!(EstopDatagram::decode(&d.encode()), Some(d));

        let d2 = EstopDatagram { latched: false, seq: 0 };
        assert_eq!(EstopDatagram::decode(&d2.encode()), Some(d2));
    }

    #[test]
    fn rejects_wrong_magic_or_short_buffer() {
        let mut bytes = EstopDatagram { latched: false, seq: 1 }.encode().to_vec();
        bytes[0] = 0x00;
        assert_eq!(EstopDatagram::decode(&bytes), None);
        assert_eq!(EstopDatagram::decode(&[ESTOP_DATAGRAM_MAGIC, 1, 2, 3]), None);
    }
}
