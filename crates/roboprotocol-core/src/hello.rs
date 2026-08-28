//! `HELLO` capability handshake negotiation (DESIGN.md §1.2, REQUIREMENTS.md FR-6).
//!
//! This module is pure negotiation logic over already-decoded fields; wire
//! encode/decode of the `RoboProtocolHello` FlatBuffers table lives in
//! `roboprotocol-proto`.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProtocolVersion {
    pub major: u8,
    pub minor: u8,
}

impl ProtocolVersion {
    pub const fn new(major: u8, minor: u8) -> Self {
        Self { major, minor }
    }

    pub const fn encode(self) -> u16 {
        ((self.major as u16) << 8) | self.minor as u16
    }

    pub const fn decode(v: u16) -> Self {
        Self { major: (v >> 8) as u8, minor: (v & 0xFF) as u8 }
    }
}

/// `capability_bitmask` bit assignments (DESIGN.md §1.2.2's "Feature flags"
/// example list). Additional bits may be added later behind the `extensions`
/// TLV tail without breaking older endpoints (NFR-5.2).
pub mod capability_bits {
    pub const TDPA_SUPPORT: u64 = 1 << 0;
    pub const FLEXFEC_SUPPORT: u64 = 1 << 1;
    pub const ROS2_BRIDGE: u64 = 1 << 2;
    pub const CONNECTION_MIGRATION: u64 = 1 << 3;
}

/// `supported_task_classes` bit assignments (DESIGN.md §1.2.2: "Bit 0=Class
/// B, 1=C, 2=D, 3=E").
pub mod task_class_bits {
    pub const CLASS_B: u8 = 1 << 0;
    pub const CLASS_C: u8 = 1 << 1;
    pub const CLASS_D: u8 = 1 << 2;
    pub const CLASS_E: u8 = 1 << 3;
}

/// `supported_quantization_tiers` bit assignments.
pub mod quantization_tier_bits {
    pub const FULL: u8 = 1 << 0;
    pub const STANDARD: u8 = 1 << 1;
    pub const COMPACT: u8 = 1 << 2;
}

#[derive(Debug, Clone, Copy)]
pub struct HelloCapabilities {
    pub protocol_version: ProtocolVersion,
    pub capability_bitmask: u64,
    pub supported_task_classes: u8,
    pub supported_quantization_tiers: u8,
    pub max_control_rate_hz: u16,
}

#[derive(Debug, Clone, Copy)]
pub struct NegotiatedSession {
    pub version: ProtocolVersion,
    pub capability_bitmask: u64,
    pub task_classes: u8,
    pub quantization_tiers: u8,
    pub max_control_rate_hz: u16,
}

#[derive(Debug, thiserror::Error, PartialEq, Eq)]
#[error("HELLO_INCOMPATIBLE: no shared major version (local={local_major}, peer={peer_major})")]
pub struct IncompatibleVersion {
    pub local_major: u8,
    pub peer_major: u8,
}

/// FR-6.3: negotiate the highest mutually supported major.minor version.
/// Two endpoints sharing a major version but differing minors interoperate
/// (minor = min of both); no shared major version is a hard rejection
/// before any actuation stream opens (FR-6.1).
///
/// FR-6.2: the operative session capability/task-class/quantization-tier
/// sets are each the bitwise AND of both endpoints' advertised sets.
pub fn negotiate(
    local: &HelloCapabilities,
    peer: &HelloCapabilities,
) -> Result<NegotiatedSession, IncompatibleVersion> {
    if local.protocol_version.major != peer.protocol_version.major {
        return Err(IncompatibleVersion {
            local_major: local.protocol_version.major,
            peer_major: peer.protocol_version.major,
        });
    }
    Ok(NegotiatedSession {
        version: ProtocolVersion::new(
            local.protocol_version.major,
            local.protocol_version.minor.min(peer.protocol_version.minor),
        ),
        capability_bitmask: local.capability_bitmask & peer.capability_bitmask,
        task_classes: local.supported_task_classes & peer.supported_task_classes,
        quantization_tiers: local.supported_quantization_tiers & peer.supported_quantization_tiers,
        max_control_rate_hz: local.max_control_rate_hz.min(peer.max_control_rate_hz),
    })
}

/// FR-6.4: if a safety-relevant capability required for the requested
/// Operational Task Profile is missing from the negotiated set (e.g. TDPA
/// support for haptic feedback), the initiating endpoint must refuse Full
/// Teleoperation Mode for that capability rather than silently degrading.
/// Returns the missing bits, if any.
pub fn missing_required_capabilities(required: u64, negotiated: &NegotiatedSession) -> Option<u64> {
    let missing = required & !negotiated.capability_bitmask;
    (missing != 0).then_some(missing)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn caps(major: u8, minor: u8, bitmask: u64, classes: u8, tiers: u8, rate: u16) -> HelloCapabilities {
        HelloCapabilities {
            protocol_version: ProtocolVersion::new(major, minor),
            capability_bitmask: bitmask,
            supported_task_classes: classes,
            supported_quantization_tiers: tiers,
            max_control_rate_hz: rate,
        }
    }

    #[test]
    fn version_round_trips_through_the_wire_encoding() {
        let v = ProtocolVersion::new(1, 3);
        assert_eq!(v.encode(), 0x0103);
        assert_eq!(ProtocolVersion::decode(0x0103), v);
    }

    #[test]
    fn same_major_different_minor_negotiates_the_lower_minor() {
        let local = caps(1, 3, capability_bits::TDPA_SUPPORT, task_class_bits::CLASS_D, quantization_tier_bits::STANDARD, 1000);
        let peer = caps(1, 0, capability_bits::TDPA_SUPPORT, task_class_bits::CLASS_D, quantization_tier_bits::STANDARD, 100);
        let session = negotiate(&local, &peer).unwrap();
        assert_eq!(session.version, ProtocolVersion::new(1, 0));
        assert_eq!(session.max_control_rate_hz, 100);
    }

    #[test]
    fn no_shared_major_version_is_rejected_before_actuation() {
        let local = caps(2, 0, 0, 0, 0, 0);
        let peer = caps(1, 5, 0, 0, 0, 0);
        let err = negotiate(&local, &peer).unwrap_err();
        assert_eq!(err, IncompatibleVersion { local_major: 2, peer_major: 1 });
    }

    #[test]
    fn capability_and_task_class_and_tier_sets_intersect() {
        let local = caps(
            1, 0,
            capability_bits::TDPA_SUPPORT | capability_bits::FLEXFEC_SUPPORT,
            task_class_bits::CLASS_B | task_class_bits::CLASS_D,
            quantization_tier_bits::FULL | quantization_tier_bits::STANDARD,
            1000,
        );
        let peer = caps(
            1, 0,
            capability_bits::FLEXFEC_SUPPORT | capability_bits::ROS2_BRIDGE,
            task_class_bits::CLASS_D | task_class_bits::CLASS_E,
            quantization_tier_bits::STANDARD | quantization_tier_bits::COMPACT,
            50,
        );
        let session = negotiate(&local, &peer).unwrap();
        assert_eq!(session.capability_bitmask, capability_bits::FLEXFEC_SUPPORT);
        assert_eq!(session.task_classes, task_class_bits::CLASS_D);
        assert_eq!(session.quantization_tiers, quantization_tier_bits::STANDARD);
    }

    #[test]
    fn missing_required_capability_is_reported_for_full_teleop_refusal() {
        let local = caps(1, 0, capability_bits::TDPA_SUPPORT, task_class_bits::CLASS_B, quantization_tier_bits::STANDARD, 1000);
        // Peer (e.g. a robot with no force sensing) never advertises TDPA.
        let peer = caps(1, 0, 0, task_class_bits::CLASS_B, quantization_tier_bits::STANDARD, 1000);
        let session = negotiate(&local, &peer).unwrap();
        let missing = missing_required_capabilities(capability_bits::TDPA_SUPPORT, &session);
        assert_eq!(missing, Some(capability_bits::TDPA_SUPPORT));
    }

    #[test]
    fn no_missing_capability_when_both_sides_advertise_it() {
        let local = caps(1, 0, capability_bits::TDPA_SUPPORT, task_class_bits::CLASS_B, quantization_tier_bits::STANDARD, 1000);
        let peer = caps(1, 0, capability_bits::TDPA_SUPPORT, task_class_bits::CLASS_B, quantization_tier_bits::STANDARD, 1000);
        let session = negotiate(&local, &peer).unwrap();
        assert_eq!(missing_required_capabilities(capability_bits::TDPA_SUPPORT, &session), None);
    }
}
