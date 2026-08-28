//! Channel B payload sizing, quantization tiers, and body-region splitting.
//!
//! 1:1 port of `simulator/roboprotocol_sim/protocol/sizing.py` (DESIGN.md
//! §2.2.1) — every constant here must match that file. Kept as the Rust
//! source of truth's sibling, not a reimplementation from the spec text,
//! so the real implementation and the simulator's validated model can't
//! silently drift apart.

/// Quantization tier: bytes-per-field, matching `sizing.py::QuantizationTier`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum QuantizationTier {
    Compact = 1,  // int8 delta-coded
    Standard = 2, // int16 fixed-point
    Full = 4,     // float32
}

impl QuantizationTier {
    pub const fn bytes_per_field(self) -> u32 {
        self as u32
    }
}

/// FR-3.1: haptic wrench/contact-force fields may never quantize below Standard.
pub const HAPTIC_MIN_TIER: QuantizationTier = QuantizationTier::Standard;

pub const NTP_TIMESTAMP_BYTES: u32 = 8;
pub const SEQUENCE_NUMBER_BYTES: u32 = 8;
pub const HMAC_SHA256_BYTES: u32 = 32;
pub const FLATBUFFERS_FRAMING_BYTES: u32 = 12;
/// Only present on datagrams carrying the haptic category.
pub const TDPA_METADATA_BYTES: u32 = 8;

pub const FIXED_OVERHEAD_BYTES: u32 =
    NTP_TIMESTAMP_BYTES + SEQUENCE_NUMBER_BYTES + HMAC_SHA256_BYTES + FLATBUFFERS_FRAMING_BYTES;

pub const SINGLE_DATAGRAM_BUDGET_BYTES: u32 = 1200;

// Wire-level overhead outside the application payload budget (link/bandwidth
// accounting only, not counted against SINGLE_DATAGRAM_BUDGET_BYTES).
pub const IPV4_HEADER_BYTES: u32 = 20;
pub const UDP_HEADER_BYTES: u32 = 8;
pub const QUIC_SHORT_HEADER_BYTES: u32 = 9;
pub const AEAD_TAG_BYTES: u32 = 16;

pub const WIRE_OVERHEAD_BYTES: u32 =
    IPV4_HEADER_BYTES + UDP_HEADER_BYTES + QUIC_SHORT_HEADER_BYTES + AEAD_TAG_BYTES;

#[derive(Debug, thiserror::Error)]
#[error(
    "{tier:?} ({bytes}B/field) is below the FR-3.1 haptic floor (Standard, {floor}B/field)",
    bytes = tier.bytes_per_field(),
    floor = HAPTIC_MIN_TIER.bytes_per_field()
)]
pub struct HapticTierViolation {
    pub tier: QuantizationTier,
}

pub fn validate_haptic_tier(tier: QuantizationTier) -> Result<(), HapticTierViolation> {
    if tier.bytes_per_field() < HAPTIC_MIN_TIER.bytes_per_field() {
        return Err(HapticTierViolation { tier });
    }
    Ok(())
}

pub const fn fixed_overhead_bytes(carries_haptic: bool) -> u32 {
    FIXED_OVERHEAD_BYTES + if carries_haptic { TDPA_METADATA_BYTES } else { 0 }
}

pub const fn application_payload_bytes(
    num_fields: u32,
    tier: QuantizationTier,
    carries_haptic: bool,
) -> u32 {
    num_fields * tier.bytes_per_field() + fixed_overhead_bytes(carries_haptic)
}

pub const fn wire_bytes(app_payload_bytes: u32) -> u32 {
    app_payload_bytes + WIRE_OVERHEAD_BYTES
}

/// One outbound datagram's field count and resulting sizes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DatagramPlan {
    pub num_fields: u32,
    pub app_bytes: u32,
    pub wire_bytes: u32,
}

/// Return the list of datagrams needed to carry `num_fields` fields at
/// `tier` precision, splitting by (synthetic) body region if the
/// single-datagram budget is exceeded. Mirrors `sizing.py::plan_datagrams`.
pub fn plan_datagrams(
    num_fields: u32,
    tier: QuantizationTier,
    carries_haptic: bool,
    budget_bytes: u32,
) -> Result<Vec<DatagramPlan>, HapticTierViolation> {
    if carries_haptic {
        validate_haptic_tier(tier)?;
    }

    let overhead = fixed_overhead_bytes(carries_haptic);
    let single_size = num_fields * tier.bytes_per_field() + overhead;
    if single_size <= budget_bytes || num_fields == 0 {
        return Ok(vec![DatagramPlan {
            num_fields,
            app_bytes: single_size,
            wire_bytes: wire_bytes(single_size),
        }]);
    }

    let max_fields_per_datagram =
        ((budget_bytes.saturating_sub(overhead)) / tier.bytes_per_field()).max(1);
    let n_datagrams = num_fields.div_ceil(max_fields_per_datagram);
    let base = num_fields / n_datagrams;
    let extra = num_fields % n_datagrams;

    let mut plans = Vec::with_capacity(n_datagrams as usize);
    for i in 0..n_datagrams {
        let fields_i = base + if i < extra { 1 } else { 0 };
        let app_bytes_i = fields_i * tier.bytes_per_field() + overhead;
        plans.push(DatagramPlan {
            num_fields: fields_i,
            app_bytes: app_bytes_i,
            wire_bytes: wire_bytes(app_bytes_i),
        });
    }
    Ok(plans)
}

pub fn default_plan_datagrams(
    num_fields: u32,
    tier: QuantizationTier,
    carries_haptic: bool,
) -> Result<Vec<DatagramPlan>, HapticTierViolation> {
    plan_datagrams(num_fields, tier, carries_haptic, SINGLE_DATAGRAM_BUDGET_BYTES)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xgo_lite_12_joints_fits_single_datagram_at_every_tier() {
        for tier in [
            QuantizationTier::Compact,
            QuantizationTier::Standard,
            QuantizationTier::Full,
        ] {
            let plans = default_plan_datagrams(12, tier, false).unwrap();
            assert_eq!(plans.len(), 1, "{tier:?} should not need splitting for 12 fields");
            assert!(plans[0].app_bytes <= SINGLE_DATAGRAM_BUDGET_BYTES);
        }
    }

    #[test]
    fn haptic_below_standard_tier_is_rejected() {
        let err = validate_haptic_tier(QuantizationTier::Compact).unwrap_err();
        assert_eq!(err.tier, QuantizationTier::Compact);
        assert!(default_plan_datagrams(6, QuantizationTier::Compact, true).is_err());
    }

    #[test]
    fn haptic_at_or_above_standard_tier_is_allowed() {
        assert!(validate_haptic_tier(QuantizationTier::Standard).is_ok());
        assert!(validate_haptic_tier(QuantizationTier::Full).is_ok());
    }

    #[test]
    fn full_body_50_dof_splits_by_body_region_at_full_tier() {
        // 50 fields * 4B/field (Full) + 60B fixed overhead = 260B, well under
        // budget -- use a synthetic tiny budget to force splitting instead,
        // exercising the same code path a real oversized full-body command
        // (e.g. combined with per-field metadata this module doesn't model)
        // would hit.
        let plans = plan_datagrams(50, QuantizationTier::Full, false, 120).unwrap();
        assert!(plans.len() > 1, "expected a split across multiple datagrams");
        let total: u32 = plans.iter().map(|p| p.num_fields).sum();
        assert_eq!(total, 50, "split must not drop or duplicate fields");
        for p in &plans {
            assert!(p.app_bytes <= 120, "each split datagram must respect the budget");
        }
    }

    #[test]
    fn overhead_matches_simulator_constants() {
        // Pinned against simulator/roboprotocol_sim/protocol/sizing.py --
        // if these ever drift, one side of the repo silently disagrees
        // with the other about what fits in a datagram.
        assert_eq!(FIXED_OVERHEAD_BYTES, 60); // 8 + 8 + 32 + 12
        assert_eq!(fixed_overhead_bytes(true), 68); // + 8 TDPA
        assert_eq!(SINGLE_DATAGRAM_BUDGET_BYTES, 1200);
        assert_eq!(WIRE_OVERHEAD_BYTES, 53); // 20 + 8 + 9 + 16
    }
}
