"""Channel B payload sizing, quantization tiers, and body-region splitting.

Implements DESIGN.md Sec.2.2.1 precisely:

* Single QUIC-datagram application-payload budget: 1200 bytes. This is
  everything inside the QUIC DATAGRAM frame that the application
  defines -- quantized fields + per-datagram metadata (timestamp,
  sequence number, anti-replay MAC, TDPA energy scalar on
  haptic-carrying datagrams, FlatBuffers table/vtable framing). QUIC
  short-header + UDP/IP + AEAD tag bytes sit *outside* this budget, on
  the wire, and are accounted for separately by `wire_bytes()` for
  link/bandwidth purposes (DESIGN.md's own accounting reserves
  ~150-250B of headroom between the 1200B payload budget and the
  ~1350-1452B path-MTU-safe ceiling for exactly this).
* Quantization tiers: Full=float32 (4B/field), Standard=int16 fixed
  point (2B/field), Compact=int8 delta-coded (1B/field) -- chosen
  independently per payload category (command / telemetry / haptic).
* Haptic fields are floored at Standard (REQUIREMENTS.md FR-3.1):
  Compact is never a legal tier for haptic wrench/contact-force
  fields, because the resulting quantization noise feeds the TDPA
  energy integral and can reintroduce non-passive energy injection.
* If a category still exceeds the budget at its minimum allowed tier,
  it is split into multiple independently-timestamped datagrams
  grouped by body region rather than fragmented (RFC 9221 datagrams
  have no fragmentation/reassembly primitive). This module models the
  split as an even partition of the field count into N datagrams, each
  re-paying the fixed per-datagram overhead -- a reasonable proxy for
  "group by body region" without needing an actual limb topology.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum


class QuantizationTier(IntEnum):
    COMPACT = 1   # int8 delta-coded
    STANDARD = 2  # int16 fixed-point
    FULL = 4      # float32


HAPTIC_MIN_TIER = QuantizationTier.STANDARD

# Fixed per-datagram application-payload overhead (bytes), all counted
# toward the 1200B budget -- see module docstring.
NTP_TIMESTAMP_BYTES = 8
SEQUENCE_NUMBER_BYTES = 8
HMAC_SHA256_BYTES = 32
FLATBUFFERS_FRAMING_BYTES = 12
TDPA_METADATA_BYTES = 8  # only present on datagrams carrying the haptic category

FIXED_OVERHEAD_BYTES = (
    NTP_TIMESTAMP_BYTES + SEQUENCE_NUMBER_BYTES + HMAC_SHA256_BYTES + FLATBUFFERS_FRAMING_BYTES
)

SINGLE_DATAGRAM_BUDGET_BYTES = 1200

# Wire-level overhead outside the application payload budget, added for
# link/bandwidth accounting only (documented assumptions -- see SPEC.md).
IPV4_HEADER_BYTES = 20
UDP_HEADER_BYTES = 8
QUIC_SHORT_HEADER_BYTES = 9  # ~1B flags + ~4B connection ID + ~4B packet number
AEAD_TAG_BYTES = 16  # AES-256-GCM

WIRE_OVERHEAD_BYTES = IPV4_HEADER_BYTES + UDP_HEADER_BYTES + QUIC_SHORT_HEADER_BYTES + AEAD_TAG_BYTES


class HapticTierViolation(ValueError):
    pass


def validate_haptic_tier(tier: QuantizationTier) -> None:
    if tier.value < HAPTIC_MIN_TIER.value:
        raise HapticTierViolation(
            f"{tier.name} ({tier.value}B/field) is below the FR-3.1 haptic floor "
            f"({HAPTIC_MIN_TIER.name}, {HAPTIC_MIN_TIER.value}B/field)"
        )


def fixed_overhead_bytes(carries_haptic: bool) -> int:
    return FIXED_OVERHEAD_BYTES + (TDPA_METADATA_BYTES if carries_haptic else 0)


def application_payload_bytes(num_fields: int, tier: QuantizationTier, carries_haptic: bool = False) -> int:
    return num_fields * tier.value + fixed_overhead_bytes(carries_haptic)


def wire_bytes(app_payload_bytes: int) -> int:
    return app_payload_bytes + WIRE_OVERHEAD_BYTES


@dataclass(frozen=True)
class DatagramPlan:
    """One outbound datagram's field count and resulting sizes."""

    num_fields: int
    app_bytes: int
    wire_bytes: int


def plan_datagrams(
    num_fields: int,
    tier: QuantizationTier,
    carries_haptic: bool = False,
    budget_bytes: int = SINGLE_DATAGRAM_BUDGET_BYTES,
) -> list[DatagramPlan]:
    """Return the list of datagrams needed to carry `num_fields` fields
    at `tier` precision, splitting by (synthetic) body region if the
    single-datagram budget is exceeded.
    """
    if carries_haptic:
        validate_haptic_tier(tier)

    overhead = fixed_overhead_bytes(carries_haptic)
    single_size = num_fields * tier.value + overhead
    if single_size <= budget_bytes or num_fields <= 0:
        return [DatagramPlan(num_fields, single_size, wire_bytes(single_size))]

    max_fields_per_datagram = max(1, (budget_bytes - overhead) // tier.value)
    n_datagrams = math.ceil(num_fields / max_fields_per_datagram)
    base, extra = divmod(num_fields, n_datagrams)
    plans = []
    for i in range(n_datagrams):
        fields_i = base + (1 if i < extra else 0)
        app_bytes_i = fields_i * tier.value + overhead
        plans.append(DatagramPlan(fields_i, app_bytes_i, wire_bytes(app_bytes_i)))
    return plans
