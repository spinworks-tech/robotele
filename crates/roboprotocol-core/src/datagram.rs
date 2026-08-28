//! QUIC unreliable datagrams are a single, unchannelized stream at the
//! transport layer -- `conn.dgram_send()`/`dgram_recv()` don't know
//! about "Channel A" vs "Channel B" vs E-Stop, they're all just bytes on
//! the same connection. Each datagram therefore needs a 1-byte type tag
//! prefix so the receiver knows how to decode it *before* attempting to
//! decode it.
//!
//! This matters more than it might look: this crate's pinned old
//! `flatbuffers` version (0.7, forced by the `flatc` 1.11/1.12 available
//! at build time -- see roboprotocol-proto's build.rs) has no buffer
//! verification API. `flatbuffers::get_root::<T>()` blindly reads
//! offsets out of the buffer with no bounds checking; feeding it bytes
//! that aren't actually a `T` doesn't return a clean error, it can
//! panic (garbage offset larger than the buffer) or silently produce
//! nonsense. Untagged datagrams found this out empirically: mixing
//! Channel A (video) and Channel B (command/telemetry) datagrams,
//! distinguished only by trying to `flatbuffers`-decode everything and
//! falling through on failure, crashed `operator-console` in the field
//! (real CM4 <-> PC test) the first time an actual video chunk arrived.
//!
//! E-Stop datagrams keep their own pre-existing magic byte (`0xE5`,
//! `estop::ESTOP_DATAGRAM_MAGIC`) rather than these tags, since that
//! path predates this module and doesn't collide with it.
//!
//! `untag` returns an owned, freshly-allocated copy of the payload
//! rather than a borrowed sub-slice of the original datagram buffer --
//! not just for convenience. The same unverified `flatbuffers` crate
//! does raw pointer reads in its scalar decoding that assume the buffer
//! starts at a suitably aligned address; a sub-slice one byte into an
//! existing buffer shifts that alignment and produced a real
//! "misaligned pointer dereference" panic empirically (found via a live
//! CM4 <-> PC test) where the untagged/unstripped buffer never did. A
//! fresh `Vec<u8>` allocation is, in practice, aligned again.

pub const DATAGRAM_TAG_CHANNEL_B: u8 = 0x01;
pub const DATAGRAM_TAG_CHANNEL_A: u8 = 0x02;

/// Prefix `payload` with a 1-byte datagram type tag.
pub fn tag(tag_byte: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(1 + payload.len());
    out.push(tag_byte);
    out.extend_from_slice(payload);
    out
}

/// Split a received datagram into its type tag and an owned, freshly
/// allocated copy of the remaining payload (see module docs for why this
/// must not be a borrowed sub-slice of the original buffer).
pub fn untag(datagram: &[u8]) -> Option<(u8, Vec<u8>)> {
    datagram.split_first().map(|(&t, rest)| (t, rest.to_vec()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tag_and_untag_round_trip() {
        let payload = [1u8, 2, 3, 4];
        let tagged = tag(DATAGRAM_TAG_CHANNEL_A, &payload);
        let (t, rest) = untag(&tagged).unwrap();
        assert_eq!(t, DATAGRAM_TAG_CHANNEL_A);
        assert_eq!(rest, &payload);
    }

    #[test]
    fn untag_empty_datagram_returns_none() {
        assert_eq!(untag(&[]), None);
    }
}
