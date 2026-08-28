//! Encode/decode `RoboProtocolHello` FlatBuffers <-> `roboprotocol_core::hello` types.

use flatbuffers::FlatBufferBuilder;
use roboprotocol_core::hello::{HelloCapabilities, ProtocolVersion};
use roboprotocol_proto::{RoboProtocolHello, RoboProtocolHelloArgs};

pub fn encode_hello(caps: &HelloCapabilities) -> Vec<u8> {
    let mut builder = FlatBufferBuilder::new();
    let args = RoboProtocolHelloArgs {
        protocol_version: caps.protocol_version.encode(),
        capability_bitmask: caps.capability_bitmask,
        supported_task_classes: caps.supported_task_classes,
        supported_quantization_tiers: caps.supported_quantization_tiers,
        max_control_rate_hz: caps.max_control_rate_hz,
        extensions: None,
    };
    let offset = RoboProtocolHello::create(&mut builder, &args);
    builder.finish(offset, None);
    builder.finished_data().to_vec()
}

pub fn decode_hello(buf: &[u8]) -> anyhow::Result<HelloCapabilities> {
    // flatc 1.11.0-generated code has no built-in buffer verification (an
    // old-generation limitation, not a v0 choice) -- a malformed/truncated
    // buffer here is undefined behavior rather than a clean error. Datagrams
    // arrive over an authenticated, integrity-protected QUIC/TLS 1.3
    // connection, which bounds this to "our own peer sent us garbage,"
    // not an open network attack surface.
    let hello = flatbuffers::get_root::<RoboProtocolHello>(buf);
    Ok(HelloCapabilities {
        protocol_version: ProtocolVersion::decode(hello.protocol_version()),
        capability_bitmask: hello.capability_bitmask(),
        supported_task_classes: hello.supported_task_classes(),
        supported_quantization_tiers: hello.supported_quantization_tiers(),
        max_control_rate_hz: hello.max_control_rate_hz(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use roboprotocol_core::hello::{capability_bits, quantization_tier_bits, task_class_bits};

    #[test]
    fn hello_round_trips_through_flatbuffers() {
        let caps = HelloCapabilities {
            protocol_version: ProtocolVersion::new(1, 0),
            capability_bitmask: capability_bits::FLEXFEC_SUPPORT,
            supported_task_classes: task_class_bits::CLASS_D,
            supported_quantization_tiers: quantization_tier_bits::STANDARD,
            max_control_rate_hz: 100,
        };
        let bytes = encode_hello(&caps);
        let decoded = decode_hello(&bytes).unwrap();
        assert_eq!(decoded.protocol_version, caps.protocol_version);
        assert_eq!(decoded.capability_bitmask, caps.capability_bitmask);
        assert_eq!(decoded.supported_task_classes, caps.supported_task_classes);
        assert_eq!(decoded.supported_quantization_tiers, caps.supported_quantization_tiers);
        assert_eq!(decoded.max_control_rate_hz, caps.max_control_rate_hz);
    }
}
