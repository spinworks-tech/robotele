//! Encode/decode `RoboProtocolHello` FlatBuffers <-> `roboprotocol_core::hello` types.
//! Mirrors robot-edge's `hello_handler.rs`.

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
    // See robot-edge's hello_handler.rs for why this is infallible parsing
    // (flatc 1.11.0 has no built-in buffer verification).
    let hello = flatbuffers::get_root::<RoboProtocolHello>(buf);
    Ok(HelloCapabilities {
        protocol_version: ProtocolVersion::decode(hello.protocol_version()),
        capability_bitmask: hello.capability_bitmask(),
        supported_task_classes: hello.supported_task_classes(),
        supported_quantization_tiers: hello.supported_quantization_tiers(),
        max_control_rate_hz: hello.max_control_rate_hz(),
    })
}
