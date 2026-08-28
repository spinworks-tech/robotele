//! FlatBuffers-generated wire types for `RoboProtocolHello`,
//! `SessionDescribe`/`SessionAccept`, and `ChannelBFrame`, compiled from
//! `schemas/roboprotocol.fbs` by `build.rs` via the `flatc` compiler.

#![allow(non_snake_case, non_camel_case_types, unused_imports, clippy::all)]

include!(concat!(env!("OUT_DIR"), "/roboprotocol_generated.rs"));

pub use roboprotocol::*;
