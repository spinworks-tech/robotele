//! Decode `ActionTrigger` FlatBuffers <-> `roboprotocol_core::action_trigger`
//! types (FR-1.8). Robot-edge only ever decodes -- the trigger flows one
//! way, operator to robot; operator-console does the encoding.

use roboprotocol_core::action_trigger::ActionTrigger;
use roboprotocol_proto::ActionTrigger as FbActionTrigger;

pub fn decode_action_trigger(buf: &[u8]) -> anyhow::Result<ActionTrigger> {
    // Same caveat as hello_handler::decode_hello: this crate's pinned old
    // flatbuffers version has no buffer verification, so a malformed
    // buffer is undefined behavior rather than a clean error -- bounded to
    // "our own peer sent us garbage" by the authenticated QUIC/TLS 1.3
    // connection, not an open attack surface.
    let trigger = flatbuffers::get_root::<FbActionTrigger>(buf);
    Ok(ActionTrigger { action_id: trigger.action_id(), trigger_seq: trigger.trigger_seq() })
}

#[cfg(test)]
mod tests {
    use super::*;
    use flatbuffers::FlatBufferBuilder;
    use roboprotocol_proto::ActionTriggerArgs;

    #[test]
    fn decodes_a_well_formed_action_trigger() {
        let mut b = FlatBufferBuilder::new();
        let offset = FbActionTrigger::create(&mut b, &ActionTriggerArgs { action_id: 12, trigger_seq: 3 });
        b.finish(offset, None);
        let trigger = decode_action_trigger(b.finished_data()).unwrap();
        assert_eq!(trigger.action_id, 12);
        assert_eq!(trigger.trigger_seq, 3);
    }
}
