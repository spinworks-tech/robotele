//! Encode `ActionTrigger` FlatBuffers <-> `roboprotocol_core::action_trigger`
//! types (FR-1.8). Operator console only ever encodes -- the trigger flows
//! one way, operator to robot; robot-edge does the decoding.

use flatbuffers::FlatBufferBuilder;
use roboprotocol_core::action_trigger::ActionTrigger;
use roboprotocol_proto::{ActionTrigger as FbActionTrigger, ActionTriggerArgs};

pub fn encode_action_trigger(trigger: &ActionTrigger) -> Vec<u8> {
    let mut b = FlatBufferBuilder::new();
    let offset = FbActionTrigger::create(
        &mut b,
        &ActionTriggerArgs { action_id: trigger.action_id, trigger_seq: trigger.trigger_seq },
    );
    b.finish(offset, None);
    b.finished_data().to_vec()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn action_trigger_encodes_to_a_parseable_flatbuffer() {
        let trigger = ActionTrigger { action_id: 2, trigger_seq: 7 };
        let bytes = encode_action_trigger(&trigger);
        let decoded = flatbuffers::get_root::<FbActionTrigger>(&bytes);
        assert_eq!(decoded.action_id(), 2);
        assert_eq!(decoded.trigger_seq(), 7);
    }
}
