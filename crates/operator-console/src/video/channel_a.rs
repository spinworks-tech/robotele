//! Reassembles Channel A video datagram chunks back into an Annex-B H.264
//! byte stream via `roboprotocol_core::video::NalReassembler`.

use roboprotocol_core::video::{ChunkHeader, NalReassembler};

const START_CODE: [u8; 4] = [0, 0, 0, 1];
const MAX_PENDING_NALS: usize = 32;

pub struct ChannelAReceiver {
    reassembler: NalReassembler,
}

impl ChannelAReceiver {
    pub fn new() -> Self {
        Self { reassembler: NalReassembler::new(MAX_PENDING_NALS) }
    }

    /// Feed one received Channel A datagram. Returns Annex-B bytes
    /// (start code + NAL payload) ready to write straight to the
    /// playback pipe, if this chunk completed a NAL.
    pub fn on_datagram(&mut self, data: &[u8]) -> Option<Vec<u8>> {
        let (header, payload) = ChunkHeader::decode(data)?;
        let nal = self.reassembler.on_chunk(header, payload)?;
        let mut out = Vec::with_capacity(START_CODE.len() + nal.len());
        out.extend_from_slice(&START_CODE);
        out.extend_from_slice(&nal);
        Some(out)
    }
}

impl Default for ChannelAReceiver {
    fn default() -> Self {
        Self::new()
    }
}
