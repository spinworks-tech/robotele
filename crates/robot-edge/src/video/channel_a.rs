//! Wires raw `rpicam-vid` stdout bytes through `AnnexBSplitter` into a
//! stream of complete NAL units, handed to the main event loop
//! (`quic_server.rs`) via `VideoRx` -- deliberately *not* a plain `mpsc`
//! queue.
//!
//! `chunk_nal` (splitting one NAL into QUIC-datagram-sized pieces) used
//! to run here, feeding an unbounded `mpsc` channel of pre-chunked
//! payloads that the main loop drained one chunk per `select!` iteration.
//! Under any real load (competing socket/tick/bridge events), that drain
//! rate falls behind the encoder's production rate, and since the queue
//! was unbounded, nothing ever caught up -- the backlog just grew for the
//! life of the connection, so video lag increased monotonically the
//! longer a session ran (confirmed on real hardware: a simple hand-wave
//! test showed roughly a second of lag that never recovered). That's
//! exactly the head-of-line-blocking-by-buffering failure mode Channel
//! B's own "latest wins, never queue stale data" design (DESIGN.md §2.2,
//! FR-1.7) exists to avoid -- video just never got the same treatment.
//!
//! A single `watch::channel` (the first fix attempted here) is *almost*
//! right but not quite: it overwrites whatever the consumer hasn't read
//! yet unconditionally, and that includes SPS/PPS/IDR NALs. If one of
//! those gets superseded before the main loop gets to it, it's silently
//! never sent -- and the decoder can't produce any picture at all without
//! them (confirmed on real hardware: ffplay's stderr filled with
//! "non-existing PPS 0 referenced" and no frame ever decoded). So NALs
//! split across two channels here, per `roboprotocol_core::video::
//! nal_is_critical`: SPS/PPS/IDR go over an unbounded `mpsc` -- safe
//! because they're rare (at most once per GOP; `--intra 30` at 30fps is
//! about once a second) and always delivered in order, no matter how
//! backlogged the consumer is. Everything else (P slices) goes over a
//! `watch::channel`, same coalescing-to-latest behavior as before.
//! `VideoRx::recv` prefers the critical channel whenever both have
//! something ready, so a decoder-breaking NAL is never left waiting
//! behind a backlog of skippable ones.
//!
//! `chunk_nal` moves to the consumer (`quic_server.rs`), fired once per
//! NAL actually processed rather than once per NAL produced -- so a
//! delta NAL that gets superseded before the main loop gets to it is
//! never chunked/sent at all, not even partially. This is safe for the
//! same reason a lossy WiFi link already is: `roboprotocol_core::video`'s
//! own module doc already documents "a lost or reordered datagram just
//! glitches/drops that NAL; the decoder recovers cleanly at the next
//! I-frame" as expected behavior, not a new failure mode -- dropping a
//! superseded *delta* NAL here is the same thing, just decided locally
//! instead of by the network. Critical NALs are exempt from this by
//! construction (they're never superseded, only queued).

use tokio::sync::{mpsc, watch};

use roboprotocol_core::recording::CONTROL_SOURCE_SENTINEL;
use roboprotocol_core::timestamp::now_micros;
use roboprotocol_core::video::{nal_is_critical, AnnexBSplitter};
use roboprotocol_recording::{Category, Record, Recorder};

/// `AnnexBSplitter::push` hands back NAL bytes without their start code
/// (deliberate -- the wire chunking doesn't waste bytes resending it, see
/// `roboprotocol_core::video`'s docs). operator-console's own recording
/// tap re-prepends this same start code before recording, so recorded
/// video payloads are uniformly "start-code + NAL" on both ends -- a
/// replay tool can concatenate them straight into a valid Annex-B stream.
const START_CODE: [u8; 4] = [0, 0, 0, 1];

/// `(nal_id, complete NAL bytes without start code)`.
pub type Nal = (u32, Vec<u8>);

/// See this module's doc comment for why this is two channels, not one.
pub struct VideoRx {
    critical: mpsc::UnboundedReceiver<Nal>,
    latest_delta: watch::Receiver<Option<Nal>>,
}

impl VideoRx {
    /// Waits for the next NAL to send. SPS/PPS/IDR NALs are always
    /// delivered, in order, and take priority over delta NALs whenever
    /// both are ready. Returns `None` (via forever-pending) once the
    /// encoder task has ended -- degraded UX, not a safety event; the
    /// caller just stops offering video for the rest of the session.
    pub async fn recv(&mut self) -> Option<Nal> {
        tokio::select! {
            biased;
            Some(nal) = self.critical.recv() => Some(nal),
            changed = self.latest_delta.changed() => {
                if changed.is_err() {
                    return std::future::pending().await;
                }
                self.latest_delta.borrow_and_update().clone()
            }
        }
    }
}

pub fn spawn_encoder(mut raw_rx: mpsc::UnboundedReceiver<Vec<u8>>, recorder: Recorder) -> VideoRx {
    let (critical_tx, critical_rx) = mpsc::unbounded_channel();
    let (delta_tx, delta_rx) = watch::channel(None);
    tokio::spawn(async move {
        let mut splitter = AnnexBSplitter::new();
        let mut next_nal_id: u32 = 0;
        while let Some(data) = raw_rx.recv().await {
            for nal in splitter.push(&data) {
                let mut payload = Vec::with_capacity(START_CODE.len() + nal.len());
                payload.extend_from_slice(&START_CODE);
                payload.extend_from_slice(&nal);
                recorder.enqueue(Category::VideoA, Record { capture_us: now_micros(), control_source: CONTROL_SOURCE_SENTINEL, payload });

                let nal_id = next_nal_id;
                next_nal_id = next_nal_id.wrapping_add(1);
                let delivered = if nal_is_critical(&nal) {
                    critical_tx.send((nal_id, nal)).is_ok()
                } else {
                    delta_tx.send(Some((nal_id, nal))).is_ok()
                };
                if !delivered {
                    return; // every receiver dropped -- nothing left to feed
                }
            }
        }
    });
    VideoRx { critical: critical_rx, latest_delta: delta_rx }
}
