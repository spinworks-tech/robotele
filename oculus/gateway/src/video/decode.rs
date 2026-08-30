//! In-process H.264 decode (`openh264`) + throttled JPEG re-encode,
//! feeding decoded frames to the WebSocket broadcaster.
//!
//! Unlike operator-console's `video/native_playback.rs` (which this was
//! originally adapted from), decode here must never skip a NAL: H.264
//! P-frames reference the immediately preceding *decoded* frame, not just
//! the last keyframe, so a "keep only the latest, drop the rest" policy
//! at the decode stage breaks that reference chain and makes the decoder
//! report `dsRefLost`/`dsNoParamSets` on nearly every subsequent frame
//! until the next IDR -- confirmed empirically against a real robot (ok=5,
//! err=130 per 5s with the old latest-wins-at-decode design). The fix:
//! every completed Channel A NAL is decoded in arrival order with nothing
//! dropped; only the comparatively expensive JPEG-encode-and-broadcast
//! step downstream of a successful decode is throttled
//! (`MIN_BROADCAST_INTERVAL`), since that's fine to skip -- the browser
//! doesn't need every single decoded frame, the decoder does.
//!
//! Runs on a plain OS thread, not a tokio task, for the same reason as
//! `native_playback.rs`: decode is a blocking, CPU-bound loop, and this
//! process's other work (QUIC I/O, the WebSocket server) is async.

use std::sync::mpsc;
use std::time::{Duration, Instant};

use openh264::decoder::Decoder;
use openh264::formats::YUVSource;

/// Caps how often a decoded frame is JPEG-encoded and pushed to the
/// broadcaster -- decode cadence itself is never throttled (see module
/// doc), only this downstream step.
const MIN_BROADCAST_INTERVAL: Duration = Duration::from_millis(66); // ~15fps cap
const JPEG_QUALITY: u8 = 75;
const SUMMARY_INTERVAL: Duration = Duration::from_secs(5);

pub struct VideoDecoderTx {
    nal_tx: mpsc::Sender<Vec<u8>>,
}

impl VideoDecoderTx {
    /// Every completed Channel A NAL, in arrival order. Must never be
    /// dropped by the caller -- see module doc.
    pub fn send(&self, bytes: Vec<u8>) {
        let _ = self.nal_tx.send(bytes);
    }
}

pub fn spawn_video_decoder(jpeg_tx: tokio::sync::mpsc::Sender<Vec<u8>>) -> VideoDecoderTx {
    let (nal_tx, nal_rx) = mpsc::channel();

    std::thread::Builder::new()
        .name("oculus-video-decode".to_string())
        .spawn(move || {
            if let Err(e) = decode_loop(nal_rx, jpeg_tx) {
                tracing::warn!(error = %e, "video decode thread exited");
            }
        })
        .expect("spawning oculus-video-decode thread");

    VideoDecoderTx { nal_tx }
}

fn decode_loop(nal_rx: mpsc::Receiver<Vec<u8>>, jpeg_tx: tokio::sync::mpsc::Sender<Vec<u8>>) -> anyhow::Result<()> {
    let mut decoder = Decoder::new()?;
    let mut rgb_buf: Vec<u8> = Vec::new();
    let mut last_broadcast = Instant::now() - MIN_BROADCAST_INTERVAL;

    // Individual decode failures (a chunk lost on Channel A's unreliable
    // datagrams -- see roboprotocol_core::video's doc comment) are
    // expected and self-recovering at the next IDR, not something to
    // alarm on one at a time -- logged at `debug!` and rolled up into a
    // periodic `info!` summary instead.
    let mut ok_count = 0u64;
    let mut err_count = 0u64;
    let mut last_summary = Instant::now();

    for nal in nal_rx.iter() {
        match decoder.decode(&nal) {
            Ok(Some(image)) => {
                ok_count += 1;
                if last_broadcast.elapsed() >= MIN_BROADCAST_INTERVAL {
                    let (w, h) = image.dimensions();
                    let needed_rgb = w * h * 3;
                    if rgb_buf.len() != needed_rgb {
                        rgb_buf.resize(needed_rgb, 0);
                    }
                    image.write_rgb8(&mut rgb_buf);

                    match encode_jpeg(&rgb_buf, w as u32, h as u32) {
                        Ok(jpeg) => {
                            // Drop rather than queue if the broadcaster is
                            // behind -- see ws_server.rs's module doc.
                            let _ = jpeg_tx.try_send(jpeg);
                            last_broadcast = Instant::now();
                        }
                        Err(e) => tracing::warn!(error = %e, "jpeg encode error"),
                    }
                }
            }
            Ok(None) => {} // header-only NAL (bare SPS/PPS) -- no picture yet
            Err(e) => {
                err_count += 1;
                tracing::debug!(error = %e, "openh264 decode error");
            }
        }

        if last_summary.elapsed() >= SUMMARY_INTERVAL {
            if err_count > 0 {
                tracing::info!(ok = ok_count, err = err_count, "video decode summary (last 5s)");
            }
            ok_count = 0;
            err_count = 0;
            last_summary = Instant::now();
        }
    }
    Ok(()) // sender dropped -- session ending
}

fn encode_jpeg(rgb: &[u8], w: u32, h: u32) -> anyhow::Result<Vec<u8>> {
    let mut out = Vec::new();
    let mut encoder = image::codecs::jpeg::JpegEncoder::new_with_quality(&mut out, JPEG_QUALITY);
    encoder.encode(rgb, w, h, image::ExtendedColorType::Rgb8)?;
    Ok(out)
}
