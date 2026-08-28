//! In-process H.264 decode (`openh264`) + direct pixel-buffer display
//! (`minifb`), as an alternative to `playback.rs`'s `ffplay` subprocess.
//!
//! Motivation: the `ffplay` path works, but everything about it is a
//! black box we can only steer through CLI flags -- and that box has
//! already bitten us once (`-avioflags direct` silently broke the
//! Annex-B demuxer, see `playback.rs`'s doc comment). Decoding in-process
//! means no subprocess, no stdin pipe, no fighting flags for buffering
//! behavior we can't see into -- we own the decode call and the frame
//! buffer directly, so "drop stale, never queue" is enforced in our own
//! code all the way to the pixel instead of hoped for from a flag.
//!
//! Runs on a plain OS thread, not a tokio task: `main.rs` uses a
//! `current_thread` runtime, and `minifb`'s window pump is a blocking
//! call that would stall every other async task (QUIC I/O, the HUD, ...)
//! if it shared that runtime's one thread. Communication with the async
//! side happens over `std::sync::mpsc` (critical NALs) and a small
//! mutex-guarded latest-value slot (delta NALs) -- the same two-tier
//! priority split as `playback.rs`'s `VideoTx`/`feed_playback`, just
//! built from `std::sync` primitives instead of `tokio::sync` since this
//! side of the channel is a synchronous thread, not an async task.

use std::sync::mpsc::{self, RecvTimeoutError};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::Context;
use openh264::decoder::{DecodedYUV, Decoder};
use openh264::formats::YUVSource;

/// How often the render thread wakes up when no critical NAL has arrived,
/// to check for a fresh delta NAL and pump the window's event loop (so a
/// closed-window click is noticed promptly even during a lull in video).
/// Well under one frame interval at any realistic camera framerate, so it
/// never becomes the latency bottleneck itself.
const POLL_INTERVAL: Duration = Duration::from_millis(5);

struct LatestSlot {
    value: Mutex<Option<Vec<u8>>>,
}

impl LatestSlot {
    fn new() -> Self {
        Self { value: Mutex::new(None) }
    }

    fn set(&self, bytes: Vec<u8>) {
        *self.value.lock().unwrap() = Some(bytes);
    }

    fn take(&self) -> Option<Vec<u8>> {
        self.value.lock().unwrap().take()
    }
}

pub struct NativeVideoTx {
    critical_tx: mpsc::Sender<Vec<u8>>,
    latest_delta: Arc<LatestSlot>,
}

impl NativeVideoTx {
    /// Same contract as `playback::VideoTx::send`: `is_critical` (from
    /// `roboprotocol_core::video::nal_is_critical`) picks the
    /// always-delivered path (SPS/PPS/IDR) or the latest-wins one (delta
    /// frames). Errors are ignored -- a dropped receiver just means the
    /// window closed or the render thread exited, not a session-safety
    /// event.
    pub fn send(&self, bytes: Vec<u8>, is_critical: bool) {
        if is_critical {
            let _ = self.critical_tx.send(bytes);
        } else {
            self.latest_delta.set(bytes);
        }
    }
}

pub fn spawn_native_playback() -> NativeVideoTx {
    let (critical_tx, critical_rx) = mpsc::channel();
    let latest_delta = Arc::new(LatestSlot::new());
    let render_delta = latest_delta.clone();

    std::thread::Builder::new()
        .name("native-video-render".to_string())
        .spawn(move || {
            if let Err(e) = render_loop(critical_rx, render_delta) {
                tracing::warn!(error = %e, "native video render thread exited");
            }
        })
        .expect("spawning native-video-render thread");

    NativeVideoTx { critical_tx, latest_delta }
}

fn render_loop(critical_rx: mpsc::Receiver<Vec<u8>>, latest_delta: Arc<LatestSlot>) -> anyhow::Result<()> {
    let mut decoder = Decoder::new().context("creating openh264 decoder")?;
    let mut window: Option<minifb::Window> = None;
    let mut rgb_buf: Vec<u8> = Vec::new();
    let mut pixel_buf: Vec<u32> = Vec::new();

    loop {
        // Critical NALs (SPS/PPS/IDR) take priority whenever one is
        // ready; `recv_timeout` both waits for one and doubles as this
        // loop's tick when none arrives, so there's no separate sleep.
        let nal = match critical_rx.recv_timeout(POLL_INTERVAL) {
            Ok(nal) => Some(nal),
            Err(RecvTimeoutError::Timeout) => latest_delta.take(),
            Err(RecvTimeoutError::Disconnected) => return Ok(()), // session ending
        };

        let mut displayed = false;
        if let Some(nal) = nal {
            match decoder.decode(&nal) {
                Ok(Some(image)) => {
                    display_frame(&image, &mut window, &mut rgb_buf, &mut pixel_buf)?;
                    displayed = true;
                }
                Ok(None) => {} // header-only NAL (e.g. bare SPS/PPS) -- no picture yet
                Err(e) => tracing::warn!(error = %e, "openh264 decode error"),
            }
        }

        // No new frame this tick -- still pump the window so a
        // close-button click or resize is noticed without waiting for
        // the next video frame to arrive.
        if !displayed {
            if let Some(win) = window.as_mut() {
                win.update();
            }
        }

        if let Some(win) = &window {
            if !win.is_open() {
                return Ok(());
            }
        }
    }
}

/// Converts one decoded YUV420 frame to minifb's packed-0RGB `u32`
/// buffer and presents it, opening the window (sized to the stream's
/// actual resolution) on the first frame.
fn display_frame(image: &DecodedYUV, window: &mut Option<minifb::Window>, rgb_buf: &mut Vec<u8>, pixel_buf: &mut Vec<u32>) -> anyhow::Result<()> {
    let (w, h) = image.dimensions();
    if window.is_none() {
        let opts = minifb::WindowOptions { resize: true, ..minifb::WindowOptions::default() };
        *window = Some(minifb::Window::new("RoboProtocol -- native video", w, h, opts).context("opening native video window")?);
    }
    let win = window.as_mut().expect("just constructed above if it was None");

    let needed_rgb = w * h * 3;
    if rgb_buf.len() != needed_rgb {
        rgb_buf.resize(needed_rgb, 0);
        pixel_buf.resize(w * h, 0);
    }
    image.write_rgb8(rgb_buf);
    for (px, chunk) in pixel_buf.iter_mut().zip(rgb_buf.chunks_exact(3)) {
        *px = (u32::from(chunk[0]) << 16) | (u32::from(chunk[1]) << 8) | u32::from(chunk[2]);
    }

    win.update_with_buffer(pixel_buf, w, h).context("presenting decoded frame")
}
