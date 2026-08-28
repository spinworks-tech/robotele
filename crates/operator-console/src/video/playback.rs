//! Pipes reassembled Annex-B H.264 bytes into an `ffplay` subprocess for
//! a live preview window. A stopgap, not NFR-2.3's fuller operator UI.
//!
//! Feeds `ffplay`'s stdin via two channels, not one `mpsc` queue --
//! `ffplay`'s own decode+render throughput is not guaranteed to keep up
//! with 30fps, especially over a remote/virtualized display, and an
//! unbounded queue in front of a consumer that's even slightly slower
//! than the producer means the backlog only ever grows: every reassembled
//! NAL ever gets shown eventually, just further and further behind real
//! time the longer the session runs (confirmed on real hardware -- video
//! lag was on the order of a second and never recovered).
//!
//! A single `watch::channel` (the first fix attempted here) is *almost*
//! right but not quite: it overwrites whatever hasn't been fed to
//! `ffplay` yet unconditionally, including SPS/PPS/IDR NALs -- and
//! without those the decoder can't produce a picture at all (confirmed on
//! real hardware: ffplay's stderr filled with "non-existing PPS 0
//! referenced" and the preview window never appeared). So NALs split
//! across two channels here, mirroring `robot-edge`'s `video/channel_a.rs`
//! (see its doc comment for the full reasoning): the caller classifies
//! each NAL via `roboprotocol_core::video::nal_is_critical` and sends it
//! as either `critical` (unbounded `mpsc`, always delivered in order --
//! safe because these are rare, at most once per GOP) or `delta` (`watch`,
//! coalesces to latest-only under backlog, same as before). `feed_playback`
//! prefers the critical channel whenever both have something ready, so a
//! decoder-breaking NAL is never left waiting behind a backlog of
//! skippable ones. Skipped delta frames read as a choppier feed, not a
//! growing delay -- the right tradeoff for a teleoperation preview.

use std::fs::OpenOptions;
use std::path::PathBuf;
use std::process::Stdio;

use anyhow::Context;
use tokio::io::AsyncWriteExt;
use tokio::process::{Child, ChildStdin, Command};
use tokio::sync::{mpsc, watch};

/// Writes text into a file `ffplay`'s own `drawtext` filter is watching
/// (`reload=1`, re-read every frame) -- lets the operator console burn
/// live text (e.g. current camera brightness/contrast/EV/shutter) directly
/// into the displayed video, since `ffplay`'s window is a separate OS
/// window ratatui's TUI can't composite into (see this module's doc).
/// Requires `ffplay`'s own `ffmpeg` build to have `libfreetype` (and
/// `libfontconfig`, for the bare `font=monospace` name to resolve without
/// an explicit `fontfile=` path) -- not all builds do, which is why this
/// is opt-in (`--video-overlay`) rather than always on: an unsupported
/// `-vf drawtext=...` argument would otherwise break video entirely for
/// anyone whose `ffplay` lacks it, not just silently skip the overlay.
pub struct OverlayHandle {
    path: PathBuf,
}

impl OverlayHandle {
    /// Best-effort: a failed write leaves the last-shown overlay text
    /// stale, which is a cosmetic problem, not a reason to interrupt
    /// teleop control -- callers ignore the `Result`, same as this
    /// module's existing `VideoTx::send` treatment of a lost video feed.
    pub fn write(&self, text: &str) -> std::io::Result<()> {
        // Write-then-rename rather than writing the target path directly,
        // so `ffplay`'s `reload=1` (which re-reads on every frame) can
        // never observe a half-written file mid-update.
        let tmp = self.path.with_extension("tmp");
        std::fs::write(&tmp, text)?;
        std::fs::rename(&tmp, &self.path)
    }
}

/// Escapes a path for use inside an ffmpeg filtergraph option value, where
/// `:` separates options and `\` is the escape character itself -- a path
/// containing either (rare on Linux, but temp dirs aren't guaranteed not
/// to) would otherwise desync the filter's own option parsing.
fn escape_filter_value(s: &str) -> String {
    s.replace('\\', "\\\\").replace(':', "\\:")
}

pub struct VideoTx {
    critical: mpsc::UnboundedSender<Vec<u8>>,
    latest_delta: watch::Sender<Option<Vec<u8>>>,
}

impl VideoTx {
    /// `is_critical` (from `roboprotocol_core::video::nal_is_critical`
    /// on the NAL bytes *without* the Annex-B start code `bytes` carries)
    /// decides whether this NAL is always delivered or may be superseded
    /// by a later one. Errors are ignored -- a dropped receiver just
    /// means playback ended, not a session-safety event.
    pub fn send(&self, bytes: Vec<u8>, is_critical: bool) {
        if is_critical {
            let _ = self.critical.send(bytes);
        } else {
            let _ = self.latest_delta.send(Some(bytes));
        }
    }
}

pub fn spawn_playback(ffplay_bin: &str, overlay: bool) -> anyhow::Result<(Child, VideoTx, Option<OverlayHandle>)> {
    // `ffplay`'s stderr must never be `Stdio::inherit()`'d here: this
    // process's own stderr is the same terminal ratatui owns the
    // alternate screen of (same reasoning as `main.rs`'s `init_tracing`
    // doc comment) -- inheriting it lets ffplay's own output tear the
    // frame. `-loglevel warning` alone doesn't fully prevent this: the
    // startup banner (version/build-config/library-versions block) is
    // gated by the separate `-hide_banner` flag and prints regardless of
    // `-loglevel`, so both are needed. Any actual warning/error still
    // gets captured, just to a file instead of the live display.
    let stderr_log = OpenOptions::new().create(true).append(true).open("ffplay.log").context("opening ffplay.log")?;

    let overlay_handle = if overlay {
        let path = std::env::temp_dir().join(format!("roboprotocol-overlay-{}.txt", std::process::id()));
        // Written once up front so the box isn't blank/stale before the
        // first camera-control nudge -- `quic_client.rs` overwrites this
        // as soon as it has real values.
        std::fs::write(&path, "").context("writing initial overlay file")?;
        Some(OverlayHandle { path })
    } else {
        None
    };

    let mut args = vec![
        "-hide_banner".to_string(),
        "-flags".to_string(), "low_delay".to_string(),
        "-fflags".to_string(), "nobuffer".to_string(),
        "-framedrop".to_string(),
        "-f".to_string(), "h264".to_string(),
        "-i".to_string(), "-".to_string(),
        "-loglevel".to_string(), "warning".to_string(),
    ];
    if let Some(handle) = &overlay_handle {
        args.push("-vf".to_string());
        args.push(format!(
            "drawtext=textfile={}:reload=1:font=monospace:fontsize=18:fontcolor=white:box=1:boxcolor=black@0.5:boxborderw=6:x=10:y=10",
            escape_filter_value(&handle.path.display().to_string())
        ));
    }

    let mut child = Command::new(ffplay_bin)
        // `-framedrop`: decode falling behind render? drop, don't queue --
        // the same "latest wins" principle this whole pipeline already
        // applies upstream, extended to ffplay's own decode/render step.
        //
        // Deliberately NOT `-avioflags direct` or a cut-down `-probesize`/
        // `-analyzeduration`: both were tried here and broke playback
        // outright on real hardware (ffplay's stderr filled with permanent
        // "non-existing PPS 0 referenced", never recovering even across
        // later IDRs) -- `-avioflags direct` alone was enough to trigger
        // it, independent of probe size. ffmpeg's raw-Annex-B demuxer
        // apparently needs its normal buffered reads to reliably find
        // NAL start codes; forcing minimal-buffering reads on the stdin
        // pipe corrupts that, and it doesn't self-heal once it happens.
        .args(&args)
        .stdin(Stdio::piped())
        .stdout(Stdio::null())
        .stderr(Stdio::from(stderr_log))
        .kill_on_drop(true)
        .spawn()?;

    let stdin = child.stdin.take().expect("piped stdin");
    let (critical_tx, critical_rx) = mpsc::unbounded_channel();
    let (delta_tx, delta_rx) = watch::channel(None);
    tokio::spawn(feed_playback(stdin, critical_rx, delta_rx));
    Ok((child, VideoTx { critical: critical_tx, latest_delta: delta_tx }, overlay_handle))
}

async fn feed_playback(mut stdin: ChildStdin, mut critical_rx: mpsc::UnboundedReceiver<Vec<u8>>, mut delta_rx: watch::Receiver<Option<Vec<u8>>>) {
    loop {
        let chunk = tokio::select! {
            biased;
            Some(chunk) = critical_rx.recv() => chunk,
            changed = delta_rx.changed() => {
                if changed.is_err() {
                    return; // sender dropped -- session ending
                }
                let Some(chunk) = delta_rx.borrow_and_update().clone() else { continue };
                chunk
            }
        };
        if stdin.write_all(&chunk).await.is_err() {
            // ffplay exited/closed its stdin -- nothing more to do here;
            // a lost video feed is a degraded-UX event, not a safety one.
            return;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn escapes_colons_and_backslashes_for_the_filtergraph() {
        assert_eq!(escape_filter_value("plain"), "plain");
        assert_eq!(escape_filter_value("has:colon"), "has\\:colon");
        assert_eq!(escape_filter_value(r"has\backslash"), r"has\\backslash");
        // Windows-style paths would carry both, in the order a real path
        // would present them -- not this project's target platform, but
        // exercises both replacements interacting rather than in isolation.
        assert_eq!(escape_filter_value(r"C:\Users\a"), r"C\:\\Users\\a");
    }

    #[test]
    fn overlay_write_is_readable_at_the_target_path_not_left_at_the_tmp_name() {
        let dir = std::env::temp_dir().join(format!("roboprotocol-overlay-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("overlay.txt");
        let handle = OverlayHandle { path: path.clone() };

        handle.write("brightness 0.5").unwrap();
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "brightness 0.5");
        assert!(!path.with_extension("tmp").exists(), "write-then-rename must not leave the .tmp file behind");

        // A second write must fully replace the first, not append/corrupt it.
        handle.write("brightness -0.5").unwrap();
        assert_eq!(std::fs::read_to_string(&path).unwrap(), "brightness -0.5");

        let _ = std::fs::remove_dir_all(&dir);
    }
}
