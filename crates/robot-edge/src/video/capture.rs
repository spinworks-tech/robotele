//! Spawns and supervises the libcamera video CLI tool, streaming raw
//! H.264 Annex-B bytes out over a channel for `channel_a.rs` to chunk
//! into QUIC datagrams.
//!
//! Binary name: `libcamera-vid` on Raspberry Pi OS Bullseye (confirmed
//! against the real XGO-Lite V2 CM4, which runs Bullseye); Raspberry Pi
//! OS Bookworm renamed the libcamera-apps CLI tools to `rpicam-vid`.
//! Overridable via `--camera-bin` for that case.
//!
//! Unlike the XGO bridge, a lost camera feed is a degraded-UX event, not
//! a safety event -- it does not force E-Stop, just logs and retries.

use std::process::Stdio;
use std::time::Duration;

use tokio::io::AsyncReadExt;
use tokio::process::Command;
use tokio::sync::{mpsc, watch};

/// Image-quality controls, live-adjustable mid-session via
/// `roboprotocol_core::camera_control::CameraControl` (Channel C) -- kept
/// separate from `CaptureConfig`'s other fields (resolution/framerate/etc,
/// fixed for the process's lifetime) since these are the ones a running
/// session can push updates to. Defaults match `libcamera-vid`'s own
/// defaults, so an unmodified session behaves exactly as before this
/// existed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraControls {
    pub brightness: f32,
    pub contrast: f32,
    pub ev: f32,
    /// Manual exposure time in microseconds; 0 = auto-exposure (omit the
    /// `--shutter` flag entirely rather than passing 0).
    pub shutter_us: u32,
}

impl Default for CameraControls {
    fn default() -> Self {
        Self { brightness: 0.0, contrast: 1.0, ev: 0.0, shutter_us: 0 }
    }
}

#[derive(Clone)]
pub struct CaptureConfig {
    pub rpicam_vid_bin: String,
    pub width: u16,
    pub height: u16,
    pub framerate: u8,
    /// I-frame interval in frames (`--intra`) -- some loss resilience even
    /// without FlexFEC/PIR, per the plan's documented v0 gap.
    pub intra: u32,
    pub max_restart_attempts: u32,
    pub restart_window: Duration,
    /// Initial image-quality controls -- live updates after this come
    /// through the `watch::Sender` returned alongside `spawn_capture`'s
    /// receiver, not by reconstructing this config.
    pub controls: CameraControls,
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self {
            rpicam_vid_bin: "libcamera-vid".to_string(),
            width: 640,
            height: 480,
            framerate: 30,
            intra: 30,
            max_restart_attempts: 5,
            restart_window: Duration::from_secs(60),
            controls: CameraControls::default(),
        }
    }
}

pub struct CaptureHandle {
    /// Raw bytes read from `rpicam-vid`'s stdout, in arrival order.
    /// NAL-unit boundary detection happens downstream in `channel_a.rs`.
    pub rx: mpsc::UnboundedReceiver<Vec<u8>>,
    /// Push a new `CameraControls` to apply live -- `capture_task` notices
    /// the change and restarts `rpicam-vid` with the new args immediately
    /// (not gated by the crash-backoff logic below, since this is a
    /// deliberate operator-requested change, not a failure).
    pub controls_tx: watch::Sender<CameraControls>,
}

pub fn spawn_capture(config: CaptureConfig) -> CaptureHandle {
    let (tx, rx) = mpsc::unbounded_channel();
    let (controls_tx, controls_rx) = watch::channel(config.controls);
    tokio::spawn(capture_task(config, controls_rx, tx));
    CaptureHandle { rx, controls_tx }
}

fn build_args(config: &CaptureConfig, controls: &CameraControls) -> Vec<String> {
    let mut args = vec![
        "--codec".to_string(), "h264".to_string(),
        "--width".to_string(), config.width.to_string(),
        "--height".to_string(), config.height.to_string(),
        "--framerate".to_string(), config.framerate.to_string(),
        "--intra".to_string(), config.intra.to_string(),
        "--inline".to_string(),
        "--flush".to_string(), // write each encoded frame out immediately
                   // instead of holding it in an internal output buffer --
                   // for a teleop preview every frame is latency-
                   // sensitive, there's nothing to gain by batching
        "--profile".to_string(), "baseline".to_string(), // no CABAC entropy
                   // coding -- costs some compression efficiency, but
                   // CABAC's extra encode/decode complexity is pure added
                   // latency for a live teleop feed, not a quality
                   // tradeoff worth making (unlike for a stored recording)
        "--bitrate".to_string(), "2000000".to_string(), // bound worst-case
                   // frame size (an unconstrained encoder can spike
                   // bitrate on a busy/complex scene) so one bad frame
                   // can't become an outsized chunk burst
        "--timeout".to_string(), "0".to_string(), // run until killed
        "--brightness".to_string(), controls.brightness.to_string(),
        "--contrast".to_string(), controls.contrast.to_string(),
        "--ev".to_string(), controls.ev.to_string(),
    ];
    // 0 means auto-exposure -- omit the flag rather than pass `--shutter
    // 0`, which libcamera-vid would take as a literal (nonsensical)
    // zero-length exposure rather than "let the AE algorithm decide".
    if controls.shutter_us != 0 {
        args.push("--shutter".to_string());
        args.push(controls.shutter_us.to_string());
    }
    args.push("-o".to_string());
    args.push("-".to_string());
    args
}

async fn capture_task(config: CaptureConfig, mut controls_rx: watch::Receiver<CameraControls>, tx: mpsc::UnboundedSender<Vec<u8>>) {
    let mut attempts: u32 = 0;
    let mut window_start = tokio::time::Instant::now();

    loop {
        let controls = *controls_rx.borrow_and_update();
        let child = Command::new(&config.rpicam_vid_bin)
            .args(build_args(&config, &controls))
            .stdout(Stdio::piped())
            .stderr(Stdio::null())
            .kill_on_drop(true)
            .spawn();

        let mut child = match child {
            Ok(c) => c,
            Err(e) => {
                tracing::warn!(error = %e, "failed to spawn rpicam-vid, will retry");
                if !backoff_or_give_up(&mut attempts, &mut window_start, &config).await {
                    tracing::error!("giving up on camera capture after repeated failures");
                    return;
                }
                continue;
            }
        };

        let mut stdout = child.stdout.take().expect("piped stdout");
        let mut buf = vec![0u8; 64 * 1024];
        // `true` once we broke out of the inner loop because the operator
        // pushed new controls (deliberate, restart immediately) rather
        // than because the process exited/errored (unexpected, back off).
        let mut controls_changed = false;
        loop {
            tokio::select! {
                res = stdout.read(&mut buf) => {
                    match res {
                        Ok(0) => break, // EOF -- process exiting
                        Ok(n) => {
                            if tx.send(buf[..n].to_vec()).is_err() {
                                // Receiver dropped: shut down capture entirely.
                                let _ = child.start_kill();
                                let _ = child.wait().await;
                                return;
                            }
                        }
                        Err(e) => {
                            tracing::warn!(error = %e, "rpicam-vid stdout read error");
                            break;
                        }
                    }
                }
                changed = controls_rx.changed() => {
                    if changed.is_err() {
                        // Sender dropped (Session gone) -- nothing more will
                        // ever arrive on this channel; keep reading video
                        // normally, there's no restart to do.
                        continue;
                    }
                    tracing::info!("camera controls changed, restarting rpicam-vid");
                    controls_changed = true;
                    break;
                }
            }
        }

        let _ = child.start_kill();
        let _ = child.wait().await;
        if controls_changed {
            // Deliberate operator-requested restart, not a failure --
            // don't consume a crash-backoff attempt for it, and don't
            // delay: the operator is waiting to see the new image.
            continue;
        }
        tracing::warn!("rpicam-vid exited, restarting camera capture");
        if !backoff_or_give_up(&mut attempts, &mut window_start, &config).await {
            tracing::error!("giving up on camera capture after repeated failures");
            return;
        }
    }
}

async fn backoff_or_give_up(attempts: &mut u32, window_start: &mut tokio::time::Instant, config: &CaptureConfig) -> bool {
    if window_start.elapsed() > config.restart_window {
        *attempts = 0;
        *window_start = tokio::time::Instant::now();
    }
    *attempts += 1;
    if *attempts > config.max_restart_attempts {
        return false;
    }
    tokio::time::sleep(Duration::from_millis(300 * 2u64.pow((*attempts).min(5)))).await;
    true
}
