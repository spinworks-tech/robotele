mod action_trigger_handler;
mod camera_control_handler;
mod channel_b;
mod hello_handler;
mod input;
mod quic_client;
mod session_cache;
mod session_handler;
mod ui;
mod video;

use std::fs::{File, OpenOptions};
use std::io;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use anyhow::{Context, Result};
use roboprotocol_core::safety::TaskClass;

use crate::quic_client::{ClientArgs, VideoBackend};

struct Cli {
    connect: SocketAddr,
    bind: SocketAddr,
    server_name: String,
    cert: String,
    key: String,
    ca: String,
    task_class: TaskClass,
    tick_hz: u32,
    ffplay_bin: String,
    enable_video: bool,
    video_backend: VideoBackend,
    video_overlay: bool,
    headless: bool,
    demo_action: Option<u8>,
    move_stale_ms: u64,
    record_dir: Option<PathBuf>,
    record_extra: Vec<roboprotocol_recording::Category>,
    record_max_segment_mb: u64,
    record_max_segment_secs: u64,
    record_budget_mb: u64,
    record_video_budget_mb: u64,
    record_flush_secs: u64,
}

/// `--record-extra`'s allowed categories -- `video`/`command`/`telemetry`/
/// `key-press` are already covered by the `'r'` runtime toggle (see
/// `DEFAULT_RECORD_CATEGORIES` in `quic_client.rs`), so only the two
/// categories that toggle doesn't reach are offered here.
fn parse_record_extra_category(name: &str) -> Result<roboprotocol_recording::Category> {
    use roboprotocol_recording::Category;
    match name {
        "haptic" => Ok(Category::ChannelBHaptic),
        "action" => Ok(Category::ActionTriggerC),
        other => anyhow::bail!("unknown --record-extra category {other} (expected haptic/action -- video/command/telemetry/key-press are already covered by 'r')"),
    }
}

impl Cli {
    fn parse() -> Result<Self> {
        let mut connect: SocketAddr = "127.0.0.1:4433".parse().unwrap();
        let mut bind: SocketAddr = "0.0.0.0:0".parse().unwrap();
        let mut server_name = "robot-edge".to_string();
        let mut cert = "certs/operator/operator.crt".to_string();
        let mut key = "certs/operator/operator.key".to_string();
        let mut ca = "certs/dev-ca/ca.crt".to_string();
        let mut task_class = TaskClass::D;
        let mut tick_hz = 50u32;
        let mut ffplay_bin = "ffplay".to_string();
        let mut enable_video = false;
        let mut video_backend = VideoBackend::Ffplay;
        let mut video_overlay = false;
        let mut headless = false;
        let mut demo_action = None;
        let mut move_stale_ms = 400u64;
        let mut record_dir = None;
        let mut record_extra = Vec::new();
        // Larger than robot-edge's defaults (NFR-6.2): this workstation
        // typically has far more storage/CPU headroom than an SD/eMMC-
        // backed edge controller.
        let mut record_max_segment_mb = 64u64;
        let mut record_max_segment_secs = 300u64;
        let mut record_budget_mb = 2048u64;
        let mut record_video_budget_mb = 512u64;
        let mut record_flush_secs = 5u64;

        let mut it = std::env::args().skip(1);
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--connect" => connect = it.next().context("--connect needs a value")?.parse()?,
                "--bind" => bind = it.next().context("--bind needs a value")?.parse()?,
                "--server-name" => server_name = it.next().context("--server-name needs a value")?,
                "--cert" => cert = it.next().context("--cert needs a value")?,
                "--key" => key = it.next().context("--key needs a value")?,
                "--ca" => ca = it.next().context("--ca needs a value")?,
                "--task-class" => {
                    task_class = match it.next().context("--task-class needs a value")?.as_str() {
                        "B" => TaskClass::B,
                        "C" => TaskClass::C,
                        "D" => TaskClass::D,
                        "E" => TaskClass::E,
                        other => anyhow::bail!("unknown task class {other}, expected B/C/D/E"),
                    }
                }
                "--tick-hz" => tick_hz = it.next().context("--tick-hz needs a value")?.parse()?,
                "--ffplay-bin" => ffplay_bin = it.next().context("--ffplay-bin needs a value")?,
                "--video" => enable_video = true,
                "--video-backend" => {
                    video_backend = match it.next().context("--video-backend needs a value")?.as_str() {
                        "ffplay" => VideoBackend::Ffplay,
                        "native" => VideoBackend::Native,
                        other => anyhow::bail!("unknown --video-backend {other}, expected ffplay/native"),
                    }
                }
                "--video-overlay" => video_overlay = true,
                "--headless" => headless = true,
                "--demo-action" => demo_action = Some(it.next().context("--demo-action needs a value")?.parse()?),
                "--move-stale-ms" => move_stale_ms = it.next().context("--move-stale-ms needs a value")?.parse()?,
                "--record-dir" => record_dir = Some(PathBuf::from(it.next().context("--record-dir needs a value")?)),
                "--record-extra" => {
                    record_extra = it
                        .next()
                        .context("--record-extra needs a value")?
                        .split(',')
                        .map(parse_record_extra_category)
                        .collect::<Result<Vec<_>>>()?
                }
                "--record-max-segment-mb" => record_max_segment_mb = it.next().context("--record-max-segment-mb needs a value")?.parse()?,
                "--record-max-segment-secs" => record_max_segment_secs = it.next().context("--record-max-segment-secs needs a value")?.parse()?,
                "--record-budget-mb" => record_budget_mb = it.next().context("--record-budget-mb needs a value")?.parse()?,
                "--record-video-budget-mb" => record_video_budget_mb = it.next().context("--record-video-budget-mb needs a value")?.parse()?,
                "--record-flush-secs" => record_flush_secs = it.next().context("--record-flush-secs needs a value")?.parse()?,
                "-h" | "--help" => {
                    println!(
                        "Usage: operator-console [--connect ADDR] [--bind ADDR] [--server-name NAME]\n  \
                         [--cert PATH] [--key PATH] [--ca PATH] [--task-class B|C|D|E] [--tick-hz N]\n  \
                         [--video] [--video-backend ffplay|native] [--video-overlay] [--ffplay-bin PATH]\n  \
                         [--move-stale-ms N]\n  \
                         [--record-dir PATH] [--record-extra haptic,action]\n  \
                         [--record-max-segment-mb N] [--record-max-segment-secs N]\n  \
                         [--record-budget-mb N] [--record-video-budget-mb N] [--record-flush-secs N]\n\n\
                         Keys: w/a/s/d move, left/right turn, space stop, i/j/k/l arm, u/o claw,\n  \
                         1 stand, 2 sit, e E-Stop, c clear, r start/stop recording, q quit.\n\
                         Movement keys set a held velocity (XGO's own move_x/move_y/turn are continuous\n  \
                         setpoints, not one-shot moves) -- if no movement key event refreshes it within\n  \
                         --move-stale-ms (default 400, relies on your terminal's normal key-repeat-while-held\n  \
                         behavior), it's automatically zeroed. Lower this if your terminal's key repeat is\n  \
                         faster than 400ms and you want a snappier stop; raise it if repeats are slower than\n  \
                         that and movement stutters while held.\n\
                         Arm/claw keys (i/j/k/l/u/o) set a held *position*, not a velocity -- xgolib's\n  \
                         arm(x, z)/claw(pos) hold wherever last commanded, so these never auto-stop.\n\
                         Recording (FR-9) needs --record-dir; 'r' then toggles video/command/telemetry/\n  \
                         key-press on and off at runtime. --record-extra adds haptic/action from launch,\n  \
                         active the whole session (they have no natural on/off point of their own).\n\
                         --video-overlay (needs --video) burns brightness/contrast/EV/shutter into the\n  \
                         ffplay window itself via its drawtext filter -- requires an ffmpeg/ffplay build\n  \
                         with libfreetype+libfontconfig; if yours lacks it, ffplay will fail to start\n  \
                         with this flag on, so leave it off rather than guess.\n\
                         --video-backend native decodes in-process (openh264 + minifb) instead of\n  \
                         spawning ffplay -- experimental, and incompatible with --video-overlay (which\n  \
                         is implemented as an ffplay-specific filter)."
                    );
                    std::process::exit(0);
                }
                other => anyhow::bail!("unrecognized argument: {other} (see --help)"),
            }
        }

        if !record_extra.is_empty() && record_dir.is_none() {
            anyhow::bail!("--record-extra given without --record-dir");
        }
        if video_overlay && !enable_video {
            anyhow::bail!("--video-overlay given without --video");
        }
        if video_overlay && video_backend == VideoBackend::Native {
            anyhow::bail!("--video-overlay is only implemented for --video-backend ffplay (its drawtext filter is ffplay-specific)");
        }

        Ok(Self {
            connect,
            bind,
            server_name,
            cert,
            key,
            ca,
            task_class,
            tick_hz,
            ffplay_bin,
            enable_video,
            video_backend,
            video_overlay,
            headless,
            demo_action,
            move_stale_ms,
            record_dir,
            record_extra,
            record_max_segment_mb,
            record_max_segment_secs,
            record_budget_mb,
            record_video_budget_mb,
            record_flush_secs,
        })
    }

    /// Builds a `Recorder` config: the four toggle-controlled default
    /// categories (`start_active: false`, flipped at runtime by `'r'` --
    /// see `quic_client.rs`'s `DEFAULT_RECORD_CATEGORIES`) plus whatever
    /// `--record-extra` named (`start_active: true` from launch). An
    /// empty `categories` list when `--record-dir` was never given is a
    /// legitimate, fully inert config -- `Recorder::start` spawns no
    /// threads for it, so `'r'` is a harmless no-op rather than needing
    /// special-cased handling elsewhere.
    fn recorder_config(&self) -> roboprotocol_recording::RecorderConfig {
        use roboprotocol_recording::{Category, CategoryConfig};
        let Some(record_dir) = self.record_dir.clone() else {
            return roboprotocol_recording::RecorderConfig::default();
        };
        let default_categories = [Category::VideoA, Category::ChannelBCommand, Category::ChannelBTelemetry, Category::KeyPress];
        let category_config = |category: Category, start_active: bool| CategoryConfig {
            category,
            ring_capacity: 256,
            max_segment_bytes: self.record_max_segment_mb * 1_000_000,
            max_segment_duration: Duration::from_secs(self.record_max_segment_secs),
            max_total_bytes: (if category == Category::VideoA { self.record_video_budget_mb } else { self.record_budget_mb }) * 1_000_000,
            flush_interval: Duration::from_secs(self.record_flush_secs),
            start_active,
        };
        let categories = default_categories
            .into_iter()
            .map(|c| category_config(c, false))
            .chain(self.record_extra.iter().map(|&c| category_config(c, true)))
            .collect();
        roboprotocol_recording::RecorderConfig { record_dir, categories }
    }
}

/// Writes to a shared `File` -- the `tracing_subscriber::fmt::MakeWriter`
/// closure clones this cheaply per log line.
#[derive(Clone)]
struct FileWriter(Arc<Mutex<File>>);

impl io::Write for FileWriter {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.0.lock().unwrap().write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.0.lock().unwrap().flush()
    }
}

/// In interactive mode the HUD owns the alternate screen, so `tracing`
/// output can't go to stdout -- it would tear the frame. `--headless`
/// (e.g. `scripts/smoke_test.sh`) keeps stdout logging, which the smoke
/// test greps for phase-transition lines.
fn init_tracing(headless: bool) -> Result<()> {
    if headless {
        tracing_subscriber::fmt::init();
        return Ok(());
    }
    let file = OpenOptions::new()
        .create(true)
        .append(true)
        .open("operator-console.log")
        .context("opening operator-console.log")?;
    let file = Arc::new(Mutex::new(file));
    tracing_subscriber::fmt().with_writer(move || FileWriter(file.clone())).init();
    Ok(())
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    let cli = Cli::parse()?;
    init_tracing(cli.headless)?;

    // Computed before the struct literal below starts moving individual
    // fields out of `cli` -- `recorder_config` borrows `cli` as a whole,
    // which a partially-moved-from `cli` can't satisfy.
    let recording = cli.recorder_config();

    let args = ClientArgs {
        connect: cli.connect,
        bind: cli.bind,
        server_name: cli.server_name,
        cert_path: cli.cert,
        key_path: cli.key,
        ca_path: cli.ca,
        task_class: cli.task_class,
        tick_hz: cli.tick_hz,
        ffplay_bin: cli.ffplay_bin,
        enable_video: cli.enable_video,
        video_backend: cli.video_backend,
        video_overlay: cli.video_overlay,
        headless: cli.headless,
        demo_action: cli.demo_action,
        move_stale_ms: cli.move_stale_ms,
        recording,
    };

    // Interactive mode enters the alternate screen inside `ui::Console`
    // (constructed in `quic_client::run`), which restores it on drop --
    // covering early `?` returns -- and `ratatui::init()` additionally
    // installs a panic hook that restores it on panic.
    quic_client::run(args).await
}
