mod action_trigger_handler;
mod bridge;
mod camera_control_handler;
mod channel_b;
mod hello_handler;
mod quic_server;
mod safety_task;
mod session_handler;
mod video;
mod xgo_profile;

use std::net::SocketAddr;
use std::path::PathBuf;
use std::time::Duration;

use anyhow::{Context, Result};
use roboprotocol_core::safety::TaskClass;

use crate::bridge::BridgeConfig;
use crate::quic_server::ServerArgs;
use crate::video::capture::CaptureConfig;

struct Cli {
    listen: SocketAddr,
    cert: String,
    key: String,
    ca: String,
    robot_id: String,
    task_class: TaskClass,
    tick_hz: u32,
    bridge_script: PathBuf,
    stub_bridge: bool,
    serial_port: String,
    watchdog_ms: f64,
    enable_camera: bool,
    camera_bin: String,
    record_dir: Option<PathBuf>,
    record: Vec<roboprotocol_recording::Category>,
    record_max_segment_mb: u64,
    record_max_segment_secs: u64,
    record_budget_mb: u64,
    record_video_budget_mb: u64,
    record_flush_secs: u64,
}

/// Maps a `--record` list entry to a category. `robot-edge` has no
/// keyboard, so `key-press` (operator-console-only) is a hard error here
/// rather than a silent no-op.
fn parse_record_category(name: &str) -> Result<roboprotocol_recording::Category> {
    use roboprotocol_recording::Category;
    match name {
        "video" => Ok(Category::VideoA),
        "command" => Ok(Category::ChannelBCommand),
        "telemetry" => Ok(Category::ChannelBTelemetry),
        "haptic" => Ok(Category::ChannelBHaptic),
        "action" => Ok(Category::ActionTriggerC),
        "key-press" => anyhow::bail!("--record key-press: robot-edge has no keyboard to log"),
        other => anyhow::bail!("unknown --record category {other} (expected video/command/telemetry/haptic/action)"),
    }
}

impl Cli {
    fn parse() -> Result<Self> {
        let mut listen: SocketAddr = "0.0.0.0:4433".parse().unwrap();
        let mut cert = "certs/robot/robot.crt".to_string();
        let mut key = "certs/robot/robot.key".to_string();
        let mut ca = "certs/dev-ca/ca.crt".to_string();
        let mut robot_id = "xgo_lite_v2".to_string();
        let mut task_class = TaskClass::D;
        let mut tick_hz = 50u32;
        let mut bridge_script = PathBuf::from("xgo_bridge/xgo_bridge.py");
        let mut stub_bridge = false;
        let mut serial_port = "/dev/ttyAMA0".to_string();
        // Matches xgo_bridge.py's default -- see that file's module
        // docstring for why 150ms fired spuriously on real hardware.
        let mut watchdog_ms = 400.0f64;
        let mut enable_camera = false;
        let mut camera_bin = "libcamera-vid".to_string();
        let mut record_dir = None;
        let mut record = Vec::new();
        let mut record_max_segment_mb = 8u64;
        let mut record_max_segment_secs = 60u64;
        let mut record_budget_mb = 64u64;
        let mut record_video_budget_mb = 16u64;
        let mut record_flush_secs = 2u64;

        let mut it = std::env::args().skip(1);
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--listen" => listen = it.next().context("--listen needs a value")?.parse()?,
                "--cert" => cert = it.next().context("--cert needs a value")?,
                "--key" => key = it.next().context("--key needs a value")?,
                "--ca" => ca = it.next().context("--ca needs a value")?,
                "--robot-id" => robot_id = it.next().context("--robot-id needs a value")?,
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
                "--bridge-script" => bridge_script = PathBuf::from(it.next().context("--bridge-script needs a value")?),
                "--stub-bridge" => stub_bridge = true,
                "--serial-port" => serial_port = it.next().context("--serial-port needs a value")?,
                "--watchdog-ms" => watchdog_ms = it.next().context("--watchdog-ms needs a value")?.parse()?,
                "--camera" => enable_camera = true,
                "--camera-bin" => camera_bin = it.next().context("--camera-bin needs a value")?,
                "--record-dir" => record_dir = Some(PathBuf::from(it.next().context("--record-dir needs a value")?)),
                "--record" => {
                    record = it
                        .next()
                        .context("--record needs a value")?
                        .split(',')
                        .map(parse_record_category)
                        .collect::<Result<Vec<_>>>()?
                }
                "--record-max-segment-mb" => record_max_segment_mb = it.next().context("--record-max-segment-mb needs a value")?.parse()?,
                "--record-max-segment-secs" => record_max_segment_secs = it.next().context("--record-max-segment-secs needs a value")?.parse()?,
                "--record-budget-mb" => record_budget_mb = it.next().context("--record-budget-mb needs a value")?.parse()?,
                "--record-video-budget-mb" => record_video_budget_mb = it.next().context("--record-video-budget-mb needs a value")?.parse()?,
                "--record-flush-secs" => record_flush_secs = it.next().context("--record-flush-secs needs a value")?.parse()?,
                "-h" | "--help" => {
                    println!(
                        "Usage: robot-edge [--listen ADDR] [--cert PATH] [--key PATH] [--ca PATH]\n  \
                         [--robot-id ID] [--task-class B|C|D|E] [--tick-hz N]\n  \
                         [--bridge-script PATH] [--stub-bridge] [--serial-port PATH] [--watchdog-ms N]\n  \
                         [--camera] [--camera-bin PATH]\n  \
                         [--record-dir PATH] [--record video,command,telemetry,haptic,action]\n  \
                         [--record-max-segment-mb N] [--record-max-segment-secs N]\n  \
                         [--record-budget-mb N] [--record-video-budget-mb N] [--record-flush-secs N]"
                    );
                    std::process::exit(0);
                }
                other => anyhow::bail!("unrecognized argument: {other} (see --help)"),
            }
        }

        if !record.is_empty() && record_dir.is_none() {
            anyhow::bail!("--record given without --record-dir");
        }

        Ok(Self {
            listen,
            cert,
            key,
            ca,
            robot_id,
            task_class,
            tick_hz,
            bridge_script,
            stub_bridge,
            serial_port,
            watchdog_ms,
            enable_camera,
            camera_bin,
            record_dir,
            record,
            record_max_segment_mb,
            record_max_segment_secs,
            record_budget_mb,
            record_video_budget_mb,
            record_flush_secs,
        })
    }

    /// Builds a `Recorder` config from the parsed flags -- an empty
    /// `categories` list (when `--record-dir` was never given) is a
    /// legitimate, fully inert config: `Recorder::start` spawns no
    /// threads at all for it, so recording is genuinely "off," not just
    /// configured-but-idle.
    fn recorder_config(&self) -> roboprotocol_recording::RecorderConfig {
        use roboprotocol_recording::{Category, CategoryConfig};
        let Some(record_dir) = self.record_dir.clone() else {
            return roboprotocol_recording::RecorderConfig::default();
        };
        let categories = self
            .record
            .iter()
            .map(|&category| CategoryConfig {
                category,
                ring_capacity: 256,
                max_segment_bytes: self.record_max_segment_mb * 1_000_000,
                max_segment_duration: Duration::from_secs(self.record_max_segment_secs),
                max_total_bytes: (if category == Category::VideoA { self.record_video_budget_mb } else { self.record_budget_mb }) * 1_000_000,
                flush_interval: Duration::from_secs(self.record_flush_secs),
                start_active: true,
            })
            .collect();
        roboprotocol_recording::RecorderConfig { record_dir, categories }
    }
}

#[tokio::main(flavor = "current_thread")]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    let cli = Cli::parse()?;

    let mut bridge_extra_args = Vec::new();
    if cli.stub_bridge {
        bridge_extra_args.push("--stub".to_string());
    } else {
        bridge_extra_args.extend([
            "--port".to_string(),
            cli.serial_port.clone(),
            "--watchdog-ms".to_string(),
            cli.watchdog_ms.to_string(),
        ]);
    }

    // Computed before the struct literal below starts moving individual
    // `String`/`PathBuf` fields out of `cli` -- `recorder_config` borrows
    // `cli` as a whole, which a partially-moved-from `cli` can't satisfy.
    let recording = cli.recorder_config();

    let args = ServerArgs {
        listen: cli.listen,
        cert_path: cli.cert,
        key_path: cli.key,
        ca_path: cli.ca,
        task_class: cli.task_class,
        robot_id: cli.robot_id,
        tick_hz: cli.tick_hz,
        bridge: BridgeConfig {
            python_bin: "python3".to_string(),
            script_path: cli.bridge_script,
            extra_args: bridge_extra_args,
            max_restart_attempts: 3,
            restart_window: Duration::from_secs(60),
        },
        camera_config: cli.enable_camera.then(|| CaptureConfig {
            rpicam_vid_bin: cli.camera_bin,
            ..CaptureConfig::default()
        }),
        recording,
    };

    let profile = xgo_profile::xgo_lite_v2_profile();
    let cameras = if cli.enable_camera { vec![xgo_profile::xgo_lite_v2_camera()] } else { Vec::new() };

    quic_server::run(args, profile, cameras).await
}
