//! Listens to robot-edge's Channel C Zenoh topics (the BabyROS sidecar, see
//! BABYROS.md) and shows the data flowing: decodes `robotele/<id>/telemetry` and
//! `robotele/<id>/command` (the operator's command plus the arbitrated control
//! source), and counts anything else published under `robotele/<id>/` (e.g. the
//! `autonomy_goal` topic) so you can see it without a flood of lines.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};

const DEFAULT_ENDPOINT: &str = "tcp/192.168.2.19:7447";
const SUMMARY_EVERY: Duration = Duration::from_secs(5);

/// What to print. `Status` is the robot's telemetry and the periodic summary;
/// `Commands` is the operator's commands (and any other non-telemetry topic such
/// as `autonomy_goal`, which is also something sent *to* the robot).
#[derive(Clone, Copy, PartialEq, Eq)]
enum Show {
    All,
    Status,
    Commands,
}

impl Show {
    fn telemetry(self) -> bool {
        self != Show::Commands
    }
    fn commands(self) -> bool {
        self != Show::Status
    }
    /// The 5 s summary line; in commands-only mode it's still printed when there
    /// are no peers, since silence would look like "no commands" instead of "not connected".
    fn summary(self, peers: usize) -> bool {
        self != Show::Commands || peers == 0
    }
}

struct Args {
    endpoints: Vec<String>,
    robot_id: String,
    key: Option<String>,
    all: bool,
    /// Only print a command when it differs from the last one printed.
    changes: bool,
    show: Show,
}

fn parse_args() -> Result<Args> {
    let mut args = Args { endpoints: Vec::new(), robot_id: "xgo_real".to_string(), key: None, all: false, changes: false, show: Show::All };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--connect" => args.endpoints.push(it.next().ok_or_else(|| anyhow!("--connect needs a value"))?),
            "--robot-id" => args.robot_id = it.next().ok_or_else(|| anyhow!("--robot-id needs a value"))?,
            "--key" => args.key = Some(it.next().ok_or_else(|| anyhow!("--key needs a value"))?),
            "--all" => args.all = true,
            "--changes" | "--diff" => args.changes = true,
            "--only" => {
                args.show = match it.next().ok_or_else(|| anyhow!("--only needs a value (commands|status)"))?.as_str() {
                    "commands" | "command" | "cmd" => Show::Commands,
                    "status" | "telemetry" => Show::Status,
                    other => return Err(anyhow!("--only expects `commands` or `status`, got `{other}`")),
                }
            }
            "-h" | "--help" => {
                println!(
                    "Usage: zenoh-monitor [--connect tcp/HOST:PORT]... [--robot-id ID] [--key KEYEXPR] [--only commands|status] [--changes] [--all]\n\n  \
                     --connect   Zenoh endpoint of robot-edge (repeatable). Default: {DEFAULT_ENDPOINT}\n  \
                     --robot-id  robot-edge's --robot-id. Default: xgo_real\n  \
                     --key       Key expression to listen on. Default: robotele/<robot-id>/**\n  \
                     --only      commands: just the operator's commands (and autonomy goals)\n              \
                     status: just the robot's telemetry + the 5 s summary. Default: both\n  \
                     --changes   Print a command only when it differs from the last one printed\n              \
                     (robot-edge re-sends an unchanged command every 250 ms)\n  \
                     --all       Print every sample (default: telemetry lines + a rate summary\n              \
                     for other topics such as autonomy_goal, which can arrive at ~10 Hz)\n\n\
                     Ctrl+C to quit."
                );
                std::process::exit(0);
            }
            other => return Err(anyhow!("unrecognized argument: {other} (see --help)")),
        }
    }
    if args.endpoints.is_empty() {
        args.endpoints.push(DEFAULT_ENDPOINT.to_string());
    }
    Ok(args)
}

/// robot-edge's compact Channel B telemetry encoding (`TelemetryData::pack`),
/// big-endian: battery u8, roll/pitch/yaw i16 (0.01 deg), then per-joint i16.
fn decode_telemetry(b: &[u8]) -> Option<String> {
    const HEADER: usize = 7;
    if b.len() < HEADER || (b.len() - HEADER) % 2 != 0 {
        return None;
    }
    let i16_at = |i: usize| i16::from_be_bytes([b[i], b[i + 1]]) as f32 / 100.0;
    let joints: Vec<String> = b[HEADER..].chunks_exact(2).map(|c| format!("{:.1}", i16::from_be_bytes([c[0], c[1]]) as f32 / 100.0)).collect();
    Some(format!(
        "battery={}%  roll={:.1} pitch={:.1} yaw={:.1}  joints[{}]=[{}]",
        b[0],
        i16_at(1),
        i16_at(3),
        i16_at(5),
        joints.len(),
        joints.join(" ")
    ))
}

const SOURCE_NAMES: [&str; 5] = ["EStop", "SafeParking", "ActiveImpedanceHold", "FullTeleop", "SemiAutonomous"];

/// `robotele/<id>/command` payload (30 bytes, big-endian; see
/// `zenoh_bridge::encode_command` in robot-edge): control_source u8, then
/// vx/vy/turn/roll/pitch/yaw as f32, arm_x/arm_z as i16, claw u8.
struct Command {
    source: u8,
    text: String,
    idle: bool,
}

fn decode_command(b: &[u8]) -> Option<Command> {
    if b.len() != 30 {
        return None;
    }
    let f = |i: usize| f32::from_be_bytes([b[i], b[i + 1], b[i + 2], b[i + 3]]);
    let (vx, vy, turn) = (f(1), f(5), f(9));
    let arm_x = i16::from_be_bytes([b[25], b[26]]);
    let arm_z = i16::from_be_bytes([b[27], b[28]]);
    let source = b[0];
    let name = SOURCE_NAMES.get(source as usize).copied().unwrap_or("?");
    Some(Command {
        source,
        idle: vx == 0.0 && vy == 0.0 && turn == 0.0,
        text: format!(
            "[{name}]  vx={vx:+.2} vy={vy:+.2} turn={turn:+.2}  rpy=({:.1},{:.1},{:.1})  arm=({arm_x},{arm_z}) claw={}",
            f(13),
            f(17),
            f(21),
            b[29]
        ),
    })
}

/// Commands can change at operator-input rate; print at most this often unless
/// the control source or the moving/idle state flips (those always print).
const COMMAND_PRINT_EVERY: Duration = Duration::from_millis(200);

/// `--changes`: is this payload identical to the last one we printed?
fn is_repeat(changes: bool, last_printed: Option<&[u8]>, payload: &[u8]) -> bool {
    changes && last_printed == Some(payload)
}

#[derive(Default)]
struct TopicStats {
    total: u64,
    window: u64,
    last_bytes: usize,
}

#[tokio::main(flavor = "multi_thread", worker_threads = 2)]
async fn main() -> Result<()> {
    let args = parse_args()?;
    let key = args.key.clone().unwrap_or_else(|| format!("robotele/{}/**", args.robot_id));

    let mut config = zenoh::Config::default();
    let endpoints = args.endpoints.iter().map(|e| format!("\"{e}\"")).collect::<Vec<_>>().join(",");
    config.insert_json5("connect/endpoints", &format!("[{endpoints}]")).map_err(|e| anyhow!("bad --connect: {e}"))?;

    println!("zenoh-monitor: connecting to {}  listening on `{key}`  (Ctrl+C to quit)", args.endpoints.join(", "));
    let session = zenoh::open(config).await.map_err(|e| anyhow!("zenoh open failed: {e}"))?;
    println!("zenoh-monitor: our ZID {}", session.zid());
    let subscriber = session.declare_subscriber(key.clone()).await.map_err(|e| anyhow!("subscribe failed: {e}"))?;

    let started = Instant::now();
    let mut stats: BTreeMap<String, TopicStats> = BTreeMap::new();
    let mut summary = tokio::time::interval(SUMMARY_EVERY);
    summary.tick().await; // first tick fires immediately; skip it
    let mut last_summary = Instant::now();
    let mut last_cmd: Option<(u8, bool, Instant)> = None;
    let mut last_printed_cmd: Option<Vec<u8>> = None;

    loop {
        tokio::select! {
            _ = tokio::signal::ctrl_c() => break,
            _ = summary.tick() => {
                let secs = last_summary.elapsed().as_secs_f32().max(0.001);
                last_summary = Instant::now();
                let peers = session.info().peers_zid().await.count();
                let mut parts: Vec<String> = stats
                    .iter_mut()
                    .map(|(k, s)| {
                        let rate = s.window as f32 / secs;
                        s.window = 0;
                        format!("{} {:.1}/s ({} total)", k.rsplit('/').next().unwrap_or(k), rate, s.total)
                    })
                    .collect();
                if parts.is_empty() {
                    parts.push("no samples yet".to_string());
                }
                let note = if peers == 0 { "  -- NO PEERS: check the robot IP/port and that robot-edge is the BabyROS build" } else { "" };
                if args.show.summary(peers) {
                    println!("[t+{:>7.1}s] peers={peers}  {}{note}", started.elapsed().as_secs_f32(), parts.join(" | "));
                }
            }
            sample = subscriber.recv_async() => {
                let sample = match sample {
                    Ok(s) => s,
                    Err(e) => { eprintln!("subscription closed: {e}"); break; }
                };
                let topic = sample.key_expr().as_str().to_string();
                let payload = sample.payload().to_bytes();
                let entry = stats.entry(topic.clone()).or_default();
                let first = entry.total == 0;
                entry.total += 1;
                entry.window += 1;
                entry.last_bytes = payload.len();

                let t = started.elapsed().as_secs_f32();
                if topic.ends_with("/telemetry") {
                    if args.show.telemetry() {
                        match decode_telemetry(&payload) {
                            Some(text) => println!("[t+{t:>7.1}s] {topic}  {} B  {text}", payload.len()),
                            None => println!("[t+{t:>7.1}s] {topic}  {} B  (unrecognized telemetry layout)", payload.len()),
                        }
                    }
                } else if !args.show.commands() {
                    // status-only: commands and other topics are still counted above, just not printed
                } else if topic.ends_with("/command") {
                    match decode_command(&payload) {
                        Some(cmd) => {
                            let flipped = last_cmd.map_or(true, |(src, idle, _)| src != cmd.source || idle != cmd.idle);
                            let due = last_cmd.map_or(true, |(_, _, at)| at.elapsed() >= COMMAND_PRINT_EVERY);
                            // Compared with the last *printed* payload, not the last received one: a change
                            // that the rate limit held back is still pending, and prints when it comes due
                            // (via robot-edge's 250 ms repeat) instead of being lost.
                            let repeat = is_repeat(args.changes, last_printed_cmd.as_deref(), &payload);
                            if !repeat && (args.all || flipped || due) {
                                println!("[t+{t:>7.1}s] {topic}  cmd {}", cmd.text);
                                last_cmd = Some((cmd.source, cmd.idle, Instant::now()));
                                last_printed_cmd = Some(payload.to_vec());
                            }
                        }
                        None => println!("[t+{t:>7.1}s] {topic}  {} B  (unrecognized command layout)", payload.len()),
                    }
                } else if args.all || first {
                    let head: String = payload.iter().take(16).map(|b| format!("{b:02x}")).collect::<Vec<_>>().join(" ");
                    let note = if first && !args.all { "  (new topic; further samples are only counted, use --all to print each)" } else { "" };
                    println!("[t+{t:>7.1}s] {topic}  {} B  {head}{note}", payload.len());
                }
            }
        }
    }

    println!("\nzenoh-monitor: {} s, totals:", started.elapsed().as_secs());
    for (k, s) in &stats {
        println!("  {k}: {} samples (last {} B)", s.total, s.last_bytes);
    }
    session.close().await.map_err(|e| anyhow!("close failed: {e}"))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_a_37_byte_sample() {
        // battery 100, roll 1.50, pitch -2.00, yaw 90.00, 15 joints of 12.34 deg
        let mut b = vec![100u8];
        b.extend_from_slice(&150i16.to_be_bytes());
        b.extend_from_slice(&(-200i16).to_be_bytes());
        b.extend_from_slice(&9000i16.to_be_bytes());
        for _ in 0..15 {
            b.extend_from_slice(&1234i16.to_be_bytes());
        }
        assert_eq!(b.len(), 37);
        let s = decode_telemetry(&b).unwrap();
        assert!(s.contains("battery=100%"));
        assert!(s.contains("roll=1.5 pitch=-2.0 yaw=90.0"));
        assert!(s.contains("joints[15]="));
        assert!(s.contains("12.3"));
    }

    #[test]
    fn decodes_a_command_sample() {
        let mut b = vec![3u8];
        for v in [0.5f32, 0.0, -0.25, 0.0, 1.5, 0.0] {
            b.extend_from_slice(&v.to_be_bytes());
        }
        b.extend_from_slice(&40i16.to_be_bytes());
        b.extend_from_slice(&(-10i16).to_be_bytes());
        b.push(128);
        assert_eq!(b.len(), 30);
        let c = decode_command(&b).unwrap();
        assert_eq!(c.source, 3);
        assert!(!c.idle);
        assert!(c.text.contains("[FullTeleop]") && c.text.contains("vx=+0.50") && c.text.contains("turn=-0.25"));
        assert!(c.text.contains("arm=(40,-10) claw=128"));
        assert!(decode_command(&[0u8; 29]).is_none());
    }

    #[test]
    fn show_filter_splits_status_from_commands() {
        assert!(Show::All.telemetry() && Show::All.commands() && Show::All.summary(1));
        assert!(Show::Status.telemetry() && !Show::Status.commands() && Show::Status.summary(1));
        assert!(!Show::Commands.telemetry() && Show::Commands.commands());
        assert!(!Show::Commands.summary(1), "commands-only hides the summary while connected");
        assert!(Show::Commands.summary(0), "but still warns when there are no peers");
    }

    #[test]
    fn changes_flag_only_suppresses_identical_commands() {
        assert!(is_repeat(true, Some(&[1, 2]), &[1, 2]));
        assert!(!is_repeat(true, Some(&[1, 2]), &[1, 3]), "a different command always passes");
        assert!(!is_repeat(true, None, &[1, 2]), "the first command always passes");
        assert!(!is_repeat(false, Some(&[1, 2]), &[1, 2]), "without --changes nothing is suppressed");
    }

    #[test]
    fn rejects_short_or_odd_payloads() {
        assert!(decode_telemetry(&[1, 2, 3]).is_none());
        assert!(decode_telemetry(&[0u8; 8]).is_none());
    }
}
