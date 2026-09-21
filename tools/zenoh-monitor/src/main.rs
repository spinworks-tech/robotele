//! Listens to robot-edge's Channel C Zenoh topics (the BabyROS sidecar, see
//! BABYROS.md) and shows the data flowing: decodes `robotele/<id>/telemetry`,
//! and counts anything else published under `robotele/<id>/` (e.g. the
//! `autonomy_goal` topic) so you can see it without a flood of lines.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};

const DEFAULT_ENDPOINT: &str = "tcp/192.168.2.19:7447";
const SUMMARY_EVERY: Duration = Duration::from_secs(5);

struct Args {
    endpoints: Vec<String>,
    robot_id: String,
    key: Option<String>,
    all: bool,
}

fn parse_args() -> Result<Args> {
    let mut args = Args { endpoints: Vec::new(), robot_id: "xgo_real".to_string(), key: None, all: false };
    let mut it = std::env::args().skip(1);
    while let Some(arg) = it.next() {
        match arg.as_str() {
            "--connect" => args.endpoints.push(it.next().ok_or_else(|| anyhow!("--connect needs a value"))?),
            "--robot-id" => args.robot_id = it.next().ok_or_else(|| anyhow!("--robot-id needs a value"))?,
            "--key" => args.key = Some(it.next().ok_or_else(|| anyhow!("--key needs a value"))?),
            "--all" => args.all = true,
            "-h" | "--help" => {
                println!(
                    "Usage: zenoh-monitor [--connect tcp/HOST:PORT]... [--robot-id ID] [--key KEYEXPR] [--all]\n\n  \
                     --connect   Zenoh endpoint of robot-edge (repeatable). Default: {DEFAULT_ENDPOINT}\n  \
                     --robot-id  robot-edge's --robot-id. Default: xgo_real\n  \
                     --key       Key expression to listen on. Default: robotele/<robot-id>/**\n  \
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
                println!("[t+{:>7.1}s] peers={peers}  {}{note}", started.elapsed().as_secs_f32(), parts.join(" | "));
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
                    match decode_telemetry(&payload) {
                        Some(text) => println!("[t+{t:>7.1}s] {topic}  {} B  {text}", payload.len()),
                        None => println!("[t+{t:>7.1}s] {topic}  {} B  (unrecognized telemetry layout)", payload.len()),
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
    fn rejects_short_or_odd_payloads() {
        assert!(decode_telemetry(&[1, 2, 3]).is_none());
        assert!(decode_telemetry(&[0u8; 8]).is_none());
    }
}
