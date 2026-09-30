//! Native clients for the Part 4 protocol comparison
//! (docs/11-protocol-comparison.md, BENCHMARK.md).
//!
//! `benchmark/*_bench.py` measure every protocol through Python clients,
//! which is fair between those protocols but not against Channel B, whose
//! `robot-edge`/`operator-console` are compiled Rust. This binary gives
//! each of the other protocols a compiled client with the same four modes
//! and the same output lines, so `benchmark/run_loopback.py` drives both
//! the same way:
//!
//! - `responder` / `pingpong`: latency, fixed-size ping echoed back;
//!   `pingpong` prints `roboprotocol_core::bench::summarize_latency`.
//! - `send` / `recv`: one-way throughput; `send` paces at `--rate-hz` and
//!   prints `sent N messages over Ts` to stderr, `recv` counts for
//!   `--duration-s` and prints `throughput_report` to stdout.
//!
//! Every message starts with the same 16-byte header as the Python scripts
//! (`>Qd`: u64 seq, f64 send time), so `--payload-bytes` must be >= 16.
//!
//! Usage: proto-bench <udp|zenoh|mqtt|webrtc> <responder|pingpong|send|recv> [options]
//!   common:  --payload-bytes N  --count N  --rate-hz N  --duration-s N
//!   udp:     --host H --port P
//!   zenoh:   --config FILE (JSON5; TLS/mTLS lives here, as in zenoh_bench.py)
//!   mqtt:    --host H --port P [--tls --ca F --cert F --key F]
//!   webrtc:  --host H --port P (signaling) [--listen|--connect] [--warmup-s N]

mod mqtt;
mod udp;
mod webrtc_dc;
mod zenoh_ps;

use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};

pub const HEADER_LEN: usize = 16;
pub const PING_TOPIC: &str = "bench/ping";
pub const PONG_TOPIC: &str = "bench/pong";
pub const THROUGHPUT_TOPIC: &str = "bench/throughput";

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Responder,
    PingPong,
    Send,
    Recv,
}

#[derive(Debug, Clone)]
pub struct Opts {
    pub mode: Mode,
    pub host: String,
    pub port: Option<u16>,
    pub payload_bytes: usize,
    pub count: usize,
    pub rate_hz: f64,
    pub duration_s: f64,
    pub warmup_s: f64,
    pub config: Option<String>,
    pub tls: bool,
    pub ca: Option<String>,
    pub cert: Option<String>,
    pub key: Option<String>,
    /// WebRTC signaling role; `None` = the mode's default.
    pub listen: Option<bool>,
}

fn parse() -> Result<(String, Opts)> {
    let mut it = std::env::args().skip(1);
    let proto = it.next().context("usage: proto-bench <udp|zenoh|mqtt|webrtc> <mode> [options]")?;
    let mode = match it.next().context("missing mode")?.as_str() {
        "responder" => Mode::Responder,
        "pingpong" => Mode::PingPong,
        "send" => Mode::Send,
        "recv" => Mode::Recv,
        other => bail!("unknown mode {other}, expected responder|pingpong|send|recv"),
    };
    let mut o = Opts {
        mode,
        host: "127.0.0.1".into(),
        port: None,
        payload_bytes: if mode == Mode::PingPong { 64 } else { 2048 },
        count: 1000,
        rate_hz: 1000.0,
        duration_s: 30.0,
        warmup_s: 1.0,
        config: None,
        tls: false,
        ca: None,
        cert: None,
        key: None,
        listen: None,
    };
    while let Some(a) = it.next() {
        let mut val = || it.next().with_context(|| format!("{a} needs a value"));
        match a.as_str() {
            "--host" => o.host = val()?,
            "--port" => o.port = Some(val()?.parse()?),
            "--payload-bytes" => o.payload_bytes = val()?.parse()?,
            "--count" => o.count = val()?.parse()?,
            "--rate-hz" => o.rate_hz = val()?.parse()?,
            "--duration-s" => o.duration_s = val()?.parse()?,
            "--warmup-s" => o.warmup_s = val()?.parse()?,
            "--config" => o.config = Some(val()?),
            "--tls" => o.tls = true,
            "--ca" => o.ca = Some(val()?),
            "--cert" => o.cert = Some(val()?),
            "--key" => o.key = Some(val()?),
            "--listen" => o.listen = Some(true),
            "--connect" => o.listen = Some(false),
            other => bail!("unrecognized argument: {other}"),
        }
    }
    if o.payload_bytes < HEADER_LEN {
        bail!("--payload-bytes must be >= {HEADER_LEN} (seq + timestamp header)");
    }
    Ok((proto, o))
}

/// A message of `len` bytes carrying the shared `>Qd` header.
pub fn message(seq: u64, len: usize) -> Vec<u8> {
    let mut buf = vec![0u8; len];
    stamp(&mut buf, seq);
    buf
}

pub fn stamp(buf: &mut [u8], seq: u64) {
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64();
    buf[..8].copy_from_slice(&seq.to_be_bytes());
    buf[8..16].copy_from_slice(&now.to_be_bytes());
}

pub fn seq_of(buf: &[u8]) -> Option<u64> {
    Some(u64::from_be_bytes(buf.get(..8)?.try_into().ok()?))
}

/// Paced send schedule shared by every `send` mode: message `i` is due at
/// `start + i / rate`. At high rates several are due per check, far below
/// any timer's granularity, so callers send everything due, then wait.
pub struct Pacer {
    start: Instant,
    end: Instant,
    rate_hz: f64,
    pub offered: u64,
}

impl Pacer {
    pub fn new(rate_hz: f64, duration_s: f64) -> Self {
        let start = Instant::now();
        Self { start, end: start + Duration::from_secs_f64(duration_s), rate_hz, offered: 0 }
    }

    pub fn done(&self) -> bool {
        Instant::now() >= self.end
    }

    /// How many messages should have been offered by now.
    pub fn due(&self) -> u64 {
        ((self.start.elapsed().as_secs_f64() * self.rate_hz) as u64 + 1).max(self.offered)
    }

    pub fn next_at(&self) -> Instant {
        (self.start + Duration::from_secs_f64(self.offered as f64 / self.rate_hz)).min(self.end)
    }

    pub fn report(&self, skipped: u64) {
        let elapsed = self.start.elapsed().as_secs_f64();
        eprintln!("sent {} messages over {elapsed:.2}s ({skipped} skipped: send buffer full)", self.offered);
    }
}

/// Collects ping-pong samples; a lost ping (unreliable transports) is a
/// lost sample, not a hang.
pub fn print_latency(rtts: &[f64], lost: usize) -> Result<()> {
    if lost > 0 {
        eprintln!("{lost} ping(s) lost");
    }
    anyhow::ensure!(!rtts.is_empty(), "no samples collected -- is the responder running?");
    println!("{}", roboprotocol_core::bench::summarize_latency(rtts));
    Ok(())
}

pub fn print_throughput(messages: u64, bytes: u64, elapsed: Duration) {
    println!("{}", roboprotocol_core::bench::throughput_report(messages, bytes, elapsed.as_secs_f64()));
}

fn main() -> Result<()> {
    let (proto, opts) = parse()?;
    match proto.as_str() {
        // Raw UDP stays on plain blocking sockets: it's the no-protocol floor,
        // so no async runtime in the way.
        "udp" => udp::run(&opts),
        _ => {
            let rt = tokio::runtime::Builder::new_multi_thread().enable_all().build()?;
            rt.block_on(async {
                match proto.as_str() {
                    "zenoh" => zenoh_ps::run(&opts).await,
                    "mqtt" => mqtt::run(&opts).await,
                    "webrtc" => webrtc_dc::run(&opts).await,
                    other => bail!("unknown protocol {other}, expected udp|zenoh|mqtt|webrtc"),
                }
            })
        }
    }
}
