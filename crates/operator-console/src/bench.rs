//! `operator-console --bench`: drives a real, fully negotiated Channel B
//! session (same QUIC/mTLS connection, HELLO and SESSION_DESCRIBE as a
//! normal run -- `quic_client::run` hands the connection over only once it
//! reaches Operating) against `robot-edge --bench`, for the Part 4 protocol
//! comparison (docs/11-protocol-comparison.md). See `roboprotocol_core::
//! bench` for the robot side.
//!
//! Output matches `benchmark/bench_stats.py` line for line, so
//! `benchmark/run_loopback.py` parses Channel B the same way as the other
//! protocols:
//!
//! - `pingpong` prints the same trimmed latency summary to stdout;
//! - `send` prints `sent N messages over Ts (...)` to stderr, where N counts
//!   every message the application offered, including ones skipped because
//!   quiche's datagram send queue was full (those count as not delivered).
//!
//! Two framings, picked by `BenchSpec::raw`:
//!
//! - **raw** (`--bench-raw`): an opaque payload -- the same 16-byte seq/time
//!   header plus padding the other protocols' benchmarks carry -- on the
//!   `DATAGRAM_TAG_BENCH_RAW` datagram, with no FlatBuffers either way. This
//!   times Channel B's transport (QUIC datagrams, TLS 1.3 mTLS, the one-byte
//!   tag) on the same bytes as everything else: the like-for-like number.
//! - **full** (default): a real FlatBuffers `ChannelBFrame` Command, decoded
//!   by `robot-edge` up to `TeleopCommand::unpack` and echoed back as a
//!   Telemetry frame. Adds Channel B's serialization, as the protocol
//!   actually runs.
//!
//! `payload_bytes` is the application payload (raw bytes, or the frame's
//! `fields`). The whole datagram must fit `dgram_max_writable_len`, since
//! Channel B never fragments.

use std::net::SocketAddr;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use roboprotocol_core::bench::{summarize_latency, BENCH_ECHO_TICK_ID};
use roboprotocol_core::{datagram, timestamp};
use tokio::net::UdpSocket;

use crate::channel_b::{self, ChannelBCategory, ChannelBFrameData, TeleopCommand, ALL_REGIONS};

#[derive(Debug, Clone, Copy)]
pub struct BenchSpec {
    pub kind: BenchKind,
    pub payload_bytes: usize,
    /// Transport only, no FlatBuffers -- see the module docs.
    pub raw: bool,
}

#[derive(Debug, Clone, Copy)]
pub enum BenchKind {
    PingPong { count: usize },
    Send { rate_hz: f64, duration_s: f64 },
}

/// Same `>Qd` header as `benchmark/*_bench.py` and `tools/proto-bench`.
const RAW_HEADER_LEN: usize = 16;

const MAX_UDP_PAYLOAD: usize = 1452;

struct Link<'a> {
    conn: &'a mut quiche::Connection,
    socket: &'a UdpSocket,
    local: SocketAddr,
    buf: Vec<u8>,
    out: Vec<u8>,
}

impl Link<'_> {
    async fn flush(&mut self) -> Result<()> {
        loop {
            match self.conn.send(&mut self.out) {
                Ok((len, info)) => {
                    self.socket.send_to(&self.out[..len], info.to).await.context("send_to")?;
                }
                Err(quiche::Error::Done) => return Ok(()),
                Err(e) => anyhow::bail!("conn.send: {e:?}"),
            }
        }
    }

    fn feed(&mut self, len: usize, from: SocketAddr) {
        let info = quiche::RecvInfo { from, to: self.local };
        if let Err(e) = self.conn.recv(&mut self.buf[..len], info) {
            if e != quiche::Error::Done {
                tracing::debug!(error = ?e, "conn.recv");
            }
        }
    }

    /// Wait for one incoming packet or a quiche timer, up to `deadline`.
    async fn pump(&mut self, deadline: Instant) -> Result<()> {
        let now = Instant::now();
        let wait = deadline.saturating_duration_since(now);
        let wait = self.conn.timeout().map_or(wait, |t| t.min(wait));
        tokio::select! {
            r = self.socket.recv_from(&mut self.buf) => {
                let (len, from) = r.context("recv_from")?;
                self.feed(len, from);
                // Drain whatever else is already queued without waiting.
                while let Ok((len, from)) = self.socket.try_recv_from(&mut self.buf) {
                    self.feed(len, from);
                }
            }
            _ = tokio::time::sleep(wait) => self.conn.on_timeout(),
        }
        self.flush().await
    }

    /// Process anything already on the socket, without blocking.
    async fn poll(&mut self) -> Result<()> {
        while let Ok((len, from)) = self.socket.try_recv_from(&mut self.buf) {
            self.feed(len, from);
        }
        if self.conn.timeout().is_some_and(|t| t.is_zero()) {
            self.conn.on_timeout();
        }
        self.flush().await
    }

    /// Echo seqs among the Channel B datagrams received so far.
    fn drain_echoes(&mut self) -> Vec<u64> {
        let mut seqs = Vec::new();
        let mut dbuf = vec![0u8; MAX_UDP_PAYLOAD];
        while let Ok(len) = self.conn.dgram_recv(&mut dbuf) {
            if dbuf[..len].first() == Some(&datagram::DATAGRAM_TAG_BENCH_RAW) {
                if let Some(seq) = dbuf.get(1..9).and_then(|b| b.try_into().ok()).map(u64::from_be_bytes) {
                    seqs.push(seq);
                }
                continue;
            }
            let Some((tag, payload)) = datagram::untag(&dbuf[..len]) else { continue };
            if tag != datagram::DATAGRAM_TAG_CHANNEL_B {
                continue;
            }
            if let Ok(f) = channel_b::decode_channel_b_frame(&payload) {
                if f.category == ChannelBCategory::Telemetry && f.tick_id == BENCH_ECHO_TICK_ID {
                    seqs.push(f.seq);
                }
            }
        }
        seqs
    }
}

fn bench_datagram(seq: u64, payload_bytes: usize, raw: bool) -> Vec<u8> {
    if !raw {
        return command_datagram(seq, payload_bytes);
    }
    let mut out = vec![0u8; 1 + payload_bytes.max(RAW_HEADER_LEN)];
    out[0] = datagram::DATAGRAM_TAG_BENCH_RAW;
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64();
    out[1..9].copy_from_slice(&seq.to_be_bytes());
    out[9..17].copy_from_slice(&now.to_be_bytes());
    out
}

fn command_datagram(seq: u64, payload_bytes: usize) -> Vec<u8> {
    let cmd = TeleopCommand { vx: 0.0, vy: 0.0, turn: 0.0, attitude_r: 0.0, attitude_p: 0.0, attitude_y: 0.0, arm_x: 0, arm_z: 0, claw: 0 };
    let mut fields = cmd.pack();
    // Padding past the real command is ignored by `TeleopCommand::unpack`,
    // which reads only its fixed-size prefix.
    fields.resize(payload_bytes.max(fields.len()), 0);
    let frame = ChannelBFrameData {
        timestamp: timestamp::now_micros(),
        seq,
        tick_id: 0,
        category: ChannelBCategory::Command,
        region_id: ALL_REGIONS,
        fields,
    };
    datagram::tag(datagram::DATAGRAM_TAG_CHANNEL_B, &channel_b::encode_channel_b_frame(&frame))
}

pub async fn run(conn: &mut quiche::Connection, socket: &UdpSocket, local: SocketAddr, spec: BenchSpec, first_seq: u64) -> Result<()> {
    let mut link = Link { conn, socket, local, buf: vec![0u8; 65535], out: vec![0u8; MAX_UDP_PAYLOAD] };
    let BenchSpec { kind, payload_bytes, raw } = spec;
    let probe = bench_datagram(0, payload_bytes, raw);
    let max = link.conn.dgram_max_writable_len().context("peer has no datagram support")?;
    anyhow::ensure!(
        probe.len() <= max,
        "{payload_bytes} B payload makes a {} B datagram, over this connection's {max} B limit (Channel B never fragments)",
        probe.len()
    );
    let framing = if raw { "raw payload, no FlatBuffers" } else { "FlatBuffers ChannelBFrame" };
    eprintln!("channel-b bench: {payload_bytes} B payload -> {} B datagram, {framing} (max {max} B)", probe.len());

    // Let the session settle (initial telemetry, stream data) before timing,
    // feeding robot-edge's command watchdog with the same keepalive
    // `--observer` sends, so it doesn't latch E-Stop in the meantime.
    let settle = Instant::now() + Duration::from_millis(500);
    let mut heartbeat_seq = first_seq;
    while Instant::now() < settle {
        let _ = link.conn.dgram_send(&datagram::tag(datagram::DATAGRAM_TAG_HEARTBEAT, &heartbeat_seq.to_be_bytes()));
        heartbeat_seq += 1;
        link.flush().await?;
        let next = (Instant::now() + Duration::from_millis(20)).min(settle);
        while Instant::now() < next {
            link.pump(next).await?;
        }
    }
    link.drain_echoes();

    let result = match kind {
        BenchKind::PingPong { count } => pingpong(&mut link, count, payload_bytes, raw, first_seq).await,
        BenchKind::Send { rate_hz, duration_s } => send(&mut link, payload_bytes, raw, rate_hz, duration_s, first_seq).await,
    };
    let _ = link.conn.close(true, 0x0, b"bench done");
    link.flush().await?;
    result
}

async fn pingpong(link: &mut Link<'_>, count: usize, payload_bytes: usize, raw: bool, first_seq: u64) -> Result<()> {
    eprintln!("pingpong: {count} round trips, {payload_bytes}B payload (QUIC datagrams, TLS 1.3 mTLS)");
    let mut rtts = Vec::with_capacity(count);
    let mut lost = 0usize;
    for i in 0..count as u64 {
        let seq = first_seq + i;
        let sent = Instant::now();
        let _ = link.conn.dgram_send(&bench_datagram(seq, payload_bytes, raw));
        link.flush().await?;
        // Datagrams are unreliable: a lost ping is a lost sample, not a hang.
        let deadline = sent + Duration::from_secs(1);
        loop {
            if link.drain_echoes().contains(&seq) {
                rtts.push(sent.elapsed().as_secs_f64());
                break;
            }
            if Instant::now() >= deadline {
                lost += 1;
                break;
            }
            link.pump(deadline).await?;
        }
    }
    if lost > 0 {
        eprintln!("{lost} ping(s) lost (unreliable datagrams)");
    }
    anyhow::ensure!(!rtts.is_empty(), "no samples collected -- is robot-edge running with --bench echo?");
    println!("{}", summarize_latency(&rtts));
    Ok(())
}

async fn send(link: &mut Link<'_>, payload_bytes: usize, raw: bool, rate_hz: f64, duration_s: f64, first_seq: u64) -> Result<()> {
    eprintln!("sending {payload_bytes}B messages at {rate_hz}Hz for {duration_s}s");
    let start = Instant::now();
    let end = start + Duration::from_secs_f64(duration_s);
    let (mut offered, mut skipped) = (0u64, 0u64);
    loop {
        let now = Instant::now();
        if now >= end {
            break;
        }
        // Catch up to the schedule: at high rates several sends are due per
        // loop turn, well below tokio's timer granularity.
        let due = ((now - start).as_secs_f64() * rate_hz) as u64 + 1;
        while offered < due {
            match link.conn.dgram_send(&bench_datagram(first_seq + offered, payload_bytes, raw)) {
                Ok(()) => {}
                Err(quiche::Error::Done) => skipped += 1, // send queue full
                Err(e) => anyhow::bail!("dgram_send: {e:?}"),
            }
            offered += 1;
        }
        link.flush().await?;
        let next = start + Duration::from_secs_f64(offered as f64 / rate_hz);
        if next > Instant::now() {
            link.pump(next.min(end)).await?;
        } else {
            link.poll().await?;
        }
    }
    let elapsed = start.elapsed().as_secs_f64();
    eprintln!("sent {offered} messages over {elapsed:.2}s ({skipped} skipped: datagram queue full)");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_datagram_carries_the_shared_header_and_no_frame() {
        let d = bench_datagram(42, 64, true);
        assert_eq!(d.len(), 65);
        assert_eq!(d[0], datagram::DATAGRAM_TAG_BENCH_RAW);
        assert_eq!(u64::from_be_bytes(d[1..9].try_into().unwrap()), 42);
    }

    #[test]
    fn padded_command_still_unpacks_and_fits_one_datagram() {
        let d = command_datagram(7, 1200);
        let (_, payload) = datagram::untag(&d).unwrap();
        let f = channel_b::decode_channel_b_frame(&payload).unwrap();
        assert_eq!(f.fields.len(), 1200);
        assert!(TeleopCommand::unpack(&f.fields).is_some());
        assert!(d.len() <= MAX_UDP_PAYLOAD);
    }
}
