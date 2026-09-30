//! WebRTC DataChannels via `webrtc-rs`: the native counterpart to
//! `webrtc_bench.py` (DimOS's aiortc stack). Same shape as that script --
//! one direct peer connection, host candidates only, a one-shot SDP
//! offer/answer over a local TCP socket, negotiated per-topic channels --
//! and the same channel settings as dimTELE: unordered with
//! maxRetransmits=0 for latency (`cmd_unreliable`), reliable and ordered
//! for throughput (`state_reliable`). DTLS is always on.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use bytes::Bytes;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, TcpStream};
use tokio::sync::mpsc;
use webrtc::api::setting_engine::SettingEngine;
use webrtc::api::APIBuilder;
use webrtc::data_channel::data_channel_init::RTCDataChannelInit;
use webrtc::data_channel::data_channel_message::DataChannelMessage;
use webrtc::data_channel::RTCDataChannel;
use webrtc::peer_connection::configuration::RTCConfiguration;
use webrtc::peer_connection::peer_connection_state::RTCPeerConnectionState;
use webrtc::peer_connection::sdp::session_description::RTCSessionDescription;
use webrtc::peer_connection::RTCPeerConnection;

use crate::{message, print_latency, print_throughput, seq_of, Mode, Opts, Pacer};

// Negotiated channel ids, identical on both peers. A negotiated placeholder
// (as in webrtc_bench.py / DimOS's CloudflareProvider) forces an SCTP
// m-line into the offer before any topic channel exists.
const PLACEHOLDER_ID: u16 = 100;
const PING_ID: u16 = 201;
const PONG_ID: u16 = 202;
const THROUGHPUT_ID: u16 = 203;
/// Stop queueing sends above this much unsent data, as webrtc_bench.py does
/// (skipped sends count as not delivered).
const MAX_BUFFERED: usize = 16 * 1024 * 1024;

async fn peer(o: &Opts, listen: bool) -> Result<Arc<RTCPeerConnection>> {
    let mut settings = SettingEngine::default();
    settings.set_include_loopback_candidate(true);
    let api = APIBuilder::new().with_setting_engine(settings).build();
    // No ICE servers: host candidates only, nothing leaves the machine/LAN.
    let pc = Arc::new(api.new_peer_connection(RTCConfiguration::default()).await?);
    channel(&pc, "_placeholder", PLACEHOLDER_ID, true).await?;

    let (state_tx, mut state_rx) = mpsc::unbounded_channel();
    pc.on_peer_connection_state_change(Box::new(move |s| {
        let _ = state_tx.send(s);
        Box::pin(async {})
    }));

    let port = o.port.unwrap_or(8765);
    let addr = format!("{}:{port}", o.host);
    eprintln!("{} for signaling on {addr}", if listen { "listening" } else { "connecting" });
    if listen {
        let (stream, _) = TcpListener::bind(&addr).await.with_context(|| format!("bind {addr}"))?.accept().await?;
        let (read, mut write) = stream.into_split();
        let mut line = String::new();
        BufReader::new(read).read_line(&mut line).await?;
        pc.set_remote_description(serde_json::from_str::<RTCSessionDescription>(&line)?).await?;
        let answer = pc.create_answer(None).await?;
        let local = gather(&pc, answer).await?;
        write.write_all(format!("{}\n", serde_json::to_string(&local)?).as_bytes()).await?;
    } else {
        let stream = loop {
            match TcpStream::connect(&addr).await {
                Ok(s) => break s,
                Err(_) => tokio::time::sleep(Duration::from_millis(100)).await, // listener still starting
            }
        };
        let (read, mut write) = stream.into_split();
        let offer = pc.create_offer(None).await?;
        let local = gather(&pc, offer).await?;
        write.write_all(format!("{}\n", serde_json::to_string(&local)?).as_bytes()).await?;
        let mut line = String::new();
        BufReader::new(read).read_line(&mut line).await?;
        pc.set_remote_description(serde_json::from_str::<RTCSessionDescription>(&line)?).await?;
    }

    let deadline = Instant::now() + Duration::from_secs(15);
    loop {
        match tokio::time::timeout(deadline.saturating_duration_since(Instant::now()), state_rx.recv()).await {
            Ok(Some(RTCPeerConnectionState::Connected)) => return Ok(pc),
            Ok(Some(RTCPeerConnectionState::Failed | RTCPeerConnectionState::Closed)) | Ok(None) => bail!("peer connection failed"),
            Ok(Some(_)) => {}
            Err(_) => bail!("peer connection timed out"),
        }
    }
}

/// Set the local description and wait for ICE gathering (non-trickle).
async fn gather(pc: &RTCPeerConnection, desc: RTCSessionDescription) -> Result<RTCSessionDescription> {
    let mut done = pc.gathering_complete_promise().await;
    pc.set_local_description(desc).await?;
    let _ = done.recv().await;
    pc.local_description().await.context("no local description")
}

async fn channel(pc: &RTCPeerConnection, label: &str, id: u16, reliable: bool) -> Result<Arc<RTCDataChannel>> {
    let init = RTCDataChannelInit {
        ordered: Some(reliable),
        max_retransmits: if reliable { None } else { Some(0) },
        negotiated: Some(id),
        ..Default::default()
    };
    Ok(pc.create_data_channel(label, Some(init)).await?)
}

async fn open(dc: &Arc<RTCDataChannel>) -> Result<()> {
    let (tx, mut rx) = mpsc::channel(1);
    dc.on_open(Box::new(move || {
        let _ = tx.try_send(());
        Box::pin(async {})
    }));
    if dc.ready_state() == webrtc::data_channel::data_channel_state::RTCDataChannelState::Open {
        return Ok(());
    }
    tokio::time::timeout(Duration::from_secs(15), rx.recv()).await.context("data channel didn't open")?;
    Ok(())
}

pub async fn run(o: &Opts) -> Result<()> {
    match o.mode {
        Mode::Responder => {
            let pc = peer(o, o.listen.unwrap_or(true)).await?;
            let ping = channel(&pc, "bench/ping", PING_ID, false).await?;
            let pong = channel(&pc, "bench/pong", PONG_ID, false).await?;
            open(&ping).await?;
            open(&pong).await?;
            ping.on_message(Box::new(move |m: DataChannelMessage| {
                let pong = pong.clone();
                Box::pin(async move {
                    let _ = pong.send(&m.data).await;
                })
            }));
            eprintln!("responder: echoing bench/ping -> bench/pong (DTLS, unordered, maxRetransmits=0)");
            std::future::pending::<()>().await;
            unreachable!()
        }
        Mode::PingPong => {
            let pc = peer(o, o.listen.unwrap_or(false)).await?;
            let ping = channel(&pc, "bench/ping", PING_ID, false).await?;
            let pong = channel(&pc, "bench/pong", PONG_ID, false).await?;
            open(&ping).await?;
            open(&pong).await?;
            let (tx, mut rx) = mpsc::unbounded_channel();
            pong.on_message(Box::new(move |m: DataChannelMessage| {
                let _ = tx.send(seq_of(&m.data));
                Box::pin(async {})
            }));
            tokio::time::sleep(Duration::from_millis(500)).await; // let the responder attach its handler
            eprintln!("pingpong: {} round trips, {}B payload (DTLS)", o.count, o.payload_bytes);
            let (mut rtts, mut lost) = (Vec::with_capacity(o.count), 0);
            for seq in 0..o.count as u64 {
                let msg = Bytes::from(message(seq, o.payload_bytes));
                let sent = Instant::now();
                ping.send(&msg).await?;
                let deadline = sent + Duration::from_secs(1);
                loop {
                    let left = deadline.saturating_duration_since(Instant::now());
                    match tokio::time::timeout(left, rx.recv()).await {
                        Ok(Some(Some(s))) if s == seq => {
                            rtts.push(sent.elapsed().as_secs_f64());
                            break;
                        }
                        Ok(Some(_)) => continue,
                        _ => {
                            lost += 1;
                            break;
                        }
                    }
                }
            }
            let _ = pc.close().await;
            print_latency(&rtts, lost)
        }
        Mode::Send => {
            let pc = peer(o, o.listen.unwrap_or(false)).await?;
            let dc = channel(&pc, "bench/throughput", THROUGHPUT_ID, true).await?;
            open(&dc).await?;
            eprintln!("sending {}B messages at {}Hz to bench/throughput (DTLS, reliable)", o.payload_bytes, o.rate_hz);
            let mut p = Pacer::new(o.rate_hz, o.duration_s);
            let mut skipped = 0;
            'send: while !p.done() {
                let due = p.due();
                while p.offered < due {
                    if dc.buffered_amount().await > MAX_BUFFERED {
                        skipped += 1;
                    } else {
                        // The receiver closes once its counting window ends,
                        // before this sender's schedule does; webrtc-rs then
                        // either errors or never completes the send.
                        let msg = Bytes::from(message(p.offered, o.payload_bytes));
                        match tokio::time::timeout(Duration::from_secs(1), dc.send(&msg)).await {
                            Ok(Ok(_)) => {}
                            _ => break 'send,
                        }
                    }
                    p.offered += 1;
                }
                tokio::time::sleep_until(p.next_at().into()).await;
            }
            p.report(skipped);
            let _ = tokio::time::timeout(Duration::from_secs(2), pc.close()).await;
            Ok(())
        }
        Mode::Recv => {
            let pc = peer(o, o.listen.unwrap_or(true)).await?;
            let dc = channel(&pc, "bench/throughput", THROUGHPUT_ID, true).await?;
            let msgs = Arc::new(AtomicU64::new(0));
            let bytes = Arc::new(AtomicU64::new(0));
            let counting = Arc::new(AtomicBool::new(false));
            let (m, b, on) = (msgs.clone(), bytes.clone(), counting.clone());
            dc.on_message(Box::new(move |msg: DataChannelMessage| {
                if on.load(Ordering::Relaxed) {
                    m.fetch_add(1, Ordering::Relaxed);
                    b.fetch_add(msg.data.len() as u64, Ordering::Relaxed);
                }
                Box::pin(async {})
            }));
            open(&dc).await?;
            // Window starts after connect + warmup, inside the send, as in webrtc_bench.py.
            tokio::time::sleep(Duration::from_secs_f64(o.warmup_s)).await;
            eprintln!("counting on bench/throughput for {}s", o.duration_s);
            counting.store(true, Ordering::Relaxed);
            let start = Instant::now();
            tokio::time::sleep(Duration::from_secs_f64(o.duration_s)).await;
            counting.store(false, Ordering::Relaxed);
            print_throughput(msgs.load(Ordering::Relaxed), bytes.load(Ordering::Relaxed), start.elapsed());
            let _ = pc.close().await;
            Ok(())
        }
    }
}
