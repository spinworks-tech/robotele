//! Zenoh via the `zenoh` 1.x crate -- the same Rust core the Python bindings
//! in `zenoh_bench.py` wrap, and the same JSON5 `--config` files
//! (`benchmark/tls/zenoh_tls_*.json5` for mTLS). Default config is peer mode
//! with multicast scouting, as in the Python script.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anyhow::{anyhow, Result};
use zenoh::{Config, Wait};

use crate::{message, print_latency, print_throughput, seq_of, Mode, Opts, Pacer, PING_TOPIC, PONG_TOPIC, THROUGHPUT_TOPIC};

fn z<T>(r: zenoh::Result<T>) -> Result<T> {
    r.map_err(|e| anyhow!("zenoh: {e}"))
}

async fn open(o: &Opts) -> Result<zenoh::Session> {
    let config = match &o.config {
        Some(path) => z(Config::from_file(path))?,
        None => Config::default(),
    };
    z(zenoh::open(config).await)
}

pub async fn run(o: &Opts) -> Result<()> {
    let transport = o.config.as_deref().map_or("default transport".to_string(), |c| format!("config {c}"));
    let session = open(o).await?;
    match o.mode {
        Mode::Responder => {
            let publisher = z(session.declare_publisher(PONG_TOPIC).await)?;
            let _sub = z(session
                .declare_subscriber(PING_TOPIC)
                .callback(move |sample| {
                    let _ = publisher.put(sample.payload().to_bytes().into_owned()).wait();
                })
                .await)?;
            eprintln!("responder: echoing {PING_TOPIC} -> {PONG_TOPIC} ({transport})");
            std::future::pending::<()>().await;
            unreachable!()
        }
        Mode::PingPong => {
            let sub = z(session.declare_subscriber(PONG_TOPIC).await)?;
            let publisher = z(session.declare_publisher(PING_TOPIC).await)?;
            tokio::time::sleep(Duration::from_millis(500)).await; // discovery/subscription settle
            eprintln!("pingpong: {} round trips, {}B payload ({transport})", o.count, o.payload_bytes);
            let (mut rtts, mut lost) = (Vec::with_capacity(o.count), 0);
            for seq in 0..o.count as u64 {
                let msg = message(seq, o.payload_bytes);
                let sent = Instant::now();
                z(publisher.put(msg).await)?;
                let deadline = sent + Duration::from_secs(1);
                loop {
                    let left = deadline.saturating_duration_since(Instant::now());
                    match tokio::time::timeout(left, sub.recv_async()).await {
                        Ok(Ok(sample)) if seq_of(&sample.payload().to_bytes()) == Some(seq) => {
                            rtts.push(sent.elapsed().as_secs_f64());
                            break;
                        }
                        Ok(Ok(_)) => continue,
                        _ => {
                            lost += 1;
                            break;
                        }
                    }
                }
            }
            print_latency(&rtts, lost)
        }
        Mode::Send => {
            let publisher = z(session.declare_publisher(THROUGHPUT_TOPIC).await)?;
            eprintln!("sending {}B messages at {}Hz to {THROUGHPUT_TOPIC} ({transport})", o.payload_bytes, o.rate_hz);
            let mut p = Pacer::new(o.rate_hz, o.duration_s);
            while !p.done() {
                let due = p.due();
                while p.offered < due {
                    // Default publisher QoS, as in zenoh_bench.py: reliable,
                    // congestion control Drop.
                    z(publisher.put(message(p.offered, o.payload_bytes)).await)?;
                    p.offered += 1;
                }
                tokio::time::sleep_until(p.next_at().into()).await;
            }
            p.report(0);
            Ok(())
        }
        Mode::Recv => {
            let msgs = Arc::new(AtomicU64::new(0));
            let bytes = Arc::new(AtomicU64::new(0));
            let counting = Arc::new(AtomicBool::new(false));
            let (m, b, on) = (msgs.clone(), bytes.clone(), counting.clone());
            let _sub = z(session
                .declare_subscriber(THROUGHPUT_TOPIC)
                .callback(move |sample| {
                    if on.load(Ordering::Relaxed) {
                        m.fetch_add(1, Ordering::Relaxed);
                        b.fetch_add(sample.payload().len() as u64, Ordering::Relaxed);
                    }
                })
                .await)?;
            eprintln!("listening on {THROUGHPUT_TOPIC} for {}s after {}s warmup", o.duration_s, o.warmup_s);
            // The publisher only routes to subscribers it has heard about;
            // over a real link that takes a while, so count after a warmup.
            tokio::time::sleep(Duration::from_secs_f64(o.warmup_s)).await;
            counting.store(true, Ordering::Relaxed);
            let start = Instant::now();
            tokio::time::sleep(Duration::from_secs_f64(o.duration_s)).await;
            counting.store(false, Ordering::Relaxed);
            let elapsed = start.elapsed();
            print_throughput(msgs.load(Ordering::Relaxed), bytes.load(Ordering::Relaxed), elapsed);
            Ok(())
        }
    }
}
