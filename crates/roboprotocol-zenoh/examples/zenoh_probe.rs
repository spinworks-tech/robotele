//! Stands in for a BabyROS node when testing topic forwarding
//! (scripts/smoke_test.sh): publishes numbered samples, or prints what it
//! receives.
//!
//! ```text
//! zenoh_probe pub ENDPOINT KEY COUNT [BYTES] [HZ]
//! zenoh_probe sub ENDPOINT KEY_EXPR
//! zenoh_probe echo ENDPOINT KEY_EXPR OUT_KEY
//! zenoh_probe ping ENDPOINT OUT_KEY IN_KEY COUNT [BYTES] [HZ]
//! ```
//!
//! `echo` republishes every sample it receives on `OUT_KEY`, unchanged.
//! `ping` publishes numbered samples on `OUT_KEY` and times each one's
//! return on `IN_KEY` (an `echo` on the far side of the bridge), on this
//! host's clock only, so the two hosts' clocks needn't agree.
//!
//! Each published payload is `BYTES` long (default 16) and starts with the
//! sample's number as a big-endian u32; the attachment is `probe-attachment`.
//! The subscriber prints `recv KEY seq=N bytes=B attachment=A` per sample.

use std::time::Duration;

use anyhow::{bail, Context, Result};

#[tokio::main(flavor = "multi_thread", worker_threads = 1)]
async fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (mode, endpoint, key) = match &args[..] {
        [m, e, k, ..] => (m.as_str(), e.as_str(), k.as_str()),
        _ => bail!("usage: zenoh_probe pub ENDPOINT KEY COUNT [BYTES] [HZ] | sub ENDPOINT KEY_EXPR | echo ENDPOINT KEY_EXPR OUT_KEY | ping ENDPOINT OUT_KEY IN_KEY COUNT [BYTES] [HZ]"),
    };
    let mut config = roboprotocol_zenoh::isolated_config("tcp/127.0.0.1:0")?;
    config.insert_json5("connect/endpoints", &format!("[{endpoint:?}]")).map_err(|e| anyhow::anyhow!("{e}"))?;
    let session = roboprotocol_zenoh::open_session(config).await?;

    match mode {
        "pub" => {
            let count: u32 = args.get(3).context("COUNT")?.parse()?;
            let bytes: usize = args.get(4).map_or(Ok(16), |b| b.parse())?;
            let hz: f64 = args.get(5).map_or(Ok(50.0), |h| h.parse())?;
            // Let the subscription declarations reach us first.
            tokio::time::sleep(Duration::from_millis(500)).await;
            for seq in 0..count {
                let mut payload = vec![0u8; bytes.max(4)];
                payload[..4].copy_from_slice(&seq.to_be_bytes());
                session.put(key, payload).attachment(b"probe-attachment".to_vec()).await.map_err(|e| anyhow::anyhow!("{e}"))?;
                tokio::time::sleep(Duration::from_secs_f64(1.0 / hz)).await;
            }
            println!("published {count} samples on {key}");
            // Give the last samples time to leave before the session closes.
            tokio::time::sleep(Duration::from_millis(500)).await;
        }
        "sub" => {
            let sub = session.declare_subscriber(key).await.map_err(|e| anyhow::anyhow!("{e}"))?;
            println!("subscribed to {key}");
            while let Ok(sample) = sub.recv_async().await {
                let payload = sample.payload().to_bytes();
                let seq = payload.get(..4).map(|b| u32::from_be_bytes(b.try_into().unwrap()));
                let attachment = sample.attachment().map(|a| String::from_utf8_lossy(&a.to_bytes()).into_owned()).unwrap_or_default();
                println!("recv {} seq={} bytes={} attachment={attachment}", sample.key_expr(), seq.map_or("-".into(), |s| s.to_string()), payload.len());
            }
        }
        "echo" => {
            let out_key = args.get(3).context("OUT_KEY")?.clone();
            let sub = session.declare_subscriber(key).await.map_err(|e| anyhow::anyhow!("{e}"))?;
            println!("echoing {key} to {out_key}");
            while let Ok(sample) = sub.recv_async().await {
                let mut put = session.put(&out_key, sample.payload().clone());
                if let Some(a) = sample.attachment() {
                    put = put.attachment(a.clone());
                }
                put.await.map_err(|e| anyhow::anyhow!("{e}"))?;
            }
        }
        "ping" => {
            let in_key = args.get(3).context("IN_KEY")?.clone();
            let count: u32 = args.get(4).context("COUNT")?.parse()?;
            let bytes: usize = args.get(5).map_or(Ok(16), |b| b.parse())?;
            let hz: f64 = args.get(6).map_or(Ok(20.0), |h| h.parse())?;
            let sent = std::sync::Arc::new(std::sync::Mutex::new(vec![None::<std::time::Instant>; count as usize]));
            let rtts = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
            let _sub = {
                let (sent, rtts) = (sent.clone(), rtts.clone());
                session
                    .declare_subscriber(&in_key)
                    .callback(move |sample| {
                        let p = sample.payload().to_bytes();
                        let Some(seq) = p.get(..4).map(|b| u32::from_be_bytes(b.try_into().unwrap()) as usize) else { return };
                        if let Some(Some(t)) = sent.lock().unwrap().get_mut(seq).map(Option::take) {
                            rtts.lock().unwrap().push(t.elapsed().as_secs_f64() * 1e3);
                        }
                    })
                    .await
                    .map_err(|e| anyhow::anyhow!("{e}"))?
            };
            tokio::time::sleep(Duration::from_millis(500)).await;
            for seq in 0..count {
                let mut payload = vec![0u8; bytes.max(4)];
                payload[..4].copy_from_slice(&seq.to_be_bytes());
                sent.lock().unwrap()[seq as usize] = Some(std::time::Instant::now());
                session.put(key, payload).await.map_err(|e| anyhow::anyhow!("{e}"))?;
                tokio::time::sleep(Duration::from_secs_f64(1.0 / hz)).await;
            }
            tokio::time::sleep(Duration::from_secs(2)).await;
            let mut r = rtts.lock().unwrap().clone();
            r.sort_by(f64::total_cmp);
            let q = |f: f64| r.get(((r.len() as f64 * f) as usize).min(r.len().saturating_sub(1))).copied().unwrap_or(f64::NAN);
            println!(
                "ping {key} -> {in_key}: {}/{count} returned, {bytes} B, rtt ms p50 {:.1} p90 {:.1} p99 {:.1} max {:.1}",
                r.len(),
                q(0.5),
                q(0.9),
                q(0.99),
                r.last().copied().unwrap_or(f64::NAN)
            );
        }
        other => bail!("unknown mode {other:?} (pub, sub, echo or ping)"),
    }
    Ok(())
}
