//! Stands in for a BabyROS node when testing topic forwarding
//! (scripts/smoke_test.sh): publishes numbered samples, or prints what it
//! receives.
//!
//! ```text
//! zenoh_probe pub ENDPOINT KEY COUNT [BYTES] [HZ]
//! zenoh_probe sub ENDPOINT KEY_EXPR
//! ```
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
        _ => bail!("usage: zenoh_probe pub ENDPOINT KEY COUNT [BYTES] [HZ] | sub ENDPOINT KEY_EXPR"),
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
        other => bail!("unknown mode {other:?} (pub or sub)"),
    }
    Ok(())
}
