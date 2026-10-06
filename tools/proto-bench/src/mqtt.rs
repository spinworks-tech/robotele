//! MQTT 3.1.1 via `rumqttc` (rustls for TLS) against the same mosquitto as
//! `mqtt_bench.py`, QoS 0 throughout (AtMostOnce), as in the Python script.
//!
//! One difference from paho: rumqttc's request queue is bounded, so a
//! sender that outruns the socket waits instead of queueing without limit.
//! Offered == sent here, and a sender that can't keep up shows up as a
//! lower `sent` rate (the driver marks those cells sender-limited).

use std::time::{Duration, Instant};

use anyhow::{bail, Context, Result};
use rumqttc::{AsyncClient, Event, EventLoop, MqttOptions, Packet, QoS, TlsConfiguration, Transport};

use crate::{message, print_latency, print_throughput, seq_of, Mode, Opts, Pacer, PING_TOPIC, PONG_TOPIC, THROUGHPUT_TOPIC};

fn connect(o: &Opts, role: &str) -> Result<(AsyncClient, EventLoop)> {
    let port = o.port.unwrap_or(if o.tls { 8883 } else { 1883 });
    let mut opts = MqttOptions::new(format!("proto-bench-{role}-{}", std::process::id()), o.host.clone(), port);
    opts.set_keep_alive(Duration::from_secs(30));
    // 2 MiB: a 1 MiB payload plus the MQTT header must fit.
    opts.set_max_packet_size(1 << 21, 1 << 21);
    if o.tls {
        let read = |p: &Option<String>, what: &str| -> Result<Vec<u8>> {
            let path = p.as_ref().with_context(|| format!("--tls needs --{what}"))?;
            std::fs::read(path).with_context(|| format!("reading {path}"))
        };
        let client_auth = match (&o.cert, &o.key) {
            (Some(_), Some(_)) => Some((read(&o.cert, "cert")?, read(&o.key, "key")?)),
            (None, None) => None,
            _ => bail!("--cert and --key go together"),
        };
        opts.set_transport(Transport::tls_with_config(TlsConfiguration::Simple { ca: read(&o.ca, "ca")?, alpn: None, client_auth }));
    }
    Ok(AsyncClient::new(opts, 4096))
}

/// Poll until the broker acknowledges our subscription.
async fn subscribe(client: &AsyncClient, events: &mut EventLoop, topic: &str) -> Result<()> {
    client.subscribe(topic, QoS::AtMostOnce).await?;
    loop {
        if let Event::Incoming(Packet::SubAck(_)) = events.poll().await? {
            return Ok(());
        }
    }
}

pub async fn run(o: &Opts) -> Result<()> {
    let security = if o.tls { "TLS" } else { "plaintext" };
    match o.mode {
        Mode::Responder => {
            let (client, mut events) = connect(o, "responder")?;
            subscribe(&client, &mut events, PING_TOPIC).await?;
            eprintln!("responder: echoing {PING_TOPIC} -> {PONG_TOPIC} ({security})");
            loop {
                if let Event::Incoming(Packet::Publish(p)) = events.poll().await? {
                    client.try_publish(PONG_TOPIC, QoS::AtMostOnce, false, p.payload.to_vec())?;
                }
            }
        }
        Mode::PingPong => {
            let (client, mut events) = connect(o, "pinger")?;
            subscribe(&client, &mut events, PONG_TOPIC).await?;
            let (tx, mut rx) = tokio::sync::mpsc::unbounded_channel();
            tokio::spawn(async move {
                while let Ok(ev) = events.poll().await {
                    if let Event::Incoming(Packet::Publish(p)) = ev {
                        let _ = tx.send(seq_of(&p.payload));
                    }
                }
            });
            eprintln!("pingpong: {} round trips, {}B payload ({security})", o.count, o.payload_bytes);
            let (mut rtts, mut lost) = (Vec::with_capacity(o.count), 0);
            for seq in 0..o.count as u64 {
                let msg = message(seq, o.payload_bytes);
                let sent = Instant::now();
                client.publish(PING_TOPIC, QoS::AtMostOnce, false, msg).await?;
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
            print_latency(&rtts, lost)
        }
        Mode::Send => {
            let (client, mut events) = connect(o, "sender")?;
            let driver = tokio::spawn(async move { while events.poll().await.is_ok() {} });
            eprintln!("sending {}B messages at {}Hz to {THROUGHPUT_TOPIC} ({security})", o.payload_bytes, o.rate_hz);
            let mut p = Pacer::new(o.rate_hz, o.duration_s);
            while !p.done() {
                let due = p.due();
                while p.offered < due {
                    client.publish(THROUGHPUT_TOPIC, QoS::AtMostOnce, false, message(p.offered, o.payload_bytes)).await?;
                    p.offered += 1;
                }
                tokio::time::sleep_until(p.next_at().into()).await;
            }
            p.report(0);
            let _ = client.disconnect().await;
            let _ = tokio::time::timeout(Duration::from_secs(2), driver).await;
            Ok(())
        }
        Mode::Recv => {
            let (client, mut events) = connect(o, "receiver")?;
            subscribe(&client, &mut events, THROUGHPUT_TOPIC).await?;
            eprintln!("listening on {THROUGHPUT_TOPIC} for {}s after {}s warmup", o.duration_s, o.warmup_s);
            // Polled through the warmup too (that drives the client); only
            // counted after it.
            let start = Instant::now() + Duration::from_secs_f64(o.warmup_s);
            let end = start + Duration::from_secs_f64(o.duration_s);
            let (mut msgs, mut bytes) = (0u64, 0u64);
            while let Ok(ev) = tokio::time::timeout(end.saturating_duration_since(Instant::now()), events.poll()).await {
                if let Event::Incoming(Packet::Publish(p)) = ev? {
                    if Instant::now() >= start {
                        msgs += 1;
                        bytes += p.payload.len() as u64;
                    }
                }
            }
            print_throughput(msgs, bytes, Instant::now().saturating_duration_since(start));
            Ok(())
        }
    }
}
