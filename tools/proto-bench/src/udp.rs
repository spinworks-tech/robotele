//! Raw UDP on blocking std sockets: the protocol-free floor. Unlike
//! `raw_udp_baseline.py` this also has responder/pingpong, so the latency
//! table gets a native UDP floor too.

use std::net::UdpSocket;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};

use crate::{message, print_latency, print_throughput, seq_of, stamp, Mode, Opts, Pacer};

const MAX_DATAGRAM: usize = 65507;

pub fn run(o: &Opts) -> Result<()> {
    let port = o.port.unwrap_or(match o.mode {
        Mode::Responder | Mode::PingPong => 5006,
        Mode::Send | Mode::Recv => 5005,
    });
    let addr = format!("{}:{port}", o.host);
    match o.mode {
        Mode::Responder => {
            let sock = UdpSocket::bind(&addr).with_context(|| format!("bind {addr}"))?;
            eprintln!("responder: echoing on {addr}");
            let mut buf = vec![0u8; MAX_DATAGRAM];
            loop {
                let (len, from) = sock.recv_from(&mut buf)?;
                let _ = sock.send_to(&buf[..len], from);
            }
        }
        Mode::PingPong => {
            let sock = UdpSocket::bind("0.0.0.0:0")?;
            sock.connect(&addr)?;
            sock.set_read_timeout(Some(Duration::from_secs(1)))?;
            eprintln!("pingpong: {} round trips, {}B payload (raw UDP)", o.count, o.payload_bytes);
            let mut out = message(0, o.payload_bytes);
            let mut buf = vec![0u8; MAX_DATAGRAM];
            let (mut rtts, mut lost) = (Vec::with_capacity(o.count), 0);
            for seq in 0..o.count as u64 {
                stamp(&mut out, seq);
                let sent = Instant::now();
                sock.send(&out)?;
                loop {
                    match sock.recv(&mut buf) {
                        Ok(len) if seq_of(&buf[..len]) == Some(seq) => {
                            rtts.push(sent.elapsed().as_secs_f64());
                            break;
                        }
                        Ok(_) => continue, // a late reply to an earlier, already-lost ping
                        Err(_) => {
                            lost += 1;
                            break;
                        }
                    }
                }
            }
            print_latency(&rtts, lost)
        }
        Mode::Send => {
            anyhow::ensure!(o.payload_bytes <= MAX_DATAGRAM, "--payload-bytes over the {MAX_DATAGRAM} B UDP maximum");
            // Unconnected, as in raw_udp_baseline.py: a connected socket
            // would surface ICMP port-unreachable (receiver not bound yet)
            // as send errors.
            let sock = UdpSocket::bind("0.0.0.0:0")?;
            let dest: std::net::SocketAddr = addr.parse()?;
            eprintln!("sending {}B datagrams at {}Hz to {addr}", o.payload_bytes, o.rate_hz);
            let mut out = message(0, o.payload_bytes);
            let mut p = Pacer::new(o.rate_hz, o.duration_s);
            let mut skipped = 0;
            while !p.done() {
                let due = p.due();
                while p.offered < due {
                    stamp(&mut out, p.offered);
                    if sock.send_to(&out, dest).is_err() {
                        skipped += 1; // ENOBUFS: kernel send buffer full
                    }
                    p.offered += 1;
                }
                let wait = p.next_at().saturating_duration_since(Instant::now());
                if wait > Duration::from_micros(100) {
                    std::thread::sleep(wait);
                }
            }
            p.report(skipped);
            Ok(())
        }
        Mode::Recv => {
            let sock = UdpSocket::bind(&addr).with_context(|| format!("bind {addr}"))?;
            eprintln!("listening on {addr} for {}s", o.duration_s);
            let mut buf = vec![0u8; MAX_DATAGRAM];
            let (mut msgs, mut bytes) = (0u64, 0u64);
            let start = Instant::now();
            let end = start + Duration::from_secs_f64(o.duration_s);
            loop {
                let left = end.saturating_duration_since(Instant::now());
                if left.is_zero() {
                    break;
                }
                sock.set_read_timeout(Some(left))?;
                match sock.recv(&mut buf) {
                    Ok(len) => {
                        msgs += 1;
                        bytes += len as u64;
                    }
                    Err(_) => break,
                }
            }
            print_throughput(msgs, bytes, start.elapsed());
            Ok(())
        }
    }
}
