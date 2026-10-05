//! Benchmark-only Channel B conventions shared by `robot-edge --bench` and
//! `operator-console --bench` (docs/11-protocol-comparison.md, BENCHMARK.md
//! Part 4). Nothing here is used outside bench mode.
//!
//! Channel B is a one-directional streaming link (commands in, telemetry
//! out), so there is no request/response to time a round trip against.
//! Bench mode adds one: `robot-edge --bench echo` answers each Command
//! frame with a Telemetry frame carrying the command's own `seq`,
//! `timestamp` and `fields`, marked with `BENCH_ECHO_TICK_ID` so the
//! operator side can tell it apart from the regular per-tick telemetry
//! (whose own `seq` counter could otherwise collide with a pending ping's).

/// `tick_id` stamped on bench echo frames. Regular telemetry always sends
/// `tick_id: 0`, so any non-zero marker is unambiguous; this one is just
/// easy to spot in a capture.
pub const BENCH_ECHO_TICK_ID: u32 = 0xEC40_EC40;

/// What `robot-edge --bench` does with each received Channel B Command,
/// instead of dispatching it to the bridge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BenchMode {
    /// Reply at once with an echo frame (latency / ping-pong).
    Echo,
    /// Count only, logging cumulative totals once a second (one-way
    /// throughput).
    Count,
    /// Send paced messages to the operator instead of receiving them:
    /// robot-to-operator throughput, the direction sensor data travels
    /// (docs/13). `operator-console --bench recv` counts them.
    Send,
}

impl BenchMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "echo" => Some(Self::Echo),
            "count" => Some(Self::Count),
            "send" => Some(Self::Send),
            _ => None,
        }
    }
}

/// `>Qd` header of every opaque bench payload: u64 seq, f64 send time
/// (seconds since the Unix epoch), as in `benchmark/*_bench.py`.
pub const RAW_HEADER_LEN: usize = 16;

/// `robot-edge --bench send` sends a payload up to this size as one
/// `DATAGRAM_TAG_BENCH_RAW` datagram, and a larger one as sensor slices
/// (`crate::sensor`), through the same queue real sensor frames use.
pub const BENCH_SLICE_THRESHOLD: usize = crate::sensor::DEFAULT_SLICE_PAYLOAD;

/// Sensor id for sliced bench messages. Never advertised in
/// SESSION_DESCRIBE, so a normal operator ignores it.
pub const BENCH_SENSOR_ID: u8 = 0xFE;

/// Element size for sliced bench messages: 17 elements to a 1,100-byte
/// slice, like a depth map's row segments.
pub const BENCH_ELEMENT_SIZE: usize = 64;

/// An opaque bench payload of `payload_bytes` (at least `RAW_HEADER_LEN`):
/// the seq/time header, then zero padding.
pub fn raw_payload(seq: u64, payload_bytes: usize) -> Vec<u8> {
    let mut out = vec![0u8; payload_bytes.max(RAW_HEADER_LEN)];
    let now = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs_f64();
    out[..8].copy_from_slice(&seq.to_be_bytes());
    out[8..16].copy_from_slice(&now.to_be_bytes());
    out
}

/// A sliced bench message's elements: `raw_payload` padded up to whole
/// `BENCH_ELEMENT_SIZE` elements.
pub fn sliced_payload(seq: u64, payload_bytes: usize) -> Vec<u8> {
    let mut out = raw_payload(seq, payload_bytes);
    out.resize(out.len().next_multiple_of(BENCH_ELEMENT_SIZE), 0);
    out
}

/// Same method and output format as `benchmark/bench_stats.py`.
///
/// Trims to the 1st/99th percentile band, then reports the median (and
/// half of it as one-way), p95 and p99. Panics on an empty slice.
pub fn summarize_latency(rtt_seconds: &[f64]) -> String {
    assert!(!rtt_seconds.is_empty(), "summarize_latency needs at least one sample");
    let mut ordered = rtt_seconds.to_vec();
    ordered.sort_by(|a, b| a.total_cmp(b));
    let n = ordered.len();
    let lo = ordered[(n as f64 * 0.01) as usize];
    let hi = ordered[((n as f64 * 0.99) as usize).min(n - 1)];
    let mut us: Vec<f64> = ordered.iter().copied().filter(|s| (lo..=hi).contains(s)).map(|s| s * 1e6).collect();
    if us.is_empty() {
        us = ordered.iter().map(|s| s * 1e6).collect();
    }
    let m = us.len();
    let median = if m % 2 == 1 { us[m / 2] } else { (us[m / 2 - 1] + us[m / 2]) / 2.0 };
    let pct = |p: f64| {
        if m == 1 {
            return us[0];
        }
        let idx = p * (m - 1) as f64;
        let lo_idx = idx as usize;
        let hi_idx = (lo_idx + 1).min(m - 1);
        us[lo_idx] + (us[hi_idx] - us[lo_idx]) * (idx - lo_idx as f64)
    };
    format!(
        "n={n} (trimmed to {m} after 1st/99th pct cut)  median RTT={median:.1}us (one-way={:.1}us)  p95={:.1}us  p99={:.1}us  min={:.1}us  max={:.1}us",
        median / 2.0,
        pct(0.95),
        pct(0.99),
        us[0],
        us[m - 1]
    )
}

/// `benchmark/bench_stats.py`'s `ThroughputReport` line, for the same parser.
pub fn throughput_report(messages: u64, bytes: u64, elapsed_s: f64) -> String {
    let (rate, bps) = if elapsed_s > 0.0 { (messages as f64 / elapsed_s, bytes as f64 * 8.0 / elapsed_s) } else { (0.0, 0.0) };
    format!("{messages} msgs, {bytes} bytes in {elapsed_s:.2}s -> {rate:.1} msg/s, {:.2} Mbps", bps / 1e6)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn send_mode_parses() {
        assert_eq!(BenchMode::parse("send"), Some(BenchMode::Send));
    }

    #[test]
    fn bench_payloads_carry_seq_and_pad_to_size() {
        let p = raw_payload(7, 64);
        assert_eq!((p.len(), u64::from_be_bytes(p[..8].try_into().unwrap())), (64, 7));
        assert_eq!(raw_payload(1, 4).len(), RAW_HEADER_LEN, "never shorter than the header");
        assert_eq!(sliced_payload(1, 2_000).len(), 2_048);
        assert_eq!(sliced_payload(1, 4_096).len(), 4_096);
    }

    #[test]
    fn latency_summary_matches_bench_stats_py() {
        // Reference: benchmark/bench_stats.py summarize_latency() on the same input.
        let rtts: Vec<f64> = (1..=200).map(|i| i as f64 * 1e-6).collect();
        assert_eq!(
            summarize_latency(&rtts),
            "n=200 (trimmed to 197 after 1st/99th pct cut)  median RTT=101.0us (one-way=50.5us)  p95=189.2us  p99=197.0us  min=3.0us  max=199.0us"
        );
    }

    #[test]
    fn throughput_report_matches_bench_stats_py() {
        // Reference: str(ThroughputReport(3000, 6144000, 3.0)).
        assert_eq!(throughput_report(3000, 6_144_000, 3.0), "3000 msgs, 6144000 bytes in 3.00s -> 1000.0 msg/s, 16.38 Mbps");
    }
}
