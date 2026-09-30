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
}

impl BenchMode {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "echo" => Some(Self::Echo),
            "count" => Some(Self::Count),
            _ => None,
        }
    }
}
