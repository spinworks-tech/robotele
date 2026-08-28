//! 64-bit datagram timestamps and RTT/OWD calculation (DESIGN.md §3.1, SR-3.1).
//!
//! v0 simplification: DESIGN.md specifies a 64-bit NTP timestamp (32.32
//! fixed-point seconds since 1900); this module uses microseconds-since-
//! UNIX_EPOCH in a `u64` instead -- same 64-bit-on-the-wire budget and
//! sub-millisecond resolution, without pulling in full NTP era/rollover
//! handling that v0 doesn't need. `one_way_delay_ms` is only meaningful
//! with synchronized clocks (PTP on a local LAN per §3.1); v0 has no PTP
//! between the CM4 and this PC, so `robot-edge`/`operator-console` should
//! rely on RTT, not OWD, for tier decisions.

use std::time::{SystemTime, UNIX_EPOCH};

pub type Timestamp64 = u64;

pub fn now_micros() -> Timestamp64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("system clock is before UNIX_EPOCH")
        .as_micros() as u64
}

/// Standard four-timestamp RTT calculation (DESIGN.md §3.1):
/// `RTT = (T_now - T_send) - (T_reply_send - T_receive)`.
///
/// - `t_send`: local send time (T1)
/// - `t_receive`: peer's receive time, echoed back (T2)
/// - `t_reply_send`: peer's reply send time, echoed back (T3)
/// - `t_now`: local receive time of the reply (T4)
pub fn compute_rtt_ms(t_send: Timestamp64, t_receive: Timestamp64, t_reply_send: Timestamp64, t_now: Timestamp64) -> f64 {
    let round_trip = t_now as i64 - t_send as i64;
    let peer_processing = t_reply_send as i64 - t_receive as i64;
    (round_trip - peer_processing) as f64 / 1000.0
}

/// Only valid with synchronized clocks (PTP). See module docs.
pub fn one_way_delay_ms(t_send: Timestamp64, t_receive: Timestamp64) -> f64 {
    (t_receive as i64 - t_send as i64) as f64 / 1000.0
}

/// Estimates the clock offset between two endpoints from paired samples of
/// (this endpoint's local receipt time, the remote endpoint's local send
/// time for the same message) -- a concrete implementation of the
/// "software-based drift estimation" §3.1 names as the no-PTP fallback but
/// never implements anywhere in this codebase (confirmed: neither this nor
/// `compute_rtt_ms`/`one_way_delay_ms` above is currently called from
/// either binary). Returns the median of `local - remote` across
/// `samples` -- median over mean because it's robust to one-way
/// network-jitter outliers a straight average isn't -- add the result to
/// a remote timestamp to translate it onto the local clock.
///
/// `roboprotocol_core::recording`'s design relies on this: every recorded
/// `capture_us` is purely local (see that module's doc comment), so a
/// replay tool reconciling two independently-recorded logs joins them on
/// a shared `seq` to build exactly these paired samples, then calls this
/// to align both logs onto one timeline. Returns `0` for an empty sample
/// set rather than panicking -- "no data yet" is a legitimate state for a
/// tool that's only just started collecting samples.
pub fn estimate_clock_offset_us(samples: &[(u64, u64)]) -> i64 {
    if samples.is_empty() {
        return 0;
    }
    let mut deltas: Vec<i64> = samples.iter().map(|&(local, remote)| local as i64 - remote as i64).collect();
    deltas.sort_unstable();
    deltas[deltas.len() / 2]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rtt_with_zero_peer_processing_time_is_just_round_trip() {
        // T1=0, T2=5000us, T3=5000us (instant echo), T4=15000us -> RTT=15ms.
        assert_eq!(compute_rtt_ms(0, 5_000, 5_000, 15_000), 15.0);
    }

    #[test]
    fn rtt_subtracts_peer_side_processing_delay() {
        // Round trip 20ms wall clock, but peer took 3ms to process before replying.
        // T1=0, T2=8000us, T3=11000us (3ms processing), T4=20000us.
        assert_eq!(compute_rtt_ms(0, 8_000, 11_000, 20_000), 17.0);
    }

    #[test]
    fn estimate_clock_offset_recovers_a_known_offset_despite_jitter() {
        let true_offset: i64 = 50_000; // 50ms, local ahead of remote
        // Deterministic pseudo-jitter so the test has no flakiness.
        let jitters: [i64; 9] = [-3000, 2000, -1000, 500, 0, 4000, -2000, 1500, -500];
        let samples: Vec<(u64, u64)> = jitters
            .iter()
            .enumerate()
            .map(|(i, &j)| {
                let remote = 1_000_000_000u64 + i as u64 * 1000;
                let local = (remote as i64 + true_offset + j) as u64;
                (local, remote)
            })
            .collect();
        let estimated = estimate_clock_offset_us(&samples);
        assert!((estimated - true_offset).abs() <= 3000, "estimated={estimated} true={true_offset}");
    }

    #[test]
    fn estimate_clock_offset_of_empty_samples_is_zero() {
        assert_eq!(estimate_clock_offset_us(&[]), 0);
    }

    #[test]
    fn now_micros_is_monotonically_increasing_in_practice() {
        let a = now_micros();
        std::thread::sleep(std::time::Duration::from_micros(50));
        let b = now_micros();
        assert!(b >= a);
    }
}
