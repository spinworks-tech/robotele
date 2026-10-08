//! Keeps video and sensor data from filling the link (docs/12, "Paced 50 Hz
//! control on the CM4").
//!
//! `lossy_queue` bounds how much lossy data waits inside quiche, but on the
//! robot's Wi-Fi that wasn't enough: under 2x overload control round trips
//! reached ~1 s and the watchdog latched, because the robot kept the Wi-Fi
//! driver's firmware queue and the access point's full -- queues below the
//! kernel, out of reach of `tc` (`fq_codel` never dropped a packet there).
//! The only lever left is to send less.
//!
//! How much less changes minute to minute on Wi-Fi, so the cap is adaptive,
//! driven by the delay we care about. QUIC's smoothed RTT minus the lowest
//! smoothed RTT seen recently (`BASE_RTT_WINDOW`) is how long packets sit in
//! queues along the path. That baseline is tracked here, not taken from
//! quiche: quiche 0.22's `PathStats::min_rtt` reports `None` from about a
//! second into a session onwards (its windowed minimum drops to exactly
//! zero, which it treats as unknown), which on the CM4 left the controller
//! blind -- it read the queueing delay as zero and raised the cap to its
//! ceiling while control round trips reached 3 s. Every
//! `UPDATE_INTERVAL` the cap on lossy traffic is cut by `DECREASE` while that
//! queueing delay is over `TARGET_QUEUE_DELAY` -- or while data went out but
//! nothing new was acknowledged, because the smoothed RTT only updates when
//! ACKs arrive and freezes at its last, low value through exactly the
//! stalls that matter most (on the CM4 that let the cap climb back to
//! ~18 Mbps of sensor data during a multi-second stall). Otherwise it's
//! raised by `INCREASE_BPS` -- the LEDBAT idea, a "scavenger" that
//! yields to interactive traffic. Control traffic is never metered. A token
//! bucket enforces the cap in front of quiche; whatever doesn't fit stays in
//! `lossy_queue`, whose latest-wins rules drop it when it goes stale.

use std::time::{Duration, Instant};

pub const UPDATE_INTERVAL: Duration = Duration::from_millis(100);
/// Queueing delay the cap aims to stay under. Control pings ride the same
/// connection, so this is roughly the delay lossy traffic may add to them.
pub const TARGET_QUEUE_DELAY: Duration = Duration::from_millis(20);
pub const DECREASE: f64 = 0.8;
pub const INCREASE_BPS: f64 = 250_000.0;
/// Never below this, so the camera keeps a (degraded) picture.
pub const MIN_BPS: f64 = 1_000_000.0;
pub const MAX_BPS: f64 = 50_000_000.0;
const START_BPS: f64 = 4_000_000.0;
/// The bucket holds this much of the current rate, at least
/// `MIN_BURST_BYTES`: enough for the gaps between flushes (one control tick
/// is 20 ms) without letting a long idle stretch turn into a big burst.
const BURST: Duration = Duration::from_millis(25);
const MIN_BURST_BYTES: f64 = 4_000.0;
/// How long the lowest smoothed RTT counts as the path's baseline. Long
/// enough to span any overload episode, short enough to follow a real path
/// change (e.g. Wi-Fi to cellular). The same window quiche uses for its own.
pub const BASE_RTT_WINDOW: Duration = Duration::from_secs(300);

#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RateStats {
    pub decreases: u64,
    pub min_bps: f64,
    pub max_bps: f64,
    /// Time-weighted mean of the cap, for the end-of-session log.
    pub mean_bps: f64,
}

pub struct LossyRateControl {
    rate_bps: f64,
    tokens: f64,
    last_refill: Instant,
    last_update: Instant,
    started: Instant,
    weighted_sum: f64,
    /// QUIC's cumulative sent and acknowledged bytes at the last update.
    last_sent_bytes: u64,
    last_acked_bytes: u64,
    last_trace: Instant,
    /// Lowest smoothed RTT seen, and when, within `BASE_RTT_WINDOW`.
    base_rtt: Option<(Duration, Instant)>,
    stats: RateStats,
}

impl LossyRateControl {
    pub fn new(now: Instant) -> Self {
        Self {
            rate_bps: START_BPS,
            tokens: MIN_BURST_BYTES,
            last_refill: now,
            last_update: now,
            started: now,
            weighted_sum: 0.0,
            last_sent_bytes: 0,
            last_acked_bytes: 0,
            last_trace: now,
            base_rtt: None,
            stats: RateStats { min_bps: START_BPS, max_bps: START_BPS, ..RateStats::default() },
        }
    }

    #[cfg(test)]
    pub fn rate_bps(&self) -> f64 {
        self.rate_bps
    }

    pub fn stats(&self, now: Instant) -> RateStats {
        let elapsed = now.duration_since(self.started).as_secs_f64();
        let since_update = now.duration_since(self.last_update).as_secs_f64();
        let sum = self.weighted_sum + self.rate_bps * since_update;
        RateStats { mean_bps: if elapsed > 0.0 { sum / elapsed } else { self.rate_bps }, ..self.stats }
    }

    /// Feeds the path's current smoothed RTT and the connection's cumulative
    /// sent and acknowledged bytes; adjusts the cap at most once per
    /// `UPDATE_INTERVAL`.
    pub fn update(&mut self, now: Instant, srtt: Duration, sent_bytes: u64, acked_bytes: u64) {
        let base = match self.base_rtt {
            Some((b, at)) if b <= srtt && now.duration_since(at) < BASE_RTT_WINDOW => b,
            _ => {
                self.base_rtt = Some((srtt, now));
                srtt
            }
        };
        let since = now.duration_since(self.last_update);
        if since < UPDATE_INTERVAL {
            return;
        }
        self.weighted_sum += self.rate_bps * since.as_secs_f64();
        self.last_update = now;
        let stalled = sent_bytes > self.last_sent_bytes && acked_bytes == self.last_acked_bytes;
        self.last_sent_bytes = sent_bytes;
        self.last_acked_bytes = acked_bytes;
        let queue_delay = srtt.saturating_sub(base);
        if now.duration_since(self.last_trace) >= Duration::from_secs(1) {
            self.last_trace = now;
            // RUST_LOG=robot_edge::lossy_rate=debug to watch it work.
            tracing::debug!(
                cap_mbps = self.rate_bps / 1e6,
                srtt_ms = srtt.as_secs_f64() * 1e3,
                base_rtt_ms = base.as_secs_f64() * 1e3,
                queue_delay_ms = queue_delay.as_secs_f64() * 1e3,
                stalled,
                "lossy rate control"
            );
        }
        if stalled || queue_delay > TARGET_QUEUE_DELAY {
            self.rate_bps *= DECREASE;
            self.stats.decreases += 1;
        } else {
            self.rate_bps += INCREASE_BPS;
        }
        self.rate_bps = self.rate_bps.clamp(MIN_BPS, MAX_BPS);
        self.stats.min_bps = self.stats.min_bps.min(self.rate_bps);
        self.stats.max_bps = self.stats.max_bps.max(self.rate_bps);
    }

    /// Whether there are tokens for `bytes` of lossy data now. The caller
    /// checks before taking a datagram out of `lossy_queue` (it can't be put
    /// back) and then charges its real size with `consume`.
    pub fn has_room(&mut self, now: Instant, bytes: usize) -> bool {
        let capacity = (self.rate_bps / 8.0 * BURST.as_secs_f64()).max(MIN_BURST_BYTES);
        let refill = self.rate_bps / 8.0 * now.duration_since(self.last_refill).as_secs_f64();
        self.tokens = (self.tokens + refill).min(capacity);
        self.last_refill = now;
        self.tokens >= bytes as f64
    }

    /// Tokens available now, after refilling: the bytes of bulk data that
    /// may follow the lossy datagrams (see `bulk_sender`).
    pub fn available(&mut self, now: Instant) -> usize {
        self.has_room(now, 0);
        self.tokens.max(0.0) as usize
    }

    pub fn consume(&mut self, bytes: usize) {
        self.tokens -= bytes as f64;
    }

    /// `has_room` then `consume`, for when the size is known up front.
    #[cfg(test)]
    pub fn try_take(&mut self, now: Instant, bytes: usize) -> bool {
        let ok = self.has_room(now, bytes);
        if ok {
            self.consume(bytes);
        }
        ok
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: Duration = Duration::from_millis(1);

    /// A controller that has seen a 2 ms baseline RTT.
    fn with_baseline(t0: Instant) -> LossyRateControl {
        let mut c = LossyRateControl::new(t0);
        c.update(t0, 2 * MS, 0, 0);
        c
    }

    #[test]
    fn queueing_delay_over_target_cuts_the_rate() {
        let t0 = Instant::now();
        let mut c = with_baseline(t0);
        c.update(t0 + 100 * MS, 60 * MS, 0, 0);
        assert!((c.rate_bps() - START_BPS * DECREASE).abs() < 1.0);
        assert_eq!(c.stats(t0 + 100 * MS).decreases, 1);
    }

    #[test]
    fn a_short_queue_raises_the_rate_slowly() {
        let t0 = Instant::now();
        let mut c = LossyRateControl::new(t0);
        for i in 1..=4 {
            c.update(t0 + i * 100 * MS, 5 * MS, 0, 0);
        }
        assert!((c.rate_bps() - (START_BPS + 4.0 * INCREASE_BPS)).abs() < 1.0);
    }

    #[test]
    fn updates_are_rate_limited_and_the_rate_is_clamped() {
        let t0 = Instant::now();
        let mut c = with_baseline(t0);
        c.update(t0 + 50 * MS, 500 * MS, 0, 0);
        assert_eq!(c.rate_bps(), START_BPS, "too soon after the last update");
        for i in 1..=100 {
            c.update(t0 + i * 100 * MS, 500 * MS, 0, 0);
        }
        assert_eq!(c.rate_bps(), MIN_BPS, "never below the floor");
    }

    #[test]
    fn sending_with_nothing_acknowledged_counts_as_congestion() {
        let t0 = Instant::now();
        let mut c = LossyRateControl::new(t0);
        // Healthy: bytes sent and acknowledged both advance; srtt is low.
        c.update(t0 + 100 * MS, 3 * MS, 10_000, 9_000);
        assert!(c.rate_bps() > START_BPS);
        let before = c.rate_bps();
        // A stall: more sent, nothing new acknowledged, srtt still frozen low.
        c.update(t0 + 200 * MS, 3 * MS, 20_000, 9_000);
        assert!((c.rate_bps() - before * DECREASE).abs() < 1.0);
        // Idle (nothing sent, nothing acked) isn't a stall.
        let idle = c.rate_bps();
        c.update(t0 + 300 * MS, 3 * MS, 20_000, 9_000);
        assert!(c.rate_bps() > idle);
    }

    #[test]
    fn the_baseline_is_the_lowest_recent_rtt_and_expires() {
        let t0 = Instant::now();
        let mut c = with_baseline(t0);
        // 60 ms over a 2 ms baseline: a queue, so a cut.
        c.update(t0 + 100 * MS, 62 * MS, 0, 0);
        assert!((c.rate_bps() - START_BPS * DECREASE).abs() < 1.0);
        // After the window, the baseline restarts from the current RTT, so a
        // path that is simply slower now (60 ms) stops looking congested.
        let later = t0 + BASE_RTT_WINDOW + 200 * MS;
        let before = c.rate_bps();
        c.update(later, 60 * MS, 0, 0);
        c.update(later + 100 * MS, 61 * MS, 0, 0);
        assert!(c.rate_bps() > before);
    }

    #[test]
    fn the_bucket_paces_to_the_rate() {
        let t0 = Instant::now();
        let mut c = LossyRateControl::new(t0);
        let mut sent = 0usize;
        // One second of 1,000-byte datagrams offered every millisecond.
        for ms in 0..1_000u32 {
            if c.try_take(t0 + ms * MS, 1_000) {
                sent += 1_000;
            }
        }
        let mbps = sent as f64 * 8.0 / 1e6;
        assert!((3.5..=4.5).contains(&mbps), "{mbps} Mbps, expected about the 4 Mbps start rate");
    }

    #[test]
    fn stats_track_the_time_weighted_mean() {
        let t0 = Instant::now();
        let mut c = with_baseline(t0);
        c.update(t0 + 100 * MS, 60 * MS, 0, 0);
        let s = c.stats(t0 + 200 * MS);
        let expected = (START_BPS * 0.1 + START_BPS * DECREASE * 0.1) / 0.2;
        assert!((s.mean_bps - expected).abs() < 1.0, "{} vs {expected}", s.mean_bps);
        assert_eq!((s.min_bps, s.max_bps), (START_BPS * DECREASE, START_BPS));
    }
}
