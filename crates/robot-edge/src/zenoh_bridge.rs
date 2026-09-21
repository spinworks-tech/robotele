//! Channel C sidecar: a non-critical Zenoh telemetry publisher and
//! autonomy-goal subscriber, bolted onto the control loop rather than
//! inserted into it (spinworks-tech/robotele#5).
//!
//! Neither direction can affect the safety-critical path:
//!   - Telemetry publish is fire-and-forget through a depth-1 channel
//!     (latest-only, dropped under backpressure) drained by its own task, so
//!     a slow/absent Zenoh peer can never stall a control tick.
//!   - The autonomy goal is a single freshness-gated flag (see
//!     `AutonomyGoal::asserted`) read once per safety tick. Its own value
//!     only ever feeds `ArbitrationInputs::autonomy_goal_asserted`, which the
//!     arbitration ladder (`roboprotocol_core::safety::arbitration`) ranks
//!     below `FullTeleoperation` -- teleop always preempts it regardless of
//!     what this module does.
//!
//! Zenoh session-open failures degrade to "no sidecar this run" (logged, not
//! fatal): this is an optional observability/semi-autonomy add-on, not
//! something teleoperation should ever depend on to start.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::mpsc;

use crate::channel_b::TelemetryData;

/// An autonomy goal is only honored while a message arrived within this
/// window -- absence of fresh messages decays to "not asserted" rather than
/// latching forever, mirroring the fail-safe default already used by the
/// Channel B command watchdog (SR-4.1) and the arbitration ladder's own
/// "hold, don't drift" fallback.
const AUTONOMY_GOAL_TTL: Duration = Duration::from_millis(500);

fn telemetry_topic(robot_id: &str) -> String {
    format!("robotele/{robot_id}/telemetry")
}

fn autonomy_goal_topic(robot_id: &str) -> String {
    format!("robotele/{robot_id}/autonomy_goal")
}

/// Shared, lock-free flag fed by the autonomy-goal subscriber and read every
/// safety tick via `SafetyTask::tick`. Freshness-gated: see `AUTONOMY_GOAL_TTL`.
#[derive(Clone)]
pub struct AutonomyGoal {
    last_seen_ms: Arc<AtomicU64>,
    epoch: Instant,
}

impl AutonomyGoal {
    fn new() -> Self {
        Self { last_seen_ms: Arc::new(AtomicU64::new(0)), epoch: Instant::now() }
    }

    fn mark_seen(&self, now: Instant) {
        let ms = now.saturating_duration_since(self.epoch).as_millis() as u64;
        self.last_seen_ms.store(ms, Ordering::Relaxed);
    }

    /// True iff an autonomy-goal message arrived within the last
    /// [`AUTONOMY_GOAL_TTL`]. Never latches: a stalled or crashed autonomy
    /// node silently stops asserting rather than leaving a stale goal in
    /// effect.
    pub fn asserted(&self) -> bool {
        let last_ms = self.last_seen_ms.load(Ordering::Relaxed);
        if last_ms == 0 {
            return false; // never seen a message this run
        }
        let now_ms = Instant::now().saturating_duration_since(self.epoch).as_millis() as u64;
        now_ms.saturating_sub(last_ms) < AUTONOMY_GOAL_TTL.as_millis() as u64
    }
}

/// Send handle for the telemetry publisher task. `publish` never blocks and
/// never returns an error the caller needs to act on: a full channel (a slow
/// or absent Zenoh peer) just drops the sample, same as any other Channel C
/// bulk-data backpressure policy in this codebase.
#[derive(Clone)]
pub struct TelemetrySink(mpsc::Sender<TelemetryData>);

impl TelemetrySink {
    pub fn publish(&self, data: TelemetryData) {
        // try_send, not send().await: this must never make the control tick
        // wait on Zenoh. Depth-1 channel means only the latest sample is ever
        // queued, so a fresh one preempts a not-yet-sent stale one instead of
        // piling up.
        if let Err(mpsc::error::TrySendError::Full(_)) = self.0.try_send(data) {
            tracing::trace!("zenoh telemetry channel full, dropping stale sample");
        }
    }
}

/// A no-op sink for when the sidecar didn't start (Zenoh session failed to
/// open) or is disabled -- callers publish unconditionally rather than
/// threading an `Option` through the control loop.
fn inert_sink() -> TelemetrySink {
    // Capacity 1 with the receiver immediately dropped: every publish() hits
    // the "full" (actually "closed") branch and is silently discarded.
    let (tx, _rx) = mpsc::channel(1);
    TelemetrySink(tx)
}

/// Builds a Zenoh config listening on `port` (TCP, all interfaces, both
/// address families) instead of the default's ephemeral (`:0`) peer-mode
/// listen port -- a fixed port a firewall rule or a BabyROS node's `connect`
/// endpoint can target, rather than one that changes every run. Multicast
/// scouting (still on by default) means most same-LAN setups never need
/// this; it matters once that's blocked or a specific unicast peer is
/// wanted instead.
fn config_with_port(port: u16) -> zenoh::Config {
    let mut config = zenoh::Config::default();
    if let Err(e) = config.insert_json5("listen/endpoints", &format!(r#"["tcp/[::]:{port}"]"#)) {
        tracing::warn!(error = ?e, port, "failed to set zenoh listen port, falling back to the default (ephemeral) port");
    }
    config
}

/// Opens a Zenoh session and starts the telemetry-publisher and
/// autonomy-goal-subscriber background tasks. Always returns usable handles:
/// on any Zenoh failure, logs a warning and returns an inert sink / a goal
/// that's never asserted, so the caller never needs a fallback branch of its
/// own.
pub async fn spawn(robot_id: &str, port: u16) -> (TelemetrySink, AutonomyGoal) {
    let goal = AutonomyGoal::new();

    let session = match zenoh::open(config_with_port(port)).await {
        Ok(session) => session,
        Err(e) => {
            tracing::warn!(error = ?e, "zenoh session open failed, Channel C sidecar disabled this run");
            return (inert_sink(), goal);
        }
    };

    let (tx, mut rx) = mpsc::channel::<TelemetryData>(1);

    match session.declare_publisher(telemetry_topic(robot_id)).await {
        Ok(publisher) => {
            tokio::spawn(async move {
                while let Some(data) = rx.recv().await {
                    // Reuses robot-edge's own compact Channel B wire encoding
                    // rather than introducing a second format; a cross-language
                    // schema (e.g. matching BabyROS/telekinesis-datatypes'
                    // codec) is a separate follow-up, not needed for this
                    // same-process producer/consumer pair.
                    if let Err(e) = publisher.put(data.pack()).await {
                        tracing::warn!(error = ?e, "zenoh telemetry publish failed");
                    }
                }
            });
        }
        Err(e) => {
            tracing::warn!(error = ?e, "zenoh telemetry publisher declare failed, telemetry sidecar disabled this run");
        }
    }

    {
        let goal = goal.clone();
        let topic = autonomy_goal_topic(robot_id);
        match session.declare_subscriber(topic).callback(move |_sample| goal.mark_seen(Instant::now())).await {
            Ok(subscriber) => {
                // Held for the process lifetime: dropping it would undeclare
                // the subscription. `robot-edge` never tears this down early.
                std::mem::forget(subscriber);
            }
            Err(e) => {
                tracing::warn!(error = ?e, "zenoh autonomy-goal subscriber declare failed, semi-autonomy disabled this run");
            }
        }
    }

    // Keep the session alive for the process lifetime, same reasoning as the
    // subscriber above -- dropping it would close the whole Zenoh session out
    // from under the publisher task.
    std::mem::forget(session);

    (TelemetrySink(tx), goal)
}
