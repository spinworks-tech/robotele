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

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use tokio::sync::{mpsc, watch};

use roboprotocol_core::recording::control_source_byte;
use roboprotocol_core::safety::ControlSource;

use crate::channel_b::{TeleopCommand, TelemetryData};

/// An autonomy goal is only honored while a message arrived within this
/// window -- absence of fresh messages decays to "not asserted" rather than
/// latching forever, mirroring the fail-safe default already used by the
/// Channel B command watchdog (SR-4.1) and the arbitration ladder's own
/// "hold, don't drift" fallback.
const AUTONOMY_GOAL_TTL: Duration = Duration::from_millis(500);

fn telemetry_topic(robot_id: &str) -> String {
    format!("robotele/{robot_id}/telemetry")
}

fn command_topic(robot_id: &str) -> String {
    format!("robotele/{robot_id}/command")
}

fn video_topic(robot_id: &str) -> String {
    format!("robotele/{robot_id}/video")
}

fn autonomy_goal_topic(robot_id: &str) -> String {
    format!("robotele/{robot_id}/autonomy_goal")
}

/// Per-axis limits applied to an autonomy velocity, in the same robot-native
/// units the operator console sends (xgolib `move_x`/`move_y`/`turn`; see
/// `operator-console`'s `MOVE_SCALE` / `TURN_SCALE` and the WASD magnitudes).
/// An autonomy source can therefore never command faster than the operator can.
pub const AUTONOMY_MAX_VX: f32 = 15.0;
pub const AUTONOMY_MAX_VY: f32 = 12.0;
pub const AUTONOMY_MAX_TURN: f32 = 60.0;

/// Velocity carried by an autonomy-goal sample, robot-native units.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AutonomyVelocity {
    pub vx: f32,
    pub vy: f32,
    pub turn: f32,
}

/// Decodes an `autonomy_goal` payload: `vx, vy, turn` as big-endian `f32`
/// (12 bytes), clamped to the `AUTONOMY_MAX_*` limits. Any other length (e.g.
/// the legacy 1-byte "flag only" sample) or a non-finite value decodes to zero
/// velocity: the goal is still asserted, the robot just holds still.
pub fn decode_autonomy_velocity(payload: &[u8]) -> AutonomyVelocity {
    let Ok(b): Result<[u8; 12], _> = payload.try_into() else {
        return AutonomyVelocity::default();
    };
    let f = |i: usize| f32::from_be_bytes(b[i..i + 4].try_into().unwrap());
    let (vx, vy, turn) = (f(0), f(4), f(8));
    if !(vx.is_finite() && vy.is_finite() && turn.is_finite()) {
        return AutonomyVelocity::default();
    }
    AutonomyVelocity {
        vx: vx.clamp(-AUTONOMY_MAX_VX, AUTONOMY_MAX_VX),
        vy: vy.clamp(-AUTONOMY_MAX_VY, AUTONOMY_MAX_VY),
        turn: turn.clamp(-AUTONOMY_MAX_TURN, AUTONOMY_MAX_TURN),
    }
}

/// Shared flag + latest velocity fed by the autonomy-goal subscriber and read
/// every safety tick via `SafetyTask::tick`. Freshness-gated: see `AUTONOMY_GOAL_TTL`.
#[derive(Clone)]
pub struct AutonomyGoal {
    last_seen_ms: Arc<AtomicU64>,
    velocity: Arc<Mutex<AutonomyVelocity>>,
    epoch: Instant,
}

impl AutonomyGoal {
    fn new() -> Self {
        Self {
            last_seen_ms: Arc::new(AtomicU64::new(0)),
            velocity: Arc::new(Mutex::new(AutonomyVelocity::default())),
            epoch: Instant::now(),
        }
    }

    fn mark_seen(&self, now: Instant, payload: &[u8]) {
        *self.velocity.lock().unwrap() = decode_autonomy_velocity(payload);
        // +1 so a sample in the epoch's own first millisecond is not stored
        // as 0, which `asserted` reads as "never seen".
        let ms = now.saturating_duration_since(self.epoch).as_millis() as u64 + 1;
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
        let now_ms = Instant::now().saturating_duration_since(self.epoch).as_millis() as u64 + 1;
        now_ms.saturating_sub(last_ms) < AUTONOMY_GOAL_TTL.as_millis() as u64
    }

    /// The last received velocity while the goal is fresh, else zero, so a
    /// stale velocity can never keep driving the robot.
    pub fn velocity(&self) -> AutonomyVelocity {
        if self.asserted() {
            *self.velocity.lock().unwrap()
        } else {
            AutonomyVelocity::default()
        }
    }
}

/// Bound on queued critical (SPS/PPS/IDR) NALs. They are rare (about one GOP's
/// worth per second) so this only fills if the Zenoh peer has wedged; a NAL
/// dropped here just means the subscriber waits for the next IDR.
const VIDEO_CRITICAL_QUEUE: usize = 32;

/// Send handle for the Zenoh video tap: every NAL the encoder produces, as an
/// Annex-B payload (start code prepended), published on `robotele/<id>/video`
/// **only while at least one subscriber matches**. Same contract as
/// `TelemetrySink`: it never blocks and never errors into the caller, so a
/// slow, absent or wedged Zenoh peer cannot affect the QUIC video path or a
/// control tick.
///
/// Mirrors `channel_a`'s own split: SPS/PPS/IDR are queued and always
/// delivered in order (a decoder cannot recover without them); everything
/// else is latest-wins and dropped under congestion.
#[derive(Clone)]
pub struct VideoSink {
    critical: mpsc::Sender<Vec<u8>>,
    delta: Arc<watch::Sender<Option<Vec<u8>>>>,
    subscribed: Arc<AtomicBool>,
}

impl VideoSink {
    /// `payload` is one NAL with its Annex-B start code; `critical` is
    /// `roboprotocol_core::video::nal_is_critical` for that NAL.
    pub fn publish(&self, payload: &[u8], critical: bool) {
        if !self.subscribed.load(Ordering::Relaxed) {
            return; // nobody is watching: no copy, no queueing
        }
        if critical {
            if self.critical.try_send(payload.to_vec()).is_err() {
                tracing::trace!("zenoh video critical queue full, dropping NAL");
            }
        } else {
            self.delta.send_replace(Some(payload.to_vec()));
        }
    }
}

/// NAL type of an Annex-B payload that starts with a 4-byte start code.
fn annexb_nal_type(payload: &[u8]) -> Option<u8> {
    payload.get(4).map(|b| b & 0x1f)
}

fn inert_video_sink() -> VideoSink {
    let (critical, _rx) = mpsc::channel(1);
    let (delta, _drx) = watch::channel(None);
    VideoSink { critical, delta: Arc::new(delta), subscribed: Arc::new(AtomicBool::new(false)) }
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

/// How often an unchanged operator command is re-published, so a monitor that
/// joins late (or missed a sample) still sees the current state.
const COMMAND_REPEAT: Duration = Duration::from_millis(250);

/// Wire layout of a `robotele/<id>/command` sample, big-endian, 30 bytes:
/// `control_source u8` (same numbering as the recordings: 0 EStop, 1
/// EmergencySafeParking, 2 ActiveImpedanceHold, 3 FullTeleoperation, 4
/// SemiAutonomous), then `vx, vy, turn, roll, pitch, yaw` as `f32`, `arm_x,
/// arm_z` as `i16`, `claw` as `u8`. It is the operator's *requested* command
/// and the source that was arbitrated for it -- not necessarily what reached
/// the motors (e.g. it is still published while E-Stopped).
pub fn encode_command(source: ControlSource, cmd: &TeleopCommand) -> Vec<u8> {
    let mut out = Vec::with_capacity(30);
    out.push(control_source_byte(source));
    for v in [cmd.vx, cmd.vy, cmd.turn, cmd.attitude_r, cmd.attitude_p, cmd.attitude_y] {
        out.extend_from_slice(&v.to_be_bytes());
    }
    out.extend_from_slice(&cmd.arm_x.to_be_bytes());
    out.extend_from_slice(&cmd.arm_z.to_be_bytes());
    out.push(cmd.claw);
    out
}

/// Send handle for the operator-command publisher. Like `TelemetrySink`, it
/// never blocks: `try_send` into a depth-1 channel drained by its own task.
/// Publishes only when the command or its arbitrated source changed, plus a
/// repeat every [`COMMAND_REPEAT`], so 50 Hz operator input doesn't become
/// 50 Hz of Zenoh traffic on the CM4. A sample dropped because the channel was
/// full is retried on the next tick (it isn't recorded as sent).
#[derive(Clone)]
pub struct CommandSink {
    tx: mpsc::Sender<Vec<u8>>,
    last: Option<(Vec<u8>, Instant)>,
}

impl CommandSink {
    pub fn publish(&mut self, source: ControlSource, cmd: &TeleopCommand) {
        self.publish_at(Instant::now(), source, cmd);
    }

    fn publish_at(&mut self, now: Instant, source: ControlSource, cmd: &TeleopCommand) {
        let payload = encode_command(source, cmd);
        if let Some((last, at)) = &self.last {
            if *last == payload && now.saturating_duration_since(*at) < COMMAND_REPEAT {
                return;
            }
        }
        if self.tx.try_send(payload.clone()).is_ok() {
            self.last = Some((payload, now));
        }
    }
}

fn inert_command_sink() -> CommandSink {
    let (tx, _rx) = mpsc::channel(1);
    CommandSink { tx, last: None }
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

/// Declares the video publishers and the task draining them. On any Zenoh
/// failure returns an inert sink (video simply isn't offered over Zenoh).
async fn spawn_video(session: &zenoh::Session, robot_id: &str) -> VideoSink {
    use zenoh::qos::CongestionControl;

    let topic = video_topic(robot_id);
    // Critical NALs must not be dropped by congestion control; delta NALs
    // may be (the decoder recovers at the next IDR).
    let (critical_pub, delta_pub) = match (
        session.declare_publisher(topic.clone()).congestion_control(CongestionControl::Block).await,
        session.declare_publisher(topic).congestion_control(CongestionControl::Drop).await,
    ) {
        (Ok(c), Ok(d)) => (c, d),
        (Err(e), _) | (_, Err(e)) => {
            tracing::warn!(error = ?e, "zenoh video publisher declare failed, video tap disabled this run");
            return inert_video_sink();
        }
    };

    let subscribed = Arc::new(AtomicBool::new(false));
    let (resub_tx, mut resub_rx) = mpsc::channel::<()>(1);
    {
        let subscribed = subscribed.clone();
        match delta_pub
            .matching_listener()
            .callback(move |status| {
                subscribed.store(status.matching(), Ordering::Relaxed);
                if status.matching() {
                    // A subscriber joined (possibly mid-GOP): replay SPS/PPS.
                    let _ = resub_tx.try_send(());
                }
            })
            .await
        {
            // Held for the process lifetime, like the other subscriptions.
            Ok(listener) => std::mem::forget(listener),
            Err(e) => {
                tracing::warn!(error = ?e, "zenoh video matching listener failed, video tap disabled this run");
                return inert_video_sink();
            }
        }
    }

    let (critical_tx, mut critical_rx) = mpsc::channel::<Vec<u8>>(VIDEO_CRITICAL_QUEUE);
    let (delta_tx, mut delta_rx) = watch::channel::<Option<Vec<u8>>>(None);
    tokio::spawn(async move {
        let (mut sps, mut pps): (Option<Vec<u8>>, Option<Vec<u8>>) = (None, None);
        loop {
            tokio::select! {
                biased;
                Some(payload) = critical_rx.recv() => {
                    match annexb_nal_type(&payload) {
                        Some(7) => sps = Some(payload.clone()),
                        Some(8) => pps = Some(payload.clone()),
                        _ => {}
                    }
                    if let Err(e) = critical_pub.put(payload).await {
                        tracing::warn!(error = ?e, "zenoh video publish failed");
                    }
                }
                Some(()) = resub_rx.recv() => {
                    for p in [&sps, &pps].into_iter().flatten() {
                        let _ = critical_pub.put(p.clone()).await;
                    }
                }
                changed = delta_rx.changed() => {
                    if changed.is_err() {
                        return; // every sink dropped
                    }
                    let payload = delta_rx.borrow_and_update().clone();
                    if let Some(p) = payload {
                        if let Err(e) = delta_pub.put(p).await {
                            tracing::warn!(error = ?e, "zenoh video publish failed");
                        }
                    }
                }
            }
        }
    });

    VideoSink { critical: critical_tx, delta: Arc::new(delta_tx), subscribed }
}

/// Opens a Zenoh session and starts the telemetry-publisher and
/// autonomy-goal-subscriber background tasks. Always returns usable handles:
/// on any Zenoh failure, logs a warning and returns an inert sink / a goal
/// that's never asserted, so the caller never needs a fallback branch of its
/// own.
pub async fn spawn(robot_id: &str, port: u16) -> (TelemetrySink, CommandSink, AutonomyGoal, VideoSink) {
    let goal = AutonomyGoal::new();

    let session = match zenoh::open(config_with_port(port)).await {
        Ok(session) => session,
        Err(e) => {
            tracing::warn!(error = ?e, "zenoh session open failed, Channel C sidecar disabled this run");
            return (inert_sink(), inert_command_sink(), goal, inert_video_sink());
        }
    };

    let (tx, mut rx) = mpsc::channel::<TelemetryData>(1);
    let (cmd_tx, mut cmd_rx) = mpsc::channel::<Vec<u8>>(1);

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

    match session.declare_publisher(command_topic(robot_id)).await {
        Ok(publisher) => {
            tokio::spawn(async move {
                while let Some(payload) = cmd_rx.recv().await {
                    if let Err(e) = publisher.put(payload).await {
                        tracing::warn!(error = ?e, "zenoh command publish failed");
                    }
                }
            });
        }
        Err(e) => {
            tracing::warn!(error = ?e, "zenoh command publisher declare failed, command stream disabled this run");
        }
    }

    {
        let goal = goal.clone();
        let topic = autonomy_goal_topic(robot_id);
        match session.declare_subscriber(topic).callback(move |sample| goal.mark_seen(Instant::now(), &sample.payload().to_bytes())).await {
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

    let video = spawn_video(&session, robot_id).await;

    // Keep the session alive for the process lifetime, same reasoning as the
    // subscriber above -- dropping it would close the whole Zenoh session out
    // from under the publisher task.
    std::mem::forget(session);

    (TelemetrySink(tx), CommandSink { tx: cmd_tx, last: None }, goal, video)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn velocity_payload(vx: f32, vy: f32, turn: f32) -> Vec<u8> {
        [vx, vy, turn].iter().flat_map(|v| v.to_be_bytes()).collect()
    }

    #[test]
    fn autonomy_velocity_decodes_and_clamps_to_operator_limits() {
        let v = decode_autonomy_velocity(&velocity_payload(5.0, -3.0, 20.0));
        assert_eq!(v, AutonomyVelocity { vx: 5.0, vy: -3.0, turn: 20.0 });
        let v = decode_autonomy_velocity(&velocity_payload(999.0, -999.0, 999.0));
        assert_eq!(v, AutonomyVelocity { vx: AUTONOMY_MAX_VX, vy: -AUTONOMY_MAX_VY, turn: AUTONOMY_MAX_TURN });
    }

    #[test]
    fn autonomy_velocity_fails_safe_to_zero() {
        assert_eq!(decode_autonomy_velocity(&[1]), AutonomyVelocity::default(), "legacy flag-only sample");
        assert_eq!(decode_autonomy_velocity(&[]), AutonomyVelocity::default());
        assert_eq!(decode_autonomy_velocity(&velocity_payload(f32::NAN, 1.0, 1.0)), AutonomyVelocity::default());
        assert_eq!(decode_autonomy_velocity(&velocity_payload(1.0, f32::INFINITY, 1.0)), AutonomyVelocity::default());
    }

    #[test]
    fn goal_velocity_is_zero_until_a_sample_arrives() {
        let goal = AutonomyGoal::new();
        assert!(!goal.asserted());
        assert_eq!(goal.velocity(), AutonomyVelocity::default());
        goal.mark_seen(Instant::now(), &velocity_payload(4.0, 0.0, 0.0));
        assert!(goal.asserted());
        assert_eq!(goal.velocity().vx, 4.0);
    }

    fn cmd(vx: f32) -> TeleopCommand {
        TeleopCommand { vx, vy: 0.0, turn: 0.0, attitude_r: 0.0, attitude_p: 0.0, attitude_y: 0.0, arm_x: 10, arm_z: -20, claw: 128 }
    }

    #[test]
    fn command_layout_is_30_bytes_with_source_first() {
        let b = encode_command(ControlSource::FullTeleoperation, &cmd(0.5));
        assert_eq!(b.len(), 30);
        assert_eq!(b[0], 3);
        assert_eq!(f32::from_be_bytes(b[1..5].try_into().unwrap()), 0.5);
        assert_eq!(i16::from_be_bytes(b[25..27].try_into().unwrap()), 10);
        assert_eq!(i16::from_be_bytes(b[27..29].try_into().unwrap()), -20);
        assert_eq!(b[29], 128);
    }

    #[test]
    fn unchanged_commands_are_throttled_but_changes_and_repeats_go_out() {
        let (tx, mut rx) = mpsc::channel(8);
        let mut sink = CommandSink { tx, last: None };
        let t0 = Instant::now();
        sink.publish_at(t0, ControlSource::FullTeleoperation, &cmd(0.5));
        sink.publish_at(t0 + Duration::from_millis(20), ControlSource::FullTeleoperation, &cmd(0.5)); // same, too soon
        assert_eq!(rx.try_recv().unwrap()[0], 3);
        assert!(rx.try_recv().is_err(), "identical command within the repeat window must not be re-sent");

        sink.publish_at(t0 + Duration::from_millis(40), ControlSource::FullTeleoperation, &cmd(0.6)); // changed
        assert!(rx.try_recv().is_ok());
        sink.publish_at(t0 + Duration::from_millis(60), ControlSource::EStop, &cmd(0.6)); // source changed
        assert_eq!(rx.try_recv().unwrap()[0], 0);
        sink.publish_at(t0 + Duration::from_millis(400), ControlSource::EStop, &cmd(0.6)); // repeat interval elapsed
        assert!(rx.try_recv().is_ok());
    }

    #[test]
    fn a_full_channel_drops_the_sample_and_retries_next_tick() {
        let (tx, mut rx) = mpsc::channel(1);
        let mut sink = CommandSink { tx, last: None };
        let t0 = Instant::now();
        sink.publish_at(t0, ControlSource::FullTeleoperation, &cmd(0.1)); // fills the channel
        sink.publish_at(t0 + Duration::from_millis(10), ControlSource::FullTeleoperation, &cmd(0.2)); // dropped (full)
        assert_eq!(f32::from_be_bytes(rx.try_recv().unwrap()[1..5].try_into().unwrap()), 0.1);
        sink.publish_at(t0 + Duration::from_millis(20), ControlSource::FullTeleoperation, &cmd(0.2)); // retried
        assert_eq!(f32::from_be_bytes(rx.try_recv().unwrap()[1..5].try_into().unwrap()), 0.2);
    }
    #[test]
    fn video_sink_is_silent_until_a_subscriber_matches() {
        let (critical, mut crx) = mpsc::channel(4);
        let (delta, drx) = watch::channel(None);
        let subscribed = Arc::new(AtomicBool::new(false));
        let sink = VideoSink { critical, delta: Arc::new(delta), subscribed: subscribed.clone() };

        sink.publish(&[0, 0, 0, 1, 0x67], true);
        sink.publish(&[0, 0, 0, 1, 0x41], false);
        assert!(crx.try_recv().is_err(), "no subscriber: critical NAL must not be queued");
        assert!(drx.borrow().is_none(), "no subscriber: delta NAL must not be stored");

        subscribed.store(true, Ordering::Relaxed);
        sink.publish(&[0, 0, 0, 1, 0x67], true);
        sink.publish(&[0, 0, 0, 1, 0x41, 1], false);
        sink.publish(&[0, 0, 0, 1, 0x41, 2], false);
        assert_eq!(crx.try_recv().unwrap(), vec![0, 0, 0, 1, 0x67]);
        assert_eq!(drx.borrow().clone().unwrap(), vec![0, 0, 0, 1, 0x41, 2], "delta is latest-wins");
    }

    #[test]
    fn video_sink_drops_critical_nals_when_the_queue_is_full_instead_of_blocking() {
        let (critical, mut crx) = mpsc::channel(1);
        let (delta, _drx) = watch::channel(None);
        let sink = VideoSink { critical, delta: Arc::new(delta), subscribed: Arc::new(AtomicBool::new(true)) };
        sink.publish(&[0, 0, 0, 1, 0x65, 1], true);
        sink.publish(&[0, 0, 0, 1, 0x65, 2], true); // full: dropped, must not panic or block
        assert_eq!(crx.try_recv().unwrap()[5], 1);
        assert!(crx.try_recv().is_err());
    }

    #[test]
    fn annexb_nal_type_reads_the_header_after_the_start_code() {
        assert_eq!(annexb_nal_type(&[0, 0, 0, 1, 0x67]), Some(7));
        assert_eq!(annexb_nal_type(&[0, 0, 0, 1, 0x68]), Some(8));
        assert_eq!(annexb_nal_type(&[0, 0, 0, 1, 0x65]), Some(5));
        assert_eq!(annexb_nal_type(&[0, 0, 0, 1]), None);
    }

}
