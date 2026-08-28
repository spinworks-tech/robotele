//! robot-edge's QUIC server: connection setup and the main driving loop.
//!
//! v0 simplification (documented): single peer connection only (no
//! connection-ID routing table for multiple clients), no Retry/address
//! validation token dance (quiche's internal anti-amplification limiting
//! still applies -- it just paces `conn.send()` output rather than
//! erroring), no connection migration.

use std::net::SocketAddr;
use std::time::{Duration, Instant};

use anyhow::{Context, Result};
use roboprotocol_core::action_trigger::{TriggerDedup, ACTION_TRIGGER_STREAM_ID};
use roboprotocol_core::camera_control::{CameraControlDedup, CAMERA_CONTROL_STREAM_ID};
use roboprotocol_core::datagram;
use roboprotocol_core::estop::{EstopDatagram, ESTOP_DATAGRAM_MAGIC, ESTOP_STREAM_ID, ESTOP_STREAM_URGENCY};
use roboprotocol_core::hello::{self, quantization_tier_bits, task_class_bits, HelloCapabilities, ProtocolVersion};
use roboprotocol_core::profile::CameraDescriptor;
use roboprotocol_core::safety::TaskClass;
use roboprotocol_core::video::chunk_nal;
use tokio::net::UdpSocket;
use tokio::time::MissedTickBehavior;

use crate::action_trigger_handler;
use crate::bridge::{BridgeCommand, BridgeConfig, BridgeSupervisor, SupervisorEvent};
use crate::camera_control_handler;
use crate::channel_b::{self, ChannelBCategory, ChannelBFrameData, TeleopCommand, ALL_REGIONS};
use crate::hello_handler;
use crate::safety_task::SafetyTask;
use crate::session_handler;
use crate::video::channel_a::VideoRx;
use crate::video::{capture, channel_a};
use roboprotocol_core::profile::RobotProfile;

const MAX_DATAGRAM_SIZE: usize = 1452;

pub struct ServerArgs {
    pub listen: SocketAddr,
    pub cert_path: String,
    pub key_path: String,
    pub ca_path: String,
    pub task_class: TaskClass,
    pub robot_id: String,
    pub tick_hz: u32,
    pub bridge: BridgeConfig,
    pub camera_config: Option<capture::CaptureConfig>,
    pub recording: roboprotocol_recording::RecorderConfig,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    AwaitingHello,
    AwaitingSessionAccept,
    Operating,
}

pub async fn run(args: ServerArgs, profile: RobotProfile, cameras: Vec<CameraDescriptor>) -> Result<()> {
    let mut config = build_quiche_config(&args)?;
    let socket = UdpSocket::bind(args.listen).await.context("binding UDP socket")?;
    let local_addr = socket.local_addr()?;
    tracing::info!(addr = %local_addr, "robot-edge listening");

    // Constructed once, outside the reconnect loop below, so segments/
    // rotation/rings persist across reconnects instead of being torn
    // down and rebuilt with every new connection. `Recorder` is cheap to
    // clone (an `Arc` underneath) into each `Session`.
    let recorder = roboprotocol_recording::Recorder::start(args.recording.clone())?;

    // v0 keeps the single-active-connection design (no CID-routing table for
    // concurrent clients -- see module docs) but must not let one connection
    // ending, however it ends, take the whole process down with it: every
    // iteration blocks for a fresh first packet and serves exactly one
    // session, the same way the original one-shot version did, and then
    // loops back instead of returning. A session ending in error is
    // recoverable -- log it and wait for the next connection -- so only
    // genuinely fatal setup failures (bind, cert load, above) still bail out
    // of `run` entirely.
    loop {
        let mut buf = vec![0u8; 65535];

        let (len, peer) = match socket.recv_from(&mut buf).await.context("waiting for first packet") {
            Ok(v) => v,
            Err(e) => {
                tracing::warn!(error = ?e, "waiting for first packet failed, retrying");
                continue;
            }
        };
        let hdr = match quiche::Header::from_slice(&mut buf[..len], quiche::MAX_CONN_ID_LEN) {
            Ok(h) => h,
            Err(e) => {
                tracing::warn!(error = ?e, "invalid initial packet, ignoring and awaiting next connection");
                continue;
            }
        };
        let scid = quiche::ConnectionId::from_vec(hdr.dcid.to_vec());
        let mut conn = match quiche::accept(&scid, None, local_addr, peer, &mut config) {
            Ok(c) => c,
            Err(e) => {
                tracing::warn!(error = ?e, "quiche::accept failed, awaiting next connection");
                continue;
            }
        };
        tracing::info!(peer = %peer, trace_id = conn.trace_id(), "accepting connection");

        // A recv() error on this first packet is *not* necessarily fatal to
        // the connection -- e.g. a rejected/undecryptable 0-RTT session
        // ticket (operator offering a stale resumption ticket from a
        // different robot-edge instance/cert) fails to decrypt the 0-RTT
        // portion but the connection can still complete a normal handshake
        // from the client's retransmitted Initial packets. Discarding `conn`
        // and going back to a fresh `quiche::accept()` here would recreate
        // the exact same failure on every retransmission forever (verified:
        // this used to loop indefinitely on CryptoFail against a stale
        // ticket) -- so just log and hand off to the session loop like every
        // other recv() error already does, and let quiche's own handshake
        // state machine recover across subsequent packets.
        let recv_info = quiche::RecvInfo { from: peer, to: local_addr };
        if let Err(e) = conn.recv(&mut buf[..len], recv_info) {
            tracing::warn!(error = ?e, "initial conn.recv failed (may still recover on a later packet)");
        }

        let bridge = BridgeSupervisor::spawn(args.bridge.clone());
        let capture_handle = args.camera_config.clone().map(capture::spawn_capture);
        let camera_controls_tx = capture_handle.as_ref().map(|h| h.controls_tx.clone());
        let video_rx = capture_handle.map(|h| channel_a::spawn_encoder(h.rx, recorder.clone()));

        let mut session = Session {
            conn,
            peer,
            local_addr,
            task_class: args.task_class,
            robot_id: args.robot_id.clone(),
            profile: profile.clone(),
            cameras: cameras.clone(),
            safety: SafetyTask::new(args.task_class, Instant::now()),
            bridge,
            video_rx,
            camera_controls_tx,
            camera_control_dedup: CameraControlDedup::new(),
            phase: Phase::AwaitingHello,
            channel_b_seq: 0,
            estop_seq: 0,
            tick_interval: Duration::from_secs_f64(1.0 / args.tick_hz.max(1) as f64),
            hello_sent: false,
            session_describe_sent: false,
            estop_stream_primed: false,
            last_reported_estop_latched: None,
            bridge_estop_active: false,
            tick_count: 0,
            latest_telemetry: None,
            trigger_dedup: TriggerDedup::new(),
            action_settle_until: None,
            sitting: false,
            recorder: recorder.clone(),
        };

        match session.run(&socket, buf).await {
            Ok(()) => tracing::info!("session ended, awaiting next connection"),
            Err(e) => tracing::warn!(error = ?e, "session ended with error, awaiting next connection"),
        }
    }
}

struct Session {
    conn: quiche::Connection,
    peer: SocketAddr,
    local_addr: SocketAddr,
    task_class: TaskClass,
    robot_id: String,
    profile: RobotProfile,
    cameras: Vec<CameraDescriptor>,
    safety: SafetyTask,
    bridge: BridgeSupervisor,
    /// See `channel_a`'s module doc: SPS/PPS/IDR NALs are always
    /// delivered, delta NALs coalesce to latest-only under backlog.
    video_rx: Option<VideoRx>,
    /// `None` when no camera is attached this session (`--camera` not
    /// passed) -- a `CameraControl` arriving on a cameraless session is a
    /// no-op, logged and ignored, not an error.
    camera_controls_tx: Option<tokio::sync::watch::Sender<crate::video::capture::CameraControls>>,
    /// At-most-once application of `CameraControl` RPCs, same reasoning as
    /// `trigger_dedup` below.
    camera_control_dedup: CameraControlDedup,
    phase: Phase,
    channel_b_seq: u64,
    estop_seq: u64,
    tick_interval: Duration,
    hello_sent: bool,
    session_describe_sent: bool,
    estop_stream_primed: bool,
    last_reported_estop_latched: Option<bool>,
    /// Tracks whether the bridge subprocess was last told it's E-Stopped,
    /// so the transition back to normal is reported exactly once -- see
    /// the `on_tick` E-Stop-to-bridge dispatch for why this can't just
    /// mirror the every-tick `Estop` send.
    bridge_estop_active: bool,
    tick_count: u64,
    latest_telemetry: Option<channel_b::TelemetryData>,
    /// FR-1.8: at-most-once application of ActionTrigger RPCs.
    trigger_dedup: TriggerDedup,
    /// Set by `on_action_trigger` for `STAND_ACTION_ID`/`SIT_ACTION_ID` --
    /// while `Instant::now()` is before this, `dispatch_teleop_command`
    /// withholds Move/Turn/Arm so they don't fight the in-progress gait.
    /// See that constant's doc comment for why.
    action_settle_until: Option<Instant>,
    /// Set by a `SIT_ACTION_ID` trigger, cleared by `STAND_ACTION_ID` or by
    /// the operator sending a genuine (non-zero) Move/Turn -- unlike
    /// `action_settle_until`'s few-second window that only protects the
    /// sit-down motion itself, this holds for as long as the operator
    /// leaves the robot sitting: confirmed on real hardware that
    /// re-asserting `move_x(0)/move_y(0)/turn(0)` (robot-edge's normal
    /// "hold at rest" idle dispatch) reads to the firmware as "resume
    /// standing," not "stay put," so continuing to send it at all -- even
    /// zero -- stands the robot back up seconds after it finishes sitting.
    /// Arm/Claw are unaffected; only Move/Turn are withheld.
    sitting: bool,
    recorder: roboprotocol_recording::Recorder,
}

/// Query the bridge for fresh telemetry every N ticks rather than every
/// tick: `xgo_bridge.py`'s `read_telemetry()` is five sequential blocking
/// serial round-trips (read_motor/read_battery/read_roll/pitch/yaw), each
/// riding on `xgolib`'s `__unpack()`, which busy-polls for up to a full
/// second per field if a reply is missed -- confirmed on real hardware
/// 2026-08-21 that the previous 5Hz guess let a single missed reply stall
/// the bridge's one JSON-lines processing thread for multiple seconds,
/// visibly delaying queued arm/move commands behind it (the thread is
/// strictly sequential -- see `xgo_bridge.py`'s `handle_line` loop).
/// Dropped to 1Hz as an immediate, low-risk mitigation: it reduces how
/// often a stall gets triggered, not the worst-case stall length itself
/// when one does happen -- the real fix (telemetry polling off the
/// command-processing thread) is still open.
const TELEMETRY_QUERY_EVERY_N_TICKS: u64 = 50;

/// xgolib's `action()` is fire-and-forget -- it writes one register and
/// returns immediately, while the actual multi-second stand-up/sit-down
/// gait runs asynchronously on the robot's own MCU (confirmed by reading
/// the xgolib source; it has no completion feedback at all). Meanwhile
/// `dispatch_teleop_command` sends Move/Turn/Arm every tick regardless,
/// which on real hardware visibly aborts a stand/sit partway through and
/// reverts to a standing/idle pose, dragging the arm along with it --
/// confirmed empirically, not documented anywhere in XGO's own docs.
/// Withholding Move/Turn/Arm for this long after triggering gives the
/// gait room to finish undisturbed (Claw is unaffected -- it doesn't
/// touch legs/body). Scoped to just these two action IDs, not every
/// action, since they're the only ones confirmed to have this
/// multi-second async-gait shape; a future instant
/// gesture wouldn't need it. A conservative guess, not a measured
/// duration -- E-Stop still overrides immediately regardless (it's
/// arbitrated before `dispatch_teleop_command` ever sees this state).
const STAND_ACTION_ID: u8 = 2;
const SIT_ACTION_ID: u8 = 12;
const ACTION_SETTLE_DURATION: Duration = Duration::from_millis(2500);

impl Session {
    async fn run(&mut self, socket: &UdpSocket, mut buf: Vec<u8>) -> Result<()> {
        let mut out = vec![0u8; MAX_DATAGRAM_SIZE];
        let mut ticker = tokio::time::interval(self.tick_interval);
        ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);

        self.flush(socket, &mut out).await?;

        loop {
            let timeout = self.conn.timeout().unwrap_or(Duration::from_millis(100));

            tokio::select! {
                recv = socket.recv_from(&mut buf) => {
                    let (len, from) = recv.context("socket recv_from failed")?;
                    let info = quiche::RecvInfo { from, to: self.local_addr };
                    match self.conn.recv(&mut buf[..len], info) {
                        Ok(_) => {}
                        Err(quiche::Error::Done) => {}
                        Err(e) => tracing::warn!(error = ?e, "conn.recv error"),
                    }
                    self.after_recv().await?;
                    self.flush(socket, &mut out).await?;
                }
                _ = tokio::time::sleep(timeout) => {
                    self.conn.on_timeout();
                    self.flush(socket, &mut out).await?;
                }
                _ = ticker.tick() => {
                    self.on_tick().await?;
                    self.flush(socket, &mut out).await?;
                }
                Some(event) = self.bridge.event_rx.recv() => {
                    self.on_bridge_event(event);
                }
                Some((nal_id, nal)) = recv_video(&mut self.video_rx) => {
                    // Chunked here, not in `channel_a::spawn_encoder`,
                    // specifically so a NAL superseded by a newer one
                    // before we get here is never chunked/sent at all --
                    // see that module's doc comment.
                    if self.phase == Phase::Operating {
                        for chunk in chunk_nal(nal_id, &nal) {
                            let _ = self.conn.dgram_send(&datagram::tag(datagram::DATAGRAM_TAG_CHANNEL_A, &chunk));
                        }
                        // Every other branch flushes right after queuing
                        // datagrams; this one didn't, so a video chunk
                        // just sat in quiche's send buffer until some
                        // unrelated event (next tick, next incoming
                        // packet, next connection timeout) happened to
                        // flush it -- up to one `tick_interval` of pure,
                        // avoidable added latency on every single frame.
                        self.flush(socket, &mut out).await?;
                    }
                }
            }

            if self.conn.is_closed() {
                tracing::info!("connection closed, trace_id={}", self.conn.trace_id());
                return Ok(());
            }
        }
    }

    async fn after_recv(&mut self) -> Result<()> {
        if self.conn.is_established() && !self.hello_sent {
            // The client's Finished and its stream-0 HELLO can legitimately
            // arrive in separate packets (e.g. a fast-path resumed handshake
            // that reaches `is_established()` a packet before the peer's own
            // stream-0 frame does) -- quiche doesn't recognize a
            // peer-initiated stream until it's seen a frame for it, so
            // `stream_send` here can transiently fail with
            // `InvalidStreamState`. That's not fatal to the session: leave
            // `hello_sent` false and retry on the next `after_recv` call,
            // once the client's own data has arrived.
            match self.send_hello() {
                Ok(()) => self.hello_sent = true,
                Err(e) => tracing::debug!(error = ?e, "HELLO send deferred, will retry"),
            }
        }

        // Drain readable streams regardless of phase -- HELLO/session data
        // arrives this way; gating on *processing* it (not receiving it)
        // happens per DESIGN.md §1.2.1/§1.3.1.
        let readable: Vec<u64> = self.conn.readable().collect();
        for stream_id in readable {
            self.on_stream_readable(stream_id)?;
        }

        // Drain Channel B / E-Stop datagrams.
        let mut dbuf = vec![0u8; MAX_DATAGRAM_SIZE];
        loop {
            match self.conn.dgram_recv(&mut dbuf) {
                Ok(len) => self.on_datagram(&dbuf[..len]),
                Err(quiche::Error::Done) => break,
                Err(e) => {
                    tracing::warn!(error = ?e, "dgram_recv error");
                    break;
                }
            }
        }
        Ok(())
    }

    fn send_hello(&mut self) -> Result<()> {
        let caps = local_hello_capabilities(self.task_class);
        let bytes = hello_handler::encode_hello(&caps);
        match self.conn.stream_send(0, &bytes, false) {
            Ok(_) => Ok(()),
            Err(quiche::Error::Done) => Ok(()),
            Err(e) => anyhow::bail!("stream_send(HELLO) failed: {e:?}"),
        }
    }

    fn on_stream_readable(&mut self, stream_id: u64) -> Result<()> {
        let mut sbuf = vec![0u8; 65535];
        loop {
            match self.conn.stream_recv(stream_id, &mut sbuf) {
                Ok((len, _fin)) => self.on_stream_data(stream_id, &sbuf[..len])?,
                Err(quiche::Error::Done) => break,
                Err(e) => {
                    tracing::warn!(error = ?e, stream_id, "stream_recv error");
                    break;
                }
            }
        }
        Ok(())
    }

    fn on_stream_data(&mut self, stream_id: u64, data: &[u8]) -> Result<()> {
        match stream_id {
            0 if self.phase == Phase::AwaitingHello => {
                let peer_caps = hello_handler::decode_hello(data)?;
                let local_caps = local_hello_capabilities(self.task_class);
                match hello::negotiate(&local_caps, &peer_caps) {
                    Ok(_negotiated) => {
                        tracing::info!("HELLO negotiated successfully");
                        self.phase = Phase::AwaitingSessionAccept;
                        self.send_session_describe()?;
                    }
                    Err(e) => {
                        tracing::error!(error = %e, "HELLO_INCOMPATIBLE");
                        let _ = self.conn.close(true, 0x1, b"HELLO_INCOMPATIBLE");
                    }
                }
            }
            1 if self.phase == Phase::AwaitingSessionAccept => {
                let accept = session_handler::decode_session_accept(data)?;
                tracing::info!(cached = accept.cached, "SESSION_ACCEPT received");
                self.phase = Phase::Operating;
            }
            ESTOP_STREAM_ID => {
                if let Some(d) = EstopDatagram::decode(data) {
                    if d.latched {
                        self.safety.trigger_explicit_estop();
                    } else {
                        self.safety.clear_explicit_estop(Instant::now());
                    }
                }
            }
            ACTION_TRIGGER_STREAM_ID if self.phase == Phase::Operating => {
                self.recorder.enqueue(
                    roboprotocol_recording::Category::ActionTriggerC,
                    roboprotocol_recording::Record {
                        capture_us: roboprotocol_core::timestamp::now_micros(),
                        control_source: roboprotocol_core::recording::CONTROL_SOURCE_SENTINEL,
                        payload: data.to_vec(),
                    },
                );
                let trigger = action_trigger_handler::decode_action_trigger(data)?;
                self.on_action_trigger(trigger);
            }
            CAMERA_CONTROL_STREAM_ID if self.phase == Phase::Operating => {
                // Reuses the ActionTriggerC recording category -- both are
                // discrete, reliably-delivered Channel C commands, and
                // adding a dedicated recording category (its own
                // subdirectory, retention accounting, replay-tool support)
                // for one more small message type isn't worth it yet.
                self.recorder.enqueue(
                    roboprotocol_recording::Category::ActionTriggerC,
                    roboprotocol_recording::Record {
                        capture_us: roboprotocol_core::timestamp::now_micros(),
                        control_source: roboprotocol_core::recording::CONTROL_SOURCE_SENTINEL,
                        payload: data.to_vec(),
                    },
                );
                let control = camera_control_handler::decode_camera_control(data)?;
                self.on_camera_control(control);
            }
            _ => tracing::debug!(stream_id, len = data.len(), "unhandled stream data"),
        }
        Ok(())
    }

    /// FR-1.8: fires a discrete one-shot trigger exactly once, deduped by
    /// its independent `trigger_seq` (not gated by Channel B staleness --
    /// this arrived over a reliable, ordered Channel C stream). Applies
    /// regardless of arbitrated control source the way E-Stop bypasses
    /// arbitration too, EXCEPT it still only fires under Full Teleoperation
    /// -- a canned gait triggered while E-Stopped or suspended would defeat
    /// the point of those states.
    fn on_action_trigger(&mut self, trigger: roboprotocol_core::action_trigger::ActionTrigger) {
        if !self.trigger_dedup.accept(trigger) {
            tracing::debug!(trigger_seq = trigger.trigger_seq, "duplicate/stale ActionTrigger ignored");
            return;
        }
        use roboprotocol_core::safety::ControlSource;
        if self.safety.tick(Instant::now()) != ControlSource::FullTeleoperation {
            tracing::debug!(action_id = trigger.action_id, "ActionTrigger dropped: not in Full Teleoperation");
            return;
        }
        let seq = self.next_seq();
        let _ = self.bridge.cmd_tx.send(BridgeCommand::Action { id: trigger.action_id, seq });
        if trigger.action_id == STAND_ACTION_ID || trigger.action_id == SIT_ACTION_ID {
            self.action_settle_until = Some(Instant::now() + ACTION_SETTLE_DURATION);
        }
        // See `sitting`'s doc comment -- sit needs to hold indefinitely,
        // not just through the settle window.
        if trigger.action_id == SIT_ACTION_ID {
            self.sitting = true;
        } else if trigger.action_id == STAND_ACTION_ID {
            self.sitting = false;
        }
    }

    /// Applies a `CameraControl` exactly once, deduped by its independent
    /// `control_seq` (same reasoning as `on_action_trigger`). Unlike
    /// `ActionTrigger`, this is **not** gated on `ControlSource` -- it
    /// doesn't move the robot's body at all, so there's no safety reason
    /// to withhold it during E-Stop/suspended/etc, the same way video
    /// streaming itself isn't safety-gated either.
    fn on_camera_control(&mut self, control: roboprotocol_core::camera_control::CameraControl) {
        if !self.camera_control_dedup.accept(control) {
            tracing::debug!(control_seq = control.control_seq, "duplicate/stale CameraControl ignored");
            return;
        }
        let Some(tx) = &self.camera_controls_tx else {
            tracing::warn!("CameraControl received but no camera attached this session, ignoring");
            return;
        };
        // Defense in depth: operator-console already clamps to these same
        // ranges before sending, but robot-edge shouldn't trust a peer's
        // clamping -- these are `libcamera-vid`'s real accepted ranges,
        // widened slightly isn't harmful but a wildly out-of-range value
        // (e.g. a corrupted/malicious payload) could otherwise make
        // `libcamera-vid` itself reject the args and fail to start.
        let controls = crate::video::capture::CameraControls {
            brightness: control.brightness.clamp(-1.0, 1.0),
            contrast: control.contrast.clamp(0.0, 2.0),
            ev: control.ev.clamp(-4.0, 4.0),
            shutter_us: control.shutter_us.clamp(0, 100_000),
        };
        tracing::info!(?controls, "applying live CameraControl");
        let _ = tx.send(controls);
    }

    fn send_session_describe(&mut self) -> Result<()> {
        if self.session_describe_sent {
            return Ok(());
        }
        let bytes = session_handler::encode_session_describe(&self.robot_id, &self.profile, &self.cameras);
        match self.conn.stream_send(1, &bytes, false) {
            Ok(_) => {
                self.session_describe_sent = true;
                Ok(())
            }
            Err(quiche::Error::Done) => Ok(()),
            Err(e) => anyhow::bail!("stream_send(SESSION_DESCRIBE) failed: {e:?}"),
        }
    }

    fn on_datagram(&mut self, data: &[u8]) {
        if data.first() == Some(&ESTOP_DATAGRAM_MAGIC) {
            if let Some(d) = EstopDatagram::decode(data) {
                if d.latched {
                    self.safety.trigger_explicit_estop();
                } else {
                    self.safety.clear_explicit_estop(Instant::now());
                }
                self.safety.on_channel_b_activity(None, Instant::now());
            }
            return;
        }
        if self.phase != Phase::Operating {
            return;
        }
        let Some((tag, payload)) = datagram::untag(data) else { return };
        if tag != datagram::DATAGRAM_TAG_CHANNEL_B {
            // robot-edge never receives Channel A (video) datagrams -- it
            // only sends them. Anything else is unexpected; drop it rather
            // than feeding untrusted bytes to the unverified flatbuffers
            // decoder below (see roboprotocol_core::datagram's docs).
            return;
        }
        match channel_b::decode_channel_b_frame(&payload) {
            Ok(frame) if frame.category == ChannelBCategory::Command => {
                self.safety.on_channel_b_activity(None, Instant::now());
                self.safety.deadman_held = true;
                self.safety.command_fresh = true;
                if let Some(cmd) = TeleopCommand::unpack(&frame.fields) {
                    let source = self.dispatch_teleop_command(&cmd);
                    // `capture_us` is this endpoint's own local receipt
                    // time, not `frame.timestamp` -- see
                    // `roboprotocol_core::recording`'s module doc for why.
                    // The operator's own embedded timestamp/seq stay
                    // recoverable from `payload` for cross-endpoint offset
                    // estimation later.
                    self.recorder.enqueue(
                        roboprotocol_recording::Category::ChannelBCommand,
                        roboprotocol_recording::Record {
                            capture_us: roboprotocol_core::timestamp::now_micros(),
                            control_source: roboprotocol_core::recording::control_source_byte(source),
                            payload: payload.to_vec(),
                        },
                    );
                }
            }
            Ok(_) => {}
            Err(e) => tracing::debug!(error = %e, "failed to decode Channel B datagram"),
        }
    }

    fn dispatch_teleop_command(&mut self, cmd: &TeleopCommand) -> roboprotocol_core::safety::ControlSource {
        let source = self.safety.tick(Instant::now());
        use roboprotocol_core::safety::ControlSource;
        match source {
            ControlSource::FullTeleoperation => {
                // Withhold Move/Turn/Arm while a stand/sit gait is still
                // settling (see ACTION_SETTLE_DURATION's doc comment) --
                // re-asserting these every tick fights the robot's own
                // in-progress async animation. Checked/cleared here rather
                // than in `on_tick` so it applies to exactly the commands
                // that need withholding, not the tick as a whole.
                let settling = self.action_settle_until.is_some_and(|until| Instant::now() < until);
                if self.action_settle_until.is_some() && !settling {
                    self.action_settle_until = None;
                }

                // While sitting, only a genuine (non-zero) move/turn
                // command counts as the operator asking to get up -- see
                // `sitting`'s doc comment for why continuing to send the
                // zero-velocity idle hold on its own stands the robot
                // back up.
                let is_idle = cmd.vx == 0.0 && cmd.vy == 0.0 && cmd.turn == 0.0;
                if self.sitting && !is_idle {
                    self.sitting = false;
                }
                let hold_sit = self.sitting && is_idle;

                if !settling && !hold_sit {
                    // Move and turn are continuous velocity-style inputs (like
                    // xgolib's own move_x/move_y/turn), sent every tick the
                    // operator holds them -- including when zero, since that's
                    // the explicit stop; xgolib's turn() only stops spinning
                    // when re-commanded with 0, it doesn't self-terminate.
                    // Discrete one-shot triggers (canned gaits) no longer ride
                    // here (FR-1.8) -- see `on_action_trigger`, fired
                    // independently off the ActionTrigger Channel C stream.
                    let move_seq = self.next_seq();
                    let _ = self.bridge.cmd_tx.send(BridgeCommand::Move { x: cmd.vx as f64, y: cmd.vy as f64, seq: move_seq });
                    let turn_seq = self.next_seq();
                    let _ = self.bridge.cmd_tx.send(BridgeCommand::Turn { step: cmd.turn as f64, seq: turn_seq });
                    // Roll/pitch/yaw attitude (xgolib's `attitude(axis, deg)`,
                    // body-pose IK with all feet planted) -- decoded off the
                    // wire since channel_b.rs's TeleopCommand carries it, but
                    // previously never forwarded to the bridge at all, so no
                    // attitude command ever reached xgolib regardless of what
                    // the operator console sent. Sent unconditionally per axis
                    // every tick like Move/Turn above (0.0 is a real "hold
                    // level" target, not "nothing to do").
                    let roll_seq = self.next_seq();
                    let _ =
                        self.bridge.cmd_tx.send(BridgeCommand::Attitude { axis: 'r', data: cmd.attitude_r as f64, seq: roll_seq });
                    let pitch_seq = self.next_seq();
                    let _ = self.bridge.cmd_tx.send(BridgeCommand::Attitude {
                        axis: 'p',
                        data: cmd.attitude_p as f64,
                        seq: pitch_seq,
                    });
                    let yaw_seq = self.next_seq();
                    let _ =
                        self.bridge.cmd_tx.send(BridgeCommand::Attitude { axis: 'y', data: cmd.attitude_y as f64, seq: yaw_seq });
                }
                if !settling {
                    // Arm/claw are position-commanded (not velocity), but sent
                    // unconditionally too -- xgolib holds wherever it's last
                    // told, so a fresh session/reconnect needs an explicit
                    // position to establish the truth rather than assuming
                    // whatever the arm physically happens to be at.
                    // Unaffected by `sitting` -- no reported issue with the
                    // arm holding position while seated.
                    let arm_seq = self.next_seq();
                    let _ = self.bridge.cmd_tx.send(BridgeCommand::Arm { x: cmd.arm_x as f64, z: cmd.arm_z as f64, seq: arm_seq });
                }
                let claw_seq = self.next_seq();
                let _ = self.bridge.cmd_tx.send(BridgeCommand::Claw { pos: cmd.claw, seq: claw_seq });
            }
            ControlSource::EStop | ControlSource::EmergencySafeParking | ControlSource::ActiveImpedanceHold => {
                let seq = self.next_seq();
                let _ = self.bridge.cmd_tx.send(BridgeCommand::Stop { seq });
            }
            ControlSource::SemiAutonomous => {} // v0: no autonomy goals exist yet
        }
        source
    }

    fn next_seq(&mut self) -> u64 {
        self.channel_b_seq += 1;
        self.channel_b_seq
    }

    async fn on_tick(&mut self) -> Result<()> {
        let now = Instant::now();
        self.tick_count += 1;
        let source = self.safety.tick(now);
        let latched = self.safety.is_estopped();

        if !self.estop_stream_primed && self.conn.is_established() {
            let _ = self.conn.stream_priority(ESTOP_STREAM_ID, ESTOP_STREAM_URGENCY, false);
            self.estop_stream_primed = true;
        }

        // Redundant E-Stop datagram, independent of phase (§1.2.1 exception).
        self.estop_seq += 1;
        let d = EstopDatagram { latched, seq: self.estop_seq };
        let _ = self.conn.dgram_send(&d.encode());

        // DESIGN.md §2.3: also carried on a dedicated high-priority
        // reliable stream, not just the redundant datagram -- only on
        // state changes, since the stream is ordered/reliable already.
        if self.estop_stream_primed && self.last_reported_estop_latched != Some(latched) {
            if latched {
                tracing::warn!(blackout_ms = ?self.safety.watchdog.threshold_ms(), "E-Stop latched");
            } else {
                tracing::info!("E-Stop cleared");
            }
            if self.conn.stream_send(ESTOP_STREAM_ID, &d.encode(), false).is_ok() {
                self.last_reported_estop_latched = Some(latched);
            }
        }

        // Keep the bridge's own local watchdog fed even with no fresh
        // Channel B command this tick (see xgo_bridge.py's watchdog design).
        let heartbeat_seq = self.next_seq();
        let _ = self.bridge.cmd_tx.send(BridgeCommand::Heartbeat { seq: heartbeat_seq });

        // The bridge subprocess (xgo_bridge.py) latches its own, separate
        // `estopped` flag on `BridgeCommand::Estop` and refuses move/turn/
        // action/etc. commands until it explicitly receives `EstopClear` --
        // it does NOT automatically un-latch just because robot-edge's own
        // safety state clears (see xgo_bridge.py's dispatch guard). `Estop`
        // is still sent every tick while latched, for redundancy; `EstopClear`
        // only needs to be sent once, on the transition, since the bridge's
        // clear is idempotent and there's no redundancy requirement on it.
        if source == roboprotocol_core::safety::ControlSource::EStop {
            self.bridge_estop_active = true;
            let estop_seq = self.next_seq();
            let _ = self.bridge.cmd_tx.send(BridgeCommand::Estop { seq: estop_seq });
        } else if self.bridge_estop_active {
            self.bridge_estop_active = false;
            let clear_seq = self.next_seq();
            let _ = self.bridge.cmd_tx.send(BridgeCommand::EstopClear { seq: clear_seq });
        }

        if self.tick_count % TELEMETRY_QUERY_EVERY_N_TICKS == 0 {
            let query_seq = self.next_seq();
            let _ = self.bridge.cmd_tx.send(BridgeCommand::Query { seq: query_seq });
        }

        if self.phase == Phase::Operating {
            self.send_telemetry(now, source)?;
        }
        Ok(())
    }

    fn send_telemetry(&mut self, now: Instant, source: roboprotocol_core::safety::ControlSource) -> Result<()> {
        let _ = now;
        // Sent every tick at the current cadence (not throttled like the
        // Query request itself) so the operator console's RTT/status
        // display stays live between real telemetry updates; the fields
        // just don't change until the next cached value lands.
        let fields = self.latest_telemetry.as_ref().map(|t| t.pack()).unwrap_or_default();
        let capture_us = roboprotocol_core::timestamp::now_micros();
        let frame = ChannelBFrameData {
            timestamp: capture_us,
            seq: self.next_seq(),
            tick_id: 0,
            category: ChannelBCategory::Telemetry,
            region_id: ALL_REGIONS,
            fields,
        };
        let bytes = channel_b::encode_channel_b_frame(&frame);
        // Locally originated, so `capture_us` here equals `frame.timestamp`
        // above by construction -- see `roboprotocol_core::recording`'s
        // module doc for why `capture_us` is always local rather than
        // special-cased per category.
        self.recorder.enqueue(
            roboprotocol_recording::Category::ChannelBTelemetry,
            roboprotocol_recording::Record { capture_us, control_source: roboprotocol_core::recording::control_source_byte(source), payload: bytes.clone() },
        );
        let _ = self.conn.dgram_send(&datagram::tag(datagram::DATAGRAM_TAG_CHANNEL_B, &bytes));
        Ok(())
    }

    fn on_bridge_event(&mut self, event: SupervisorEvent) {
        match event {
            SupervisorEvent::FromBridge(crate::bridge::BridgeEvent::Telemetry { motors, battery, roll, pitch, yaw, .. }) => {
                self.latest_telemetry = Some(channel_b::TelemetryData {
                    battery,
                    roll: roll as f32,
                    pitch: pitch as f32,
                    yaw: yaw as f32,
                    motors: motors.into_iter().map(|m| m as f32).collect(),
                });
            }
            SupervisorEvent::FromBridge(ev) => tracing::debug!(?ev, "bridge event"),
            SupervisorEvent::ProcessDied { attempt, will_retry } => {
                tracing::warn!(attempt, will_retry, "xgo_bridge.py died, forcing E-Stop");
                self.safety.trigger_explicit_estop();
            }
            SupervisorEvent::GaveUp => {
                tracing::error!("xgo_bridge.py restart attempts exhausted, latched E-Stop, manual intervention required");
                self.safety.trigger_explicit_estop();
            }
        }
    }

    async fn flush(&mut self, socket: &UdpSocket, out: &mut [u8]) -> Result<()> {
        loop {
            match self.conn.send(out) {
                Ok((len, send_info)) => {
                    if let Err(e) = socket.send_to(&out[..len], send_info.to).await {
                        tracing::warn!(error = %e, "socket send_to failed");
                        break;
                    }
                }
                Err(quiche::Error::Done) => break,
                Err(e) => {
                    tracing::warn!(error = ?e, "conn.send error");
                    break;
                }
            }
        }
        Ok(())
    }
}

async fn recv_video(rx: &mut Option<VideoRx>) -> Option<(u32, Vec<u8>)> {
    match rx {
        Some(rx) => rx.recv().await,
        None => std::future::pending().await,
    }
}

fn local_hello_capabilities(task_class: TaskClass) -> HelloCapabilities {
    let task_class_bit = match task_class {
        TaskClass::B => task_class_bits::CLASS_B,
        TaskClass::C => task_class_bits::CLASS_C,
        TaskClass::D => task_class_bits::CLASS_D,
        TaskClass::E => task_class_bits::CLASS_E,
    };
    HelloCapabilities {
        protocol_version: ProtocolVersion::new(1, 0),
        // No TDPA/FlexFEC/ROS2/connection-migration support in v0 -- see
        // the plan's explicit deferral list. This hardware also has no
        // haptic sensing at all, so TDPA is N/A, not just unimplemented.
        capability_bitmask: 0,
        supported_task_classes: task_class_bit,
        supported_quantization_tiers: quantization_tier_bits::STANDARD,
        max_control_rate_hz: 50,
    }
}

fn build_quiche_config(args: &ServerArgs) -> Result<quiche::Config> {
    let mut config = quiche::Config::new(quiche::PROTOCOL_VERSION).context("quiche::Config::new")?;
    config.set_application_protos(&[b"roboprotocol/1"]).context("set_application_protos")?;
    config.set_max_idle_timeout(10_000);
    config.set_max_recv_udp_payload_size(MAX_DATAGRAM_SIZE);
    config.set_max_send_udp_payload_size(MAX_DATAGRAM_SIZE);
    config.set_initial_max_data(10_000_000);
    config.set_initial_max_stream_data_bidi_local(1_000_000);
    config.set_initial_max_stream_data_bidi_remote(1_000_000);
    config.set_initial_max_streams_bidi(16);
    config.set_initial_max_streams_uni(16);
    config.enable_dgram(true, 4096, 4096);
    // 0-RTT resumption -- accepts an operator's early-arriving HELLO on
    // reconnect. See operator-console's matching config for the full
    // rationale; the read side here already drains streams regardless of
    // phase, so no other server-side change is needed to accept it.
    config.enable_early_data();
    config.verify_peer(true);
    config.load_cert_chain_from_pem_file(&args.cert_path).context("loading robot cert chain")?;
    config.load_priv_key_from_pem_file(&args.key_path).context("loading robot private key")?;
    config.load_verify_locations_from_file(&args.ca_path).context("loading CA for peer verification")?;
    Ok(config)
}
