//! operator-console's QUIC client: connection setup and the main driving loop.
//! Mirrors robot-edge's `quic_server.rs` structure; see that file's module
//! docs for the shared v0 simplifications (single connection, no Retry,
//! no connection migration).

use std::net::SocketAddr;
use std::time::Duration;

use anyhow::{Context, Result};
use rand::RngCore;
use roboprotocol_core::action_trigger::{ActionTrigger, ACTION_TRIGGER_STREAM_ID};
use roboprotocol_core::camera_control::{CameraControl, CAMERA_CONTROL_STREAM_ID};
use roboprotocol_core::datagram;
use roboprotocol_core::estop::{EstopDatagram, ESTOP_DATAGRAM_MAGIC, ESTOP_STREAM_ID};
use roboprotocol_core::hello::{self, quantization_tier_bits, task_class_bits, HelloCapabilities, ProtocolVersion};
use roboprotocol_core::safety::TaskClass;
use roboprotocol_core::timestamp;
use roboprotocol_core::video::nal_is_critical;
use tokio::net::UdpSocket;
use tokio::process::Child;
use tokio::time::MissedTickBehavior;

use crate::action_trigger_handler;
use crate::camera_control_handler;
use crate::channel_b::{self, ChannelBCategory, ChannelBFrameData, TeleopCommand, ALL_REGIONS};
use crate::hello_handler;
use crate::gamepad::GamepadReader;
use crate::input::{InputReader, TeleopInput};
use crate::session_cache;
use crate::session_handler::{self, SessionDescribeInfo};
use crate::ui::{ConnPhase, Console, HudState};
use crate::video::channel_a::ChannelAReceiver;
use crate::video::native_playback::{self, NativeVideoTx};
use crate::video::playback::{self, VideoTx};

/// Which video path `--video` drives: the proven `ffplay` subprocess, or
/// the in-process `openh264` decode + `minifb` display path -- see
/// `video/native_playback.rs`'s module doc for why the latter exists.
/// Mutually exclusive; exactly one of `Client`'s `video_tx`/
/// `native_video_tx` fields is ever `Some` for a given session.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VideoBackend {
    Ffplay,
    Native,
}

const MAX_DATAGRAM_SIZE: usize = 1452;

// Arm/claw ranges per xgolib (see crates/robot-edge/src/xgo_profile.rs's
// arm-region comment): arm(x, z) in mm, claw(pos) 0-255. `ARM_CLAW_NEUTRAL`
// is a genuine midpoint, not a guess at open vs. closed -- that polarity is
// unverified, see input.rs's u/o key docs.
const ARM_X_RANGE: std::ops::RangeInclusive<i16> = -80..=155;
const ARM_Z_RANGE: std::ops::RangeInclusive<i16> = -95..=155;
const ARM_CLAW_NEUTRAL: u8 = 128;

// Whole-body attitude ranges, per xgolib's `ATTITUDE_LIMIT` for the
// "xgolite" hardware version (`changePara` in xgolib's `__init__.py`:
// Roll/Pitch/Yaw = [20, 10, 12] deg) -- confirmed by reading the vendor
// source on the real Pi (xgo_bridge.py's default `--xgo-version xgolite`),
// not derived from geometry. Don't assume xgomini shares these (its pitch
// is +/-22 deg, not +/-10).
const ROLL_RANGE: std::ops::RangeInclusive<f32> = -20.0..=20.0;
const PITCH_RANGE: std::ops::RangeInclusive<f32> = -10.0..=10.0;
const YAW_RANGE: std::ops::RangeInclusive<f32> = -12.0..=12.0;

// Camera image-quality control ranges. Brightness/contrast/ev mirror
// `libcamera-vid`'s own accepted ranges (contrast/ev clamped to a
// practically-useful sub-range of what the tool technically accepts, since
// its full range goes well past where the image is usable); shutter is a
// manual exposure time in microseconds, 0 = auto (see camera_control.rs's
// module doc for why this restarts `rpicam-vid` rather than being a live
// in-process tweak). `CameraControls::default()` in robot-edge's
// `capture.rs` is this module's neutral starting point (0.0/1.0/0.0/0),
// mirrored here so a fresh session's HUD reads the same as what's actually
// running before the first nudge.
const BRIGHTNESS_RANGE: std::ops::RangeInclusive<f32> = -1.0..=1.0;
const CONTRAST_RANGE: std::ops::RangeInclusive<f32> = 0.0..=2.0;
const EV_RANGE: std::ops::RangeInclusive<f32> = -4.0..=4.0;
const SHUTTER_RANGE_US: std::ops::RangeInclusive<u32> = 0..=100_000;

/// Held camera-control state, mirroring `last_command`'s "position, not
/// velocity" treatment of arm/attitude -- these are "set and hold" values
/// sent as a discrete `CameraControl` RPC (Channel C) on change, not a
/// per-tick Channel B field. Default matches `libcamera-vid`'s own
/// defaults (and robot-edge's `CameraControls::default()`).
#[derive(Debug, Clone, Copy)]
struct CameraControlState {
    brightness: f32,
    contrast: f32,
    ev: f32,
    shutter_us: u32,
}

impl Default for CameraControlState {
    fn default() -> Self {
        Self { brightness: 0.0, contrast: 1.0, ev: 0.0, shutter_us: 0 }
    }
}

/// Front-screen self-collision keep-out for this specific XGO-Lite V2
/// unit. `ARM_X_RANGE`/`ARM_Z_RANGE` are the SDK's raw joint-space limits
/// and know nothing about the robot's own body -- confirmed on real
/// hardware 2026-08-21 that the arm physically touches the robot's front
/// screen at (x=70, z=0) and again at (x=30, z=35). Modeled as a straight
/// line through those two measured points (the only calibration this
/// robot has had), which is only valid *interpolated* within that
/// z=[0, 35] window -- NOT extrapolated further. An earlier version of
/// this let z run unclamped into the formula and, confirmed on real
/// hardware, that extrapolated the "safe" x past `ARM_X_RANGE`'s own
/// declared max (155) once z got very negative (z=-95 computed x=179),
/// which fought every attempt to move the arm back down and pinned it
/// against its hardware limit. Clamping `arm_z` into the tested window
/// first, and the caller clamping the result back into `ARM_X_RANGE`,
/// both guard against that now.
fn min_safe_arm_x(arm_z: i16) -> i16 {
    let z = arm_z.clamp(0, 35);
    (70.0 - (40.0 / 35.0) * z as f32).round() as i16
}

/// What `'r'` (`TeleopInput::ToggleRecording`) turns on/off -- video, the
/// operator's own commands, telemetry (motor angles + roll/pitch/yaw),
/// and key presses. `Haptic`/`ActionTriggerC` have no natural on/off
/// point of their own during a session, so they're `--record-extra`-only
/// (see `main.rs`'s `Cli::recorder_config`), not part of this toggle.
const DEFAULT_RECORD_CATEGORIES: [roboprotocol_recording::Category; 4] = [
    roboprotocol_recording::Category::VideoA,
    roboprotocol_recording::Category::ChannelBCommand,
    roboprotocol_recording::Category::ChannelBTelemetry,
    roboprotocol_recording::Category::KeyPress,
];

pub struct ClientArgs {
    pub connect: SocketAddr,
    pub bind: SocketAddr,
    pub server_name: String,
    pub cert_path: String,
    pub key_path: String,
    pub ca_path: String,
    pub task_class: TaskClass,
    pub tick_hz: u32,
    pub ffplay_bin: String,
    pub enable_video: bool,
    pub video_backend: VideoBackend,
    /// Burn brightness/contrast/EV/shutter into the `ffplay` window itself
    /// via its `drawtext` filter -- see `playback::spawn_playback`'s doc
    /// for the `libfreetype`/`libfontconfig` dependency this needs.
    /// `Ffplay`-only; `main.rs` rejects this combined with `Native`.
    pub video_overlay: bool,
    /// Skip keyboard/raw-terminal-mode setup entirely -- for headless runs
    /// (CI, the localhost integration test) where there's no real TTY.
    pub headless: bool,
    /// Fire one scripted action command (e.g. 2 = stand, 12 = sit) on the
    /// first tick after reaching Operating, then continue normally --
    /// for non-interactive validation without a live keyboard.
    pub demo_action: Option<u8>,
    /// Auto-zero `vx`/`vy`/`turn` if no movement key event refreshes them
    /// within this many ms -- see `Client::on_tick`'s staleness check.
    pub move_stale_ms: u64,
    pub recording: roboprotocol_recording::RecorderConfig,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    AwaitingHello,
    AwaitingSessionDescribe,
    Operating,
}

/// How long a *resumed* (0-RTT ticket offered) handshake gets to show signs
/// of life before we give up on it and retry fresh. Deliberately much
/// shorter than the connection's own `max_idle_timeout` (10s): a stale
/// ticket (e.g. robot-edge restarted with fresh certs since it was cached)
/// doesn't just fail to help, it can make the *whole* connection attempt
/// unrecoverable -- verified empirically that a rejected 0-RTT packet can
/// poison the coalesced Initial datagram enough that quiche closes the
/// connection outright rather than gracefully falling back on its own.
/// Waiting out the full idle timeout on an attempt that was never going to
/// work is exactly the kind of silent-feeling stall this whole feature was
/// supposed to avoid.
const RESUME_ESTABLISH_TIMEOUT: Duration = Duration::from_millis(1500);

/// Backoff after a reconnect attempt errors out immediately (as opposed to
/// timing out waiting on an unreachable peer) -- see
/// `Client::reconnect_until_success_or_quit`.
const RECONNECT_ERROR_BACKOFF: Duration = Duration::from_millis(500);

/// One connection attempt: build config, generate a fresh SCID, connect,
/// and optionally offer `session` for 0-RTT resumption. Returns the
/// connection and whether a session was actually offered.
fn fresh_connect(args: &ClientArgs, local_addr: SocketAddr, session: Option<&[u8]>) -> Result<(quiche::Connection, bool)> {
    let mut config = build_quiche_config(args)?;
    let mut scid_bytes = [0u8; 16];
    rand::thread_rng().fill_bytes(&mut scid_bytes);
    let scid = quiche::ConnectionId::from_vec(scid_bytes.to_vec());
    let server_name = (!args.server_name.is_empty()).then_some(args.server_name.as_str());
    let mut conn = quiche::connect(server_name, &scid, local_addr, args.connect, &mut config)
        .map_err(|e| anyhow::anyhow!("quiche::connect failed: {e:?}"))?;
    tracing::info!(peer = %args.connect, trace_id = conn.trace_id(), "connecting");

    // 0-RTT: must be called immediately after creating the connection,
    // before any packet is sent or received (quiche::Connection::set_session's
    // own doc comment).
    let mut offered = false;
    if let Some(ticket) = session {
        match conn.set_session(ticket) {
            Ok(()) => {
                tracing::info!("offering cached session for 0-RTT resumption");
                offered = true;
            }
            Err(e) => tracing::debug!(error = ?e, "cached session ticket rejected locally, doing a full handshake"),
        }
    }
    Ok((conn, offered))
}

/// Connects, offering a cached 0-RTT session ticket if one exists. If that
/// resumption attempt hasn't established (or died outright) within
/// `RESUME_ESTABLISH_TIMEOUT`, evicts the ticket and retries once with a
/// completely fresh connection -- bounded to this single startup attempt,
/// not a general reconnect-after-drop loop (a connection that's actually
/// become established and later drops is a different, deliberately
/// out-of-scope problem -- see `draw_disconnected`'s doc comment instead).
async fn connect_with_resume_fallback(args: &ClientArgs, socket: &UdpSocket, local_addr: SocketAddr) -> Result<quiche::Connection> {
    let cached = session_cache::load_ticket(args.connect);
    let (mut conn, offered_ticket) = fresh_connect(args, local_addr, cached.as_deref())?;
    flush_once(&mut conn, socket).await?;

    if !offered_ticket {
        return Ok(conn);
    }

    let mut buf = vec![0u8; 65535];
    let deadline = tokio::time::sleep(RESUME_ESTABLISH_TIMEOUT);
    tokio::pin!(deadline);
    loop {
        tokio::select! {
            recv = socket.recv_from(&mut buf) => {
                let (len, from) = recv.context("socket recv_from failed")?;
                let info = quiche::RecvInfo { from, to: local_addr };
                let _ = conn.recv(&mut buf[..len], info);
                // Deliberately skip flushing here once established: the
                // caller (`run`) sends the client's HELLO immediately after
                // this returns, and flushing now would ack/Finish in one
                // packet and the HELLO in a separate one right behind it.
                // robot-edge's `after_recv` tolerates that split today (it
                // retries a transiently-failed HELLO send), but there's no
                // reason to pay the extra datagram -- let the caller's
                // `after_recv` + single `flush_once` carry both together.
                if conn.is_established() || conn.is_closed() {
                    break;
                }
                flush_once(&mut conn, socket).await?;
            }
            _ = &mut deadline => break,
        }
    }

    if conn.is_established() {
        return Ok(conn);
    }

    tracing::warn!("resumed handshake didn't complete in time, evicting cached session and retrying fresh");
    session_cache::evict_ticket(args.connect);
    let (mut conn, _) = fresh_connect(args, local_addr, None)?;
    flush_once(&mut conn, socket).await?;
    Ok(conn)
}

pub async fn run(args: ClientArgs) -> Result<()> {
    let socket = UdpSocket::bind(args.bind).await.context("binding UDP socket")?;
    let local_addr = socket.local_addr()?;

    let conn = connect_with_resume_fallback(&args, &socket, local_addr).await?;

    let (video_child, video_tx, native_video_tx, video_overlay) = if args.enable_video {
        match args.video_backend {
            VideoBackend::Ffplay => {
                let (child, tx, overlay) = playback::spawn_playback(&args.ffplay_bin, args.video_overlay)?;
                (Some(child), Some(tx), None, overlay)
            }
            VideoBackend::Native => (None, None, Some(native_playback::spawn_native_playback()), None),
        }
    } else {
        (None, None, None, None)
    };

    let recorder = roboprotocol_recording::Recorder::start(args.recording.clone())?;

    let mut client = Client {
        conn,
        peer: args.connect,
        local_addr,
        task_class: args.task_class,
        phase: Phase::AwaitingHello,
        session_info: None,
        console: Console::init(args.headless),
        input: (!args.headless).then(InputReader::new),
        // `GamepadReader::new()` itself returns `Option` (gamepad support
        // is optional even when interactive -- see its doc), hence the
        // `.flatten()` on top of headless's own `Option`.
        gamepad: (!args.headless).then(GamepadReader::new).flatten(),
        video_rx: ChannelAReceiver::new(),
        video_tx,
        native_video_tx,
        _video_child: video_child,
        video_overlay,
        channel_b_seq: 0,
        estop_seq: 0,
        next_trigger_seq: 0,
        next_camera_control_seq: 0,
        camera_controls: CameraControlState::default(),
        tick_interval: Duration::from_secs_f64(1.0 / args.tick_hz.max(1) as f64),
        last_command: TeleopCommand {
            vx: 0.0,
            vy: 0.0,
            turn: 0.0,
            attitude_r: 0.0,
            attitude_p: 0.0,
            attitude_y: 0.0,
            arm_x: 0,
            arm_z: 0,
            claw: ARM_CLAW_NEUTRAL,
        },
        estopped: false,
        hud: HudState::new(args.connect, args.tick_hz),
        last_send_ts: 0,
        demo_action: args.demo_action,
        last_move_input_at: None,
        move_stale: Duration::from_millis(args.move_stale_ms),
        last_saved_session: None,
        args,
        recorder,
    };

    // `connect_with_resume_fallback` can leave `conn` already established
    // (0-RTT resumption succeeding, or the bounded fallback's own recv loop
    // completing the handshake) before `Client` even exists -- `after_recv`
    // is what notices "established, send HELLO now" today, but it's normally
    // only invoked by the main loop in response to a *new* incoming packet.
    // Without this, an already-established connection with nothing left to
    // arrive deadlocks: HELLO is client-initiated, so the server has
    // nothing to proactively send until it hears it, and the client never
    // notices it's time to send it. One explicit check before the loop
    // starts covers the case whether or not anything is buffered.
    client.after_recv();
    flush_once(&mut client.conn, &socket).await?;
    // Seeds the overlay file with the real default values (rather than
    // leaving it blank) before the operator's first camera-control nudge.
    client.sync_camera_hud_and_overlay();

    client.run(socket).await
}

struct Client {
    conn: quiche::Connection,
    peer: SocketAddr,
    local_addr: SocketAddr,
    task_class: TaskClass,
    phase: Phase,
    session_info: Option<SessionDescribeInfo>,
    input: Option<InputReader>,
    /// `None` under `--headless` (same as `input`) or if the platform
    /// gamepad backend couldn't initialize -- see `GamepadReader::new`.
    gamepad: Option<GamepadReader>,
    video_rx: ChannelAReceiver,
    /// `watch`, not a queue -- see `playback.rs`'s module doc for why a
    /// queue in front of `ffplay` is exactly the unbounded-video-lag bug
    /// this replaced.
    video_tx: Option<VideoTx>,
    /// See `VideoBackend` -- mutually exclusive with `video_tx`.
    native_video_tx: Option<NativeVideoTx>,
    _video_child: Option<Child>,
    /// `None` unless both `--video` and `--video-overlay` were passed --
    /// see `playback::spawn_playback`'s doc for why this is opt-in.
    video_overlay: Option<playback::OverlayHandle>,
    channel_b_seq: u64,
    estop_seq: u64,
    /// FR-1.8's independent monotonic identity for ActionTrigger -- not
    /// the same sequence space as `channel_b_seq`.
    next_trigger_seq: u64,
    /// Independent monotonic identity for CameraControl -- not the same
    /// sequence space as `next_trigger_seq` or `channel_b_seq`.
    next_camera_control_seq: u64,
    camera_controls: CameraControlState,
    tick_interval: Duration,
    last_command: TeleopCommand,
    estopped: bool,
    hud: HudState,
    console: Console,
    last_send_ts: u64,
    /// One-shot scripted action for non-interactive validation (e.g. CI,
    /// headless hardware smoke tests) -- see `ClientArgs::demo_action`.
    demo_action: Option<u8>,
    /// Wall-clock time of the last Move/Turn key event -- see
    /// `on_tick`'s staleness check, which zeros `last_command`'s vx/vy/turn
    /// once this is older than `move_stale`. `None` means no movement key
    /// has ever been pressed (nothing to go stale yet).
    last_move_input_at: Option<std::time::Instant>,
    move_stale: Duration,
    /// Last TLS session ticket written to `.session-cache/` -- lets
    /// `after_recv` skip redundant disk writes when `conn.session()`
    /// hasn't actually changed since the last check.
    last_saved_session: Option<Vec<u8>>,
    /// Kept around (rather than just the pieces used at startup) so
    /// `reconnect_until_success_or_quit` can call `connect_with_resume_fallback`
    /// again with the exact same connection parameters after a drop.
    args: ClientArgs,
    /// A plain field, not `Option` -- persists unchanged across
    /// `reconnect_until_success_or_quit` (which only ever replaces
    /// `self.conn`), and is a legitimate, fully inert no-op recorder when
    /// `--record-dir` was never given (see `Cli::recorder_config` in
    /// `main.rs`).
    recorder: roboprotocol_recording::Recorder,
}

impl Client {
    async fn run(&mut self, socket: UdpSocket) -> Result<()> {
        let mut buf = vec![0u8; 65535];
        let mut ticker = tokio::time::interval(self.tick_interval);
        ticker.set_missed_tick_behavior(MissedTickBehavior::Delay);

        loop {
            let timeout = self.conn.timeout().unwrap_or(Duration::from_millis(100));

            tokio::select! {
                recv = socket.recv_from(&mut buf) => {
                    let (len, from) = recv.context("socket recv_from failed")?;
                    let info = quiche::RecvInfo { from, to: self.local_addr };
                    match self.conn.recv(&mut buf[..len], info) {
                        Ok(_) | Err(quiche::Error::Done) => {}
                        Err(e) => tracing::warn!(error = ?e, "conn.recv error"),
                    }
                    self.after_recv();
                    flush_once(&mut self.conn, &socket).await?;
                }
                _ = tokio::time::sleep(timeout) => {
                    self.conn.on_timeout();
                    flush_once(&mut self.conn, &socket).await?;
                }
                _ = ticker.tick() => {
                    self.on_tick();
                    flush_once(&mut self.conn, &socket).await?;
                    self.console.render(&self.hud);
                }
                Some(input) = recv_input(&mut self.input) => {
                    if self.dispatch_teleop_input(input, &socket).await? {
                        return Ok(());
                    }
                }
                Some(input) = recv_gamepad(&mut self.gamepad) => {
                    if self.dispatch_teleop_input(input, &socket).await? {
                        return Ok(());
                    }
                }
            }

            if self.conn.is_closed() {
                self.hud.phase = ConnPhase::Closed;
                self.hud.disconnected_at = Some(std::time::Instant::now());
                self.hud.reconnect_attempts = 0;
                self.console.render(&self.hud);
                tracing::warn!("connection closed/lost");
                if self.input.is_none() {
                    // Headless (e.g. scripts/smoke_test.sh): no human is
                    // watching a TUI for this, and automation expects the
                    // process to actually exit -- keep that behavior, no
                    // reconnect loop.
                    return Ok(());
                }
                if !self.reconnect_until_success_or_quit(&socket).await? {
                    return Ok(());
                }
                // Reconnected -- fall through and keep driving the same
                // loop against the new `self.conn`.
            }
        }
    }

    /// Entered once the connection is gone for good. Retries indefinitely,
    /// offering the last-saved 0-RTT session ticket on every attempt just
    /// like the startup path (see `connect_with_resume_fallback`), until
    /// either a new connection is established or the operator presses 'q'.
    /// Returns `Ok(true)` on the former, `Ok(false)` on the latter -- there's
    /// no other way out: unlike the startup fallback's bounded wait, this
    /// keeps going across an arbitrarily long drop on purpose, since only
    /// the human watching the screen knows when it's time to give up.
    async fn reconnect_until_success_or_quit(&mut self, socket: &UdpSocket) -> Result<bool> {
        loop {
            self.hud.reconnect_attempts += 1;
            self.console.render(&self.hud);

            // Scoped so the `attempt` future -- and the borrow of
            // `self.args` it holds -- is dropped before `outcome` is
            // matched below, which needs `&mut self` again.
            let outcome = {
                let attempt = connect_with_resume_fallback(&self.args, socket, self.local_addr);
                tokio::pin!(attempt);
                tokio::select! {
                    result = &mut attempt => Some(result),
                    input = recv_input(&mut self.input) => {
                        if matches!(input, Some(TeleopInput::Quit) | None) {
                            return Ok(false);
                        }
                        None // nothing left to control while disconnected; keep retrying
                    }
                    // Drain-only: the gamepad never emits `Quit` (see
                    // `gamepad.rs`'s doc), so there's nothing to act on here
                    // -- this just keeps its unbounded channel from piling
                    // up held-stick refreshes for the length of the outage.
                    _ = recv_gamepad(&mut self.gamepad) => None,
                }
            };

            match outcome {
                Some(Ok(conn)) => {
                    self.conn = conn;
                    self.phase = Phase::AwaitingHello;
                    self.session_info = None;
                    self.hud.phase = ConnPhase::AwaitingHello;
                    self.hud.robot_id = None;
                    self.hud.dof_count = None;
                    self.hud.camera_shape = None;
                    self.hud.disconnected_at = None;
                    tracing::info!("reconnected");
                    // Same deadlock hazard as the startup path in `run` --
                    // see its comment for why this call is needed here too.
                    self.after_recv();
                    flush_once(&mut self.conn, socket).await?;
                    return Ok(true);
                }
                Some(Err(e)) => {
                    tracing::debug!(error = ?e, "reconnect attempt failed, retrying");
                    // Only guards against a tight loop on a persistent
                    // local error (e.g. config/socket) -- the "peer just
                    // isn't there" case is already paced by
                    // `connect_with_resume_fallback`'s own bounded wait
                    // and the resulting connection's idle timeout.
                    tokio::select! {
                        _ = tokio::time::sleep(RECONNECT_ERROR_BACKOFF) => {}
                        input = recv_input(&mut self.input) => {
                            if matches!(input, Some(TeleopInput::Quit) | None) {
                                return Ok(false);
                            }
                        }
                        _ = recv_gamepad(&mut self.gamepad) => {} // drain-only, see above
                    }
                }
                None => {} // input branch already handled above; loop back
            }
        }
    }

    fn after_recv(&mut self) {
        // Only the HELLO stream (Channel C stream 0) is ever sent this
        // early -- see build_quiche_config's enable_early_data() comment
        // for why that's the full scope of what 0-RTT covers here.
        if (self.conn.is_established() || self.conn.is_in_early_data()) && self.phase == Phase::AwaitingHello {
            self.send_hello();
        }

        let readable: Vec<u64> = self.conn.readable().collect();
        for stream_id in readable {
            self.on_stream_readable(stream_id);
        }

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

        // Cheap: a memory compare plus an occasional small file write when
        // the TLS session ticket actually changes (typically once, shortly
        // after the handshake completes). Checked here rather than only at
        // a clean-shutdown hook since this process exits all sorts of ways.
        if let Some(session) = self.conn.session() {
            if self.last_saved_session.as_deref() != Some(session) {
                session_cache::save_ticket(self.peer, session);
                self.last_saved_session = Some(session.to_vec());
            }
        }
    }

    fn send_hello(&mut self) {
        let caps = local_hello_capabilities(self.task_class);
        let bytes = hello_handler::encode_hello(&caps);
        if let Err(e) = self.conn.stream_send(0, &bytes, false) {
            if !matches!(e, quiche::Error::Done) {
                tracing::warn!(error = ?e, "stream_send(HELLO) failed");
            }
        }
    }

    /// FR-1.8: a discrete one-shot trigger (e.g. a canned stand/sit gait)
    /// sent immediately over its own Channel C reliable stream, not queued
    /// into the next Channel B tick -- see `channel_b.rs`'s `TeleopCommand`
    /// docs for why it no longer rides inside `VelocityAttitudeCommand`.
    fn send_action_trigger(&mut self, action_id: u8) {
        if self.phase != Phase::Operating {
            return;
        }
        // A canned action (e.g. sit) and a still-held movement key can
        // otherwise race: robot-edge only withholds continuous Move/Turn
        // while sitting if they're genuinely idle (see its
        // `dispatch_teleop_command` doc comment) -- a lingering non-zero
        // vx/vy/turn from a key the operator was already holding (or that
        // hasn't hit `move_stale` yet) reads as "resume moving" and
        // immediately stands the robot back up. Zeroing here, right at
        // the trigger, closes that window instead of waiting on staleness.
        self.last_command.vx = 0.0;
        self.last_command.vy = 0.0;
        self.last_command.turn = 0.0;
        self.hud.move_vx = 0.0;
        self.hud.move_vy = 0.0;
        self.hud.turn = 0.0;
        self.next_trigger_seq += 1;
        let trigger = ActionTrigger { action_id, trigger_seq: self.next_trigger_seq };
        let bytes = action_trigger_handler::encode_action_trigger(&trigger);
        self.recorder.enqueue(
            roboprotocol_recording::Category::ActionTriggerC,
            roboprotocol_recording::Record {
                capture_us: timestamp::now_micros(),
                control_source: roboprotocol_core::recording::CONTROL_SOURCE_SENTINEL,
                payload: bytes.clone(),
            },
        );
        if let Err(e) = self.conn.stream_send(ACTION_TRIGGER_STREAM_ID, &bytes, false) {
            if !matches!(e, quiche::Error::Done) {
                tracing::warn!(error = ?e, "stream_send(ActionTrigger) failed");
            }
        } else {
            self.hud.record_action_sent(action_id, self.channel_b_seq);
        }
    }

    /// Mirrors `self.camera_controls` into the HUD display fields and, if
    /// `--video-overlay` is on, the `ffplay` on-video text overlay --
    /// called after every nudge/reset, before the actual `CameraControl`
    /// send below (both are just "what's currently held" displays, not
    /// contingent on the send succeeding).
    fn sync_camera_hud_and_overlay(&mut self) {
        self.hud.camera_brightness = self.camera_controls.brightness;
        self.hud.camera_contrast = self.camera_controls.contrast;
        self.hud.camera_ev = self.camera_controls.ev;
        self.hud.camera_shutter_us = self.camera_controls.shutter_us;
        if let Some(overlay) = &self.video_overlay {
            let shutter = if self.camera_controls.shutter_us == 0 {
                "auto".to_string()
            } else {
                format!("{}us", self.camera_controls.shutter_us)
            };
            let text = format!(
                "brightness {:.1}  contrast {:.1}  EV {:.1}  shutter {}",
                self.camera_controls.brightness, self.camera_controls.contrast, self.camera_controls.ev, shutter
            );
            if let Err(e) = overlay.write(&text) {
                tracing::warn!(error = %e, "failed to write video overlay text");
            }
        }
    }

    /// Sends `self.camera_controls` as a discrete `CameraControl` RPC --
    /// called after every nudge/reset, mirroring `send_action_trigger`'s
    /// shape (own monotonic seq, Channel C reliable stream) but without
    /// that method's Move/Turn-zeroing side effect, which is specific to
    /// the sit/stand race it guards against and has nothing to do with the
    /// camera.
    fn send_camera_control(&mut self) {
        if self.phase != Phase::Operating {
            return;
        }
        self.next_camera_control_seq += 1;
        let control = CameraControl {
            brightness: self.camera_controls.brightness,
            contrast: self.camera_controls.contrast,
            ev: self.camera_controls.ev,
            shutter_us: self.camera_controls.shutter_us,
            control_seq: self.next_camera_control_seq,
        };
        let bytes = camera_control_handler::encode_camera_control(&control);
        self.recorder.enqueue(
            roboprotocol_recording::Category::ActionTriggerC,
            roboprotocol_recording::Record {
                capture_us: timestamp::now_micros(),
                control_source: roboprotocol_core::recording::CONTROL_SOURCE_SENTINEL,
                payload: bytes.clone(),
            },
        );
        if let Err(e) = self.conn.stream_send(CAMERA_CONTROL_STREAM_ID, &bytes, false) {
            if !matches!(e, quiche::Error::Done) {
                tracing::warn!(error = ?e, "stream_send(CameraControl) failed");
            }
        }
    }

    fn on_stream_readable(&mut self, stream_id: u64) {
        let mut sbuf = vec![0u8; 65535];
        loop {
            match self.conn.stream_recv(stream_id, &mut sbuf) {
                Ok((len, _fin)) => self.on_stream_data(stream_id, &sbuf[..len]),
                Err(quiche::Error::Done) => break,
                Err(e) => {
                    tracing::warn!(error = ?e, stream_id, "stream_recv error");
                    break;
                }
            }
        }
    }

    fn on_stream_data(&mut self, stream_id: u64, data: &[u8]) {
        match stream_id {
            0 if self.phase == Phase::AwaitingHello => match hello_handler::decode_hello(data) {
                Ok(peer_caps) => {
                    let local_caps = local_hello_capabilities(self.task_class);
                    match hello::negotiate(&local_caps, &peer_caps) {
                        Ok(_negotiated) => {
                            tracing::info!("HELLO negotiated successfully");
                            self.phase = Phase::AwaitingSessionDescribe;
                            self.hud.phase = ConnPhase::AwaitingSessionDescribe;
                        }
                        Err(e) => {
                            tracing::error!(error = %e, "HELLO_INCOMPATIBLE");
                            let _ = self.conn.close(true, 0x1, b"HELLO_INCOMPATIBLE");
                        }
                    }
                }
                Err(e) => tracing::warn!(error = %e, "failed to decode peer HELLO"),
            },
            1 if self.phase == Phase::AwaitingSessionDescribe => match session_handler::decode_session_describe(data) {
                Ok(info) => {
                    tracing::info!(robot_id = %info.robot_id, dof = info.profile.dof_count, "SESSION_DESCRIBE received");
                    let cached = session_cache::check_and_record_profile(&info.robot_id, info.profile_hash);
                    let accept_bytes = session_handler::encode_session_accept_full(&info, cached);
                    if let Err(e) = self.conn.stream_send(1, &accept_bytes, false) {
                        tracing::warn!(error = ?e, "stream_send(SESSION_ACCEPT) failed");
                    }
                    self.hud.robot_id = Some(info.robot_id.clone());
                    self.hud.dof_count = Some(info.profile.dof_count);
                    self.hud.camera_shape = info
                        .cameras
                        .first()
                        .map(|c| format!("{}x{} {:?} @{}fps", c.resolution_w, c.resolution_h, c.codec, c.max_fps));
                    self.session_info = Some(info);
                    self.phase = Phase::Operating;
                    self.hud.phase = ConnPhase::Operating;
                }
                Err(e) => tracing::warn!(error = %e, "failed to decode SESSION_DESCRIBE"),
            },
            ESTOP_STREAM_ID => {
                if let Some(d) = EstopDatagram::decode(data) {
                    self.estopped = d.latched;
                    self.hud.estopped = d.latched;
                    self.console.render(&self.hud);
                }
            }
            _ => tracing::debug!(stream_id, len = data.len(), "unhandled stream data"),
        }
    }

    fn on_datagram(&mut self, data: &[u8]) {
        if data.first() == Some(&ESTOP_DATAGRAM_MAGIC) {
            if let Some(d) = EstopDatagram::decode(data) {
                self.estopped = d.latched;
                self.hud.estopped = d.latched;
                self.console.render(&self.hud);
            }
            return;
        }

        // Video (Channel A) and command/telemetry (Channel B) datagrams
        // share one QUIC connection's unreliable-datagram stream with no
        // built-in demultiplexing, so each is prefixed with an explicit
        // type tag (roboprotocol_core::datagram) rather than guessed at
        // by trying to flatbuffers-decode everything -- this crate's
        // pinned old flatbuffers version has no buffer verification, so
        // feeding it a raw video chunk doesn't fail cleanly, it can panic.
        let Some((tag, payload)) = datagram::untag(data) else { return };
        match tag {
            datagram::DATAGRAM_TAG_CHANNEL_B if self.phase == Phase::Operating => {
                if let Ok(frame) = channel_b::decode_channel_b_frame(&payload) {
                    // Recorded ahead of the Telemetry-only HUD gate below,
                    // so recording isn't accidentally scoped to only what
                    // the HUD currently reads (covers Haptic automatically
                    // too, the day anything sends it). `capture_us` is
                    // this endpoint's own local receipt time, not
                    // `frame.timestamp` -- see
                    // `roboprotocol_core::recording`'s module doc for why.
                    self.recorder.enqueue(
                        recording_category_for(frame.category),
                        roboprotocol_recording::Record {
                            capture_us: timestamp::now_micros(),
                            control_source: roboprotocol_core::recording::CONTROL_SOURCE_SENTINEL,
                            payload: payload.to_vec(),
                        },
                    );
                    if frame.category == ChannelBCategory::Telemetry {
                        self.on_telemetry(&frame, payload.len());
                    }
                }
            }
            datagram::DATAGRAM_TAG_CHANNEL_A => {
                let now = std::time::Instant::now();
                self.hud.video_dgram_rate.record(now, payload.len());
                if let Some(nal_bytes) = self.video_rx.on_datagram(&payload) {
                    self.hud.video_frame_rate.record(now, nal_bytes.len());
                    self.recorder.enqueue(
                        roboprotocol_recording::Category::VideoA,
                        roboprotocol_recording::Record {
                            capture_us: timestamp::now_micros(),
                            control_source: roboprotocol_core::recording::CONTROL_SOURCE_SENTINEL,
                            payload: nal_bytes.clone(),
                        },
                    );
                    // `nal_bytes` is "start code + NAL"
                    // (`ChannelAReceiver::on_datagram` prepends it) --
                    // `nal_is_critical` needs the raw NAL, so skip the
                    // 4-byte start code when classifying. Exactly one of
                    // `video_tx`/`native_video_tx` is ever `Some` per
                    // `VideoBackend`, so `nal_bytes` only ever moves once.
                    let is_critical = nal_bytes.get(4..).is_some_and(nal_is_critical);
                    if let Some(tx) = &self.video_tx {
                        tx.send(nal_bytes, is_critical);
                    } else if let Some(tx) = &self.native_video_tx {
                        tx.send(nal_bytes, is_critical);
                    }
                }
            }
            _ => {}
        }
    }

    fn on_telemetry(&mut self, frame: &ChannelBFrameData, wire_len: usize) {
        let now_us = timestamp::now_micros();
        if frame.timestamp > 0 && now_us > frame.timestamp {
            self.hud.rtt_ms = Some((now_us - frame.timestamp) as f64 / 1000.0);
        }
        let now = std::time::Instant::now();
        self.hud.last_telemetry_at = Some(now);
        self.hud.telemetry_rate.record(now, wire_len);
        self.hud.telemetry_last_seq = Some(frame.seq);
        if let Some(t) = channel_b::TelemetryData::unpack(&frame.fields) {
            self.hud.battery = Some(t.battery);
            self.hud.roll = Some(t.roll as f64);
            self.hud.pitch = Some(t.pitch as f64);
            self.hud.yaw = Some(t.yaw as f64);
            self.hud.motors = t.motors;
        }
        self.console.render(&self.hud);
    }

    /// Shared by the keyboard and gamepad `select!` arms in `run` -- logs
    /// the input (here rather than in `on_input`, specifically so `Quit`
    /// is captured too, since the caller returns before `on_input` ever
    /// sees it), then either starts a clean shutdown (`Quit`, returning
    /// `Ok(true)` so the caller knows to exit the loop) or dispatches it.
    async fn dispatch_teleop_input(&mut self, input: TeleopInput, socket: &UdpSocket) -> Result<bool> {
        self.recorder.enqueue(
            roboprotocol_recording::Category::KeyPress,
            roboprotocol_recording::Record {
                capture_us: roboprotocol_core::timestamp::now_micros(),
                control_source: roboprotocol_core::recording::CONTROL_SOURCE_SENTINEL,
                payload: format!("{input:?}").into_bytes(),
            },
        );
        if matches!(input, TeleopInput::Quit) {
            let _ = self.conn.close(true, 0x0, b"operator quit");
            flush_once(&mut self.conn, socket).await?;
            return Ok(true);
        }
        self.on_input(input);
        Ok(false)
    }

    fn on_input(&mut self, input: TeleopInput) {
        match input {
            TeleopInput::Move { vx, vy } => {
                tracing::info!(vx, vy, "move key event");
                self.last_command.vx = vx;
                self.last_command.vy = vy;
                self.hud.move_vx = vx;
                self.hud.move_vy = vy;
                self.last_move_input_at = Some(std::time::Instant::now());
            }
            TeleopInput::Turn { turn } => {
                tracing::info!(turn, "turn key event");
                self.last_command.turn = turn;
                self.hud.turn = turn;
                self.last_move_input_at = Some(std::time::Instant::now());
            }
            TeleopInput::Action(id) => self.send_action_trigger(id),
            TeleopInput::ArmNudge { dx, dz } => {
                self.last_command.arm_x = (self.last_command.arm_x + dx).clamp(*ARM_X_RANGE.start(), *ARM_X_RANGE.end());
                self.last_command.arm_z = (self.last_command.arm_z + dz).clamp(*ARM_Z_RANGE.start(), *ARM_Z_RANGE.end());
                // Front-screen self-collision keep-out (see
                // `min_safe_arm_x`'s doc) -- applied after the general
                // range clamp above, and unconditionally on every nudge
                // (not just x-axis ones) since a z-only nudge can make an
                // already-held x unsafe by shrinking the keep-out margin.
                // Re-clamped into ARM_X_RANGE afterward as a second,
                // independent guard: this constraint must only ever make
                // the result *safer*, never override the hardware's own
                // physical range in the other direction.
                self.last_command.arm_x =
                    self.last_command.arm_x.max(min_safe_arm_x(self.last_command.arm_z)).min(*ARM_X_RANGE.end());
                // Position-commanded, not velocity -- no staleness timer,
                // it holds wherever last sent (see xgo_profile.rs's comment).
                self.hud.arm_x = self.last_command.arm_x;
                self.hud.arm_z = self.last_command.arm_z;
            }
            TeleopInput::AttitudeNudge { axis, delta } => {
                // Position-commanded, not velocity -- no staleness timer,
                // it holds wherever last sent (same as arm/claw above).
                match axis {
                    'r' => {
                        self.last_command.attitude_r =
                            (self.last_command.attitude_r + delta).clamp(*ROLL_RANGE.start(), *ROLL_RANGE.end());
                        self.hud.attitude_r_cmd = self.last_command.attitude_r;
                    }
                    'p' => {
                        self.last_command.attitude_p =
                            (self.last_command.attitude_p + delta).clamp(*PITCH_RANGE.start(), *PITCH_RANGE.end());
                        self.hud.attitude_p_cmd = self.last_command.attitude_p;
                    }
                    'y' => {
                        self.last_command.attitude_y =
                            (self.last_command.attitude_y + delta).clamp(*YAW_RANGE.start(), *YAW_RANGE.end());
                        self.hud.attitude_y_cmd = self.last_command.attitude_y;
                    }
                    other => tracing::warn!(axis = %other, "unknown attitude axis, ignoring"),
                }
            }
            TeleopInput::AttitudeReset => {
                self.last_command.attitude_r = 0.0;
                self.last_command.attitude_p = 0.0;
                self.last_command.attitude_y = 0.0;
                self.hud.attitude_r_cmd = 0.0;
                self.hud.attitude_p_cmd = 0.0;
                self.hud.attitude_y_cmd = 0.0;
            }
            // Arm/claw back to the same neutral values `Client::run`
            // constructs `last_command` with at boot, plus a reused
            // `AttitudeReset` for the level part -- see
            // `TeleopInput::NeutralPose`'s doc.
            TeleopInput::NeutralPose => {
                self.last_command.arm_x = 0;
                self.last_command.arm_z = 0;
                self.last_command.claw = ARM_CLAW_NEUTRAL;
                self.hud.arm_x = 0;
                self.hud.arm_z = 0;
                self.hud.claw = ARM_CLAW_NEUTRAL;
                self.on_input(TeleopInput::AttitudeReset);
            }
            TeleopInput::CameraNudge { param, delta } => {
                match param {
                    'b' => {
                        self.camera_controls.brightness =
                            (self.camera_controls.brightness + delta).clamp(*BRIGHTNESS_RANGE.start(), *BRIGHTNESS_RANGE.end());
                    }
                    'c' => {
                        self.camera_controls.contrast =
                            (self.camera_controls.contrast + delta).clamp(*CONTRAST_RANGE.start(), *CONTRAST_RANGE.end());
                    }
                    'v' => {
                        self.camera_controls.ev = (self.camera_controls.ev + delta).clamp(*EV_RANGE.start(), *EV_RANGE.end());
                    }
                    's' => {
                        // Integer arithmetic in i64 to avoid underflow going
                        // below 0 (u32) before the clamp gets a chance to run.
                        let next = self.camera_controls.shutter_us as i64 + delta.round() as i64;
                        self.camera_controls.shutter_us =
                            next.clamp(*SHUTTER_RANGE_US.start() as i64, *SHUTTER_RANGE_US.end() as i64) as u32;
                    }
                    other => tracing::warn!(param = %other, "unknown camera control param, ignoring"),
                }
                self.sync_camera_hud_and_overlay();
                self.send_camera_control();
            }
            TeleopInput::CameraReset => {
                self.camera_controls = CameraControlState::default();
                self.sync_camera_hud_and_overlay();
                self.send_camera_control();
            }
            TeleopInput::ClawNudge { delta } => {
                self.last_command.claw = (self.last_command.claw as i16 + delta as i16).clamp(0, 255) as u8;
                self.hud.claw = self.last_command.claw;
            }
            TeleopInput::Estop => {
                self.estop_seq += 1;
                let d = EstopDatagram { latched: true, seq: self.estop_seq };
                let _ = self.conn.dgram_send(&d.encode());
                let _ = self.conn.stream_send(ESTOP_STREAM_ID, &d.encode(), false);
            }
            TeleopInput::EstopClear => {
                self.estop_seq += 1;
                let d = EstopDatagram { latched: false, seq: self.estop_seq };
                let _ = self.conn.dgram_send(&d.encode());
                let _ = self.conn.stream_send(ESTOP_STREAM_ID, &d.encode(), false);
            }
            // See `TeleopInput::EstopToggle`'s doc -- resolved here since
            // this is the one place that actually knows `self.estopped`.
            TeleopInput::EstopToggle => {
                self.on_input(if self.estopped { TeleopInput::EstopClear } else { TeleopInput::Estop });
            }
            TeleopInput::ToggleRecording => {
                let new_state = !self.hud.recording_active;
                for category in DEFAULT_RECORD_CATEGORIES {
                    self.recorder.set_active(category, new_state);
                }
                self.hud.recording_active = new_state;
                self.hud.recording_started_at = new_state.then(std::time::Instant::now);
                tracing::info!(new_state, "recording toggled");
            }
            TeleopInput::Quit => {}
        }
        self.console.render(&self.hud);
    }

    fn on_tick(&mut self) {
        if self.phase != Phase::Operating {
            return;
        }

        self.hud.gamepad_connected = self.gamepad.as_ref().is_some_and(GamepadReader::is_connected);
        self.hud.gamepad_stick_mode = self.gamepad.as_ref().map_or("turn", GamepadReader::stick_mode_label);

        // Refreshed every tick, ahead of the render that follows this
        // call in `run`'s select loop -- the one visible signal that
        // FR-9.3's "drop oldest under pressure, never block" is actually
        // happening, not something silently wrong.
        self.hud.recording_dropped =
            DEFAULT_RECORD_CATEGORIES.iter().filter_map(|&c| self.recorder.stats(c)).map(|s| s.records_dropped).sum();

        // XGO's move_x/move_y/turn are continuous velocity setpoints, not
        // one-shot moves (they write a register and return immediately --
        // the robot's own firmware just keeps executing the last value
        // forever). Terminal key auto-repeat keeps `last_move_input_at`
        // fresh while a movement key is physically held; once repeats stop
        // (key released) and nothing has refreshed it for `move_stale`,
        // zero the held command so the robot actually stops.
        if let Some(last) = self.last_move_input_at {
            let elapsed = last.elapsed();
            if elapsed >= self.move_stale && (self.last_command.vx != 0.0 || self.last_command.vy != 0.0 || self.last_command.turn != 0.0) {
                tracing::info!(elapsed_ms = elapsed.as_millis() as u64, "movement stale, auto-zeroing");
                self.last_command.vx = 0.0;
                self.last_command.vy = 0.0;
                self.last_command.turn = 0.0;
                self.hud.move_vx = 0.0;
                self.hud.move_vy = 0.0;
                self.hud.turn = 0.0;
            }
        }

        if let Some(action_id) = self.demo_action.take() {
            self.send_action_trigger(action_id);
        }

        self.channel_b_seq += 1;
        self.last_send_ts = timestamp::now_micros();
        let frame = ChannelBFrameData {
            timestamp: self.last_send_ts,
            seq: self.channel_b_seq,
            tick_id: 0,
            category: ChannelBCategory::Command,
            region_id: ALL_REGIONS,
            fields: self.last_command.pack(),
        };
        let bytes = channel_b::encode_channel_b_frame(&frame);
        // Locally originated, so `capture_us` here equals `self.last_send_ts`
        // (== `frame.timestamp`) by construction -- see
        // `roboprotocol_core::recording`'s module doc for why `capture_us`
        // is always local rather than special-cased per category.
        self.recorder.enqueue(
            roboprotocol_recording::Category::ChannelBCommand,
            roboprotocol_recording::Record {
                capture_us: self.last_send_ts,
                control_source: roboprotocol_core::recording::CONTROL_SOURCE_SENTINEL,
                payload: bytes.clone(),
            },
        );
        let _ = self.conn.dgram_send(&datagram::tag(datagram::DATAGRAM_TAG_CHANNEL_B, &bytes));
        self.hud.command_rate.record(std::time::Instant::now(), bytes.len());
        self.hud.command_last_seq = self.channel_b_seq;
    }
}

async fn recv_input(input: &mut Option<InputReader>) -> Option<TeleopInput> {
    match input {
        Some(input) => input.next().await,
        None => std::future::pending().await,
    }
}

async fn recv_gamepad(gamepad: &mut Option<GamepadReader>) -> Option<TeleopInput> {
    match gamepad {
        Some(gamepad) => gamepad.next().await,
        None => std::future::pending().await,
    }
}

async fn flush_once(conn: &mut quiche::Connection, socket: &UdpSocket) -> Result<()> {
    let mut out = vec![0u8; MAX_DATAGRAM_SIZE];
    loop {
        match conn.send(&mut out) {
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

fn recording_category_for(category: ChannelBCategory) -> roboprotocol_recording::Category {
    match category {
        ChannelBCategory::Command => roboprotocol_recording::Category::ChannelBCommand,
        ChannelBCategory::Telemetry => roboprotocol_recording::Category::ChannelBTelemetry,
        ChannelBCategory::Haptic => roboprotocol_recording::Category::ChannelBHaptic,
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
        // v0: no capabilities advertised yet -- see robot-edge's matching comment.
        capability_bitmask: 0,
        supported_task_classes: task_class_bit,
        supported_quantization_tiers: quantization_tier_bits::STANDARD,
        max_control_rate_hz: 50,
    }
}

fn build_quiche_config(args: &ClientArgs) -> Result<quiche::Config> {
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
    // 0-RTT resumption: lets a reconnect to the same still-running robot-edge
    // skip the full handshake RTT -- only HELLO (Channel C stream 0) is ever
    // actually sent as early data, per DESIGN.md SecR-1.3's explicit carve-out;
    // Channel B stays gated behind the full SESSION_ACCEPT round trip regardless.
    config.enable_early_data();
    config.verify_peer(true);
    config.load_cert_chain_from_pem_file(&args.cert_path).context("loading operator cert chain")?;
    config.load_priv_key_from_pem_file(&args.key_path).context("loading operator private key")?;
    config.load_verify_locations_from_file(&args.ca_path).context("loading CA for peer verification")?;
    Ok(config)
}
