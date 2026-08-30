//! Monitor-only QUIC client: connects to `robot-edge` exactly like
//! operator-console (HELLO -> SESSION_DESCRIBE/SESSION_ACCEPT -> Operating),
//! but only ever *reads*. It never sends a Channel B command, an
//! ActionTrigger, a CameraControl, or an E-Stop -- see
//! `crates/operator-console/src/quic_client.rs` for the full teleop
//! version this is trimmed from.
//!
//! Two consequences worth knowing before pointing this at a real robot:
//!
//! 1. `robot-edge` is v0 single-connection only (see its own
//!    `quic_server.rs` module doc) -- this gateway's connection and a live
//!    `operator-console` teleop session cannot both be attached at once.
//!    Run this against an otherwise-idle `robot-edge`, or use it in place
//!    of `operator-console` for a monitor-only session.
//! 2. `robot-edge`'s safety watchdog (`roboprotocol_core::safety::Watchdog`)
//!    latches E-Stop after `TaskClass::D`'s 400ms of Channel B Command
//!    silence. Since this client never sends one, the robot will E-Stop
//!    itself shortly after reaching Operating -- expected and safe (it's
//!    the correct response to "no commands incoming"), not a bug to chase.

use std::net::SocketAddr;
use std::time::Duration;

use anyhow::{Context, Result};
use rand::RngCore;
use roboprotocol_core::datagram;
use roboprotocol_core::estop::{EstopDatagram, ESTOP_DATAGRAM_MAGIC, ESTOP_STREAM_ID};
use roboprotocol_core::hello::{self, quantization_tier_bits, task_class_bits, HelloCapabilities, ProtocolVersion};
use roboprotocol_core::safety::TaskClass;
use roboprotocol_core::timestamp;
use serde::Serialize;
use tokio::net::UdpSocket;

use crate::channel_b::{self, ChannelBCategory};
use crate::hello_handler;
use crate::session_handler;
use crate::video::channel_a::ChannelAReceiver;
use crate::video::decode::VideoDecoderTx;
use crate::ws_server::GatewayState;

const MAX_DATAGRAM_SIZE: usize = 1452;
/// Backoff between reconnect attempts after the connection is lost --
/// there's no operator watching a TUI to decide "give up", so this client
/// just keeps trying (a Quest viewer reopening the page is the only
/// "give up" signal that matters here).
const RECONNECT_BACKOFF: Duration = Duration::from_secs(2);

pub struct ClientArgs {
    pub connect: SocketAddr,
    pub bind: SocketAddr,
    pub server_name: String,
    pub cert_path: String,
    pub key_path: String,
    pub ca_path: String,
    pub task_class: TaskClass,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    AwaitingHello,
    AwaitingSessionDescribe,
    Operating,
}

#[derive(Serialize)]
#[serde(tag = "type", rename_all = "snake_case")]
enum GatewayEvent<'a> {
    Status { phase: &'a str, robot_id: Option<&'a str>, dof_count: Option<u16>, camera: Option<&'a str>, estopped: bool },
    Telemetry { battery: u8, roll: f32, pitch: f32, yaw: f32, motors: &'a [f32], seq: u64, rtt_ms: Option<f64> },
}

fn emit(state: &GatewayState, event: &GatewayEvent) {
    if let Ok(json) = serde_json::to_string(event) {
        let _ = state.event_tx.send(json);
    }
}

/// Runs forever: connects, drives one session until it ends (peer closes,
/// network drops, etc.), then reconnects after `RECONNECT_BACKOFF`. Meant
/// to be spawned as a long-lived task alongside the web server.
pub async fn run(args: ClientArgs, state: GatewayState, video_tx: VideoDecoderTx) -> Result<()> {
    loop {
        emit(&state, &GatewayEvent::Status { phase: "connecting", robot_id: None, dof_count: None, camera: None, estopped: false });
        match run_session(&args, &state, &video_tx).await {
            Ok(()) => tracing::info!("session ended cleanly"),
            Err(e) => tracing::warn!(error = %e, "session ended with error"),
        }
        emit(&state, &GatewayEvent::Status { phase: "disconnected", robot_id: None, dof_count: None, camera: None, estopped: false });
        tokio::time::sleep(RECONNECT_BACKOFF).await;
    }
}

async fn run_session(args: &ClientArgs, state: &GatewayState, video_tx: &VideoDecoderTx) -> Result<()> {
    let socket = UdpSocket::bind(args.bind).await.context("binding UDP socket")?;
    let local_addr = socket.local_addr()?;

    let mut config = build_quiche_config(args)?;
    let mut scid_bytes = [0u8; 16];
    rand::thread_rng().fill_bytes(&mut scid_bytes);
    let scid = quiche::ConnectionId::from_vec(scid_bytes.to_vec());
    let server_name = (!args.server_name.is_empty()).then_some(args.server_name.as_str());
    let mut conn = quiche::connect(server_name, &scid, local_addr, args.connect, &mut config)
        .map_err(|e| anyhow::anyhow!("quiche::connect failed: {e:?}"))?;
    tracing::info!(peer = %args.connect, trace_id = conn.trace_id(), "connecting");
    flush_once(&mut conn, &socket).await?;

    let mut client = Client {
        conn,
        phase: Phase::AwaitingHello,
        video_rx: ChannelAReceiver::new(),
        robot_id: None,
        dof_count: None,
        camera: None,
        estopped: false,
        task_class: args.task_class,
        state,
        video_tx,
    };
    client.after_recv();
    flush_once(&mut client.conn, &socket).await?;
    client.run(socket).await
}

struct Client<'a> {
    conn: quiche::Connection,
    phase: Phase,
    video_rx: ChannelAReceiver,
    robot_id: Option<String>,
    dof_count: Option<u16>,
    camera: Option<String>,
    estopped: bool,
    task_class: TaskClass,
    state: &'a GatewayState,
    video_tx: &'a VideoDecoderTx,
}

impl Client<'_> {
    async fn run(&mut self, socket: UdpSocket) -> Result<()> {
        let mut buf = vec![0u8; 65535];
        loop {
            let timeout = self.conn.timeout().unwrap_or(Duration::from_millis(100));
            tokio::select! {
                recv = socket.recv_from(&mut buf) => {
                    let (len, from) = recv.context("socket recv_from failed")?;
                    let info = quiche::RecvInfo { from, to: socket.local_addr()? };
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
            }

            if self.conn.is_closed() {
                anyhow::bail!("connection closed");
            }
        }
    }

    fn after_recv(&mut self) {
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
                            self.emit_status();
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
                    let accept_bytes = session_handler::encode_session_accept_full(&info, false);
                    if let Err(e) = self.conn.stream_send(1, &accept_bytes, false) {
                        tracing::warn!(error = ?e, "stream_send(SESSION_ACCEPT) failed");
                    }
                    self.robot_id = Some(info.robot_id.clone());
                    self.dof_count = Some(info.profile.dof_count);
                    self.camera = info
                        .cameras
                        .first()
                        .map(|c| format!("{}x{} {:?} @{}fps", c.resolution_w, c.resolution_h, c.codec, c.max_fps));
                    self.phase = Phase::Operating;
                    self.emit_status();
                }
                Err(e) => tracing::warn!(error = %e, "failed to decode SESSION_DESCRIBE"),
            },
            ESTOP_STREAM_ID => self.on_estop_datagram(data),
            _ => tracing::debug!(stream_id, len = data.len(), "unhandled stream data"),
        }
    }

    fn on_estop_datagram(&mut self, data: &[u8]) {
        if let Some(d) = EstopDatagram::decode(data) {
            self.estopped = d.latched;
            self.emit_status();
        }
    }

    fn on_datagram(&mut self, data: &[u8]) {
        if data.first() == Some(&ESTOP_DATAGRAM_MAGIC) {
            self.on_estop_datagram(data);
            return;
        }

        let Some((tag, payload)) = datagram::untag(data) else { return };
        match tag {
            datagram::DATAGRAM_TAG_CHANNEL_B if self.phase == Phase::Operating => {
                if let Ok(frame) = channel_b::decode_channel_b_frame(&payload) {
                    if frame.category == ChannelBCategory::Telemetry {
                        self.on_telemetry(&frame);
                    }
                }
            }
            datagram::DATAGRAM_TAG_CHANNEL_A => {
                // Every completed NAL, in arrival order -- decode.rs must
                // never have one dropped here (see its module doc for why
                // that broke the H.264 reference chain).
                if let Some(nal_bytes) = self.video_rx.on_datagram(&payload) {
                    self.video_tx.send(nal_bytes);
                }
            }
            _ => {}
        }
    }

    fn on_telemetry(&mut self, frame: &channel_b::ChannelBFrameData) {
        let now_us = timestamp::now_micros();
        let rtt_ms = (frame.timestamp > 0 && now_us > frame.timestamp).then(|| (now_us - frame.timestamp) as f64 / 1000.0);
        if let Some(t) = channel_b::TelemetryData::unpack(&frame.fields) {
            emit(
                self.state,
                &GatewayEvent::Telemetry {
                    battery: t.battery,
                    roll: t.roll,
                    pitch: t.pitch,
                    yaw: t.yaw,
                    motors: &t.motors,
                    seq: frame.seq,
                    rtt_ms,
                },
            );
        }
    }

    fn emit_status(&self) {
        let phase = match self.phase {
            Phase::AwaitingHello => "awaiting_hello",
            Phase::AwaitingSessionDescribe => "awaiting_session_describe",
            Phase::Operating => "operating",
        };
        emit(
            self.state,
            &GatewayEvent::Status {
                phase,
                robot_id: self.robot_id.as_deref(),
                dof_count: self.dof_count,
                camera: self.camera.as_deref(),
                estopped: self.estopped,
            },
        );
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

fn local_hello_capabilities(task_class: TaskClass) -> HelloCapabilities {
    let task_class_bit = match task_class {
        TaskClass::B => task_class_bits::CLASS_B,
        TaskClass::C => task_class_bits::CLASS_C,
        TaskClass::D => task_class_bits::CLASS_D,
        TaskClass::E => task_class_bits::CLASS_E,
    };
    HelloCapabilities {
        protocol_version: ProtocolVersion::new(1, 0),
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
    config.verify_peer(true);
    config.load_cert_chain_from_pem_file(&args.cert_path).context("loading operator cert chain")?;
    config.load_priv_key_from_pem_file(&args.key_path).context("loading operator private key")?;
    config.load_verify_locations_from_file(&args.ca_path).context("loading CA for peer verification")?;
    Ok(config)
}
