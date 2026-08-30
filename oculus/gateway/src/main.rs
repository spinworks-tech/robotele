//! `oculus-gateway`: read-only bridge between `robot-edge`'s real
//! QUIC/mTLS/FlatBuffers protocol and a stock Meta Quest 2 (or any WebXR
//! browser) -- Option 0 from the Quest 2 VR-teleop discussion. Decodes
//! Channel A (video) to JPEG and Channel B telemetry to JSON, and fans
//! both out over a WebSocket to the static page in `oculus/web/`. See
//! `quic_client.rs`'s module doc for what this deliberately never sends.

mod channel_b;
mod hello_handler;
mod quic_client;
mod session_handler;
mod video;
mod ws_server;

use std::net::SocketAddr;
use std::path::PathBuf;

use anyhow::{Context, Result};
use rcgen::CertifiedKey;
use roboprotocol_core::safety::TaskClass;

use crate::quic_client::ClientArgs;
use crate::ws_server::GatewayState;

struct Cli {
    connect: SocketAddr,
    bind: SocketAddr,
    server_name: String,
    cert: String,
    key: String,
    ca: String,
    task_class: TaskClass,
    listen: SocketAddr,
    web_dir: PathBuf,
    tls_san: Vec<String>,
}

impl Cli {
    fn parse() -> Result<Self> {
        let mut connect: SocketAddr = "127.0.0.1:4433".parse().unwrap();
        let mut bind: SocketAddr = "0.0.0.0:0".parse().unwrap();
        let mut server_name = "robot-edge".to_string();
        let mut cert = "certs/operator/operator.crt".to_string();
        let mut key = "certs/operator/operator.key".to_string();
        let mut ca = "certs/dev-ca/ca.crt".to_string();
        let mut task_class = TaskClass::D;
        let mut listen: SocketAddr = "0.0.0.0:8443".parse().unwrap();
        let mut web_dir = PathBuf::from("oculus/web");
        let mut tls_san = vec!["localhost".to_string(), "127.0.0.1".to_string()];

        let mut it = std::env::args().skip(1);
        while let Some(arg) = it.next() {
            match arg.as_str() {
                "--connect" => connect = it.next().context("--connect needs a value")?.parse()?,
                "--bind" => bind = it.next().context("--bind needs a value")?.parse()?,
                "--server-name" => server_name = it.next().context("--server-name needs a value")?,
                "--cert" => cert = it.next().context("--cert needs a value")?,
                "--key" => key = it.next().context("--key needs a value")?,
                "--ca" => ca = it.next().context("--ca needs a value")?,
                "--task-class" => {
                    task_class = match it.next().context("--task-class needs a value")?.as_str() {
                        "B" => TaskClass::B,
                        "C" => TaskClass::C,
                        "D" => TaskClass::D,
                        "E" => TaskClass::E,
                        other => anyhow::bail!("unknown task class {other}, expected B/C/D/E"),
                    }
                }
                "--listen" => listen = it.next().context("--listen needs a value")?.parse()?,
                "--web-dir" => web_dir = PathBuf::from(it.next().context("--web-dir needs a value")?),
                "--tls-san" => tls_san.push(it.next().context("--tls-san needs a value")?),
                "-h" | "--help" => {
                    println!(
                        "Usage: oculus-gateway [--connect ADDR] [--bind ADDR] [--server-name NAME]\n  \
                         [--cert PATH] [--key PATH] [--ca PATH] [--task-class B|C|D|E]\n  \
                         [--listen ADDR] [--web-dir PATH] [--tls-san NAME]\n\n\
                         Monitor-only Quest 2 / WebXR bridge -- see quic_client.rs's module doc.\n\
                         --connect is robot-edge's QUIC address (default 127.0.0.1:4433).\n\
                         --listen is this process's own HTTPS/WSS address for the WebXR page\n  \
                         (default 0.0.0.0:8443) -- open https://<this-host>:8443/ in the Quest\n  \
                         Browser (or any browser) and accept the self-signed cert warning.\n\
                         --tls-san adds a Subject Alternative Name to that self-signed cert\n  \
                         (repeatable) -- pass this gateway's LAN IP if you're connecting from a\n  \
                         Quest 2 over Wi-Fi so the cert's SAN actually matches the address you\n  \
                         type into the browser (localhost/127.0.0.1 are always included)."
                    );
                    std::process::exit(0);
                }
                other => anyhow::bail!("unrecognized argument: {other} (see --help)"),
            }
        }

        Ok(Self { connect, bind, server_name, cert, key, ca, task_class, listen, web_dir, tls_san })
    }
}

/// Self-signed cert for the browser-facing HTTPS/WSS server, generated
/// fresh on every startup -- see `ws_server.rs`'s module doc for why this
/// needs to be HTTPS at all (WebXR's secure-context requirement). Not
/// persisted to disk: this is a LAN dev tool, and regenerating on restart
/// means one fewer file to gitignore or accidentally commit, at the minor
/// cost of the browser needing to re-accept the warning after a restart.
async fn build_tls_config(sans: Vec<String>) -> Result<axum_server::tls_rustls::RustlsConfig> {
    let CertifiedKey { cert, key_pair } = rcgen::generate_simple_self_signed(sans).context("generating self-signed TLS cert")?;
    let cert_pem = cert.pem();
    let key_pem = key_pair.serialize_pem();
    axum_server::tls_rustls::RustlsConfig::from_pem(cert_pem.into_bytes(), key_pem.into_bytes())
        .await
        .context("loading generated TLS cert")
}

#[tokio::main]
async fn main() -> Result<()> {
    tracing_subscriber::fmt::init();
    let cli = Cli::parse()?;

    let state = GatewayState::new();
    let tls = build_tls_config(cli.tls_san).await?;

    // Bridges the decode thread's bounded tokio channel into the
    // broadcast fan-out every connected WS client subscribes to.
    let (jpeg_tx, mut jpeg_rx) = tokio::sync::mpsc::channel(2);
    let video_decoder = video::decode::spawn_video_decoder(jpeg_tx);
    let broadcast_video_tx = state.video_tx.clone();
    tokio::spawn(async move {
        while let Some(jpeg) = jpeg_rx.recv().await {
            let _ = broadcast_video_tx.send(jpeg);
        }
    });

    let client_args = ClientArgs {
        connect: cli.connect,
        bind: cli.bind,
        server_name: cli.server_name,
        cert_path: cli.cert,
        key_path: cli.key,
        ca_path: cli.ca,
        task_class: cli.task_class,
    };
    let quic_state = state.clone();
    let quic_task = tokio::spawn(async move { quic_client::run(client_args, quic_state, video_decoder).await });

    let web_task = tokio::spawn(async move { ws_server::serve(cli.listen, cli.web_dir, tls, state).await });

    tokio::select! {
        res = quic_task => res.context("quic client task panicked")??,
        res = web_task => res.context("web server task panicked")??,
    }
    Ok(())
}
