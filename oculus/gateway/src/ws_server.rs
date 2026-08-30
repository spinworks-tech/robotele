//! Browser-facing side: serves the static WebXR page (`oculus/web/`) and a
//! single `/ws` endpoint that fans out JPEG video frames (binary messages)
//! and telemetry/status events (text/JSON messages) to every connected
//! viewer via `tokio::sync::broadcast` -- a slow viewer naturally sees
//! frames skip ahead (`broadcast::error::RecvError::Lagged`) rather than
//! this server building an unbounded per-client backlog, the same
//! "drop stale, never queue" rule `video/decode.rs` already applies on
//! the decode side.
//!
//! Served over HTTPS/WSS with a self-signed cert generated at startup
//! (`main.rs`) -- WebXR's `navigator.xr` is only available in a secure
//! context, and a plain-HTTP page served from a LAN IP (not `localhost`)
//! doesn't qualify. The Quest Browser will show a certificate warning on
//! first visit; "proceed anyway" still satisfies the secure-context check
//! since the origin is delivered over TLS, just not a browser-trusted CA.

use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::Arc;

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::State;
use axum::response::IntoResponse;
use axum::routing::get;
use axum::Router;
use futures_util::{SinkExt, StreamExt};
use tokio::sync::broadcast;
use tower_http::services::ServeDir;

#[derive(Clone)]
pub struct GatewayState {
    pub video_tx: broadcast::Sender<Vec<u8>>,
    pub event_tx: broadcast::Sender<String>,
}

impl GatewayState {
    pub fn new() -> Self {
        // Capacity 2: only ever care about the newest frame/event: a
        // lagging receiver just skips ahead rather than draining a
        // backlog of stale ones (see module doc).
        let (video_tx, _) = broadcast::channel(2);
        let (event_tx, _) = broadcast::channel(16);
        Self { video_tx, event_tx }
    }
}

pub async fn serve(listen: SocketAddr, web_dir: PathBuf, tls: axum_server::tls_rustls::RustlsConfig, state: GatewayState) -> anyhow::Result<()> {
    let app = Router::new()
        .route("/ws", get(ws_handler))
        .fallback_service(ServeDir::new(web_dir))
        .with_state(Arc::new(state));

    tracing::info!(%listen, "oculus-gateway web server listening (https)");
    axum_server::bind_rustls(listen, tls).serve(app.into_make_service()).await?;
    Ok(())
}

async fn ws_handler(ws: WebSocketUpgrade, State(state): State<Arc<GatewayState>>) -> impl IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, state))
}

async fn handle_socket(socket: WebSocket, state: Arc<GatewayState>) {
    let (mut sender, mut receiver) = socket.split();
    let mut video_rx = state.video_tx.subscribe();
    let mut event_rx = state.event_tx.subscribe();

    // Drain-only: we don't accept any control messages from the browser
    // (this viewer is read-only, see the crate's top-level doc) -- this
    // task's only job is noticing the socket closed so the loop below can
    // stop pushing frames to a dead connection.
    let mut recv_task = tokio::spawn(async move {
        while let Some(msg) = receiver.next().await {
            if msg.is_err() {
                break;
            }
        }
    });

    loop {
        tokio::select! {
            frame = video_rx.recv() => {
                match frame {
                    Ok(bytes) => {
                        if sender.send(Message::Binary(bytes.into())).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
            event = event_rx.recv() => {
                match event {
                    Ok(text) => {
                        if sender.send(Message::Text(text.into())).await.is_err() {
                            break;
                        }
                    }
                    Err(broadcast::error::RecvError::Lagged(_)) => continue,
                    Err(broadcast::error::RecvError::Closed) => break,
                }
            }
            _ = &mut recv_task => break,
        }
    }
    recv_task.abort();
}
