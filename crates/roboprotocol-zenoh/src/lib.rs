//! Zenoh on each side, RoboProtocol across the link
//! (spinworks-tech/robotele#36).
//!
//! A `Forwarder` subscribes to the key expressions in its rules on the local
//! Zenoh session and hands each sample to the QUIC session loop
//! (`Forwarder::recv`), which sends it to the peer as a
//! `roboprotocol_core::topic` sample. Samples the peer forwards are given to
//! `Forwarder::republish` and put on the local session under the same key.
//! BabyROS (or any Zenoh) nodes on either host don't change at all.
//!
//! Nothing here can stall the session loop: both directions go through
//! bounded channels with `try_send`, and a full channel drops the sample.
//!
//! Loops: the subscribers only take samples from *other* Zenoh sessions
//! (`Locality::Remote`), so a sample this process republishes is never
//! picked up and forwarded again on the same side. That isn't enough if the
//! two hosts' Zenoh domains are also connected to each other directly (for
//! example by multicast scouting on the same Wi-Fi): then a sample would
//! reach the far side both ways and loop between the two forwarders. The
//! robot declares a liveliness token (`announce`), and the operator side
//! pauses forwarding in both directions while it can see that token on its
//! own domain (`pause_while_visible`).

pub mod link;

use std::str::FromStr;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use anyhow::{anyhow, Context, Result};
use tokio::sync::mpsc;
use zenoh::sample::{Locality, SampleKind};

use roboprotocol_core::topic::{check_key, TopicMode, TopicSample};

/// Samples waiting to cross the QUIC session, each way. Latest-mode samples
/// collapse per key once they reach the session's `TopicOutbox`, so this
/// only needs to absorb a burst between two session loop iterations.
const CHANNEL_DEPTH: usize = 256;
/// How often the operator side checks whether it can see the robot's
/// liveliness token.
const JOIN_CHECK_EVERY: Duration = Duration::from_secs(2);
const JOIN_CHECK_TIMEOUT: Duration = Duration::from_secs(1);

/// The liveliness token a robot's forwarder declares on its own domain.
pub fn presence_key(robot_id: &str) -> String {
    format!("robotele/forwarder/{robot_id}")
}

/// One `--zenoh-forward` rule: forward local samples matching `key_expr`
/// to the peer, in `mode`. Parsed from `<mode>:<key-expr>`, e.g.
/// `latest:robot/pose` or `reliable:planner/**`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ForwardRule {
    pub mode: TopicMode,
    pub key_expr: String,
}

impl FromStr for ForwardRule {
    type Err = anyhow::Error;

    fn from_str(s: &str) -> Result<Self> {
        let (mode, key_expr) = s.split_once(':').ok_or_else(|| anyhow!("expected <latest|reliable>:<key-expr>, got {s:?}"))?;
        let mode = TopicMode::parse(mode).ok_or_else(|| anyhow!("unknown forward mode {mode:?} (expected latest or reliable)"))?;
        zenoh::key_expr::KeyExpr::try_from(key_expr).map_err(|e| anyhow!("bad key expression {key_expr:?}: {e}"))?;
        Ok(Self { mode, key_expr: key_expr.to_string() })
    }
}

/// A local sample on its way to the peer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Outgoing {
    pub mode: TopicMode,
    pub sample: TopicSample,
}

/// Running totals, for the session-end log line.
#[derive(Debug, Default)]
pub struct ForwardStats {
    pub captured: AtomicU64,
    /// Not forwarded: the channel to the session loop was full, forwarding
    /// was paused, or the key can't be republished (`check_key`).
    pub capture_dropped: AtomicU64,
    pub republished: AtomicU64,
    /// Not republished: the channel was full or forwarding was paused.
    pub republish_dropped: AtomicU64,
}

impl ForwardStats {
    pub fn snapshot(&self) -> [u64; 4] {
        [&self.captured, &self.capture_dropped, &self.republished, &self.republish_dropped].map(|c| c.load(Ordering::Relaxed))
    }
}

pub struct Forwarder {
    session: zenoh::Session,
    outgoing_rx: mpsc::Receiver<Outgoing>,
    republish_tx: mpsc::Sender<TopicSample>,
    paused: Arc<AtomicBool>,
    stats: Arc<ForwardStats>,
    watcher: Option<tokio::task::JoinHandle<()>>,
}

impl Forwarder {
    /// Declares a subscriber per rule and starts the republishing task.
    /// Fails if a subscriber can't be declared.
    pub async fn start(session: zenoh::Session, rules: &[ForwardRule]) -> Result<Self> {
        let (outgoing_tx, outgoing_rx) = mpsc::channel(CHANNEL_DEPTH);
        let (republish_tx, mut republish_rx) = mpsc::channel::<TopicSample>(CHANNEL_DEPTH);
        let paused = Arc::new(AtomicBool::new(false));
        let stats = Arc::new(ForwardStats::default());

        for rule in rules {
            let (tx, paused, stats, mode) = (outgoing_tx.clone(), paused.clone(), stats.clone(), rule.mode);
            let subscriber = session
                .declare_subscriber(rule.key_expr.clone())
                .allowed_origin(Locality::Remote)
                .callback(move |sample| {
                    if sample.kind() != SampleKind::Put {
                        return;
                    }
                    let key = sample.key_expr().as_str();
                    if paused.load(Ordering::Relaxed) || check_key(key).is_err() {
                        stats.capture_dropped.fetch_add(1, Ordering::Relaxed);
                        return;
                    }
                    let sample = TopicSample {
                        key: key.to_string(),
                        attachment: sample.attachment().map(|a| a.to_bytes().into_owned()).unwrap_or_default(),
                        payload: sample.payload().to_bytes().into_owned(),
                    };
                    match tx.try_send(Outgoing { mode, sample }) {
                        Ok(()) => stats.captured.fetch_add(1, Ordering::Relaxed),
                        Err(_) => stats.capture_dropped.fetch_add(1, Ordering::Relaxed),
                    };
                })
                .await
                .map_err(|e| anyhow!("declaring a subscriber on {:?}: {e}", rule.key_expr))?;
            // Lives as long as the session: dropping it would undeclare it.
            std::mem::forget(subscriber);
            tracing::info!(key_expr = %rule.key_expr, mode = rule.mode.name(), "forwarding zenoh topic");
        }

        {
            let session = session.clone();
            tokio::spawn(async move {
                while let Some(sample) = republish_rx.recv().await {
                    let mut put = session.put(sample.key.clone(), sample.payload);
                    if !sample.attachment.is_empty() {
                        put = put.attachment(sample.attachment);
                    }
                    if let Err(e) = put.await {
                        tracing::warn!(key = %sample.key, error = %e, "republishing a forwarded sample failed");
                    }
                }
            });
        }

        Ok(Self { session, outgoing_rx, republish_tx, paused, stats, watcher: None })
    }

    /// The next local sample to send to the peer.
    pub async fn recv(&mut self) -> Option<Outgoing> {
        self.outgoing_rx.recv().await
    }

    /// The next local sample if one is already waiting.
    pub fn try_recv(&mut self) -> Option<Outgoing> {
        self.outgoing_rx.try_recv().ok()
    }

    /// Drops whatever local samples are waiting, e.g. ones captured while no
    /// peer was connected, so a reconnect doesn't deliver them stale.
    pub fn discard_waiting(&mut self) -> usize {
        std::iter::from_fn(|| self.try_recv()).count()
    }

    /// Puts a sample the peer forwarded on the local session. Never blocks.
    pub fn republish(&self, sample: TopicSample) {
        if self.paused.load(Ordering::Relaxed) || self.republish_tx.try_send(sample).is_err() {
            self.stats.republish_dropped.fetch_add(1, Ordering::Relaxed);
        } else {
            self.stats.republished.fetch_add(1, Ordering::Relaxed);
        }
    }

    pub fn is_paused(&self) -> bool {
        self.paused.load(Ordering::Relaxed)
    }

    pub fn stats(&self) -> &ForwardStats {
        &self.stats
    }

    /// Robot side: declares `presence_key(robot_id)` on the local session
    /// for as long as the process runs.
    pub async fn announce(&self, robot_id: &str) -> Result<()> {
        let token = self.session.liveliness().declare_token(presence_key(robot_id)).await.map_err(|e| anyhow!("declaring the presence token: {e}"))?;
        std::mem::forget(token);
        Ok(())
    }

    /// Operator side: pauses forwarding in both directions while
    /// `presence_key(robot_id)` is visible on the local session, i.e. while
    /// the local Zenoh domain is joined with that robot's. Replaces any
    /// earlier watch (a reconnect may reach a different robot).
    pub fn pause_while_visible(&mut self, robot_id: &str) {
        if let Some(w) = self.watcher.take() {
            w.abort();
        }
        self.paused.store(false, Ordering::Relaxed);
        let (session, paused, key, robot_id) = (self.session.clone(), self.paused.clone(), presence_key(robot_id), robot_id.to_string());
        self.watcher = Some(tokio::spawn(async move {
            loop {
                let visible = match session.liveliness().get(key.as_str()).timeout(JOIN_CHECK_TIMEOUT).await {
                    Ok(replies) => {
                        let mut seen = false;
                        while let Ok(reply) = replies.recv_async().await {
                            seen |= reply.result().is_ok();
                        }
                        seen
                    }
                    Err(e) => {
                        tracing::debug!(error = %e, "liveliness query failed");
                        false
                    }
                };
                if paused.swap(visible, Ordering::Relaxed) != visible {
                    if visible {
                        tracing::warn!(
                            robot_id = %robot_id,
                            "zenoh: this host's Zenoh domain is connected to the robot's directly; pausing topic forwarding so samples don't loop (see BABYROS.md, \"Forwarding topics\")"
                        );
                    } else {
                        tracing::info!(robot_id = %robot_id, "zenoh: the robot's domain is no longer visible; topic forwarding resumed");
                    }
                }
                tokio::time::sleep(JOIN_CHECK_EVERY).await;
            }
        }));
    }
}

impl Drop for Forwarder {
    fn drop(&mut self) {
        if let Some(w) = self.watcher.take() {
            w.abort();
        }
    }
}

/// A Zenoh config that only listens on `listen` and doesn't scout by
/// multicast, so the session joins no other domain unless something connects
/// to it on purpose. Used by the operator side by default.
pub fn isolated_config(listen: &str) -> Result<zenoh::Config> {
    let mut config = zenoh::Config::default();
    config.insert_json5("listen/endpoints", &format!("[{listen:?}]")).map_err(|e| anyhow!("listen endpoint {listen:?}: {e}"))?;
    config.insert_json5("scouting/multicast/enabled", "false").map_err(|e| anyhow!("{e}"))?;
    Ok(config)
}

pub async fn open_session(config: zenoh::Config) -> Result<zenoh::Session> {
    zenoh::open(config).await.map_err(|e| anyhow!("opening the zenoh session: {e}"))
}

/// Loads a Zenoh config file (JSON5), for setups `isolated_config` doesn't
/// cover (a router, TLS, ...).
pub fn config_from_file(path: &std::path::Path) -> Result<zenoh::Config> {
    zenoh::Config::from_file(path).map_err(|e| anyhow!("{e}")).with_context(|| format!("loading zenoh config {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rules_parse_mode_then_key_expr() {
        assert_eq!("latest:robot/pose".parse::<ForwardRule>().unwrap(), ForwardRule { mode: TopicMode::Latest, key_expr: "robot/pose".into() });
        assert_eq!("reliable:planner/**".parse::<ForwardRule>().unwrap().mode, TopicMode::Reliable);
        assert!("robot/pose".parse::<ForwardRule>().is_err(), "no mode");
        assert!("bulk:robot/pose".parse::<ForwardRule>().is_err(), "unknown mode");
        assert!("latest:/robot".parse::<ForwardRule>().is_err(), "leading slash");
        assert!("latest:".parse::<ForwardRule>().is_err(), "empty key expression");
    }

    #[test]
    fn isolated_config_builds() {
        isolated_config("tcp/127.0.0.1:7448").unwrap();
    }

    /// Two sessions linked by hand stand in for the two hosts' Zenoh
    /// domains, with the "QUIC link" being this test moving samples between
    /// the forwarders.
    async fn linked_pair() -> (zenoh::Session, zenoh::Session) {
        let port = {
            let l = std::net::TcpListener::bind("127.0.0.1:0").unwrap();
            l.local_addr().unwrap().port()
        };
        let a = zenoh::open(isolated_config(&format!("tcp/127.0.0.1:{port}")).unwrap()).await.unwrap();
        let mut cfg = isolated_config("tcp/127.0.0.1:0").unwrap();
        cfg.insert_json5("connect/endpoints", &format!("[\"tcp/127.0.0.1:{port}\"]")).unwrap();
        let b = zenoh::open(cfg).await.unwrap();
        (a, b)
    }

    async fn isolated() -> zenoh::Session {
        zenoh::open(isolated_config("tcp/127.0.0.1:0").unwrap()).await.unwrap()
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn forwards_remote_samples_with_attachment_but_not_its_own_republishes() {
        // `node` is a BabyROS node; `bridge` is the forwarder's session on the same host.
        let (node, bridge) = linked_pair().await;
        let mut fwd = Forwarder::start(bridge, &["reliable:demo/**".parse().unwrap()]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(300)).await;

        node.put("demo/a", vec![1u8, 2, 3]).attachment(b"codec".to_vec()).await.unwrap();
        let out = tokio::time::timeout(Duration::from_secs(3), fwd.recv()).await.expect("sample forwarded").unwrap();
        assert_eq!(out.mode, TopicMode::Reliable);
        assert_eq!(out.sample, TopicSample { key: "demo/a".into(), attachment: b"codec".to_vec(), payload: vec![1, 2, 3] });

        // A sample from the peer, republished here: the node sees it, the
        // forwarder must not send it back.
        let sub = node.declare_subscriber("demo/**").await.unwrap();
        tokio::time::sleep(Duration::from_millis(300)).await;
        fwd.republish(TopicSample { key: "demo/b".into(), attachment: vec![], payload: vec![9] });
        let got = tokio::time::timeout(Duration::from_secs(3), sub.recv_async()).await.expect("republished").unwrap();
        assert_eq!(got.key_expr().as_str(), "demo/b");
        assert_eq!(got.payload().to_bytes().as_ref(), &[9]);
        assert!(got.attachment().is_none());
        assert!(tokio::time::timeout(Duration::from_millis(500), fwd.recv()).await.is_err(), "own republish must not be forwarded");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn reserved_keys_are_not_forwarded() {
        let (node, bridge) = linked_pair().await;
        let mut fwd = Forwarder::start(bridge, &["latest:**".parse().unwrap()]).await.unwrap();
        tokio::time::sleep(Duration::from_millis(300)).await;
        node.put("robotele/xgo/autonomy_goal", vec![1u8]).await.unwrap();
        node.put("ok", vec![2u8]).await.unwrap();
        let out = tokio::time::timeout(Duration::from_secs(3), fwd.recv()).await.unwrap().unwrap();
        assert_eq!(out.sample.key, "ok");
    }

    #[tokio::test(flavor = "multi_thread")]
    async fn operator_pauses_while_the_robots_token_is_visible() {
        let (robot_side, operator_side) = linked_pair().await;
        let robot = Forwarder::start(robot_side, &[]).await.unwrap();
        robot.announce("xgo").await.unwrap();

        let mut joined = Forwarder::start(operator_side, &[]).await.unwrap();
        joined.pause_while_visible("xgo");
        let mut separate = Forwarder::start(isolated().await, &[]).await.unwrap();
        separate.pause_while_visible("xgo");

        // The first check can run before the token has propagated; the next
        // one is JOIN_CHECK_EVERY later.
        tokio::time::sleep(JOIN_CHECK_EVERY + Duration::from_millis(1500)).await;
        assert!(joined.is_paused(), "same domain as the robot");
        assert!(!separate.is_paused(), "isolated domain");
        joined.republish(TopicSample { key: "a".into(), attachment: vec![], payload: vec![] });
        assert_eq!(joined.stats().snapshot()[3], 1, "paused: republish dropped");

        joined.pause_while_visible("other-robot");
        tokio::time::sleep(JOIN_CHECK_TIMEOUT + Duration::from_millis(500)).await;
        assert!(!joined.is_paused(), "a different robot's token doesn't count");
    }
}
