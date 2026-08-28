//! Spawns and supervises the `xgo_bridge.py` subprocess.
//!
//! An unexpected process exit is a distinct failure mode from network
//! loss (see `xgo_bridge.py`'s module docstring) and is reported to the
//! caller as `SupervisorEvent::ProcessDied` / `GaveUp` so the safety task
//! can force E-Stop -- this module does not itself touch the safety
//! state machine, keeping the process-supervision and safety-decision
//! concerns separate.

use std::path::PathBuf;
use std::process::Stdio;
use std::time::{Duration, Instant};

use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::process::Command;
use tokio::sync::mpsc;

use super::protocol::{BridgeCommand, BridgeEvent};

#[derive(Clone)]
pub struct BridgeConfig {
    pub python_bin: String,
    pub script_path: PathBuf,
    /// e.g. `["--port", "/dev/ttyAMA0", "--watchdog-ms", "150"]` or `["--stub"]`.
    pub extra_args: Vec<String>,
    pub max_restart_attempts: u32,
    pub restart_window: Duration,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            python_bin: "python3".to_string(),
            script_path: PathBuf::from("xgo_bridge/xgo_bridge.py"),
            extra_args: Vec::new(),
            max_restart_attempts: 3,
            restart_window: Duration::from_secs(60),
        }
    }
}

#[derive(Debug)]
pub enum SupervisorEvent {
    FromBridge(BridgeEvent),
    /// The subprocess exited unexpectedly. `will_retry` is false once
    /// `max_restart_attempts` within `restart_window` has been exhausted
    /// (a `GaveUp` event follows immediately after).
    ProcessDied { attempt: u32, will_retry: bool },
    GaveUp,
}

pub struct BridgeSupervisor {
    pub cmd_tx: mpsc::UnboundedSender<BridgeCommand>,
    pub event_rx: mpsc::UnboundedReceiver<SupervisorEvent>,
}

impl BridgeSupervisor {
    pub fn spawn(config: BridgeConfig) -> Self {
        let (cmd_tx, cmd_rx) = mpsc::unbounded_channel();
        let (event_tx, event_rx) = mpsc::unbounded_channel();
        tokio::spawn(supervisor_task(config, cmd_rx, event_tx));
        Self { cmd_tx, event_rx }
    }
}

async fn supervisor_task(
    config: BridgeConfig,
    mut cmd_rx: mpsc::UnboundedReceiver<BridgeCommand>,
    event_tx: mpsc::UnboundedSender<SupervisorEvent>,
) {
    let mut attempts: u32 = 0;
    let mut window_start = Instant::now();

    loop {
        let mut child = match Command::new(&config.python_bin)
            .arg(&config.script_path)
            .args(&config.extra_args)
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .kill_on_drop(true)
            .spawn()
        {
            Ok(child) => child,
            Err(e) => {
                tracing::error!(error = %e, "failed to spawn xgo_bridge.py");
                if !record_failure_and_maybe_give_up(&mut attempts, &mut window_start, &config, &event_tx) {
                    return;
                }
                tokio::time::sleep(backoff_for(attempts)).await;
                continue;
            }
        };

        let mut stdin = child.stdin.take().expect("piped stdin");
        let stdout = child.stdout.take().expect("piped stdout");
        let mut lines = BufReader::new(stdout).lines();

        let died_unexpectedly = loop {
            tokio::select! {
                maybe_cmd = cmd_rx.recv() => {
                    match maybe_cmd {
                        Some(cmd) => {
                            let mut line = match cmd.to_line() {
                                Ok(l) => l,
                                Err(e) => { tracing::error!(error = %e, "failed to serialize bridge command"); continue; }
                            };
                            line.push('\n');
                            if let Err(e) = stdin.write_all(line.as_bytes()).await {
                                tracing::warn!(error = %e, "bridge stdin write failed, treating as crash");
                                break true;
                            }
                        }
                        // Caller dropped the sender -- deliberate shutdown, not a crash.
                        None => {
                            let _ = child.start_kill();
                            let _ = child.wait().await;
                            return;
                        }
                    }
                }
                line = lines.next_line() => {
                    match line {
                        Ok(Some(l)) => match BridgeEvent::from_line(&l) {
                            Ok(ev) => { let _ = event_tx.send(SupervisorEvent::FromBridge(ev)); }
                            Err(e) => tracing::warn!(error = %e, line = %l, "malformed line from xgo_bridge.py"),
                        },
                        Ok(None) => break true,  // stdout EOF: bridge process is going away
                        Err(e) => { tracing::warn!(error = %e, "error reading xgo_bridge.py stdout"); break true; }
                    }
                }
                status = child.wait() => {
                    tracing::warn!(?status, "xgo_bridge.py exited");
                    break true;
                }
            }
        };

        if died_unexpectedly {
            let _ = child.start_kill();
            let _ = child.wait().await;
            if !record_failure_and_maybe_give_up(&mut attempts, &mut window_start, &config, &event_tx) {
                return;
            }
            tokio::time::sleep(backoff_for(attempts)).await;
        }
    }
}

fn backoff_for(attempt: u32) -> Duration {
    Duration::from_millis(200 * 2u64.pow(attempt.min(5)))
}

/// Returns `false` if the supervisor should give up entirely (after
/// emitting `GaveUp`); otherwise emits `ProcessDied` and sleeps a backoff
/// before the caller retries.
fn record_failure_and_maybe_give_up(
    attempts: &mut u32,
    window_start: &mut Instant,
    config: &BridgeConfig,
    event_tx: &mpsc::UnboundedSender<SupervisorEvent>,
) -> bool {
    if window_start.elapsed() > config.restart_window {
        *attempts = 0;
        *window_start = Instant::now();
    }
    *attempts += 1;

    if *attempts > config.max_restart_attempts {
        let _ = event_tx.send(SupervisorEvent::GaveUp);
        return false;
    }

    let _ = event_tx.send(SupervisorEvent::ProcessDied { attempt: *attempts, will_retry: true });
    true
}
