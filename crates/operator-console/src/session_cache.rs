//! Per-server-address 0-RTT session ticket persistence, plus a small
//! `(robot_id, profile_hash)` cache used to make `SESSION_ACCEPT`'s
//! `cached` flag honest instead of the hardcoded `false` it used to be.
//!
//! Both live under `.session-cache/` (gitignored -- this is private
//! per-machine runtime state, not the demo certs in `certs/`). Keyed by
//! the connect *address*, not a robot identity: if the Pi reboots and
//! gets a new IP (as it did three times in one evening of testing), the
//! new address correctly has no cached ticket and falls back to a normal
//! full handshake -- a stale ticket for a now-dead server process would
//! fail anyway, this just skips ever offering it.

use std::fs;
use std::net::SocketAddr;
use std::path::PathBuf;

const CACHE_DIR: &str = ".session-cache";

fn cache_dir() -> PathBuf {
    PathBuf::from(CACHE_DIR)
}

fn sanitize_addr(addr: SocketAddr) -> String {
    addr.to_string().replace([':', '.'], "_")
}

fn ticket_path(connect: SocketAddr) -> PathBuf {
    cache_dir().join(format!("{}.ticket", sanitize_addr(connect)))
}

pub fn load_ticket(connect: SocketAddr) -> Option<Vec<u8>> {
    fs::read(ticket_path(connect)).ok()
}

pub fn save_ticket(connect: SocketAddr, session: &[u8]) {
    let _ = fs::create_dir_all(cache_dir());
    let _ = fs::write(ticket_path(connect), session);
}

/// Removes a cached ticket that turned out to be stale (e.g. the resumed
/// handshake it was offered for never established) so it isn't offered
/// again on the next attempt.
pub fn evict_ticket(connect: SocketAddr) {
    let _ = fs::remove_file(ticket_path(connect));
}

fn profile_cache_path() -> PathBuf {
    cache_dir().join("profiles.txt")
}

/// Returns `true` if `(robot_id, profile_hash)` was already seen in a
/// prior session (a real cache hit -- SESSION_ACCEPT.cached should say
/// so), and records it either way so the *next* session sees it too.
pub fn check_and_record_profile(robot_id: &str, profile_hash: u64) -> bool {
    let path = profile_cache_path();
    let key = format!("{robot_id}:{profile_hash}");
    let existing = fs::read_to_string(&path).unwrap_or_default();
    let hit = existing.lines().any(|line| line == key);
    if !hit {
        let _ = fs::create_dir_all(cache_dir());
        let mut updated = existing;
        if !updated.is_empty() && !updated.ends_with('\n') {
            updated.push('\n');
        }
        updated.push_str(&key);
        updated.push('\n');
        let _ = fs::write(&path, updated);
    }
    hit
}
