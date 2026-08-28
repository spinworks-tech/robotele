//! Operator console HUD: a full-screen ratatui terminal UI in interactive
//! mode, or a single status line in `--headless` mode (no TTY -- see
//! `scripts/smoke_test.sh`, which greps this process's stdout/stderr).
//!
//! Two things this HUD deliberately does NOT show, because the wire
//! protocol doesn't support them yet (v0):
//!   - Action *confirmation*. `robot-edge` fire-and-forgets `action_id` to
//!     the bridge subprocess; nothing echoes execution back over Channel B.
//!     The command panel shows "sent @ tick N", never "confirmed".
//!   - A "reconnecting" phase distinct from `Closed` -- `HudState::phase`
//!     stays `Closed` for the whole retry-until-success-or-quit window
//!     (see `Client::reconnect_until_success_or_quit`), with
//!     `reconnect_attempts` giving `draw_disconnected` something to show
//!     instead of a dedicated `ConnPhase` variant.

use std::collections::VecDeque;
use std::io::Write;
use std::net::SocketAddr;
use std::time::{Duration, Instant};

use ratatui::layout::{Constraint, Direction, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::text::{Line, Span};
use ratatui::widgets::{Block, Borders, Cell, Paragraph, Row, Table};
use ratatui::Frame;

/// Sliding-window rate counter for the channels panel -- not a general
/// metrics library, just enough to show an observed Hz/bytes-per-sec per
/// channel without pulling in a dependency. `record` prunes samples older
/// than `window` on every call, so the deque never grows unboundedly as
/// long as something keeps calling it (true for all three channels here,
/// even at rest -- Command sends every tick).
pub struct RateCounter {
    window: Duration,
    samples: VecDeque<(Instant, usize)>,
}

impl RateCounter {
    pub fn new(window: Duration) -> Self {
        Self { window, samples: VecDeque::new() }
    }

    pub fn record(&mut self, now: Instant, bytes: usize) {
        self.samples.push_back((now, bytes));
        while let Some(&(t, _)) = self.samples.front() {
            if now.saturating_duration_since(t) > self.window {
                self.samples.pop_front();
            } else {
                break;
            }
        }
    }

    /// (events/sec, bytes/sec) over the trailing window, as of `now`.
    /// Read-only (no pruning) so this can be called from render code that
    /// only holds `&HudState` -- `record` already keeps the deque bounded.
    pub fn rates(&self, now: Instant) -> (f64, f64) {
        let secs = self.window.as_secs_f64();
        let mut count = 0usize;
        let mut bytes = 0usize;
        for &(t, b) in &self.samples {
            if now.saturating_duration_since(t) <= self.window {
                count += 1;
                bytes += b;
            }
        }
        (count as f64 / secs, bytes as f64 / secs)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnPhase {
    AwaitingHello,
    AwaitingSessionDescribe,
    Operating,
    Closed,
}

impl ConnPhase {
    fn label(self) -> &'static str {
        match self {
            ConnPhase::AwaitingHello => "awaiting HELLO",
            ConnPhase::AwaitingSessionDescribe => "awaiting SESSION_DESCRIBE",
            ConnPhase::Operating => "operating",
            ConnPhase::Closed => "closed",
        }
    }
}

/// One of the two canned actions the keyboard can fire (see `input.rs`).
/// Tracks when it was last *sent* -- not confirmed, see module docs.
#[derive(Debug, Clone)]
pub struct ActionSlot {
    pub key: char,
    pub name: &'static str,
    pub id: u8,
    pub last_tick: Option<u64>,
    pub last_sent_at: Option<Instant>,
}

impl ActionSlot {
    fn new(key: char, name: &'static str, id: u8) -> Self {
        Self { key, name, id, last_tick: None, last_sent_at: None }
    }
}

pub struct HudState {
    pub phase: ConnPhase,
    pub peer: SocketAddr,
    pub tick_hz: u32,
    pub session_start: Instant,
    pub robot_id: Option<String>,
    pub dof_count: Option<u16>,
    /// Pre-formatted "WxH CODEC @FPSfps" from SESSION_DESCRIBE's first
    /// camera, if any -- static shape info, not the observed frame rate
    /// (that's `video_frame_rate`, in the channels panel).
    pub camera_shape: Option<String>,
    /// Set the instant `phase` transitions to `Closed` -- drives the
    /// full-screen disconnected overlay's "lost Ns ago" readout. `None`
    /// until that happens.
    pub disconnected_at: Option<Instant>,
    /// Bumped once per reconnect attempt while `phase == Closed` -- see
    /// `Client::reconnect_until_success_or_quit`. `0` until the first
    /// attempt fires.
    pub reconnect_attempts: u32,

    pub estopped: bool,
    pub rtt_ms: Option<f64>,
    pub battery: Option<u8>,
    pub roll: Option<f64>,
    pub pitch: Option<f64>,
    pub yaw: Option<f64>,
    pub motors: Vec<f32>,
    pub last_telemetry_at: Option<Instant>,

    pub move_vx: f32,
    pub move_vy: f32,
    pub turn: f32,
    pub actions: [ActionSlot; 2],

    /// Held arm/claw position (mm / 0-255) -- not a velocity, mirrors
    /// `Client.last_command`'s arm_x/arm_z/claw for display.
    pub arm_x: i16,
    pub arm_z: i16,
    pub claw: u8,
    /// Held whole-body attitude commands (deg) -- mirror
    /// `Client.last_command.attitude_r/p/y` for display. Distinct from
    /// `roll`/`pitch`/`yaw` above, which are the robot's *measured*
    /// telemetry attitude.
    pub attitude_r_cmd: f32,
    pub attitude_p_cmd: f32,
    pub attitude_y_cmd: f32,

    /// Held camera image-quality controls -- mirror
    /// `Client.camera_controls` for display. No corresponding "measured"
    /// telemetry counterpart (unlike roll/pitch/yaw above): the camera
    /// doesn't report these back, so this is the only source of truth for
    /// what's currently applied.
    pub camera_brightness: f32,
    pub camera_contrast: f32,
    pub camera_ev: f32,
    pub camera_shutter_us: u32,

    /// Channel A (video): every received datagram, and every NAL the
    /// reassembler completes -- tracked separately since one is bandwidth,
    /// the other is closer to observed fps.
    pub video_dgram_rate: RateCounter,
    pub video_frame_rate: RateCounter,
    /// Channel B outbound (Command): mirrors `Client::channel_b_seq` for
    /// display -- this is the operator's own send rate, so it's normally
    /// just `tick_hz`, but tracking it observed (not assumed) catches a
    /// stalled tick loop instead of silently assuming it's fine.
    pub command_rate: RateCounter,
    pub command_last_seq: u64,
    /// Channel B inbound (Telemetry).
    pub telemetry_rate: RateCounter,
    pub telemetry_last_seq: Option<u64>,

    /// Local recording (FR-9), toggled by `'r'` -- see
    /// `quic_client.rs`'s `DEFAULT_RECORD_CATEGORIES`.
    pub recording_active: bool,
    pub recording_started_at: Option<Instant>,
    /// Summed `CategoryStats::records_dropped` across the default
    /// categories, polled fresh each render -- the one visible signal
    /// that FR-9.3's "degrade under pressure, never block" behavior is
    /// actually happening, rather than something silently wrong.
    pub recording_dropped: u64,
}

const CHANNEL_STATS_WINDOW: Duration = Duration::from_secs(1);

impl HudState {
    pub fn new(peer: SocketAddr, tick_hz: u32) -> Self {
        Self {
            phase: ConnPhase::AwaitingHello,
            peer,
            tick_hz,
            session_start: Instant::now(),
            robot_id: None,
            camera_shape: None,
            disconnected_at: None,
            reconnect_attempts: 0,
            dof_count: None,
            estopped: false,
            rtt_ms: None,
            battery: None,
            roll: None,
            pitch: None,
            yaw: None,
            motors: Vec::new(),
            last_telemetry_at: None,
            move_vx: 0.0,
            move_vy: 0.0,
            turn: 0.0,
            actions: [ActionSlot::new('1', "stand", 2), ActionSlot::new('2', "sit", 12)],
            arm_x: 0,
            arm_z: 0,
            claw: 128, // must match quic_client.rs's ARM_CLAW_NEUTRAL
            attitude_r_cmd: 0.0,
            attitude_p_cmd: 0.0,
            attitude_y_cmd: 0.0,
            camera_brightness: 0.0,
            camera_contrast: 1.0,
            camera_ev: 0.0,
            camera_shutter_us: 0,
            video_dgram_rate: RateCounter::new(CHANNEL_STATS_WINDOW),
            video_frame_rate: RateCounter::new(CHANNEL_STATS_WINDOW),
            command_rate: RateCounter::new(CHANNEL_STATS_WINDOW),
            command_last_seq: 0,
            telemetry_rate: RateCounter::new(CHANNEL_STATS_WINDOW),
            telemetry_last_seq: None,
            recording_active: false,
            recording_started_at: None,
            recording_dropped: 0,
        }
    }

    /// Record that `action_id` was just sent on `tick` -- called right
    /// before the one-shot action_id is cleared in `on_tick`.
    pub fn record_action_sent(&mut self, action_id: u8, tick: u64) {
        if let Some(slot) = self.actions.iter_mut().find(|s| s.id == action_id) {
            slot.last_tick = Some(tick);
            slot.last_sent_at = Some(Instant::now());
        }
    }
}

/// Owns the alternate-screen terminal in interactive mode; `None` in
/// `--headless` mode, where `render` falls back to a single status line.
/// `Drop` restores the terminal (raw mode + alternate screen) even if
/// `run()` returns early via `?` -- `ratatui::init()` separately installs
/// a panic hook that restores it on panic, so both early-return and panic
/// exits leave the TTY sane, unlike the old hand-rolled enable/disable pair.
pub struct Console {
    terminal: Option<ratatui::DefaultTerminal>,
}

impl Console {
    pub fn init(headless: bool) -> Self {
        let terminal = if headless { None } else { Some(ratatui::init()) };
        Self { terminal }
    }

    pub fn render(&mut self, hud: &HudState) {
        match &mut self.terminal {
            Some(term) => {
                let _ = term.draw(|f| draw(f, hud));
            }
            None => print_status_line(hud),
        }
    }
}

impl Drop for Console {
    fn drop(&mut self) {
        if self.terminal.take().is_some() {
            let _ = ratatui::try_restore();
        }
    }
}

fn print_status_line(hud: &HudState) {
    let rtt = hud.rtt_ms.map(|v| format!("{v:.1}")).unwrap_or_else(|| "--".to_string());
    let battery = hud.battery.map(|v| v.to_string()).unwrap_or_else(|| "--".to_string());
    let rpy = match (hud.roll, hud.pitch, hud.yaw) {
        (Some(r), Some(p), Some(y)) => format!("{r:.1}/{p:.1}/{y:.1}"),
        _ => "--/--/--".to_string(),
    };
    print!(
        "\rphase={} rtt={}ms estop={} battery={}% rpy={}    ",
        hud.phase.label(),
        rtt,
        hud.estopped,
        battery,
        rpy
    );
    let _ = std::io::stdout().flush();
}

// ─── interactive layout ────────────────────────────────────────────────

const BATTERY_YELLOW_PCT: u8 = 50;
const BATTERY_RED_PCT: u8 = 20;
const RTT_YELLOW_MS: f64 = 50.0;
const RTT_RED_MS: f64 = 150.0;
const TELEMETRY_AGE_YELLOW: Duration = Duration::from_millis(300);
const TELEMETRY_AGE_RED: Duration = Duration::from_secs(1);

fn battery_color(pct: u8) -> Color {
    if pct < BATTERY_RED_PCT {
        Color::Red
    } else if pct < BATTERY_YELLOW_PCT {
        Color::Yellow
    } else {
        Color::Green
    }
}

fn rtt_color(ms: f64) -> Color {
    if ms >= RTT_RED_MS {
        Color::Red
    } else if ms >= RTT_YELLOW_MS {
        Color::Yellow
    } else {
        Color::Green
    }
}

fn age_color(age: Duration) -> Color {
    if age >= TELEMETRY_AGE_RED {
        Color::Red
    } else if age >= TELEMETRY_AGE_YELLOW {
        Color::Yellow
    } else {
        Color::Green
    }
}

fn draw(f: &mut Frame, hud: &HudState) {
    if hud.phase == ConnPhase::Closed {
        draw_disconnected(f, hud);
        return;
    }

    let root = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(4), // header
            Constraint::Length(3), // e-stop banner
            Constraint::Min(6),    // body
            Constraint::Length(6), // channels panel
            Constraint::Length(1), // footer
        ])
        .split(f.area());

    draw_header(f, root[0], hud);
    draw_estop_banner(f, root[1], hud);
    draw_body(f, root[2], hud);
    draw_channels_panel(f, root[3], hud);
    draw_footer(f, root[4]);
}

/// Takes over the whole screen once the connection closes for any reason
/// (idle timeout, peer close, network drop) -- deliberately replaces the
/// normal HUD entirely rather than leaving stale telemetry/motion panels
/// on screen looking current when they aren't. Before this existed,
/// `Client::run` just returned `Ok(())` on `conn.is_closed()`, silently
/// exiting the whole process the instant the connection died -- on a
/// robot-control console, "the window just vanished" reads as a crash,
/// not a status update. Now the process stays up and retries indefinitely
/// in the background (see `Client::reconnect_until_success_or_quit`) while
/// still honoring an explicit 'q' -- a lost connection is something the
/// operator sees and can choose to wait out, not something that happens
/// to them.
fn draw_disconnected(f: &mut Frame, hud: &HudState) {
    let elapsed = hud.disconnected_at.map(|t| t.elapsed().as_secs_f64()).unwrap_or(0.0);
    let robot = hud.robot_id.as_deref().unwrap_or("(unknown)");
    let battery = hud.battery.map(|b| format!("{b}%")).unwrap_or_else(|| "unknown".to_string());
    let lines = vec![
        Line::from(""),
        Line::from(Span::styled(
            "  CONNECTION LOST  ",
            Style::default().bg(Color::Red).fg(Color::White).add_modifier(Modifier::BOLD),
        )),
        Line::from(""),
        Line::from(format!("Lost contact with {robot} {elapsed:.0}s ago -- robot state is no longer known.")),
        Line::from("This console has no control over the robot until you reconnect."),
        Line::from(""),
        Line::from(format!("Last known: battery {battery}, estop {}", if hud.estopped { "LATCHED" } else { "clear" })),
        Line::from(""),
        Line::from(format!("Reconnecting... (attempt {})", hud.reconnect_attempts)),
        Line::from(""),
        Line::from(Span::styled("Press 'q' to give up and quit.", Style::default().add_modifier(Modifier::BOLD))),
    ];
    let block = Block::default().borders(Borders::ALL).title("disconnected");
    f.render_widget(Paragraph::new(lines).block(block).alignment(ratatui::layout::Alignment::Center), f.area());
}

/// "● REC 12s (3 dropped)" while active, a dim reminder of the key while
/// not -- dropped-record count is the one visible signal that FR-9.3's
/// "degrade under pressure, never block" behavior is actually happening.
fn recording_span(hud: &HudState) -> Span<'static> {
    if hud.recording_active {
        let elapsed = hud.recording_started_at.map(|t| t.elapsed().as_secs()).unwrap_or(0);
        let dropped = if hud.recording_dropped > 0 { format!(" ({} dropped)", hud.recording_dropped) } else { String::new() };
        Span::styled(format!("\u{25cf} REC {elapsed}s{dropped}"), Style::default().fg(Color::Red).add_modifier(Modifier::BOLD))
    } else {
        Span::styled("rec: off ('r' to start)", Style::default().fg(Color::DarkGray))
    }
}

fn draw_header(f: &mut Frame, area: Rect, hud: &HudState) {
    let elapsed = hud.session_start.elapsed();
    let robot_line = match (&hud.robot_id, hud.dof_count) {
        (Some(id), Some(dof)) => format!("robot={id} dof={dof}"),
        _ => "robot=(pending SESSION_DESCRIBE)".to_string(),
    };
    let text = vec![
        Line::from(vec![
            Span::styled("RoboProtocol operator-console", Style::default().add_modifier(Modifier::BOLD)),
            Span::raw("  "),
            Span::styled(hud.phase.label(), Style::default().fg(Color::Cyan)),
            Span::raw("  "),
            recording_span(hud),
        ]),
        Line::from(format!(
            "{robot_line}  peer={} tick={}Hz  session={}s",
            hud.peer,
            hud.tick_hz,
            elapsed.as_secs()
        )),
    ];
    f.render_widget(Paragraph::new(text).block(Block::default().borders(Borders::ALL)), area);
}

fn draw_estop_banner(f: &mut Frame, area: Rect, hud: &HudState) {
    let (text, style) = if hud.estopped {
        ("E-STOPPED -- press 'c' to clear", Style::default().bg(Color::Red).fg(Color::White).add_modifier(Modifier::BOLD))
    } else {
        ("armed -- press 'e' to E-STOP", Style::default().fg(Color::Green))
    };
    f.render_widget(
        Paragraph::new(text).style(style).block(Block::default().borders(Borders::ALL)).alignment(ratatui::layout::Alignment::Center),
        area,
    );
}

fn draw_body(f: &mut Frame, area: Rect, hud: &HudState) {
    let cols = Layout::default()
        .direction(Direction::Horizontal)
        .constraints([Constraint::Percentage(60), Constraint::Percentage(40)])
        .split(area);
    draw_telemetry_panel(f, cols[0], hud);
    draw_command_panel(f, cols[1], hud);
}

/// Leg names/order match `xgo_profile.rs`'s `LEG_NAMES` and its "wire order
/// is leg-major/position-minor" comment: `hud.motors[0..3]` is front-left's
/// lower/middle/upper, `[3..6]` front-right, `[6..9]` rear-right, `[9..12]`
/// rear-left. Abbreviated here purely for grid-cell width, not a
/// reinterpretation of that ordering.
const LEG_ABBREVIATIONS: [&str; 4] = ["FL", "FR", "RR", "RL"];

fn draw_telemetry_panel(f: &mut Frame, area: Rect, hud: &HudState) {
    let block = Block::default().borders(Borders::ALL).title("telemetry");
    let inner = block.inner(area);
    f.render_widget(block, area);

    let battery_span = match hud.battery {
        Some(b) => Span::styled(format!("{b}%"), Style::default().fg(battery_color(b))),
        None => Span::styled("no data", Style::default().fg(Color::DarkGray)),
    };
    let rtt_span = match hud.rtt_ms {
        Some(ms) => Span::styled(format!("{ms:.1} ms"), Style::default().fg(rtt_color(ms))),
        None => Span::styled("--", Style::default().fg(Color::DarkGray)),
    };
    let age_span = match hud.last_telemetry_at {
        Some(t) => {
            let age = t.elapsed();
            Span::styled(format!("{:.1}s ago", age.as_secs_f64()), Style::default().fg(age_color(age)))
        }
        None => Span::styled("no data yet", Style::default().fg(Color::Red)),
    };
    let rpy = match (hud.roll, hud.pitch, hud.yaw) {
        (Some(r), Some(p), Some(y)) => format!("{r:.1} / {p:.1} / {y:.1}"),
        _ => "--/--/--".to_string(),
    };
    let summary_lines = vec![
        Line::from(vec![Span::raw("battery:    "), battery_span]),
        Line::from(vec![Span::raw("rtt:        "), rtt_span]),
        Line::from(vec![Span::raw("last recv:  "), age_span]),
        Line::from(format!("roll/pitch/yaw: {rpy}")),
    ];

    // Fewer than 12 motors means no/partial telemetry yet (this robot's
    // profile is always exactly 12 -- legs only -- or 15 -- legs + arm, per
    // `xgo_profile.rs`'s `dof_count`) -- summary-only in that case, nothing
    // to grid.
    if hud.motors.len() < 12 {
        f.render_widget(Paragraph::new(summary_lines), inner);
        return;
    }

    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([
            Constraint::Length(4),                              // battery/rtt/last-recv/rpy
            Constraint::Length(1),                               // spacer
            Constraint::Length(3),                               // leg grid row 1 (FL/FR)
            Constraint::Length(3),                               // leg grid row 2 (RR/RL)
            Constraint::Length(if hud.motors.len() > 12 { 1 } else { 0 }), // arm, if attached
        ])
        .split(inner);
    f.render_widget(Paragraph::new(summary_lines), rows[0]);

    for (row_idx, row_area) in [rows[2], rows[3]].into_iter().enumerate() {
        let cols = Layout::default()
            .direction(Direction::Horizontal)
            .constraints([Constraint::Percentage(50), Constraint::Percentage(50)])
            .split(row_area);
        for (col_idx, cell_area) in cols.iter().enumerate() {
            let leg = row_idx * 2 + col_idx;
            let base = leg * 3;
            let (lower, middle, upper) = (hud.motors[base], hud.motors[base + 1], hud.motors[base + 2]);
            let cell = Paragraph::new(Line::from(format!("L{lower:>6.1} M{middle:>6.1} U{upper:>6.1}")))
                .block(Block::default().borders(Borders::ALL).title(LEG_ABBREVIATIONS[leg]));
            f.render_widget(cell, *cell_area);
        }
    }

    if hud.motors.len() > 12 {
        let arm: Vec<String> = hud.motors[12..].iter().map(|m| format!("{m:.1}")).collect();
        f.render_widget(Paragraph::new(format!("arm: {}", arm.join(" / "))), rows[4]);
    }
}

fn draw_command_panel(f: &mut Frame, area: Rect, hud: &HudState) {
    let block = Block::default().borders(Borders::ALL).title("commands");
    let inner = block.inner(area);
    f.render_widget(block, area);

    let rows = Layout::default()
        .direction(Direction::Vertical)
        .constraints([Constraint::Length(6), Constraint::Min(3)])
        .split(inner);

    let held_lines = vec![
        Line::from(format!("vx={:.1} vy={:.1} turn={:.1}", hud.move_vx, hud.move_vy, hud.turn)),
        Line::from(format!("arm x={} z={} claw={}", hud.arm_x, hud.arm_z, hud.claw)),
        Line::from(format!(
            "attitude roll={:.1} (+/-20) pitch={:.1} (+/-10) yaw={:.1} (+/-12) deg",
            hud.attitude_r_cmd, hud.attitude_p_cmd, hud.attitude_y_cmd
        )),
        Line::from(format!(
            "camera brightness={:.1} contrast={:.1} ev={:.1} shutter={}",
            hud.camera_brightness,
            hud.camera_contrast,
            hud.camera_ev,
            if hud.camera_shutter_us == 0 { "auto".to_string() } else { format!("{}us", hud.camera_shutter_us) }
        )),
    ];
    f.render_widget(Paragraph::new(held_lines).block(Block::default().borders(Borders::ALL).title("motion / arm (held)")), rows[0]);

    let header = Row::new(vec![Cell::from("key"), Cell::from("action"), Cell::from("last sent")]);
    let action_rows: Vec<Row> = hud
        .actions
        .iter()
        .map(|slot| {
            let sent = match (slot.last_tick, slot.last_sent_at) {
                (Some(tick), Some(at)) => format!("tick {tick}, {:.1}s ago", at.elapsed().as_secs_f64()),
                _ => "-- (never sent)".to_string(),
            };
            Row::new(vec![Cell::from(slot.key.to_string()), Cell::from(slot.name), Cell::from(sent)])
        })
        .collect();
    let table = Table::new(action_rows, [Constraint::Length(4), Constraint::Length(8), Constraint::Min(10)])
        .header(header)
        .block(Block::default().title("sent — robot does not currently ack actions"));
    f.render_widget(table, rows[1]);
}

fn fmt_rate(bytes_per_sec: f64) -> String {
    if bytes_per_sec >= 1024.0 * 1024.0 {
        format!("{:.1} MB/s", bytes_per_sec / (1024.0 * 1024.0))
    } else if bytes_per_sec >= 1024.0 {
        format!("{:.1} KB/s", bytes_per_sec / 1024.0)
    } else {
        format!("{bytes_per_sec:.0} B/s")
    }
}

fn draw_channels_panel(f: &mut Frame, area: Rect, hud: &HudState) {
    let now = Instant::now();
    let header = Row::new(vec![
        Cell::from("channel"),
        Cell::from("hz"),
        Cell::from("rate"),
        Cell::from("seq"),
        Cell::from("note"),
    ]);

    let (video_dgram_hz, video_dgram_bps) = hud.video_dgram_rate.rates(now);
    let (video_frame_hz, _) = hud.video_frame_rate.rates(now);
    let (command_hz, command_bps) = hud.command_rate.rates(now);
    let (telemetry_hz, telemetry_bps) = hud.telemetry_rate.rates(now);

    let telemetry_age = match hud.last_telemetry_at {
        Some(t) => format!("age {:.1}s", t.elapsed().as_secs_f64()),
        None => "no data yet".to_string(),
    };
    let video_note = match &hud.camera_shape {
        Some(shape) => format!("{shape}, {video_frame_hz:.1} fps live"),
        None => format!("{video_frame_hz:.1} fps"),
    };

    let rows = vec![
        Row::new(vec![
            Cell::from("A (video)"),
            Cell::from(format!("{video_dgram_hz:.1}")),
            Cell::from(fmt_rate(video_dgram_bps)),
            Cell::from("--"),
            Cell::from(video_note),
        ]),
        Row::new(vec![
            Cell::from("B (command)"),
            Cell::from(format!("{command_hz:.1}")),
            Cell::from(fmt_rate(command_bps)),
            Cell::from(hud.command_last_seq.to_string()),
            Cell::from(format!("target {}Hz", hud.tick_hz)),
        ]),
        Row::new(vec![
            Cell::from("B (telemetry)"),
            Cell::from(format!("{telemetry_hz:.1}")),
            Cell::from(fmt_rate(telemetry_bps)),
            Cell::from(hud.telemetry_last_seq.map(|s| s.to_string()).unwrap_or_else(|| "--".to_string())),
            Cell::from(telemetry_age),
        ]),
    ];

    let table = Table::new(
        rows,
        [Constraint::Length(14), Constraint::Length(6), Constraint::Length(11), Constraint::Length(10), Constraint::Min(14)],
    )
    .header(header)
    .block(Block::default().borders(Borders::ALL).title("channels (1s window)"));
    f.render_widget(table, area);
}

fn draw_footer(f: &mut Frame, area: Rect) {
    let text = "w/a/s/d move | left/right turn | up/down (or =/-) pitch | [/] roll | ,/. yaw | 0 level attitude | b/B brightness f/F contrast v/V ev h/H shutter | 9 reset camera | space stop | i/j/k/l arm | u/o claw | 1 stand | 2 sit | e E-Stop | c clear | r record | q quit";
    f.render_widget(Paragraph::new(text).style(Style::default().fg(Color::DarkGray)), area);
}

#[cfg(test)]
mod telemetry_grid_tests {
    use super::*;
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;
    use std::net::SocketAddr;

    fn hud_with_motors(n: usize) -> HudState {
        let mut hud = HudState::new("127.0.0.1:4433".parse::<SocketAddr>().unwrap(), 50);
        hud.motors = (0..n).map(|i| i as f32 * 1.5 - 10.0).collect();
        hud.battery = Some(80);
        hud.roll = Some(1.2);
        hud.pitch = Some(-0.5);
        hud.yaw = Some(3.3);
        hud
    }

    #[test]
    fn renders_12_motor_leg_grid_without_panicking() {
        let hud = hud_with_motors(12);
        let backend = TestBackend::new(60, 20);
        let mut terminal = Terminal::new(backend).unwrap();
        terminal.draw(|f| draw_telemetry_panel(f, f.area(), &hud)).unwrap();
        let content = terminal.backend().buffer().content.iter().map(|c| c.symbol()).collect::<String>();
        assert!(content.contains("FL"), "expected FL leg label, got:\n{content}");
        assert!(content.contains("RL"), "expected RL leg label, got:\n{content}");
    }

    #[test]
    fn renders_15_motor_leg_grid_plus_arm_line_without_panicking() {
        let hud = hud_with_motors(15);
        let backend = TestBackend::new(60, 20);
        let mut terminal = Terminal::new(backend).unwrap();
        terminal.draw(|f| draw_telemetry_panel(f, f.area(), &hud)).unwrap();
        let content = terminal.backend().buffer().content.iter().map(|c| c.symbol()).collect::<String>();
        assert!(content.contains("arm:"), "expected an arm summary line, got:\n{content}");
    }

    #[test]
    fn renders_summary_only_when_motors_below_12_without_panicking() {
        let hud = hud_with_motors(0);
        let backend = TestBackend::new(60, 20);
        let mut terminal = Terminal::new(backend).unwrap();
        terminal.draw(|f| draw_telemetry_panel(f, f.area(), &hud)).unwrap();
    }

    #[test]
    fn renders_correctly_even_in_a_short_terminal() {
        // Regression guard: the grid needs ~13 rows; a shorter area must not panic.
        let hud = hud_with_motors(15);
        let backend = TestBackend::new(60, 8);
        let mut terminal = Terminal::new(backend).unwrap();
        terminal.draw(|f| draw_telemetry_panel(f, f.area(), &hud)).unwrap();
    }
}
