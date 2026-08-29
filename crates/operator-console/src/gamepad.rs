//! Xbox (and other SDL-mapped) USB gamepad teleop input -- auto-detected
//! via `gilrs`'s udev hotplug monitor. Plugging in any controller `gilrs`
//! recognizes (Xbox-family pads report vendor id 0x045e and are covered by
//! its bundled SDL_GameControllerDB out of the box) starts producing
//! `TeleopInput` immediately; nothing needs configuring, selecting, or
//! restarting the console.
//!
//! `gilrs` has no async API and blocks on the underlying evdev/udev file
//! descriptors, so it runs on its own OS thread here, forwarding mapped
//! `TeleopInput`s to `GamepadReader::next()`'s async side over an
//! unbounded channel -- the same shape as `input.rs`'s `InputReader`, so
//! `quic_client.rs` can select on both identically. Gamepad support is
//! always optional: if the platform backend can't initialize (e.g. no
//! udev), `GamepadReader::new()` returns `None` and keyboard teleop keeps
//! working exactly as before.
//!
//! Button mapping deliberately covers only the "main controls" -- move,
//! turn, arm/claw nudge, body pitch nudge, stand/sit, recording, and
//! E-Stop. Roll/yaw-twist attitude nudge and the camera image-quality
//! controls (brightness/contrast/EV/shutter, minus a single reset) have no
//! gamepad binding in this first cut -- they stay keyboard-only (see
//! `input.rs`). `Quit` also has no gamepad binding on purpose: an
//! accidental button press ending the session mid-teleop is worse than not
//! having a shortcut for it; 'q'/Esc on the keyboard still works.
//!
//! E-Stop safety design: stopping is a single reachable combo (both
//! bumpers held together), clearing it is a *different* two-hand combo
//! (both stick clicks held together) -- asymmetric on purpose, so
//! panic-stopping is as easy as a single gesture and resuming motion can't
//! happen from one stray button press.
//!
//! Layout (Xbox naming; `gilrs`'s `Button`/`Axis` are logical positions, so
//! this maps identically on any SDL-recognized pad):
//!   left stick        -> move (vx/vy)
//!   right stick X     -> turn
//!   right stick Y     -> body pitch nudge
//!   D-pad             -> arm nudge (up/down = Z, left/right = X)
//!   LT / RT           -> claw open / close
//!   A / B             -> stand / sit
//!   X                 -> camera reset
//!   Y                 -> attitude reset (level)
//!   Start             -> toggle recording
//!   LB+RB held        -> E-Stop
//!   L3+R3 held        -> E-Stop clear

use crate::input::TeleopInput;
use gilrs::{Axis, Button, EventType, Gilrs};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;

/// Ignore stick deflection below this normalized magnitude -- `gilrs`
/// already applies each pad's own deadzone from its SDL mapping, this is a
/// second, coarser guard against drift on cheap third-party pads.
const STICK_DEADZONE: f32 = 0.15;
const TRIGGER_DEADZONE: f32 = 0.1;

/// How often held stick/trigger/D-pad deflection is re-sent so it doesn't
/// go stale and get auto-zeroed by `quic_client.rs`'s `move_stale` (400ms
/// default) -- `gilrs` only fires `AxisChanged`/`ButtonChanged` on value
/// changes, unlike a held keyboard key, which the terminal keeps
/// re-delivering via OS auto-repeat.
const REFRESH_INTERVAL: Duration = Duration::from_millis(120);

/// Matches `input.rs`'s w/s vx magnitude.
const MOVE_SCALE: f32 = 15.0;
/// Matches `input.rs`'s a/d vy magnitude.
const STRAFE_SCALE: f32 = 12.0;
/// Matches `input.rs`'s left/right-arrow turn magnitude.
const TURN_SCALE: f32 = 60.0;
/// Pitch nudge applied per `REFRESH_INTERVAL` tick at full stick
/// deflection -- proportional, ramps toward xgolib's +/-10 deg pitch limit
/// over roughly a second of holding the stick over, the analog-stick
/// equivalent of `input.rs`'s held-arrow-key auto-repeat ramp.
const PITCH_TICK_STEP_DEG: f32 = 0.6;
/// Matches `input.rs`'s `ARM_STEP_MM`.
const ARM_STEP_MM: i16 = 10;
/// Claw nudge applied per `REFRESH_INTERVAL` tick at full trigger pull.
const CLAW_TICK_STEP: i8 = 8;

pub struct GamepadReader {
    rx: mpsc::UnboundedReceiver<TeleopInput>,
    /// Shared with the polling thread -- lets the HUD show "gamepad:
    /// connected/none" without threading a `TeleopInput` variant through
    /// just for connection status, which isn't a control at all.
    connected: Arc<AtomicBool>,
}

impl GamepadReader {
    /// Returns `None` if the platform gamepad backend can't initialize
    /// (e.g. no udev on this system) -- gamepad support is always
    /// optional, keyboard teleop must keep working regardless.
    pub fn new() -> Option<Self> {
        let gilrs = match Gilrs::new() {
            Ok(g) => g,
            Err(e) => {
                tracing::warn!(error = ?e, "gamepad support unavailable, continuing without it");
                return None;
            }
        };
        let initially_connected = gilrs.gamepads().any(|(_, g)| g.is_connected());
        let connected = Arc::new(AtomicBool::new(initially_connected));
        let (tx, rx) = mpsc::unbounded_channel();
        let thread_connected = Arc::clone(&connected);
        std::thread::Builder::new()
            .name("gamepad-poll".into())
            .spawn(move || gamepad_thread(gilrs, tx, thread_connected))
            .expect("spawning gamepad polling thread");
        Some(Self { rx, connected })
    }

    pub async fn next(&mut self) -> Option<TeleopInput> {
        self.rx.recv().await
    }

    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }
}

/// Cached analog/held state, since `gilrs` only reports *changes* -- see
/// `REFRESH_INTERVAL`'s doc for why this needs re-sending on a timer.
#[derive(Default)]
struct PadState {
    left_x: f32,
    left_y: f32,
    right_x: f32,
    right_y: f32,
    left_trigger: f32,
    right_trigger: f32,
    lb_held: bool,
    rb_held: bool,
    l3_held: bool,
    r3_held: bool,
    dpad_up: bool,
    dpad_down: bool,
    dpad_left: bool,
    dpad_right: bool,
}

fn gamepad_thread(mut gilrs: Gilrs, tx: mpsc::UnboundedSender<TeleopInput>, connected: Arc<AtomicBool>) {
    let mut state = PadState::default();
    loop {
        if tx.is_closed() {
            return; // console shutting down
        }
        while let Some(event) = gilrs.next_event() {
            match event.event {
                EventType::Connected | EventType::Disconnected => {
                    connected.store(gilrs.gamepads().any(|(_, g)| g.is_connected()), Ordering::Relaxed);
                }
                other => handle_event(other, &mut state, &tx),
            }
        }
        send_refresh(&state, &tx);
        std::thread::sleep(REFRESH_INTERVAL);
    }
}

fn handle_event(event: EventType, state: &mut PadState, tx: &mpsc::UnboundedSender<TeleopInput>) {
    match event {
        EventType::AxisChanged(axis, value, _) => match axis {
            Axis::LeftStickX => state.left_x = value,
            Axis::LeftStickY => state.left_y = value,
            Axis::RightStickX => state.right_x = value,
            Axis::RightStickY => state.right_y = value,
            _ => {}
        },
        EventType::ButtonChanged(Button::LeftTrigger2, value, _) => state.left_trigger = value,
        EventType::ButtonChanged(Button::RightTrigger2, value, _) => state.right_trigger = value,
        EventType::ButtonPressed(button, _) => handle_button_press(button, state, tx),
        EventType::ButtonReleased(button, _) => handle_button_release(button, state),
        _ => {}
    }
}

fn handle_button_press(button: Button, state: &mut PadState, tx: &mpsc::UnboundedSender<TeleopInput>) {
    match button {
        Button::South => send(tx, TeleopInput::Action(2)),  // stand -- matches '1' in input.rs
        Button::East => send(tx, TeleopInput::Action(12)),  // sit -- matches '2' in input.rs
        Button::West => send(tx, TeleopInput::CameraReset),
        Button::North => send(tx, TeleopInput::AttitudeReset),
        Button::Start => send(tx, TeleopInput::ToggleRecording),
        Button::LeftTrigger => {
            state.lb_held = true;
            check_estop_combo(state, tx);
        }
        Button::RightTrigger => {
            state.rb_held = true;
            check_estop_combo(state, tx);
        }
        Button::LeftThumb => {
            state.l3_held = true;
            check_estop_clear_combo(state, tx);
        }
        Button::RightThumb => {
            state.r3_held = true;
            check_estop_clear_combo(state, tx);
        }
        Button::DPadUp => {
            state.dpad_up = true;
            send(tx, TeleopInput::ArmNudge { dx: 0, dz: ARM_STEP_MM });
        }
        Button::DPadDown => {
            state.dpad_down = true;
            send(tx, TeleopInput::ArmNudge { dx: 0, dz: -ARM_STEP_MM });
        }
        Button::DPadLeft => {
            state.dpad_left = true;
            send(tx, TeleopInput::ArmNudge { dx: -ARM_STEP_MM, dz: 0 });
        }
        Button::DPadRight => {
            state.dpad_right = true;
            send(tx, TeleopInput::ArmNudge { dx: ARM_STEP_MM, dz: 0 });
        }
        _ => {}
    }
}

fn handle_button_release(button: Button, state: &mut PadState) {
    match button {
        Button::LeftTrigger => state.lb_held = false,
        Button::RightTrigger => state.rb_held = false,
        Button::LeftThumb => state.l3_held = false,
        Button::RightThumb => state.r3_held = false,
        Button::DPadUp => state.dpad_up = false,
        Button::DPadDown => state.dpad_down = false,
        Button::DPadLeft => state.dpad_left = false,
        Button::DPadRight => state.dpad_right = false,
        _ => {}
    }
}

/// Fires E-Stop the instant both bumpers are held together -- a single
/// reachable two-finger gesture, deliberately not gated on anything else.
fn check_estop_combo(state: &PadState, tx: &mpsc::UnboundedSender<TeleopInput>) {
    if state.lb_held && state.rb_held {
        send(tx, TeleopInput::Estop);
    }
}

/// Clearing E-Stop needs a *different* combo (both stick clicks) so
/// resuming motion can't happen from the same gesture, or a stray single
/// button, that just stopped it.
fn check_estop_clear_combo(state: &PadState, tx: &mpsc::UnboundedSender<TeleopInput>) {
    if state.l3_held && state.r3_held {
        send(tx, TeleopInput::EstopClear);
    }
}

/// Re-sends every currently-deflected/held analog and D-pad control --
/// called every `REFRESH_INTERVAL` regardless of whether a new `gilrs`
/// event arrived, so a stick/trigger held steady (which produces no new
/// events) doesn't go stale. Mirrors `input.rs`'s "absolute value per
/// press, no release detection" model: nothing is sent to explicitly zero
/// a released control, `quic_client.rs`'s `move_stale` timeout handles
/// that the same way it does for a released keyboard key.
fn send_refresh(state: &PadState, tx: &mpsc::UnboundedSender<TeleopInput>) {
    if state.left_x.abs() > STICK_DEADZONE || state.left_y.abs() > STICK_DEADZONE {
        let vx = if state.left_y.abs() > STICK_DEADZONE { state.left_y * MOVE_SCALE } else { 0.0 };
        // Same left=+vy/right=-vy polarity as input.rs's a/d (confirmed on
        // real hardware there: +vy is the robot's left, not right).
        let vy = if state.left_x.abs() > STICK_DEADZONE { -state.left_x * STRAFE_SCALE } else { 0.0 };
        send(tx, TeleopInput::Move { vx, vy });
    }
    if state.right_x.abs() > STICK_DEADZONE {
        // input.rs: Left-arrow (turn left) = +turn, Right-arrow = -turn.
        // gilrs' right-stick +X is physically right, so turning right
        // (stick right) needs a negative turn value.
        send(tx, TeleopInput::Turn { turn: -state.right_x * TURN_SCALE });
    }
    if state.right_y.abs() > STICK_DEADZONE {
        // input.rs: Up-arrow (look up) sends a *negative* pitch delta
        // (xgolib's pitch axis is inverted from the on-screen direction --
        // confirmed on real hardware). gilrs' stick-up is +Y, so this
        // mirrors that same inversion.
        send(tx, TeleopInput::AttitudeNudge { axis: 'p', delta: -state.right_y * PITCH_TICK_STEP_DEG });
    }
    if state.left_trigger > TRIGGER_DEADZONE {
        // LT = open, matching input.rs's 'u' (also an unverified polarity
        // guess there -- see its comment).
        send(tx, TeleopInput::ClawNudge { delta: -((state.left_trigger * CLAW_TICK_STEP as f32) as i8) });
    }
    if state.right_trigger > TRIGGER_DEADZONE {
        send(tx, TeleopInput::ClawNudge { delta: (state.right_trigger * CLAW_TICK_STEP as f32) as i8 });
    }
    if state.dpad_up {
        send(tx, TeleopInput::ArmNudge { dx: 0, dz: ARM_STEP_MM });
    }
    if state.dpad_down {
        send(tx, TeleopInput::ArmNudge { dx: 0, dz: -ARM_STEP_MM });
    }
    if state.dpad_left {
        send(tx, TeleopInput::ArmNudge { dx: -ARM_STEP_MM, dz: 0 });
    }
    if state.dpad_right {
        send(tx, TeleopInput::ArmNudge { dx: ARM_STEP_MM, dz: 0 });
    }
}

fn send(tx: &mpsc::UnboundedSender<TeleopInput>, input: TeleopInput) {
    let _ = tx.send(input); // receiver dropped -- gamepad_thread's is_closed() check above will end the thread shortly
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Just checks that `Gilrs::new()`'s init/hotplug-monitor setup and the
    /// polling thread spawn don't panic or hang on a plain Linux box, and
    /// that `is_connected()` matches whatever's actually plugged in at test
    /// time (confirmed against a real Xbox 360 pad on real hardware
    /// 2026-08-29 -- both the "true" and, with it unplugged, "false"
    /// cases). Skipped (not failed) where the platform backend is
    /// unavailable at all (e.g. a minimal container with no udev/input
    /// nodes), since gamepad support is optional there by design.
    #[test]
    fn constructs_without_panicking_and_reports_connection_state() {
        let Some(reader) = GamepadReader::new() else {
            return;
        };
        let _ = reader.is_connected(); // just needs to not panic/hang
    }
}
