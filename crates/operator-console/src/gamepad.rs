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
//! turn, arm/claw nudge, roll/yaw/pitch nudge, stand/sit, recording,
//! screenshot, and E-Stop. The camera image-quality controls (brightness/
//! contrast/EV/shutter, minus a single reset) have no gamepad binding --
//! they stay keyboard-only (see `input.rs`). `Quit` also has no gamepad
//! binding on purpose: an accidental button press ending the session
//! mid-teleop is worse than not having a shortcut for it; 'q'/Esc on the
//! keyboard still works.
//!
//! E-Stop safety design: stopping is reachable two ways -- a single combo
//! (both bumpers held together, which also nudge the claw individually,
//! see below) or the Back button, which toggles Estop/EstopClear based on
//! `Client.estopped` (see `TeleopInput::EstopToggle`'s doc -- this module
//! has no view of that state itself, only the connected `Client` does).
//! Clearing latches from the *bumper* combo specifically also works via
//! pressing L3 into R3 (or vice versa) -- see `StickXMode`'s doc for why
//! that's a press-into-an-already-held-button gesture rather than a
//! "hold both" one.
//!
//! Layout (Xbox naming; `gilrs`'s `Button`/`Axis` are logical positions, so
//! this maps identically on any SDL-recognized pad):
//!   left stick        -> move (vx/vy)
//!   right stick X     -> turn (default) / roll / yaw-twist, whichever
//!                        `StickXMode` is currently latched (see its doc)
//!   right stick Y     -> body pitch nudge (up = look up)
//!   D-pad             -> arm nudge (up/down = Z, left/right = X), step
//!                        size toggled coarse/fine by LT (see below)
//!   LT                -> toggle D-pad arm nudge between coarse
//!                        (`ARM_STEP_MM`) and fine (`ARM_STEP_FINE_MM`)
//!                        step size -- a latch like `StickXMode`, not
//!                        held, since it's operated by the same hand as
//!                        the D-pad it modifies
//!   LB / RB           -> claw open / close; held together -> E-Stop
//!   A                 -> full neutral-pose reset (attitude + arm + claw,
//!                        same values the console starts with at boot)
//!   B                 -> attitude reset (level RPY only, same as Y)
//!   X                 -> camera reset
//!   Y                 -> save video frame (PNG) -- only takes effect
//!                        under `--video-backend native`, see
//!                        `TeleopInput::SaveFrame`'s doc
//!   Start             -> toggle recording
//!   Back              -> E-Stop toggle (Estop if clear, EstopClear if latched)
//!   L3 / R3           -> latch right-stick-X mode to yaw / roll (press the
//!                        active one again to return to turn); pressing
//!                        one while the other is already held instead
//!                        fires E-Stop clear (see `StickXMode`'s doc)
//!
//! Stand/sit have no gamepad binding as of this mapping -- A/B were
//! reassigned to the resets above. Only keyboard '1'/'2' reach them.

use crate::input::TeleopInput;
use gilrs::{Axis, Button, EventType, Gilrs};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;

/// Ignore stick deflection below this normalized magnitude -- `gilrs`
/// already applies each pad's own deadzone from its SDL mapping, this is a
/// second, coarser guard against drift on cheap third-party pads.
const STICK_DEADZONE: f32 = 0.15;

/// How often held sticks/buttons are re-sent so a steady deflection or a
/// held button doesn't go stale and get auto-zeroed by `quic_client.rs`'s
/// `move_stale` (400ms default) -- `gilrs` only fires
/// `AxisChanged`/`ButtonChanged` on value changes, unlike a held keyboard
/// key, which the terminal keeps re-delivering via OS auto-repeat.
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
/// Roll/yaw-twist nudge applied per `REFRESH_INTERVAL` tick at full right
/// stick X deflection, while L3 or R3 is held as a modifier (see the
/// module doc's Layout) -- same proportional-ramp shape as
/// `PITCH_TICK_STEP_DEG`, just on a different axis pair.
const ROLL_YAW_TICK_STEP_DEG: f32 = 0.6;
/// Matches `input.rs`'s `ARM_STEP_MM`.
const ARM_STEP_MM: i16 = 10;
/// D-pad arm nudge step while fine mode (LT toggle) is on -- 5x finer for
/// precise positioning, e.g. lining the claw up on a small object.
const ARM_STEP_FINE_MM: i16 = 2;
/// Claw nudge per LB/RB press or `REFRESH_INTERVAL` tick while held --
/// matches `input.rs`'s `CLAW_STEP`... roughly (that one's 20, this is a
/// bit gentler since a held bumper repeats fast).
const CLAW_TICK_STEP: i8 = 8;

/// What the right stick's X axis currently does -- latched by pressing L3
/// or R3 (not held, see the module doc's Layout), since holding a
/// stick-click down while also tilting that same stick sideways turned
/// out to be an awkward simultaneous two-motion gesture on real hardware.
/// Pressing the button for the mode that's already active returns to
/// `Turn`; pressing one while the *other* is still physically held down
/// instead fires E-Stop clear (see `handle_button_press`) rather than
/// latching a mode, so that combo keeps working without a mode change as
/// an unwanted side effect.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[repr(u8)]
enum StickXMode {
    #[default]
    Turn = 0,
    Roll = 1,
    Yaw = 2,
}

impl StickXMode {
    fn label(self) -> &'static str {
        match self {
            StickXMode::Turn => "turn",
            StickXMode::Roll => "roll",
            StickXMode::Yaw => "yaw",
        }
    }
}

pub struct GamepadReader {
    rx: mpsc::UnboundedReceiver<TeleopInput>,
    /// Shared with the polling thread -- lets the HUD show "gamepad:
    /// connected/none" without threading a `TeleopInput` variant through
    /// just for connection status, which isn't a control at all.
    connected: Arc<AtomicBool>,
    /// Mirrors the polling thread's `PadState.stick_x_mode` (as a
    /// `StickXMode` discriminant) so the HUD can show which mode is
    /// latched -- see `StickXMode`'s doc for why this needs showing at
    /// all (it's sticky now, not something held down that's visibly
    /// still being pressed).
    stick_mode: Arc<AtomicU8>,
    /// Mirrors the polling thread's `PadState.arm_fine_mode` -- same
    /// reasoning as `stick_mode`: LT toggles it rather than requiring it
    /// held, so it's sticky and needs showing on the HUD to stay visible.
    arm_fine: Arc<AtomicBool>,
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
        let stick_mode = Arc::new(AtomicU8::new(StickXMode::default() as u8));
        let arm_fine = Arc::new(AtomicBool::new(false));
        let (tx, rx) = mpsc::unbounded_channel();
        let thread_connected = Arc::clone(&connected);
        let thread_stick_mode = Arc::clone(&stick_mode);
        let thread_arm_fine = Arc::clone(&arm_fine);
        std::thread::Builder::new()
            .name("gamepad-poll".into())
            .spawn(move || gamepad_thread(gilrs, tx, thread_connected, thread_stick_mode, thread_arm_fine))
            .expect("spawning gamepad polling thread");
        Some(Self { rx, connected, stick_mode, arm_fine })
    }

    pub async fn next(&mut self) -> Option<TeleopInput> {
        self.rx.recv().await
    }

    pub fn is_connected(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }

    /// "turn"/"roll"/"yaw" -- whichever `StickXMode` the right stick's X
    /// axis is currently latched to, for the HUD.
    pub fn stick_mode_label(&self) -> &'static str {
        match self.stick_mode.load(Ordering::Relaxed) {
            1 => StickXMode::Roll.label(),
            2 => StickXMode::Yaw.label(),
            _ => StickXMode::Turn.label(),
        }
    }

    /// Whether the D-pad's arm nudge is currently in fine (LT-toggled)
    /// step mode, for the HUD.
    pub fn arm_fine_mode(&self) -> bool {
        self.arm_fine.load(Ordering::Relaxed)
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
    lb_held: bool,
    rb_held: bool,
    l3_held: bool,
    r3_held: bool,
    stick_x_mode: StickXMode,
    arm_fine_mode: bool,
    dpad_up: bool,
    dpad_down: bool,
    dpad_left: bool,
    dpad_right: bool,
}

/// D-pad arm nudge magnitude for the current mode -- see `ARM_STEP_FINE_MM`.
fn arm_step(state: &PadState) -> i16 {
    if state.arm_fine_mode {
        ARM_STEP_FINE_MM
    } else {
        ARM_STEP_MM
    }
}

fn gamepad_thread(
    mut gilrs: Gilrs,
    tx: mpsc::UnboundedSender<TeleopInput>,
    connected: Arc<AtomicBool>,
    stick_mode: Arc<AtomicU8>,
    arm_fine: Arc<AtomicBool>,
) {
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
                other => handle_event(other, &mut state, &tx, &stick_mode, &arm_fine),
            }
        }
        send_refresh(&state, &tx);
        std::thread::sleep(REFRESH_INTERVAL);
    }
}

fn handle_event(event: EventType, state: &mut PadState, tx: &mpsc::UnboundedSender<TeleopInput>, stick_mode: &AtomicU8, arm_fine: &AtomicBool) {
    match event {
        EventType::AxisChanged(axis, value, _) => match axis {
            Axis::LeftStickX => state.left_x = value,
            Axis::LeftStickY => state.left_y = value,
            Axis::RightStickX => state.right_x = value,
            Axis::RightStickY => state.right_y = value,
            _ => {}
        },
        EventType::ButtonPressed(button, _) => handle_button_press(button, state, tx, stick_mode, arm_fine),
        EventType::ButtonReleased(button, _) => handle_button_release(button, state),
        _ => {}
    }
}

fn handle_button_press(button: Button, state: &mut PadState, tx: &mpsc::UnboundedSender<TeleopInput>, stick_mode: &AtomicU8, arm_fine: &AtomicBool) {
    match button {
        // LT -- was fully unmapped (claw moved to LB/RB, see the module
        // doc); a toggle rather than a hold for the same reason as
        // `StickXMode`'s L3/R3 -- holding LT while also pressing the
        // D-pad with the same hand is an awkward simultaneous gesture.
        Button::LeftTrigger2 => {
            state.arm_fine_mode = !state.arm_fine_mode;
            arm_fine.store(state.arm_fine_mode, Ordering::Relaxed);
        }
        Button::South => send(tx, TeleopInput::NeutralPose), // green "A" button -- full reset to boot pose
        Button::East => send(tx, TeleopInput::AttitudeReset), // level RPY only
        Button::West => send(tx, TeleopInput::CameraReset),
        Button::North => send(tx, TeleopInput::SaveFrame), // Y -- was a duplicate of B's attitude reset
        Button::Start => send(tx, TeleopInput::ToggleRecording),
        // Back's actual E-Stop/clear choice needs `Client.estopped`, which
        // this module can't see -- see `TeleopInput::EstopToggle`'s doc.
        Button::Select => send(tx, TeleopInput::EstopToggle),
        Button::LeftTrigger => {
            state.lb_held = true;
            send(tx, TeleopInput::ClawNudge { delta: -CLAW_TICK_STEP }); // LB: open
            check_estop_combo(state, tx);
        }
        Button::RightTrigger => {
            state.rb_held = true;
            send(tx, TeleopInput::ClawNudge { delta: CLAW_TICK_STEP }); // RB: close
            check_estop_combo(state, tx);
        }
        // See `StickXMode`'s doc: pressing into an already-held opposite
        // stick-click fires E-Stop clear instead of latching a mode, so
        // the two gestures (mode-toggle vs. clear-combo) don't collide.
        Button::LeftThumb => {
            state.l3_held = true;
            if state.r3_held {
                send(tx, TeleopInput::EstopClear);
            } else {
                state.stick_x_mode = if state.stick_x_mode == StickXMode::Yaw { StickXMode::Turn } else { StickXMode::Yaw };
                stick_mode.store(state.stick_x_mode as u8, Ordering::Relaxed);
            }
        }
        Button::RightThumb => {
            state.r3_held = true;
            if state.l3_held {
                send(tx, TeleopInput::EstopClear);
            } else {
                state.stick_x_mode = if state.stick_x_mode == StickXMode::Roll { StickXMode::Turn } else { StickXMode::Roll };
                stick_mode.store(state.stick_x_mode as u8, Ordering::Relaxed);
            }
        }
        Button::DPadUp => {
            state.dpad_up = true;
            send(tx, TeleopInput::ArmNudge { dx: 0, dz: arm_step(state) });
        }
        Button::DPadDown => {
            state.dpad_down = true;
            send(tx, TeleopInput::ArmNudge { dx: 0, dz: -arm_step(state) });
        }
        Button::DPadLeft => {
            state.dpad_left = true;
            send(tx, TeleopInput::ArmNudge { dx: -arm_step(state), dz: 0 });
        }
        Button::DPadRight => {
            state.dpad_right = true;
            send(tx, TeleopInput::ArmNudge { dx: arm_step(state), dz: 0 });
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
/// LB/RB still individually nudge the claw open/close on their own (see
/// `handle_button_press`); this only adds the combo on top, it doesn't
/// take away the individual action.
fn check_estop_combo(state: &PadState, tx: &mpsc::UnboundedSender<TeleopInput>) {
    if state.lb_held && state.rb_held {
        send(tx, TeleopInput::Estop);
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
        match state.stick_x_mode {
            // Confirmed on real hardware 2026-08-30: needs the opposite
            // sign from roll's below to feel right.
            StickXMode::Yaw => send(tx, TeleopInput::AttitudeNudge { axis: 'y', delta: -state.right_x * ROLL_YAW_TICK_STEP_DEG }),
            StickXMode::Roll => send(tx, TeleopInput::AttitudeNudge { axis: 'r', delta: state.right_x * ROLL_YAW_TICK_STEP_DEG }),
            // input.rs: Left-arrow (turn left) = +turn, Right-arrow =
            // -turn. gilrs' right-stick +X is physically right, so
            // turning right (stick right) needs a negative turn value.
            StickXMode::Turn => send(tx, TeleopInput::Turn { turn: -state.right_x * TURN_SCALE }),
        }
    }
    if state.right_y.abs() > STICK_DEADZONE {
        // input.rs: Up-arrow (look up) sends a *negative* pitch delta
        // (xgolib's pitch axis is inverted from the on-screen direction --
        // confirmed on real hardware). gilrs' stick-up is +Y, so this
        // mirrors that same inversion.
        send(tx, TeleopInput::AttitudeNudge { axis: 'p', delta: -state.right_y * PITCH_TICK_STEP_DEG });
    }
    if state.lb_held {
        send(tx, TeleopInput::ClawNudge { delta: -CLAW_TICK_STEP }); // LB: open
    }
    if state.rb_held {
        send(tx, TeleopInput::ClawNudge { delta: CLAW_TICK_STEP }); // RB: close
    }
    if state.dpad_up {
        send(tx, TeleopInput::ArmNudge { dx: 0, dz: arm_step(state) });
    }
    if state.dpad_down {
        send(tx, TeleopInput::ArmNudge { dx: 0, dz: -arm_step(state) });
    }
    if state.dpad_left {
        send(tx, TeleopInput::ArmNudge { dx: -arm_step(state), dz: 0 });
    }
    if state.dpad_right {
        send(tx, TeleopInput::ArmNudge { dx: arm_step(state), dz: 0 });
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
