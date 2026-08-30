//! Keyboard teleop input (WASD move, left/right arrows turn, up/down arrows
//! camera pitch, [/] roll, ,/. yaw twist, 0 reset attitude to level, b/f/v/h
//! (shift = increase) brightness/contrast/exposure/shutter, 9 reset camera
//! controls, space stop, e/c E-Stop/clear, 1/2 canned stand/sit actions,
//! i/j/k/l arm, u/o claw, r start/stop recording, p save video frame,
//! q quit). A stopgap
//! terminal UI, not NFR-2.3's fuller operator console.
//!
//! Each key press sets an absolute `vx`/`vy`/`turn`, not a delta -- there
//! is deliberately no key-release detection here (that needs the Kitty
//! keyboard protocol, which misbehaves on some terminals badly enough to
//! break input parsing entirely -- see git history). Movement staleness
//! (auto-stop if nothing refreshes the command) is handled one layer up,
//! in `quic_client.rs`'s tick loop, using plain terminal auto-repeat --
//! see that module's docs.

use crossterm::event::{Event, EventStream, KeyCode, KeyEvent};
use tokio_stream::StreamExt;

#[derive(Debug, Clone, Copy)]
pub enum TeleopInput {
    Move { vx: f32, vy: f32 },
    Turn { turn: f32 },
    Action(u8),
    Estop,
    EstopClear,
    /// Toggle E-Stop based on whatever `Client.estopped` currently is --
    /// unlike `Estop`/`EstopClear` above, this is never emitted by
    /// `InputReader` (the keyboard has dedicated 'e'/'c' keys, so it
    /// already knows which one it means). It exists for `gamepad.rs`'s
    /// Back button, which has no separate keys to dedicate and no view of
    /// `Client.estopped` from its own polling thread -- `Client::on_input`
    /// is what actually resolves this into a concrete `Estop`/`EstopClear`.
    EstopToggle,
    /// Nudge the held arm position (mm) -- not a velocity, see
    /// `quic_client.rs`'s `Client.last_command` docs.
    ArmNudge { dx: i16, dz: i16 },
    /// Nudge a held whole-body attitude axis (deg) -- xgolib's
    /// `attitude(axis, deg)` keeps all four feet planted and rotates the
    /// body (and the head-mounted camera rigidly fixed to it) via IK
    /// across all 12 leg joints, not just the knees. Position-commanded
    /// like `ArmNudge`, not a velocity. `axis` is 'r'/'p'/'y', matching
    /// xgolib's own convention and `BridgeCommand::Attitude`'s field.
    AttitudeNudge { axis: char, delta: f32 },
    /// Snap all three held attitude axes back to level (0 deg) in one shot
    /// -- an escape hatch so recovering from an accumulated roll/pitch/yaw
    /// nudge doesn't mean counting opposite-direction keypresses back to
    /// zero by hand.
    AttitudeReset,
    /// Full reset to the same neutral pose `Client::run` starts with at
    /// boot -- attitude level (like `AttitudeReset`) *and* arm/claw back
    /// to their own neutral values (`arm_x`/`arm_z` 0, claw
    /// `ARM_CLAW_NEUTRAL`). Never emitted by `InputReader` (no keyboard
    /// key for it); `gamepad.rs`'s "A" button is the only source.
    NeutralPose,
    /// Nudge a held camera image-quality control -- `param` is
    /// 'b'(rightness)/'c'(ontrast)/'v'(EV/exposure)/'s'(hutter), matching
    /// no particular wire field name, just this module's own dispatch key.
    /// Unlike `AttitudeNudge`, applying this means restarting
    /// `libcamera-vid` on the robot (see `roboprotocol_core::camera_control`'s
    /// module doc) -- expect a brief video hiccup per press, not a smooth
    /// live preview while nudging.
    CameraNudge { param: char, delta: f32 },
    /// Reset all four camera controls to their libcamera-vid defaults
    /// (brightness 0, contrast 1, EV 0, shutter auto) in one shot.
    CameraReset,
    /// Nudge the held claw/gripper position (0-255).
    ClawNudge { delta: i8 },
    /// Save the currently-displayed video frame to disk as a PNG -- only
    /// takes effect with `VideoBackend::Native` (see `native_playback.rs`),
    /// since that's the only path where a decoded frame ever exists in
    /// this process; a no-op (with a log line) otherwise.
    SaveFrame,
    /// Starts/stops local recording (FR-9) -- see `quic_client.rs`'s
    /// `on_input` handling for what the default toggled-on category set
    /// is.
    ToggleRecording,
    Quit,
}

/// Arm/claw jog step per keypress -- small enough for controlled nudging
/// via key-repeat, large enough to cross the full range in a reasonable
/// number of presses. Easy to retune here if it feels too coarse/fine.
const ARM_STEP_MM: i16 = 10;
const CLAW_STEP: i8 = 20;
/// Attitude jog step per keypress, in degrees, shared by roll/pitch/yaw.
/// XGO-Lite's vendor SDK (`xgolib`) caps commanded body roll/pitch/yaw at
/// +/-20/10/12 deg respectively (confirmed by reading `ATTITUDE_LIMIT` in
/// `xgolib`'s source on real hardware) -- a small step still reaches the
/// tightest of those (yaw, +/-12) in a handful of presses.
const ATTITUDE_STEP_DEG: f32 = 2.0;
/// Camera control jog steps per keypress. Shutter's is in microseconds
/// (passed through `CameraNudge.delta` as a float and rounded on the
/// receiving end) -- 2000us covers `quic_client.rs`'s 0..100000 manual
/// range in 50 presses, coarse enough that "auto" (0) isn't many presses
/// away from any manually-set value.
const BRIGHTNESS_STEP: f32 = 0.1;
const CONTRAST_STEP: f32 = 0.1;
const EV_STEP: f32 = 0.5;
const SHUTTER_STEP_US: f32 = 2000.0;

pub struct InputReader {
    stream: EventStream,
}

impl InputReader {
    pub fn new() -> Self {
        Self { stream: EventStream::new() }
    }

    pub async fn next(&mut self) -> Option<TeleopInput> {
        loop {
            let event = self.stream.next().await?;
            let Ok(Event::Key(KeyEvent { code, .. })) = event else { continue };
            let input = match code {
                KeyCode::Char('w') => TeleopInput::Move { vx: 15.0, vy: 0.0 },
                KeyCode::Char('s') => TeleopInput::Move { vx: -15.0, vy: 0.0 },
                // XGO's +vy is the robot's left, not right (confirmed on
                // real hardware: the naive vy>0=right assumption had a/d
                // backwards) -- a=left gets +vy, d=right gets -vy.
                KeyCode::Char('a') => TeleopInput::Move { vx: 0.0, vy: 12.0 },
                KeyCode::Char('d') => TeleopInput::Move { vx: 0.0, vy: -12.0 },
                KeyCode::Left => TeleopInput::Turn { turn: 60.0 },
                KeyCode::Right => TeleopInput::Turn { turn: -60.0 },
                // Up = look up (nose/camera tilts up), Down = look down
                // toward a grasp target. xgolib's pitch axis is inverted
                // from that -- confirmed on real hardware 2026-08-23 --
                // so Up sends a *negative* delta and Down a positive one to
                // get the intuitive on-screen direction.
                //
                // '='/'-' are a fallback binding for the same action: some
                // terminals/IDE integrations (e.g. VS Code's integrated
                // terminal command-history/suggestion navigation) capture
                // plain Up/Down for their own UI and never forward the
                // escape sequence to the raw-mode app, while Left/Right
                // (also arrows, used for turn above) pass through fine
                // since they're not used for list navigation there. Same
                // up/down mnemonic as the browser zoom keys.
                KeyCode::Up | KeyCode::Char('=') => TeleopInput::AttitudeNudge { axis: 'p', delta: -ATTITUDE_STEP_DEG },
                KeyCode::Down | KeyCode::Char('-') => TeleopInput::AttitudeNudge { axis: 'p', delta: ATTITUDE_STEP_DEG },
                // Roll (lean sideways) and yaw-twist (pan in place, feet
                // planted -- distinct from Left/Right's walking turn above,
                // and much smaller range, +/-12 deg vs turn's continuous
                // spin). '['/']' and ','/'.' are plain characters like the
                // pitch fallback above, not arrow escape sequences, so
                // nothing should be able to intercept them for its own UI.
                // Sign polarity is an educated guess (unlike pitch, not yet
                // confirmed against real hardware) -- swap if backwards.
                KeyCode::Char('[') => TeleopInput::AttitudeNudge { axis: 'r', delta: -ATTITUDE_STEP_DEG },
                KeyCode::Char(']') => TeleopInput::AttitudeNudge { axis: 'r', delta: ATTITUDE_STEP_DEG },
                KeyCode::Char(',') => TeleopInput::AttitudeNudge { axis: 'y', delta: -ATTITUDE_STEP_DEG },
                KeyCode::Char('.') => TeleopInput::AttitudeNudge { axis: 'y', delta: ATTITUDE_STEP_DEG },
                // '0' resets roll/pitch/yaw to level -- adjacent to 1/2's
                // stand/sit cluster below, and 0 deg is the natural
                // mnemonic for "neutral attitude".
                KeyCode::Char('0') => TeleopInput::AttitudeReset,
                // Camera image controls: lowercase decreases, uppercase
                // (shift) increases -- one letter per parameter instead of
                // two, since the remaining free keys are scarce. Letters
                // chosen where a mnemonic was available (v/V for EV) and
                // arbitrarily from what's left otherwise (b for brightness
                // is the one genuine mnemonic besides EV; contrast/shutter
                // don't have letters spelling them out that aren't already
                // taken elsewhere in this keymap).
                KeyCode::Char('b') => TeleopInput::CameraNudge { param: 'b', delta: -BRIGHTNESS_STEP },
                KeyCode::Char('B') => TeleopInput::CameraNudge { param: 'b', delta: BRIGHTNESS_STEP },
                KeyCode::Char('f') => TeleopInput::CameraNudge { param: 'c', delta: -CONTRAST_STEP },
                KeyCode::Char('F') => TeleopInput::CameraNudge { param: 'c', delta: CONTRAST_STEP },
                KeyCode::Char('v') => TeleopInput::CameraNudge { param: 'v', delta: -EV_STEP },
                KeyCode::Char('V') => TeleopInput::CameraNudge { param: 'v', delta: EV_STEP },
                KeyCode::Char('h') => TeleopInput::CameraNudge { param: 's', delta: -SHUTTER_STEP_US },
                KeyCode::Char('H') => TeleopInput::CameraNudge { param: 's', delta: SHUTTER_STEP_US },
                KeyCode::Char('9') => TeleopInput::CameraReset,
                KeyCode::Char(' ') => TeleopInput::Move { vx: 0.0, vy: 0.0 },
                KeyCode::Char('e') => TeleopInput::Estop,
                KeyCode::Char('c') => TeleopInput::EstopClear,
                KeyCode::Char('1') => TeleopInput::Action(2),  // stand
                KeyCode::Char('2') => TeleopInput::Action(12), // sit
                // Arm/claw jog: IJKL is the standard "second WASD" cluster
                // (same diamond shape, immediately adjacent) since WASD/
                // arrows/space are already taken by base movement; U/O
                // flank I on the row above, reachable from the same hand
                // position.
                //   u(claw open)  i(arm Z+)  o(claw close)
                //   j(arm X-)     k(arm Z-)  l(arm X+)
                // Claw open/close direction is an unverified guess (xgolib's
                // exact claw(pos) polarity wasn't confirmed against real
                // hardware) -- swap the two if it's backwards.
                KeyCode::Char('i') => TeleopInput::ArmNudge { dx: 0, dz: ARM_STEP_MM },
                KeyCode::Char('k') => TeleopInput::ArmNudge { dx: 0, dz: -ARM_STEP_MM },
                KeyCode::Char('j') => TeleopInput::ArmNudge { dx: -ARM_STEP_MM, dz: 0 },
                KeyCode::Char('l') => TeleopInput::ArmNudge { dx: ARM_STEP_MM, dz: 0 },
                KeyCode::Char('u') => TeleopInput::ClawNudge { delta: -CLAW_STEP },
                KeyCode::Char('o') => TeleopInput::ClawNudge { delta: CLAW_STEP },
                KeyCode::Char('r') => TeleopInput::ToggleRecording,
                KeyCode::Char('p') => TeleopInput::SaveFrame,
                KeyCode::Char('q') | KeyCode::Esc => TeleopInput::Quit,
                _ => continue,
            };
            return Some(input);
        }
    }
}

impl Default for InputReader {
    fn default() -> Self {
        Self::new()
    }
}
