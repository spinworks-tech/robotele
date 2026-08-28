//! Decodes a recording segment directory (DESIGN.md §10.2 framing) into a
//! plain CSV timeline, reusing the real wire-format decode
//! (`roboprotocol_core`/`roboprotocol_proto`) rather than re-deriving the
//! FlatBuffers/fixed-point layouts elsewhere (Python, spreadsheets, ...).
//!
//! `--category command` (the default) is `tools/replay/
//! recording_to_mp4.py`'s `--overlay-arm` dependency -- its exact
//! invocation (`replay-decode <dir>`, no other args) and exact
//! `capture_us,arm_x,arm_z,claw` header/column shape are load-bearing for
//! that tool and must not change. The other categories are new, standalone
//! CSV dumps for offline analysis of a recorded session; each is its own
//! function below so `command`'s behavior is untouched by them.

use std::fs;
use std::io::{self, Write};
use std::path::PathBuf;

use roboprotocol_core::profile::{CartesianCommand, VelocityAttitudeCommand};
use roboprotocol_core::recording::decode_records;
use roboprotocol_proto::{ActionTrigger as FbActionTrigger, CameraControl as FbCameraControl, ChannelBCategory, ChannelBFrame};

const VELOCITY_ATTITUDE_STANDARD_LEN: usize = 12;
const CARTESIAN_STANDARD_LEN: usize = 8;
const TELEMETRY_HEADER_LEN: usize = 7;

fn sorted_segments(dir: &PathBuf) -> anyhow::Result<Vec<PathBuf>> {
    let mut segments: Vec<PathBuf> = fs::read_dir(dir)?
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|e| e == "rec"))
        .collect();
    segments.sort();
    Ok(segments)
}

/// Wraps a field for CSV output: quoted (with internal quotes doubled) if
/// it contains a comma, quote, or newline, plain otherwise. Only the
/// free-text `key-press` category needs this -- every other category's
/// fields are numbers, which can never contain a CSV-special character.
fn csv_field(s: &str) -> String {
    if s.contains(',') || s.contains('"') || s.contains('\n') {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

fn main() -> anyhow::Result<()> {
    let mut dir: Option<PathBuf> = None;
    let mut category = "command".to_string();
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--category" => category = args.next().ok_or_else(|| anyhow::anyhow!("--category needs a value"))?,
            other if dir.is_none() => dir = Some(PathBuf::from(other)),
            other => anyhow::bail!("unexpected argument: {other}"),
        }
    }
    let dir = dir.ok_or_else(|| {
        anyhow::anyhow!(
            "usage: replay-decode <segment dir> [--category command|telemetry|haptic|key-press|action-trigger]\n\
             (command is the default, and channel-b-command's arm/claw shape -- recording_to_mp4.py's --overlay-arm depends on it exactly)"
        )
    })?;

    // A plain println! panics on SIGPIPE/EPIPE (e.g. piping into `head`) --
    // write through a handle every category can fail out of quietly
    // through instead.
    let stdout = io::stdout();
    let mut out = stdout.lock();
    let write_row = |out: &mut io::StdoutLock, line: &str| -> bool { writeln!(out, "{line}").is_ok() };

    let segments = sorted_segments(&dir)?;
    match category.as_str() {
        "command" => decode_command(&segments, &mut out, &write_row)?,
        "telemetry" => decode_telemetry(&segments, &mut out, &write_row)?,
        "haptic" => decode_haptic(&segments, &mut out, &write_row)?,
        "key-press" => decode_key_press(&segments, &mut out, &write_row)?,
        "action-trigger" => decode_action_trigger(&segments, &mut out, &write_row)?,
        other => anyhow::bail!("unknown --category {other} (expected command/telemetry/haptic/key-press/action-trigger)"),
    }
    Ok(())
}

/// Unchanged from before `--category` existed -- see this module's doc
/// comment for why. Channel B `Command` frames only; arm x/z/claw, the
/// fields `recording_to_mp4.py --overlay-arm` needs.
fn decode_command(segments: &[PathBuf], out: &mut io::StdoutLock, write_row: &impl Fn(&mut io::StdoutLock, &str) -> bool) -> anyhow::Result<()> {
    if !write_row(out, "capture_us,arm_x,arm_z,claw") {
        return Ok(());
    }
    for seg in segments {
        let buf = fs::read(seg)?;
        for (header, payload) in decode_records(&buf) {
            let frame = flatbuffers::get_root::<ChannelBFrame>(payload);
            if frame.category() != ChannelBCategory::Command {
                continue;
            }
            let Some(fields) = frame.fields().map(|v| v.to_vec()) else { continue };
            let Some(va_bytes) = fields.get(0..VELOCITY_ATTITUDE_STANDARD_LEN) else { continue };
            let Some(arm_bytes) = fields.get(VELOCITY_ATTITUDE_STANDARD_LEN..VELOCITY_ATTITUDE_STANDARD_LEN + CARTESIAN_STANDARD_LEN) else { continue };
            // vx/vy/turn aren't needed for this overlay, but unpacking
            // confirms the field layout is the expected shape before we
            // trust the arm bytes right after it.
            let Some(_velocity_attitude) = VelocityAttitudeCommand::unpack_standard(va_bytes) else { continue };
            let Some(arm) = CartesianCommand::unpack_standard(arm_bytes) else { continue };
            if !write_row(out, &format!("{},{},{},{}", header.capture_us, arm.x as i32, arm.z as i32, arm.gripper)) {
                return Ok(());
            }
        }
    }
    Ok(())
}

/// Channel B `Telemetry` frames: battery, roll/pitch/yaw, and every motor
/// angle. Wire layout mirrors `TelemetryData::unpack` in
/// `robot-edge`/`operator-console`'s own (duplicated) `channel_b.rs` --
/// reimplemented here rather than shared, matching how those two crates
/// already each keep their own copy. Motor angles are semicolon-joined
/// into one field instead of one CSV column each, since the motor count
/// varies by robot profile and a fixed column count would either truncate
/// or ragged-row depending on which profile recorded the session.
fn decode_telemetry(segments: &[PathBuf], out: &mut io::StdoutLock, write_row: &impl Fn(&mut io::StdoutLock, &str) -> bool) -> anyhow::Result<()> {
    if !write_row(out, "capture_us,seq,tick_id,region_id,battery,roll,pitch,yaw,motors") {
        return Ok(());
    }
    for seg in segments {
        let buf = fs::read(seg)?;
        for (header, payload) in decode_records(&buf) {
            let frame = flatbuffers::get_root::<ChannelBFrame>(payload);
            if frame.category() != ChannelBCategory::Telemetry {
                continue;
            }
            let Some(fields) = frame.fields().map(|v| v.to_vec()) else { continue };
            if fields.len() < TELEMETRY_HEADER_LEN || !(fields.len() - TELEMETRY_HEADER_LEN).is_multiple_of(2) {
                continue;
            }
            let i16_at = |i: usize| i16::from_be_bytes([fields[i], fields[i + 1]]);
            let battery = fields[0];
            let roll = i16_at(1) as f32 / 100.0;
            let pitch = i16_at(3) as f32 / 100.0;
            let yaw = i16_at(5) as f32 / 100.0;
            let motors: Vec<String> = fields[TELEMETRY_HEADER_LEN..].chunks_exact(2).map(|c| (i16::from_be_bytes([c[0], c[1]]) as f32 / 100.0).to_string()).collect();
            let row = format!(
                "{},{},{},{},{},{},{},{},{}",
                header.capture_us,
                frame.seq(),
                frame.tick_id(),
                frame.region_id(),
                battery,
                roll,
                pitch,
                yaw,
                motors.join(";")
            );
            if !write_row(out, &row) {
                return Ok(());
            }
        }
    }
    Ok(())
}

/// Channel B `Haptic` frames. Nothing in this codebase constructs one yet
/// (confirmed by grepping for `ChannelBCategory::Haptic`/
/// `FbCategory::Haptic` builders -- there are none), so unlike `Command`/
/// `Telemetry` there's no real traffic to derive a field layout from.
/// Dumps the raw `fields` blob as hex rather than guessing a shape that
/// can't be verified against anything real.
fn decode_haptic(segments: &[PathBuf], out: &mut io::StdoutLock, write_row: &impl Fn(&mut io::StdoutLock, &str) -> bool) -> anyhow::Result<()> {
    if !write_row(out, "capture_us,seq,tick_id,region_id,fields_hex") {
        return Ok(());
    }
    for seg in segments {
        let buf = fs::read(seg)?;
        for (header, payload) in decode_records(&buf) {
            let frame = flatbuffers::get_root::<ChannelBFrame>(payload);
            if frame.category() != ChannelBCategory::Haptic {
                continue;
            }
            let hex: String = frame.fields().map(|v| v.iter().map(|b| format!("{b:02x}")).collect()).unwrap_or_default();
            let row = format!("{},{},{},{},{}", header.capture_us, frame.seq(), frame.tick_id(), frame.region_id(), hex);
            if !write_row(out, &row) {
                return Ok(());
            }
        }
    }
    Ok(())
}

/// `KeyPress` records aren't FlatBuffers at all -- `quic_client.rs` writes
/// the raw `format!("{input:?}", ...)` Debug string straight as the
/// payload (see its `on_input`/key-recording call site). Free text, so it
/// needs real CSV quoting (a `Move { vx: 1.0, .. }`-shaped Debug string
/// contains commas and braces).
fn decode_key_press(segments: &[PathBuf], out: &mut io::StdoutLock, write_row: &impl Fn(&mut io::StdoutLock, &str) -> bool) -> anyhow::Result<()> {
    if !write_row(out, "capture_us,input") {
        return Ok(());
    }
    for seg in segments {
        let buf = fs::read(seg)?;
        for (header, payload) in decode_records(&buf) {
            let text = String::from_utf8_lossy(payload);
            if !write_row(out, &format!("{},{}", header.capture_us, csv_field(&text))) {
                return Ok(());
            }
        }
    }
    Ok(())
}

/// Reads a FlatBuffers root table's vtable size (bytes) directly:
/// `payload[0..4]` is a `u32` LE offset from the buffer start to the root
/// table; at that position, an `i32` LE `soffset` points *back* to the
/// vtable; the vtable's own first `u16` LE is its byte size. Structural,
/// not value-based -- see `decode_action_trigger`'s doc for why that
/// matters here. Returns `None` on a buffer too short to hold these
/// fields rather than panicking (a truncated/corrupt record).
fn flatbuffers_vtable_size(payload: &[u8]) -> Option<u16> {
    let root = u32::from_le_bytes(payload.get(0..4)?.try_into().ok()?) as usize;
    let soffset = i32::from_le_bytes(payload.get(root..root + 4)?.try_into().ok()?);
    let vtable_pos = root.checked_add_signed(-(soffset as isize))?;
    Some(u16::from_le_bytes(payload.get(vtable_pos..vtable_pos + 2)?.try_into().ok()?))
}

/// `ActionTriggerC` is a deliberately shared recording category for two
/// distinct Channel C message types -- `ActionTrigger` and `CameraControl`
/// (see `quic_server.rs`'s `CAMERA_CONTROL_STREAM_ID` arm's comment for
/// why: both are discrete, reliably-delivered commands, and a dedicated
/// recording category for one more small message type wasn't worth it).
/// Recorded bytes alone don't carry which type they are -- that came from
/// which QUIC stream they arrived on, and that context doesn't survive
/// into the `.rec` file.
///
/// Disambiguated by vtable size, not by sanity-checking decoded values --
/// a first attempt tried the latter (CameraControl's fields have real
/// physical ranges to check against), but it doesn't work: reading a
/// smaller `ActionTrigger` table as the larger `CameraControl` shape
/// doesn't produce garbage for the fields `ActionTrigger` doesn't have --
/// FlatBuffers returns each field's schema-declared default (`0.0` for
/// every float here), which trivially passes any "is this in range"
/// check. Vtable size is structural instead of semantic: `ActionTrigger`
/// (2 fields) always encodes an 8-byte vtable, `CameraControl` (5 fields,
/// all written unconditionally by its generated `create()`) always
/// encodes 14 -- confirmed against real encoded output from both types,
/// not assumed. `> 10` is the split, comfortably between the two.
fn decode_action_trigger(segments: &[PathBuf], out: &mut io::StdoutLock, write_row: &impl Fn(&mut io::StdoutLock, &str) -> bool) -> anyhow::Result<()> {
    if !write_row(out, "capture_us,kind,action_id,trigger_seq,brightness,contrast,ev,shutter_us,control_seq") {
        return Ok(());
    }
    for seg in segments {
        let buf = fs::read(seg)?;
        for (header, payload) in decode_records(&buf) {
            let row = if flatbuffers_vtable_size(payload).is_some_and(|s| s > 10) {
                let cc = flatbuffers::get_root::<FbCameraControl>(payload);
                format!("{},camera_control,,,{},{},{},{},{}", header.capture_us, cc.brightness(), cc.contrast(), cc.ev(), cc.shutter_us(), cc.control_seq())
            } else {
                let at = flatbuffers::get_root::<FbActionTrigger>(payload);
                format!("{},action_trigger,{},{},,,,,", header.capture_us, at.action_id(), at.trigger_seq())
            };
            if !write_row(out, &row) {
                return Ok(());
            }
        }
    }
    Ok(())
}
