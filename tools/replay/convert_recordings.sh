#!/usr/bin/env bash
# Converts every recording category found under <recordings-dir> into a
# human-usable file in <output-dir>: video-a -> an .mp4 (via
# recording_to_mp4.py, always with the crosshair/stats/capture_us
# overlays baked in, plus arm-position when a channel-b-command directory
# is also present -- see that script's own doc for what each overlay
# shows; this means video-a always gets re-encoded here, never the
# lossless `-c:v copy` remux recording_to_mp4.py does with no overlay
# flags at all), and every other present category -> a CSV (via
# replay-decode --category ..., see tools/replay-decode/src/main.rs's
# doc for each category's column shape). A category directory that
# doesn't exist, or exists but holds no .rec segments (e.g. a session
# that never toggled 'r' for haptic/key-press), is silently skipped --
# not every session records every category, and that's normal, not an
# error.
#
# Usage: tools/replay/convert_recordings.sh <recordings-dir> [output-dir]
#   <recordings-dir>  the --record-dir passed to robot-edge/operator-console
#   [output-dir]      default: <recordings-dir>/converted
#
# Example:
#   tools/replay/convert_recordings.sh recordings
#   tools/replay/convert_recordings.sh recordings recordings/converted
set -euo pipefail

RECORDINGS_DIR="${1:?usage: convert_recordings.sh <recordings-dir> [output-dir]}"
OUT_DIR="${2:-$RECORDINGS_DIR/converted}"

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

REPLAY_DECODE_BIN="$REPO_ROOT/target/release/replay-decode"
[[ -x "$REPLAY_DECODE_BIN" ]] || REPLAY_DECODE_BIN="$REPO_ROOT/target/debug/replay-decode"
if [[ ! -x "$REPLAY_DECODE_BIN" ]]; then
    echo "replay-decode not built -- run: cargo build --release -p replay-decode" >&2
    exit 1
fi
if ! command -v ffmpeg >/dev/null; then
    echo "ffmpeg not found on PATH -- install it first" >&2
    exit 1
fi

mkdir -p "$OUT_DIR"

# "<recording-dir-name>:<replay-decode --category value>", per
# roboprotocol-recording's Category::dir_name(). video-a is handled
# separately below (it converts to .mp4, not CSV).
CSV_CATEGORIES=(
    "channel-b-command:command"
    "channel-b-telemetry:telemetry"
    "channel-b-haptic:haptic"
    "key-press:key-press"
    "action-trigger-c:action-trigger"
)

for entry in "${CSV_CATEGORIES[@]}"; do
    dir_name="${entry%%:*}"
    category="${entry##*:}"
    seg_dir="$RECORDINGS_DIR/$dir_name"
    if [[ -d "$seg_dir" ]] && compgen -G "$seg_dir/*.rec" >/dev/null; then
        out_csv="$OUT_DIR/$dir_name.csv"
        echo "== $dir_name -> $(basename "$out_csv") =="
        "$REPLAY_DECODE_BIN" "$seg_dir" --category "$category" >"$out_csv"
        echo "   $(($(wc -l <"$out_csv") - 1)) rows"
    fi
done

VIDEO_DIR="$RECORDINGS_DIR/video-a"
if [[ -d "$VIDEO_DIR" ]] && compgen -G "$VIDEO_DIR/*.rec" >/dev/null; then
    out_mp4="$OUT_DIR/video-a.mp4"
    echo "== video-a -> $(basename "$out_mp4") =="
    overlay_args=(--crosshair --overlay-stats --overlay-capture-us)
    command_dir="$RECORDINGS_DIR/channel-b-command"
    if [[ -d "$command_dir" ]] && compgen -G "$command_dir/*.rec" >/dev/null; then
        overlay_args+=(--overlay-arm "$command_dir" --replay-decode-bin "$REPLAY_DECODE_BIN")
    fi
    python3 "$REPO_ROOT/tools/replay/recording_to_mp4.py" "$VIDEO_DIR" "$out_mp4" "${overlay_args[@]}"
fi

echo
echo "== done -- output in $OUT_DIR =="
ls -la "$OUT_DIR"
