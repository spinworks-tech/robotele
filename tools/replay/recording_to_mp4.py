#!/usr/bin/env python3
"""Convert a recorded Channel A (video) segment directory into an .mp4,
with optional debug overlays.

`roboprotocol-recording` writes each category's segments framed per
DESIGN.md §10.2 (`record_len:u32 BE` + `capture_us:u64 BE` +
`control_source:u8` + payload) under `<record-dir>/video-a/*.rec` --
not raw H.264. For `video-a` specifically, each record's payload is
already a complete, start-code-prefixed H.264 NAL unit (recorded before
Channel A's own wire-chunking -- see `channel_a.rs`'s `spawn_encoder`),
so stripping the framing and concatenating payloads in segment-filename
order (segments are named by their first record's `capture_us`, so
sorting by name is sorting by time) reconstructs a valid Annex-B
elementary stream that ffmpeg can remux directly, with no re-encode.

With no overlay flags, output is a lossless `-c:v copy` remux. Any
overlay flag forces a re-encode (libx264), since burning in graphics
requires actually decoding and redrawing each frame.

Requires `ffmpeg` on PATH. `--overlay-arm` additionally requires the
`replay-decode` tool built (`cargo build --release -p replay-decode`
from the repo root) -- it decodes `channel-b-command` recordings using
the real wire-format logic rather than re-deriving FlatBuffers/
fixed-point parsing here.

Usage:
    recording_to_mp4.py recordings/video-a output.mp4
    recording_to_mp4.py recordings/video-a output.mp4 --crosshair
    recording_to_mp4.py recordings/video-a output.mp4 --overlay-stats
    recording_to_mp4.py recordings/video-a output.mp4 --overlay-capture-us
    recording_to_mp4.py recordings/video-a output.mp4 \\
        --overlay-arm recordings/channel-b-command
    recording_to_mp4.py recordings/video-a output.mp4 \\
        --crosshair --overlay-stats --overlay-capture-us \\
        --overlay-arm recordings/channel-b-command
"""
from __future__ import annotations

import argparse
import shutil
import struct
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Iterator, List, Optional, Tuple

HEADER_LEN = 4 + 8 + 1  # record_len:u32 BE, capture_us:u64 BE, control_source:u8

REPO_ROOT = Path(__file__).resolve().parents[2]
REPLAY_DECODE_CANDIDATES = [
    REPO_ROOT / "target" / "release" / "replay-decode",
    REPO_ROOT / "target" / "debug" / "replay-decode",
]


def iter_records(buf: bytes) -> Iterator[Tuple[int, int, bytes]]:
    """Yields (capture_us, control_source, payload) for each complete
    record in buf, stopping cleanly (not erroring) at a truncated final
    record -- see roboprotocol_core::recording's module doc for why a
    segment can legitimately end mid-record (e.g. a crash mid-write)."""
    pos = 0
    while pos + HEADER_LEN <= len(buf):
        record_len = struct.unpack_from(">I", buf, pos)[0]
        capture_us = struct.unpack_from(">Q", buf, pos + 4)[0]
        control_source = buf[pos + 12]
        start = pos + HEADER_LEN
        end = start + record_len
        if end > len(buf):
            break
        yield capture_us, control_source, buf[start:end]
        pos = end


def escape_drawtext(text: str) -> str:
    """ffmpeg's drawtext filter treats `:`, `'`, and `\\` specially in the
    filter-graph string itself (separate from any escaping the text's own
    content might need)."""
    return text.replace("\\", "\\\\").replace(":", "\\:").replace("'", "\\'")


def escape_filter_value(s: str) -> str:
    """Escapes a bare (unquoted) filtergraph option value -- `:` separates
    options and `\\` is the escape character itself. Narrower than
    `escape_drawtext`: an option value like a file path is never wrapped
    in its own quotes the way `text=` is, so `'` needs no escaping here."""
    return s.replace("\\", "\\\\").replace(":", "\\:")


def build_crosshair_filters(size: int) -> List[str]:
    # A small centered "+" (game-HUD style), not full-width/height lines --
    # `size` is each arm's length in pixels, 2px thick. t=fill so these
    # thin boxes are solid, not just a border outline (drawbox's default
    # `t` is a border thickness, not a fill).
    return [
        f"drawbox=x=(iw-{size})/2:y=(ih-2)/2:w={size}:h=2:color=white:t=fill",
        f"drawbox=x=(iw-2)/2:y=(ih-{size})/2:w=2:h={size}:color=white:t=fill",
    ]


def build_stats_filter(total_records: int, total_bytes: int, duration_s: float) -> str:
    fps = total_records / duration_s if duration_s > 0 else 0.0
    kbps = (total_bytes * 8 / 1000) / duration_s if duration_s > 0 else 0.0
    text = escape_drawtext(f"{fps:.1f} fps | {kbps:.0f} kbps")
    return f"drawtext=text='{text}':fontcolor=white:fontsize=18:x=w-tw-10:y=10:box=1:boxcolor=black@0.5:boxborderw=5"


def build_capture_us_overlay_filters(capture_us_list: List[int], fps: int) -> Tuple[List[str], Optional[Path]]:
    """Burns each frame's own `capture_us` into the bottom-right corner.
    Unlike the arm overlay, this value is different on essentially every
    frame, so the "one drawtext per held interval" trick doesn't apply --
    instead this drives a *single* named `drawtext` filter instance via
    `sendcmd`, which schedules a `reinit` (ffmpeg's generic
    "re-apply this option string" runtime command) at each frame's own
    timestamp. The instance name (`captureus`) is required, not
    decorative: `sendcmd`'s target matches by filter instance name, and
    plain unnamed `drawtext=...` filters (this script's crosshair/stats/
    arm overlays) all default to the same name -- an unnamed target here
    would rewrite whichever of those happens to match first instead of
    this one. The command file's target must be the full
    `filtertype@instancename` (`drawtext@captureus`), not the bare
    instance name -- confirmed empirically after the bare-name form
    silently matched nothing at all (no error, the reinit just never
    applied and the initial text stayed on screen for the whole video).

    Returns the filter fragments to append to the main filter chain, and
    the temp commands file's path (the caller must keep it alive until
    ffmpeg has run, then delete it -- ffmpeg reads it during encoding,
    not up front)."""
    if not capture_us_list:
        return [], None

    style = "x=w-tw-10:y=h-th-10:fontcolor=white:fontsize=18:box=1:boxcolor=black@0.5:boxborderw=5"
    cmds_file = tempfile.NamedTemporaryFile(mode="w", suffix=".sendcmd.txt", delete=False)
    with cmds_file:
        for i, capture_us in enumerate(capture_us_list):
            t = i / fps
            cmds_file.write(f"{t:.6f} drawtext@captureus reinit 'text={capture_us}:{style}';\n")

    filters = [
        f"sendcmd=f={escape_filter_value(cmds_file.name)}",
        f"drawtext@captureus=text='':{style}",
    ]
    return filters, Path(cmds_file.name)


def find_replay_decode(override: Optional[str]) -> Path:
    if override:
        p = Path(override)
        if not p.is_file():
            sys.exit(f"--replay-decode-bin {p} not found")
        return p
    for candidate in REPLAY_DECODE_CANDIDATES:
        if candidate.is_file():
            return candidate
    sys.exit(
        "replay-decode not built -- run `cargo build --release -p replay-decode` "
        "from the repo root, or pass --replay-decode-bin"
    )


def load_arm_timeline(command_dir: str, replay_decode_bin: Path) -> List[Tuple[int, int, int, int]]:
    """Returns [(capture_us, arm_x, arm_z, claw), ...], sorted, via the
    real Rust decoder (see recording_to_mp4.py's module doc for why this
    isn't re-implemented in Python)."""
    result = subprocess.run([str(replay_decode_bin), command_dir], capture_output=True, text=True)
    if result.returncode != 0:
        sys.exit(f"replay-decode failed:\n{result.stderr}")
    rows = []
    for line in result.stdout.splitlines()[1:]:  # skip CSV header
        capture_us, arm_x, arm_z, claw = line.split(",")
        rows.append((int(capture_us), int(arm_x), int(arm_z), int(claw)))
    rows.sort()
    return rows


def build_arm_overlay_filters(arm_timeline: List[Tuple[int, int, int, int]], video_start_us: int, video_duration_s: float) -> List[str]:
    # Collapse consecutive identical (arm_x, arm_z, claw) into one interval
    # -- robot-edge resends the same position every tick (see
    # dispatch_teleop_command's doc), so without this a held position
    # would generate one drawtext filter per tick instead of one per
    # actual change.
    collapsed: List[Tuple[float, int, int, int]] = []
    for capture_us, arm_x, arm_z, claw in arm_timeline:
        t = (capture_us - video_start_us) / 1_000_000
        if collapsed and collapsed[-1][1:] == (arm_x, arm_z, claw):
            continue
        collapsed.append((t, arm_x, arm_z, claw))

    filters = []
    for i, (t_start, arm_x, arm_z, claw) in enumerate(collapsed):
        t_end = collapsed[i + 1][0] if i + 1 < len(collapsed) else video_duration_s
        if t_end <= 0 or t_start >= video_duration_s:
            continue  # entirely before the video starts or after it ends
        t_start = max(t_start, 0.0)
        claw_label = "closed" if claw >= 128 else "open"
        text = escape_drawtext(f"arm x={arm_x} z={arm_z} {claw_label}")
        filters.append(
            f"drawtext=text='{text}':fontcolor=white:fontsize=18:x=10:y=h-th-10:"
            f"box=1:boxcolor=black@0.5:boxborderw=5:enable='between(t,{t_start:.3f},{t_end:.3f})'"
        )
    return filters


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("segment_dir", help="e.g. recordings/video-a")
    ap.add_argument("output_mp4")
    ap.add_argument("--fps", type=int, default=30, help="capture framerate (default: 30, matching CaptureConfig's default)")
    ap.add_argument("--crosshair", action="store_true", help="overlay a small white center crosshair")
    ap.add_argument("--crosshair-size", type=int, default=20, help="crosshair arm length in pixels (default: 20)")
    ap.add_argument("--overlay-stats", action="store_true", help="overlay average fps/bitrate in the top-right corner")
    ap.add_argument("--overlay-capture-us", action="store_true", help="overlay each frame's own capture_us in the bottom-right corner")
    ap.add_argument("--overlay-arm", metavar="COMMAND_SEGMENT_DIR", help="overlay arm x/z/claw state (e.g. recordings/channel-b-command)")
    ap.add_argument("--replay-decode-bin", help="override the replay-decode binary path (default: auto-detect in target/release or target/debug)")
    args = ap.parse_args()

    if shutil.which("ffmpeg") is None:
        sys.exit("ffmpeg not found on PATH -- install it first")

    segment_dir = Path(args.segment_dir)
    segments = sorted(segment_dir.glob("*.rec"))
    if not segments:
        sys.exit(f"no .rec segments found in {segment_dir}")

    want_overlay = args.crosshair or args.overlay_stats or args.overlay_capture_us or args.overlay_arm

    replay_decode_bin = None
    if args.overlay_arm:
        replay_decode_bin = find_replay_decode(args.replay_decode_bin)

    total_records = 0
    total_bytes = 0
    first_capture_us: Optional[int] = None
    last_capture_us: Optional[int] = None
    payloads: List[bytes] = []
    capture_us_list: List[int] = []
    for seg in segments:
        buf = seg.read_bytes()
        n = 0
        for capture_us, _control_source, payload in iter_records(buf):
            if want_overlay:
                payloads.append(payload)
                if args.overlay_capture_us:
                    capture_us_list.append(capture_us)
            total_bytes += len(payload)
            if first_capture_us is None:
                first_capture_us = capture_us
            last_capture_us = capture_us
            n += 1
        print(f"{seg.name}: {n} records, {len(buf)} bytes", file=sys.stderr)
        total_records += n

    if first_capture_us is None or last_capture_us is None:
        sys.exit("no records found in any segment")
    duration_s = total_records / args.fps  # capture_us spans arrival time, not presentation time -- frame count/fps is the honest duration

    filters: List[str] = []
    if args.crosshair:
        filters += build_crosshair_filters(args.crosshair_size)
    if args.overlay_stats:
        filters.append(build_stats_filter(total_records, total_bytes, duration_s))
    sendcmd_file: Optional[Path] = None
    if args.overlay_capture_us:
        capture_us_filters, sendcmd_file = build_capture_us_overlay_filters(capture_us_list, args.fps)
        filters += capture_us_filters
    if args.overlay_arm:
        arm_timeline = load_arm_timeline(args.overlay_arm, replay_decode_bin)
        filters += build_arm_overlay_filters(arm_timeline, first_capture_us, duration_s)

    cmd = ["ffmpeg", "-y", "-r", str(args.fps), "-i", "-"]
    if filters:
        cmd += ["-vf", ",".join(filters), "-c:v", "libx264", "-preset", "veryfast", "-crf", "20"]
    else:
        cmd += ["-c:v", "copy"]
    cmd.append(args.output_mp4)

    try:
        proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stderr=subprocess.PIPE)
        assert proc.stdin is not None
        if want_overlay:
            for payload in payloads:
                proc.stdin.write(payload)
        else:
            for seg in segments:
                for _capture_us, _control_source, payload in iter_records(seg.read_bytes()):
                    proc.stdin.write(payload)
        proc.stdin.close()

        # Not proc.communicate() -- it unconditionally tries to flush stdin
        # itself, which raises since we already closed it above.
        stderr = proc.stderr.read()
        proc.wait()
        if proc.returncode != 0:
            sys.exit(f"ffmpeg failed (exit {proc.returncode}):\n{stderr.decode(errors='replace')}")
    finally:
        # ffmpeg reads this file live during encoding (sendcmd), not
        # up front -- only safe to remove after proc.wait() above.
        if sendcmd_file is not None:
            sendcmd_file.unlink(missing_ok=True)

    print(f"wrote {args.output_mp4} from {total_records} records across {len(segments)} segment(s)", file=sys.stderr)


if __name__ == "__main__":
    main()
