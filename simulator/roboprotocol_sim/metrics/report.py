"""Per-run report generation: CSV dumps, matplotlib PNGs, and a markdown
summary (headline stats + the NFR pass/fail table from nfr_checks.py).
"""
from __future__ import annotations

import matplotlib

matplotlib.use("Agg")

from pathlib import Path
from typing import Optional

import matplotlib.pyplot as plt
import pandas as pd

from ..protocol.naive_session import LIVENESS_TIMEOUT_S
from ..protocol.safety_state_machine import TASK_CLASS_THRESHOLDS, TIER_NAMES
from .nfr_checks import run_all_checks


def _mark_failure_events(ax, run) -> None:
    for ev in run.timeline.events:
        ax.axvline(ev["t"], color="grey", linestyle="--", linewidth=0.7, alpha=0.6)


def _plot_rtt_and_tier(run, dfs: dict[str, pd.DataFrame], path: Path) -> None:
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 5.5))
    rtt_df = dfs["rtt"]
    if not rtt_df.empty:
        ax1.plot(rtt_df["t"], rtt_df["rtt_ms"], linewidth=0.8, color="tab:blue")
    th = TASK_CLASS_THRESHOLDS[run.scenario.task_class]
    for label, val in [("T0", th.tier0_max), ("T1", th.tier1_max), ("T2", th.tier2_max), ("T3", th.tier3_max)]:
        ax1.axhline(val, color="tab:red", linestyle=":", linewidth=0.6)
    ax1.set_ylabel("RTT (ms)")
    ax1.set_title(f"{run.scenario.label} | failure={run.failure_name} | {run.network_profile_name}")
    _mark_failure_events(ax1, run)

    tiers = [(t.t, t.new_tier) for t in run.safety_sm.transitions]
    if tiers:
        xs = [t for t, _ in tiers] + [run.duration]
        ys = [y for _, y in tiers] + [tiers[-1][1]]
        ax2.step(xs, ys, where="post", color="tab:orange")
    for ev in run.watchdog.trigger_events:
        ax2.axvline(ev.t, color="black", linewidth=1.2)
        ax2.text(ev.t, 4.3, "E-STOP", rotation=90, fontsize=7, va="bottom")
    ax2.set_yticks(range(5))
    ax2.set_yticklabels([TIER_NAMES[i] for i in range(5)], fontsize=7)
    ax2.set_ylabel("Safety Tier")
    ax2.set_xlabel("simulation time (s)")
    _mark_failure_events(ax2, run)

    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _rolling_bandwidth(dgram_df: pd.DataFrame, channel: str, direction: str, window_s: float = 0.5) -> pd.DataFrame:
    df = dgram_df[(dgram_df["channel"] == channel) & (dgram_df["direction"] == direction)]
    if df.empty:
        return pd.DataFrame(columns=["t", "mbps"])
    df = df.sort_values("send_time")
    bins = (df["send_time"] // window_s) * window_s
    grouped = df.groupby(bins)["size_wire_bytes"].sum() * 8 / window_s / 1e6
    return pd.DataFrame({"t": grouped.index.values, "mbps": grouped.values})


def _plot_bandwidth_utilization(run, dfs: dict[str, pd.DataFrame], path: Path) -> None:
    dgram_df = dfs["datagrams"]
    fig, ax = plt.subplots(figsize=(9, 3.5))
    a = _rolling_bandwidth(dgram_df, "A", "robot_to_operator")
    b = _rolling_bandwidth(dgram_df, "B", "robot_to_operator")
    if not a.empty:
        ax.plot(a["t"], a["mbps"], label="Channel A (video, robot->operator)", color="tab:purple")
    if not b.empty:
        ax.plot(b["t"], b["mbps"], label="Channel B (robot->operator)", color="tab:green")
    cap = [
        run.timeline.effective_bandwidth_mbps(t, "robot_to_operator")
        for t in (a["t"].tolist() if not a.empty else [i * 0.5 for i in range(int(run.duration / 0.5))])
    ]
    cap_t = a["t"].tolist() if not a.empty else [i * 0.5 for i in range(int(run.duration / 0.5))]
    ax.plot(cap_t, cap, label="uplink capacity", color="black", linestyle="--", linewidth=0.8)
    ax.set_ylabel("Mbps")
    ax.set_xlabel("simulation time (s)")
    ax.set_title("Robot -> operator bandwidth utilization vs uplink capacity")
    ax.legend(fontsize=7)
    _mark_failure_events(ax, run)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _plot_video_bitrate(run, dfs: dict[str, pd.DataFrame], path: Path) -> None:
    video_df = dfs["video_bitrate"]
    fig, ax = plt.subplots(figsize=(9, 3.5))
    for stream, g in video_df.groupby("stream"):
        ax.plot(g["t"], g["target_bps"] / 1e6, label=stream, linewidth=1.0)
    ax.axhline(2.0, color="grey", linestyle=":", linewidth=0.6)
    ax.axhline(20.0, color="grey", linestyle=":", linewidth=0.6)
    ax.set_ylabel("Target bitrate (Mbps)")
    ax.set_xlabel("simulation time (s)")
    ax.set_title("Channel A adaptive bitrate (GCC-like controller)")
    ax.legend(fontsize=7)
    _mark_failure_events(ax, run)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _plot_tdpa_energy(run, path: Path) -> None:
    if not run.tdpa.energy_log:
        return
    ts = [t for t, _ in run.tdpa.energy_log]
    es = [e for _, e in run.tdpa.energy_log]
    fig, ax = plt.subplots(figsize=(9, 3))
    ax.plot(ts, es, color="tab:red", linewidth=0.8)
    ax.axhline(0.0, color="black", linewidth=0.6)
    for v in run.tdpa.violations[::10]:  # thin markers to keep the plot legible
        ax.axvline(v.t, color="tab:red", alpha=0.08)
    ax.set_ylabel("TDPA energy integral E(t)")
    ax.set_xlabel("simulation time (s)")
    ax.set_title(f"Haptic passivity proxy ({len(run.tdpa.violations)} violation samples, E(t) > 0 = non-passive)")
    _mark_failure_events(ax, run)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _plot_naive_comparison(run, naive_run, dfs: dict[str, pd.DataFrame], naive_dfs: dict[str, pd.DataFrame], path: Path) -> None:
    fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(9, 6))

    dgram_df = dfs["datagrams"]
    quic_cmd = dgram_df[(dgram_df["channel"] == "B") & (dgram_df["category"] == "command") & dgram_df["delay_ms"].notna()]
    naive_seg = naive_dfs["segments"]
    naive_cmd = naive_seg[(naive_seg["category"] == "command") & naive_seg["delay_ms"].notna()]

    if not quic_cmd.empty:
        ax1.plot(quic_cmd["send_time"], quic_cmd["delay_ms"], linewidth=0.6, color="tab:green", label="RoboProtocol (QUIC datagram, Channel B)")
    if not naive_cmd.empty:
        ax1.plot(naive_cmd["send_time"], naive_cmd["delay_ms"], linewidth=0.6, color="tab:red", label="Naive WebSocket/TCP (one ordered stream)")
    ax1.set_yscale("log")
    ax1.set_ylabel("Command delivery delay (ms, log)")
    ax1.set_title(f"{run.scenario.label} | failure={run.failure_name} -- control latency, RoboProtocol vs naive WS/TCP")
    for ev in run.watchdog.trigger_events:
        ax1.axvline(ev.t, color="tab:green", linewidth=1.4, alpha=0.8)
    for t, _silence_ms in naive_run.collector.liveness_events:
        ax1.axvline(t, color="tab:red", linewidth=1.4, alpha=0.8, linestyle="--")
    if not quic_cmd.empty or not naive_cmd.empty:
        ax1.legend(fontsize=7, loc="upper right")
    _mark_failure_events(ax1, run)

    quic_video = dgram_df[(dgram_df["channel"] == "A") & dgram_df["delay_ms"].notna()]
    naive_video = naive_seg[(naive_seg["category"] == "video") & naive_seg["delay_ms"].notna()]
    if not quic_video.empty:
        ax2.plot(quic_video["send_time"], quic_video["delay_ms"], linewidth=0.3, color="tab:green", alpha=0.5, label="RoboProtocol video datagrams")
    if not naive_video.empty:
        ax2.plot(naive_video["send_time"], naive_video["delay_ms"], linewidth=0.3, color="tab:red", alpha=0.5, label="Naive video segments")
    ax2.set_yscale("log")
    ax2.set_ylabel("Video segment delay (ms, log)")
    ax2.set_xlabel("simulation time (s)")
    if not quic_video.empty or not naive_video.empty:
        ax2.legend(fontsize=7, loc="upper right")
    _mark_failure_events(ax2, run)

    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def _naive_comparison_md(run, naive_run, dfs: dict[str, pd.DataFrame], naive_dfs: dict[str, pd.DataFrame]) -> list[str]:
    lines = ["## Naive WebSocket/TCP baseline comparison", ""]
    lines.append(
        "Same scenario and failure injection, replayed over a single ordered, reliable, "
        "unprioritized stream (one TCP connection carrying commands/telemetry/haptic/video "
        "together, full float32 fields, fixed video bitrate, no FEC) instead of RoboProtocol's "
        "QUIC channels -- i.e., how a from-scratch `ws://` teleop demo would actually behave "
        "under the same network conditions."
    )
    lines.append("")

    dgram_df = dfs["datagrams"]
    quic_cmd = dgram_df[(dgram_df["channel"] == "B") & (dgram_df["category"] == "command") & dgram_df["delay_ms"].notna()]
    naive_cmd = naive_dfs["segments"]
    naive_cmd = naive_cmd[(naive_cmd["category"] == "command") & naive_cmd["delay_ms"].notna()]

    lines.append("| Metric | RoboProtocol (QUIC) | Naive WebSocket/TCP |")
    lines.append("|---|---|---|")
    if not quic_cmd.empty and not naive_cmd.empty:
        lines.append(
            f"| Command delivery delay: p50 / p95 / max | "
            f"{quic_cmd['delay_ms'].median():.1f} / {quic_cmd['delay_ms'].quantile(0.95):.1f} / {quic_cmd['delay_ms'].max():.1f} ms | "
            f"{naive_cmd['delay_ms'].median():.1f} / {naive_cmd['delay_ms'].quantile(0.95):.1f} / {naive_cmd['delay_ms'].max():.1f} ms |"
        )
    watchdog_t = run.watchdog.trigger_events[0].t if run.watchdog.trigger_events else None
    watchdog_ms = run.watchdog.trigger_events[0].blackout_ms if run.watchdog.trigger_events else None
    naive_dead_t = naive_run.session.dead_detected_at
    naive_silence_ms = naive_run.collector.liveness_events[0][1] if naive_run.collector.liveness_events else None
    lines.append(
        f"| Connection-loss detection latency | "
        f"{f'{watchdog_ms:.0f} ms (t={watchdog_t:.2f}s, hardware watchdog, SR-4.1)' if watchdog_t is not None else 'not triggered'} | "
        f"{f'{naive_silence_ms:.0f} ms (t={naive_dead_t:.2f}s, {LIVENESS_TIMEOUT_S:.0f}s-class WS ping/pong timeout)' if naive_dead_t is not None else 'not triggered'} |"
    )
    naive_video = naive_dfs["segments"]
    naive_video = naive_video[(naive_video["category"] == "video") & naive_video["delay_ms"].notna()]
    quic_video = dgram_df[(dgram_df["channel"] == "A") & dgram_df["delay_ms"].notna()]
    if not quic_video.empty and not naive_video.empty:
        lines.append(
            f"| Video segment delay: p95 / max | "
            f"{quic_video['delay_ms'].quantile(0.95):.1f} / {quic_video['delay_ms'].max():.1f} ms | "
            f"{naive_video['delay_ms'].quantile(0.95):.1f} / {naive_video['delay_ms'].max():.1f} ms |"
        )
    lines.append("")
    lines.append("![RoboProtocol vs naive WebSocket/TCP](naive_comparison.png)")
    lines.append("")
    return lines


def _write_summary_md(
    run, dfs: dict[str, pd.DataFrame], checks, path: Path,
    naive_run=None, naive_dfs: Optional[dict[str, pd.DataFrame]] = None,
) -> None:
    dgram_df = dfs["datagrams"]
    delivered = dgram_df[dgram_df["delivered"].notna()]
    loss_rate = 1.0 - delivered["delivered"].mean() if not delivered.empty else float("nan")
    rtt_df = dfs["rtt"]

    lines = []
    lines.append(f"# {run.scenario.label}")
    lines.append("")
    lines.append(f"- Failure preset: `{run.failure_name}`")
    lines.append(f"- Network profile: `{run.network_profile_name}`")
    lines.append(f"- Task class: {run.scenario.task_class.value}")
    lines.append(f"- Duration: {run.duration:.1f}s, control loop: {run.scenario.control_hz:.0f}Hz")
    lines.append(f"- Datagrams sent: {len(dgram_df)}, observed loss rate: {loss_rate:.2%}")
    if not rtt_df.empty:
        lines.append(
            f"- RTT: mean={rtt_df['rtt_ms'].mean():.1f}ms p95={rtt_df['rtt_ms'].quantile(0.95):.1f}ms "
            f"max={rtt_df['rtt_ms'].max():.1f}ms"
        )
    lines.append(f"- Safety tier transitions: {len(run.safety_sm.transitions) - 1}")
    lines.append(f"- Watchdog / hardware E-Stop triggers: {len(run.watchdog.trigger_events)}")
    lines.append(f"- TDPA passivity violation samples: {len(run.tdpa.violations)}")
    lines.append("")

    lines.append("## NFR / SR verification checks")
    lines.append("")
    lines.append("| ID | Description | Status | Detail |")
    lines.append("|---|---|---|---|")
    for c in checks:
        lines.append(f"| {c.id} | {c.description} | **{c.status}** | {c.detail} |")
    lines.append("")

    if run.timeline.events:
        lines.append("## Injected failure timeline")
        lines.append("")
        lines.append("| t (s) | event |")
        lines.append("|---|---|")
        for ev in sorted(run.timeline.events, key=lambda e: e["t"]):
            lines.append(f"| {ev['t']:.2f} | {ev['kind']} {ev['detail']} |")
        lines.append("")

    lines.append("## Plots")
    lines.append("")
    lines.append("![RTT and safety tier](rtt_and_tier.png)")
    lines.append("")
    lines.append("![Bandwidth utilization](bandwidth.png)")
    lines.append("")
    if not dfs["video_bitrate"].empty:
        lines.append("![Video bitrate](video_bitrate.png)")
        lines.append("")
    lines.append("![TDPA passivity](tdpa_energy.png)")
    lines.append("")

    if naive_run is not None and naive_dfs is not None:
        lines.extend(_naive_comparison_md(run, naive_run, dfs, naive_dfs))

    path.write_text("\n".join(lines))


def generate_report(run, out_dir: Path, naive_run=None) -> list:
    out_dir.mkdir(parents=True, exist_ok=True)
    dfs = run.collector.to_dataframes()
    for name, df in dfs.items():
        df.to_csv(out_dir / f"{name}.csv", index=False)

    checks = run_all_checks(run, dfs)

    _plot_rtt_and_tier(run, dfs, out_dir / "rtt_and_tier.png")
    _plot_bandwidth_utilization(run, dfs, out_dir / "bandwidth.png")
    if not dfs["video_bitrate"].empty:
        _plot_video_bitrate(run, dfs, out_dir / "video_bitrate.png")
    _plot_tdpa_energy(run, out_dir / "tdpa_energy.png")

    naive_dfs = None
    if naive_run is not None:
        naive_dfs = naive_run.collector.to_dataframes()
        for name, df in naive_dfs.items():
            df.to_csv(out_dir / f"naive_{name}.csv", index=False)
        _plot_naive_comparison(run, naive_run, dfs, naive_dfs, out_dir / "naive_comparison.png")

    _write_summary_md(run, dfs, checks, out_dir / "summary.md", naive_run=naive_run, naive_dfs=naive_dfs)
    return checks
