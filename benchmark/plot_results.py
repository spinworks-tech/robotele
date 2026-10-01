#!/usr/bin/env python3
"""Charts for docs/11-protocol-comparison.md from run_loopback.py results.

Writes a light and a dark PNG per chart (the doc picks one per theme with
<picture>). Colors follow the entity across every chart: MQTT blue, Zenoh
orange, WebRTC aqua; raw UDP and iperf3 are references, drawn neutral.

Usage:
    plot_results.py --plain results/<run> --tls results/<run> --webrtc results/<run> \\
        [--out ../docs/img/protocol-comparison]
"""
import argparse
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.ticker import FuncFormatter  # noqa: E402

THEMES = {
    "light": {
        "surface": "#fcfcfb", "ink": "#0b0b0b", "ink2": "#52514e", "muted": "#898781",
        "grid": "#e1e0d9", "axis": "#c3c2b7",
        "mqtt": "#2a78d6", "zenoh": "#eb6834", "webrtc": "#1baf7a", "channelb": "#eda100",
    },
    "dark": {
        "surface": "#1a1a19", "ink": "#ffffff", "ink2": "#c3c2b7", "muted": "#898781",
        "grid": "#2c2c2a", "axis": "#383835",
        "mqtt": "#3987e5", "zenoh": "#d95926", "webrtc": "#199e70", "channelb": "#c98500",
    },
}
NAMES = {"mqtt": "MQTT", "zenoh": "Zenoh", "webrtc": "WebRTC (DimOS)", "udp": "Raw UDP",
         "channelb": "Channel B"}
MARKERS = {"mqtt": "o", "zenoh": "s", "webrtc": "D", "channelb": "^"}  # secondary encoding beside color
SIZES = [16, 2048, 4096, 8192, 16384, 32768, 65536]
NATIVE_SIZES = [16, 1024, 2048, 4096, 8192, 16384, 32768, 65536]
IPERF3_GBPS = 78.5


def size_label(b: int) -> str:
    return f"{b // 1024} KB" if b >= 1024 else f"{b} B"


def load(run_dir: str) -> dict:
    return json.loads((Path(run_dir) / "results.json").read_text())


def parse_latency(s: str) -> dict:
    get = lambda k: float(re.search(rf"{k}=([\d.]+)us", s).group(1))  # noqa: E731
    return {"median": get("median RTT"), "p95": get("p95"), "p99": get("p99")}


def style(ax, t, grid_axis="y"):
    ax.set_facecolor(t["surface"])
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(t["axis"])
    ax.tick_params(colors=t["muted"], labelcolor=t["ink2"], labelsize=9)
    ax.grid(axis=grid_axis, color=t["grid"], linewidth=0.8)
    ax.set_axisbelow(True)


def new_fig(t, w, h, ncols=1, sharey=False):
    fig, axes = plt.subplots(1, ncols, figsize=(w, h), sharey=sharey)
    fig.patch.set_facecolor(t["surface"])
    return fig, axes


def title(fig, t, text, sub):
    fig.text(0.012, 0.975, text, ha="left", va="top", fontsize=13, weight="bold", color=t["ink"])
    fig.text(0.012, 0.915, sub, ha="left", va="top", fontsize=9.5, color=t["ink2"])


def save(fig, out: Path, name: str, mode: str):
    fig.savefig(out / f"{name}-{mode}.png", dpi=200, facecolor=fig.get_facecolor())
    plt.close(fig)


def latency_chart(plain, tls, webrtc, t, out, mode):
    # (protocol, encrypted?, stats) -- grouped per protocol; hollow = plaintext, solid = mTLS/DTLS
    rows = [
        ("zenoh", False, parse_latency(plain["latency"]["zenoh"])),
        ("zenoh", True, parse_latency(tls["latency"]["zenoh-tls"])),
        ("mqtt", False, parse_latency(plain["latency"]["mqtt"])),
        ("mqtt", True, parse_latency(tls["latency"]["mqtt-tls"])),
        ("webrtc", True, parse_latency(webrtc["latency"]["webrtc"])),
    ]
    fig, ax = new_fig(t, 8.0, 4.2)
    style(ax, t, grid_axis="x")
    ys, labels = [], []
    y = 0.0
    for i, (proto, enc, s) in enumerate(rows):
        if i and rows[i - 1][0] != proto:
            y += 0.45  # gap between protocol groups
        c = t[proto]
        ax.barh(y, s["median"], height=0.62, color=c if enc else t["surface"], edgecolor=c,
                linewidth=2 if not enc else 0)
        # p99 whisker: thin line from the median out to p99, capped
        ax.plot([s["median"], s["p99"]], [y, y], color=t["ink2"], linewidth=1)
        ax.plot([s["p99"]], [y], marker="|", markersize=9, color=t["ink2"], markeredgewidth=1.2)
        ax.text(s["p99"] + 6, y, f"{s['median']:.0f} µs  (p99 {s['p99']:.0f})", va="center",
                fontsize=8.5, color=t["ink"])
        ys.append(y)
        labels.append(f"{NAMES[proto]}  ·  {'mTLS' if enc and proto != 'webrtc' else 'DTLS' if enc else 'plaintext'}")
        y += 1
    ax.set_yticks(ys, labels)
    ax.invert_yaxis()
    ax.set_xlim(0, max(r[2]["p99"] for r in rows) * 1.32)
    ax.set_xlabel("round-trip time (µs) — bar: median, whisker: p99", color=t["ink2"], fontsize=9)
    ax.tick_params(axis="y", length=0, labelsize=9.5, labelcolor=t["ink"])
    title(fig, t, "Round-trip latency, 64 B ping-pong (loopback)",
          "Hollow bars: plaintext · solid bars: encrypted · 2,000 round trips, 1st/99th pct trimmed")
    fig.subplots_adjust(left=0.25, right=0.97, top=0.82, bottom=0.14)
    save(fig, out, "latency", mode)


def best_gbps(run: dict, key: str):
    b = run["throughput"].get(key, {}).get("best")
    return b["mbps"] / 1000 if b else None


def throughput_chart(plain, tls, webrtc, t, out, mode):
    fig, axes = new_fig(t, 10.0, 4.6, ncols=2, sharey=True)
    x = list(range(len(SIZES)))
    panels = [
        (axes[0], "Unencrypted", [("mqtt", plain, "mqtt"), ("zenoh", plain, "zenoh")]),
        (axes[1], "Encrypted (mTLS / DTLS)",
         [("mqtt", tls, "mqtt-tls"), ("zenoh", tls, "zenoh-tls"), ("webrtc", webrtc, "webrtc")]),
    ]
    udp = [best_gbps(plain, f"udp/{s}") for s in SIZES]
    for ax, name, series in panels:
        style(ax, t)
        ax.set_yscale("log")
        ax.axhline(IPERF3_GBPS, color=t["muted"], linewidth=1, linestyle=(0, (2, 3)))
        ax.text(0, IPERF3_GBPS * 1.2, "iperf3 TCP ceiling", ha="left", fontsize=8, color=t["muted"])
        ax.plot(x, udp, color=t["muted"], linewidth=1.5, linestyle=(0, (5, 3)))
        ax.annotate("raw UDP", (x[-1], udp[-1]), xytext=(6, 0), textcoords="offset points",
                    fontsize=8.5, color=t["muted"], va="center")
        for proto, run, prefix in series:
            ys = [best_gbps(run, f"{prefix}/{s}") for s in SIZES]
            pts = [(xi, yi) for xi, yi in zip(x, ys) if yi]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=t[proto], linewidth=2,
                    marker=MARKERS[proto], markersize=5.5, markeredgecolor=t["surface"],
                    markeredgewidth=1.2, label=NAMES[proto])
            lx, ly = pts[-1]
            ax.annotate(NAMES[proto], (lx, ly), xytext=(6, -3), textcoords="offset points",
                        fontsize=8.5, color=t["ink"], va="center")
        ax.set_xticks(x, [size_label(s) for s in SIZES])
        ax.set_xlim(-0.3, len(SIZES) - 0.2 + 0.9)
        ax.set_title(name, loc="left", fontsize=10.5, color=t["ink"], pad=8)
        ax.set_xlabel("payload size", color=t["ink2"], fontsize=9)
    axes[0].set_ylabel("sustained throughput, ≥99% delivered", color=t["ink2"], fontsize=9)
    axes[0].yaxis.set_major_formatter(FuncFormatter(
        lambda v, _: f"{v:g} Gbps" if v >= 1 else f"{v * 1000:g} Mbps"))
    axes[0].set_ylim(5e-4, 200)
    handles, labels = axes[1].get_legend_handles_labels()
    leg = fig.legend(handles, labels, loc="upper right", bbox_to_anchor=(0.985, 0.985), ncol=3,
                     frameon=False, fontsize=9)
    for txt in leg.get_texts():
        txt.set_color(t["ink"])
    title(fig, t, "Sustained throughput vs payload size (loopback)",
          "Highest paced rate with ≥99% delivered · WebRTC could not sustain 1k msg/s at 32–64 KB")
    fig.subplots_adjust(left=0.09, right=0.97, top=0.8, bottom=0.12, wspace=0.08)
    save(fig, out, "throughput", mode)


def overload_chart(plain, t, out, mode, size=8192):
    fig, ax = new_fig(t, 8.0, 4.0)
    style(ax, t)
    # Zenoh first with larger markers, MQTT on top: both sit at 100% up to 50k msg/s
    for proto, ms, z in (("zenoh", 7.5, 2), ("mqtt", 5.5, 3)):
        by_rate = {}
        for tr in plain["throughput"][f"{proto}/{size}"]["trials"]:
            by_rate[tr["target_hz"]] = tr  # a retried trial overwrites its failed first try
        pts = sorted((r if r < 1e9 else tr["sent_hz"], min(tr["ratio"], 1.0) * 100)
                     for r, tr in by_rate.items())
        ax.plot([p[0] for p in pts], [p[1] for p in pts], color=t[proto], linewidth=2,
                marker=MARKERS[proto], markersize=ms, markeredgecolor=t["surface"],
                markeredgewidth=1.2, label=NAMES[proto], zorder=z)
        lx, ly = pts[-1]
        ax.annotate(f"{NAMES[proto]}: {ly:.1f}% at {lx / 1000:.0f}k msg/s", (lx, ly),
                    xytext=(-8, 10 if ly < 50 else -12), textcoords="offset points",
                    ha="right", fontsize=8.5, color=t["ink"])
    ax.axhline(99, color=t["muted"], linewidth=1, linestyle=(0, (2, 3)))
    ax.text(1000, 92, "99% pass line · both 100% up to 50k msg/s", fontsize=8, color=t["muted"])
    ax.set_xscale("log")
    ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v / 1000:g}k"))
    ax.set_ylim(-4, 108)
    ax.set_xlabel("offered rate (msg/s)", color=t["ink2"], fontsize=9)
    ax.set_ylabel("delivered (%)", color=t["ink2"], fontsize=9)
    leg = ax.legend(loc="lower left", frameon=False, fontsize=9)
    for txt in leg.get_texts():
        txt.set_color(t["ink"])
    title(fig, t, f"Delivery under overload, {size_label(size)} messages (unencrypted)",
          "Past its limit MQTT (QoS 0) loses almost everything; Zenoh degrades gradually")
    fig.subplots_adjust(left=0.1, right=0.97, top=0.8, bottom=0.14)
    save(fig, out, "overload", mode)


def native_latency_chart(native, t, out, mode):
    """Bars: median RTT; whisker: p99. Hollow = plaintext, solid = encrypted;
    raw UDP is the neutral reference floor."""
    rows = [
        ("udp", None, "Raw UDP (floor)", "rs-udp"),
        ("channelb", True, "Channel B · transport only", "channel-b-raw"),
        ("channelb", True, "Channel B · full frame", "channel-b"),
        ("zenoh", False, "Zenoh · plaintext", "rs-zenoh"),
        ("zenoh", True, "Zenoh · mTLS", "rs-zenoh-tls"),
        ("mqtt", False, "MQTT · plaintext", "rs-mqtt"),
        ("mqtt", True, "MQTT · mTLS", "rs-mqtt-tls"),
        ("webrtc", True, "WebRTC (webrtc-rs) · DTLS", "rs-webrtc"),
    ]
    stats = [parse_latency(native["latency"][key]) for *_, key in rows]
    fig, ax = new_fig(t, 8.4, 5.0)
    style(ax, t, grid_axis="x")
    ys, y, prev = [], 0.0, None
    for (proto, enc, label, _), s in zip(rows, stats):
        if prev is not None and proto != prev:
            y += 0.45
        prev = proto
        c = t["muted"] if proto == "udp" else t[proto]
        solid = bool(enc)
        ax.barh(y, s["median"], height=0.62, color=c if solid else t["surface"], edgecolor=c,
                linewidth=0 if solid else 2)
        ax.plot([s["median"], s["p99"]], [y, y], color=t["ink2"], linewidth=1)
        ax.plot([s["p99"]], [y], marker="|", markersize=9, color=t["ink2"], markeredgewidth=1.2)
        ax.text(s["p99"] + 3, y, f"{s['median']:.0f} µs  (p99 {s['p99']:.0f})", va="center",
                fontsize=8.5, color=t["ink"])
        ys.append(y)
        y += 1
    ax.set_yticks(ys, [r[2] for r in rows])
    ax.invert_yaxis()
    ax.set_xlim(0, max(s["p99"] for s in stats) * 1.35)
    ax.set_xlabel("round-trip time (µs) — bar: median, whisker: p99", color=t["ink2"], fontsize=9)
    ax.tick_params(axis="y", length=0, labelsize=9.5, labelcolor=t["ink"])
    title(fig, t, "Round-trip latency, native clients, 64 B (loopback)",
          "Hollow: plaintext · solid: encrypted · middle of 3 runs × 2,000 round trips")
    fig.subplots_adjust(left=0.28, right=0.97, top=0.85, bottom=0.11)
    save(fig, out, "native-latency", mode)


def native_throughput_chart(native, t, out, mode):
    fig, axes = new_fig(t, 10.0, 4.8, ncols=2, sharey=True)
    x = list(range(len(NATIVE_SIZES)))
    gbps = lambda key: best_gbps(native, key)  # noqa: E731
    udp = [gbps(f"rs-udp/{s}") for s in NATIVE_SIZES]
    panels = [
        (axes[0], "Unencrypted", [("mqtt", "rs-mqtt", "MQTT"), ("zenoh", "rs-zenoh", "Zenoh")]),
        (axes[1], "Encrypted (mTLS / DTLS)",
         [("mqtt", "rs-mqtt-tls", "MQTT"), ("zenoh", "rs-zenoh-tls", "Zenoh"),
          ("webrtc", "rs-webrtc", "WebRTC"), ("channelb", "channel-b-raw", "Channel B")]),
    ]
    for ax, name, series in panels:
        style(ax, t)
        ax.set_yscale("log")
        ax.axhline(IPERF3_GBPS, color=t["muted"], linewidth=1, linestyle=(0, (2, 3)))
        ax.text(0, IPERF3_GBPS * 1.2, "iperf3 TCP ceiling", ha="left", fontsize=8, color=t["muted"])
        ax.plot(x, udp, color=t["muted"], linewidth=1.5, linestyle=(0, (5, 3)))
        ax.annotate("raw UDP", (x[-1], udp[-1]), xytext=(6, 0), textcoords="offset points",
                    fontsize=8.5, color=t["muted"], va="center")
        for proto, prefix, label in series:
            pts = [(xi, v) for xi, s in zip(x, NATIVE_SIZES) if (v := gbps(f"{prefix}/{s}"))]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], color=t[proto], linewidth=2,
                    marker=MARKERS[proto], markersize=6, markeredgecolor=t["surface"],
                    markeredgewidth=1.2, label=NAMES[proto] if proto != "webrtc" else "WebRTC (webrtc-rs)")
            lx, ly = pts[-1]
            if proto == "channelb":
                # Ends at 1 KB, among the steep lines: label it in the open
                # space under WebRTC, with a leader to its last point.
                ax.annotate("Channel B (one datagram max, ≤1.4 KB)", (lx, ly), xytext=(1.5, 0.09),
                            textcoords="data", fontsize=8.5, color=t["ink"], va="center",
                            arrowprops={"arrowstyle": "-", "color": t["ink2"], "linewidth": 0.8})
            else:
                ax.annotate(label, (lx, ly), xytext=(6, -3), textcoords="offset points",
                            fontsize=8.5, color=t["ink"], va="center")
        ax.set_xticks(x, [size_label(s) for s in NATIVE_SIZES], fontsize=8.5)
        ax.set_xlim(-0.3, len(x) - 0.2 + 0.9)
        ax.set_title(name, loc="left", fontsize=10.5, color=t["ink"], pad=8)
        ax.set_xlabel("payload size", color=t["ink2"], fontsize=9)
    axes[0].set_ylabel("sustained throughput, ≥99% delivered", color=t["ink2"], fontsize=9)
    axes[0].yaxis.set_major_formatter(FuncFormatter(
        lambda v, _: f"{v:g} Gbps" if v >= 1 else f"{v * 1000:g} Mbps"))
    axes[0].set_ylim(0.02, 200)
    handles, labels = axes[1].get_legend_handles_labels()
    leg = fig.legend(handles, labels, loc="lower center", bbox_to_anchor=(0.53, 0.0), ncol=4,
                     frameon=False, fontsize=9)
    for txt in leg.get_texts():
        txt.set_color(t["ink"])
    title(fig, t, "Sustained throughput vs payload size, native clients (loopback)",
          "Highest paced rate with ≥99% delivered · Channel B carries one QUIC datagram per message (≤1.4 KB)")
    fig.subplots_adjust(left=0.09, right=0.97, top=0.8, bottom=0.2, wspace=0.08)
    save(fig, out, "native-throughput", mode)


def native_msgrate_chart(native, t, out, mode):
    """Teleop-sized messages, encrypted transports: sustained msg/s at 16 B and 1 KB."""
    bars = [("channelb", "channel-b-raw", "Channel B\ntransport only", True),
            ("channelb", "channel-b", "Channel B\nfull frame", False),
            ("zenoh", "rs-zenoh-tls", "Zenoh\nmTLS", True),
            ("mqtt", "rs-mqtt-tls", "MQTT\nmTLS", True),
            ("webrtc", "rs-webrtc", "WebRTC\nDTLS", True)]
    sizes = [16, 1024]
    fig, axes = new_fig(t, 9.0, 4.4, ncols=2, sharey=True)
    for ax, size in zip(axes, sizes):
        style(ax, t)
        for i, (proto, key, label, solid) in enumerate(bars):
            b = native["throughput"][f"{key}/{size}"]["best"]
            v = b["recv_hz"] / 1000
            c = t[proto]
            ax.bar(i, v, width=0.66, color=c if solid else t["surface"], edgecolor=c,
                   linewidth=0 if solid else 2)
            tag = " †" if b.get("sender_limited") else ""
            ax.text(i, v + 8, f"{v:.0f}k{tag}", ha="center", va="bottom", fontsize=8.5, color=t["ink"])
        ax.set_xticks(range(len(bars)), [b[2] for b in bars], fontsize=8.5, color=t["ink"])
        ax.tick_params(axis="x", length=0)
        ax.set_title(f"{size_label(size)} messages", loc="left", fontsize=10.5, color=t["ink"], pad=8)
    axes[0].set_ylabel("sustained msg/s (thousands), ≥99% delivered", color=t["ink2"], fontsize=9)
    axes[0].set_ylim(0, 580)
    title(fig, t, "Message rate at teleop-sized payloads, encrypted transports",
          "Lower bounds: each bar is the highest rate step passed · † sender-limited · hollow: adds FlatBuffers")
    fig.subplots_adjust(left=0.09, right=0.98, top=0.8, bottom=0.17, wspace=0.08)
    save(fig, out, "native-msgrate", mode)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plain", required=True, help="run dir with mqtt/zenoh/udp")
    ap.add_argument("--tls", required=True, help="run dir with mqtt-tls/zenoh-tls")
    ap.add_argument("--webrtc", required=True, help="run dir with webrtc")
    ap.add_argument("--native", help="run dir with rs-* and channel-b[-raw] (native charts)")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent
                                          / "docs/img/protocol-comparison"))
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    plain, tls, webrtc = load(args.plain), load(args.tls), load(args.webrtc)
    native = load(args.native) if args.native else None
    plt.rcParams["font.family"] = ["Inter", "Ubuntu", "DejaVu Sans"]
    for mode, t in THEMES.items():
        latency_chart(plain, tls, webrtc, t, out, mode)
        throughput_chart(plain, tls, webrtc, t, out, mode)
        overload_chart(plain, t, out, mode)
        if native:
            native_latency_chart(native, t, out, mode)
            native_throughput_chart(native, t, out, mode)
            native_msgrate_chart(native, t, out, mode)
    print(f"wrote {sorted(p.name for p in out.glob('*.png'))} to {out}")


if __name__ == "__main__":
    main()
