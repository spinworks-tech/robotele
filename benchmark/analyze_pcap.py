#!/usr/bin/env python3
"""Summarize a Wireshark/tshark capture of RoboProtocol traffic.

Requires `tshark` on PATH (part of the Wireshark install). Reports packet
count, wire bytes, throughput, and inter-arrival jitter for a given UDP
port -- use it to compare an encrypted QUIC capture against a plaintext
raw_udp_baseline.py capture (BENCHMARK.md part a/b), or to eyeball gaps
around a wifi-drop test (part c).

Usage:
    analyze_pcap.py capture.pcapng --port 4433
    analyze_pcap.py capture.pcapng --port 4433 --csv out.csv
"""
from __future__ import annotations

import argparse
import csv
import statistics
import subprocess
import sys


def run_tshark(pcap: str, port: int) -> list[tuple[float, int]]:
    fields = ["frame.time_epoch", "frame.len"]
    cmd = ["tshark", "-r", pcap, "-Y", f"udp.port == {port}",
           "-T", "fields", "-E", "separator=,", "-E", "quote=n"]
    for f in fields:
        cmd += ["-e", f]
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, check=True).stdout
    except FileNotFoundError:
        sys.exit("tshark not found on PATH -- install Wireshark/tshark first")
    except subprocess.CalledProcessError as e:
        sys.exit(f"tshark failed: {e.stderr}")
    rows = []
    for line in out.splitlines():
        parts = line.split(",")
        if len(parts) < 2 or not parts[0]:
            continue
        rows.append((float(parts[0]), int(parts[1])))
    return rows


def summarize(rows: list[tuple[float, int]], csv_path: str | None) -> None:
    if not rows:
        print("no packets matched that port in this capture", file=sys.stderr)
        return
    times = [r[0] for r in rows]
    sizes = [r[1] for r in rows]
    duration = times[-1] - times[0]
    total_bytes = sum(sizes)
    gaps_ms = [(b - a) * 1000 for a, b in zip(times, times[1:])]

    print(f"packets:         {len(rows)}")
    print(f"duration:        {duration:.3f}s")
    print(f"total wire bytes:{total_bytes} (incl. UDP/IP headers, and QUIC/TLS if present)")
    if duration > 0:
        print(f"throughput:      {total_bytes / duration:.1f} B/s, {len(rows) / duration:.1f} pkt/s")
    print(f"frame length:    mean={statistics.mean(sizes):.1f}B  min={min(sizes)}B  max={max(sizes)}B")
    if len(gaps_ms) > 1:
        print(f"inter-arrival:   mean={statistics.mean(gaps_ms):.2f}ms  "
              f"stdev={statistics.stdev(gaps_ms):.2f}ms  max={max(gaps_ms):.2f}ms")
        big_gaps = [g for g in gaps_ms if g > 5 * statistics.mean(gaps_ms)]
        if big_gaps:
            print(f"outage gaps:     {len(big_gaps)} gap(s) > 5x mean interval "
                  f"(largest={max(big_gaps):.1f}ms) -- likely wifi-drop or loss events")

    if csv_path:
        with open(csv_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["time_epoch", "frame_len"])
            w.writerows(rows)
        print(f"raw rows written to {csv_path}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pcap")
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--csv", help="also write per-packet rows to this CSV path")
    args = ap.parse_args()
    summarize(run_tshark(args.pcap, args.port), args.csv)


if __name__ == "__main__":
    main()
