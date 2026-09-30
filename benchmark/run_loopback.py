#!/usr/bin/env python3
"""Loopback driver for BENCHMARK.md Part 4 (docs/11-protocol-comparison.md).

Latency: ping-pong (64 B, 2000 round trips) per protocol, plus a ping floor.
Throughput: for each protocol x payload size, step up the paced send rate
until the receiver gets < 99% of what was sent (or the sender can't keep
up); report the highest rate that passed. Same method for every protocol,
since unpaced flooding makes mosquitto drop QoS 0 to the subscriber and
measures queue overflow, not the protocol.

Protocols:
  Python clients:  mqtt zenoh udp mqtt-tls zenoh-tls webrtc   (*_bench.py)
  native clients:  rs-udp rs-zenoh rs-mqtt rs-zenoh-tls rs-mqtt-tls rs-webrtc
                   (tools/proto-bench)
  Channel B:       channel-b-raw  transport only: QUIC datagrams + mTLS on
                                  the same opaque bytes as the others
                   channel-b      full frame: adds FlatBuffers encode/decode
                   (robot-edge / operator-console --bench, release builds)
Native protocols and Channel B need `cargo build --release -p proto-bench
-p robot-edge -p operator-console` first.

Usage: run_loopback.py [proto ...]
Writes everything under benchmark/results/<timestamp>/.
"""
import json
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

BENCH = Path(__file__).resolve().parent
PY = str(BENCH / ".venv/bin/python")
OUT = BENCH / "results" / datetime.now().strftime("%Y%m%d-%H%M%S")
OUT.mkdir(parents=True)

SIZES = [16, 1024, 2048, 4096, 8192, 16384, 32768, 65536]
# Channel B never fragments: every message is one QUIC datagram (~1.4 KB max).
CHANNEL_B_SIZES = [16, 1024]
# The real TeleopCommand is 20 bytes; full-frame Channel B pads up from there.
CHANNEL_B_MIN_FIELDS = 20
RATES = [1_000, 2_000, 5_000, 10_000, 20_000, 50_000, 100_000, 200_000, 500_000, 1e9]
SEND_S, WARMUP_S, WINDOW_S = 10.0, 1.0, 8.0
PASS_RATIO = 0.99
LATENCY_RUNS = 3
# Protocols to run, e.g. `run_loopback.py webrtc`; default is the unencrypted table.
PROTOS = sys.argv[1:] or ["mqtt", "zenoh", "udp"]

log_f = open(OUT / "run.log", "w")


def log(msg: str) -> None:
    print(msg, flush=True)
    log_f.write(msg + "\n")
    log_f.flush()


def run(cmd, timeout=60):
    return subprocess.run(cmd, cwd=BENCH, capture_output=True, text=True, timeout=timeout)


REPO = BENCH.parent
CERTS = REPO / "certs"
TLS = BENCH / "tls"
RS = str(REPO / "target/release/proto-bench")
EDGE = str(REPO / "target/release/robot-edge")
OPC = str(REPO / "target/release/operator-console")
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def sizes_for(proto: str) -> list[int]:
    return CHANNEL_B_SIZES if proto.startswith("channel-b") else SIZES


def _mqtt_tls(cert: str) -> list[str]:
    return ["--port", "8883", "--tls", "--ca", str(CERTS / "dev-ca/ca.crt"),
            "--cert", str(CERTS / f"{cert}/{cert}.crt"), "--key", str(CERTS / f"{cert}/{cert}.key")]


def proto_args(proto: str, mode: str) -> list[str]:
    """Script + connection args for one side of a run. The side that starts
    first (responder / sender, or the webrtc receiver) is the one that listens."""
    if proto.startswith("rs-"):
        name = proto[3:]
        args = [RS, name.removesuffix("-tls"), mode]
        if name == "mqtt-tls":
            args += _mqtt_tls("robot" if mode in ("responder", "recv") else "operator")
        elif name == "zenoh-tls":
            role = "listen" if mode in ("responder", "send") else "connect"
            args += ["--config", str(TLS / f"zenoh_tls_{role}.json5")]
        return args
    base = proto.removesuffix("-tls")
    args = [PY, f"{base}_bench.py", mode]
    if base == "mqtt":
        args += ["--host", "127.0.0.1"]
        if proto == "mqtt-tls":
            args += _mqtt_tls("robot" if mode in ("responder", "recv") else "operator")
    elif proto == "zenoh-tls":
        role = "listen" if mode in ("responder", "send") else "connect"
        args += ["--config", str(TLS / f"zenoh_tls_{role}.json5")]
    return args


def start_edge(mode: str, log_path: Path) -> subprocess.Popen:
    """robot-edge --bench <echo|count> on the stub bridge; a fresh process per
    run, so the operator side must not offer a stale 0-RTT ticket."""
    shutil.rmtree(REPO / ".session-cache", ignore_errors=True)
    edge = subprocess.Popen([EDGE, "--listen", "127.0.0.1:4433", "--stub-bridge", "--bench", mode,
                             "--zenoh-port", "17447"], cwd=REPO, stdout=open(log_path, "w"),
                            stderr=subprocess.STDOUT)
    time.sleep(1.5)
    return edge


def opc_cmd(proto: str, *extra: str) -> list[str]:
    raw = ["--bench-raw"] if proto == "channel-b-raw" else []
    return [OPC, "--connect", "127.0.0.1:4433", "--server-name", "robot-edge", "--headless", *raw, *extra]


def stop(p: subprocess.Popen) -> None:
    p.terminate()
    try:
        p.wait(timeout=10)
    except subprocess.TimeoutExpired:
        p.kill()
        p.wait()


def latency(proto: str) -> str:
    if proto.startswith("channel-b"):
        resp = start_edge("echo", OUT / "edge.log")
        pingpong = opc_cmd(proto, "--bench", "pingpong", "--bench-count", "2000", "--bench-payload-bytes", "64")
    else:
        resp = subprocess.Popen(proto_args(proto, "responder"), cwd=BENCH,
                                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(2)
        pingpong = proto_args(proto, "pingpong") + ["--count", "2000"]
    try:
        r = subprocess.run(pingpong, cwd=REPO if proto.startswith("channel-b") else BENCH,
                           capture_output=True, text=True, timeout=120)
    finally:
        stop(resp)
    # operator-console --headless logs to stdout too, so pick out the summary line.
    m = re.search(r"n=\d+ \(trimmed.*", ANSI.sub("", r.stdout))
    return m.group(0) if m else (r.stdout.strip() or r.stderr.strip())[-400:]


def send_cmd(proto, size, rate):
    if proto == "udp":
        return [PY, "raw_udp_baseline.py", "send", "--host", "127.0.0.1", "--port", "5005",
                "--payload-bytes", str(min(size, 65507)), "--rate-hz", str(rate),
                "--duration-s", str(SEND_S)]
    if proto == "rs-udp":
        size = min(size, 65507)  # the UDP datagram maximum, as for the Python floor
    return proto_args(proto, "send") + ["--payload-bytes", str(size),
                                        "--rate-hz", str(rate), "--duration-s", str(SEND_S)]


def recv_cmd(proto):
    if proto == "udp":
        return [PY, "raw_udp_baseline.py", "recv", "--host", "127.0.0.1", "--port", "5005",
                "--duration-s", str(WINDOW_S)]
    extra = ["--warmup-s", str(WARMUP_S)] if proto.endswith("webrtc") else []  # window starts after connect
    return proto_args(proto, "recv") + extra + ["--duration-s", str(WINDOW_S)]


def channel_b_trial(proto, size, rate):
    """robot-edge counts (once-a-second cumulative log lines, timed from its
    first received message); the receive window is the same WARMUP_S..
    WARMUP_S+WINDOW_S slice of the send as for the other protocols."""
    log_path = OUT / "edge.log"
    edge = start_edge("count", log_path)
    try:
        s = subprocess.run(opc_cmd(proto, "--bench", "send", "--bench-payload-bytes", str(size),
                                   "--bench-rate-hz", str(rate), "--bench-duration-s", str(SEND_S)),
                           cwd=REPO, capture_output=True, text=True, timeout=120)
        time.sleep(1.2)  # let the last once-a-second count land
    finally:
        stop(edge)
    points = [tuple(map(int, m)) for m in
              re.findall(r"bench_rx t_ms=(\d+) msgs=(\d+) bytes=(\d+)", ANSI.sub("", log_path.read_text()))]
    lo, hi = WARMUP_S * 1000, (WARMUP_S + WINDOW_S) * 1000
    inside = [p for p in points if lo <= p[0] <= hi]
    if len(inside) >= 2:
        (t0, m0, b0), (t1, m1, b1) = inside[0], inside[-1]
        report = f"{m1 - m0} msgs, {b1 - b0} bytes in {(t1 - t0) / 1000:.2f}s"
    else:
        report = f"0 msgs, 0 bytes in {WINDOW_S:.2f}s"
    return _score(proto, size, rate, s.stderr, subprocess.CompletedProcess([], 0, report, ""))


def trial(proto, size, rate):
    if proto.startswith("channel-b"):
        return channel_b_trial(proto, size, rate)
    if proto.endswith("webrtc"):  # receiver is the signaling listener, so it starts first
        receiver = subprocess.Popen(recv_cmd(proto), cwd=BENCH, stdout=subprocess.PIPE,
                                    stderr=subprocess.PIPE, text=True)
        s = run(send_cmd(proto, size, rate), timeout=120)
        r_out, r_err = receiver.communicate(timeout=60)
        r = subprocess.CompletedProcess(receiver.args, receiver.returncode, r_out, r_err)
        return _score(proto, size, rate, s.stderr, r)
    sender = subprocess.Popen(send_cmd(proto, size, rate), cwd=BENCH,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    time.sleep(WARMUP_S)
    r = run(recv_cmd(proto), timeout=60)
    try:
        _, s_err = sender.communicate(timeout=60)
    except subprocess.TimeoutExpired:
        sender.kill()
        _, s_err = sender.communicate()
    return _score(proto, size, rate, s_err, r)


def _score(proto, size, rate, s_err, r):
    m = re.search(r"sent (\d+) \w+ over ([\d.]+)s", s_err)
    sent_rate = int(m.group(1)) / float(m.group(2)) if m else 0.0
    if proto == "udp":
        m = re.search(r"received (\d+) datagrams in ([\d.]+)s", r.stderr)
        msgs, elapsed = (int(m.group(1)), float(m.group(2))) if m else (0, WINDOW_S)
        payload = min(size, 65507)
    else:
        m = re.search(r"(\d+) msgs, (\d+) bytes in ([\d.]+)s", r.stdout)
        msgs, elapsed = (int(m.group(1)), float(m.group(3))) if m else (0, WINDOW_S)
        payload = {"channel-b": max(size, CHANNEL_B_MIN_FIELDS), "rs-udp": min(size, 65507)}.get(proto, size)
    recv_rate = msgs / elapsed if elapsed else 0.0
    offered = min(rate, sent_rate) if sent_rate else rate
    return {"target_hz": rate, "sent_hz": round(sent_rate, 1), "recv_hz": round(recv_rate, 1),
            "ratio": round(recv_rate / offered, 4) if offered else 0.0,
            "mbps": round(recv_rate * payload * 8 / 1e6, 2), "payload": payload}


def search(proto, size):
    best, trials = None, []
    for rate in RATES:
        t = trial(proto, size, rate)
        if t["ratio"] < PASS_RATIO:  # one retry: a single miss is often connect/subscribe timing
            trials.append(t)
            t = trial(proto, size, rate)
            t["retried"] = True
        trials.append(t)
        tgt = "flood" if rate >= 1e9 else f"{int(rate)}"
        log(f"  {proto:13} {size:6}B target={tgt:>7} sent={t['sent_hz']:>10} "
            f"recv={t['recv_hz']:>10} ratio={t['ratio']:.3f} {t['mbps']} Mbps")
        if t["ratio"] < PASS_RATIO:
            break
        best = t
        if rate < 1e9 and t["sent_hz"] < 0.9 * rate:
            best["sender_limited"] = True  # Python sender maxed out; no point going higher
            break
    return {"best": best, "trials": trials}


def main():
    meta = {
        "git": run(["git", "rev-parse", "--short", "HEAD"]).stdout.strip(),
        "branch": run(["git", "rev-parse", "--abbrev-ref", "HEAD"]).stdout.strip(),
        "python": run([PY, "--version"]).stdout.strip(),
        "rustc": run(["rustc", "--version"]).stdout.strip(),
        "packages": run(["uv", "pip", "list", "-p", PY]).stdout,
        "mosquitto": subprocess.run(["docker", "exec", "bench-mosquitto", "mosquitto", "-h"],
                                    capture_output=True, text=True).stdout.splitlines()[:1],
        "topology": "loopback, single host, mosquitto in docker --network host, zenoh peer mode",
    }
    results = {"meta": meta, "latency": {}, "throughput": {}}
    log(f"output: {OUT}  commit {meta['git']} ({meta['branch']})")

    log(f"== latency (64B, 2000 round trips, {LATENCY_RUNS} runs each; keeping the middle median)")
    results["latency_runs"] = {}
    for proto in [p for p in PROTOS if p != "udp"]:
        runs = [latency(proto) for _ in range(LATENCY_RUNS)]
        results["latency_runs"][proto] = runs
        medians = [float(m.group(1)) if (m := re.search(r"median RTT=([\d.]+)us", x)) else float("inf") for x in runs]
        results["latency"][proto] = runs[sorted(range(len(runs)), key=medians.__getitem__)[len(runs) // 2]]
        log(f"  {proto}: {results['latency'][proto]}")
        log(f"  {'':{len(proto)}}  medians across runs: {', '.join(f'{m:.1f}' for m in medians)} us")
    results["latency"]["ping"] = run(["ping", "-c", "20", "-s", "36", "127.0.0.1"]).stdout.strip().splitlines()[-1]
    log(f"  ping: {results['latency']['ping']}")

    log("== throughput (max paced rate with >=99% delivered)")
    for size in sorted({s for p in PROTOS for s in sizes_for(p)}):
        for proto in [p for p in PROTOS if size in sizes_for(p)]:
            results["throughput"][f"{proto}/{size}"] = search(proto, size)
            (OUT / "results.json").write_text(json.dumps(results, indent=2))
    log("DONE")


if __name__ == "__main__":
    sys.exit(main())
