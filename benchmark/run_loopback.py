#!/usr/bin/env python3
"""Loopback driver for BENCHMARK.md Part 4 (unencrypted table).

Latency: MQTT + Zenoh ping-pong (64 B, 2000 round trips), ping floor.
Throughput: for each protocol x payload size, step up the paced send rate
until the receiver gets < 99% of what was sent (or the Python sender can't
keep up); report the highest rate that passed. Same method for all three,
since unpaced flooding makes mosquitto drop QoS 0 to the subscriber and
measures queue overflow, not the protocol.

Usage: run_loopback.py [proto ...]  (mqtt zenoh udp mqtt-tls zenoh-tls webrtc)
Writes everything under benchmark/results/<timestamp>/.
"""
import json
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

BENCH = Path(__file__).resolve().parent
PY = str(BENCH / ".venv/bin/python")
OUT = BENCH / "results" / datetime.now().strftime("%Y%m%d-%H%M%S")
OUT.mkdir(parents=True)

SIZES = [16, 2048, 4096, 8192, 16384, 32768, 65536]
RATES = [1_000, 2_000, 5_000, 10_000, 20_000, 50_000, 100_000, 200_000, 500_000, 1e9]
SEND_S, WARMUP_S, WINDOW_S = 10.0, 1.0, 8.0
PASS_RATIO = 0.99
# Protocols to run, e.g. `run_loopback.py webrtc`; default is the unencrypted table.
PROTOS = sys.argv[1:] or ["mqtt", "zenoh", "udp"]

log_f = open(OUT / "run.log", "w")


def log(msg: str) -> None:
    print(msg, flush=True)
    log_f.write(msg + "\n")
    log_f.flush()


def run(cmd, timeout=60):
    return subprocess.run(cmd, cwd=BENCH, capture_output=True, text=True, timeout=timeout)


CERTS = BENCH.parent / "certs"
TLS = BENCH / "tls"


def _mqtt_tls(cert: str) -> list[str]:
    return ["--port", "8883", "--tls", "--ca", str(CERTS / "dev-ca/ca.crt"),
            "--cert", str(CERTS / f"{cert}/{cert}.crt"), "--key", str(CERTS / f"{cert}/{cert}.key")]


def proto_args(proto: str, mode: str) -> list[str]:
    """Script + connection args for one side of a run. The side that starts
    first (responder / sender, or the webrtc receiver) is the one that listens."""
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


def latency(proto: str) -> str:
    resp = subprocess.Popen(proto_args(proto, "responder"), cwd=BENCH,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    try:
        r = run(proto_args(proto, "pingpong") + ["--count", "2000"], timeout=120)
    finally:
        resp.terminate()
        resp.wait()
    return r.stdout.strip() or r.stderr.strip()


def send_cmd(proto, size, rate):
    if proto == "udp":
        return [PY, "raw_udp_baseline.py", "send", "--host", "127.0.0.1", "--port", "5005",
                "--payload-bytes", str(min(size, 65507)), "--rate-hz", str(rate),
                "--duration-s", str(SEND_S)]
    return proto_args(proto, "send") + ["--payload-bytes", str(size),
                                        "--rate-hz", str(rate), "--duration-s", str(SEND_S)]


def recv_cmd(proto):
    if proto == "udp":
        return [PY, "raw_udp_baseline.py", "recv", "--host", "127.0.0.1", "--port", "5005",
                "--duration-s", str(WINDOW_S)]
    extra = ["--warmup-s", str(WARMUP_S)] if proto == "webrtc" else []  # window starts after connect
    return proto_args(proto, "recv") + extra + ["--duration-s", str(WINDOW_S)]


def trial(proto, size, rate):
    if proto == "webrtc":  # receiver is the signaling listener, so it starts first
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
        payload = size
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
        log(f"  {proto:5} {size:6}B target={tgt:>7} sent={t['sent_hz']:>10} "
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
        "packages": run(["uv", "pip", "list", "-p", PY]).stdout,
        "mosquitto": subprocess.run(["docker", "exec", "bench-mosquitto", "mosquitto", "-h"],
                                    capture_output=True, text=True).stdout.splitlines()[:1],
        "topology": "loopback, single host, mosquitto in docker --network host, zenoh peer mode",
    }
    results = {"meta": meta, "latency": {}, "throughput": {}}
    log(f"output: {OUT}  commit {meta['git']} ({meta['branch']})")

    log("== latency (64B, 2000 round trips)")
    for proto in [p for p in PROTOS if p != "udp"]:
        results["latency"][proto] = latency(proto)
        log(f"  {proto}: {results['latency'][proto]}")
    results["latency"]["ping"] = run(["ping", "-c", "20", "-s", "36", "127.0.0.1"]).stdout.strip().splitlines()[-1]
    log(f"  ping: {results['latency']['ping']}")

    log("== throughput (max paced rate with >=99% delivered)")
    for size in SIZES:
        for proto in PROTOS:
            results["throughput"][f"{proto}/{size}"] = search(proto, size)
            (OUT / "results.json").write_text(json.dumps(results, indent=2))
    log("DONE")


if __name__ == "__main__":
    sys.exit(main())
