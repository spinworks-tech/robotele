#!/usr/bin/env python3
"""Protocol comparison between the robot's CM4 and an operator laptop over
real Wi-Fi (the two-host counterpart of run_loopback.py; see
docs/11-protocol-comparison.md).

Same method as run_loopback.py: ping-pong latency (64 B, 2000 round trips,
3 runs, middle median kept), and for each protocol x payload size the
highest paced rate at which the receiver gets >= 99% of what was offered.
Two differences:

- **Direction.** Throughput runs robot -> operator: the CM4 sends and the
  laptop receives, the direction sensor data travels (docs/13). Ping-pong
  is laptop-initiated, as before.
- **Sizes up to 1 MB.** Channel B sends a payload above one datagram as
  sensor slices through robot-edge's lossy queue, exactly like a real
  sensor frame, and the operator counts it only if every slice arrives.
  The other protocols deliver whole messages (raw UDP up to 65,507 B,
  webrtc-rs up to 65,535 B).

All protocols are encrypted with mutual TLS (Channel B: QUIC; Zenoh: TLS;
MQTT: TLS via a broker on the laptop; WebRTC: DTLS), plus raw UDP as the
floor. Certificates come from benchmark/results/wifi/certs, generated with
both LAN IPs as SANs so hostname checks stay on.

Setup (see the run's README in the results directory for what was used):
  - robot: release robot-edge and proto-bench under ~/RoboProtocol, the
    certs + zenoh_tls_robot.json5 under ~/RoboProtocol/benchmark-wifi/
  - laptop: release robot-edge/operator-console/proto-bench, and the
    bench-mosquitto-wifi container (ports 1884/8884, the wifi certs)

Usage: run_wifi.py [proto ...]    (default: all)
"""
import json
import os
import re
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

BENCH = Path(__file__).resolve().parent
REPO = BENCH.parent
WIFI = BENCH / "results" / "wifi"
OUT = WIFI / datetime.now().strftime("%Y%m%d-%H%M%S")
OUT.mkdir(parents=True)

PI = "pi@192.168.2.19"
PI_IP, OP_IP = "192.168.2.19", "192.168.2.24"
PI_DIR = "/home/pi/RoboProtocol"
PI_CERTS = f"{PI_DIR}/benchmark-wifi/certs"
CERTS = WIFI / "certs"

RS = str(REPO / "target/release/proto-bench")
OPC = str(REPO / "target/release/operator-console")
PI_RS = f"{PI_DIR}/target/release/proto-bench"
PI_EDGE = f"{PI_DIR}/target/release/robot-edge"

SIZES = [int(x) for x in os.environ.get("SIZES", "16,1024,4096,16384,65536,262144,1048576").split(",")]
SKIP_LATENCY = os.environ.get("SKIP_LATENCY") == "1"  # for a throughput-only rerun
SLICE_THRESHOLD = 1100  # roboprotocol_core::bench::BENCH_SLICE_THRESHOLD
MAX_SIZE = {"rs-udp": 65507, "rs-webrtc": 65535}
RATES = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1_000, 2_000, 5_000, 10_000, 20_000, 50_000, 100_000, 200_000]
START_MBPS = 2.0  # first rate tried: the largest whose offered load is <= this
SEND_S, WARMUP_S, WINDOW_S = 10.0, 1.0, 8.0
# 98%, not run_loopback.py's 99%: each receiver counts over a fixed 8 s of
# wall-clock time, and Wi-Fi delay swings (~90 ms at p95) shift that count by
# about +-1% even when nothing is lost -- measured with reliable protocols at
# a fraction of the link (98.9%, 99.0%, 101.1%).
PASS_RATIO = 0.98
LATENCY_RUNS = 3
PROTOS = sys.argv[1:] or ["channel-b-raw", "rs-zenoh-tls", "rs-mqtt-tls", "rs-webrtc", "rs-udp"]
LATENCY_PROTOS = ["channel-b", "channel-b-raw", "rs-zenoh-tls", "rs-mqtt-tls", "rs-webrtc", "rs-udp"]
ANSI = re.compile(r"\x1b\[[0-9;]*m")

log_f = open(OUT / "run.log", "w")


def log(msg):
    print(msg, flush=True)
    log_f.write(msg + "\n")
    log_f.flush()


# --- the robot's half, over ssh ------------------------------------------------

def ssh(cmd, timeout=60):
    return subprocess.run(["ssh", "-o", "BatchMode=yes", PI, cmd], capture_output=True, text=True, timeout=timeout)


def pi_start(argv, name):
    """Start argv on the robot in the background; output goes to a log file
    there. Returns the log path. The command travels on stdin, so it never
    appears in a process's command line for `pgrep -f` to trip over."""
    log_path = f"/tmp/bench-{name}.log"
    # `cd` on its own line: `cd X && cmd &` would background the whole list
    # as a subshell that keeps ssh's output open, so ssh would never return.
    script = f"cd {PI_DIR}\nsetsid {' '.join(argv)} > {log_path} 2>&1 < /dev/null &\n"
    subprocess.run(["ssh", "-o", "BatchMode=yes", PI, "bash -s"], input=script, text=True, timeout=30)
    return log_path


def wait_for_link(max_wait_s=600):
    """Waits for the robot to answer ping, so a Wi-Fi drop (the laptop has
    roamed off the robot's network twice) pauses the run instead of
    recording zeros. Raises if it doesn't come back."""
    t0 = time.time()
    while subprocess.run(["ping", "-c", "1", "-W", "2", PI_IP], capture_output=True).returncode != 0:
        if time.time() - t0 > max_wait_s:
            raise RuntimeError(f"robot {PI_IP} unreachable for {max_wait_s}s")
        if int(time.time() - t0) % 30 == 0:
            log(f"  !! robot unreachable, waiting ({int(time.time() - t0)}s)")
        time.sleep(2)
    if time.time() - t0 > 3:
        log(f"  !! link back after {time.time() - t0:.0f}s")


def pi_stop():
    wait_for_link()
    # Exact process names only (see pi_start): robot-edge ignores SIGINT,
    # so straight to SIGKILL.
    ssh("pkill -KILL -x robot-edge; pkill -KILL -x proto-bench; sleep 0.5; true")


def pi_read(log_path):
    return ANSI.sub("", ssh(f"cat {log_path} 2>/dev/null").stdout)


def tls_args(side):
    """MQTT mTLS args for one side ("robot" on the CM4, "operator" here)."""
    base = PI_CERTS if side == "robot" else str(CERTS)
    return ["--port", "8884", "--tls", "--ca", f"{base}/dev-ca/ca.crt",
            "--cert", f"{base}/{side}/{side}.crt", "--key", f"{base}/{side}/{side}.key"]


def rs_args(proto, mode, side):
    """proto-bench args for one side. `side` is "robot" or "operator"."""
    name = proto[3:].replace("-tls", "")
    binary = PI_RS if side == "robot" else RS
    args = [binary, name, mode]
    if name == "udp":
        args += ["--host", "0.0.0.0" if mode in ("responder", "recv") else (PI_IP if side == "operator" else OP_IP), "--port", "5005"]
    elif name == "zenoh":
        cfg = f"{PI_DIR}/benchmark-wifi/zenoh_tls_robot.json5" if side == "robot" else str(WIFI / "zenoh_tls_operator.json5")
        args += ["--config", cfg]
    elif name == "mqtt":
        args += ["--host", OP_IP] + tls_args(side)
    elif name == "webrtc":
        listens = mode in ("responder", "recv")
        args += ["--host", "0.0.0.0" if listens else (PI_IP if side == "operator" else OP_IP), "--port", "8765"]
    return args


def edge_args(mode, *extra):
    return [PI_EDGE, "--listen", "0.0.0.0:4433", "--stub-bridge", "--bench", mode, "--zenoh-port", "17448",
            "--cert", f"{PI_CERTS}/robot/robot.crt", "--key", f"{PI_CERTS}/robot/robot.key",
            "--ca", f"{PI_CERTS}/dev-ca/ca.crt", *extra]


def opc_args(proto, *extra):
    raw = ["--bench-raw"] if proto == "channel-b-raw" else []
    return [OPC, "--connect", f"{PI_IP}:4433", "--server-name", "robot-edge", "--headless",
            "--cert", str(CERTS / "operator/operator.crt"), "--key", str(CERTS / "operator/operator.key"),
            "--ca", str(CERTS / "dev-ca/ca.crt"), *raw, *extra]


def local(argv, timeout):
    # A fresh process per run: never offer a stale 0-RTT ticket.
    subprocess.run(["rm", "-rf", str(REPO / ".session-cache")])
    return subprocess.run(argv, cwd=REPO, capture_output=True, text=True, timeout=timeout)


# --- latency -------------------------------------------------------------------

def latency(proto):
    pi_stop()
    if proto.startswith("channel-b"):
        pi_start(edge_args("echo"), "lat")
        time.sleep(2)
        argv = opc_args(proto, "--bench", "pingpong", "--bench-count", "2000", "--bench-payload-bytes", "64")
    else:
        pi_start(rs_args(proto, "responder", "robot"), "lat")
        time.sleep(3)
        argv = rs_args(proto, "pingpong", "operator") + ["--count", "2000", "--payload-bytes", "64"]
    try:
        r = local(argv, timeout=180)
    finally:
        pi_stop()
    m = re.search(r"n=\d+ \(trimmed.*", ANSI.sub("", r.stdout))
    return m.group(0) if m else (r.stdout.strip() or r.stderr.strip())[-400:]


# --- throughput, robot -> operator ---------------------------------------------

def trial(proto, size, rate):
    pi_stop()
    if proto.startswith("channel-b"):
        robot_log = pi_start(edge_args("send", "--bench-payload-bytes", str(size), "--bench-rate-hz", str(rate),
                                       "--bench-duration-s", str(SEND_S)), "send")
        time.sleep(2)
        r = local(opc_args(proto, "--bench", "recv", "--bench-warmup-s", str(WARMUP_S),
                           "--bench-duration-s", str(WINDOW_S)), timeout=120)
        r = subprocess.CompletedProcess(r.args, r.returncode, r.stdout + r.stderr, "")
        time.sleep(1)
    elif proto == "rs-webrtc":  # the receiver is the signaling listener: it starts first
        recv = subprocess.Popen(rs_args(proto, "recv", "operator") + ["--warmup-s", str(WARMUP_S), "--duration-s", str(WINDOW_S)],
                                cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        time.sleep(1)
        robot_log = pi_start(rs_args(proto, "send", "robot") + ["--payload-bytes", str(size), "--rate-hz", str(rate),
                                                               "--duration-s", str(SEND_S)], "send")
        out, err = recv.communicate(timeout=90)
        r = subprocess.CompletedProcess(recv.args, recv.returncode, out, err)
        time.sleep(2)
    else:
        # Sender runs 2 s longer than the others: the receiver counts only
        # after a warmup (its subscription has to reach the publisher across
        # the link first), and the window must still end inside the send.
        robot_log = pi_start(rs_args(proto, "send", "robot") + ["--payload-bytes", str(size), "--rate-hz", str(rate),
                                                               "--duration-s", str(SEND_S + 2)], "send")
        time.sleep(1)  # ssh start-up
        r = local(rs_args(proto, "recv", "operator") + ["--warmup-s", str(WARMUP_S + 0.5), "--duration-s", str(WINDOW_S)],
                  timeout=60)
        time.sleep(2)
    sender = pi_read(robot_log)
    pi_stop()
    return score(proto, size, rate, sender, r.stdout)


def score(proto, size, rate, sender, recv_out):
    m = re.search(r"sent (\d+) \w+ over ([\d.]+)s", sender)
    sent_rate = int(m.group(1)) / float(m.group(2)) if m and float(m.group(2)) > 0 else 0.0
    m = re.search(r"(\d+) msgs, (\d+) bytes in ([\d.]+)s", recv_out)
    msgs, nbytes, elapsed = (int(m.group(1)), int(m.group(2)), float(m.group(3))) if m else (0, 0, WINDOW_S)
    recv_rate = msgs / elapsed if elapsed else 0.0
    offered = min(rate, sent_rate) if sent_rate else rate
    t = {"target_hz": rate, "sent_hz": round(sent_rate, 1), "recv_hz": round(recv_rate, 1),
         "ratio": round(recv_rate / offered, 4) if offered else 0.0,
         "mbps": round(nbytes * 8 / elapsed / 1e6, 2) if elapsed else 0.0, "payload": size}
    m = re.search(r"(\d+) slice bytes received", recv_out)
    if proto.startswith("channel-b") and size > SLICE_THRESHOLD and m:
        # Sliced Channel B: scored on data delivered, since a frame missing a
        # few slices is still usable; complete messages reported alongside.
        padded = -(-size // 64) * 64
        data_rate = int(m.group(1)) / elapsed if elapsed else 0.0
        t["complete_ratio"] = t["ratio"]
        t["ratio"] = round(data_rate / (offered * padded), 4) if offered else 0.0
        t["data_mbps"] = round(data_rate * 8 / 1e6, 2)
    return t


def search(proto, size):
    start = max([r for r in RATES if r * size * 8 <= START_MBPS * 1e6] or [RATES[0]])
    best, trials = None, []
    for rate in [r for r in RATES if r >= start]:
        t = trial(proto, size, rate)
        if t["ratio"] < PASS_RATIO:  # one retry: a single miss is often connect/subscribe timing
            trials.append(t)
            t = trial(proto, size, rate)
            t["retried"] = True
        trials.append(t)
        extra = (f" (complete msgs {t['complete_ratio']:.3f}, data {t['data_mbps']} Mbps)"
                 if "complete_ratio" in t else "")
        log(f"  {proto:14} {size:8}B target={rate:>7} sent={t['sent_hz']:>9} recv={t['recv_hz']:>9} "
            f"ratio={t['ratio']:.3f} {t['mbps']} Mbps{extra}")
        if t["ratio"] < PASS_RATIO:
            if best is None and rate == start:
                best = {"below_start": True, **t}
            break
        best = t
        if t["sent_hz"] < 0.9 * rate:
            best["sender_limited"] = True
            break
    return {"best": best, "trials": trials}


def git(*args):
    return subprocess.run(["git", *args], cwd=REPO, capture_output=True, text=True).stdout.strip()


def main():
    meta = {
        "git": git("rev-parse", "--short", "HEAD"), "branch": git("rev-parse", "--abbrev-ref", "HEAD"),
        "robot": ssh("tr -d '\\0' < /proc/device-tree/model; echo; uname -r; iwgetid -r 2>/dev/null; "
                     "cat /proc/net/wireless | tail -1").stdout.strip(),
        "operator": subprocess.run(["bash", "-c", "uname -r; nmcli -t -f ACTIVE,SSID,SIGNAL,FREQ dev wifi | grep ^yes"],
                                   capture_output=True, text=True).stdout.strip(),
        "topology": f"robot {PI_IP} (CM4, Wi-Fi) <-> operator {OP_IP} (laptop, Wi-Fi), same AP; "
                    "mosquitto on the operator laptop; throughput robot -> operator",
        "sizes": SIZES, "send_s": SEND_S, "window_s": WINDOW_S, "pass_ratio": PASS_RATIO,
    }
    results = {"meta": meta, "latency": {}, "latency_runs": {}, "throughput": {}}
    log(f"output: {OUT}  commit {meta['git']} ({meta['branch']})")

    log(f"== latency (64B, 2000 round trips, {LATENCY_RUNS} runs each; keeping the middle median)")
    lat_protos = [] if SKIP_LATENCY else [p for p in LATENCY_PROTOS if p in PROTOS or p == "channel-b" and "channel-b-raw" in PROTOS]
    for proto in lat_protos:
        runs = [latency(proto) for _ in range(LATENCY_RUNS)]
        results["latency_runs"][proto] = runs
        med = [float(m.group(1)) if (m := re.search(r"median RTT=([\d.]+)us", x)) else float("inf") for x in runs]
        results["latency"][proto] = runs[sorted(range(len(runs)), key=med.__getitem__)[len(runs) // 2]]
        log(f"  {proto}: {results['latency'][proto]}")
        log(f"  {'':{len(proto)}}  medians across runs: {', '.join(f'{m:.1f}' for m in med)} us")
        (OUT / "results.json").write_text(json.dumps(results, indent=2))
    results["latency"]["ping"] = subprocess.run(["ping", "-c", "20", "-s", "36", PI_IP], capture_output=True,
                                                text=True).stdout.strip().splitlines()[-1]
    log(f"  ping: {results['latency']['ping']}")

    log(f"== throughput robot -> operator (max paced rate with >= {PASS_RATIO:.0%} delivered)")
    for size in SIZES:
        for proto in [p for p in PROTOS if size <= MAX_SIZE.get(p, size)]:
            results["throughput"][f"{proto}/{size}"] = search(proto, size)
            (OUT / "results.json").write_text(json.dumps(results, indent=2))
    log("DONE")


if __name__ == "__main__":
    sys.exit(main())
