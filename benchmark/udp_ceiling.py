#!/usr/bin/env python3
"""Where does datagram throughput from the robot's CM4 stop, and why?
(docs/14-protocol-comparison-wifi.md found every datagram protocol topping
out at 8-12 Mbps robot -> operator, while MQTT over TCP reached 26 Mbps.)

For each direction and payload size, steps the paced send rate of raw UDP
(proto-bench udp) up until the receiver gets < 98% of what was offered, and
reports the highest passing rate in packets/s and Mbps. A ceiling that stays
at the same packets/s across sizes is per-packet cost; one at the same Mbps
is bandwidth. For robot -> operator trials it also reads the robot's
interface and UDP counters, to tell drops on the robot (its Wi-Fi driver
can't drain the queue) from loss in the air. Then one bulk TCP stream,
robot -> operator, as the reference for what the radio carries.

Needs proto-bench at /tmp/proto-bench on the robot (copied in for the test)
and target/release/proto-bench here. Hosts as in run_wifi.py.

Usage: udp_ceiling.py
"""
import json
import re
import subprocess
import time
from datetime import datetime
from pathlib import Path

BENCH = Path(__file__).resolve().parent
REPO = BENCH.parent
OUT = BENCH / "results" / "wifi" / ("ceiling-" + datetime.now().strftime("%Y%m%d-%H%M%S"))
OUT.mkdir(parents=True)
PI, PI_IP, OP_IP = "pi@192.168.2.19", "192.168.2.19", "192.168.2.24"
PI_RS, RS = "/tmp/proto-bench", str(REPO / "target/release/proto-bench")
SIZES = [64, 256, 512, 1024, 1400]
RATES = [500, 1_000, 1_500, 2_000, 2_500, 3_000, 4_000, 5_000, 6_000, 8_000, 10_000, 12_000, 16_000, 20_000, 30_000]
SEND_S, WARMUP_S, WINDOW_S, PASS = 7.0, 1.5, 4.0, 0.98
log_f = open(OUT / "run.log", "w")


def log(msg):
    print(msg, flush=True)
    log_f.write(msg + "\n")
    log_f.flush()


def ssh(cmd, timeout=60, stdin=None):
    return subprocess.run(["ssh", "-o", "BatchMode=yes", PI, cmd], input=stdin, capture_output=True,
                          text=True, timeout=timeout)


def pi_bg(cmd, log_path):
    # cd-free and on its own line: see run_wifi.py's pi_start.
    ssh("bash -s", stdin=f"setsid {cmd} > {log_path} 2>&1 < /dev/null &\n", timeout=30)


def pi_counters():
    """wlan0 TX packets/dropped and UDP send-buffer errors on the robot."""
    out = ssh("cat /sys/class/net/wlan0/statistics/tx_packets /sys/class/net/wlan0/statistics/tx_dropped; "
              "grep '^Udp:' /proc/net/snmp | tail -1").stdout.split("\n")
    udp = out[2].split()
    return {"tx_packets": int(out[0]), "tx_dropped": int(out[1]), "snd_buf_errors": int(udp[6])}


def parse_sent(text):
    m = re.search(r"sent (\d+) \w+ over ([\d.]+)s", text)
    return int(m.group(1)) / float(m.group(2)) if m and float(m.group(2)) else 0.0


def parse_recv(text):
    m = re.search(r"(\d+) msgs, (\d+) bytes in ([\d.]+)s", text)
    return int(m.group(1)) / float(m.group(3)) if m and float(m.group(3)) else 0.0


def trial(direction, size, rate):
    send_args = f"--payload-bytes {size} --rate-hz {rate} --duration-s {SEND_S}"
    if direction == "robot->operator":
        before = pi_counters()
        recv = subprocess.Popen([RS, "udp", "recv", "--host", "0.0.0.0", "--port", "5005",
                                 "--duration-s", str(WINDOW_S + 6)], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True)
        time.sleep(0.5)
        pi_bg(f"{PI_RS} udp send --host {OP_IP} --port 5005 {send_args}", "/tmp/ceil-send.log")
        # The receiver counts from its own start; take its rate over a window
        # inside the send by running it long and using the sender's own rate
        # as the offered load.
        out, _ = recv.communicate(timeout=60)
        time.sleep(0.5)
        sent_hz = parse_sent(ssh("cat /tmp/ceil-send.log").stdout)
        after = pi_counters()
        recv_total = re.search(r"(\d+) msgs", out)
        received = int(recv_total.group(1)) if recv_total else 0
        delta = {k: after[k] - before[k] for k in after}
    else:
        pi_bg(f"{PI_RS} udp recv --host 0.0.0.0 --port 5005 --duration-s {WINDOW_S + 6}", "/tmp/ceil-recv.log")
        time.sleep(1.0)
        s = subprocess.run([RS, "udp", "send", "--host", PI_IP, "--port", "5005", *send_args.split()],
                           capture_output=True, text=True, timeout=60)
        time.sleep(4.0)
        sent_hz = parse_sent(s.stderr)
        received = int(m.group(1)) if (m := re.search(r"(\d+) msgs", ssh("cat /tmp/ceil-recv.log").stdout)) else 0
        delta = {}
    # Every datagram sent lands inside the receiver's run (it starts first and
    # outlasts the send), so the ratio is simply received / sent.
    sent_total = sent_hz * SEND_S
    ratio = received / sent_total if sent_total else 0.0
    return {"direction": direction, "size": size, "target_hz": rate, "sent_hz": round(sent_hz, 1),
            "ratio": round(ratio, 4), "mbps": round(sent_hz * size * 8 / 1e6, 2), **delta}


def search(direction, size):
    best, trials = None, []
    for rate in RATES:
        t = trial(direction, size, rate)
        if t["ratio"] < PASS:
            trials.append(t)
            t = trial(direction, size, rate)
            t["retried"] = True
        trials.append(t)
        extra = (f" robot: tx_pkts={t['tx_packets']} tx_dropped={t['tx_dropped']} sndbuf_err={t['snd_buf_errors']}"
                 if "tx_packets" in t else "")
        log(f"  {direction:16} {size:5}B target={rate:>6} sent={t['sent_hz']:>8} ratio={t['ratio']:.3f} "
            f"{t['mbps']} Mbps{extra}")
        if t["ratio"] < PASS:
            break
        best = t
        if t["sent_hz"] < 0.9 * rate:
            best["sender_limited"] = True
            break
    return {"best": best, "trials": trials}


def tcp_reference(seconds=10):
    """One bulk TCP stream robot -> operator (Python sockets on both ends)."""
    server = subprocess.Popen(["python3", "-c", (
        "import socket,time\n"
        "s=socket.socket(); s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1); s.bind(('0.0.0.0', 5006)); s.listen(1)\n"
        "c,_=s.accept(); n=0; t0=None\n"
        "while True:\n"
        "    b=c.recv(1<<16)\n"
        "    if not b: break\n"
        "    t0=t0 or time.time(); n+=len(b)\n"
        "print(n, time.time()-t0)\n")], stdout=subprocess.PIPE, text=True)
    time.sleep(0.5)
    ssh(f"python3 -c \"import socket,time; s=socket.create_connection(('{OP_IP}',5006)); b=bytes(1<<16); "
        f"e=time.time()+{seconds}\nwhile time.time()<e: s.sendall(b)\ns.close()\"", timeout=seconds + 30)
    n, secs = server.communicate(timeout=30)[0].split()
    return round(int(n) * 8 / float(secs) / 1e6, 2)


def main():
    results = {"sizes": SIZES, "pass": PASS, "send_s": SEND_S, "ceilings": {}}
    log(f"output: {OUT}")
    for direction in ["robot->operator", "operator->robot"]:
        log(f"== {direction}: raw UDP, highest paced rate with >= {PASS:.0%} delivered")
        for size in SIZES:
            results["ceilings"][f"{direction}/{size}"] = search(direction, size)
            (OUT / "results.json").write_text(json.dumps(results, indent=2))
    log("== reference: one bulk TCP stream robot -> operator, 10 s")
    results["tcp_robot_to_operator_mbps"] = [tcp_reference() for _ in range(3)]
    log(f"  {results['tcp_robot_to_operator_mbps']} Mbps")
    (OUT / "results.json").write_text(json.dumps(results, indent=2))
    log("DONE")


if __name__ == "__main__":
    main()
