#!/usr/bin/env python3
"""Control under load on the robot (docs/12-control-under-load-benchmark.md),
Channel B over real Wi-Fi: 50 Hz paced pings from the operator laptop to
robot-edge on the CM4 while the robot's uplink carries its camera and
`--sim-sensor` data.

Loads (robot -> operator, against a ~20 Mbps 2.4 GHz link):
  L0  control only
  L1  camera (640x480, 30 fps, ~2 Mbps)
  L2  camera + navigation cloud (~8 Mbps)
  L3  camera + lidar + depth + cloud (~40 Mbps offered, about 2x the link)

Each run: robot-edge --bench echo (stub bridge) with that load, then
operator-console --bench pingpace: 64 B pings at 50 Hz, 10 s warmup, 60 s
timed, each sent on schedule whether or not earlier replies are back. Both
Channel B variants (transport only, and full FlatBuffers frame). Loads and
variants alternate within each round. Also counts watchdog E-Stop latches
in robot-edge's log: a latch means control went quiet long enough to stop
the real robot.

Uses the default dev certs; needs release robot-edge on the robot and
operator-console here. Usage: control_under_load.py [rounds]
"""
import json
import re
import statistics
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

BENCH = Path(__file__).resolve().parent
REPO = BENCH.parent
OUT = BENCH / "results" / "wifi" / ("control-" + datetime.now().strftime("%Y%m%d-%H%M%S"))
OUT.mkdir(parents=True)
PI, PI_IP = "pi@192.168.2.19", "192.168.2.19"
OPC = str(REPO / "target/release/operator-console")
LOADS = {
    "L0": [],
    "L1": ["--camera"],
    "L2": ["--camera", "--sim-sensor", "cloud"],
    "L3": ["--camera", "--sim-sensor", "lidar", "--sim-sensor", "depth", "--sim-sensor", "cloud"],
}
VARIANTS = {"raw": ["--bench-raw"], "full": []}
RATE_HZ, WARMUP_S, DURATION_S = 50, 10, 60
ROUNDS = int(sys.argv[1]) if len(sys.argv) > 1 else 2
FIELDS = ["sent", "replies", "lost", "p50", "p99", "max", "late100", "late400", "video_mbps", "sensor_mbps"]
log_f = open(OUT / "run.log", "w")


def log(msg):
    print(msg, flush=True)
    log_f.write(msg + "\n")
    log_f.flush()


def ssh(script, timeout=60):
    # The script travels on stdin, with cd on its own line (see run_wifi.py).
    return subprocess.run(["ssh", "-o", "BatchMode=yes", PI, "bash -s"], input=script, capture_output=True,
                          text=True, timeout=timeout)


def run(load, variant):
    ssh("pkill -INT -x robot-edge; sleep 1; pkill -KILL -x robot-edge; true")
    ssh(f"cd /home/pi/RoboProtocol\nsetsid target/release/robot-edge --stub-bridge --bench echo --robot-id control-load "
        f"{' '.join(LOADS[load])} > /tmp/control-load.log 2>&1 < /dev/null &\n")
    time.sleep(5)  # camera start-up
    subprocess.run(["rm", "-rf", str(REPO / ".session-cache")])
    r = subprocess.run([OPC, "--connect", f"{PI_IP}:4433", "--server-name", "robot-edge", "--headless",
                        "--bench", "pingpace", *VARIANTS[variant], "--bench-payload-bytes", "64",
                        "--bench-rate-hz", str(RATE_HZ), "--bench-warmup-s", str(WARMUP_S),
                        "--bench-duration-s", str(DURATION_S)],
                       cwd=REPO, capture_output=True, text=True, timeout=WARMUP_S + DURATION_S + 60)
    robot_log = ssh("sed 's/\\x1b\\[[0-9;]*m//g' /tmp/control-load.log").stdout
    ssh("pkill -INT -x robot-edge; sleep 1; rm -f /tmp/control-load.log; true")
    m = re.search(r"pingpace sent=(\d+) replies=(\d+) lost=(\d+) p50=([\d.na]+) p99=([\d.na]+) max=([\d.na]+) ms "
                  r"late100=([\d.]+) late400=([\d.]+) video_mbps=([\d.]+) sensor_mbps=([\d.]+)", r.stdout)
    if not m:
        return {"load": load, "variant": variant, "error": (r.stdout + r.stderr)[-300:]}
    t = dict(zip(FIELDS, (float(x) for x in m.groups())))
    t.update(load=load, variant=variant, estop_latches=robot_log.count("E-Stop latched"))
    return t


def main():
    results = []
    log(f"output: {OUT}  ({ROUNDS} rounds, {RATE_HZ} Hz, {WARMUP_S}+{DURATION_S} s per run)")
    for rnd in range(1, ROUNDS + 1):
        for load in LOADS:
            for variant in VARIANTS:
                t = run(load, variant)
                t["round"] = rnd
                results.append(t)
                (OUT / "results.json").write_text(json.dumps(results, indent=2))
                if "error" in t:
                    log(f"  round {rnd} {load} {variant:4} ERROR {t['error']}")
                    continue
                log(f"  round {rnd} {load} {variant:4} p50={t['p50']:.1f} p99={t['p99']:.1f} max={t['max']:.1f} ms "
                    f">100ms={t['late100']:.2%} >400ms={t['late400']:.2%} lost={int(t['lost'])} "
                    f"video={t['video_mbps']:.1f} sensors={t['sensor_mbps']:.1f} Mbps estop={t['estop_latches']}")
    log("== medians over rounds (p99 inflation against L0 of the same variant)")
    for variant in VARIANTS:
        base = None
        for load in LOADS:
            ts = [t for t in results if t["load"] == load and t["variant"] == variant and "error" not in t]
            if not ts:
                continue
            med = {k: statistics.median(t[k] for t in ts) for k in FIELDS + ["estop_latches"]}
            base = base or med["p99"]
            log(f"  {variant:4} {load} p50={med['p50']:.1f} p99={med['p99']:.1f} max={med['max']:.1f} ms "
                f"inflation={med['p99'] / base:.1f}x >100ms={med['late100']:.2%} >400ms={med['late400']:.2%} "
                f"lost={med['lost']:.0f} video={med['video_mbps']:.1f} sensors={med['sensor_mbps']:.1f} Mbps "
                f"estop={med['estop_latches']:.0f}")
    log("DONE")


if __name__ == "__main__":
    main()
