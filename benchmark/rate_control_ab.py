#!/usr/bin/env python3
"""robot-edge --lossy-rate-control off vs on, on the CM4 over Wi-Fi
(docs/12): 50 Hz paced control under load L3 (camera + lidar + depth +
cloud, ~2x the link), alternating over rounds, plus a camera-only pair to
check the cap doesn't starve video on a link that isn't overloaded. Logs an
idle ping before each round, since this Wi-Fi varies a lot. Reads the
controller's own end-of-session stats from robot-edge's log. Uses
control_under_load.py's run().

Usage: rate_control_ab.py [rounds]
"""
import json
import re
import statistics
import subprocess
import sys

import control_under_load as c

ROUNDS = int(sys.argv[1]) if len(sys.argv) > 1 else 3
BASE = {"L3": list(c.LOADS["L3"]), "L1": list(c.LOADS["L1"])}
FLAGS = {"off": [], "on": ["--lossy-rate-control", "on"]}
_orig_ssh, _log = c.ssh, {}


def _ssh(script, timeout=60):
    r = _orig_ssh(script, timeout)
    if "sed 's/" in script:
        _log["robot"] = r.stdout
    return r


c.ssh = _ssh


def idle_ping():
    out = subprocess.run(["ping", "-c", "20", "-i", "0.2", c.PI_IP], capture_output=True, text=True).stdout
    times = sorted(float(t) for t in re.findall(r"time=([\d.]+)", out))
    return {"p50": times[len(times) // 2], "p90": times[int(len(times) * 0.9)]} if times else {}


def go(load, label, rnd, rows):
    c.LOADS[load] = BASE[load] + FLAGS[label]
    t = c.run(load, "raw")
    m = re.search(r"mean_mbps=([\d.]+) min_mbps=([\d.]+) max_mbps=([\d.]+) decreases=(\d+)", _log.get("robot", ""))
    t.update(label=label, round=rnd, cap_mean_mbps=float(m.group(1)) if m else None, cap_cuts=int(m.group(4)) if m else None)
    rows.append(t)
    if "error" in t:
        c.log(f"  round {rnd} {load} cap {label:3} ERROR {t['error'][-200:]}")
        return
    cap = f" cap mean {t['cap_mean_mbps']:.1f} Mbps, {t['cap_cuts']} cuts" if t["cap_mean_mbps"] is not None else ""
    c.log(f"  round {rnd} {load} cap {label:3} p50={t['p50']:.1f} p99={t['p99']:.0f} max={t['max']:.0f} ms >100ms={t['late100']:.1%} "
          f">400ms={t['late400']:.1%} lost={int(t['lost'])} video={t['video_mbps']:.1f} sensors={t['sensor_mbps']:.1f} Mbps "
          f"latches={t['estop_latches']}{cap}")


def main():
    rows = []
    for rnd in range(1, ROUNDS + 1):
        p = idle_ping()
        c.log(f"== round {rnd}: idle ping p50={p.get('p50')} p90={p.get('p90')} ms")
        for label in FLAGS:
            go("L3", label, rnd, rows)
    c.log("== camera only")
    for label in FLAGS:
        go("L1", label, 1, rows)
    (c.OUT / "rate_control.json").write_text(json.dumps(rows, indent=2))
    c.log("== L3 medians over rounds")
    for label in FLAGS:
        ts = [t for t in rows if t["load"] == "L3" and t["label"] == label and "error" not in t]
        if not ts:
            continue
        m = lambda k: statistics.median(t[k] for t in ts)  # noqa: E731
        c.log(f"  cap {label:3} p50={m('p50'):.1f} p99={m('p99'):.0f} ms >100ms={m('late100'):.1%} >400ms={m('late400'):.1%} "
              f"lost={m('lost'):.0f} video={m('video_mbps'):.1f} sensors={m('sensor_mbps'):.1f} Mbps "
              f"latched in {sum(t['estop_latches'] > 0 for t in ts)}/{len(ts)} runs")
    c.log("DONE")


if __name__ == "__main__":
    main()
