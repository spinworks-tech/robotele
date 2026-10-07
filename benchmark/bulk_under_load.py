#!/usr/bin/env python3
"""Do bulk objects hurt control? (docs/13 "Bulk objects", docs/12.) 50 Hz
paced control from the operator laptop while robot-edge on the CM4 sends its
camera plus sim-map occupancy grids (600 KB each) at different rates, with
and without --lossy-rate-control. Cases alternate within rounds. Reports
control round trips alongside the maps actually delivered. Uses
control_under_load.py's run().

Usage: bulk_under_load.py [rounds]
"""
import json
import statistics
import sys

import control_under_load as c

ROUNDS = int(sys.argv[1]) if len(sys.argv) > 1 else 2
CASES = {
    "camera": ["--camera"],
    "camera+map 1/s": ["--camera", "--sim-map", "1"],
    "camera+map 4/s": ["--camera", "--sim-map", "0.25"],
    "camera+map 4/s+cap": ["--camera", "--sim-map", "0.25", "--lossy-rate-control", "on"],
}


def main():
    rows = []
    for rnd in range(1, ROUNDS + 1):
        for name, flags in CASES.items():
            c.LOADS["bulk"] = flags
            t = c.run("bulk", "raw")
            t.update(case=name, round=rnd)
            rows.append(t)
            if "error" in t:
                c.log(f"  round {rnd} {name:20} ERROR {t['error'][-200:]}")
                continue
            c.log(f"  round {rnd} {name:20} p50={t['p50']:.1f} p99={t['p99']:.0f} ms >100ms={t['late100']:.1%} "
                  f">400ms={t['late400']:.1%} lost={int(t['lost'])} video={t['video_mbps']:.1f} Mbps "
                  f"maps={int(t['bulk_objects'])} ({t['bulk_mbps']:.1f} Mbps) latches={t['estop_latches']}")
    (c.OUT / "bulk.json").write_text(json.dumps(rows, indent=2))
    c.log("== medians over rounds")
    for name in CASES:
        ts = [t for t in rows if t["case"] == name and "error" not in t]
        if ts:
            m = lambda k: statistics.median(t[k] for t in ts)  # noqa: E731
            c.log(f"  {name:20} p50={m('p50'):.1f} p99={m('p99'):.0f} ms >100ms={m('late100'):.1%} >400ms={m('late400'):.1%} "
                  f"lost={m('lost'):.0f} video={m('video_mbps'):.1f} Mbps maps={m('bulk_objects'):.0f} ({m('bulk_mbps'):.1f} Mbps) "
                  f"latched in {sum(t['estop_latches'] > 0 for t in ts)}/{len(ts)}")
    c.log("DONE")


if __name__ == "__main__":
    main()
