#!/usr/bin/env python3
"""Channel B over Wi-Fi: robot-edge --sensor-frame-policy cut against
finish, at fixed rates near where 16, 64 and 256 KB messages started
failing (docs/14, "Full-size slices on the CM4"). For each trial it
records data delivered, messages complete, and the delay of complete
messages above the run's fastest (so the robot/operator clock offset
cancels). Policies alternate within each round; rounds repeat, so Wi-Fi
drift shows up between rounds rather than between policies. Uses
run_wifi.py (benchmark/setup_wifi.sh up first).

Usage: frame_policy_ab.py
"""
import json
import statistics

import run_wifi as w

SCENARIOS = [(16384, 50), (65536, 20), (262144, 10)]
POLICIES = ["cut", "finish"]
ROUNDS = 4


def main():
    trials = []
    for rnd in range(1, ROUNDS + 1):
        for size, rate in SCENARIOS:
            for policy in POLICIES:
                w.EDGE_EXTRA[:] = ["--sensor-frame-policy", policy]
                t = w.trial("channel-b-raw", size, rate)
                t.update(round=rnd, policy=policy)
                trials.append(t)
                w.log(f"  round {rnd} {size:7}B x {rate:3}/s {policy:6} data={t['ratio']:.3f} "
                      f"complete={t.get('complete_ratio', 0):.3f} msgs/s={t['recv_hz']} "
                      f"delay p50={t.get('delay_p50_ms')} p90={t.get('delay_p90_ms')} ms")
                (w.OUT / "frame_policy_ab.json").write_text(json.dumps(trials, indent=2))
    w.log("== medians over rounds")
    for size, rate in SCENARIOS:
        for policy in POLICIES:
            ts = [t for t in trials if t["payload"] == size and t["policy"] == policy]
            med = lambda k: statistics.median(t[k] for t in ts if t.get(k) is not None)  # noqa: E731
            w.log(f"  {size:7}B x {rate:3}/s {policy:6} data={med('ratio'):.3f} complete={med('complete_ratio'):.3f} "
                  f"msgs/s={med('recv_hz'):.1f} delay p50={med('delay_p50_ms'):.1f} p90={med('delay_p90_ms'):.1f} ms")
    w.log("DONE")


if __name__ == "__main__":
    main()
