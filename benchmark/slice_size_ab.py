#!/usr/bin/env python3
"""Channel B throughput robot -> operator over Wi-Fi with 1,100-byte sensor
slices against full-size ones (robot-edge --slice-payload-bytes 1100 vs 0),
alternating the two at each size and doing everything twice, so drift in
the Wi-Fi shows up as round-to-round differences rather than as a
difference between the variants. Uses run_wifi.py's rate search and setup
(benchmark/setup_wifi.sh up first).

Usage: slice_size_ab.py
"""
import json

import run_wifi as w

SIZES = [16384, 65536, 262144]
VARIANTS = {"1100": ["--slice-payload-bytes", "1100"], "full": ["--slice-payload-bytes", "0"]}
ROUNDS = 2


def main():
    results = {"sizes": SIZES, "rounds": ROUNDS, "runs": []}
    for rnd in range(1, ROUNDS + 1):
        for size in SIZES:
            for name, flags in VARIANTS.items():
                w.EDGE_EXTRA[:] = flags
                w.log(f"== round {rnd}, {size} B, slices {name}")
                r = w.search("channel-b-raw", size)
                results["runs"].append({"round": rnd, "size": size, "slices": name, **r})
                (w.OUT / "slice_size_ab.json").write_text(json.dumps(results, indent=2))
    w.log("DONE")


if __name__ == "__main__":
    main()
