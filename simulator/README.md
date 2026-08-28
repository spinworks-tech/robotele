# RoboProtocol Simulator

A packet-level discrete-event simulator of RoboProtocol (see
`../REQUIREMENTS.md`, `../DESIGN.md`) over a standard consumer network
connection between a teleoperator and a humanoid robot. It runs four
progressively more complex teleoperation scenarios against a set of
injected network failures, and reports whether the protocol's own
stated NFR/SR targets actually hold.

For the full design, traceability to specific REQUIREMENTS.md/DESIGN.md
clauses, documented assumptions, and findings surfaced while building
this, see [SPEC.md](SPEC.md).

## Install

Runs on Python 3.8+. If you already have numpy/pandas/matplotlib/pytest
installed, no install is needed; otherwise:

```bash
cd simulator
pip install -r requirements.txt
```

## Scenarios

| Key | Description |
|---|---|
| `simple_arm` | Single-arm control, no video |
| `arm_wrist_cam` | + one wrist camera |
| `arm_wrist_fixed_cam` | + a fixed/head stereo camera |
| `dual_arm` | Both arms + both wrist cameras + head camera |

## Failure presets

`bandwidth_drop`, `blackout`, `packet_loss_20`, `packet_loss_40`,
`jitter_spike`, `latency_ramp`, `latency_spike_recover`, `handover`,
`compound_congestion`, or `none` for baseline. Run
`python -m roboprotocol_sim list-failures` for a one-line description
of each.

## Usage

```bash
# One scenario x one failure x one network profile:
python -m roboprotocol_sim run \
  --scenario dual_arm --failure bandwidth_drop --network cellular_5g \
  --duration 20 --out output

# Same run, plus a naive WebSocket/TCP baseline for comparison:
python -m roboprotocol_sim run \
  --scenario dual_arm --failure blackout --compare-naive --out output

# Every scenario x every failure preset, plus a comparison table:
python -m roboprotocol_sim sweep --network home_broadband_wifi6 --out output

# What's available:
python -m roboprotocol_sim list-scenarios
python -m roboprotocol_sim list-failures
```

Each run writes to `output/<scenario>__<failure>__<network>/`:

* `summary.md` — headline stats + a PASS/FAIL/N/A table against 7
  specific REQUIREMENTS.md NFR/SR clauses
* `rtt_and_tier.png` — RTT over time + the safety-tier timeline
* `bandwidth.png` — Channel A/B bandwidth utilization vs uplink capacity
* `video_bitrate.png` — adaptive bitrate per camera stream (if any)
* `tdpa_energy.png` — haptic passivity proxy energy integral
* `*.csv` — the raw per-datagram / per-sample data behind the plots

`sweep` additionally writes `output/comparison.md`, one row per
scenario × failure combination.

### Naive WebSocket/TCP baseline (`--compare-naive`)

Adding `--compare-naive` to `run` or `sweep` replays the same
scenario/failure over a second transport model: one ordered, reliable,
unprioritized stream (full float32 fields, fixed video bitrate, no
FEC, no dedicated watchdog) carrying everything together — how a
from-scratch `ws://` teleop demo would actually be built, and how it
behaves under the exact same network conditions. It adds
`naive_comparison.png` and a comparison table to `summary.md`. See
[SPEC.md](SPEC.md#naive-websockettcp-baseline-comparison) for the
full model. The headline result: under the standard `blackout` preset,
RoboProtocol's watchdog E-Stops within 200ms (Class B), while the
naive stack's coarse ping/pong liveness check never even fires —
commands are just severely delayed by head-of-line blocking and then
silently resume once the link recovers, with no safety trigger at all.

## Tests

```bash
pytest -q
```

38 tests covering: payload sizing/quantization/splitting math (including
the FR-3.1 haptic floor), safety-tier boundary values and the corrected
SR-4.1 watchdog thresholds, the TDPA passivity proxy, FlexFEC's
recovery/failure boundary, the naive baseline's head-of-line blocking
and coarse liveness detection, and an end-to-end smoke run of every
scenario × every failure preset.
