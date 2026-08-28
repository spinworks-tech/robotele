# Performance & Benchmarks

Source of truth: `BENCHMARK.md` (the runbook) and `PERFORMANCE.md` (results). These
numbers are from real `robot-edge`/`operator-console` binaries against real XGO-Lite
V2 hardware over a real Wi-Fi hop — not the simulator, not loopback.

## CPU hot-path cost (software-only)

Measured directly in a release build, 1,000,000 iterations each:

| Stage | Cost |
|---|---|
| `TeleopCommand::pack` | 54.9 ns |
| `encode_channel_b_frame` (FlatBuffers) | 223.8 ns |
| `datagram::tag` | 8.0 ns |
| `datagram::untag` | 8.4 ns |
| `decode_channel_b_frame` | 9.3 ns |
| `TeleopCommand::unpack` | 9.6 ns |
| Recording copy (`payload.to_vec()`) | 8.7 ns |
| **Full sender-side path** (pack + encode + tag) | **337.6 ns** |
| **Full receiver-side path** (untag + decode + unpack + record-copy) | **31.9 ns** |

**Conclusion:** a full command round-trip costs ~370 ns of CPU time — against the
~24–31 ms RTTs measured below, that's roughly 0.0015% of the latency budget. This
workload is not, and was never going to be, CPU-bound; no code-level optimization
here would move the needle on end-to-end latency.

## Real hardware: Channel B only (no video)

`robot-edge --stub-bridge --tick-hz 50` vs `operator-console --headless --tick-hz 50`,
41s capture over real Wi-Fi:

```text
packets:         7106       duration: 41.010s     total wire bytes: 913910
throughput:      22285.0 B/s, 173.3 pkt/s
frame length:    mean=128.6B  min=81B  max=1454B
inter-arrival:   mean=5.77ms  stdev=28.68ms  max=2074.64ms
```

Plaintext UDP baseline, same hop, fixed 128B payload @ 50Hz:

```text
packets:         1500       duration: 29.981s     total wire bytes: 255000
throughput:      8505.2 B/s, 50.0 pkt/s
frame length:    mean=170.0B (fixed)
inter-arrival:   mean=20.00ms  stdev=5.35ms  max=108.50ms
```

QUIC's mean frame length looks smaller than the plaintext baseline's fixed 170B —
this is *not* "QUIC is cheaper than plaintext." QUIC generates roughly 2x the packet
count of the baseline for the same data rate (bare ACK-only packets mixed into the
capture), dragging the mean down.

## Real hardware: Channel A (video) + Channel B together

`--camera` added (real `ov5647`, `libcamera-vid`, 640x480@30fps H.264), 47.45s:

```text
packets:         23425      duration: 47.450s     total wire bytes: 11614161
throughput:      244764.0 B/s (~1.96 Mbps), 493.7 pkt/s
frame length:    mean=495.8B
inter-arrival:   mean=2.03ms  max=3703.31ms
```

The E-Stop that fired mid-run wasn't a video-contention failure — the timestamp lined
up exactly with the test harness's own `timeout 40` killing `operator-console`. The
Class D watchdog correctly latched E-Stop 400 ms after Channel B activity stopped:
SR-3.2/SR-3.3 working as designed. During the genuine ~40s of concurrent Channel A + B
streaming before that, no watchdog trip occurred.

## Investigating the "multi-second stall" — corrected

An initial read of the data flagged 1888 "outage gaps" and a max 3.7s inter-arrival
gap as possible radio-level stalls. Pulling exact gap timestamps showed each run's
largest gaps forming a clean doubling sequence (130.6 / 260.3 / 520.4 / 1037.9 /
2074.6 ms) — that's **QUIC's own PTO (probe timeout) exponential backoff during test
teardown**, not radio jitter. Both flagged "outages" were each test's own teardown
tail after the benchmark harness's `timeout` wrapper killed the client — nothing
indicates a real Wi-Fi/mesh problem in either run's actual steady-state traffic.

## Host contention: a real, separate finding

The vendor XGO demo process (`sudo python3 main.py`) was running continuously at
~30% CPU in uninterruptible-I/O state for the Pi's entire uptime, on top of the
benchmark's own `tshark` capture overhead — on a 4-core CM4 that showed a load
average of ~2.0 even at idle-ish moments. Killing it and re-running:

| | Encrypted, vendor running | Encrypted, vendor killed | Plaintext, vendor running | Plaintext, vendor killed |
|---|---|---|---|---|
| inter-arrival stdev | 28.68ms | **5.98ms** | 5.35ms | **0.93ms** |
| inter-arrival max | 2074.6ms* | **20.96ms** | 108.5ms | **33.08ms** |
| live RTT reading | `--ms` (unpopulated) | **~19-20ms** | n/a | n/a |

\*Since corrected above — that value was QUIC teardown backoff, not a real stall.
Even so, steady-state jitter tightened roughly 5x on both encrypted and plaintext
runs once the vendor process was removed — consistent with host scheduling
contention mattering at the margin, even though it was never the source of the
dramatic gaps originally attributed to it.

**Resolution:** the vendor demo is disabled permanently. Its autostart line in
`/home/pi/start1.sh` (invoked from `rc.local`) is commented out, not deleted, and the
original is preserved at `/home/pi/start1.sh.bak` — a one-line uncomment away from
being restored if ever needed, and it survives reboots either way since the change is
on the Pi's own filesystem, not this repository.

## Bugs found and fixed during this pass

1. **`benchmark/analyze_pcap.py` couldn't run on target hardware** — used
   `str | None` (PEP 604) syntax requiring Python 3.10+, but the Pi runs 3.9.2 and
   the dev machine 3.8.10. Fixed with `from __future__ import annotations`.
2. **Zombie `libcamera-vid` process on every `--camera` session** —
   `crates/robot-edge/src/video/capture.rs` (the "downstream receiver dropped, shut
   down capture" path) called `child.start_kill()` and returned without
   `child.wait().await`, unlike the normal EOF/error path just below it. Confirmed
   on real hardware: the child outlived its session as a `<defunct>` zombie until
   `robot-edge` itself was killed. Fixed by adding `let _ = child.wait().await;`
   after `start_kill()`. Verified on real hardware — session ended, `ffplay`/
   `libcamera-vid` fully gone after the QUIC idle-timeout teardown, not even a
   transient zombie.

## Reproducing these measurements

Full step-by-step runbook: [`BENCHMARK.md`](../BENCHMARK.md).

1. **Plain vs. encrypted overhead** — capture real Channel B traffic with `tshark`,
   measure with `benchmark/analyze_pcap.py`, capture a size/rate-matched plaintext
   UDP baseline with `benchmark/raw_udp_baseline.py`, and diff the two.
2. **Wi-Fi drop test** — `benchmark/wifi_drop_helper.sh wlan0 15` drops the
   operator's interface mid-session; confirm the Class D watchdog latches E-Stop
   within ~400 ms of the last received frame, then recovers once the interface
   returns.

Tools needed: `cargo`, `tshark` (Wireshark ≥3.4), `python3` (stdlib only — no pip
installs), `sudo ip link`/`nmcli`/`rfkill` for the drop test, `chrony`/`ntpd`
(optional, only for cross-host one-way latency).

The dev-only mTLS certs at `certs/` work fine for a two-host test with no
regeneration — see [Security § Dev certificates](04-security.md#dev-certificates-are-not-production-credentials).
