# Protocol comparison over Wi-Fi: robot CM4 to operator laptop

This report repeats [11 — Protocol comparison](11-protocol-comparison.md)
on real hardware: the XGO-Lite's Raspberry Pi CM4 and an operator laptop,
talking over Wi-Fi through the same access point. It compares RoboProtocol's
Channel B with Zenoh, MQTT and WebRTC, all with mutual TLS, plus raw UDP as
the floor.

Two things are new compared with doc 11:

- **Throughput runs robot to operator.** The CM4 sends and the laptop
  receives. That is the direction sensor data travels.
- **Messages go up to 1 MB.** Channel B carries anything larger than one
  datagram as sensor slices ([13](13-large-sensor-payloads.md)), through the
  same queue `robot-edge` uses for real sensor frames.

## Summary

- **Latency.** Channel B's median round trip is 2.0–2.2 ms with mutual
  TLS, 0.4–0.6 ms above unencrypted raw UDP (1.6 ms) and ahead of WebRTC
  (2.6 ms). Zenoh and MQTT, both over TCP, sit at 8.4–8.8 ms, about 4×
  Channel B. Channel B's p95 is 4.7–5.8 ms. The laptop's Wi-Fi power saving
  was hiding this: with it on, Channel B's median was 3.8–3.9 ms and its
  p95 about 90 ms.
- **Small messages.** At 16 B, Channel B sustained 20,000 msg/s, Zenoh
  50,000 and MQTT about 41,000 (its sender's limit). Raw UDP could not
  sustain even 10,000. Protocols that pack several small messages into one
  packet win on this link, where packets, not bytes, run out first.
- **Large messages.** Datagram traffic from the CM4 (raw UDP, Channel B,
  WebRTC) topped out at about 8–12 Mbps. MQTT over one TCP connection
  reached 26 Mbps on the same link. The cause isn't isolated yet.
- **Channel B's sliced messages are loss-tolerant, not reliable.** It
  delivered 1 MB messages at 1 per second with all of the data accounted for
  in the scoring window, but only half of them arrived *complete*. A 1 MB
  message is 964 datagrams, and Wi-Fi drops the odd one. That is what slices
  are for (a lidar frame missing a few points is still usable), but whole
  objects such as maps need doc 13's reliable bulk-object streams, which
  are not built yet.

## Setup

| | |
| --- | --- |
| Robot | Raspberry Pi CM4 (XGO-Lite V2), Wi-Fi 2.4 GHz channel 6, PHY rate 65 Mbps, signal −33 dBm, power management off |
| Operator | Laptop, Wi-Fi 2.4 GHz channel 6, PHY rate 130 Mbps, signal −40 dBm. Power save **on** for the throughput runs and the first latency run, **off** for the latency rerun. |
| Path | Both on the same access point; idle ping 2.5 ms minimum |
| Channel B | `robot-edge --bench` (stub bridge) and `operator-console --bench`: QUIC + mTLS. Release builds. |
| Others | `tools/proto-bench` native clients: Zenoh (TLS, mTLS, peer to peer, robot listening), MQTT (TLS, mTLS, mosquitto 2 on the laptop), WebRTC (webrtc-rs data channel, DTLS), raw UDP |
| Certificates | A fresh dev set from `dev-certs --robot-san 192.168.2.19 --robot-san 192.168.2.24`, so every TLS client still checks the server's name |

**Method.** This is `benchmark/run_wifi.py`, the two-host counterpart of
`run_loopback.py`. It starts the robot's half of each run over ssh.

- **Latency:** 2,000 ping-pongs of 64 B, laptop-initiated, 3 runs per
  protocol; the run with the middle median is shown.
- **Throughput:** for each protocol and size, the sender steps up its rate
  until the receiver gets less than 98% of what was offered in an 8 s
  window. Each failing step is retried once. The table shows the highest
  rate that passed.

Three differences from doc 11:

1. **The pass bar is 98%, not 99%.** Each receiver counts over a fixed 8 s
   of wall-clock time, and Wi-Fi delay swings move that count by about ±1%
   even when nothing is lost. Reliable protocols at a small fraction of the
   link scored 98.9–101.1% in a check run.
2. **Receivers count after a warm-up.** Zenoh's publisher only sends to
   subscribers it has heard about, and over Wi-Fi that takes a while: without
   a warm-up, Zenoh "lost" a quarter of its 256 KB messages at 1 per second.
   The Zenoh and MQTT receivers now skip the first 1.5 s; WebRTC and Channel
   B already did.
3. **Sliced Channel B is scored on data delivered.** Every slice that arrives
   counts, whether or not its message completed, and the share of complete
   messages is reported next to it. Sizes up to 1,024 B are a single datagram
   and are scored like every other protocol.

## Latency (64 B ping-pong)

With the laptop's Wi-Fi power saving **off** (`iw dev wlp0s20f3 set
power_save off`; the robot's was already off):

| Protocol | Median | p95 | p99 | Medians of the 3 runs |
| --- | --- | --- | --- | --- |
| Raw UDP (no encryption) | 1.56 ms | 3.9 ms | 9.4 ms | 1.55, 1.56, 1.64 |
| **Channel B, transport only** | **1.96 ms** | **4.7 ms** | 16.9 ms | 1.94, 1.96, 2.00 |
| **Channel B, full frame** | **2.16 ms** | **5.8 ms** | 7.8 ms | 2.23, 2.16, 1.96 |
| WebRTC (DTLS) | 2.57 ms | 5.4 ms | 22.8 ms | 2.60, 2.57, 2.52 |
| Zenoh (mTLS) | 8.36 ms | 13.4 ms | 99.5 ms | 8.33, 8.36, 8.57 |
| MQTT (mTLS) | 8.78 ms | 47.3 ms | 131.8 ms | 8.78, 8.83, 8.68 |

- **Channel B adds 0.4–0.6 ms to raw UDP** for QUIC, mutual TLS and (full
  frame) FlatBuffers. On loopback, doc 11 measured 5.3 µs for the same
  overhead. The extra here hasn't been broken down; the CM4's slower CPU
  and each end's event loop are the candidates.
- **The TCP-based protocols are 4× slower at the median,** 8.4–8.8 ms
  against 2.0–2.2 ms. Doc 11 saw the same ordering on loopback, where the
  gap was tens of microseconds. Over Wi-Fi it is several milliseconds, which
  fits TCP's acknowledgement timing adding a radio turnaround or two to each
  exchange, but that hasn't been isolated.
- **ICMP ping, one per second, still averaged 20 ms** (1.6 ms minimum,
  115 ms maximum). An idle link still pays a wake-up cost somewhere,
  possibly in the robot's Wi-Fi firmware or the access point; back-to-back
  traffic, as in these ping-pongs, doesn't.

With the laptop's power saving **on**, the default on this machine, the
same test gave:

| Protocol | Median | p95 | p99 |
| --- | --- | --- | --- |
| Raw UDP | 2.52 ms | 85.7 ms | 97.8 ms |
| Channel B, transport only | 3.77 ms | 93.7 ms | 184.3 ms |
| Channel B, full frame | 3.93 ms | 88.5 ms | 114.1 ms |
| WebRTC | 3.91 ms | 88.4 ms | 144.7 ms |
| Zenoh | 10.66 ms | 20.3 ms | 76.2 ms |
| MQTT | 10.82 ms | 25.9 ms | 116.3 ms |

A dozing radio receives buffered packets at the next beacon, about every
100 ms, which matches the ~90 ms tails. **Operator laptops should run with
Wi-Fi power saving off:** it costs every protocol 1–2 ms at the median and
turns Channel B's p95 from about 5 ms into about 90 ms.

## Throughput, robot to operator

Highest rate with at least 98% delivered: messages per second, and Mbps of
payload at that rate. A dash means the protocol cannot send that size (raw
UDP's maximum is 65,507 B, webrtc-rs's 65,535 B).

| Size | Channel B | Zenoh (mTLS) | MQTT (mTLS) | WebRTC | Raw UDP |
| --- | --- | --- | --- | --- | --- |
| 16 B | 20,000 · 2.6 | **50,000 · 6.4** | 41,260 · 5.3 ᵃ | 10,000 · 1.3 | < 10,000 ᵇ |
| 1 KB | 1,000 · 8.2 | 500 · 4.1 | **2,000 · 16.4** | 500 · 4.1 ᵃ | 1,000 · 8.2 |
| 4 KB | 200 · 6.6 ᶜ | 200 · 6.6 | **500 · 16.4** | 200 · 6.6 ᵃ | 200 · 6.6 |
| 16 KB | 20 · 2.6 ᶜ | 50 · 6.6 | **200 · 26.2** | 20 · 2.6 ᵃ | 50 · 6.6 |
| 64 KB | 10 · 5.2 ᶜ | 2 · 1.0 | **20 · 10.5** | – | – |
| 256 KB | 5 · 10.5 ᶜ | 5 · 10.5 | 5 · 10.5 | – | – |
| 1 MB | 1 · 8.4 ᶜ ᵈ | fails at 1 ᵉ | 1 · 8.4 | – | – |

ᵃ The sender couldn't keep up with the next step. MQTT's 16 B client sent
at most ~41,000 msg/s. webrtc-rs sent at most 422 msg/s at 1 KB, 270 at 4 KB
and 44 at 16 KB, delivering all of it.
ᵇ The search starts at 10,000 msg/s for 16 B, and raw UDP delivered only
2,769 of them per second.
ᶜ Sliced: scored on data delivered. Complete messages at the passing rate:
100% at 4 KB, 99.4% at 16 KB, 100% at 64 KB and 256 KB.
ᵈ Only 50% of 1 MB messages arrived complete; the rest each missed a few of
their 964 slices.
ᵉ Failed at 1 msg/s on both tries (none arrived on the second). Zenoh's publisher defaults to dropping
messages it can't queue (`CongestionControl::Drop`); with `Block` it would
hold them instead. This was not tested.

### What the throughput numbers say

- **The link runs out of packets before it runs out of bytes.** Raw UDP,
  one packet per message, peaked at about 1,000–1,500 packets per second
  (about 12 Mbps of 1 KB datagrams) before the receiver fell behind. At
  16 B it couldn't sustain 10,000. Channel B reached 20,000 msg/s at 16 B
  because QUIC packs many small datagrams into one packet, and Zenoh and
  MQTT batch similarly. Doc 11's planned batched sends (`sendmmsg`/GSO)
  matter more on this hardware than on loopback.
- **TCP went further on large messages.** MQTT, on one TCP connection,
  passed at 26 Mbps with 16 KB messages. Every datagram-based protocol,
  Channel B included, stayed at or below about 12 Mbps. Possible causes,
  none tested yet: the CM4's Wi-Fi driver aggregating TCP better than UDP,
  TCP offloads (GSO/TSO) that datagram sends don't get, and Channel B's
  CUBIC window over a jittery link. This is the next thing to investigate.
- **Channel B's results are uneven between neighbouring sizes.** 16 KB
  passed only at 20 msg/s (2.6 Mbps) while 4 KB passed 6.6 and 64 KB 5.2.
  Each rate was tried once (twice on failure) on a shared Wi-Fi channel; a
  single bad stretch fails a step. Read single rows with that in mind.
- **Reliable versus loss-tolerant.** MQTT and Zenoh retransmit, so a
  passing step means every message arrived whole. Channel B's sliced
  messages never retransmit; a passing step means at least 98% of the data
  arrived. These are different guarantees for different data.

## Caveats

- **The throughput runs had the laptop's Wi-Fi power saving on.** A
  continuous stream keeps the radio awake, so they should be much less
  exposed than ping-pong was, but they haven't been rerun to check.
- **The access point and channel were shared** with other networks and
  devices nearby, with no control over their traffic. Earlier the same day,
  a power cut took the router down, and the laptop twice fell off the
  network; no drop happened during these runs.
- **The MQTT broker runs on the laptop,** so MQTT's path has one Wi-Fi hop,
  the same as every other protocol's.
- **Ratios above 1.0** appear in the raw results (up to 1.25). A queue
  somewhere on the path (the broker, the Wi-Fi driver, the access point)
  was draining messages sent before the window opened. The rate above such
  a step always failed.
- **Three runs feed this report.** Throughput for Channel B, WebRTC and raw
  UDP, and the power-save-on latency, come from the first run (commit
  `0a0ad8d`). Zenoh and MQTT throughput was rerun with the receiver warm-up
  and a 2 MiB MQTT packet limit (commit `76f5752`); their first-run numbers
  were artifacts of the missing warm-up and are not used. The power-save-off
  latency is a third run, on the same code.

## Reproducing

On the robot: release builds of `robot-edge` and `proto-bench` (add
`tools/proto-bench` to the Pi's workspace), and the certificate set plus
`zenoh_tls_robot.json5` under `~/RoboProtocol/benchmark-wifi/`. On the
laptop: release builds of `operator-console` and `proto-bench`, and a
mosquitto 2 container on ports 1884/8884 using the same certificate set.
Then:

```bash
python3 benchmark/run_wifi.py            # everything, about 70 minutes
SKIP_LATENCY=1 python3 benchmark/run_wifi.py rs-zenoh-tls rs-mqtt-tls   # throughput only
```

`run_wifi.py` stops any running `robot-edge` on the robot, and pauses for
up to 10 minutes if the robot stops answering ping. Results go to
`benchmark/results/wifi/<timestamp>/` (not committed).

## Next steps

1. **Find the datagram ceiling.** Raw UDP from the CM4 stops near
   1,000–1,500 packets per second, while TCP reaches twice the throughput.
   Look at the Wi-Fi driver's handling of UDP, and at UDP GSO.
2. **Batched sends for Channel B** (`sendmmsg`/GSO), as doc 11 planned.
3. **Bulk objects** ([13](13-large-sensor-payloads.md)), for large messages
   that must arrive whole.
4. **Control under load** ([12](12-control-under-load-benchmark.md)), on
   this same setup.
