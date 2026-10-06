# Protocol comparison over Wi-Fi: robot CM4 to operator laptop

This report repeats [11 — Protocol comparison](11-protocol-comparison.md)
on real hardware: the XGO-Lite's Raspberry Pi CM4 and an operator laptop,
talking over Wi-Fi through the same access point. It compares RoboProtocol's
Channel B with Zenoh, MQTT and WebRTC, all with mutual TLS, plus raw UDP as
the floor.

## TL;DR

- **Control messages: Channel B is the fastest encrypted option.** Its
  median round trip over Wi-Fi is 2.0–2.2 ms, about 0.5 ms above
  unencrypted raw UDP and about 4× faster than Zenoh or MQTT (8.4–8.8 ms).
- **Turn off the operator laptop's Wi-Fi power saving.** With it on, every
  UDP-based protocol's p95 jumps from about 5 ms to about 90 ms.
- **Many small messages: batching wins.** At 16 B, Zenoh sustained 50,000
  msg/s and Channel B 20,000, while raw UDP couldn't sustain 10,000.
- **Bulk data: the CM4 sends about 2,000 packets per second, whatever
  their size.** So bigger packets mean more throughput: raw UDP with
  full-size 1,400-byte packets reached 22 Mbps, the same as TCP (21–26).
  Channel B, with 1,100-byte slices and QUIC's overhead, stopped at about
  8–12 Mbps. A full-rate lidar or depth stream doesn't fit until that gap
  closes. See [Where the datagram ceiling is](#where-the-datagram-ceiling-is).
- **Large messages as slices deliver data, not whole messages.** At 1 MB,
  all of the data arrived but only half the messages complete. That suits
  point clouds; anything that must arrive whole needs reliable streams.

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
  WebRTC) topped out at about 8–12 Mbps in these runs; MQTT over one TCP
  connection reached 26 Mbps. A follow-up found the CM4 sends about 2,000
  packets per second at any size, so full-size 1,400-byte UDP packets
  reach about 22 Mbps, like TCP; the datagram protocols were sending
  smaller packets ([Where the datagram ceiling is](#where-the-datagram-ceiling-is)).
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
  MQTT batch similarly.
- **TCP went further on large messages.** MQTT, on one TCP connection,
  passed at 26 Mbps with 16 KB messages. Every datagram-based protocol,
  Channel B included, stayed at or below about 12 Mbps here. A follow-up
  test found why: [Where the datagram ceiling is](#where-the-datagram-ceiling-is).
- **Channel B's results are uneven between neighbouring sizes.** 16 KB
  passed only at 20 msg/s (2.6 Mbps) while 4 KB passed 6.6 and 64 KB 5.2.
  Each rate was tried once (twice on failure) on a shared Wi-Fi channel; a
  single bad stretch fails a step. Read single rows with that in mind.
- **Reliable versus loss-tolerant.** MQTT and Zenoh retransmit, so a
  passing step means every message arrived whole. Channel B's sliced
  messages never retransmit; a passing step means at least 98% of the data
  arrived. These are different guarantees for different data.

## Where the datagram ceiling is

A follow-up test (`benchmark/udp_ceiling.py`, 2026-10-06) swept raw UDP at
five packet sizes in both directions, finding the highest rate with at
least 98% delivered. It counts everything received against everything
sent, so it doesn't need the 8 s window or the 98% allowance for window
edges above. For each robot-to-operator trial it also read the robot's
interface and UDP counters.

| Packet | Robot → operator | Operator → robot |
| --- | --- | --- |
| 64 B | 2,500 pkt/s · 1.3 Mbps | 8,000 pkt/s · 4.1 Mbps |
| 256 B | 1,500 pkt/s · 3.1 Mbps | 2,000 pkt/s · 4.1 Mbps |
| 512 B | 2,000 pkt/s · 8.2 Mbps | 3,000 pkt/s · 12.2 Mbps |
| 1,024 B | 1,500 pkt/s · 12.3 Mbps | 1,000 pkt/s · 8.2 Mbps |
| 1,400 B | **2,000 pkt/s · 22.4 Mbps** | 2,354 pkt/s · 26.4 Mbps (laptop's sending limit) |
| TCP, one bulk stream | **20.9–25.7 Mbps** (3 runs) | – |

- **The ceiling is packets, not bytes.** The robot sent about 1,500–2,500
  packets per second at every size, so its throughput grows with packet
  size, up to 22 Mbps at 1,400 B.
- **Full-size UDP packets match TCP.** 22.4 Mbps against 21–26. TCP did not
  get more out of the radio; it always sends full-size packets, and the
  datagram protocols above mostly didn't.
- **The robot drops nothing itself.** At every rate, including the ones
  that failed, every packet left through its Wi-Fi interface: no interface
  drops, no UDP send-buffer errors. Loss happens after it: in the air, at
  the access point, or at the laptop.
- **Small packets are limited on the robot's sending side.** The laptop
  sent the robot 8,000 packets of 64 B per second cleanly; the robot
  managed 2,500 the other way. One candidate is per-packet cost in the
  CM4's Wi-Fi interface (the Wi-Fi chip sits on an SDIO bus), but this
  hasn't been checked.
- **The operator-to-robot column is noisy at middle sizes** (1,024 B
  failed at 1,500 pkt/s while 1,400 B passed 2,354); each rate was run once,
  twice on failure. Read it for the 64 B and 1,400 B rows only.

**What this means for Channel B.** Its sliced payloads are 1,100 bytes, and
QUIC adds its own header and the laptop's acknowledgement packets, which
share the same half-duplex airtime. At ~2,000 packets/s, 1,100-byte slices
cap it near 17 Mbps before any of that overhead, and it measured 8–12. So:

- **Full-size slices** (about 1,350 bytes of payload, filling the 1,452-byte
  datagram limit) are worth about 20% more per packet.
- **QUIC's acknowledgement traffic** is the next suspect for the rest of the
  gap.
- **Batched sends (`sendmmsg`/GSO) are unlikely to help here.** They save
  system calls and CPU, but they put the same number of packets on the air,
  and the robot isn't dropping anything locally.

### Full-size slices on the CM4

`robot-edge` now fills each slice as far as the connection allows: 21
elements of 64 bytes (1,344 bytes) per slice on this link, against 17
(1,088 bytes) before. `--slice-payload-bytes` caps it for paths with a
smaller MTU. `benchmark/slice_size_ab.py` (2026-10-06) compared the two over
Wi-Fi, alternating them at each size, two rounds, laptop power saving off.
Highest passing data rate:

| Size | 1,100-byte slices, round 1 / 2 | Full-size slices, round 1 / 2 |
| --- | --- | --- |
| 16 KB | 6.6 / 6.6 Mbps | 6.6 / 6.6 Mbps |
| 64 KB | 10.5 / 5.2 Mbps | 5.2 / 5.2 Mbps |
| 256 KB | 4.2 / 10.5 Mbps | **10.5 / 21.0 Mbps** |

- **At 256 KB, full-size slices passed one rate step higher in both rounds,**
  reaching 21 Mbps with every message complete: twice doc 14's earlier
  best, and level with full-size raw UDP and TCP on this link. They needed
  12–19% fewer packets for the same data.
- **At 16 and 64 KB there's no consistent difference;** the two rounds
  differ more than the two variants. At those sizes a different limit
  appears to apply: frames come every 10–50 ms, and the robot's
  latest-wins queue cuts any frame still unsent when the next one arrives.
  The link stalled longer than that about one ping in ten (p90 72 ms that
  day), whatever the packet size. At 256 KB × 5–10 per second each frame has
  100–200 ms, so bandwidth, and so packet count, is what limits it. This
  reading fits the data but hasn't been tested directly.
- **Acknowledgements cost 10–20% extra packets,** counted as packets the
  operator sent per packet it received (heartbeats included). Over Wi-Fi
  that's well below the one-in-three seen on loopback, so ACK frequency is
  not a strong lever; quiche 0.22 doesn't support the QUIC ACK-frequency
  extension anyway.
- One 12-second network drop happened during round 2; the run paused and
  carried on.

### Cut or finish a frame the link can't keep up with?

When a new sensor frame arrives while the previous one is still partly
unsent, `robot-edge` can either **cut** the old frame (drop its unsent
slices; the default) or let it **finish** and keep only the newest frame
waiting behind it (`--sensor-frame-policy finish`). `benchmark/frame_policy_ab.py`
(2026-10-06) ran both at fixed rates near the limits found above,
alternating them over 4 rounds. "Delay" is a complete message's arrival
minus capture time, above the fastest in its run, so the clock offset
between robot and laptop cancels. Medians over the 4 rounds:

| Messages | Policy | Data delivered | Complete | Complete msgs/s | Delay p50 / p90 |
| --- | --- | --- | --- | --- | --- |
| 16 KB × 50/s | cut | 97.3% | 96.6% | 48.3 | 4.5 / 87 ms |
| | finish | 99.3% | 99.3% | 49.6 | 3.1 / 64 ms |
| 64 KB × 20/s | cut | 94.9% | 93.1% | 18.6 | 7.5 / 141 ms |
| | finish | 91.3% | 91.6% | 18.4 | 9.4 / 133 ms |
| 256 KB × 10/s | cut | 77.8% | 61.5% | 6.1 | 9.9 / 78 ms |
| | finish | 78.1% | **78.1%** | **7.8** | 18.2 / **330 ms** |

- **At 256 KB, finishing trades freshness for whole frames.** It delivered
  28% more complete frames per second from the same total data, but its
  p90 delay was 330 ms against 78 ms.
- **At 16 and 64 KB the policy barely matters;** the differences are within
  round-to-round variation. So at those sizes, cutting frames during link
  stalls is *not* the main reason frames arrive incomplete, contrary to the
  guess above. What's left is presumably packet loss in the air, which no
  queueing policy can recover.
- **Cut stays the default.** For teleoperation and navigation a frame's age
  matters more than its completeness. Finish suits slow, large frames where
  whole frames matter more, such as depth for 3D mapping, and belongs in
  per-sensor settings once the session handshake carries them.

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

**Raw results.** Every trial behind this report, with each run's log, is
attached to the
[`bench-wifi-2026-10-05`](https://github.com/spinworks-tech/robotele/releases/tag/bench-wifi-2026-10-05)
release (`wifi-bench-results-2026-10-05.tar.gz`). The access point's name
and local paths are replaced with placeholders. No certificates or
binaries are published.

**Rerunning.** The two hosts are set in `benchmark/run_wifi.py` (`PI`,
`PI_IP`, `OP_IP`).

1. **The robot:**
   - Add `"tools/proto-bench"` to the members in `~/RoboProtocol/Cargo.toml`.
   - Sync `crates/` and `tools/proto-bench/` to the robot.
   - Build both in release: `cargo build --release -p robot-edge -p proto-bench`.
     That takes about 20 minutes from scratch on a CM4.
2. **The laptop:** build in release with
   `cargo build --release -p operator-console -p proto-bench`.
3. **Set up, run, tear down:**

   ```bash
   benchmark/setup_wifi.sh up               # certs, configs, broker, robot-side files
   python3 benchmark/run_wifi.py            # everything, about 70 minutes
   SKIP_LATENCY=1 python3 benchmark/run_wifi.py rs-zenoh-tls rs-mqtt-tls   # throughput only
   benchmark/setup_wifi.sh down             # remove certs, configs and broker again
   ```

What `setup_wifi.sh up` creates:

- **A throwaway certificate set,** whose robot certificate also names both
  LAN IPs. Without that, the TLS clients couldn't check the server's name
  while connecting by IP. The set includes private keys, a CA key among
  them, so `down` deletes it. Never commit or publish it.
- **The robot's share of it,** copied to `~/RoboProtocol/benchmark-wifi/`:
  its own certificate and key, the CA certificate (not the CA key), and the
  Zenoh config.
- **Configs** for Zenoh and a mosquitto 2 broker (container
  `bench-mosquitto-wifi`, ports 1884/8884).

It never touches the default certificates in `certs/`, which `robot-edge`
and `operator-console` keep using outside the benchmark.

`run_wifi.py` stops any running `robot-edge` on the robot, so restart it
afterwards (or from the robot's LCD panel). It pauses for up to 10 minutes
whenever the robot stops answering ping. Results go to
`benchmark/results/wifi/<timestamp>/` (gitignored).

Turn the operator laptop's Wi-Fi power saving off first (`sudo iw dev
<interface> set power_save off`); see [Latency](#latency-64-b-ping-pong).

## Next steps

1. **Control under load** ([12](12-control-under-load-benchmark.md)), on
   this same setup: a paced 50 Hz control loop while camera and sensors
   fill the uplink.
2. **Per-sensor budgets** ([13](13-large-sensor-payloads.md)): on this
   robot a single full-rate lidar or depth stream exceeds the link.
3. **Bulk objects** ([13](13-large-sensor-payloads.md)), for large messages
   that must arrive whole.
4. **Batched sends** (`sendmmsg`/GSO), as doc 11 planned, now lower
   priority: they save CPU on the robot, not packets on the air.

5 GHz would relieve the 2.4 GHz airtime limit, but the XGO-Lite's Wi-Fi
doesn't support it (tried), so every result here stands for this robot.
