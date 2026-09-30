# Protocol comparison: MQTT, Zenoh, WebRTC and raw UDP (loopback)

This report compares the latency and throughput of three pub/sub transports
often used for robot teleoperation and telemetry, plus a raw UDP floor. Each
is measured with encryption off and on where the protocol allows it. It is
Part 4 of the RoboProtocol benchmark runbook ([`BENCHMARK.md`](../BENCHMARK.md)).
The method follows Zenoh's 2023 comparison
([Zenoh vs MQTT vs Kafka vs DDS](https://zenoh.io/blog/2023-03-21-zenoh-vs-mqtt-kafka-dds/))
and adds two things: an encrypted variant of every protocol, and WebRTC as
used by DimOS's hosted teleop (dimTELE).

> **Status: partial.** RoboProtocol's own teleop channel (Channel B,
> QUIC + mTLS) is **not in these tables yet**. Channel B streams in one
> direction and has no echo mode, so it can't answer a ping-pong latency test.
> Its row will be added once `robot-edge` gets one (see
> [Next steps](#next-steps)). Everything below is the reference field
> Channel B will be measured against.

## Key findings

- **Zenoh had the lowest latency, with or without TLS.** Its median round
  trip was 57.6 µs in plaintext and 66.2 µs with mutual TLS. That is about
  40% less than MQTT (93.0 µs and 114.0 µs) and a third of WebRTC
  (172.4 µs). Zenoh also had the tightest tail: p99 was 108 µs with mTLS,
  against 244 µs for MQTT and 368 µs for WebRTC.
- **TLS cost little on loopback, except MQTT's throughput on large
  messages.** It added about 9 µs round trip for Zenoh and about 21 µs for
  MQTT. Zenoh's throughput with mTLS matched plaintext at every payload
  size. MQTT's matched up to 8 KB, then halved to 2.6 Gbps from 16 KB up.
  The gap between protocols is much bigger than what encryption costs
  within any one of them.
- **Zenoh sustained 4–10× MQTT's throughput.** Zenoh reached 26 Gbps at
  16–64 KB messages, and at least 200k msg/s up to 16 KB. MQTT (QoS 0
  through mosquitto) topped out around 5–6.6 Gbps and 50k msg/s. Above
  that point MQTT doesn't level off: it collapses. At 8 KB and 100k msg/s
  offered, 0.1% of messages arrived.
- **WebRTC, using DimOS's stack, capped out at about 180–190 Mbps** whatever
  the message size. That is two orders of magnitude below Zenoh on the same
  machine. The limit comes from `aiortc`, the pure-Python WebRTC stack that
  dimTELE runs on the robot, rather than from WebRTC as such. A libwebrtc
  client would do far better.

## What was measured

| Protocol | Variant | Encryption | Topology |
|---|---|---|---|
| MQTT 3.1.1 | QoS 0 | none | client → mosquitto broker → client |
| MQTT 3.1.1 | QoS 0 | TLS 1.3 + client certs (mTLS) | client → mosquitto broker → client |
| Zenoh 1.x | peer to peer | none | direct peer link |
| Zenoh 1.x | peer to peer | TLS + mTLS | direct peer link |
| WebRTC | DimOS `WebRTCPubSub` over aiortc DataChannels | DTLS (always on) | direct peer connection, see [below](#webrtc-dimtele) |
| Raw UDP | none | none | direct datagrams; the no-protocol floor |
| RoboProtocol Channel B | QUIC datagrams | TLS 1.3 + mTLS (always on) | *pending* |

The results fall into two groups:

- **Unencrypted:** MQTT, Zenoh and raw UDP. This matches the setup of the
  original Zenoh post.
- **Encrypted (the fair comparison for RoboProtocol):** MQTT + mTLS,
  Zenoh + mTLS, WebRTC, and Channel B. Channel B's encryption can't be
  turned off, so only encrypted rows compare like with like.

### WebRTC (dimTELE)

dimTELE, DimOS's hosted teleop, sends operator commands over WebRTC
DataChannels and robot video as a WebRTC media track. Every connection path
DimOS ships goes through Cloudflare's Realtime SFU (a WebRTC relay server):

- **Production:** the robot dials out to Dimensional's teleop broker, which
  relays through Cloudflare.
- **DimOS's own benchmark mode:** loops messages through a Cloudflare edge
  server.

Neither path can run on loopback. So these numbers were measured differently:

- The benchmark drives **DimOS's own `WebRTCPubSub`**, from DimOS commit
  `edd7c34`, with no changes.
- It runs over a small **peer-to-peer provider** written for this benchmark
  ([`benchmark/webrtc_bench.py`](../benchmark/webrtc_bench.py)). It
  implements DimOS's `Provider` interface and sets up channels the same way
  DimOS's Cloudflare provider does, but uses a direct peer connection with a
  local SDP exchange in place of Cloudflare's signaling and relay.
- **Channel settings match dimTELE's.** The latency test uses an unordered
  channel with `maxRetransmits=0`, like its `cmd_unreliable` command channel.
  The throughput test uses a reliable, ordered channel, like
  `state_reliable`.

These results are **dimTELE's WebRTC stack without Cloudflare**. A real
dimTELE session adds an internet round trip to the nearest Cloudflare edge
on top of everything measured here. DimOS's docs also note that Cloudflare
silently drops DataChannel messages above about 64 KB.

## Setup

| | |
|---|---|
| Machine | Intel Core i5-1240P (12 cores / 16 threads, up to 4.4 GHz), 16 GB RAM |
| OS | Ubuntu 20.04.6 LTS, kernel 5.15.0-139-generic, CPU governor `powersave` (not pinned) |
| Topology | Single host, loopback (`127.0.0.1`) |
| Harness | Python 3.11, one process per endpoint |
| MQTT | `paho-mqtt` 2.1.0 client; mosquitto 2.1.2 in Docker (`--network host`) |
| Zenoh | `eclipse-zenoh` 1.10.1 (Python bindings), peer mode, multicast scouting off for TLS runs |
| WebRTC | `aiortc` 1.15.0, DimOS `WebRTCPubSub` at commit `edd7c34` |
| Reference ceiling | `iperf3` TCP over loopback: **78.5 Gbps** |
| Repo | RoboProtocol commit `0fb2612` (`discussion-6`) |

**How TLS was set up for MQTT and Zenoh:**

- **Certificates:** both use the same dev CA and ECDSA P-256 certificates as
  Channel B ([`certs/`](../certs)). The robot cert is used on the listening
  side and the operator cert on the connecting side.
- **mosquitto:** a TLS-only listener that accepts TLS 1.3 only and requires a
  client certificate. Sessions negotiated `TLS_AES_256_GCM_SHA384`. Channel
  B's QUIC stack is `quiche` 0.22, which uses BoringSSL for TLS. Its
  negotiated cipher suite will be recorded when Channel B is measured.
- **Zenoh:** runs with `enable_mtls` and `verify_name_on_connect` on.
- **mTLS is enforced, not just offered.** For both protocols, a client
  without a certificate got no service. For Zenoh, a plain-TCP client also
  got no service.

## Method

### Latency

- **Test:** ping-pong round trips with a fixed 64-byte payload (ICMP-sized,
  as in the Zenoh post), 2,000 round trips per run.
- **Reporting:** the fastest and slowest 1% are trimmed, then the median,
  p95 and p99 RTT are reported. One-way latency is taken as half the median
  RTT.
- **Delivery mode:** best effort wherever the protocol offers it (MQTT QoS 0,
  unreliable WebRTC channel), so retransmission doesn't show up in the
  latency.

### Throughput

- **Test:** one-way, at payload sizes of 16 B, 2, 4, 8, 16, 32 and 64 KB.
- **Search:** the sender is paced at a fixed rate that steps up through
  1k, 2k, 5k, 10k, 20k, 50k, 100k, 200k and 500k msg/s, then unpaced.
- **What each cell reports:** the highest rate at which the receiver got at
  least 99% of what was sent.
- **Timing:** each trial sends for 10 s. The receiver counts over an 8 s
  window that sits inside the send period, so start-up and shutdown are
  excluded.
- **Retries:** a trial that falls below 99% is re-run once before the search
  stops, because a single miss is often just connection timing.

We use a rate search, not the "flood and count" approach, because flooding
measures queue overflow, not the protocol. With an unpaced QoS 0 publisher,
mosquitto drops messages for a subscriber whose queue is full. A flood test
reported MQTT at 324 msg/s, which is meaningless.

### How this differs from the Zenoh post

- **Payload cap of 64 KB, not 512 MB.** This benchmark is about teleop and
  telemetry links. Raw UDP can't send a datagram larger than 65,507 B, which
  is the size used in its 64 KB row.
- **Smallest payload is 16 B, not 8 B.** Every message carries a 16-byte
  sequence number and timestamp header.
- **Python clients, not compiled ones.** Absolute throughput is lower than a
  C or Rust client would reach, and the harness itself caps some cells
  (marked †). The Zenoh post, on a fixed-clock Ryzen 7 5800X, reported
  67 Gbps for Zenoh peer to peer and about 9 Gbps for MQTT on one machine.
  Our numbers are 26 Gbps and 6.6 Gbps. The ranking is the same.

## Results

### Latency (64 B ping-pong, 2,000 round trips)

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="img/protocol-comparison/latency-dark.png">
  <img alt="Horizontal bar chart of median round-trip latency with p99 whiskers: Zenoh 58 µs plaintext and 66 µs mTLS, MQTT 93 µs plaintext and 114 µs mTLS, WebRTC 172 µs with DTLS." src="img/protocol-comparison/latency-light.png">
</picture>

| Protocol | Encryption | Median RTT | One-way | p95 | p99 |
|---|---|---|---|---|---|
| Zenoh (P2P) | none | 57.6 µs | 28.8 µs | 78.5 µs | 107.8 µs |
| MQTT (mosquitto) | none | 93.0 µs | 46.5 µs | 213.8 µs | 603.8 µs |
| Zenoh (P2P) | mTLS | 66.2 µs | 33.1 µs | 85.3 µs | 108.0 µs |
| MQTT (mosquitto) | TLS 1.3 + mTLS | 114.0 µs | 57.0 µs | 173.5 µs | 244.3 µs |
| WebRTC (DimOS stack) | DTLS | 172.4 µs | 86.2 µs | 261.8 µs | 368.3 µs |
| RoboProtocol Channel B | TLS 1.3 + mTLS | *pending* | | | |

Reference: `ping` over loopback had a minimum RTT of 26 µs. Its average was
70–90 µs, but that is inflated because `ping` sends one packet per second
and the CPU drops into idle states between packets. Use the minimum, not
the average, as the floor.

MQTT's plaintext p99 (604 µs) is higher than its TLS p99 (244 µs). The two
runs were a day apart on a laptop whose CPU clock wasn't fixed (governor
`powersave`), so results vary between runs. Across three runs, Zenoh's
plaintext median stayed between 52 and 58 µs. MQTT's varied more: 87, 93
and 142 µs. The table uses one run per protocol variant. Treat MQTT's
tail percentiles as rough, and read the protocol ranking, which held in
every run, rather than exact values.

### Throughput (highest rate with ≥99% delivered)

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="img/protocol-comparison/throughput-dark.png">
  <img alt="Two log-scale line charts of sustained throughput against payload size, unencrypted and encrypted. Zenoh tracks raw UDP up to 26 Gbps in both; MQTT peaks at 6.6 Gbps plaintext and 2.6 Gbps with mTLS; WebRTC stays near 100 Mbps." src="img/protocol-comparison/throughput-light.png">
</picture>

The dashed grey line in both panels is plaintext raw UDP from the Python sender, shown as a reference. The dotted line is loopback TCP measured with `iperf3`. The tables below have the exact values.

### Throughput, unencrypted

| Payload | MQTT (QoS 0) | Zenoh (P2P) | Raw UDP |
|---|---|---|---|
| 16 B | 50k msg/s, 6 Mbps | 500k msg/s, 64 Mbps | 462k msg/s, 59 Mbps † |
| 2 KB | 50k msg/s, 818 Mbps | 496k msg/s, 8.1 Gbps | 411k msg/s, 6.7 Gbps † |
| 4 KB | 50k msg/s, 1.6 Gbps | 200k msg/s, 6.6 Gbps | 389k msg/s, 12.7 Gbps † |
| 8 KB | 50k msg/s, 3.3 Gbps | 200k msg/s, 13.1 Gbps | 322k msg/s, 21.1 Gbps † |
| 16 KB | 50k msg/s, 6.6 Gbps | 200k msg/s, 26.2 Gbps | 283k msg/s, 37.1 Gbps † |
| 32 KB | 20k msg/s, 5.2 Gbps | 100k msg/s, 26.2 Gbps | 199k msg/s, 52.1 Gbps |
| 64 KB | 10k msg/s, 5.2 Gbps | 50k msg/s, 26.2 Gbps | 99k msg/s, 52.1 Gbps |

† The Python sender couldn't reach the next rate step, but everything it
sent arrived. These cells show the harness's limit, not UDP's.

Each cell is the highest *step* that passed, so it is a lower bound. The
real maximum is somewhere below the next step. That's why Zenoh reads
496k msg/s at 2 KB but 200k at 4 KB: 4 KB failed the 500k step and
passed 200k.

What happens when MQTT is pushed past its maximum rate:

<picture>
  <source media="(prefers-color-scheme: dark)" srcset="img/protocol-comparison/overload-dark.png">
  <img alt="Line chart of delivered percentage against offered rate at 8 KB: both at 100% up to 50k msg/s; MQTT then drops to 0.1% at 100k msg/s while Zenoh holds 100% to 200k and falls to 73% at 500k." src="img/protocol-comparison/overload-light.png">
</picture>

| Payload | Offered | Delivered |
|---|---|---|
| 2 KB | 100k msg/s | 26% |
| 4 KB | 100k msg/s | 8% |
| 8 KB | 100k msg/s | 0.1% |

### Throughput, encrypted

| Payload | MQTT + mTLS | Zenoh + mTLS | Channel B |
|---|---|---|---|
| 16 B | 50k msg/s, 6 Mbps | 500k msg/s, 64 Mbps | *pending* |
| 2 KB | 50k msg/s, 815 Mbps | 497k msg/s, 8.1 Gbps | *pending* |
| 4 KB | 50k msg/s, 1.6 Gbps | 200k msg/s, 6.6 Gbps | *pending* |
| 8 KB | 50k msg/s, 3.3 Gbps | 200k msg/s, 13.1 Gbps | *pending* |
| 16 KB | 20k msg/s, 2.6 Gbps | 200k msg/s, 26.2 Gbps | *pending* |
| 32 KB | 10k msg/s, 2.6 Gbps | 100k msg/s, 26.2 Gbps | *pending* |
| 64 KB | 5k msg/s, 2.6 Gbps | 50k msg/s, 26.2 Gbps | *pending* |

With mTLS, Zenoh passed the same rate step as in plaintext at every
payload size. Any cost from TLS is smaller than one rate step. MQTT with
mTLS matched plaintext up to 8 KB, then dropped to half from 16 KB up
(2.6 Gbps, against 5.2–6.6 Gbps in plaintext). At large messages the
broker's TLS work appears to become MQTT's limit.

WebRTC (DimOS stack, reliable ordered channel):

| Payload | Highest rate with ≥99% delivered | Peak goodput (any delivery ratio) |
|---|---|---|
| 16 B | 10k msg/s, 1 Mbps | 1 Mbps |
| 2 KB | 5k msg/s, 82 Mbps | 124 Mbps (76% delivered) |
| 4 KB | 2k msg/s, 66 Mbps | 142 Mbps (87%) |
| 8 KB | 2k msg/s, 131 Mbps | 188 Mbps (57%) |
| 16 KB | 1k msg/s, 131 Mbps | 182 Mbps (69%) |
| 32 KB | below 1k msg/s | 184 Mbps (700 msg/s) |
| 64 KB | below 1k msg/s | 188 Mbps (358 msg/s) |

In this table, "delivered" means messages the receiver counted as a share
of what the application tried to send. The WebRTC sender stops queueing
once 16 MB is waiting unsent. Messages skipped because of that count as
not delivered, so the ratio reflects everything the application offered,
not just what reached the socket.

## Discussion

**Latency.** On one machine, protocol overhead is almost all CPU time per
message, so these numbers rank software stacks more than network paths:

- **Zenoh vs MQTT:** Zenoh's peer-to-peer link makes one hop. MQTT makes
  two, via the broker, plus the broker's own processing, and the gap
  between them roughly reflects that.
- **WebRTC:** a DataChannel message passes through SCTP framing and then
  DTLS encryption, both implemented in Python by aiortc. That accounts for
  its extra ~100 µs.

On a real network the link's own delay is added to every row. That makes
the relative gaps smaller, but the order stays the same.

**Encryption.** TLS adds a few microseconds per message on a modern CPU
with AES-NI (hardware AES support). Only at large messages through a
broker (MQTT at 16 KB and up) did encryption become the bottleneck: there,
every message is decrypted and re-encrypted once for each of the two
links. This is the main reason the encrypted
table matters for RoboProtocol: always-on encryption costs Channel B
little, so how it compares should come down to design (QUIC datagrams, no
broker, one handshake per session) rather than the crypto.

**Throughput shape.**

- **Zenoh and raw UDP** scale close to linearly with payload size until
  memory copying dominates. Zenoh levels off at 26 Gbps for 16 KB and
  larger messages, a third of loopback TCP's 78.5 Gbps.
- **MQTT** is stuck at 50k msg/s whatever the payload size up to 16 KB.
  That points to a per-message cost (the Python client and the broker's
  per-message dispatch) rather than bandwidth.
- **MQTT's collapse under overload is the result that matters most for
  teleop.** Past its maximum rate, a QoS 0 subscriber loses nearly
  everything instead of the excess. Any link that can be overloaded (video
  alongside commands, or a slow consumer) must be designed around that.

**WebRTC.** At about 185 Mbps, dimTELE's robot-side stack is fine for its
actual workload: commands, telemetry and a compressed video track, which
fit in a few Mbps. But it leaves little headroom for raw sensor data. The
cap is aiortc's Python SCTP and DTLS, so a native WebRTC stack (libwebrtc,
as in browsers) should reach much higher throughput on the same protocol.

## Limitations

- **Loopback only.** There's no real link, so no link delay, loss or
  reordering, and no bandwidth limit below the memory bus. These results
  rank software overhead. A two-machine wired LAN run is the next step
  for claims about network behaviour.
- **Python harness.** Every client in these tables is Python, including
  the raw UDP floor. That caps the highest-rate cells and adds per-message
  cost to every protocol. All protocols share the same cost, so the
  comparison between them is fair, but absolute numbers are lower than
  native clients would reach. Channel B's implementation is compiled Rust
  (`quiche`), so **its results must not be put next to these Python rows**
  as if they were like for like. It needs to be compared against native
  clients for the other protocols.
- **CPU clock not fixed.** The CPU governor was `powersave` on a laptop
  with a mix of performance and efficiency cores, and processes weren't
  pinned to cores. Tail latencies vary between runs; medians are stable.
- **Coarse rate steps.** Throughput cells are lower bounds, accurate to
  within one rate step (up to 2.5× apart).
- **WebRTC without Cloudflare.** The WebRTC rows exclude the Cloudflare
  relay that every real dimTELE session goes through.
- **DDS and Kafka not tested.** Kafka is built for durable logs, not
  control loops. DDS would matter for a ROS 2 comparison, which isn't the
  goal here.

## Reproducing

Everything runs from `benchmark/`. Commands assume a Python 3.11 venv at
`benchmark/.venv`.

```bash
# client libraries
uv venv -p 3.11 benchmark/.venv
uv pip install -p benchmark/.venv/bin/python paho-mqtt "eclipse-zenoh>=1,<2" \
    "aiortc>=1.14.0" "aiohttp>=3.9.0" pydantic "structlog>=25.5.0,<26" matplotlib
# DimOS WebRTC pubsub only (the heavy base deps aren't needed)
git clone https://github.com/dimensionalOS/dimos && git -C dimos checkout edd7c34
DIMOS_ALLOW_MISSING_COCKPIT=1 uv pip install -p benchmark/.venv/bin/python --no-deps ./dimos

# broker: plaintext on 1883, mTLS (TLS 1.3, client cert required) on 8883
docker run -d --rm --name bench-mosquitto --network host \
    -v $PWD/benchmark/tls/mosquitto.conf:/mosquitto/config/mosquitto.conf:ro \
    -v $PWD/certs:/certs:ro eclipse-mosquitto:2

# unencrypted table, then encrypted, then WebRTC; each writes results/<timestamp>/
cd benchmark
.venv/bin/python run_loopback.py mqtt zenoh udp
.venv/bin/python run_loopback.py mqtt-tls zenoh-tls
.venv/bin/python run_loopback.py webrtc
.venv/bin/python summarize.py results/<run-dir> [results/<run-dir> ...]
# charts in docs/img/protocol-comparison/ (light + dark)
.venv/bin/python plot_results.py --plain results/<run> --tls results/<run> --webrtc results/<run>
```

Each run directory has `results.json` (every trial, not just the best
ones) and `run.log`. The per-protocol scripts (`mqtt_bench.py`,
`zenoh_bench.py`, `webrtc_bench.py`, `raw_udp_baseline.py`) can also be run
by hand; each one's `--help` shows its modes.

## Next steps

1. **Channel B.** Add an echo mode to `robot-edge`: it replies to a tagged
   `TeleopCommand` with a `TelemetryData` carrying the same sequence number
   and timestamp. Then run Channel B through the same latency and rate
   search and add it to the encrypted tables.
2. **Two-machine wired LAN.** Rerun everything with a real link between
   two machines.
3. **Large payloads.** An appendix at 256 KB to 16 MB, sized for camera
   frames and point clouds, for MQTT and Zenoh only. Raw UDP and
   Cloudflare-relayed WebRTC can't carry these sizes.
4. **Fixed CPU clock.** Pin the governor to `performance`, and pin each
   process to a core, so tail latencies repeat between runs.
