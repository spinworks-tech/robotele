# Benchmark plan: control latency under video load

**Status: plan, not yet run.** This document defines the benchmark; results
will be added to it once the testbed exists.

This follows [11 — Protocol comparison](11-protocol-comparison.md), which
measures each protocol on an idle link. Idle-link numbers don't answer the
question that matters most for a teleoperated robot:

> When a camera stream fills the robot's uplink, does a 50 Hz control
> command still arrive in time?

A protocol that shares one queue between video and control lets the video
stream decide how stale the control loop is. This benchmark measures how
much control latency each protocol loses to a competing video stream, on
real hardware, over a link that actually saturates.

## Hypotheses

- **H1.** A protocol with no priority between video and control degrades
  with video load. That includes MQTT on one TCP connection, and Channel B's
  shared datagram queue as it ships today (see below). Once video exceeds
  link capacity, control round trips grow to the depth of the bottleneck
  queue, which can be hundreds of milliseconds.
- **H2.** A protocol that prioritizes control keeps control round trips
  close to the idle-link value at any video load. Candidates are Zenoh with
  priority lanes, WebRTC with a separate congestion-controlled RTP track,
  and Channel B once the fix below is in.
- **H3.** Above link capacity, the protocols that behave well get there by
  dropping video (lower frame rate or quality), not by delaying control.

## Known gap in Channel B

The test has to be able to show Channel B failing, because as shipped it may
fail it:

- **The application layer already avoids its own backlog.**
  `robot-edge/src/video/channel_a.rs` coalesces video to the latest delta
  frame under backlog, while always sending SPS/PPS/IDR NAL units.
  `robot-edge` never builds up an unbounded video queue of its own.
- **The transport queue has no priority.** Once a video chunk is handed to
  quiche with `dgram_send`, it joins one first-in-first-out datagram queue,
  up to 4,096 datagrams deep (`enable_dgram(true, 4096, 4096)` in
  `quic_server.rs`). Channel B telemetry shares that queue with no priority
  over video. Only the E-Stop *stream* is prioritized.
- **The robot-to-operator direction is exposed.** On a 20 Mbps uplink,
  4,096 full video datagrams (~1.4 KB each, about 5.6 MB) take about
  2 seconds to drain, so telemetry and the benchmark echo can sit behind up
  to 2 s of video. Commands travel the other way; they are affected only on
  shared-airtime links such as Wi-Fi, or through delayed ACKs.

So the benchmark runs in two phases:

1. **Baseline:** Channel B exactly as it ships.
2. **With the fix:** `robot-edge` stops handing video to quiche once its
   datagram queue passes a small threshold (`dgram_send_queue_len()` /
   `dgram_send_queue_byte_size()`), and purges queued video
   (`dgram_purge_outgoing`) when a Channel B frame needs to go out. This is
   a protocol change and gets its own review.

Both phases get published. Showing only the "after" numbers would hide the
very issue this benchmark exists to catch.

## Testbed

```text
      "operator"                                     "robot"
  +---------------+        Ethernet switch       +---------------+
  |  Raspberry Pi | ---------[ unmanaged ]------ |  Raspberry Pi |
  |   (Pi B)      |                              |   (Pi A)      |
  +---------------+                              +---------------+
   operator-console / proto-bench clients          robot-edge --bench (stub bridge)
   control ping-pong source                        video source (H.264 file)
   tc shaping on egress (downlink)                 tc shaping on egress (uplink)
   mosquitto broker (see Topology)
```

- **Two Raspberry Pis**, the same model and OS image, one per role. Pi A is
  the robot: it sends video and answers control. Pi B is the operator: it
  sends control and receives video. Both run the same class of hardware as
  the real robot (the XGO-Lite's CM4), so the robot-side cost of each
  protocol is realistic.
- **An unmanaged gigabit switch** with nothing else attached during runs.
  A direct cable would also work; the switch adds a few microseconds and
  matches a real deployment better.
- **No other traffic:** Wi-Fi off on both Pis (`rfkill block wifi`) unless
  the Wi-Fi profile is being run, and no desktop session.

**Checks before every run, recorded with the results:**

- CPU governor set to `performance`, so the clock doesn't change mid-run.
- Heatsinks fitted; `vcgencmd get_throttled` must read `0x0` both before
  and after the run. Discard any run that throttled.
- `uname -a`, the Pi model (`cat /proc/device-tree/model`), the git commit,
  and the binary versions.
- The `tc qdisc show dev eth0` output on both Pis.

### Topology per protocol

| Protocol | Where it runs |
|---|---|
| Channel B | `robot-edge --bench` on Pi A, `operator-console --bench` on Pi B. One QUIC connection carries Channel A video and Channel B control. |
| Zenoh | Peer to peer, one session per side, direct link, no router. |
| MQTT | mosquitto on **Pi B**, the operator side, as in a typical deployment where the robot publishes to a base-station broker. Video and control share each client's single TCP connection. |
| WebRTC | One peer connection. Video as an RTP H.264 track, with WebRTC's own congestion control. Control on a data channel, unordered with `maxRetransmits=0`. |

## Link profiles

Shaping is applied with `tc` on each Pi's egress (`eth0`). `netem` sets delay
and loss, and `tbf` below it sets rate and buffer. Each Pi shapes only its
own sending direction, so uplink and downlink are set independently.

| Profile | Robot uplink (Pi A egress) | Operator downlink (Pi B egress) | One-way delay | Loss | Bottleneck buffer |
|---|---|---|---|---|---|
| **E0** Ethernet, unshaped | ~940 Mbps (line rate) | line rate | none added | 0 | NIC default |
| **W1** Wi-Fi-like | 20 Mbps | 50 Mbps | 5 ms | 0.1% | 1 MB (bufferbloat-prone) |
| **C1** Cellular-like | 5 Mbps | 20 Mbps | 25 ms | 0.5% | 256 KB |

For example, W1 on Pi A:

```bash
sudo tc qdisc replace dev eth0 root handle 1: netem delay 5ms loss 0.1%
sudo tc qdisc add dev eth0 parent 1: handle 2: tbf rate 20mbit burst 32kb limit 1mb
```

The 1 MB buffer in W1 is deliberate. Deep buffers in consumer Wi-Fi gear are
where head-of-line blocking hurts, so a protocol that fills the buffer will
show it.

A later **W2** profile could use the Pis' real Wi-Fi through an access point
instead of emulation. Its results go in a separate table, since real radio
conditions can't be repeated exactly.

## Workloads

### Video (robot to operator)

- **Source:** the same pre-encoded H.264 file for every protocol, so every
  protocol carries identical bytes.
- **Encoding:** 1280×720 at 30 fps, one I-frame per second, constant
  bitrate, encoded at three rates for each link profile:

  | Video load | W1 (20 Mbps uplink) | C1 (5 Mbps uplink) |
  |---|---|---|
  | **L0:** none | control only | control only |
  | **L1:** 50% of uplink | 10 Mbps | 2.5 Mbps |
  | **L2:** 90% of uplink | 18 Mbps | 4.5 Mbps |
  | **L3:** 150% of uplink | 30 Mbps | 7.5 Mbps |

- **Channel B:** `robot-edge --camera --camera-bin <wrapper>`. The wrapper
  ignores `rpicam-vid`'s arguments and plays the file at real-time rate with
  `ffmpeg -re -i video.h264 -c copy -f h264 -`. The video then goes through
  `robot-edge`'s real Channel A path: NAL splitting, coalescing and
  chunking.
- **Other protocols:** a new `video` mode in `tools/proto-bench` that sends
  the same file's NAL units at their real-time timestamps:
  - **Zenoh:** publishes on a video key at `Priority::DataLow` with
    `CongestionControl::Drop`.
  - **MQTT:** publishes at QoS 0 on a video topic.
  - **WebRTC:** writes the NAL units to an RTP H.264 track.

### Control (operator to robot and back)

- **Rate:** 50 Hz, the operator console's default tick, for 60 s per run,
  after a 10 s warm-up with the video running.
- **Message:** 64 B ping-pong, echoed by the robot side.
- **Channel B:** runs both the transport-only variant (`--bench-raw`) and the
  full-frame variant (`channel-b`), as in doc 11.
- **Settings for the other protocols:**
  - **Zenoh:** `Priority::RealTime`, with `CongestionControl::Block` for
    control. This is the protocol's own recommended setup.
  - **MQTT:** QoS 0 on a control topic.
  - **WebRTC:** an unordered data channel with `maxRetransmits=0`.
- **Pacing:** each ping is sent on schedule whether or not the previous
  reply has come back. This is unlike the idle-link ping-pong in doc 11,
  which waits for each reply. A control loop doesn't stop sending because
  the network is slow, so a late reply counts as late, not as a pause.

## Metrics

Round trips are measured on the operator Pi. Only a round trip can be timed
on one clock: NTP-level sync on a LAN is too coarse for these numbers, and
the Pi 4's NIC has no hardware PTP timestamping.

| Metric | Definition | Why |
|---|---|---|
| Control RTT p50 / p99 / max | Round trip of each 50 Hz ping | How stale the control loop runs |
| Late fraction > 100 ms | Share of pings whose reply took over 100 ms | Past the point an operator notices lag |
| Watchdog-equivalent fraction > 400 ms | Share over 400 ms, `robot-edge`'s task-class-D blackout threshold | Replies this late would have tripped a safety stop on the real robot |
| Control loss | Pings with no reply within 2 s | Unreliable modes lose instead of delay |
| RTT inflation | Loaded p99 ÷ idle (L0) p99 on the same profile | The headline number; independent of absolute hardware speed |
| Video goodput | Video bytes delivered per second at the operator | Shows who kept control fast by dropping video |
| Video frames delivered | Complete frames received ÷ frames sent | Same, counted in frames |

For Channel B, the robot side additionally counts `E-Stop latched` events
in `robot-edge`'s log. The benchmark stub bridge doesn't actuate anything,
but a latch still shows the watchdog would have fired.

**The primary result** is RTT inflation and the >400 ms fraction at load
L3 on profile W1: video at 150% of a Wi-Fi-like uplink.

## Procedure

For each **link profile × protocol × video load**:

1. Apply the link profile on both Pis and record the `tc` output.
2. Start the robot side: the responder and the video source.
3. Start the operator side: the video receiver and the control ping-pong.
4. Wait for the 10 s warm-up, then record for 60 s.
5. Stop everything, collect the logs, and check throttling.
6. Repeat three times. Report the run with the middle p99, and store all
   three.

A driver script (`benchmark/run_under_load.py`) will run the whole matrix
over SSH from a third machine and write to `benchmark/results/`, like
`run_loopback.py`.

## What counts as an advantage

A protocol handles video contention well on a profile if, at load L3:

- RTT inflation (p99) is at most 2×;
- no pings exceed 400 ms; and
- it still delivers video, degraded but not zero.

A protocol that keeps control fast only by stalling video completely has
not passed. The robot still needs its camera.

## Implementation work

| Item | Where | Notes |
|---|---|---|
| Bench echo/count | `robot-edge --bench`, `operator-console --bench` | Done (doc 11) |
| Paced control ping-pong, reporting late fractions | `operator-console --bench`, `tools/proto-bench` | New mode: fixed-rate pings, replies matched by sequence number |
| File-backed camera wrapper | `benchmark/` script | For `robot-edge --camera-bin` |
| Video publisher and receiver | `tools/proto-bench` | Zenoh, MQTT, and WebRTC (RTP track) |
| Priority settings | `tools/proto-bench` | Zenoh `RealTime` for control and `DataLow` + `Drop` for video |
| Cross-compiling for the Pis | `Cross.toml` exists | aarch64 builds of `robot-edge`, `operator-console`, `proto-bench` |
| Matrix driver | `benchmark/run_under_load.py` | SSH, `tc` setup, collection |
| Channel B datagram priority fix | `robot-edge/src/quic_server.rs` | Phase 2, separate review |

## Open questions

1. **Which Pi model?** A Pi 4 or Pi 5 has real gigabit Ethernet. A Pi 3B+
   tops out around 300 Mbps over USB, which is fine for W1 and C1 but not
   for E0.
2. **64-bit OS?** The cross-compile target depends on it: aarch64 or armv7.
3. **Operator hardware:** a second Pi keeps the setup symmetric and simple.
   A laptop is closer to a real operator station. Recommendation: two Pis
   for the main matrix, with a single laptop-operator run as a sanity check.
4. **Switch model and speed,** recorded for the report.
5. **The W2 profile,** the Pis' real Wi-Fi through an access point: now, or
   after W1 and C1?
