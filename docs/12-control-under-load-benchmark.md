# Benchmark plan: control latency under video load

**Status: Channel B measured on the robot; the multi-protocol testbed is
not built yet.** This document defines the benchmark. Channel B's results
on the XGO-Lite's CM4 over Wi-Fi are in
[Paced 50 Hz control on the CM4](#paced-50-hz-control-on-the-cm4); the
comparison with Zenoh, MQTT and WebRTC on the two-Pi testbed is still to
do.

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

## Channel B datagram priority

The test has to be able to show Channel B failing, because before the fix
below it did.

### The gap

- **The application layer already avoided its own backlog.**
  `robot-edge/src/video/channel_a.rs` coalesces video to the latest delta
  frame under backlog, while always sending SPS/PPS/IDR NAL units.
- **The transport queue has no priority.** A datagram handed to quiche with
  `dgram_send` joins one first-in-first-out queue, up to 4,096 datagrams
  deep (`enable_dgram(true, 4096, 4096)` in `quic_server.rs`). Channel B
  telemetry shared that queue with video, and later with sensor slices
  ([13](13-large-sensor-payloads.md)), with no priority. Only the E-Stop
  *stream* is prioritized.
- **The robot-to-operator direction is exposed.** On a 20 Mbps uplink,
  4,096 full datagrams (~1.4 KB each, about 5.6 MB) take about 2 seconds to
  drain, so telemetry and the benchmark echo could sit behind up to 2 s of
  lossy data. Commands travel the other way; they are affected only on
  shared-airtime links such as Wi-Fi, or through delayed ACKs.

### The fix

Video chunks and sensor slices no longer go to `dgram_send` directly. They
wait in `robot-edge/src/lossy_queue.rs`, and `flush` tops quiche's queue up
from there, before every packet, only while quiche holds fewer than 8
datagrams. Channel B telemetry, heartbeat and bench echoes, and E-Stop
datagrams still go straight to quiche, so they wait behind at most 8 lossy
datagrams (about 4 ms at 20 Mbps).

Holding lossy data outside quiche also lets stale data be dropped before it
is sent:

- **Video:** a delta NAL is dropped only once it has waited more than
  150 ms; then every waiting delta goes and later deltas are skipped until
  the next IDR, because each delta references the one before it and the
  rest of the group of pictures would only decode as smear. A new IDR drops
  everything older except the latest SPS and PPS. (The first version
  dropped a waiting delta whenever a newer one arrived; on the CM4 that
  smeared nearly half the video under load -- see below.)
- **Sensors:** a new frame replaces the unsent slices of that sensor's
  previous frame. Slices already sent still decode, as a thinner frame.
- **Sharing:** video and sensors alternate one datagram at a time while both
  are waiting, and sensors take turns among themselves.

The plan above also proposed purging queued video
(`dgram_purge_outgoing`) when a Channel B frame goes out. With quiche's
queue capped at 8 lossy datagrams there is nothing worth purging, and a
purge could throw away the chunks of an IDR.

### First results, on a namespace link

Before the Pi testbed exists, `benchmark/netns_load_test.sh` runs the same
question on one machine: `robot-edge` and `operator-console` in two network
namespaces joined by a veth pair, each direction shaped with `netem` to
20 Mbps, 10 ms delay and a 100-packet queue. `robot-edge --bench echo` runs
with sim lidar, depth and cloud (about 40 Mbps offered, twice the link), and
`operator-console --bench pingpong` times 300 Channel B round trips. Release
builds; the idle-link round trip is 20.9 ms.

| Build | Median | p95 | p99 | Pings lost | Uplink used |
| --- | --- | --- | --- | --- | --- |
| Before the fix | 102.3 ms | 134.4 ms | 152.0 ms | 3 of 300 | -- |
| With the fix (CUBIC) | 53.7 ms | 58.9 ms | 59.2 ms | 0 of 300 | 19.8 Mbps |

The remaining ~33 ms above the idle round trip is the bottleneck's own
queue, which CUBIC, quiche's default congestion control, keeps nearly full.

`robot-edge --cc` selects quiche's other algorithms. On the same link:

| `--cc` | Median | p95 | p99 | Pings lost | Uplink used |
| --- | --- | --- | --- | --- | --- |
| `cubic` (default) | 54.1 ms | 58.9 ms | 59.3 ms | 1 | 19.8 Mbps |
| `bbr` | 42.3 ms | 43.3 ms | 58.7 ms | 10 | 10.7 Mbps |
| `bbr2` | 30.8 ms | 57.3 ms | 58.8 ms | 2 | 11.7 Mbps |

BBR and BBRv2 lower latency mostly by using about half the link, and their
tails vary between runs (a second `bbr2` run measured a 42 ms p95). One
likely cause: `robot-edge` sends packets as soon as quiche produces them and
ignores the pacing time quiche returns (`send_info.at`), which BBR relies
on. CUBIC stays the default until pacing is honoured and the Pi testbed
says otherwise.

### Results on the CM4 over Wi-Fi

`robot-edge` on the XGO-Lite's CM4 (OV5647 camera, 640x480, 30 fps, IDR
every 30 frames, 2 Mbps), `operator-console` on a laptop, both on the same
Wi-Fi network. Idle round trip: 3.2 ms median. Load: the camera plus sim
lidar, depth and cloud, about 40 Mbps offered, well above what the CM4's
Wi-Fi uplink carries.

**Video.** NAL ids traced end to end (temporary debug logging, not
committed), 30 s per run. "Corrupted" counts delta frames that arrived after
an earlier delta of their group of pictures was lost, so they decode as
smear.

| Run | NALs sent | Lost | Lost SPS/PPS/IDR | Deltas shown clean | Deltas shown corrupted |
| --- | --- | --- | --- | --- | --- |
| Camera only | 1,058 | 3 | 0 | -- | -- |
| Under load, first rule (newer delta drops older) | 1,016 | 160 | 0 | 408 | 349 |
| Under load, stale rule (150 ms, then skip to IDR) | 949 | 60 | 3 | 613 | 180 |

Under load, sensors still arrived at close to their target rates, as
thinner frames (about a third of each lidar and depth frame's slices, three
quarters of the cloud's). Of the 60 NALs lost with the stale rule, 40 were
skip-to-IDR runs (clean freezes) and 20 were scattered single losses from
the saturated Wi-Fi link; those cause the remaining corruption. Only the
receiver can turn those into freezes: the operator would stop feeding deltas
to the decoder after a gap in NAL ids, until the next IDR. That is
operator-side work, not yet done. (The operator's playback also keeps only
the latest delta when deltas arrive faster than the decoder takes them,
which drops about 60-70 deltas in 30 s even on an idle link, with the same
smearing. Same fix.)

**Channel B round trips** (`--bench pingpong`, 300 pings, camera plus the
three sim sensors):

| Build | Median | p95 | p99 | Pings lost |
| --- | --- | --- | --- | --- |
| Idle link | 3.2 ms | 5.9 ms | 7.0 ms | 0 |
| Before the fix, run 1 | 123 ms | 402 ms | 594 ms | 5 |
| Before the fix, run 2 | 212 ms | 369 ms | 657 ms | 11 |
| With the fix, run 1 | 180 ms | 336 ms | 516 ms | 9 |
| With the fix, run 2 | 135 ms | 388 ms | 557 ms | 9 |
| With the fix, `--cc bbr2` | 483 ms | 969 ms | 989 ms | 245 |
| With the fix, `--cc bbr2`, again | 327 ms | 877 ms | 955 ms | 250 |

On Wi-Fi the fix makes no measurable difference to control latency: the
runs differ more from each other than the builds do. Here the queue that
grows is not quiche's but the one below it, in the CM4's Wi-Fi driver and
the access point, which CUBIC keeps full. That is the queue the namespace
test's netem buffer stood in for, and it is much deeper on real Wi-Fi.
BBRv2 is far worse here: it lost over 80% of pings. Do not use `--cc bbr2`
on Wi-Fi.

Bringing control latency under load down on Wi-Fi therefore needs the robot
to send less than the link carries -- doc 13's per-sensor budgets, sized
from quiche's delivery-rate estimate -- or a shorter queue under quiche
(for example `fq_codel` and a smaller transmit queue on the CM4's
`wlan0`), or both. Those are the next experiments.

### Paced 50 Hz control on the CM4

The test this document is about, run for Channel B on the robot itself
(2026-10-06, `benchmark/control_under_load.py`, PR #31's build). The
operator laptop sends 64 B pings at a paced 50 Hz, each on schedule whether
or not earlier replies are back (`operator-console --bench pingpace`):
10 s warm-up, then 60 s timed, 3,000 pings per run. `robot-edge --bench
echo` answers them while its uplink carries a load. Instead of this
document's pre-encoded video file, the load is the robot's own camera plus
`--sim-sensor` data, since only Channel B is being measured. Same 2.4 GHz
link as the results above, laptop Wi-Fi power saving off.

| Load (robot → operator) | Delivered |
| --- | --- |
| **L0** control only | – |
| **L1** camera, 640×480 at 30 fps | 0.6–2.1 Mbps of video |
| **L2** camera + navigation cloud | + 6.5 Mbps of sensor data |
| **L3** camera + lidar + depth + cloud, about 40 Mbps offered (2× the link) | ~1 Mbps of video + ~21 Mbps of sensor data |

Medians over 2 rounds. "Inflation" is p99 against L0 of the same variant.

| Load | Variant | p50 | p99 | Max | > 100 ms | > 400 ms | Lost (of 3,000) | Inflation |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| L0 | transport only | 2.2 ms | 130 ms | 268 ms | 5.3% | 0% | 0 | 1.0× |
| L0 | full frame | 2.4 ms | 151 ms | 419 ms | 5.7% | 0.18% | 12 | 1.0× |
| L1 | transport only | 2.9 ms | 218 ms | 391 ms | 8.0% | 0.07% | 5 | 1.7× |
| L1 | full frame | 4.6 ms | 148 ms | 250 ms | 7.2% | 0% | 2 | 1.0× |
| L2 | transport only | 6.3 ms | 263 ms | 408 ms | 15.1% | 0.03% | 34 | 2.0× |
| L2 | full frame | 7.7 ms | 288 ms | 442 ms | 15.1% | 0.14% | 28 | 1.9× |
| L3 | transport only | **249 ms** | **922 ms** | 1,102 ms | **78.7%** | **18.8%** | 176 | **7.1×** |
| L3 | full frame | **196 ms** | **1,046 ms** | 1,223 ms | **67.8%** | **11.7%** | 156 | **6.9×** |

**The watchdog.** At L3 the robot's watchdog (400 ms without a command,
task class D) latched in all four runs of a repeat. A latch stays latched
until cleared, so this means "at least once per minute". On a real session
the robot would have stopped. So under overload, pings *to* the robot were
delayed too, not only the replies queued behind sensor data. The first full
run's latch counts aren't usable: the bench paused sending while it waited
for late replies at the end of each run, and that pause itself tripped the
watchdog. The bench now keeps heartbeats going during that wait; an L0 check
afterwards showed no latch. In a repeat that logged the time, the latch
came 11.6 s into the session, about 1.6 s after the warm-up: under
overload the robot stops within seconds.

What it shows:

- **The link itself stalls.** With no load at all, 5% of pings took over
  100 ms. Any protocol running on this Wi-Fi inherits that.
- **Moderate load is tolerable.** With the camera alone the median barely
  moves. At L2 (about 8.5 Mbps delivered) the median stays under 10 ms and
  the p99 doubles, but almost nothing passes 400 ms.
- **Overload breaks control.** At L3 the median goes to 200–250 ms, the p99
  to about 1 s, and the watchdog latches. The robot's own queue is bounded
  (see [The fix](#the-fix)), but the robot keeps the Wi-Fi driver's queue
  and the access point's full. The CM4's `wlan0` queue alone holds 1,000
  packets, about half a second at this link's ~2,000 packets per second.
- **Video survived, degraded:** about 1 Mbps still arrived at L3.

**Against [What counts as an advantage](#what-counts-as-an-advantage)**, at
L3: p99 inflation is 7× (the bar is 2×) and pings exceed 400 ms (the bar
is none). Video is delivered, degraded, which passes. So Channel B fails
this benchmark on the robot under overload. The fix has to keep the robot
from sending more than the link carries: doc 13's per-sensor budgets, sized
from quiche's delivery-rate estimate, and/or a short queue under quiche
(`fq_codel` on `wlan0`).

Caveats: two rounds only, on a shared 2.4 GHz channel. The camera's bitrate
differed between rounds (2.1 against 0.6 Mbps, lighting most likely), which
is why L1 and L2 vary. The link also varies from day to day: three more L3
runs the next morning gave p50 194–643 ms and p99 3.0–4.8 s, worse than the
table, with the watchdog latching in every run. One of those diagnostic runs
failed without a summary; its error text wasn't captured, and two repeats
completed normally. A plausible cause is QUIC's 10 s idle timeout closing
the connection when replies stall that long, which would itself be a
failure mode under overload, but it wasn't confirmed.

So the Pi benchmark runs in two phases:

1. **Baseline:** a build from before the fix (any commit before
   `lossy_queue.rs`).
2. **With the fix**, and with each `--cc` setting.

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
| Paced control ping-pong, reporting late fractions | `operator-console --bench pingpace` (done), `tools/proto-bench` (to do) | Fixed-rate pings, replies matched by sequence number; Channel B measured on the CM4 |
| File-backed camera wrapper | `benchmark/` script | For `robot-edge --camera-bin` |
| Video publisher and receiver | `tools/proto-bench` | Zenoh, MQTT, and WebRTC (RTP track) |
| Priority settings | `tools/proto-bench` | Zenoh `RealTime` for control and `DataLow` + `Drop` for video |
| Cross-compiling for the Pis | `Cross.toml` exists | aarch64 builds of `robot-edge`, `operator-console`, `proto-bench` |
| Matrix driver | `benchmark/run_under_load.py` | SSH, `tc` setup, collection |
| Channel B datagram priority fix | `robot-edge/src/lossy_queue.rs` | Done; see [Channel B datagram priority](#channel-b-datagram-priority) |
| Honour quiche's pacing (`send_info.at`) | `robot-edge/src/quic_server.rs` | Needed before BBR/BBRv2 can be judged fairly |

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
