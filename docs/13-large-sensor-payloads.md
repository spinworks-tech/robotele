# Proposal: large sensor payloads (point clouds, lidar, radar, depth)

**Status: approved; sensor slices implemented, the rest pending.** This
document describes how RoboProtocol carries sensor data that doesn't fit in
one QUIC datagram. Sensor slices, their negotiation, and a synthetic sensor
source (`robot-edge --sim-sensor`) are implemented; per-sensor budgets,
`SensorControl`, bulk objects and recording are not yet. First
measurements on the robot's CM4 are in [Measured on the CM4](#measured-on-the-cm4);
see [Implementation work](#implementation-work) for what remains.

Today every RoboProtocol message is either one datagram (Channel B, at most
1,200 bytes of payload) or a reliable stream (Channel C). Video is the one
exception: Channel A splits each video frame into 1,100-byte chunks and
reassembles them. Navigation and perception data is much larger than a
Channel B frame, and none of the existing paths suits it:

- **Channel B** refuses anything over one datagram, by design
  ([02 — Payload sizing](02-protocol-architecture.md#payload-sizing--quantization)).
- **Channel C** (reliable streams, including ROS 2 tunneling) delivers every
  byte in order. On a lossy link, a lost packet holds up everything behind it
  until it is retransmitted, so a live sensor feed falls further and further
  behind real time.
- **Channel A's chunking** needs every chunk of a frame to arrive. That works
  for a 20-chunk video frame but not for a 500-chunk depth map (see
  [Why not fragment and reassemble](#why-not-fragment-and-reassemble)).

This proposal covers three use cases:

1. Point clouds for navigation.
2. Lidar and radar.
3. Stereo vision and depth maps.

## What the QUIC standards provide

- **RFC 9221 (DATAGRAM frames)** doesn't allow a datagram to be split across
  packets. Each one must fit in a single QUIC packet, and the RFC leaves
  larger data to the application.
- **RFC 9000 (QUIC)** forbids IP fragmentation. It offers streams, which are
  reliable with no size limit, and path MTU discovery (§14, and DPLPMTUD in
  RFC 8899), which grows packets from 1,200 bytes up to what the path
  carries. That is about 1,452 bytes on the internet, which is already our
  `MAX_DATAGRAM_SIZE`, and about 9,000 bytes on a wired network with jumbo
  frames.
- **IETF work in progress.** RESET_STREAM_AT
  (draft-ietf-quic-reliable-stream-reset) makes streams partially reliable.
  Media over QUIC (MoQ) carries large objects by giving each one its own
  stream, which it can cancel.

None of these standards defines what this proposal needs: large, loss-tolerant
data whose pieces are useful on their own. That has to be designed at the
application layer, as below.

## How big the data is

Example sizes, raw versus the encodings proposed below. "Slices" is the
number of datagrams per frame, each carrying up to 1,100 bytes of whole
elements (points, lidar columns, or 32-pixel depth-row segments), so the
counts round up per element rather than per byte.

| Data | Example | Raw | Proposed encoding | Slices per frame |
| --- | --- | --- | --- | --- |
| Navigation point cloud | 10,000 points after voxel filtering, 10 Hz | 16 B/point (float32 x, y, z, intensity): 160 KB, 12.8 Mbps | 7 B/point (int16 x, y, z + uint8 intensity): 70 KB, 5.6 Mbps | 64 |
| 3D lidar | 64 beams × 1,024 columns, 10 Hz (e.g. Ouster OS1-64) | 16 B/point: 1.05 MB, 84 Mbps | Range image, 3 B/point (uint16 range + uint8 intensity): 197 KB, 15.7 Mbps | 205 |
| 2D lidar | 720-sample scan, 15 Hz | 8 B/sample: 5.8 KB, 0.26 Mbps | Range image, one row, 3 B/sample: 2.2 KB | 2 |
| Radar detections | 300 detections, 20 Hz | 20 B/detection: 6 KB | 9 B/detection (int16 x, y, z, Doppler + uint8 SNR): 2.7 KB, 0.43 Mbps | 3 |
| Depth map | 640 × 480, 30 Hz | 2 B/pixel: 614 KB, 147 Mbps | 320 × 240 at 15 Hz, uint16 mm: 154 KB, 18.4 Mbps before compression | 142 |
| Stereo pair | 2 × 640 × 480 grayscale, 30 Hz | 614 KB, 147 Mbps | Video on Channel A (see [Stereo](#stereo-vision)) | n/a |

Two things follow from this table:

- **Encoding matters more than transport.** Quantizing to the precision the
  operator needs cuts these streams by 2–5× before any compression, the same
  approach Channel B already takes with its quantization tiers.
- **Most of these streams don't fit a teleop uplink at full rate.** A 20 Mbps
  cellular uplink carries the navigation cloud easily, but not raw lidar or
  depth alongside video. Rate control and choosing what to drop
  ([Rate control](#rate-control-and-priority)) are part of the feature, not
  an extra.

## Proposal

Two delivery modes, chosen per data type:

| Mode | For | Path | On loss |
| --- | --- | --- | --- |
| **Sensor slices** (new) | Live sensor frames: point clouds, lidar, radar, depth | Datagrams, new tag `0x04` | The frame arrives with lower density; nothing is retransmitted |
| **Bulk objects** (new, on Channel C) | Things that must arrive whole: maps, occupancy grids, keyframe clouds, full-resolution snapshots on request | One unidirectional QUIC stream per object | Retransmitted; a newer version cancels the older one |

Channel B stays one datagram per message. Control latency depends on that.

### Sensor slices

A sensor frame is split into **slices**: datagrams that each decode on their
own. This is different from fragmentation, where a piece is useless without
the others. Doc 02 already requires this for oversized Channel B payloads,
which are split "across multiple independently-timestamped datagrams grouped
by body region." Sensor slices apply the same rule to sensor data.

**Slices interleave instead of tiling.** Slice *k* of *n* carries elements
*k*, *k + n*, *k + 2n*, and so on:

| Data | An "element" is | Losing one slice of *n* costs |
| --- | --- | --- |
| Point cloud, radar | One point | 1/*n* of the points, spread across the whole cloud |
| Range image (3D or 2D lidar) | One column (one azimuth) | 1/*n* of the angular resolution, everywhere |
| Depth map | A 32-pixel segment of one row (a full 640-pixel row wouldn't fit a slice) | 1/*n* of the pixels, spread across the image |

Loss makes the data sparser rather than leaving a hole. That matters for
navigation: a missing wedge of a point cloud can hide an obstacle, while a
uniformly thinner cloud still shows it.

**Slice header.** A fixed binary header, not FlatBuffers, like Channel A's
chunk header:

| Field | Type | Meaning |
| --- | --- | --- |
| tag | u8 | `0x04` |
| sensor_id | u8 | Which sensor, from the session's `SensorDescriptor` list |
| frame_seq | u32 | Frame counter, per sensor |
| capture_time_us | u64 | Robot-clock capture time; the operator maps it to its own clock with the existing offset estimate (`timestamp::estimate_clock_offset_us`) |
| slice_index | u16 | *k* |
| slice_count | u16 | *n* |

That is 18 bytes. The tables in this document assume 1,100 bytes of slice
payload, the same as Channel A's chunks, and every element must fit in
1,100 bytes so a descriptor is valid on any path. The sender fills each
slice as far as the connection allows (`dgram_max_writable_len`): on the
CM4's 1,452-byte datagrams that's 21 elements of 64 bytes, 1,344 bytes per
slice. Packets per second are what limit Wi-Fi from the CM4, so fuller
slices carry more ([14 — Full-size slices on the CM4](14-protocol-comparison-wifi.md#full-size-slices-on-the-cm4)).
`robot-edge --slice-payload-bytes` caps the size for a path with a smaller
MTU.

**Encodings.** Three are enough for v1:

- **Points.** Quantized int16 x, y, z in the sensor frame, at a scale set in
  the descriptor (1 cm covers ±327 m, 1 mm covers ±32 m), plus optional
  extra fields: uint8 intensity, int16 Doppler and uint8 SNR for radar. This
  covers navigation clouds, filtered 3D lidar and radar detections.
- **Range image.** uint16 range per (beam, column), plus optional uint8
  intensity. The operator rebuilds x, y, z from the beam angles in the
  descriptor. That's a third of the size of quantized points. A 2D lidar is a
  range image with one beam.
- **Depth image.** uint16 depth in millimetres per pixel, in 32-pixel row
  segments. The operator back-projects with the camera intrinsics in the
  descriptor.

Compressing each slice on its own (for example zstd with a trained
dictionary) keeps slices independent. Whether it pays off on ~1 KB slices
needs measuring before it goes into the spec.

**Processing stays on the robot.** Motion correction (deskewing a lidar
sweep), voxel filtering and depth computation from stereo all happen before
encoding. The operator receives data ready to display, in the sensor frame,
with the sensor's mounting pose in its descriptor. The robot's pose at
capture time travels in Channel B telemetry.

**Receiving.** The operator shows a frame when all of its slices have arrived,
or when its deadline passes (by default 1.5 frame periods), whichever comes
first, and reports how complete each frame was. Pending frames are capped in
number, as `NalReassembler` already does for video, so memory stays bounded
under loss.

### Bulk objects

Some large data must arrive whole and is updated rarely: an occupancy grid
or map, an accumulated keyframe cloud, or a full-resolution depth frame the
operator asks for. These go on Channel C, using the Media over QUIC pattern.
**Implemented** (`roboprotocol_core::bulk`, `robot-edge/src/bulk_sender.rs`):

- **One unidirectional stream per object.** Each object goes on its own
  server-initiated unidirectional stream: a 24-byte header (format, kind,
  source id, encoding, version, byte length, capture time), the bytes, then
  FIN. Streams are reliable, so an object arrives whole or not at all. The
  receiver checks the length against the header, caps objects at 64 MB, and
  drops partial or outdated versions.
- **Kinds:** occupancy grid (encoding 1: a 20-byte grid header, then one
  byte per cell, 0 free / 100 occupied / 255 unknown), keyframe cloud, depth
  snapshot, and opaque blob.
- **A version in flight always finishes; the newest waits.** While one
  version of an object is being sent, a newer one waits, and a still-newer
  one replaces the waiting version, which never started. This proposal first
  said the newer version should cancel the stream in flight
  (RESET_STREAM). That starves: with 600 KB maps every 0.5 s on a 5 Mbps
  link, all 30 versions were cancelled in flight and none arrived. Live
  sensor frames can be cut because a partial frame is still useful; a
  partial map is not.
- **Lowest priority, one at a time.** Bulk streams use urgency 7
  (E-Stop's is 0) and are non-incremental, so quiche finishes one before
  starting the next. quiche puts datagrams ahead of stream data in every
  packet, so a map never sits in front of a control command inside a packet.
  When `--lossy-rate-control` is on, bulk bytes also come out of the rate
  cap's budget, after video and sensor datagrams.
- **A small flow-control window: 256 KB per stream.** quiche's default of 0
  let neither side send on a unidirectional stream at all. 4 MB was too
  much: the sender counts an object as done once quiche has accepted all of
  it, and with a window that size whole maps were "done" long before they
  were sent, so newer versions piled up in quiche's buffers. 256 KB still
  allows ~100 Mbps at a 20 ms round trip.
- **Trying it:** `robot-edge --sim-map SECONDS` publishes the sim room's
  occupancy grid (1,000 × 600 cells at 2 cm, 600 KB).
  `operator-console --bulk-save-dir DIR` writes the latest version of each
  object there, a map also as a `.pgm` image, and the HUD shows a bulk row.

On a 5 Mbps link with a map published every 0.5 s, the operator received a
complete map about every second (versions 1, 3, 5, ... 26 of 30), which is
as fast as 600 KB fits through 5 Mbps.

**Effect on control, on the CM4 over Wi-Fi** (`benchmark/bulk_under_load.py`,
2026-10-07, 50 Hz paced control as in doc 12, camera running, medians over 2
rounds):

| Robot sends | Control p50 | p99 | Over 400 ms | Maps delivered |
| --- | --- | --- | --- | --- |
| Camera only | 9 ms | 186 ms | 0% | – |
| + a map every second | 29 ms | 344 ms | 0.4% | 60 of 60 (4.8 Mbps) |
| + 4 maps a second | 53 ms | 448 ms | 1.6% | 134 (10.8 Mbps) |

A map a second is affordable: every one arrived and control stayed usable,
its median up ~20 ms and its p99 roughly doubled. Four a second costs
more: the tail grew and one run tripped the watchdog repeatedly. A fourth
case, 4 maps a second with `--lossy-rate-control on`, came out far worse
(p50 206 ms, p99 5.9 s), but it always ran last in its round, and the
Wi-Fi that evening swung between good and very poor within minutes;
control was bad while the cap held our own traffic to ~1 Mbps, so the
congestion wasn't ours. On a stable shaped 20 Mbps link the same case
behaves as intended: the cap took control from p50 50 / p99 62 ms to 22 /
34 ms while still delivering 14 Mbps of maps (16 without the cap). The
script now rotates the case order and logs the idle ping before each case;
the CM4 cap case needs rerunning that way.

Not done yet: requesting an object (such as a full-resolution snapshot) from
the operator side, recording bulk objects, and advertising available objects
in `SESSION_DESCRIBE`.

Streams are also the fallback for any sensor stream the operator wants
lossless, at the cost of latency under loss.

### Stereo vision

Raw stereo imagery is video and goes on Channel A, as two cameras in
`SESSION_DESCRIBE`. Depth is computed on the robot from the uncompressed
pair, and the depth map goes out as sensor slices. Computing depth on the
operator side from compressed video doesn't work well: compression artifacts
corrupt the pixel matching that stereo depth relies on. A stereoscopic
display (for example a VR headset) needs left and right frames paired by
capture time; see [Open questions](#open-questions).

## Rate control and priority

Sensor slices use the robot-to-operator direction, the same one doc 12
identifies as exposed: everything sent there joins one first-in, first-out
datagram queue in quiche, with no priority. A 205-slice lidar frame in that
queue would delay Channel B telemetry and haptic feedback exactly as video
does today. So:

1. **Doc 12's fix comes first** (now implemented). Lossy data waits in
   `robot-edge` and enters quiche's queue only while it holds fewer than 8
   datagrams, so Channel B waits behind at most 8 lossy datagrams.
2. **The latest frame wins, per sensor.** When a new frame is ready and the
   previous one hasn't been fully handed to quiche, the rest of the old frame
   is dropped. Channel A already does this for video. `robot-edge
   --sensor-frame-policy finish` instead lets a started frame complete and
   keeps only the newest frame waiting; on the CM4 that delivered more whole
   256 KB frames but with a much longer delay tail, and made no measurable
   difference at 16 and 64 KB
   ([14 — Cut or finish](14-protocol-comparison-wifi.md#cut-or-finish-a-frame-the-link-cant-keep-up-with)).
   Which suits a sensor should become a per-sensor setting.
3. **Each sensor has a budget.** The robot advertises a maximum rate and
   bitrate per sensor. The operator picks a budget for each one in
   `SESSION_ACCEPT`, and can change it during the session with a
   `SensorControl` message on Channel C (like `CameraControl`). When the link
   can't carry the budget, the robot first lowers the frame rate, then
   coarsens the data (larger voxels, fewer columns, a smaller depth map).
4. **A fixed priority order.** E-Stop, then Channel B (commands, telemetry,
   haptic), then video and sensor slices, then bulk objects. The operator
   chooses whether video or sensors are dropped first, because which matters
   more depends on the task: a navigation task may want the point cloud over
   the camera.

## Negotiation

New, additive FlatBuffers fields, so older peers are unaffected:

```text
SensorDescriptor                       (robot → operator, in SessionDescribe.sensors)
├── sensor_id, label
├── kind: PointCloud | RangeImage | Depth   (radar: PointCloud with Doppler/SNR)
├── encoding fields: quantization scale, extra point fields
├── geometry: beam angles (range image) or intrinsics + size (depth)
├── mounting pose in the robot base frame
└── max_hz, max_bitrate_kbps

SessionAccept                          (operator → robot)
├── selected_sensors: [sensor_id]
└── sensor_budgets: [(sensor_id, hz, bitrate_kbps)]   (not implemented yet)
```

The robot sends slices only for sensors the operator selected. An operator
built before this change never selects any, and ignores tag `0x04` if one
arrives anyway.

Sensor descriptors can push `SESSION_DESCRIBE` past one packet: a 64-beam
lidar's elevation table alone is 256 bytes. The robot therefore ends
`SESSION_DESCRIBE` with a FIN on stream 1, and receivers buffer stream 1
until the FIN before decoding. Until this change, receivers decoded each
read on its own, which broke (with a panic in the unverified FlatBuffers
decoder) as soon as a describe spanned two packets. A receiver built with
this change therefore needs a robot built with it too.

## Why not fragment and reassemble

Fragmentation, which Channel A uses for video, loses the whole frame when any
fragment is lost. The chance that a frame of *n* fragments arrives intact at
packet loss *p* is (1 − *p*)<sup>*n*</sup>:

| Frame | Fragments | Intact at 0.1% loss | Intact at 1% loss |
| --- | --- | --- | --- |
| Navigation cloud | 64 | 94% | 53% |
| 3D lidar range image | 205 | 81% | 13% |
| Depth map, 320 × 240 | 142 | 87% | 24% |
| Depth map, 640 × 480 | 565 | 57% | 0.3% |

With slices, 1% loss means each frame arrives with 99% of its data. Forward
error correction could make fragmentation workable, but it spends bandwidth on
redundancy that slicing doesn't need.

## Alternatives considered

- **All sensor data on reliable streams.** The data arrives whole, but late.
  Under loss, retransmission holds up every frame behind the lost packet, and
  the operator sees stale data. This fits bulk objects, not live feeds.
- **Larger packets only** (path MTU discovery, jumbo frames). Worth enabling,
  and slices grow with it automatically, but it doesn't help on the internet,
  where packets top out around 1,452 bytes.
- **Sensor data over Zenoh** (as the DimOS integration in doc 10 already
  does for video). Zenoh fragments large messages and has priority lanes,
  but a second transport means a second connection outside the session's
  mTLS, priorities, connection migration and recording. It stays an option
  for integrators, not the protocol's answer.

## Measured on the CM4

Sensor slices have now run on real hardware: the XGO-Lite's CM4 sending
to an operator laptop over 2.4 GHz Wi-Fi. Two sets of measurements apply:

- [14 — Protocol comparison over Wi-Fi](14-protocol-comparison-wifi.md):
  synthetic payloads from 16 B to 1 MB, robot to operator, Channel B
  against Zenoh, MQTT and WebRTC.
- [12 — Results on the CM4 over Wi-Fi](12-control-under-load-benchmark.md#results-on-the-cm4-over-wi-fi):
  the camera plus `--sim-sensor` lidar, depth and cloud at once.

### The loss argument holds

[Why not fragment and reassemble](#why-not-fragment-and-reassemble) predicts
that a frame of *n* datagrams arrives intact with probability
(1 − *p*)<sup>*n*</sup>. Measured, at rates the link carried:

| Message | Slices | Data delivered | Messages complete |
| --- | --- | --- | --- |
| 16 KB × 20/s | 16 | ≥ 99.8% | 99.4% |
| 64 KB × 10/s | 61 | ≥ 99.9% | 100% |
| 256 KB × 5/s | 241 | ≥ 99.9% | 100% |
| 1 MB × 1/s | 964 | ≥ 99.9% | 50% |

At 1 MB, half the messages missed at least one of their 964 slices while
essentially all of the data arrived. That implies roughly 0.07% datagram
loss: (1 − 0.0007)<sup>964</sup> ≈ 0.5. It's a rough figure, based on
about eight 1 MB messages in the counting window. A design that needed
every fragment would have lost half those frames; with slices, each one
was shown with a few points missing.

### The bandwidth table doesn't fit this robot

On the CM4, every datagram-based protocol topped out at about **8–12 Mbps**
robot to operator: Channel B, WebRTC and unencrypted raw UDP alike. MQTT,
over one TCP connection, reached 26 Mbps on the same link. A follow-up
([14 — Where the datagram ceiling is](14-protocol-comparison-wifi.md#where-the-datagram-ceiling-is))
found the CM4 sends about 2,000 packets per second whatever their size, so
raw UDP with full-size 1,400-byte packets matches TCP at about 22 Mbps;
Channel B's 1,100-byte slices and QUIC's overhead are what hold it lower.
Against
[How big the data is](#how-big-the-data-is):

- **3D lidar (15.7 Mbps) or a 320 × 240 depth map (18.4 Mbps)** exceeds the
  ceiling alone, before any video.
- **The navigation cloud (5.6 Mbps)** fits, but with 2 Mbps of video it
  is already near the bottom of that range.
- **2D lidar and radar** (under 0.5 Mbps each) fit easily.

With the camera plus sim lidar, depth and cloud together (about 40 Mbps
offered), sensors kept their frame rates but arrived thin: about a third
of each lidar and depth frame, and three quarters of each cloud frame. That
is the latest-wins rule working, but it means per-sensor budgets are a
requirement on this class of robot, not a refinement.

### What the measurements don't cover yet

- **Frame age at display,** and control latency while sensors load the
  link: doc 12's benchmark with a sensor workload.
- **Real sensor data.** Doc 14 used opaque payloads and the CM4 test used
  `--sim-sensor`; no recorded lidar or depth data has gone through the
  encodings yet.

## Implementation work

Done:

- `roboprotocol-core::sensor`: slice header, the three encodings,
  interleaving, a bounded latest-wins `FrameAssembler` with deadlines, and
  `SensorDescriptor` with validation.
- Schema: `SensorDescriptor` in `SessionDescribe`, `selected_sensors` in
  `SessionAccept`.
- `robot-edge`: `--sim-sensor cloud|lidar|radar|depth`, which ray-casts a
  small room (the reference robot has no lidar or depth sensor), and
  per-sensor latest-frame-wins sending.
- Doc 12's queue fix: video and sensor data wait in `lossy_queue.rs` and
  enter quiche's queue at most 8 datagrams at a time, so Channel B never
  waits behind more than that
  ([12 — Channel B datagram priority](12-control-under-load-benchmark.md#channel-b-datagram-priority)).
  Video and sensors share the link equally for now. On the CM4 over Wi-Fi
  this bounds quiche's queue but not the Wi-Fi driver's, so control latency
  under sensor load still depends on budgets (next item).
- `operator-console`: accepts every valid sensor, assembles and decodes
  frames, and shows a row per sensor in the channels panel (slice rate,
  bandwidth, frame rate, completeness, sample count).
- `SESSION_DESCRIBE` ends with a FIN and is decoded whole (see
  [Negotiation](#negotiation)).
- Doc 02's tag table now lists `0x03`, `0x04` and `0x7F`.
- The localhost smoke test streams sim lidar and radar and checks that
  both assemble.

Remaining, in order (reordered after [the CM4 measurements](#measured-on-the-cm4)):

1. Per-sensor budgets in `SessionAccept`, sized from quiche's delivery-rate
   estimate, and `SensorControl` to change them during a session. On the
   CM4 a single lidar or depth stream exceeds the link.
2. Throughput on the CM4. Done: full-size slices (21 Mbps at 256 KB, level
   with raw UDP and TCP), ACK cost measured (10–20% extra packets), and a
   `finish` frame policy (whole frames at the cost of delay; no gain at
   16–64 KB, where remaining loss is in the air). Next: make the frame
   policy a per-sensor setting alongside the budgets. (5 GHz isn't an
   option: the XGO-Lite's Wi-Fi only does 2.4 GHz.)
   Batched sends (`sendmmsg`/GSO) save CPU but not packets on the air, so
   they come later.
3. Bulk objects: done (see [Bulk objects](#bulk-objects)). Remaining:
   on-request objects, recording, and advertising them in the handshake.
4. A display (Rerun, which the DimOS work already uses, renders points and
   depth images).
5. Recording ([07](07-recording-and-replay.md)) and `replay-decode`
   support for the new tag.
6. Benchmark: add a sensor workload to doc 12's testbed and measure frame
   age at display and the effect on control round trips (completeness under
   load is now measured; see above).

## Open questions

- **Test data.** The XGO-Lite V2 reference robot has no lidar or depth
  sensor. `--sim-sensor` covers development; a recorded real-sensor dataset
  is still needed to judge the encodings on real data.
- **Compression.** Is per-slice compression worth it on ~1 KB slices, and
  which method (zstd with a dictionary, or a depth-specific codec such as
  RVL)?
- **Stereo pairing.** Does Channel A carry a capture timestamp per frame that
  a stereoscopic display can pair on? If not, that's a Channel A change.
- **CPU on the robot.** Voxel filtering, deskewing and quantizing a 64-beam
  lidar at 10 Hz on a Raspberry Pi-class computer need measuring.
- **Default drop order.** Should video or sensors be dropped first by
  default, or should every session have to choose?
