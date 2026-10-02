# Proposal: large sensor payloads (point clouds, lidar, radar, depth)

**Status: proposal, not implemented.** This document proposes how
RoboProtocol carries sensor data that doesn't fit in one QUIC datagram. It
needs review before any code or schema changes.

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

That is 18 bytes, leaving up to about 1,180 bytes of slice payload within
the 1,200-byte budget. The tables in this document assume 1,100 bytes, the
same as Channel A's chunks, to leave room for per-slice compression framing.
The sender sizes slices from `dgram_max_writable_len`, so
slices grow automatically wherever path MTU discovery or jumbo frames allow
bigger packets.

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
operator asks for. These go on Channel C, using the MoQ pattern:

- **One unidirectional stream per object.** The stream starts with a small
  header (object kind, source id, version, byte length, encoding), followed
  by the bytes.
- **The latest version wins.** When version *v+1* of an object starts,
  the sender cancels any unfinished stream for version *v*
  (`stream_shutdown`, which sends RESET_STREAM). The receiver drops the
  partial copy.
- **Low priority.** Streams are sent at a lower urgency than the E-Stop
  stream (`stream_priority`, already used for E-Stop).
- **A config change is needed first.** Neither endpoint calls
  `set_initial_max_stream_data_uni`, and quiche's default is 0, so the peer
  currently can't send any data on a unidirectional stream.

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

1. **Doc 12's fix comes first.** That fix stops handing lossy data to quiche
   once the datagram queue passes a small threshold, and purges queued lossy
   data when a Channel B frame needs to go out. `dgram_purge_outgoing` takes
   a filter, so it can drop tag `0x04` and `0x02` datagrams while keeping
   Channel B.
2. **The latest frame wins, per sensor.** When a new frame is ready and the
   previous one hasn't been fully handed to quiche, the rest of the old frame
   is dropped. Channel A already does this for video.
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
├── kind: PointCloud | RangeImage | Radar | Depth
├── encoding fields: quantization scale, extra point fields
├── geometry: beam angles (range image) or intrinsics + size (depth)
├── mounting pose in the robot base frame
└── max_hz, max_bitrate_kbps

SessionAccept                          (operator → robot)
├── selected_sensors: [sensor_id]
└── sensor_budgets: [(sensor_id, hz, bitrate_kbps)]
```

The robot sends slices only for sensors the operator selected. An operator
built before this change never selects any, and ignores tag `0x04` if one
arrives anyway.

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

## Implementation work

1. Doc 12's queue fix in `robot-edge` (a prerequisite, already planned).
2. `roboprotocol-core::sensor`: slice header, the three encodings,
   interleaving, and a bounded receiver modelled on `NalReassembler`.
3. Schema: `SensorDescriptor`, the `SessionAccept` additions, `SensorControl`.
4. `robot-edge`: per-sensor latest-frame-wins sending, with budgets.
5. `operator-console`: frame assembly with deadlines, completeness stats, and
   a display (Rerun, which the DimOS work already uses, renders points and
   depth images).
6. Bulk objects: unidirectional-stream config, the object header,
   cancel-on-supersede.
7. Recording ([07](07-recording-and-replay.md)) and `replay-decode`
   support for the new tag.
8. Doc 02: add tag `0x04`, and also the `0x03` heartbeat and `0x7F` bench tags,
   which are in the code but missing from the tag table.
9. Benchmark: add a sensor workload to doc 12's testbed and measure frame
   completeness, age at display, and the effect on control round trips.

## Open questions

- **Test data.** The XGO-Lite V2 reference robot has no lidar or depth
  sensor. The first implementation needs a recorded dataset or a simulator
  as its source.
- **Compression.** Is per-slice compression worth it on ~1 KB slices, and
  which method (zstd with a dictionary, or a depth-specific codec such as
  RVL)?
- **Stereo pairing.** Does Channel A carry a capture timestamp per frame that
  a stereoscopic display can pair on? If not, that's a Channel A change.
- **CPU on the robot.** Voxel filtering, deskewing and quantizing a 64-beam
  lidar at 10 Hz on a Raspberry Pi-class computer need measuring.
- **Default drop order.** Should video or sensors be dropped first by
  default, or should every session have to choose?
