# Design Review & Roadmap

Source of truth: `REVIEW.md` (a critical internal review) and `INTERLATENT.md` (a
comparison against a competing open-source teleoperation SDK). Both are worth reading
directly for full detail; this page summarizes the verdicts and tracks which gaps
this repository has since closed.

## What's genuinely novel

- **Task-adaptive, human-perception-grounded safety tiers** (Task Classes B–E). No
  existing robotics middleware (ROS 2/DDS, rosbridge, Isaac SDK) embeds
  network-health-aware safety semantics at the transport layer.
- **TDPA at the protocol level.** Existing teleoperation systems implement passivity
  observers in application-layer controllers, invisible to the network stack and
  impossible to standardize across vendors. Here it rides in the datagram header
  itself.
- **Channel separation** (video / control / state+E-Stop) with distinct reliability
  and priority semantics per channel.
- **Decay-based dead reckoning** instead of linear Kalman extrapolation — eliminates
  a class of dangerous momentum-continuation collisions.
- **A cross-platform hardware acceleration matrix** (Intel/AMD/ARM/ESP32/Broadcom)
  giving implementers concrete silicon targets instead of "use hardware acceleration"
  hand-waving.

## Gaps identified — and their status in this repository

`REVIEW.md`'s original critique was written against a spec-only draft, before a
reference implementation existed. Several of its top items have since been addressed
directly in `DESIGN.md`/`REQUIREMENTS.md` and this codebase:

| Priority | Original gap | Status |
| --- | --- | --- |
| P0 | No wire-format specification — "custom RTP/UDP" descriptions weren't implementable byte layouts | **Addressed.** `DESIGN.md` §1–2 now specifies exact FlatBuffers tables, byte budgets, and the 1-byte channel discriminator; `crates/roboprotocol-proto/schemas/roboprotocol.fbs` is the literal wire schema. |
| P0 | No version negotiation or extension mechanism | **Addressed.** The `HELLO` capability handshake (`DESIGN.md` §1.2, `roboprotocol-core::hello`) does exactly this. |
| P1 | QUIC not evaluated as a transport alternative | **Addressed and adopted.** The protocol is now built on QUIC (RFC 9000) + RFC 9221 datagrams throughout, implemented via `quiche` — see [Protocol Architecture](02-protocol-architecture.md). |
| P0/P1 | No reference implementation — "only the original authors can implement it" | **Addressed for one hardware target.** `robot-edge`/`operator-console`/`xgo_bridge` are a real, running implementation against the XGO-Lite V2 — see [Reference Implementation](05-reference-implementation.md). Third-party interoperability (a second independent implementation) is still unproven. |
| P1 | Define ROS2/DDS bridge specification | **Specified, not yet implemented.** `DESIGN.md`/`REQUIREMENTS.md` FR-5 defines the bridge; no `rclcpp` node exists in this codebase yet. |
| P1 | No device discovery or pairing mechanism | **Partially addressed.** NAT traversal / relay fallback is now specified (`DESIGN.md` §1.4); mDNS/QR-code *pairing* (as opposed to path establishment to an already-known peer) is still open. |
| P2 | No shared autonomy / AI-assisted control semantics | **Open.** `FR-4.1.2` names Semi-Autonomous Mode but the protocol semantics for autonomy-level negotiation remain undefined. |
| P2 | Telesurgery (Class A) scope creep | **Resolved by simplification.** The current Task Class taxonomy runs B–E; a Class A medical extension profile is not part of the core spec. |
| P2 | Publish a reference SDK | **In progress.** One Rust reference implementation exists; a second-language SDK doesn't yet. |
| P3 | Submit to a standards body | **Not started.** |

## Comparison against Interlatent (a shipped competing SDK)

`INTERLATENT.md` reviews a real, deployed VR-teleoperation-for-data-collection
product — Quest headset → browser-side IK → WebTransport/QUIC → a robot arm, with
every episode recorded for imitation-learning training. It is not a fair
"better/worse" comparison — the two solve adjacent but different problems — but the
contrast is instructive:

| Dimension | Interlatent | RoboProtocol |
| --- | --- | --- |
| Target domain | Single/dual-arm manipulators, demonstration recording for policy training | Humanoids + high-DoF manipulators, full-body, balance-critical locomotion |
| Loss handling (control) | Send each datagram 2–3×, latest-seq-wins, no FEC | Inline FlexFEC + decay-based dead reckoning |
| Haptic feedback | **None** — one-way joint targets out | Core differentiator — bilateral wrench/contact-force feedback with TDPA |
| Safety model | Fixed 250 ms staleness cutoff, 4-source priority ladder | 6-tier latency state machine scaled per Task Class B–E, TDPA, WBC/ZMP override |
| Security | Not specified in the public post | Explicit TLS 1.3, X25519, mTLS Ed25519 (v0: P-256), HMAC anti-replay, scoped 0-RTT |
| Reference implementation | **Ships** — open-source SDK, real deployed product | Now also ships, for one hardware target (XGO-Lite V2) |

**Lessons already adopted or worth adopting further:**

1. Dial-out relay as an ICE/STUN/TURN fallback — adopted in `DESIGN.md` §1.4.
2. A fixed priority-ladder arbiter as one pure function of booleans — adopted, see
   [Safety & Control § Control source arbitration](03-safety-and-control.md#control-source-arbitration).
3. Velocity clamp integrated against the last *commanded* position, not measured
   position, to avoid a gravity-loaded-joint deadlock — adopted, see
   [Safety & Control § Kinematic clamping](03-safety-and-control.md#kinematic-clamping--tdpa-passivity).
4. Data provenance / `control_source` labeling per timestep — adopted as `FR-9.8`'s
   `ControlSource` recording field, though RoboProtocol still has no dataset/training
   story analogous to Interlatent's LeRobot export.

## Where this leaves the project

The safety intuitions — task-adaptive latency tiers, TDPA passivity, decay-based dead
reckoning, WBC override authority — remain the project's strongest, most differentiated
material, and none of it is offered by an existing robotics middleware today. The
original packaging critique ("implementable only by the original authors") is
materially less true than when `REVIEW.md` was written: the protocol now has a byte-
level wire format, a version/capability handshake, and a real reference
implementation running on physical hardware. What remains open before a wider
adoption push is largely about *reach*, not soundness: a second independent
implementation to prove real interoperability, a working ROS 2 bridge, a device
discovery/pairing flow, and — per `REVIEW.md`'s P3 — eventual standards-body
engagement if the goal is a vendor-neutral protocol rather than a single project's
reference stack.
