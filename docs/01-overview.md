# Overview

RoboProtocol is a specialized network protocol and safety architecture for real-time,
high-fidelity teleoperation of physical systems — particularly humanoid robots and
high degree-of-freedom (DoF) manipulators. It exists because general-purpose internet
protocols (TCP, WebSockets, gRPC, standard WebRTC) were designed for static data
transfer, web applications, or human-to-human media — not for the physics, safety
envelopes, and control loops of a physical robot on the other end of the wire.

## Why not just use an existing protocol?

Five problems recur in real teleoperation deployments that off-the-shelf protocols
don't address:

### 1. Head-of-line (HoL) blocking

TCP and standard WebSockets prioritize guaranteed delivery over timeliness: if a
single packet drops, the network stack holds up every subsequent packet until the
drop is retransmitted. In a 100 Hz–1 kHz control loop, a 20 ms stall causes command
starvation — actuators see sudden torque drops or velocity jumps, which is a
mechanical-instability risk, not just a UX annoyance.

**RoboProtocol's answer:** media and control traffic ride unreliable, multiplexed
QUIC datagrams (RFC 9221). A stale control packet is dropped instantly rather than
blocking the queue; a lost packet triggers local decay-based dead reckoning instead
of waiting for a retransmit.

### 2. Bilateral haptic passivity

Force-feedback teleoperation is a closed loop: network delay introduces phase lag,
and phase-lagged force feedback injects virtual energy into that loop. The result is
a non-passive master–slave system — violent, uncontrollable oscillation that can
break the robot or hurt the operator.

**RoboProtocol's answer:** the Time-Domain Passivity Approach (TDPA) is built into
the Channel B datagram headers themselves. The transport layer continuously monitors
energy flow (`E(t) = ∫ F·v dt`) and attenuates haptic feedback the instant virtual
energy is detected — see [Safety & Control](03-safety-and-control.md).

### 3. Human perception is task-dependent

The latency at which a task stops being performable varies by an order of magnitude
depending on what the operator is doing. A 1.0 s suspension threshold is fine for
gross outdoor walking and catastrophically late for fine tool manipulation, which
needs suspension by ~150 ms.

**RoboProtocol's answer:** a Task-Adaptive Safety State Machine (Task Classes B
through E) scales warnings, velocity clamping, and suspension thresholds to the
active task — see [Safety & Control §Task-Adaptive Thresholds](03-safety-and-control.md#task-adaptive-safety-state-machine).

### 4. Dynamic IP mobility & handover

Mobile robots switch between cellular and Wi-Fi constantly. A handoff changes the
robot's IP address; a session identified only by the IP/port 4-tuple drops and forces
a multi-RTT cryptographic re-handshake mid-task.

**RoboProtocol's answer:** QUIC Connection IDs (RFC 9000 §19) let the secure session
persist across a network handover via QUIC Connection Migration, with no
re-handshake and no control interruption.

### 5. Kinematic & balance override authority

Standard protocols treat every payload equally and have no notion of the robot's
physical state. A command that violates a joint limit or pushes the robot's center
of mass outside its support polygon should never simply reach the actuators.

**RoboProtocol's answer:** the robot edge controller enforces low-level Whole-Body
Controller (WBC) override authority — incoming trajectory commands are clamped or
overridden by balance-recovery routines before they reach the joints, on any robot
whose base morphology is dynamically balanced.

## What "the protocol" actually is

RoboProtocol is QUIC (RFC 9000) plus unreliable datagrams (RFC 9221), carrying three
purpose-built logical channels over one connection:

- **Channel A** — video, as unreliable datagrams with inline FlexFEC.
- **Channel B** — command, telemetry, and haptic force feedback, as unreliable
  datagrams with embedded timestamps and TDPA metadata.
- **Channel C** — reliable QUIC streams for session setup, RPCs, ROS 2 tunneling,
  and a redundant high-priority E-Stop path.

See [Protocol Architecture](02-protocol-architecture.md) for the full breakdown.

## What exists today

This repository contains both the specification (`DESIGN.md`, `REQUIREMENTS.md`) and
a working reference implementation: a Rust QUIC server/client pair (`robot-edge` /
`operator-console`) plus a Python bridge (`xgo_bridge`) that drives a real XGO-Lite V2
quadruped over the protocol. See [Reference Implementation](05-reference-implementation.md)
and the [XGO-Lite V2 Guide](06-xgo-lite-guide.md) to run it yourself.

`REVIEW.md` — condensed in [Design Review & Roadmap](09-design-review-and-roadmap.md) —
is a candid internal critique of what's still missing for third-party interoperability
and consumer adoption; it's worth reading alongside the spec, not instead of it.
