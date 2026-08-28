# RoboProtocol Simulator — Design & Traceability

A packet-level discrete-event simulator of RoboProtocol's Channel A/B/C
behaviour over a "standard consumer network connection," built to show
how the protocol specified in `../REQUIREMENTS.md` and `../DESIGN.md`
actually behaves under realistic conditions and injected network
failures. It is not a physical robot / balance / kinematics simulator
— see "Scope boundary" below.

This document is the spec for the simulator itself: what it models,
what it deliberately simplifies, what it assumes where the source
documents don't pin a number, and what it found along the way.

## Scope boundary

In scope: packet timing, sizing, quantization, loss, queueing,
FEC recovery, the safety-tier state machine, the hardware watchdog,
and a TDPA passivity proxy — i.e. everything needed to evaluate the
*protocol's* behaviour.

Out of scope: robot dynamics (WBC/ZMP/CoM balance), actual video
encode/decode, ROS2/DDS bridging, kinematic retargeting/collision.
Video streams are modelled as datagram-generating byte-rate processes,
not pixels.

## Scenarios

Four progressive scenarios (`roboprotocol_sim/scenarios/definitions.py`),
each adding one axis of complexity on top of the last, per the
"then X, then Y, then Z, then W" framing of the original request:

1. **simple_arm** — single arm, Channel B only, no video.
2. **arm_wrist_cam** — (1) + one wrist camera on Channel A.
3. **arm_wrist_fixed_cam** — (2) + a fixed/head stereo camera.
4. **dual_arm** — both arms, *and* both scenario-3 cameras doubled up
   for the wrist (left+right) plus the shared head camera. This is a
   documented assumption (`DUAL_ARM_INCLUDES_CAMERAS` in
   `definitions.py`) rather than a control-only comparison point —
   flip that flag for the alternative reading if wanted.

DoF/field-count model (REQUIREMENTS.md doesn't pin an exact DoF count
for "an arm"):

* One arm = 7-DoF anthropomorphic arm + 1-DoF parallel gripper = 8
  actuated DoF.
* Channel B **command** fields = DoF × 3 (position, velocity, torque).
* Channel B **telemetry** fields = one 6-axis body/wrist IMU + DoF × 3
  + battery voltage/temperature (2 fields). No foot force-torque
  sensors — these are seated/stationary arm-manipulation scenarios,
  not legged locomotion, so **Task Class B** (Fine Tool & Utensil
  Manipulation) is the default active class for all four (configurable
  per scenario).
* Channel B **haptic** fields = wrist wrench (Fx,Fy,Fz,Tx,Ty,Tz) + 2
  simplified per-gripper contact-force scalars = 8 fields per arm.

## Failure injections

`roboprotocol_sim/scenarios/failure_presets.py`, mapped to
REQUIREMENTS.md §2's failure-mode table and the specific modes
requested:

| Preset | Models |
|---|---|
| `none` | Baseline, profile noise only |
| `bandwidth_drop` | Sustained congestion (~75% uplink cut) |
| `blackout` | Sudden total connection drop (§2 "Wireless Blackouts") |
| `packet_loss_20` / `packet_loss_40` | Uniform burst loss either side of NFR-3.1's 30% FlexFEC boundary |
| `jitter_spike` | Jitter blown past NFR-1.3's 15ms tolerance |
| `latency_ramp` | Gradual congestion buildup, held at peak — walks Tiers 0→4 |
| `latency_spike_recover` | Triangular spike past SUSPENDED and back — the only preset that exercises SR-3.3's resume path (see "Findings" below) |
| `handover` | Wi-Fi↔5G path switch (DESIGN.md §6 QUIC Connection Migration) |
| `compound_congestion` | Realistic congested-cellular mix: bandwidth drop + loss burst together |

`handover` deliberately implements DESIGN.md §6's actual mechanism
(reactive path validation + switch) rather than NFR-3.2's literal
"concurrent dual-interface bonding" wording — those two clauses are in
tension (a previously-flagged spec ambiguity); §6 is the one with a
concrete algorithm, so that's what's modelled.

## Network profiles

`roboprotocol_sim/network/profiles.py`. REQUIREMENTS.md only pins
NFR-1.2's RTT targets (<30ms Wi-Fi, <60ms 5G) and NFR-2.1's 2-20Mbps
video envelope — everything else is a documented assumption for a WAN
path between operator and robot:

| Profile | One-way OWD | Jitter σ | Baseline loss | Robot uplink | Operator uplink |
|---|---|---|---|---|---|
| `home_broadband_wifi6` | 12ms | 3ms | 0.1% | 20Mbps | 20Mbps |
| `cellular_5g` | 20ms | 8ms | 0.5% | 30Mbps | 15Mbps |

Bandwidth is asymmetric/upload-constrained on the robot's side since
it's the one uploading the larger flow (video).

## Byte-budget model (DESIGN.md §2.2.1)

Implemented in `roboprotocol_sim/protocol/sizing.py`, precisely:

* Single-datagram **application-payload** budget: 1200 bytes. This
  covers everything inside the QUIC DATAGRAM frame the application
  defines: quantized fields + 8B NTP timestamp + 8B sequence number +
  32B HMAC-SHA256 + 12B FlatBuffers framing + (haptic-carrying
  datagrams only) an 8B TDPA energy scalar.
* Quantization tiers: Full=float32 (4B/field), Standard=int16 (2B),
  Compact=int8 (1B) — chosen per payload category.
* Haptic fields are hard-floored at Standard (`HapticTierViolation` is
  raised, not just discouraged, if a caller tries Compact on a
  haptic-carrying payload) — FR-3.1.
* If a category still exceeds budget at its minimum tier, it's split
  into multiple same-tick datagrams by (synthetic) body region rather
  than fragmented — RFC 9221 datagrams have no fragmentation
  primitive. None of the four named scenarios ever trigger this (their
  DoF counts fit comfortably even at Full tier); `test_sizing.py`
  exercises it directly with a synthetic 400-field stress case.
* Wire-level overhead — IPv4 (20B) + UDP (8B) + QUIC short header
  (~9B) + AES-256-GCM tag (16B) = 53B — sits *outside* the 1200B
  budget and is added by `wire_bytes()` only for link/bandwidth
  accounting, matching DESIGN.md's own ~150-250B headroom framing
  between the 1200B budget and the ~1350-1452B path-MTU-safe ceiling.

## Safety tier state machine & watchdog (REQUIREMENTS.md §5, SR-2/3/4)

`roboprotocol_sim/protocol/safety_state_machine.py` implements two
genuinely independent, concurrently-evaluated axes — this distinction
is load-bearing for how the simulator behaves, not just a modeling
nicety:

* **Tier 0-4**, driven by RTT (SR-3.1). Channel B's RTT sample is
  approximated as this tick's command OWD (operator→robot) plus this
  tick's telemetry OWD (robot→operator) — what an explicit
  timestamp-echo protocol converges to in expectation, without
  implementing one. If either leg is lost, that tick contributes no
  sample. Crossing Tier 4 (SUSPENDED) requires an explicit deadman
  reset *and* a continuous 2s window below `tier0_max - 20ms hysteresis`
  to resume (SR-3.3).
* **Tier 5 / hardware watchdog**, driven purely by heartbeat-silence
  duration (SR-4.1, using the corrected per-class blackout thresholds:
  200/300/400/500ms for B/C/D/E), independent of the RTT state
  machine. This is deliberate: during a total blackout there is no
  fresh RTT data for Tier 0-4 to react to, which is exactly why SR-4.1
  exists as a separate mechanism. The watchdog's heartbeat is fed by
  any successfully-delivered operator→robot Channel B datagram,
  standing in for SR-4.2's dedicated redundant 1kHz E-Stop stream
  (both exist purely to detect loss of contact; modelling them as
  literally separate flows wouldn't change observable behaviour here).
  Once triggered, the watchdog latches — real E-Stop recovery is an
  out-of-band operational procedure, not a network-driven transition,
  so it isn't auto-reset by the simulator.

## TDPA passivity proxy (DESIGN.md §3.4, FR-3.1/3.2)

`roboprotocol_sim/protocol/tdpa.py`. Not a research-grade
implementation — a small, deterministic model chosen because it
reproduces the textbook mechanism by which delay destabilizes force
feedback: a synthetic operator velocity `v(t) = v0·sin(2πft)` paired
against a robot-side damper's force `F(t) = -k·v(t)` that the operator
only receives after the network's *current* one-way delay. Once delay
is comparable to the motion's own period, `F_received(t)·v(t)` goes
periodically positive — virtual energy injection — which is exactly
what FR-3.2's passivity observer exists to catch. Cumulative energy
`E(t)` is tracked; `E(t) > tolerance` is logged as a violation. A
sub-floor-quantization run (never reachable through the four named
scenarios, since FR-3.1's floor is hard-enforced) adds a documented
force-error bias to demonstrate why the floor matters.

## FlexFEC proxy (NFR-3.1)

`roboprotocol_sim/protocol/fec.py`. A Reed-Solomon-style erasure-code
proxy, not bit-accurate: a block of `block_size=10` video datagrams
gets `n_redundant=3` (30%) parity datagrams; the block recovers iff
losses ≤ 3. Good enough to demonstrate NFR-3.1's claimed "recovers up
to 30% loss" boundary under injected loss.

## Channel A / GCC-like bitrate controller (FR-2.1, NFR-1.1, NFR-2.1)

`roboprotocol_sim/protocol/channel_a.py`. PIR is modelled as
near-constant per-frame byte rate (no periodic I-frame spikes) rather
than encoding real pixels. The adaptive controller is fed the
*actually observed* end-to-end delay of delivered datagrams (arrival
minus send time — i.e. including queueing delay from a bandwidth-cap
failure), not the network profile's propagation-only OWD parameter;
see "Findings" below for why that distinction mattered. `target_bps`
represents total wire-equivalent throughput budget (data + FlexFEC
parity), so the frame-data generation rate is backed off by
`1/(1+redundancy_ratio)` to keep actual wire usage within the
controller's own adapted cap.

## Naive WebSocket/TCP baseline comparison

`roboprotocol_sim/network/naive_link.py`, `roboprotocol_sim/protocol/naive_session.py`,
`roboprotocol_sim/metrics/naive_collector.py`. Every scenario x failure
combination can optionally also be replayed over a second, deliberately
naive transport model, so the report shows not just "RoboProtocol meets
its own targets" but "here is what the same network conditions do to
the kind of from-scratch `ws://` teleop stack most projects reach for
first." Enabled with `--compare-naive` on `run` or `sweep`.

The naive model carries the *same* traffic (same per-scenario DoF/video
config) over a *single ordered, reliable, unprioritized stream* — one
TCP-like connection multiplexing commands, telemetry, haptic, and video
together — instead of RoboProtocol's independent QUIC datagrams/channels:

* **Full float32 fields, no quantization tiers, no anti-replay framing.**
  A naive implementation just serializes whatever the message is (16
  bytes of minimal length-prefixed struct framing, no NTP timestamp/
  sequence number/MAC) rather than applying DESIGN.md §2.2.1's tiers.
* **Fixed video bitrate.** No FlexFEC, no GCC-like adaptation — the
  encoder just runs at its configured rate regardless of link state.
* **No priority.** Commands, telemetry, haptic, and video share one
  FIFO in send order — there is no DSCP-EF-equivalent preemption, so a
  large video segment ahead of a control command delays the command by
  the video segment's full serialization time.
* **Pipelined transmission, in-order delivery.** Segments are put on
  the wire back-to-back at the bandwidth-limited service rate (real
  TCP doesn't stall sending on every segment's propagation delay), but
  delivery to the application is strictly in order: a segment that
  itself crosses the wire cleanly is still withheld from the
  application until every segment queued ahead of it has *also*
  arrived — true head-of-line blocking, modelled as a small in-order
  release gate (`NaiveReliableLink._release_in_order`), the same
  mechanism a real TCP receive buffer implements. QUIC's independent
  per-datagram framing on Channel A/B is specifically what RoboProtocol
  gets out of having to do this.
* **Retransmission via a simplified RTO**: 4x the currently-observed
  RTT, floored at 200ms, doubling per consecutive loss of the same
  segment, capped at 3s — the same qualitative backoff shape real TCP
  stacks use, not a literal RFC 6298 implementation.
* **No dedicated watchdog.** Liveness is inferred from a coarse
  periodic check (`LIVENESS_CHECK_INTERVAL_S=5s`, `LIVENESS_TIMEOUT_S=10s`
  silence threshold) standing in for a typical WebSocket client's
  ping/pong reconnect policy — two to three orders of magnitude slower
  than RoboProtocol's dedicated hardware watchdog (SR-4.1,
  200-500ms). For the standard `blackout` preset (2.5s), this
  threshold is *never crossed* — the naive stack doesn't even notice
  anything went wrong; commands just resume flowing once the link
  recovers, with no safety trigger of any kind. That silent
  degradation, not just "it's slower," is the headline finding.
* **No QUIC Connection Migration.** On the `handover` preset only, the
  naive run's own `FailureTimeline` gets an extra blackout window
  (`NAIVE_RECONNECT_COST_S=1.5s`) layered on top of whatever gap the
  QUIC side experiences, representing the cost of detecting a dead TCP
  socket and paying a fresh TCP handshake + WebSocket upgrade — work
  QUIC's Connection-ID-based migration (DESIGN.md §6) avoids entirely.

The comparison runs as a fully separate simulation (own `Scheduler`,
own `FailureTimeline` built from the identical failure preset) rather
than sharing RNG draws lockstep with the QUIC run — the two are
statistically comparable under the same injected conditions, not
bit-for-bit replays of each other. `metrics/report.py` adds a
`naive_comparison.png` plot (command delivery delay and video segment
delay, both log-scaled, RoboProtocol vs naive, with watchdog/liveness
trigger points marked) and a comparison table to `summary.md` when
`--compare-naive` is set.

## Requirement verification checks

`roboprotocol_sim/metrics/nfr_checks.py` mechanically checks 7
REQUIREMENTS.md clauses per run and reports PASS/FAIL/N/A (never a
forced boolean — e.g. a watchdog-timing check is meaningless on a run
that never triggered it). This is a concrete down-payment on
REQUIREMENTS.md §7's own sparse Requirement Verification Matrix, for
the subset a network/protocol simulation can actually adjudicate:
NFR-1.2, NFR-1.3, NFR-2.1, NFR-3.1, SR-4.1, SR-3.3, FR-3.1.

Nominal-condition checks (NFR-1.2, NFR-1.3) are evaluated only up to
the first injected-failure event — those NFRs are steady-state claims,
and a deliberately-injected blackout blowing through the RTT target is
the point of running it, not a defect.

## Findings surfaced while building this

Several of these came from the simulator behaving unexpectedly during
development and turned out to be real gaps, not simulator noise —
worth carrying back into the protocol spec, not just this codebase:

1. **GCC controller was blind to queueing delay.** Originally fed from
   the network profile's propagation-only OWD, it never reacted to a
   `bandwidth_drop` failure at all, because propagation delay doesn't
   move when the link's *service rate* collapses — only queueing delay
   does. Fixed by feeding it real observed (queueing-inclusive) delay.
   Any real GCC-style implementation needs this too.
2. **Bitrate target didn't account for FlexFEC overhead.** The
   controller adapted a "target bitrate" that governed encoder output
   only; FEC parity datagrams added another ~30% on top at the wire,
   so actual link usage could run ~30% over the controller's own
   adapted cap — silently defeating the congestion response it exists
   to provide. Fixed by treating `target_bps` as the wire-inclusive
   budget.
3. **SR-3.3's resume path was unreachable in every failure preset.**
   A held latency ramp never comes back down, and a total blackout
   gives the RTT-driven state machine no samples to resume from at
   all (by design — that's the watchdog's job). Added a triangular
   `latency_spike_recover` preset specifically to exercise it.
4. **SR-3.3's 20ms hysteresis margin is tight relative to realistic
   WAN jitter.** On the `home_broadband_wifi6` profile, Class B's
   nominal ceiling (40ms) minus the 20ms hysteresis leaves only a
   20ms resume band — comparable to or smaller than the profile's own
   baseline RTT jitter (~24ms mean, ~4ms σ per leg). In several runs
   the state machine reaches SUSPENDED correctly but the resume
   condition is never satisfied within a realistic run length. This
   may be intentional (a deliberately strict anti-flapping bar) or may
   warrant a second look at the 20ms figure relative to real consumer
   network jitter — the simulator can't tell you which, but it can
   show you the number.
5. **NFR-1.2 is marginal even at baseline on `home_broadband_wifi6`.**
   p95 RTT sits right around the 30ms target even with *no* failure
   injected, given the profile's 12ms one-way propagation assumption
   plus jitter. This is a consequence of the profile's own documented
   assumptions above, not a bug — but it's a useful sanity check on
   how much margin NFR-1.2 actually has in practice.
6. **A per-stream video bitrate floor initially violated NFR-2.1.**
   The wrist camera's `min_bps` was set to 1Mbps, below NFR-2.1's
   stated 2Mbps floor — the simulator's own scenario config
   miscalibrated a lower bound the requirement doesn't allow. Fixed to
   2Mbps.

## Verification (how to run this)

```bash
cd simulator
pip install -r requirements.txt   # numpy, pandas, matplotlib, pytest
pytest -q                          # unit + smoke tests
python -m roboprotocol_sim list-scenarios
python -m roboprotocol_sim list-failures
python -m roboprotocol_sim run --scenario dual_arm --failure bandwidth_drop --network cellular_5g
python -m roboprotocol_sim run --scenario dual_arm --failure blackout --compare-naive
python -m roboprotocol_sim sweep --compare-naive   # every scenario x every failure, + output/comparison.md
```

Each run writes CSVs, PNG plots (RTT/tier timeline, bandwidth vs
capacity, video bitrate adaptation, TDPA energy), and a `summary.md`
with the NFR check table to `output/<scenario>__<failure>__<network>/`.
With `--compare-naive`, it additionally writes `naive_segments.csv`,
`naive_liveness.csv`, `naive_comparison.png`, and a comparison table
appended to `summary.md`.
