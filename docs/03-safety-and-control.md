# Safety & Control

Source of truth: `DESIGN.md` §3, `REQUIREMENTS.md` §5 (Safety Requirements). This page
organizes that material around the two questions a safety-critical teleop system has
to answer every control tick: *who is allowed to command the robot right now*, and
*is what they're commanding actually safe to execute*.

## Control source arbitration

Exactly one control source is admitted per tick, resolved top-down by a single
fixed-priority decision function (`arbitrate()` in spec terms; `safety_task.rs` /
`roboprotocol-core::safety::arbitration` in the reference implementation):

```text
+------------------------------------------------------------------------------------------------+
|                              CONTROL SOURCE ARBITRATION LADDER                                   |
+------------------------------------------------------------------------------------------------+
| Priority | Source                       | Admitted when...                                      |
+----------+-------------------------------+-------------------------------------------------------+
| 1        | E-Stop                        | Watchdog blackout or explicit E-Stop signal. Latched, |
|          |                                | level-triggered — re-read every tick, cleared only    |
|          |                                | by explicit human act.                                |
| 2        | Emergency Safe Parking         | Explicit request, or the operator-interlock resume    |
|          |                                | did not complete within a configurable dwell time      |
|          |                                | after entering SUSPENDED.                              |
| 3        | Active Impedance Hold          | Tier 4 SUSPENDED, until the operator interlock + 2s   |
|          |                                | stability window completes.                            |
| 4        | Full Teleoperation             | Operator deadman held, latest command fresh, measured  |
|          |                                | latency within Tier 0-3 for the active Task Class.     |
| 5        | Semi-Autonomous                | Full Teleoperation not engaged, an autonomy goal is    |
|          |                                | asserted via Channel C RPC.                            |
+------------------------------------------------------------------------------------------------+
```

The **WBC/ZMP Balance Override** is deliberately *not* a rung on this ladder — it's a
cross-cutting safety layer applied to whichever source wins arbitration, the same way
§ below layers workspace → velocity → passivity clamps. This keeps "who's allowed to
command this tick" and "is that command safe to execute" as two separate, composable
concerns. The layer is present in the arbitration path for every robot regardless of
morphology; it only does anything when `base_type = BipedLegs` — see
[Protocol Architecture § SESSION_DESCRIBE](02-protocol-architecture.md#session_describe--session_accept--robot--media-profile).

Every recorded/telemetered tick carries the arbitrated source, so post-hoc review can
tell which layer actually drove the robot at any moment (`FR-9.8`).

## Task-adaptive safety state machine

Human sensorimotor research gives three well-established failure regimes under network
delay:

- **Just-Noticeable Difference (JND):** haptic phase-lag becomes perceptible at
  ~15–20 ms; visual latency at ~45–50 ms.
- **Move-and-Wait (Poulton's Law):** past ~100–150 ms, operators abandon continuous
  closed-loop control for intermittent "move-and-wait" sub-movements — completion time
  up 300–500%, collisions way up.
- **Pilot-Induced Oscillations (PIO):** in high-frequency balance/steering loops
  (>150 ms delay), corrective inputs phase-lag the system into destructive resonance.

Because the latency at which these regimes kick in varies by task, thresholds are
scaled per **Task Class**:

| Tier Level | Class B (Fine Tools) | Class C (Coarse Pick/Place) | Class D (Indoor Mobile-Base) | Class E (Outdoor Locomotion) |
| --- | --- | --- | --- | --- |
| Tier 0: NOMINAL | < 40 ms | < 80 ms | < 100 ms | < 200 ms |
| Tier 1: DEGRADED | 40–80 ms | 80–150 ms | 100–250 ms | 200–400 ms |
| Tier 2: CAUTIONARY | 80–120 ms | 150–220 ms | 250–400 ms | 400–700 ms |
| Tier 3: CRITICAL | 120–150 ms | 220–300 ms | 400–500 ms | 700–1000 ms |
| Tier 4: SUSPENDED | ≥ 150 ms | ≥ 300 ms | ≥ 500 ms | ≥ 1000 ms |
| Tier 5: E-STOP | Blackout > 200 ms | Blackout > 300 ms | Blackout > 400 ms | Blackout > 500 ms |

Class D was renamed from "Indoor Humanoid Locomotion" to "Indoor Mobile-Base
Locomotion" — the thresholds describe an indoor-tight-space *task* and are
morphology-agnostic; balance-critical behavior is gated separately by `base_type`
(see above), not implied by which Task Class is active.

### Automated suspension (Tier 4)

1. **Input isolation** — operator master inputs are instantly decoupled from robot
   joint controllers.
2. **Kinematic deceleration** — end-effector/joint targets ramp to 0 rad/s via smooth
   S-curve velocity profiles.
3. **Active impedance holding** — actuators switch to local active impedance holding
   / passive gravity compensation.
4. **Operator interlock & resume handshake** — re-engaging teleop requires an explicit
   deadman reset **and** a 2-second continuous stability window where latency stays
   below the nominal limit for the active Task Class (20 ms hysteresis).

### Watchdog & E-Stop

- **Hardware watchdog:** timeout = `max(Tier 5 blackout floor, K × control period)`,
  `K = 3`. At the assumed 100 Hz–1 kHz range this reduces to the fixed thresholds
  above unchanged; at a slow operator-chosen rate (e.g. 10 Hz) it prevents a single
  missed legitimate command from false-tripping the watchdog purely because of the
  configured rate rather than an actual fault.
- **E-Stop transport:** a dedicated, redundant path — reliable stream *and* 1 kHz
  unreliable datagrams — for sub-5 ms local edge processing, completely bypassing
  reliable-transport head-of-line blocking. This 1 kHz rate is fixed, independent of
  whatever control rate the operator configured.

The reference implementation's `scripts/smoke_test.sh` exercises this end-to-end: it
kills `operator-console` mid-session and asserts `robot-edge`'s log shows `E-Stop
latched` within the Class D 400 ms threshold (allowing generous scheduling margin).

## Kinematic clamping & TDPA passivity

**Dynamic velocity clamping** scales the maximum allowable Cartesian velocity with
measured latency τ:

```
V_safe(τ) = V_max · max(0, 1 − (τ − τ_nominal) / (τ_cutoff − τ_nominal))
```

The per-tick step this bounds is integrated against the **last commanded target, not
the last measured position** — clamping against measured position would let a joint
under sustained external load (e.g. gravity on an unpowered axis) stall indefinitely,
since both commanded and actual would sit near-motionless and the delta would never
exceed the clamp. Integrating from the last commanded target keeps the command
advancing at the full clamped rate regardless of how far actual position has lagged,
turning a lagging joint into a torque/tuning problem for the low-level controller, not
a protocol-level deadlock.

**Passivity bounding (TDPA)** monitors network energy flow, `E(t) = ∫ F·v dt`. If
delay introduces virtual energy (`E(t) < 0`), an adaptive temporal dissipation
observer attenuates force feedback to preserve passivity. The energy-integral metadata
rides directly in Channel B datagram headers — not a separate stream, and not
something bolted on at the application layer after the fact, which is what makes it
standardizable rather than vendor-specific.

## WBC / ZMP balance override

Applies only when `base_type = BipedLegs` — the one morphology in the taxonomy that's
dynamically balanced and has a real Center-of-Mass/support-polygon fall risk:

- **Low-level WBC precedence:** the robot's on-board Whole-Body Controller and ZMP
  estimator have absolute override authority over operator commands.
- **CoM stability clamping:** if operator motion would push the estimated CoM outside
  the dynamic support polygon, the local controller clamps end-effector velocity and
  triggers automatic stepping balance adjustments.

For `WheeledStandard`, `WheeledHolonomic`, `QuadrupedLegs`, and `Stationary` robots
this is a documented no-op — those morphologies are statically stable (or immobile)
and have nothing for the override to guard against. The reference XGO-Lite V2 is
`QuadrupedLegs`, so this layer is present in its arbitration path but inert.

## Latency measurement

Every Channel B datagram embeds a 64-bit timestamp. The receiver echoes it to compute
sliding-window RTT/OWD, sampled at whatever rate Channel B itself runs (not a fixed
1 kHz probe). PTP (IEEE 1588v2) hardware timestamping is used on local LANs where
available; the protocol falls back to RTT thresholds and software drift estimation
over public WANs/5G where hardware timestamping isn't available.
