# RoboProtocol Documentation

RoboProtocol is a QUIC-based network protocol and safety architecture for real-time,
high-fidelity teleoperation of physical robots — from a single wheeled arm to a
full-body, dynamically-balanced humanoid. This directory is the documentation hub:
protocol specification, system architecture, and the XGO-Lite V2 reference
implementation that runs the protocol against real hardware.

If you're new here, read in this order:

| # | Document | What it covers |
|---|---|---|
| 1 | [Overview](01-overview.md) | Why teleoperation needs a purpose-built protocol; the five problems RoboProtocol exists to solve |
| 2 | [Protocol Architecture](02-protocol-architecture.md) | The layered stack, Channel A/B/C semantics, session handshake, NAT traversal |
| 3 | [Safety & Control](03-safety-and-control.md) | The task-adaptive safety state machine, control-source arbitration, TDPA, watchdogs |
| 4 | [Security](04-security.md) | TLS 1.3 / mTLS, key exchange, anti-replay, 0-RTT scoping |
| 5 | [Reference Implementation](05-reference-implementation.md) | The Rust workspace (`robot-edge`, `operator-console`, core crates) and how they map to the spec |
| 6 | [XGO-Lite V2 Guide](06-xgo-lite-guide.md) | Running the reference implementation against a real XGO-Lite V2 quadruped |
| 7 | [Recording & Replay](07-recording-and-replay.md) | Black-box session logging and the replay/conversion tooling |
| 8 | [Performance & Benchmarks](08-performance-and-benchmarks.md) | Real-hardware latency/throughput measurements and how to reproduce them |
| 9 | [Design Review & Roadmap](09-design-review-and-roadmap.md) | Known gaps, strengths, and the path from prototype to standard |

## Repository map

```text
RoboProtocol/
├── README.md              Project pitch and motivation
├── BENCHMARK.md            Network benchmarking runbook
├── LICENSE                  PolyForm Noncommercial License 1.0.0
├── docs/                    You are here — curated, browsable documentation
├── crates/                  Rust workspace: protocol + reference binaries
│   ├── roboprotocol-core      Transport-agnostic protocol logic (safety, HELLO, sizing, timestamps)
│   ├── roboprotocol-proto     FlatBuffers schema + generated bindings
│   ├── roboprotocol-recording Bounded-queue black-box recording pipeline
│   ├── robot-edge              QUIC server binary — runs on the robot
│   └── operator-console        QUIC client + TUI binary — runs on the operator's machine
├── xgo_bridge/               Python bridge from robot-edge to the XGO-Lite V2 vendor SDK
├── tools/                    dev-certs (mTLS cert generation), replay-decode (recording → CSV/MP4)
├── simulator/                 Python network/protocol simulator for scenario testing
├── benchmark/                 pcap analysis + raw UDP baseline scripts
└── certs/                     Committed dev-only mTLS certificates
```

## Conventions used in these docs

- **Spec citations** like `DESIGN.md §2.2` or `REQUIREMENTS.md FR-1.7` point at
  numbered sections/requirements in Spinworks' internal source-of-truth documents,
  which are not part of this public repository. These docs summarize and organize
  that material for browsing here.
- Code paths are relative to the repository root, e.g. `crates/robot-edge/src/quic_server.rs`.
