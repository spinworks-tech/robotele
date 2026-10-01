# RoboProtocol network benchmark

How to measure latency, throughput, and wifi-resilience of Channel B (the
QUIC/mTLS teleop + telemetry link between `operator-console` and
`robot-edge`; see [DESIGN.md](DESIGN.md) §2.2, §1.3.3) with Wireshark, and
how that compares against a plaintext UDP baseline of the same size/rate.

## What we're testing

1. **Plain vs encrypted** — how much does QUIC + TLS 1.3 (mTLS, `quiche`)
   cost over the same payload, same rate, same network path?
2. **Overhead vs raw data** — for the real Channel B traffic, what
   fraction of each wire packet is application payload (the FlatBuffers
   `ChannelBFrame`, packing an 18-byte `TeleopCommand` or a
   `TelemetryData` record) vs UDP/IP/QUIC/TLS framing?
3. **Wifi drop behavior** — when the radio link disappears mid-session,
   how long until the last frame gets through, how does QUIC react
   (retransmits, idle timeout), and does the Class D watchdog actually
   latch E-Stop within its 400ms budget (see `scripts/smoke_test.sh` for
   the same check done locally over loopback)?

Everything here measures the real `robot-edge`/`operator-console`
binaries and real traffic — not a synthetic protocol. The one synthetic
piece is the plaintext baseline in part 1, which exists solely to isolate
QUIC+TLS's wire cost.

## Comparing against other protocols (MQTT / Zenoh / DDS / Kafka)

If you're benchmarking Channel B against other pub/sub or messaging
protocols (e.g. reproducing the methodology from
[zenoh.io's Zenoh-vs-MQTT-vs-Kafka-vs-DDS post](https://zenoh.io/blog/2023-03-21-zenoh-vs-mqtt-kafka-dds/)),
two things need to be nailed down first or the numbers won't be
comparable to anyone's, including our own past runs.

### Encryption can't be turned off on our side

That post's benchmarks run every protocol unencrypted. MQTT, DDS, and
Zenoh can all do that — TLS is optional and bolted on. **QUIC cannot.**
Encryption (TLS 1.3, including header protection on every packet) is
mandatory in the QUIC wire format per RFC 9001 — there's no cleartext
QUIC mode to fall back to, so "disable encryption on Channel B" isn't a
real option.

Practical consequence: don't build one table that pretends Channel B has
an apples-to-apples unencrypted mode. Instead run **both** an unencrypted
and a TLS-enabled variant of MQTT and Zenoh, and report three things,
clearly labeled:

1. **Unencrypted protocol-to-protocol numbers** — MQTT-no-TLS vs
   Zenoh-no-TLS vs raw UDP as the protocol-free floor. Channel B has no
   entry here — there is no unencrypted QUIC to put in this row, and
   leaving it out (rather than faking one) is the honest move.
2. **Encrypted protocol-to-protocol numbers** — MQTT-with-TLS
   (standard TLS 1.2/1.3 over TCP) vs Zenoh-with-TLS vs Channel B
   (QUIC+mTLS, encryption mandatory). **This is the fair comparison**,
   since every entry now pays a real encryption cost, and it's the one
   worth leading with: QUIC's 1-RTT (0-RTT on resumption) handshake and
   single-roundtrip integration of transport+crypto is exactly the kind
   of structural advantage that shows up when TLS is mandatory for
   everyone, vs. MQTT/Zenoh's separate TCP handshake + TLS handshake on
   top. Use mTLS on the MQTT/Zenoh side too (client cert auth) if their
   client libraries support it, since Channel B's mTLS does client auth
   as well as encryption — an MQTT run with server-only TLS is a weaker
   comparison than one with mutual auth on both sides.
3. **Isolated encryption tax** — reuse Part 1/2 below (QUIC+TLS real
   traffic vs a plaintext UDP run at the same size/rate) to quantify
   exactly what Channel B's mandatory encryption costs in µs and bytes on
   its own, independent of any other protocol, so table 2's result can be
   explained rather than just asserted.

Table 1 exists for completeness/context (it's what the original Zenoh
post published), but table 2 is the one that actually answers "is our
protocol competitive," since it's the only row-for-row comparison where
nobody gets an unencrypted advantage.

### Hardware/topology has to match the other side's, not our deployment

The Zenoh post's benchmarks run on generic benchmarking machines — a
single-machine loopback case, and a multi-machine case connected by
**100 Gb Ethernet**. That's a controlled, high-headroom environment meant
to isolate protocol overhead, not a real embedded deployment.

Our real deployment is `robot-edge` on a resource-constrained Raspberry
Pi talking over wifi (see Part 3 below) — and that's the right topology
for *wifi-resilience* testing, but it is **not** a fair topology for a
protocol-to-protocol throughput/latency comparison: a Pi's CPU and wifi's
jitter/loss would dominate the numbers and make Channel B look worse for
reasons that have nothing to do with the protocol. That wouldn't be a
fair comparison, it'd be a Pi-vs-server comparison wearing a protocol
label.

So for the protocol-comparison suite specifically:

- Run the real `robot-edge`/`operator-console` binaries (not a
  synthetic protocol), but on the **same class of machine** used for the
  other protocols under test — e.g. two dev boxes/VMs on the same wired
  LAN, or same-machine loopback — not the Pi.
- If you *also* want an on-robot number for context, report it
  separately and labeled as "real deployment (Pi + wifi)", never merged
  into the same table as the controlled comparison — the two answer
  different questions ("how does our protocol compare to X" vs "how does
  our protocol perform on our actual hardware").
- Keep Part 3's wifi-drop test as-is; it's deliberately testing the Pi +
  wifi path and isn't meant to be compared against other protocols at
  all.

## Tools needed

| Tool | Purpose | Install |
|---|---|---|
| `cargo` / Rust workspace | build `robot-edge`, `operator-console`, `dev-certs` | already required by this repo |
| `tshark` (Wireshark ≥3.4) | packet capture + QUIC dissection | `sudo apt install tshark` |
| `python3` (stdlib only) | runs parts 1-3's `benchmark/*.py` — no pip installs needed | usually preinstalled |
| `sudo ip link` / `nmcli` / `rfkill` | simulate a wifi drop | preinstalled on most Linux |
| `chrony` or `ntpd` (optional) | clock sync, only needed if you want cross-host one-way latency, not just RTT/throughput | `sudo apt install chrony` |
| `mosquitto` | MQTT broker for part 4 | `sudo apt install mosquitto` |
| `paho-mqtt` (pip) | MQTT client, used by `benchmark/mqtt_bench.py` | `pip install paho-mqtt` |
| `eclipse-zenoh` (pip) | Zenoh client, used by `benchmark/zenoh_bench.py` — pin to major version 1, matching the `zenoh` Rust crate this repo already depends on for Channel C | `pip install "eclipse-zenoh>=1,<2"` |
| `iperf3` (optional) | independent throughput ceiling to cross-check part 4 against | `sudo apt install iperf3` |

No new Rust crate or Cargo dependency is needed — `quiche`'s QUIC framing
is visible in Wireshark's dissector without decryption (packet numbers,
ACK frames, PTO/idle-timeout events are all in the unencrypted QUIC
header), which is enough for latency/throughput/loss analysis even though
this repo doesn't wire up a TLS keylog file for full payload decryption.

## Topology

Run `robot-edge` on the real robot Pi (`pi@192.168.68.65` — see repo
memory for serial-port gotchas) and `operator-console` on your dev
machine, talking over real wifi. Loopback is fine for the plain-vs-encrypted
comparison (part 1) since that's about protocol overhead, not the radio
link, but the wifi-drop test (part 3) needs a real wifi hop — an interface
`down`/`up` on loopback proves nothing.

Capture on **both** ends (or better, a single mirrored/monitor-mode
vantage point if your AP supports it) so you're not relying on cross-host
clock sync to line up events. If you do capture on both hosts separately,
run `chronyc tracking` on each first and don't trust one-way latency
numbers past whatever offset it reports.

The dev-only mTLS certs already committed at `certs/` (SANs: `robot-edge`,
`localhost`, `127.0.0.1`) work fine for this even over a real LAN, since
`operator-console --server-name robot-edge` is what's checked against the
cert's SAN — it doesn't need to match the `--connect` IP. No need to
regenerate certs for a two-host test.

## Part 1+2: plain vs encrypted, and overhead vs raw data

**Step 1 — build:**

```bash
cargo build --workspace --release
```

**Step 2 — capture the real (encrypted) Channel B traffic.**

On the Pi:
```bash
sudo tshark -i wlan0 -f "udp port 4433" -w /tmp/quic_real.pcapng &
./target/release/robot-edge --listen 0.0.0.0:4433 \
  --cert certs/robot/robot.crt --key certs/robot/robot.key --ca certs/dev-ca/ca.crt \
  --stub-bridge --robot-id bench --tick-hz 50
```
(`--stub-bridge` skips real hardware I/O so the network path is isolated;
drop it if you specifically want serial-bridge latency included.)

On the dev machine:
```bash
./target/release/operator-console --connect 192.168.68.65:4433 --server-name robot-edge \
  --cert certs/operator/operator.crt --key certs/operator/operator.key --ca certs/dev-ca/ca.crt \
  --headless --tick-hz 50
```

Let it run ~30-60s, then Ctrl-C both sides and stop the capture (`kill %1`
or Ctrl-C the `tshark`).

**Step 3 — measure the real traffic:**

```bash
python3 benchmark/analyze_pcap.py /tmp/quic_real.pcapng --port 4433
```

This prints packet count, wire bytes, throughput, mean frame length, and
inter-arrival jitter. The mean frame length here is your real-world
"encrypted" datagram size — note it down (this is the fair size to feed
into the baseline run below for an apples-to-apples comparison; it isn't
the same as a fixed FlatBuffers payload size since QUIC coalesces headers
and short-packet framing on top).

**Step 4 — capture a plaintext baseline of the same size/rate:**

On the Pi:
```bash
sudo tshark -i wlan0 -f "udp port 5005" -w /tmp/udp_baseline.pcapng &
python3 benchmark/raw_udp_baseline.py recv --port 5005 --duration-s 40
```

On the dev machine (use the mean frame length from Step 3 as
`--payload-bytes`, and the same `--tick-hz` as `--rate-hz`):
```bash
python3 benchmark/raw_udp_baseline.py send --host 192.168.68.65 --port 5005 \
  --payload-bytes <mean_frame_len_from_step_3> --rate-hz 50 --duration-s 30
```

**Step 5 — measure the baseline and diff:**

```bash
python3 benchmark/analyze_pcap.py /tmp/udp_baseline.pcapng --port 5005
```

Compare the two `analyze_pcap.py` outputs side by side:
- **throughput (B/s) and mean frame length** — the delta between the QUIC
  run and the UDP baseline is the QUIC+TLS wire cost per packet (answers
  part 1).
- **mean frame length vs the underlying FlatBuffers payload size**
  (`TeleopCommand::pack()` is 18 bytes, `TelemetryData::pack()` is
  `7 + 2*joint_count` bytes — see [channel_b.rs](crates/robot-edge/src/channel_b.rs))
  — the delta there is UDP(8B)/IP(20B)/QUIC/TLS framing overhead on the
  real traffic (answers part 2).

## Part 3: wifi drop

Run the same two-process setup as Part 1, capturing on both ends (or the
mirrored vantage point), and additionally tail both processes' stderr/logs
so you can grep for `E-Stop latched` / `E-Stop cleared` afterwards (see
`scripts/smoke_test.sh` for the exact log lines to watch for — same check,
done here over a real radio link instead of loopback).

Once the session is up and steady (HELLO negotiated, Channel B frames
flowing), drop the operator's wifi interface mid-session:

```bash
sudo ./benchmark/wifi_drop_helper.sh wlan0 15
```

This logs UTC timestamps for when it took the interface down and back up.
After the run:

```bash
python3 benchmark/analyze_pcap.py /tmp/quic_real.pcapng --port 4433
```

Look at the `outage gaps` line it prints (gaps >5x the mean inter-arrival
interval) — that's your observed outage window in the capture. Compare it
against:
- the `wifi_drop_helper.sh` timestamps (how much of the outage was your
  simulated drop vs actual radio/driver behavior),
- the `E-Stop latched` log line's timestamp on `robot-edge` (should latch
  within ~400ms of the last frame it received — that's the Class D
  watchdog threshold both binaries default to),
- the `E-Stop cleared`/reconnection log lines once the interface comes
  back, and how many QUIC PTO retransmissions/handshake-less
  reconnects (if any — v0 has no connection migration, see
  [quic_server.rs](crates/robot-edge/src/quic_server.rs) module docs) show
  up in the pcap between drop and recovery.

The plaintext baseline is not useful for this part — it has no loss
detection or recovery signal at all, which is itself worth noting as a
qualitative result: dropping wifi under the raw-UDP baseline is silent on
both ends, where QUIC's ACK/PTO machinery and the app-level watchdog both
observably react.

## Part 4: protocol-to-protocol comparison (MQTT / Zenoh / raw UDP, encrypted + unencrypted)

`benchmark/mqtt_bench.py` and `benchmark/zenoh_bench.py` implement the
MQTT and Zenoh side of this (each has `responder`/`pingpong` for latency
and `send`/`recv` for throughput, `--tls`/`--ca`/`--cert`/`--key` for
MQTT's TLS+mTLS, `--config` pointing at a JSON5 file for Zenoh's — see
`benchmark/zenoh_tls.json5.example`). `benchmark/raw_udp_baseline.py`
already covers the UDP floor. `benchmark/bench_stats.py` has the shared
percentile/throughput reporting both scripts use, matching the blog's
"trim to 1st/99th percentile, report median" methodology.

**Channel B has no ping-pong mode yet.** Unlike MQTT/Zenoh, `robot-edge`
and `operator-console` don't have a built-in echo — Channel B is a
one-directional streaming teleop/telemetry link, not a request/response
protocol, so there's nothing today that would answer a synthetic ping.
Two ways to get a genuinely comparable Channel B latency number, neither
of which is scripted yet:
- add a minimal echo mode to `robot-edge` (reply to a tagged
  `TeleopCommand` with an immediate `TelemetryData` carrying the same
  seq/timestamp) — the correct fix, but it's a protocol change, not a
  benchmark-script change, so it needs its own review;
  or
- approximate one-way latency from `analyze_pcap.py`'s existing
  inter-arrival timestamps on a live capture (Part 1's tooling), which
  isn't a true RTT but is directly comparable to Zenoh/MQTT's one-way
  numbers from the same `pingpong` runs. Say clearly in any published
  result which of the two you used — they are not interchangeable
  numbers.

**Topology**: two same-class machines (dev-box-to-dev-box, wired LAN) or
one machine (loopback) — **not** the Pi. Report Pi+wifi numbers
separately if wanted, never mixed into this table.

**Protocols**: run each of MQTT and Zenoh in **two** variants —
TLS-disabled (matches the original blog) and TLS-enabled with client
cert auth if the client library supports it (matches Channel B's mTLS).
Channel B (QUIC+mTLS, `robot-edge`/`operator-console` binaries,
encryption always on) only ever has one variant and only appears in the
TLS-enabled table — see the caveat above. Raw UDP
(`benchmark/raw_udp_baseline.py`, already exists) is the protocol-free
floor and only belongs in the unencrypted table. Skip Kafka/DDS unless
there's a specific reason to include them — Kafka is a
durability/broker system, not a control-loop fit, and DDS only matters
here if a ROS2 comparison is the actual goal.

So each run produces two tables:
- unencrypted: MQTT-no-TLS, Zenoh-no-TLS, raw UDP
- encrypted (the one to lead with): MQTT+TLS(+mTLS if available),
  Zenoh+TLS(+mTLS if available), Channel B

Both latency and throughput tests below get run once per variant per
protocol.

**Latency test**: fixed 64-byte payload (matches ICMP so it's comparable
to a plain `ping` baseline), ping-pong round trip, best-effort/unreliable
QoS where the protocol supports it (reliability retry logic shouldn't be
in the latency number), report **half the median RTT** plus p95/p99,
outliers outside the 1st/99th percentile trimmed. ≥1000 round trips per
run. Cross-check against `ping` RTT/2 on the same path as a floor.

**Throughput test**: payload sweep — 8 B, 2 KB, 4 KB, 8 KB, 16 KB, 32 KB,
64 KB (cap well below the blog's 512 MB ceiling; that's testing a
generic pub/sub system's limits, not a teleop/telemetry link's realistic
range). One-way, sender floods at max rate, reliable QoS (matches
Channel B's real config), receiver reports msg/s and bytes/s over a fixed
window. Cross-check against `iperf3` on the same path as a ceiling.

**Reporting**: median + trimmed percentiles, not mean (wifi-adjacent
paths are long-tailed even on wired LAN under load). Same message
size/rate across all protocols in a given run. Note git commit / binary
versions per BENCHMARK's existing "Output locations" convention below.

## Other things worth trying once the above works

- **Handshake cost vs steady-state**: measure time from first UDP packet
  to `HELLO negotiated successfully` (mTLS handshake + HELLO) separately
  from steady-state per-frame latency — they have very different costs.
- **Channel A (video) contention**: if `--camera`/`--video` is enabled on
  both sides, capture Channel B alongside it and see whether Channel B
  jitter/latency degrades when the video stream saturates the link — a
  realistic bufferbloat scenario for a teleop robot with a camera.
- **Tick-rate sweep**: rerun Part 1 at a few `--tick-hz` values (e.g. 10,
  50, 100) to see how per-packet overhead trades off against packet-count
  overhead as rate changes.

## Output locations

There's no fixed convention yet — this doc's example commands write
captures to `/tmp/*.pcapng` and print analysis to stdout. If you want
results kept for comparison across runs, redirect `analyze_pcap.py`'s
output to a file (or use its `--csv` flag) under a directory of your
choosing and note the git commit / binary versions the run was against,
since none of this is wired into CI.
