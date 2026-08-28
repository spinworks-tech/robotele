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

## Tools needed

| Tool | Purpose | Install |
|---|---|---|
| `cargo` / Rust workspace | build `robot-edge`, `operator-console`, `dev-certs` | already required by this repo |
| `tshark` (Wireshark ≥3.4) | packet capture + QUIC dissection | `sudo apt install tshark` |
| `python3` (stdlib only) | runs `benchmark/*.py` — no pip installs needed | usually preinstalled |
| `sudo ip link` / `nmcli` / `rfkill` | simulate a wifi drop | preinstalled on most Linux |
| `chrony` or `ntpd` (optional) | clock sync, only needed if you want cross-host one-way latency, not just RTT/throughput | `sudo apt install chrony` |

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
