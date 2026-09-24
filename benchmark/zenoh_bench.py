#!/usr/bin/env python3
"""Zenoh latency/throughput harness for the Part 4 protocol comparison
(see BENCHMARK.md). Same shape as mqtt_bench.py so the two are directly
comparable: responder/pingpong for latency, send/recv for throughput.

Requires the `eclipse-zenoh` pip package, major version 1 to match the
Rust `zenoh` crate this repo already depends on for Channel C
(crates/robot-edge/src/zenoh_bridge.rs):
    pip install "eclipse-zenoh>=1,<2"

Zenoh is peer-to-peer by default (no broker needed), matching Channel B's
direct QUIC link more closely than MQTT's broker model -- run in `peer`
mode for that comparison, or point `--connect` at a `zenohd` router if you
want the brokered topology instead.

TLS: zenoh's TLS config lives in its JSON5 config, not simple flags, since
it configures a transport link rather than a per-connection option. Pass
`--config path/to/tls.json5` (see zenoh docs for `transport/link/tls`,
including `root_ca_certificate`/`connect_certificate`/`connect_private_key`
for the mTLS variant -- mutual auth needed to match Channel B's mTLS, see
BENCHMARK.md's "encrypted protocol-to-protocol" table). Without --config,
zenoh runs its default unencrypted UDP/TCP multicast-discovery transport.

Usage:
    zenoh_bench.py responder --key-expr bench
    zenoh_bench.py pingpong --key-expr bench --count 1000
    zenoh_bench.py recv --key-expr bench/throughput --duration-s 30
    zenoh_bench.py send --key-expr bench/throughput --payload-bytes 2048 \\
        --rate-hz 1000 --duration-s 30

    # TLS + mTLS:
    zenoh_bench.py pingpong --key-expr bench --config benchmark/zenoh_tls.json5 --count 1000
"""
from __future__ import annotations

import argparse
import queue
import struct
import sys
import time

try:
    import zenoh
except ImportError:
    sys.exit('eclipse-zenoh is required: pip install "eclipse-zenoh>=1,<2"')

from bench_stats import ThroughputReport, summarize_latency

HEADER = struct.Struct(">Qd")  # seq: u64, send_time: f64 (epoch seconds)


def _open_session(args: argparse.Namespace) -> "zenoh.Session":
    if args.config:
        conf = zenoh.Config.from_file(args.config)
    else:
        conf = zenoh.Config()
    return zenoh.open(conf)


def responder(args: argparse.Namespace) -> None:
    session = _open_session(args)
    ping_key = f"{args.key_expr}/ping"
    pong_key = f"{args.key_expr}/pong"
    pub = session.declare_publisher(pong_key)

    def on_ping(sample: "zenoh.Sample") -> None:
        pub.put(bytes(sample.payload))

    session.declare_subscriber(ping_key, on_ping)
    print(f"responder: echoing {ping_key} -> {pong_key} "
          f"({'TLS via ' + args.config if args.config else 'default transport'})",
          file=sys.stderr)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()


def pingpong(args: argparse.Namespace) -> None:
    if args.payload_bytes < HEADER.size:
        sys.exit(f"--payload-bytes must be >= {HEADER.size}")
    session = _open_session(args)
    ping_key = f"{args.key_expr}/ping"
    pong_key = f"{args.key_expr}/pong"
    pong_q: queue.Queue[bytes] = queue.Queue()

    session.declare_subscriber(pong_key, lambda sample: pong_q.put(bytes(sample.payload)))
    pub = session.declare_publisher(ping_key)
    time.sleep(0.5)  # let discovery/subscription settle before pinging

    payload = bytearray(args.payload_bytes)
    rtts: list[float] = []
    print(f"pingpong: {args.count} round trips, {args.payload_bytes}B payload "
          f"({'TLS via ' + args.config if args.config else 'default transport'})",
          file=sys.stderr)
    for seq in range(args.count):
        sent = time.perf_counter()
        HEADER.pack_into(payload, 0, seq, sent)
        pub.put(bytes(payload))
        try:
            pong_q.get(timeout=5.0)
        except queue.Empty:
            sys.exit(f"timed out waiting for pong #{seq} -- is `responder` running?")
        rtts.append(time.perf_counter() - sent)

    session.close()
    print(summarize_latency(rtts))


def send(args: argparse.Namespace) -> None:
    session = _open_session(args)
    pub = session.declare_publisher(args.key_expr)
    payload = bytearray(args.payload_bytes)
    interval = 1.0 / args.rate_hz
    seq = 0
    start = time.perf_counter()
    next_tick = start
    end = start + args.duration_s
    print(f"sending {args.payload_bytes}B messages at {args.rate_hz}Hz to "
          f"{args.key_expr} "
          f"({'TLS via ' + args.config if args.config else 'default transport'})",
          file=sys.stderr)
    while time.perf_counter() < end:
        HEADER.pack_into(payload, 0, seq, time.time())
        pub.put(bytes(payload))
        seq += 1
        next_tick += interval
        sleep_s = next_tick - time.perf_counter()
        if sleep_s > 0:
            time.sleep(sleep_s)
    session.close()
    print(f"sent {seq} messages over {time.perf_counter() - start:.2f}s", file=sys.stderr)


def recv(args: argparse.Namespace) -> None:
    session = _open_session(args)
    counted = {"messages": 0, "bytes": 0}

    def on_sample(sample: "zenoh.Sample") -> None:
        counted["messages"] += 1
        counted["bytes"] += len(bytes(sample.payload))

    session.declare_subscriber(args.key_expr, on_sample)
    print(f"listening on {args.key_expr} for {args.duration_s}s", file=sys.stderr)
    start = time.perf_counter()
    time.sleep(args.duration_s)
    session.close()
    elapsed = time.perf_counter() - start
    report = ThroughputReport(counted["messages"], counted["bytes"], elapsed)
    print(report)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    def add_common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--config", help="zenoh JSON5 config file (needed for TLS/mTLS)")

    p_resp = sub.add_parser("responder", help="echo ping key-expr to pong key-expr")
    add_common(p_resp)
    p_resp.add_argument("--key-expr", default="bench")

    p_pp = sub.add_parser("pingpong", help="measure round-trip latency")
    add_common(p_pp)
    p_pp.add_argument("--key-expr", default="bench")
    p_pp.add_argument("--payload-bytes", type=int, default=64)
    p_pp.add_argument("--count", type=int, default=1000)

    p_send = sub.add_parser("send", help="throughput flood")
    add_common(p_send)
    p_send.add_argument("--key-expr", default="bench/throughput")
    p_send.add_argument("--payload-bytes", type=int, default=2048)
    p_send.add_argument("--rate-hz", type=float, default=1000.0)
    p_send.add_argument("--duration-s", type=float, default=30.0)

    p_recv = sub.add_parser("recv", help="count throughput flood")
    add_common(p_recv)
    p_recv.add_argument("--key-expr", default="bench/throughput")
    p_recv.add_argument("--duration-s", type=float, default=30.0)

    args = ap.parse_args()
    {"responder": responder, "pingpong": pingpong, "send": send, "recv": recv}[args.mode](args)


if __name__ == "__main__":
    main()
