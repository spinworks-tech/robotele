#!/usr/bin/env python3
"""MQTT latency/throughput harness for the Part 4 protocol comparison
(see BENCHMARK.md). Runs against a real mosquitto (or any MQTT 3.1.1/5
broker) with QoS 0, matching the settings the Zenoh-vs-MQTT/Kafka/DDS blog
post used, plus an unencrypted-vs-TLS(+mTLS) toggle this repo's comparison
adds on top.

Requires the `paho-mqtt` pip package (not stdlib -- this is the one script
in `benchmark/` that needs a pip install, since Python has no built-in MQTT
client):
    pip install paho-mqtt

Two things get measured, mirroring raw_udp_baseline.py's send/recv split:

  responder  -- subscribes to the ping topic, republishes each payload
                unchanged to the pong topic. Run this once, leave it up.
  pingpong   -- publishes N timestamped pings to the ping topic, waits for
                each echo on the pong topic, reports RTT stats (see
                bench_stats.py). Fixed 64-byte payload by convention,
                matching the blog's ICMP-sized latency payload.
  send/recv  -- one-way throughput flood at a fixed payload size and rate,
                same shape as raw_udp_baseline.py send/recv.

TLS: pass --tls with --ca (and --cert/--key for mutual TLS, matching
Channel B's mTLS -- see BENCHMARK.md's "encrypted protocol-to-protocol"
table). Without --tls, connects in cleartext -- only use that for the
unencrypted-context table, not the headline comparison.

Usage:
    # broker side (mosquitto), unencrypted:
    mosquitto -p 1883
    # or with TLS: mosquitto -p 8883 -c mosquitto-tls.conf (require_certificate true for mTLS)

    mqtt_bench.py responder --host localhost --port 1883
    mqtt_bench.py pingpong --host localhost --port 1883 --count 1000
    mqtt_bench.py recv --host localhost --port 1883 --duration-s 30
    mqtt_bench.py send --host localhost --port 1883 --payload-bytes 2048 \\
        --rate-hz 50 --duration-s 30

    # TLS + mTLS:
    mqtt_bench.py pingpong --host broker --port 8883 --tls \\
        --ca certs/dev-ca/ca.crt --cert certs/operator/operator.crt \\
        --key certs/operator/operator.key --count 1000
"""
from __future__ import annotations

import argparse
import queue
import ssl
import struct
import sys
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt is required: pip install paho-mqtt")

from bench_stats import ThroughputReport, summarize_latency

HEADER = struct.Struct(">Qd")  # seq: u64, send_time: f64 (epoch seconds)
PING_TOPIC = "bench/ping"
PONG_TOPIC = "bench/pong"
THROUGHPUT_TOPIC = "bench/throughput"


def _make_client(args: argparse.Namespace, client_id: str) -> mqtt.Client:
    client = mqtt.Client(
        callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
        client_id=client_id,
        protocol=mqtt.MQTTv311,
    )
    if args.tls:
        client.tls_set(
            ca_certs=args.ca,
            certfile=args.cert,
            keyfile=args.key,
            tls_version=ssl.PROTOCOL_TLS_CLIENT,
        )
    client.connect(args.host, args.port, keepalive=30)
    return client


def responder(args: argparse.Namespace) -> None:
    client = _make_client(args, "bench-responder")

    def on_connect(c, userdata, flags, rc, properties=None):
        c.subscribe(PING_TOPIC, qos=0)

    def on_message(c, userdata, msg):
        c.publish(PONG_TOPIC, msg.payload, qos=0)

    client.on_connect = on_connect
    client.on_message = on_message
    print(f"responder: echoing {PING_TOPIC} -> {PONG_TOPIC} "
          f"({'TLS' if args.tls else 'plaintext'})", file=sys.stderr)
    client.loop_forever()


def pingpong(args: argparse.Namespace) -> None:
    if args.payload_bytes < HEADER.size:
        sys.exit(f"--payload-bytes must be >= {HEADER.size}")
    pong_q: queue.Queue[bytes] = queue.Queue()
    client = _make_client(args, "bench-pinger")

    def on_connect(c, userdata, flags, rc, properties=None):
        c.subscribe(PONG_TOPIC, qos=0)

    def on_message(c, userdata, msg):
        pong_q.put(msg.payload)

    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_start()
    time.sleep(0.5)  # let the subscription land before we start pinging

    payload = bytearray(args.payload_bytes)
    rtts: list[float] = []
    print(f"pingpong: {args.count} round trips, {args.payload_bytes}B payload "
          f"({'TLS' if args.tls else 'plaintext'})", file=sys.stderr)
    for seq in range(args.count):
        sent = time.perf_counter()
        HEADER.pack_into(payload, 0, seq, sent)
        client.publish(PING_TOPIC, bytes(payload), qos=0)
        try:
            pong_q.get(timeout=5.0)
        except queue.Empty:
            sys.exit(f"timed out waiting for pong #{seq} -- is `responder` running?")
        rtts.append(time.perf_counter() - sent)

    client.loop_stop()
    print(summarize_latency(rtts))


def send(args: argparse.Namespace) -> None:
    client = _make_client(args, "bench-sender")
    client.loop_start()
    payload = bytearray(args.payload_bytes)
    interval = 1.0 / args.rate_hz
    seq = 0
    start = time.perf_counter()
    next_tick = start
    end = start + args.duration_s
    print(f"sending {args.payload_bytes}B messages at {args.rate_hz}Hz to "
          f"{THROUGHPUT_TOPIC} ({'TLS' if args.tls else 'plaintext'})", file=sys.stderr)
    while time.perf_counter() < end:
        HEADER.pack_into(payload, 0, seq, time.time())
        client.publish(THROUGHPUT_TOPIC, bytes(payload), qos=0)
        seq += 1
        next_tick += interval
        sleep_s = next_tick - time.perf_counter()
        if sleep_s > 0:
            time.sleep(sleep_s)
    client.loop_stop()
    print(f"sent {seq} messages over {time.perf_counter() - start:.2f}s", file=sys.stderr)


def recv(args: argparse.Namespace) -> None:
    counted = {"messages": 0, "bytes": 0}
    client = _make_client(args, "bench-receiver")

    def on_connect(c, userdata, flags, rc, properties=None):
        c.subscribe(THROUGHPUT_TOPIC, qos=0)

    def on_message(c, userdata, msg):
        counted["messages"] += 1
        counted["bytes"] += len(msg.payload)

    client.on_connect = on_connect
    client.on_message = on_message
    client.loop_start()
    print(f"listening on {THROUGHPUT_TOPIC} for {args.duration_s}s", file=sys.stderr)
    start = time.perf_counter()
    time.sleep(args.duration_s)
    client.loop_stop()
    elapsed = time.perf_counter() - start
    report = ThroughputReport(counted["messages"], counted["bytes"], elapsed)
    print(report)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    def add_common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--host", required=True)
        p.add_argument("--port", type=int, default=1883)
        p.add_argument("--tls", action="store_true")
        p.add_argument("--ca", help="CA cert (required with --tls)")
        p.add_argument("--cert", help="client cert, for mTLS")
        p.add_argument("--key", help="client key, for mTLS")

    p_resp = sub.add_parser("responder", help="echo ping topic to pong topic")
    add_common(p_resp)

    p_pp = sub.add_parser("pingpong", help="measure round-trip latency")
    add_common(p_pp)
    p_pp.add_argument("--payload-bytes", type=int, default=64)
    p_pp.add_argument("--count", type=int, default=1000)

    p_send = sub.add_parser("send", help="throughput flood")
    add_common(p_send)
    p_send.add_argument("--payload-bytes", type=int, default=2048)
    p_send.add_argument("--rate-hz", type=float, default=1000.0)
    p_send.add_argument("--duration-s", type=float, default=30.0)

    p_recv = sub.add_parser("recv", help="count throughput flood")
    add_common(p_recv)
    p_recv.add_argument("--duration-s", type=float, default=30.0)

    args = ap.parse_args()
    if args.tls and not args.ca:
        ap.error("--tls requires --ca")
    {"responder": responder, "pingpong": pingpong, "send": send, "recv": recv}[args.mode](args)


if __name__ == "__main__":
    main()
