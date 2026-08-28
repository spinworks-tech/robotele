#!/usr/bin/env python3
"""Plaintext UDP baseline for RoboProtocol benchmarking (see BENCHMARK.md).

Replays fixed-size, fixed-rate datagrams with no QUIC/TLS, so a Wireshark
capture of this traffic can be diffed against a real Channel B capture to
isolate QUIC+TLS wire overhead. Payload content is a seq/timestamp counter,
not real teleop data -- for a fair size comparison, only --payload-bytes
needs to match the real encoded Channel B frame size (measure that first
with analyze_pcap.py against a real capture; see BENCHMARK.md part b).

Usage:
    raw_udp_baseline.py recv --host 0.0.0.0 --port 5005 --duration-s 30
    raw_udp_baseline.py send --host <robot-ip> --port 5005 \\
        --payload-bytes 45 --rate-hz 50 --duration-s 30
"""
import argparse
import socket
import struct
import sys
import time

HEADER = struct.Struct(">Qd")  # seq: u64, send_time: f64 (epoch seconds)


def send(args: argparse.Namespace) -> None:
    if args.payload_bytes < HEADER.size:
        sys.exit(f"--payload-bytes must be >= {HEADER.size} (seq+timestamp header)")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload = bytearray(args.payload_bytes)
    interval = 1.0 / args.rate_hz
    seq = 0
    start = time.monotonic()
    next_tick = start
    end = start + args.duration_s if args.duration_s else None
    print(f"sending {args.payload_bytes}B datagrams at {args.rate_hz}Hz to "
          f"{args.host}:{args.port}", file=sys.stderr)
    while end is None or time.monotonic() < end:
        HEADER.pack_into(payload, 0, seq, time.time())
        sock.sendto(bytes(payload), (args.host, args.port))
        seq += 1
        next_tick += interval
        sleep_s = next_tick - time.monotonic()
        if sleep_s > 0:
            time.sleep(sleep_s)
    print(f"sent {seq} datagrams over {time.monotonic() - start:.2f}s", file=sys.stderr)


def recv(args: argparse.Namespace) -> None:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.host, args.port))
    count = 0
    lost = 0
    last_seq = None
    start = time.monotonic()
    end = start + args.duration_s if args.duration_s else None
    print(f"listening on {args.host}:{args.port}", file=sys.stderr)
    while end is None or time.monotonic() < end:
        remaining = (end - time.monotonic()) if end else 5.0
        sock.settimeout(max(0.05, remaining))
        try:
            data, _ = sock.recvfrom(65535)
        except socket.timeout:
            break
        if len(data) >= HEADER.size:
            seq, _sent_ts = HEADER.unpack_from(data, 0)
            if last_seq is not None and seq != last_seq + 1:
                lost += seq - last_seq - 1
            last_seq = seq
        count += 1
    elapsed = time.monotonic() - start
    rate = count / elapsed if elapsed > 0 else 0.0
    print(f"received {count} datagrams in {elapsed:.2f}s ({rate:.1f} pkt/s), "
          f"{lost} gap(s) detected in sequence", file=sys.stderr)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    p_send = sub.add_parser("send", help="send datagrams to a receiver")
    p_send.add_argument("--host", required=True)
    p_send.add_argument("--port", type=int, required=True)
    p_send.add_argument("--payload-bytes", type=int, default=64)
    p_send.add_argument("--rate-hz", type=float, default=50.0)
    p_send.add_argument("--duration-s", type=float, default=30.0)

    p_recv = sub.add_parser("recv", help="listen for datagrams")
    p_recv.add_argument("--host", default="0.0.0.0")
    p_recv.add_argument("--port", type=int, required=True)
    p_recv.add_argument("--duration-s", type=float, default=0.0,
                         help="0 = listen until 5s of silence")

    args = ap.parse_args()
    (send if args.mode == "send" else recv)(args)


if __name__ == "__main__":
    main()
