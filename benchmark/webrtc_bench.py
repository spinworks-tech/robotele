#!/usr/bin/env python3
"""WebRTC (DimOS dimTELE stack) latency/throughput harness for the Part 4
protocol comparison (see BENCHMARK.md). Same shape as mqtt_bench.py and
zenoh_bench.py: responder/pingpong for latency, send/recv for throughput.

What it runs: DimOS's own `WebRTCPubSub` (dimos/protocol/pubsub/impl/webrtc),
the pubsub layer dimTELE's hosted teleop uses, over aiortc DataChannels.
dimTELE itself only ships Cloudflare-backed providers (the hosted broker, or
a direct Cloudflare Realtime SFU session), so every real dimTELE message
crosses the internet to a Cloudflare edge. To compare like for like with the
other loopback/LAN rows, this script adds `P2PProvider`: the same Provider
contract and channel settings as `CloudflareProvider`, but a direct peer
connection with a one-shot SDP exchange over a local TCP socket instead of
Cloudflare's REST signaling and SFU. Numbers from it are "dimTELE's WebRTC
stack, minus Cloudflare" -- say so wherever they are published.

Channel settings mirror dimTELE's: latency runs unordered with
maxRetransmits=0 (its `cmd_unreliable` command channel); throughput runs
reliable + ordered (its `state_reliable` channel). DataChannels are always
DTLS-encrypted, so this only belongs in the encrypted table, next to
Channel B.

Requires DimOS (its WebRTC pubsub only; the heavy base deps are not needed)
plus its `webrtc` extra:
    pip install --no-deps "dimos @ git+https://github.com/dimensionalOS/dimos@<commit>"
    pip install "aiortc>=1.14.0" "aiohttp>=3.9.0" pydantic structlog

Signaling: one side listens (--listen, default for responder/recv), the
other connects (default for pingpong/send). Start the listener first.

Usage:
    webrtc_bench.py responder
    webrtc_bench.py pingpong --count 1000
    webrtc_bench.py recv --duration-s 30
    webrtc_bench.py send --payload-bytes 2048 --rate-hz 1000 --duration-s 30
"""
from __future__ import annotations

import argparse
import asyncio
from collections import defaultdict
from collections.abc import Callable
import json
import logging
import queue
import struct
import sys
import time
import zlib
from typing import Any

try:
    from aiortc import (
        RTCConfiguration,
        RTCDataChannel,
        RTCPeerConnection,
        RTCSessionDescription,
    )
    from dimos.protocol.pubsub.impl.webrtc.providers.spec import (
        AsyncProviderBase,
        wait_connected,
        wait_open,
    )
    from dimos.protocol.pubsub.impl.webrtc.webrtcpubsub import WebRTCPubSub
except ImportError as e:
    sys.exit(f"{e}: needs dimos (--no-deps is enough) + aiortc/aiohttp, see the docstring")

from bench_stats import ThroughputReport, summarize_latency

HEADER = struct.Struct(">Qd")  # seq: u64, send_time: f64 (epoch seconds)
PING_TOPIC = "bench/ping"
PONG_TOPIC = "bench/pong"
THROUGHPUT_TOPIC = "bench/throughput"
# Same trick as CloudflareProvider: a negotiated placeholder forces an SCTP
# m-line into the offer, and topic channel ids stay clear of it.
_PLACEHOLDER_DC_ID = 100
# Stop queueing new sends above this much unsent data (aiortc's send queue is
# unbounded; a flood would otherwise just measure memory growth).
MAX_BUFFERED = 16 * 1024 * 1024


def _dc_id(topic: str) -> int:
    """Deterministic negotiated channel id both peers derive from the topic."""
    return 200 + zlib.crc32(topic.encode()) % 800


class P2PProvider(AsyncProviderBase):
    """Direct peer-to-peer DataChannel provider (no SFU, no broker).

    Implements DimOS's Provider contract (start/stop/publish/subscribe/
    is_connected) so it plugs into WebRTCPubSub unchanged. One bidirectional
    RTCPeerConnection per process; per-topic negotiated channels are created
    lazily on first publish or subscribe, with the same id on both peers.
    """

    def __init__(self, listen: bool, host: str, port: int, ordered: bool,
                 max_retransmits: int | None) -> None:
        super().__init__()
        self._listen = listen
        self._host = host
        self._port = port
        self._ordered = ordered
        self._max_retransmits = max_retransmits
        self._pc: RTCPeerConnection | None = None
        self._channel_lock: asyncio.Lock | None = None
        # Guarded by self._lock (from the base); never held across an await.
        self._channels: dict[str, RTCDataChannel] = {}
        self._callbacks: dict[str, list[Callable[[bytes, str], None]]] = defaultdict(list)

    # ─── Connect / Disconnect (loop thread) ──────────────────────────

    async def _connect(self) -> None:
        self._channel_lock = asyncio.Lock()
        # No ICE servers: host candidates only, nothing leaves the machine/LAN.
        self._pc = RTCPeerConnection(configuration=RTCConfiguration(iceServers=[]))
        self._pc.createDataChannel("_placeholder", negotiated=True, id=_PLACEHOLDER_DC_ID)
        if self._listen:
            await self._answer()
        else:
            await self._offer()
        await wait_connected(self._pc)

    async def _offer(self) -> None:
        assert self._pc
        for attempt in range(100):  # listener may still be starting
            try:
                reader, writer = await asyncio.open_connection(self._host, self._port)
                break
            except OSError:
                await asyncio.sleep(0.1)
        else:
            raise RuntimeError(f"no signaling listener on {self._host}:{self._port}")
        # aiortc gathers ICE fully inside setLocalDescription (non-trickle).
        await self._pc.setLocalDescription(await self._pc.createOffer())
        writer.write(json.dumps({"type": "offer", "sdp": self._pc.localDescription.sdp}).encode() + b"\n")
        await writer.drain()
        answer = json.loads(await reader.readline())
        writer.close()
        await self._pc.setRemoteDescription(RTCSessionDescription(**answer))

    async def _answer(self) -> None:
        assert self._pc
        got: asyncio.Future[None] = asyncio.get_running_loop().create_future()

        async def _on_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
            offer = json.loads(await reader.readline())
            await self._pc.setRemoteDescription(RTCSessionDescription(**offer))
            await self._pc.setLocalDescription(await self._pc.createAnswer())
            writer.write(json.dumps({"type": "answer", "sdp": self._pc.localDescription.sdp}).encode() + b"\n")
            await writer.drain()
            writer.close()
            if not got.done():
                got.set_result(None)

        server = await asyncio.start_server(_on_client, self._host, self._port)
        async with server:
            await asyncio.wait_for(got, timeout=300.0)

    async def _disconnect(self) -> None:
        if self._pc:
            await self._pc.close()
            self._pc = None
        with self._lock:
            self._channels.clear()

    # ─── Channel management (loop thread) ────────────────────────────

    async def _ensure_channel(self, topic: str) -> RTCDataChannel:
        assert self._channel_lock and self._pc
        async with self._channel_lock:
            with self._lock:
                ch = self._channels.get(topic)
            if ch is not None:
                return ch
            ch = self._pc.createDataChannel(
                topic,
                negotiated=True,
                id=_dc_id(topic),
                ordered=self._ordered,
                maxRetransmits=self._max_retransmits,
            )

            @ch.on("message")
            def _on_msg(payload: Any) -> None:
                if isinstance(payload, str):
                    payload = payload.encode()
                with self._lock:
                    callbacks = list(self._callbacks.get(topic, ()))
                for cb in callbacks:
                    try:
                        cb(payload, topic)
                    except Exception:
                        logging.exception("WebRTC subscriber callback error")

            await wait_open(ch)
            with self._lock:
                self._channels[topic] = ch
            return ch

    # ─── Public API (Provider) ───────────────────────────────────────

    def publish(self, topic: str, data: bytes) -> None:
        if not self.is_connected:
            self.start()
        with self._lock:
            ch = self._channels.get(topic)
        if ch is None:
            ch = self._run_sync(self._ensure_channel(topic))
        with self._lock:
            if not self._started or self._loop is None:
                return
            self._loop.call_soon_threadsafe(ch.send, bytes(data))

    def subscribe(self, topic: str, callback: Callable[[bytes, str], None]) -> Callable[[], None]:
        if not self.is_connected:
            self.start()
        with self._lock:
            self._callbacks[topic].append(callback)
        self._run_sync(self._ensure_channel(topic))

        def _unsub() -> None:
            with self._lock:
                try:
                    self._callbacks[topic].remove(callback)
                except ValueError:
                    pass

        return _unsub

    def open_channel(self, topic: str) -> None:
        """Open a topic's channel from a non-loop thread, ahead of publishing
        from a callback (callbacks run on the loop thread, where publish's
        lazy _run_sync would deadlock)."""
        self._run_sync(self._ensure_channel(topic))

    def buffered_amount(self) -> int:
        with self._lock:
            return sum(ch.bufferedAmount for ch in self._channels.values())


def _open(args: argparse.Namespace, default_listen: bool, reliable: bool) -> tuple[WebRTCPubSub, P2PProvider]:
    listen = default_listen if args.listen is None else args.listen
    provider = P2PProvider(
        listen=listen, host=args.host, port=args.port,
        ordered=reliable, max_retransmits=None if reliable else 0,
    )
    print(f"{'listening' if listen else 'connecting'} for signaling on {args.host}:{args.port} "
          f"({'reliable/ordered' if reliable else 'unordered, maxRetransmits=0'})", file=sys.stderr)
    pubsub = WebRTCPubSub(provider=provider)
    pubsub.start()
    return pubsub, provider


def responder(args: argparse.Namespace) -> None:
    pubsub, provider = _open(args, default_listen=True, reliable=False)
    provider.open_channel(PONG_TOPIC)
    pubsub.subscribe(PING_TOPIC, lambda data, _t: pubsub.publish(PONG_TOPIC, data))
    print(f"responder: echoing {PING_TOPIC} -> {PONG_TOPIC}", file=sys.stderr)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        pubsub.stop()


def pingpong(args: argparse.Namespace) -> None:
    if args.payload_bytes < HEADER.size:
        sys.exit(f"--payload-bytes must be >= {HEADER.size}")
    pubsub, _ = _open(args, default_listen=False, reliable=False)
    pong_q: queue.Queue[bytes] = queue.Queue()
    pubsub.subscribe(PONG_TOPIC, lambda data, _t: pong_q.put(data))
    pubsub.publish(PING_TOPIC, bytes(args.payload_bytes))  # opens the ping channel
    time.sleep(0.5)  # let both peers' channels open before timing
    while not pong_q.empty():
        pong_q.get_nowait()

    payload = bytearray(args.payload_bytes)
    rtts: list[float] = []
    lost = 0
    print(f"pingpong: {args.count} round trips, {args.payload_bytes}B payload (DTLS)", file=sys.stderr)
    for seq in range(args.count):
        sent = time.perf_counter()
        HEADER.pack_into(payload, 0, seq, sent)
        pubsub.publish(PING_TOPIC, bytes(payload))
        # Unreliable channel: a lost ping is a lost sample, not a hang.
        while True:
            try:
                reply = pong_q.get(timeout=1.0)
            except queue.Empty:
                lost += 1
                break
            if HEADER.unpack_from(reply, 0)[0] == seq:
                rtts.append(time.perf_counter() - sent)
                break
    pubsub.stop()
    if lost:
        print(f"{lost} ping(s) lost (unreliable channel)", file=sys.stderr)
    print(summarize_latency(rtts))


def send(args: argparse.Namespace) -> None:
    pubsub, provider = _open(args, default_listen=False, reliable=True)
    payload = bytearray(args.payload_bytes)
    pubsub.publish(THROUGHPUT_TOPIC, bytes(payload))  # opens the channel before timing
    interval = 1.0 / args.rate_hz
    seq = skipped = 0
    start = time.perf_counter()
    next_tick = start
    end = start + args.duration_s
    print(f"sending {args.payload_bytes}B messages at {args.rate_hz}Hz to {THROUGHPUT_TOPIC}",
          file=sys.stderr)
    while time.perf_counter() < end:
        if provider.buffered_amount() > MAX_BUFFERED:
            skipped += 1
        else:
            HEADER.pack_into(payload, 0, seq, time.time())
            pubsub.publish(THROUGHPUT_TOPIC, bytes(payload))
        seq += 1
        next_tick += interval
        sleep_s = next_tick - time.perf_counter()
        if sleep_s > 0:
            time.sleep(sleep_s)
    elapsed = time.perf_counter() - start
    pubsub.stop()
    print(f"sent {seq} messages over {elapsed:.2f}s ({skipped} skipped: send buffer full)",
          file=sys.stderr)


def recv(args: argparse.Namespace) -> None:
    pubsub, _ = _open(args, default_listen=True, reliable=True)
    counted = {"messages": 0, "bytes": 0, "on": False}

    def on_msg(data: bytes, _topic: str) -> None:
        if counted["on"]:
            counted["messages"] += 1
            counted["bytes"] += len(data)

    pubsub.subscribe(THROUGHPUT_TOPIC, on_msg)
    time.sleep(args.warmup_s)  # window starts after connect + warmup, not at process start
    print(f"counting on {THROUGHPUT_TOPIC} for {args.duration_s}s", file=sys.stderr)
    counted["on"] = True
    start = time.perf_counter()
    time.sleep(args.duration_s)
    counted["on"] = False
    elapsed = time.perf_counter() - start
    pubsub.stop()
    print(ThroughputReport(counted["messages"], counted["bytes"], elapsed))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    def add_common(p: argparse.ArgumentParser) -> None:
        p.add_argument("--host", default="127.0.0.1", help="signaling address")
        p.add_argument("--port", type=int, default=8765, help="signaling TCP port")
        role = p.add_mutually_exclusive_group()
        role.add_argument("--listen", dest="listen", action="store_true", default=None)
        role.add_argument("--connect", dest="listen", action="store_false")

    p_resp = sub.add_parser("responder", help="echo pings back as pongs")
    add_common(p_resp)

    p_pp = sub.add_parser("pingpong", help="measure round-trip latency")
    add_common(p_pp)
    p_pp.add_argument("--payload-bytes", type=int, default=64)
    p_pp.add_argument("--count", type=int, default=1000)

    p_send = sub.add_parser("send", help="one-way throughput sender")
    add_common(p_send)
    p_send.add_argument("--payload-bytes", type=int, default=2048)
    p_send.add_argument("--rate-hz", type=float, default=1000.0)
    p_send.add_argument("--duration-s", type=float, default=30.0)

    p_recv = sub.add_parser("recv", help="one-way throughput receiver")
    add_common(p_recv)
    p_recv.add_argument("--duration-s", type=float, default=30.0)
    p_recv.add_argument("--warmup-s", type=float, default=1.0)

    args = ap.parse_args()
    {"responder": responder, "pingpong": pingpong, "send": send, "recv": recv}[args.mode](args)


if __name__ == "__main__":
    main()
