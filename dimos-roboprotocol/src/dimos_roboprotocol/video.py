"""H.264 decoding for the robot-edge Zenoh video tap (`robotele/<id>/video`).

Each Zenoh sample is one NAL unit with its Annex-B start code prepended, in
stream order (robot-edge crates/robot-edge/src/zenoh_bridge.rs `VideoSink`).
Concatenated, they are a valid Annex-B H.264 stream, so a stock decoder can
consume them. SPS/PPS are replayed when a subscriber joins; the picture starts
at the next IDR (about one second with `--intra 30` at 30 fps).
"""
from __future__ import annotations

import numpy as np


class H264Decoder:
    """Feeds Annex-B NAL units in, returns decoded RGB frames out.

    Not thread-safe: use from one callback thread, or guard with a lock.
    """

    def __init__(self) -> None:
        import av  # PyAV; imported lazily so the rest of the package needs no ffmpeg

        self._av = av
        self._ctx = av.CodecContext.create("h264", "r")

    def decode(self, annexb: bytes) -> list[np.ndarray]:
        """Returns zero or more HxWx3 uint8 RGB frames.

        The parser only emits a frame once it sees the start of the next one,
        so output lags input by one frame. Data the decoder cannot use (a
        mid-GOP join before the first IDR, or a lost NAL) yields no frames
        instead of raising: the stream recovers at the next IDR.
        """
        frames: list[np.ndarray] = []
        try:
            for packet in self._ctx.parse(annexb):
                for frame in self._ctx.decode(packet):
                    frames.append(frame.to_ndarray(format="rgb24"))
        except self._av.error.FFmpegError:
            pass
        return frames
