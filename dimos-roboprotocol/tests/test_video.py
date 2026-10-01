import pytest

av = pytest.importorskip("av")
np = pytest.importorskip("numpy")

from dimos_roboprotocol.video import H264Decoder

START = b"\x00\x00\x00\x01"


def _encode_annexb(n_frames=8, w=64, h=48):
    """A tiny real H.264 Annex-B stream, split into start-code-prefixed NALs."""
    import io

    buf = io.BytesIO()
    out = av.open(buf, "w", format="h264")
    st = out.add_stream("libx264", rate=10)
    st.width, st.height, st.pix_fmt = w, h, "yuv420p"
    st.options = {"g": "4", "bf": "0", "tune": "zerolatency"}
    for i in range(n_frames):
        frame = av.VideoFrame.from_ndarray(np.full((h, w, 3), i * 20, np.uint8), format="rgb24")
        for p in st.encode(frame):
            out.mux(p)
    for p in st.encode():
        out.mux(p)
    out.close()
    stream = buf.getvalue()
    starts = [i for i in range(len(stream) - 3) if stream[i : i + 4] == START]
    return [stream[a:b] for a, b in zip(starts, starts[1:] + [len(stream)])]


def test_decodes_a_stream_fed_nal_by_nal():
    nals = _encode_annexb()
    dec = H264Decoder()
    frames = [f for nal in nals for f in dec.decode(nal)]
    assert len(frames) >= 6
    assert frames[0].shape == (48, 64, 3) and frames[0].dtype == np.uint8


def test_undecodable_data_yields_nothing_instead_of_raising():
    nals = _encode_annexb(12)
    dec = H264Decoder()
    # Inter frames only (no SPS/PPS/IDR seen): nothing to decode, and no exception.
    inter = [n for n in nals if (n[4] & 0x1F) == 1]
    assert inter
    assert [f for nal in inter[:2] for f in dec.decode(nal)] == []
    # Then the whole stream, as when a subscriber joins and SPS/PPS + an IDR arrive.
    assert [f for nal in nals for f in dec.decode(nal)]
