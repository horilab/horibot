"""영상 메시지 형식 (docs/decisions/0039-camera-stream.md)."""

import numpy as np
import pytest

from horibot_camera.message import End, Frame, Header, decode, encode, end_message, exposure_middle


def _header(width: int = 4, height: int = 3) -> Header:
    return Header(
        seq=7,
        depth_no=120,
        color_no=121,
        width=width,
        height=height,
        depth_scale=0.0001,
        captured=1_764_300_000.125,
        received=1_764_300_000.170,
        sent=1_764_300_000.171,
    )


def test_frame_round_trip_keeps_values_and_shapes() -> None:
    rng = np.random.default_rng(1)
    header = _header()
    depth = rng.integers(0, 65535, size=(3, 4), dtype=np.uint16)
    color = rng.integers(0, 255, size=(3, 4, 2), dtype=np.uint8)
    parts = encode(header, depth.astype("<u2").tobytes(), color.tobytes())
    frame = decode([bytes(p) for p in parts])
    assert isinstance(frame, Frame)
    assert frame.header == header
    assert np.array_equal(frame.depth, depth)
    assert np.array_equal(frame.color_yuyv, color)


def test_sizes_must_match_the_header() -> None:
    header = _header()
    good = bytes(4 * 3 * 2)
    with pytest.raises(ValueError, match="뎁스"):
        encode(header, bytes(10), good)
    parts = [bytes(p) for p in encode(header, good, good)]
    with pytest.raises(ValueError, match="장 크기"):
        decode([parts[0], parts[1], bytes(10)])


def test_end_message_carries_the_summary() -> None:
    end = decode(end_message({"sent": 42}))
    assert isinstance(end, End)
    assert end.summary == {"sent": 42}


def test_exposure_middle_subtracts_half_the_exposure() -> None:
    # 읽기 시작이 1000 ms, frame_timestamp − sensor_timestamp가 1000 µs(노출의 반)
    assert exposure_middle(1000.0, 5000.0, 4000.0) == pytest.approx(0.999)
    assert exposure_middle(1000.0, None, 4000.0) is None
