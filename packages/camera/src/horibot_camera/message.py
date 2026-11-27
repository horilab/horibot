"""영상 메시지 형식 (docs/decisions/0039-camera-stream.md).

zmq 메시지 하나가 [머리 JSON, 뎁스 바이트, 컬러 바이트]다. 카메라가 USB로 내는 형식 그대로 압축 없이 담는다:
뎁스는 Z16(화소마다 uint16, 한 칸이 depth_scale m), 컬러는 YUYV(화소 둘이 4바이트). 시각은 모두 파이 시계의
유닉스 시각 [s]다. 보내는 쪽이 멈출 때는 머리만 있는 끝 메시지({"end": true, ...})를 보낸다.
"""

from __future__ import annotations

import json
from collections.abc import Sequence
from dataclasses import asdict, dataclass
from typing import Any

import numpy as np

Part = bytes | bytearray | memoryview


@dataclass(frozen=True)
class Header:
    """장 하나의 머리."""

    seq: int  # 보낸 번호 (보내는 쪽이 켤 때마다 0부터 센다)
    depth_no: int  # 카메라의 장 번호
    color_no: int
    width: int
    height: int
    depth_scale: float  # 뎁스 한 칸 [m]
    captured: float | None  # 찍은 때: 뎁스 장 노출 가운데 (메타데이터가 없으면 None)
    received: float  # 파이가 장을 받은 때
    sent: float  # 보낸 때


@dataclass(frozen=True)
class Frame:
    """받은 장: 뎁스 (h, w) uint16, 컬러 YUYV (h, w, 2) uint8."""

    header: Header
    depth: np.ndarray
    color_yuyv: np.ndarray


@dataclass(frozen=True)
class End:
    """보내는 쪽이 멈추며 보낸 끝 메시지 (보낸 장, 빠진 장 같은 요약)."""

    summary: dict[str, Any]


def exposure_middle(
    timestamp_ms: float, frame_timestamp_us: float | None, sensor_timestamp_us: float | None
) -> float | None:
    """찍은 때 [s]: librealsense 전역 시각(get_timestamp(), 파이 시계 ms)은 읽기를 시작한 때라서, 메타데이터
    frame_timestamp − sensor_timestamp(노출의 반, µs)를 뺀다 (docs/software.md). 메타데이터가 없으면 None."""
    if frame_timestamp_us is None or sensor_timestamp_us is None:
        return None
    return (timestamp_ms - (frame_timestamp_us - sensor_timestamp_us) / 1000.0) / 1000.0


def frame_bytes(width: int, height: int) -> int:
    """뎁스나 컬러 한 장의 바이트 수 (Z16도 YUYV도 화소마다 2바이트)."""
    return width * height * 2


def encode(header: Header, depth: Part, color: Part) -> list[Part]:
    size = frame_bytes(header.width, header.height)
    for name, part in (("뎁스", depth), ("컬러", color)):
        if memoryview(part).nbytes != size:
            raise ValueError(
                f"{name} {memoryview(part).nbytes}바이트: {header.width}×{header.height}면 {size}바이트다"
            )
    return [json.dumps(asdict(header)).encode(), depth, color]


def end_message(summary: dict[str, Any]) -> list[Part]:
    return [json.dumps({"end": True, **summary}).encode()]


def decode(parts: Sequence[Part]) -> Frame | End:
    head = json.loads(bytes(parts[0]))
    if head.get("end"):
        return End({k: v for k, v in head.items() if k != "end"})
    if len(parts) != 3:
        raise ValueError(f"장 메시지는 세 부분이다: {len(parts)}")
    header = Header(**head)
    size = frame_bytes(header.width, header.height)
    depth_part, color_part = memoryview(parts[1]), memoryview(parts[2])
    if depth_part.nbytes != size or color_part.nbytes != size:
        raise ValueError(
            f"장 크기가 머리와 다르다: 뎁스 {depth_part.nbytes}, 컬러 {color_part.nbytes}, 바랐던 {size}바이트"
        )
    depth = np.frombuffer(depth_part, dtype="<u2").reshape(header.height, header.width)
    color = np.frombuffer(color_part, dtype=np.uint8).reshape(header.height, header.width, 2)
    return Frame(header, depth, color)
