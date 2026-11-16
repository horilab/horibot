"""형상을 화면이 읽는 형상 파일로 바꾼다.

형상 파일(JSON): 꼭짓점을 형상 상자의 가운데(origin)에서 잰 정수(칸 q mm)로, 삼각형을 꼭짓점 번호로 묶어
base64로 쓴다. 꼭짓점은 int16, 번호는 꼭짓점이 65,535개 이하면 uint16, 넘으면 uint32다.

같은 형상(옮기기만 다른 나사, 너트)은 origin에서 잰 값이 같아서 id(내용의 해시)가 같고, 한 번만 쓴다. 장면에는
놓는 자리로 origin만 둔다 (화면은 꼭짓점 × q에 origin을 더한다).
"""

from __future__ import annotations

import base64
import hashlib
import json
import math
import sys
from array import array
from dataclasses import dataclass
from typing import Any

from build123d import Compound

# 삼각형으로 바꾸는 허용 오차 [mm]와 각 [rad]: 지금의 3D 페이지와 같다
TOLERANCE = 0.06
ANGULAR = 0.3
STEP = 0.01  # 꼭짓점 칸 [mm]. 형상이 655 mm보다 크면 칸을 늘린다
INT16 = 32767


@dataclass(frozen=True)
class Mesh:
    id: str
    origin: tuple[float, float, float]
    q: float
    size: tuple[float, float, float]
    vertices: bytes  # int16 x, y, z ...
    indices: bytes
    index_type: str  # "u16" | "u32"
    triangles: int

    def to_json(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "q": self.q,
            "size": [round(s, 3) for s in self.size],
            "v": base64.b64encode(self.vertices).decode("ascii"),
            "i": base64.b64encode(self.indices).decode("ascii"),
            "it": self.index_type,
            "tris": self.triangles,
        }


def tessellate(
    shape: Compound,
) -> tuple[list[tuple[float, float, float]], list[tuple[int, int, int]]]:
    verts, tris = shape.tessellate(TOLERANCE, ANGULAR)
    return [(v.X, v.Y, v.Z) for v in verts], [(t[0], t[1], t[2]) for t in tris]


def pack(points: list[tuple[float, float, float]], triangles: list[tuple[int, int, int]]) -> Mesh:
    """꼭짓점과 삼각형을 형상 파일로 묶는다."""
    if not points or not triangles:
        raise ValueError("빈 형상이다")
    lo = tuple(min(p[a] for p in points) for a in range(3))
    hi = tuple(max(p[a] for p in points) for a in range(3))
    size = (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])
    center = tuple((lo[a] + hi[a]) / 2 for a in range(3))
    q = max(STEP, max(size) / (2 * INT16 - 2))
    q = math.ceil(q / STEP - 1e-9) * STEP
    coords: array[int] = array("h")
    for p in points:
        for a in range(3):
            coords.append(round((p[a] - center[a]) / q))
    flat = [k for t in triangles for k in t]
    index_type = "u16" if len(points) <= 65535 else "u32"
    idx: array[int] = array("H" if index_type == "u16" else "I", flat)
    if coords.itemsize != 2 or idx.itemsize != (2 if index_type == "u16" else 4):
        raise RuntimeError("이 파이썬의 array 크기가 기대와 다르다")
    vb, ib = _little(coords), _little(idx)
    digest = hashlib.sha1(json.dumps(round(q, 6)).encode() + vb + ib).hexdigest()[:12]
    origin = (center[0], center[1], center[2])
    return Mesh(digest, origin, q, size, vb, ib, index_type, len(triangles))


def _little(a: array[int]) -> bytes:
    """little-endian 바이트 (브라우저의 TypedArray가 읽는 순서)."""
    if sys.byteorder == "big":
        a = array(a.typecode, a)
        a.byteswap()
    return a.tobytes()


def unpack(data: dict[str, Any]) -> tuple[list[tuple[float, float, float]], list[int]]:
    """형상 파일을 상자 가운데에서 잰 꼭짓점 [mm]과 번호로 되돌린다 (시험용)."""
    q = float(data["q"])
    coords: array[int] = array("h")
    coords.frombytes(base64.b64decode(data["v"]))
    idx: array[int] = array("H" if data["it"] == "u16" else "I")
    idx.frombytes(base64.b64decode(data["i"]))
    pts = [(coords[k] * q, coords[k + 1] * q, coords[k + 2] * q) for k in range(0, len(coords), 3)]
    return pts, list(idx)
