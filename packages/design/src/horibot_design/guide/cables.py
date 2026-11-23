"""첫 관절(J5)에서 서보 선이 출력 원판 테에 감기는 모양: 관절 각에 따라 바뀐다 (0031).

선은 손목 1 앞벽의 가운데 홈에서 나와 살 위에 눕고, 테에 닿기 전에 세워져(폭이 출력축 방향) 두 몸통 사이 층에서
테에 감긴 뒤, 손목 2의 뒷덮개 쪽 모서리로 들어간다. D405 선은 관절 밖으로 넘어서 이 층에 들지 않는다.

층 안의 모양은 두 끝을 잡은 끈(길이가 정해짐)을 풀어서 낸다: 굽힘을 고르게 펴고, 마디 길이를 지키고, 테와 멈춤
귀 밖에 두고, 자기와 겹치지 않게 민다. 관절 각을 한쪽 멈춤에서 다른 멈춤까지 조금씩 옮기며 앞 모양에서 이어
풀어서, 남는 선의 고리가 끊기지 않고 테에 감기며 줄어든다. 선 길이는 가장 길게 감기는 멈춤(-163°)에서 손으로 테에
붙였을 때의 길이다 (0031: 선은 가장 길 때에 맞춘다).

모양은 어림이다. 실제 고리가 어디로 나오는지는 선의 뻣뻣함과 버릇에 달려서 첫 관절에서 감아 돌려 본다 (0031).
좌표는 첫 관절 조립 안내의 장면 좌표(부모 관절의 서보 좌표, mm, y가 출력축)다. 방위각은 atan2(z, x)이고, 자식은
관절 각 q만큼 방위각이 커지는 쪽으로 돈다 (parts/joint.py의 turned).
"""

from __future__ import annotations

import itertools
import math
from collections.abc import Sequence
from dataclasses import dataclass, replace

import numpy as np
from numpy.typing import NDArray

from horibot_design.guide.model import Flex
from horibot_design.parts import joint as j
from horibot_design.studies import wiring as w

Arr = NDArray[np.float64]

# 세워서 테에 감긴 선의 가운데 높이: 위아래 모두 층 안에 든다 (층은 앞면에서 자식 발 바닥까지)
LAYER_Y = j.ATOP - 0.4 - j.WIRE_W / 2
FLAT_Y = j.FRONT + j.WIRE_T / 2  # 살 위에 눕힌 선의 가운데 높이
# 홈에서 나온 선은 살 위에 눕고(앞쪽 FLAT_MM), 테에 닿기 전에 세워진다(EDGE_MM부터). 납작한 선은 눕힌 채로는 테
# 둘레로 굽지 않는다
FLAT_MM, EDGE_MM = 5.0, 15.0
EAR_OUT = j.R_POST + j.EAR_HW  # 멈춤 귀의 바깥 끝 (출력 원판의 귀)
ROD_N = 56  # 층 안의 선을 나누는 점 수
STOP = j.STOP_DEG
LONG_STOP = -STOP  # 선이 가장 길게 감기는 멈춤 (손목 2로 들어가는 자리가 살에서 156° 쪽이라서)
PASSES = 15  # 한 번 풀 때 마디 길이를 맞추는 횟수 (적으면 선이 늘어난다)
PRESS = 0.04  # 한 번 풀 때 남는 선을 바깥으로 미는 거리 [mm]
SPARE = 0.5  # 가장 길게 감길 때보다 더 남는 선: 멈춤에서 손으로 테에 붙였을 때의 느슨함 (어림)


def rot(p: Arr, q_deg: float) -> Arr:
    """(x, z) 점들을 방위각이 커지는 쪽으로 q만큼 돌린다 (자식이 도는 쪽)."""
    a = math.radians(q_deg)
    c, s = math.cos(a), math.sin(a)
    out = np.empty_like(p)
    out[..., 0] = p[..., 0] * c - p[..., 1] * s
    out[..., 1] = p[..., 0] * s + p[..., 1] * c
    return out


@dataclass(frozen=True)
class Rod:
    """층 안의 선 한 가닥 (평면 (x, z)). 부모 쪽 끝은 부모에 잡혀 있고, 끝점은 자식과 같이 돈다."""

    start: tuple[float, float]  # 부모 쪽 끝
    lead: tuple[float, float] | None  # 부모 쪽 끝에서 나가는 방향 (홈이 정한다). 없으면 자유다
    end: tuple[float, float]  # 자식 쪽 끝 (관절 각 0)
    length: float  # 층 안의 선 길이
    radius: float  # 선 가운데가 테에서 가장 가까울 때의 반지름
    half: float  # 평면에서 본 선의 반 두께 (겹침을 볼 때)


def _wrap(d: float) -> float:
    return (d + 180.0) % 360.0 - 180.0


def _delta(rod: Rod, q_deg: float) -> float:
    """부모 쪽 끝에서 자식 쪽 끝까지의 방위각 [°]. 관절 각과 함께 이어지고(±163°에서는 반 바퀴를 넘는다),
    부호가 선이 감기는 쪽이다."""
    a0 = math.atan2(rod.end[1], rod.end[0]) - math.atan2(rod.start[1], rod.start[0])
    return _wrap(math.degrees(a0)) + q_deg


def _tight(rod: Rod, q_deg: float) -> tuple[list[tuple[float, float]], float]:
    """테에 팽팽히 감긴 길의 점들과 길이 (studies/wiring.py의 Crossing.length와 같은 셈, 높이 차이 없이)."""
    sx, sz = rod.start
    ex, ez = (float(v) for v in rot(np.array(rod.end), q_deg))
    r = rod.radius
    re, ra = max(math.hypot(sx, sz), r), max(math.hypot(ex, ez), r)
    delta = math.radians(_delta(rod, q_deg))
    sgn = 1.0 if delta >= 0 else -1.0
    wrapped = abs(delta) - math.acos(r / re) - math.acos(r / ra)
    if wrapped <= 0:
        return [(sx, sz), (ex, ez)], math.hypot(ex - sx, ez - sz)
    t1 = math.atan2(sz, sx) + sgn * math.acos(r / re)
    steps = max(2, int(wrapped / math.radians(4)))
    arc = [
        (r * math.cos(t1 + sgn * wrapped * k / steps), r * math.sin(t1 + sgn * wrapped * k / steps))
        for k in range(steps + 1)
    ]
    length = math.sqrt(re * re - r * r) + r * wrapped + math.sqrt(ra * ra - r * r)
    return [(sx, sz), *arc, (ex, ez)], length


def tight_length(rod: Rod, q_deg: float) -> float:
    return _tight(rod, q_deg)[1]


def longest(rod: Rod) -> float:
    """두 멈춤 가운데 팽팽한 길이가 긴 쪽."""
    return max(tight_length(rod, -STOP), tight_length(rod, STOP))


def servo_rod(spare: float = SPARE) -> Rod:
    """J5를 넘는 서보 선 (studies/wiring.py의 crossings): 손목 1 앞벽의 가운데 홈(살 쪽, 180°)에서 축 쪽으로 나와
    테에 감기고, 손목 2의 뒷덮개 쪽 모서리로 들어간다."""
    c = w.j5_in_layer()
    a_out, a_in = math.radians(c.exit_az), math.radians(c.entry_az)
    rod = Rod(
        start=(c.exit_r * math.cos(a_out), c.exit_r * math.sin(a_out)),
        lead=(-math.cos(a_out), -math.sin(a_out)),
        end=(c.entry_r * math.cos(a_in), c.entry_r * math.sin(a_in)),
        length=0.0,
        radius=c.drum_r + j.WIRE_T / 2,
        half=j.WIRE_T / 2,
    )
    return replace(rod, length=longest(rod) + spare)


def tail_mm(rod: Rod, cable_mm: float = 200.0) -> float:
    """선 끝(플러그 뒤)이 손목 2의 뒷덮개 쪽 모서리를 지나 들어가는 길이: 손목 1 속의 길과 층 안의 선을 뺀 나머지."""
    return w.usable(cable_mm) - w.length(w.wrist_out()) - rod.length


def _tight_points(rod: Rod, q_deg: float, n: int) -> Arr:
    """팽팽한 길을 n개 점으로 (처음 모양)."""
    return _resample(np.array(_tight(rod, q_deg)[0]), n)


def _resample(pts: Arr, n: int) -> Arr:
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    t = np.linspace(0.0, s[-1], n)
    return np.stack([np.interp(t, s, pts[:, 0]), np.interp(t, s, pts[:, 1])], axis=1)


def _push_apart(p: Arr, dmin: float, skip: int) -> Arr:
    """같은 끈에서 skip 마디보다 먼 두 점이 dmin보다 가까우면 밀어낼 몫."""
    d = p[:, None, :] - p[None, :, :]
    dist = np.linalg.norm(d, axis=2)
    idx = np.arange(len(p))
    bad = (dist < dmin) & (np.abs(idx[:, None] - idx[None, :]) > skip)
    if not bad.any():
        return np.zeros_like(p)
    k = np.where(bad, (dmin - dist) / np.maximum(dist, 1e-6), 0.0)[:, :, None] * d * 0.5
    return k.sum(axis=1)


def _relax(p: Arr, rod: Rod, q_deg: float, iters: int) -> None:
    """끈을 관절 각 q에서 푼다 (제자리에서 고친다)."""
    ear_u = np.array([math.cos(math.radians(q_deg)), math.sin(math.radians(q_deg))])
    ear_a, ear_b = ear_u * j.R_DRUM, ear_u * EAR_OUT
    end = rot(np.array(rod.end), q_deg)
    seg = rod.length / (ROD_N - 1)
    f = 2 if rod.lead else 1  # 부모 쪽에서 잡힌 점 수

    def pin() -> None:
        p[0], p[-1] = rod.start, end
        if rod.lead:
            p[1] = np.array(rod.start) + np.array(rod.lead) * seg

    for _ in range(iters):
        # 굽힘을 고르게 펴고, 남는 선은 바깥으로 조금씩 민다: 선이 뻣뻣해서 느슨한 몫이 테에 달라붙지 않고 테
        # 둘레에 고르게 뜬다 (멈춤 귀가 지나가는 반지름 밖으로)
        p[f:-1] += 0.25 * (0.5 * (p[f - 1 : -2] + p[f + 1 :]) - p[f:-1])
        rr = np.linalg.norm(p[f:-1], axis=1, keepdims=True)
        p[f:-1] += PRESS * p[f:-1] / np.maximum(rr, 1e-6)
        for _k in range(PASSES):  # 마디 길이: 짝수 마디와 홀수 마디를 번갈아 맞춘다
            for odd in (0, 1):
                a = p[odd:-1:2]
                b = p[odd + 1 :: 2]
                m = min(len(a), len(b))
                d = b[:m] - a[:m]
                dist = np.linalg.norm(d, axis=1, keepdims=True)
                c = (dist - seg) / np.maximum(dist, 1e-9) * d * 0.5
                p[odd : odd + 2 * m : 2] += c
                p[odd + 1 : odd + 1 + 2 * m : 2] -= c
                pin()
        rr = np.linalg.norm(p, axis=1)
        low = rr < rod.radius
        p[low] *= (rod.radius / rr[low])[:, None]
        # 멈춤 귀 (테에서 바깥으로 나온 판, 반폭 EAR_HW)
        ab = ear_b - ear_a
        t = np.clip(((p - ear_a) @ ab) / (ab @ ab), 0.0, 1.0)
        near = ear_a + t[:, None] * ab
        dv = p - near
        dd = np.linalg.norm(dv, axis=1)
        lim = j.EAR_HW + rod.half
        hit = dd < lim
        p[hit] = near[hit] + dv[hit] / np.maximum(dd[hit], 1e-6)[:, None] * lim
        p += _push_apart(p, 2 * rod.half + 0.3, skip=3)  # 자기와 겹치지 않게
        pin()


def solve(rod: Rod, angles: Sequence[float], sub: int = 4, iters: int = 70) -> list[Arr]:
    """관절 각마다 끈의 모양 [(ROD_N, 2) 배열]. 첫 각에서 팽팽한 길로 시작해 각 사이를 sub칸으로 나눠 이어 푼다."""
    p = _tight_points(rod, angles[0], ROD_N)
    _relax(p, rod, angles[0], iters * 4)
    out = [p.copy()]
    for a0, a1 in itertools.pairwise(angles):
        for k in range(1, sub + 1):
            _relax(p, rod, a0 + (a1 - a0) * k / sub, iters)
        out.append(p.copy())
    return out


def wrap_angles(step: float = 4.0) -> list[float]:
    """선을 붙이는 멈춤(-163°)에서 반대쪽 멈춤까지 step°쯤 간격의 관절 각."""
    n = math.ceil(2 * STOP / step) + 1
    return [float(a) for a in np.linspace(LONG_STOP, -LONG_STOP, n)]


def flex(rod: Rod | None = None, angles: Sequence[float] | None = None) -> Flex:
    """작업 안내가 관절 각에 맞춰 그리는 층 안의 서보 선 (model.Flex)."""
    rod = rod or servo_rod()
    angles = list(angles or wrap_angles())
    frames = solve(rod, angles)
    if angles[0] > angles[-1]:
        angles, frames = angles[::-1], frames[::-1]
    return Flex(
        angles=tuple(angles),
        frames=tuple(tuple((float(x), float(z)) for x, z in fr) for fr in frames),
        width=j.WIRE_W,
        thick=j.WIRE_T,
        y_flat=FLAT_Y,
        y_edge=LAYER_Y,
        flat_mm=FLAT_MM,
        edge_mm=EDGE_MM,
    )
