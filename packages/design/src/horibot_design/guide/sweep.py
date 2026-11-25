"""끼우는 길 검사: 단계의 움직임(Move)마다 부품을 들어오는 길을 따라 밀어, 그때 보이는 부품과 겹치는지 본다.

제자리 겹침과 손이 닿는지만 봐서는 넣는 길에서 걸리는 것을 잡지 못한다 (J1의 허브와 누름 틀, 2026-11-24). 그래서
움직이는 부품을 by에서 제자리까지 곧게 옮기며 몇 군데에서 겹친 부피를 잰다. 안내의 움직임이 실제로 넣는 길과 같아야
뜻이 있다 (안내를 쓸 때 그렇게 쓴다).

빼고 보는 것: 나사, 너트, 와셔(목록의 hardware: 나사는 조여 넣는 구멍과 겹치게 그린다), 선(색 구실 wire: 휜다),
같은 단계에서 같이 움직이는 부품끼리. 제자리 바로 앞(LAST_MM)까지만 잰다: 끼움으로 겹치게 그린 곳이 있다. 플러그(색
구실 plug)는 단자 속으로 들어가는 길이(PLUG_MATE_MM)를 빼고 잰다: 단자를 속이 찬 상자로 그렸다.

단계가 관절 각(joint, joint2)을 주면 부품을 화면처럼 그 자세로 돌려 놓고 잰다 (영점 핀은 관절을 핀 자세로 돌린
단계에서 꽂는다).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from build123d import Axis, Compound, Pos

from horibot_design.guide.build import Box, Resolved

STEP_MM = 2.0  # 길을 따라 재는 간격
LAST_MM = 0.4  # 제자리에서 이만큼 앞까지 잰다
MIN_VOLUME = 0.5  # 이보다 큰 겹침만 알린다 [mm³]
PLUG_MATE_MM = 7.0  # 플러그가 단자 속으로 들어가는 길이 (서보 플러그, USB-C)


@dataclass(frozen=True)
class Hit:
    step: str
    part: str
    other: str
    before_mm: float  # 제자리에서 이만큼 앞에서 처음 겹친다
    volume: float  # 겹친 부피의 최대 [mm³]


def _skip(r: Resolved, i: int) -> bool:
    p = r.parts[i]
    return r.items[p.key].kind == "hardware" or p.color == "wire" or p.flex is not None


def _meet(a: Box, b: Box, shift: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> bool:
    (a0, a1), (b0, b1) = a, b
    return all(a0[k] + shift[k] < b1[k] and b0[k] < a1[k] + shift[k] for k in range(3))


def _swept(box: Box, by: tuple[float, float, float]) -> Box:
    lo, hi = box
    return (
        tuple(min(lo[k], lo[k] + by[k]) for k in range(3)),  # type: ignore[return-value]
        tuple(max(hi[k], hi[k] + by[k]) for k in range(3)),
    )


def _volume(shape: Compound) -> float:
    return sum(s.volume for s in shape.solids()) if shape else 0.0


def _box(shape: Compound) -> Box:
    bb = shape.bounding_box()
    return ((bb.min.X, bb.min.Y, bb.min.Z), (bb.max.X, bb.max.Y, bb.max.Z))


def _posed(r: Resolved, step: dict[str, Any], i: int) -> Compound:
    """단계의 관절 각대로 놓은 부품. 화면과 같이 둘째 관절을 축(axis2) 둘레로 돌린 뒤 첫째 관절을 장면의 y축 둘레로
    돌린다 (관절 각 q는 그 축 둘레로 -q, parts/joint.py의 turned)."""
    p = r.parts[i]
    shape = p.shape
    q1, q2 = float(step.get("joint") or 0.0), float(step.get("joint2") or 0.0)
    if p.moving2 and q2 and r.guide.axis2 is not None:
        o, d = r.guide.axis2
        shape = shape.rotate(Axis(o, d), -q2)
    if p.moving and q1:
        shape = shape.rotate(Axis((0.0, 0.0, 0.0), (0.0, 1.0, 0.0)), -q1)
    return shape


def _turn(
    v: tuple[float, float, float], axis: tuple[float, float, float], deg: float
) -> tuple[float, float, float]:
    """벡터를 축 둘레로 deg만큼 돌린다 (오른손 법칙, 로드리게스 식)."""
    n = math.sqrt(sum(a * a for a in axis))
    k = (axis[0] / n, axis[1] / n, axis[2] / n)
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    kv = sum(k[m] * v[m] for m in range(3))
    kxv = (k[1] * v[2] - k[2] * v[1], k[2] * v[0] - k[0] * v[2], k[0] * v[1] - k[1] * v[0])
    return tuple(v[m] * c + kxv[m] * s + k[m] * kv * (1 - c) for m in range(3))  # type: ignore[return-value]


def _posed_by(
    r: Resolved, step: dict[str, Any], i: int, by: tuple[float, float, float]
) -> tuple[float, float, float]:
    """움직임 방향도 부품과 같이 돌린다: 화면은 도는 관절 묶음의 좌표로 부품을 옮긴다."""
    p = r.parts[i]
    q1, q2 = float(step.get("joint") or 0.0), float(step.get("joint2") or 0.0)
    if p.moving2 and q2 and r.guide.axis2 is not None:
        by = _turn(by, r.guide.axis2[1], -q2)
    if p.moving and q1:
        by = _turn(by, (0.0, 1.0, 0.0), -q1)
    return by


def check(r: Resolved) -> list[Hit]:
    """안내 하나의 끼우는 길에서 겹치는 곳."""
    hits: list[Hit] = []
    for step in r.steps:
        if not step["moves"]:
            continue
        movers = {i for mv in step["moves"] for i in mv["p"]}
        obstacles = [i for i in step["show"] if i not in movers and not _skip(r, i)]
        posed = bool(step.get("joint") or step.get("joint2"))
        shapes: dict[int, Compound] = {}
        boxes: dict[int, Box] = {}
        for i in {*movers, *obstacles}:
            shapes[i] = _posed(r, step, i) if posed else r.parts[i].shape
            boxes[i] = _box(shapes[i]) if posed else r.boxes[i]
        for mv in step["moves"]:
            by0: tuple[float, float, float] = tuple(float(v) for v in mv["by"])  # type: ignore[assignment]
            dist = math.sqrt(sum(v * v for v in by0))
            for i in mv["p"]:
                if _skip(r, i):
                    continue
                by = _posed_by(r, step, i, by0) if posed else by0
                last = PLUG_MATE_MM if r.parts[i].color == "plug" else LAST_MM
                if dist <= last:
                    continue
                n = max(1, math.ceil((dist - last) / STEP_MM))
                ts = [dist - k * (dist - last) / n for k in range(n + 1)]  # 먼 데서 제자리 쪽으로
                shape = shapes[i]
                path = _swept(boxes[i], by)  # type: ignore[arg-type]
                for o in obstacles:
                    if not _meet(path, boxes[o]):
                        continue
                    first: float | None = None
                    worst = 0.0
                    for t in ts:
                        f = t / dist
                        shift = (by[0] * f, by[1] * f, by[2] * f)
                        if not _meet(boxes[i], boxes[o], shift):
                            continue
                        vol = _volume(Pos(*shift) * shape & shapes[o])
                        if vol > MIN_VOLUME:
                            first = t if first is None else first
                            worst = max(worst, vol)
                    if first is not None:
                        hits.append(Hit(step["id"], r.parts[i].name, r.parts[o].name, first, worst))
    return hits
