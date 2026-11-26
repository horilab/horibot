"""나사에 공구가 닿는지: 단계에서 조이는 나사(움직임이 있는 나사)마다, 머리 위로 곧은 렌치가 그때 보이는 부품에
막히지 않는지 본다.

끼우는 길 검사(sweep)는 나사와, 같은 단계에서 같이 움직이는 부품끼리를 보지 않아서 공구가 막히는 곳을 잡지 못한다
(2026-11-25: ㄱ자 젠더가 파이 판 나사 바로 위를 덮었고, 손목 잇기의 발 나사는 머리 위 14 mm에서 손목 1 몸통에 막혔다).

- 곧은 렌치: 나사 축을 따라 머리 위로 REACH_MM 길이의 기둥이다. 반지름은 나사 머리에 맞는 공구로 정한다 (TOOL_R).
- 막히면 L자 렌치를 찾는다: 짧은 팔(축 방향)이 머리 위 SHORT_MM에서 꺾이고, 긴 팔(LONG_MM)이 축에 수직인 방향 하나로
  나간다. 짧은 팔이 막히거나 긴 팔이 나갈 방향이 없으면 그 자리에는 렌치가 들어가지 않는다.
- 긴 팔이 어느 방향으로도 나가면 손으로 찾아 돌리면 된다. 일부 방향으로만 나가면, 그 단계에 렌치(부품 목록 키 hexkey인
  장면 부품)를 그려 넣는 방향을 보여야 한다. 그린 렌치는 나사 말고 다른 부품과 겹치지 않아야 한다.

조이는 나사는 그 단계에서 처음 보이며 움직여 들어오는 나사다 (그림이 나사인 것: draw의 t가 screw나 tap). 앞 단계에서
보인 나사는 이미 조인 것이라, 묶음째 들어올 때 다시 보지 않는다. 빼고 보는 것은 선(색 구실 wire, 손으로 비킨다)과 조이는
나사 자신이다. 단계가 관절 각을 주면 끼우는 길 검사처럼 그 자세로 돌려 놓고 본다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from build123d import Compound, Plane, Solid, Vector

from horibot_design.guide.build import Box, Resolved
from horibot_design.guide.sweep import _box, _meet, _posed, _posed_by

# 나사 그림(draw의 t)마다 공구 기둥의 반지름: 둥근머리 M3는 2 mm 육각(꼭짓점 반지름 1.15 mm), 유두 M3는 2.5 mm 육각
# (1.44 mm), 십자 자가탭은 작은 십자 드라이버(축 지름 3 mm로 어림)
TOOL_R = {"screw": 1.2, "cap": 1.5, "tap": 1.5}
KEY_R = TOOL_R["screw"]  # 그려 넣는 L자 렌치의 반지름
REACH_MM = 40.0  # 곧은 렌치가 머리 위로 비어야 하는 길이
# L자 렌치의 짧은 팔이 꺾이는 높이: 머리 윗면에서 긴 팔 가운데까지 (가정이다, 가진 렌치를 재지 않았다)
SHORT_MM = 12.0
LONG_MM = 45.0  # L자 렌치의 긴 팔
TURN_STEP = 15  # 긴 팔 방향을 찾는 간격 [°]
MIN_VOLUME = 0.01  # 이보다 큰 겹침만 막힘으로 본다 [mm³]
HINT_KEY = "hexkey"


@dataclass(frozen=True)
class Reach:
    step: str
    screw: str
    blocked_mm: float | None  # 곧은 렌치가 처음 막히는 머리 위 높이 (None이면 막히지 않는다)
    by: tuple[str, ...]  # 막는 부품
    turns: tuple[int, ...]  # L자 렌치의 긴 팔이 나갈 수 있는 방향 [°] (막힐 때만 찾는다)


@dataclass(frozen=True)
class Problem:
    step: str
    text: str


def _screws(r: Resolved) -> dict[int, float]:
    """나사인 장면 부품과 그 공구 기둥의 반지름."""
    out: dict[int, float] = {}
    for i, p in enumerate(r.parts):
        item = r.items[p.key]
        if item.kind == "hardware" and item.draw.get("t") in TOOL_R:
            out[i] = TOOL_R[item.draw["t"]]
    return out


def _unit(v: tuple[float, float, float]) -> Vector:
    n = math.sqrt(sum(a * a for a in v))
    return Vector(v[0] / n, v[1] / n, v[2] / n)


def head_top(shape: Compound | Solid, out: Vector) -> Vector:
    """나사 머리 윗면의 가운데: 무게 중심(축 위)에서 바깥쪽(out)으로 가장 먼 꼭짓점까지 간 자리."""
    c = shape.center()
    far = max((Vector(v.X, v.Y, v.Z) - c).dot(out) for v in shape.vertices())
    return c + out * far


def _basis(out: Vector) -> tuple[Vector, Vector]:
    """축(out)에 수직인 두 단위 벡터. 장면의 y(위)를 먼저 쓰고, 축이 y와 나란하면 x를 쓴다."""
    ref = Vector(0, 1, 0) if abs(out.Y) < 0.9 else Vector(1, 0, 0)
    e1 = (ref - out * ref.dot(out)).normalized()
    return e1, out.cross(e1)


def l_key(
    top: Vector,
    out: Vector,
    turn_deg: float,
    radius: float = KEY_R,
    short: float = SHORT_MM,
    long: float = LONG_MM,
) -> Solid:
    """L자 렌치: 머리 윗면 top에서 축(out)을 따라 짧은 팔, 꺾여서 축에 수직인 방향 turn_deg로 긴 팔.

    turn_deg는 _basis의 첫 벡터(장면의 위를 축에 수직으로 눕힌 것)에서 둘째 벡터 쪽으로 잰 각이다. 짧은 팔은 머리의
    육각 구멍에 0.5 mm 들어간 데서 시작한다."""
    e1, e2 = _basis(out)
    t = math.radians(turn_deg)
    u = e1 * math.cos(t) + e2 * math.sin(t)
    arm = Solid.make_cylinder(
        radius, short + radius + 0.5, Plane(origin=top - out * 0.5, z_dir=out)
    )
    bend = top + out * short
    return arm.fuse(Solid.make_cylinder(radius, long, Plane(origin=bend, z_dir=u)))  # type: ignore[return-value]


def _bbox(shape: Compound | Solid) -> Box:
    bb = shape.bounding_box()
    return ((bb.min.X, bb.min.Y, bb.min.Z), (bb.max.X, bb.max.Y, bb.max.Z))


def _hits(
    tool: Compound | Solid, others: list[int], shapes: dict[int, Compound], boxes: dict[int, Box]
) -> list[tuple[int, Any]]:
    """tool과 겹치는 부품과 겹친 형상."""
    box = _bbox(tool)
    out: list[tuple[int, Any]] = []
    for o in others:
        if not _meet(box, boxes[o]):
            continue
        cut = tool & shapes[o]
        if cut and sum(s.volume for s in cut.solids()) > MIN_VOLUME:
            out.append((o, cut))
    return out


def reaches(r: Resolved) -> list[Reach]:
    """단계마다 조이는 나사의 렌치 자리."""
    screws = _screws(r)
    out: list[Reach] = []
    seen: set[int] = set()
    for step in r.steps:
        # 조이는 나사: 이 단계에서 처음 보이며 움직여 들어오는 나사 (앞 단계에서 보인 나사는 이미 조인 것이다)
        moving = [(i, mv) for mv in step["moves"] for i in mv["p"] if i in screws and i not in seen]
        seen.update(step["show"])
        if not moving:
            continue
        posed = bool(step.get("joint") or step.get("joint2"))
        shown = [
            i
            for i in step["show"]
            if r.parts[i].color != "wire" and r.parts[i].flex is None and r.parts[i].key != HINT_KEY
        ]
        shapes: dict[int, Compound] = {}
        boxes: dict[int, Box] = {}
        for i in {*shown, *(i for i, _ in moving)}:
            shapes[i] = _posed(r, step, i) if posed else r.parts[i].shape
            boxes[i] = _box(shapes[i]) if posed else r.boxes[i]
        for i, mv in moving:
            by: tuple[float, float, float] = tuple(float(v) for v in mv["by"])  # type: ignore[assignment]
            out_dir = _unit(_posed_by(r, step, i, by) if posed else by)
            top = head_top(shapes[i], out_dir)
            rad = screws[i]
            others = [o for o in shown if o != i]
            column = Solid.make_cylinder(
                rad, REACH_MM, Plane(origin=top + out_dir * 0.2, z_dir=out_dir)
            )
            hits = _hits(column, others, shapes, boxes)
            if not hits:
                out.append(Reach(step["id"], r.parts[i].name, None, (), ()))
                continue
            first = min(
                min((Vector(v.X, v.Y, v.Z) - top).dot(out_dir) for v in cut.vertices())
                for _, cut in hits
            )
            names = tuple(sorted({r.parts[o].name for o, _ in hits}))
            turns = tuple(
                d
                for d in range(0, 360, TURN_STEP)
                if not _hits(l_key(top, out_dir, d, rad), others, shapes, boxes)
            )
            out.append(Reach(step["id"], r.parts[i].name, max(first, 0.0), names, turns))
    return out


def check(r: Resolved, found: list[Reach] | None = None) -> list[Problem]:
    """렌치가 들어가지 않는 나사, L자로만 들어가는데 렌치 방향을 그리지 않은 단계, 다른 부품과 겹치게 그린 렌치.

    found는 reaches(r)의 결과다 (주지 않으면 여기서 구한다)."""
    problems: list[Problem] = []
    by_step: dict[str, list[Reach]] = {}
    for x in reaches(r) if found is None else found:
        by_step.setdefault(x.step, []).append(x)
    steps = {s["id"]: s for s in r.steps}
    screws = _screws(r)
    for sid, xs in by_step.items():
        blocked = [x for x in xs if x.blocked_mm is not None]
        for x in blocked:
            if not x.turns:
                problems.append(
                    Problem(
                        sid,
                        f"'{x.screw}': 곧은 렌치가 머리 위 {x.blocked_mm:.1f} mm에서 {', '.join(x.by[:3])}에 막히고, "
                        f"L자 렌치(짧은 팔 {SHORT_MM:.0f} mm)도 들어가지 않는다",
                    )
                )
        step = steps[sid]
        hints = [i for i in step["show"] if r.parts[i].key == HINT_KEY]
        some = [x for x in blocked if 0 < len(x.turns) < 360 // TURN_STEP]  # 일부 방향으로만 돌린다
        if some and not hints:
            names = ", ".join(x.screw for x in some[:4])
            problems.append(
                Problem(
                    sid,
                    f"곧은 렌치가 막히고 L자 렌치도 일부 방향으로만 돈다 ({names}: 머리 위 "
                    f"{min(x.blocked_mm or 0.0 for x in some):.1f} mm). 렌치를 넣는 방향을 그 단계에 그려 보인다 "
                    f"(키 {HINT_KEY}인 장면 부품)",
                )
            )
        posed = bool(step.get("joint") or step.get("joint2"))
        others = [
            o
            for o in step["show"]
            if o not in hints
            and o not in screws
            and r.parts[o].color != "wire"
            and r.parts[o].flex is None
        ]
        shapes = {o: _posed(r, step, o) if posed else r.parts[o].shape for o in (*others, *hints)}
        boxes = {o: _box(shapes[o]) if posed else r.boxes[o] for o in shapes}
        for h in hints:
            for o, _ in _hits(shapes[h], others, shapes, boxes):
                problems.append(
                    Problem(sid, f"그린 렌치 '{r.parts[h].name}'가 '{r.parts[o].name}'와 겹친다")
                )
    return problems
