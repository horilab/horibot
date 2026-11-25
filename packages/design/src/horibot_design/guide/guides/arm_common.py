"""위팔, 아래팔, 손목 잇기 안내가 같이 쓰는 장면 도구 (0029의 링크와 STS 관절, 0031의 선).

팔 부품은 placement의 틀(관절마다의 서보 좌표)로 놓는다. 장면 좌표는 J1의 서보 좌표다 (parts/base.py: x 로봇 앞,
y 위, z 로봇 오른쪽). 자세는 범위 가운데(운동학 각 0, 0, 0, 180, 0, 0: 팔이 곧게 선다)이고, 안내마다 관절 하나를 둘째
관절(axis2)로 돌린다. 장면의 첫째 관절은 J1이다 (base_pi와 같다). 링크 끝 관절(J3, J4)을 조립하는 단계는 링크를 손에
든 단계라 그 링크만 보이고(focus), 손에 드는 방향은 그 관절의 서보 좌표로 정한다 (link_view).

링크 끝 관절의 선 (0031, 2026-11-25): 서보를 넣으면 서보 먼 끝과 목 끝 벽 사이가 2.3 mm라(서보 먼 끝 35.1, 벽 안쪽
37.4) 플러그(두께 4.75)가 지나가지 못한다. 그래서 그 관절에 드나드는 선 둘(관 속에서 오는 선, 선 창으로 나가는 선)은
서보를 넣기 전에 다 꿰어 두고, 서보를 잠근 뒤 커넥터에 꽂는다 (뒤에서는 서보 받침 판의 터진 쪽으로 꽂힌다).
"""

from __future__ import annotations

import functools
import itertools
from collections.abc import Iterable, Sequence
from dataclasses import dataclass

import numpy as np
from build123d import Compound

from horibot_design import placement as pl
from horibot_design.guide.model import ScenePart, Vec, View
from horibot_design.parts import joint as j
from horibot_design.parts import links
from horibot_design.parts.shapes import box, cyl_y, rod, solid
from horibot_design.robot import Mat
from horibot_design.studies import wiring as w

POSE: tuple[float, ...] = tuple(float(v) for v in pl.RANGE_MID_DEG)
# 쉬는 자세 (0037): J1은 장면의 첫째 관절로 돌리고, J2 ~ J6만 여기서 놓는다
REST: tuple[float, ...] = (0.0, 52.0, 154.0, 325.0, 0.0, 0.0)
REST_J1 = -pl.DESIGN_SIGN[0] * 55.0  # 운동학 J1 -55°를 장면의 첫째 관절 각(설계 각)으로: +55
_J1_FROM_BASE: Mat = np.linalg.inv(pl.T_BASE_J1)
WIRE_R = 1.1  # 그림의 서보 선 굵기 (반지름). 실제는 4.75 × 1.5 mm 납작한 선이다
PHOTO = "막히면 그 자리를 사진으로 보내 주세요."


@functools.cache
def frames(q: tuple[float, ...] = POSE) -> dict[str, Mat]:
    """운동학 각 q [°]에서 틀마다 장면(J1 서보) 좌표로의 변환."""
    return {k: _J1_FROM_BASE @ T for k, T in pl.frames_mm(np.radians(q)).items()}


def put(shape: Compound, frame: str, q: tuple[float, ...] = POSE) -> Compound:
    return solid(shape.moved(pl.location_of(frames(q)[frame])))


def pt(frame: str, p: Sequence[float], q: tuple[float, ...] = POSE) -> Vec:
    v = frames(q)[frame] @ np.array([*p, 1.0])
    return (round(float(v[0]), 3), round(float(v[1]), 3), round(float(v[2]), 3))


def vec(frame: str, d: Sequence[float], q: tuple[float, ...] = POSE) -> Vec:
    v = frames(q)[frame][:3, :3] @ np.array(d, dtype=float)
    return (round(float(v[0]), 4), round(float(v[1]), 4), round(float(v[2]), 4))


def axis_of(frame: str) -> tuple[Vec, Vec]:
    """둘째 관절의 축: 그 관절 서보 좌표의 원점과 +y(출력 쪽). 둘째 관절 각은 설계 각(joint.turned)이다."""
    return pt(frame, (0.0, 0.0, 0.0)), vec(frame, (0.0, 1.0, 0.0))


def link_view(frame: str, up: Sequence[float], face: Sequence[float], el: float = 28.0) -> View:
    """관절 서보 좌표의 방향으로 정한 손에 드는 방향 (링크를 따로 들고 하는 단계)."""
    return View(vec(frame, up), vec(frame, face), el, 0.0)


def mid_view(up: Sequence[float], face: Sequence[float], el: float = 25.0) -> View:
    return View(tuple(float(v) for v in up), tuple(float(v) for v in face), el, 0.0)  # type: ignore[arg-type]


# ------------------------------------------------------------------------------------------------
# 장면 부품


@dataclass(frozen=True)
class Piece:
    """장면에 놓은 부품 하나. frame은 같이 도는 관절을 가리는 데만 쓴다 (placement의 틀 이름)."""

    name: str
    shape: Compound
    key: str
    color: str
    frame: str
    count: bool = True

    def scene(self, axis_joint: int) -> ScenePart:
        """axis_joint는 이 안내의 둘째 관절 번호다. 베이스에 서 있는 것("j1")만 첫째 관절(J1)과 같이 돌지 않는다."""
        link = pl.link_of(self.frame)
        moving = link >= 1
        return ScenePart(
            self.name,
            self.shape,
            self.key,
            self.color,
            moving,
            self.count,
            moving2=link >= axis_joint,
        )


def scene_of(pieces: Iterable[Piece], axis_joint: int) -> list[ScenePart]:
    return [p.scene(axis_joint) for p in pieces]


def _sts_name(rest: str, jname: str) -> tuple[str, str, bool] | None:
    """placement의 관절 부품 이름(묶음 뒤) → (이 안내의 이름, 목록 키, 개수에 넣는지). 키의 서보와 뒷면 나사는
    부르는 쪽이 바꾼다 ("servo", "rear"). 쓰지 않으면 None."""
    k = rest.rsplit(" ", 1)[-1]
    if rest in ("플러그", "선 자리"):
        return None  # 커넥터 둘의 플러그와 눕힌 선 자리: 선은 안내가 따로 그린다
    table = {
        "베어링 6806": ("베어링", "bearing_6806", True),
        "허브 판": ("허브 판", "hub_plate", True),
        "출력 원판": ("출력 원판", "out_disc", True),
        "누름 틀": ("누름 틀", "retainer", True),
        "서보 받침": ("서보 받침", "carrier", True),
        "뒷덮개": ("뒷덮개", "link_cover", True),
        "서보 몸통": ("서보 몸통", "servo", True),
        "혼": ("혼", "servo", False),
        "멈춤 나사 M3x10": ("멈춤 나사", "m3x10", True),
        "너트 (멈춤 기둥)": ("너트 (멈춤 기둥)", "nut_m3", True),
        "너트 (누름 틀)": ("너트 (누름 틀)", "nut_m3", True),
        "너트 (서보 받침)": ("너트 (서보 받침)", "nut_m3", True),
    }
    if rest in table:
        name, key, count = table[rest]
        return (f"{jname} {name}", key, count)
    if rest.startswith("벽 나사"):
        return (f"{jname} 벽 나사 {k}", "m3x8", True)
    if rest.startswith("혼 볼트"):
        return (f"{jname} 혼 볼트 {k}", "m3x20", True)
    if rest.startswith("너트 (출력 원판)"):
        return (f"{jname} 너트 (출력 원판) {k}", "nut_m3", True)
    if rest.startswith("서보 뒷면 나사"):
        return (f"{jname} 서보 뒷면 나사 {k}", "rear", True)
    if rest.startswith("덮개 나사"):
        return (f"{jname} 덮개 나사 {k}", "m3x12", True)
    if rest.startswith("위팔 나사"):
        return (f"위팔 나사 {k}", "m3x10", True)
    raise ValueError(f"모르는 관절 부품: {rest}")


def link_pieces(
    group: str, servo_key: str, rear_key: str, q: tuple[float, ...] = POSE, prefix: str = ""
) -> list[Piece]:
    """링크 하나(placement의 묶음 "upper_arm" 또는 "forearm"): 링크 몸통, 끝 관절(J3, J4)의 속 부품과 나사, 위팔이면
    어깨 원판에 다는 나사."""
    link_key = group
    jname = "J3" if group == "upper_arm" else "J4"
    out: list[Piece] = []
    seen: dict[str, int] = {}
    for p in pl.group(group):
        rest = p.name.split("/", 1)[1]
        if rest == pl.LINK_NAME[group]:
            out.append(
                Piece(f"{prefix}{rest}", put(p.shape, p.frame, q), link_key, "body", p.frame)
            )
            continue
        got = _sts_name(rest, jname)
        if got is None:
            continue
        name, key, count = got
        if rest in ("너트 (누름 틀)", "너트 (서보 받침)"):
            # 이름이 같은 너트에 나온 차례로 번호를 붙인다: 누름 틀 1 +z, 2 -z, 3 +x / 서보 받침 1 +z, 2 -z
            seen[name] = seen.get(name, 0) + 1
            name = f"{name} {seen[name]}"
        key = servo_key if key == "servo" else rear_key if key == "rear" else key
        out.append(Piece(f"{prefix}{name}", put(p.shape, p.frame, q), key, p.color, p.frame, count))
    return out


def group_pieces(
    group: str, key: str, q: tuple[float, ...] = POSE, prefix: str = ""
) -> list[Piece]:
    """placement의 묶음 하나를 이름만 붙여 그대로 놓는다 (조립해 둔 것: 개수에 넣지 않는다)."""
    return [
        Piece(f"{prefix}{p.name}", put(p.shape, p.frame, q), key, p.color, p.frame, False)
        for p in pl.group(group)
        if "플러그" not in p.name.split("/", 1)[1] and "선 자리" not in p.name
    ]


# ------------------------------------------------------------------------------------------------
# 서보 선과 영점 핀


def wire(points: Sequence[Sequence[float]], frame: str, q: tuple[float, ...] = POSE) -> Compound:
    """꺾은선(틀의 좌표)을 따라가는 서보 선 그림 (둥근 막대를 이은 것)."""
    pts = [pt(frame, p, q) for p in points]
    return wire_scene(pts)


def wire_scene(pts: Sequence[Vec]) -> Compound:
    """장면 좌표 꺾은선의 서보 선 그림."""
    shape: Compound | None = None
    for a, b in itertools.pairwise(pts):
        if np.linalg.norm(np.subtract(b, a)) < 0.05:
            continue
        seg = rod(a, b, WIRE_R)
        shape = seg if shape is None else solid(shape + seg)
    if shape is None:
        raise ValueError("선이 비었다")
    return shape


def plug(side: int, model: str, frame: str, q: tuple[float, ...] = POSE) -> Compound:
    """서보 뒷면의 커넥터(side: +1이면 +z 쪽, -1이면 -z 쪽)에 꽂은 플러그 (그 관절의 서보 좌표 틀)."""
    dy = j.servo_dy(model)  # type: ignore[arg-type]
    z0, z1 = (0.45, 9.46) if side > 0 else (-9.46, -0.45)
    return put(box(-16.6, -11.85, j.PLUG_TOP + dy, j.L1 + 0.5 + dy, z0, z1), frame, q)


def loose_plug(at: Vec, along_dir: Vec) -> Compound:
    """선 끝에 매달린 플러그 (장면 좌표 at에서 along_dir 쪽으로 9.6 mm). 모양은 둥근 막대로 줄였다."""
    end = tuple(a + 9.6 * d for a, d in zip(at, along_dir, strict=True))
    return rod(at, end, 3.0)  # type: ignore[arg-type]


def link_pin(group: str) -> Compound:
    """링크 발의 영점 핀 (그 링크의 틀 좌표): 귀 위로 3 mm 나오고 부모의 살에 5 mm 든다 (범위 가운데에 꽂는다)."""
    length, _r, foot = links.LINKS[group]
    top = j.FRONT - j.EAR_T  # 귀 윗면 (부모 원판에서 먼 쪽이 -y)
    return cyl_y(1.5, top - 3.0, top - 3.0 + 20.0, -length - foot.pin_r, 0.0)


Pt = tuple[float, float, float]


def _pts(points: Iterable[Sequence[float]]) -> list[Pt]:
    return [(float(p[0]), float(p[1]), float(p[2])) for p in points]


def link_in(group: str, high_window: bool = True) -> list[Pt]:
    """링크 발 → 관 속 → 끝 관절의 -z 커넥터 (0031). high_window면 발 끝벽의 높은 창으로 들어온다 (발 겉으로 도는 길,
    J2·J3의 주된 길). 아니면 끝벽 밑의 낮은 구멍이다 (테에 감는 길)."""
    run = _pts(w.link_in(group))
    if not high_window:
        return run
    length, _r, foot = links.LINKS[group]
    x_wall = -length - foot.hx
    z = sum(links.FOOT_SLOT_Z) / 2
    y_win = j.FRONT - sum(links.FOOT_WINDOW) / 2
    return [
        (x_wall - 6.0, y_win, z),
        (x_wall, y_win, z),
        (x_wall + 1.3, j.FRONT - 1.3, z),
        *run[1:],
    ]


def window_point(out: float = 0.0) -> tuple[float, float, float]:
    """링크 끝 집의 선 창 (목 앞면) 가운데, out만큼 밖 (서보 좌표)."""
    return (sum(j.WIN_X) / 2, j.FRONT + out, 0.0)


def link_out_run(spare_x: float | None = None, yc: float | None = None) -> list[Pt]:
    """링크 끝 관절의 +z 커넥터 → 서보 먼 끝을 돌아 → 선 창 (0031의 link_out). spare_x를 주면 남는 선을 관 속 x까지 접어
    넣었다 나오는 고리를 끼운다 (위팔: J3 → J4 선의 남는 길이를 관 가운데에 둔다)."""
    run = _pts(w.link_out())
    if spare_x is None or yc is None:
        return run
    win = sum(j.WIN_X) / 2
    loop: list[Pt] = [
        (links.NECK_END - 6.0, yc + 4.0, 3.0),
        (spare_x, yc + 4.0, 3.0),
        (spare_x, yc + 4.0, -3.0),
        (links.NECK_END - 6.0, yc + 4.0, -3.0),
        (win, j.NECK_FRONT_IN - 1.0, 0.0),
    ]
    return [*run[:4], *loop, run[-1]]


def tube_center_y(group: str) -> float:
    _length, tube_r, _foot = links.LINKS[group]
    return j.FRONT - (tube_r - links.EMBED)


def exact(names: Iterable[str]) -> tuple[str, ...]:
    """이름이 꼭 같은 것만 고르는 선택자들."""
    return tuple(f"{n}$" for n in names)
