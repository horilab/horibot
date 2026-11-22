"""팔 부품(parts/)을 관절 각대로 베이스 좌표에 놓는다.

부품은 저마다 자기 관절의 서보 좌표(mm)로 그려져 있다 (parts/joint.py). 이 모듈은 관절마다의 서보 좌표를 베이스
좌표로 옮기는 변환을 부품 코드의 쌓임(child_at, from_parent)에서 차례로 만들고, 등록한 부품을 그 변환으로 옮긴다.
닿음, 가림, 선 고리처럼 팔 전체를 한 자세에 놓고 보는 검토에 쓴다. 막대·상자 모델(studies/ur_arm.py)과 같은
골격인지는 tests/test_placement.py가 묶는다.

관절 각은 운동학의 각이다 (docs/conventions.md: 관절 축에 대한 오른손 법칙, rad). 부품 코드의 설계 각
(joint.turned: 서보 출력 쪽에서 볼 때 시계 방향이 +)으로는 관절마다 DESIGN_SIGN을 곱해 바꾼다.

틀(frame)의 이름:
- "j1" ~ "j6": 관절 k의 서보 좌표. 집과 함께 서 있다 (링크 k-1에 붙는다). "j1"은 베이스 몸통의 좌표다.
  손목 몸통과 링크는 자기가 품은 서보의 좌표로 그려져 있어서, 관절 k의 자식 몸통은 "j(k+1)"에 둔다.
- "j1_out" ~ "j6_out": 관절 k의 출력 원판과 함께 도는 좌표 (링크 k에 붙는다). 범위의 가운데에서 "jk"와 겹친다.
- "flange": 플랜지 좌표 (z가 도구 쪽, 막대 모델의 T_last_flange와 같은 뜻).

부품을 더할 때는 그 부품의 좌표에 맞는 틀을 골라 ArmPart로 만들고, 묶음 함수를 GROUPS에 넣는다. 형상은 묶음마다
한 번만 만들고(functools.cache), 자세를 놓을 때는 자리만 옮긴다 (불 연산이 없다).

    uv run python -m horilab_design.placement [q1 ... q6 (°)]
"""

from __future__ import annotations

import functools
import math
import sys
import time
from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np
from build123d import Compound, Location, Plane, Pos, Rot
from numpy.typing import ArrayLike

from horilab_design.parts import (
    base,
    camera_mount,
    flange,
    gripper,
    links,
    pi_mount,
    rest_shelf,
    shoulder,
    wire_clamps,
    wrist,
)
from horilab_design.parts import joint as j
from horilab_design.parts.shapes import far_apart, solid
from horilab_design.robot import Mat, Vec3
from horilab_design.values import ServoModel

JOINT_COUNT = 6
# 설계 각 = DESIGN_SIGN × 운동학 각. 서보 출력이 관절 축의 + 쪽을 보는 관절은 -1이다 (설계 각은 출력 쪽에서 볼 때
# 시계 방향이 +라서). 출력이 축의 - 쪽을 보는 것은 J3뿐이다. J5와 J6는 서보를 돌린 사진으로 확인했다 (2026-11-19).
# J1 ~ J4는 부품의 쌓임에서 나온 값이고, tests/test_placement.py가 막대 모델과 견주어 묶는다
DESIGN_SIGN = (-1, -1, 1, -1, -1, -1)
# 범위의 가운데 (운동학 각 [°]): 출력 원판의 멈춤 기둥이 살의 맞은편(설계 각 0)에 오는 자세다. J4는 범위의 가운데가
# 180°라서, 손목 1 몸통을 출력 원판에 다른 관절과 반 바퀴 다르게 단다 (0029)
RANGE_MID_DEG = (0.0, 0.0, 0.0, 180.0, 0.0, 0.0)
FRAMES = (
    *(f"j{k}{tail}" for k in range(1, JOINT_COUNT + 1) for tail in ("", "_out")),
    "flange",
)


def body_deg(joint: int, q: float) -> float:
    """관절 joint(1 ~ 6)의 운동학 각 q [rad]에서 자식 몸통의 설계 각 [°] (joint.turned에 주는 값)."""
    return DESIGN_SIGN[joint - 1] * math.degrees(q)


def disc_deg(joint: int, q: float) -> float:
    """관절 joint의 운동학 각 q [rad]에서 출력 원판의 설계 각 [°]. 범위의 가운데에서 0이다."""
    return DESIGN_SIGN[joint - 1] * (math.degrees(q) - RANGE_MID_DEG[joint - 1])


def turn(q_deg: float) -> Mat:
    """joint.turned와 같은 돌림: 서보 좌표의 +y 둘레로, 방위각(+x가 0°, +z가 90°)이 q만큼 커지는 쪽."""
    c, s = math.cos(math.radians(q_deg)), math.sin(math.radians(q_deg))
    T = np.eye(4)
    T[:3, :3] = [[c, 0.0, -s], [0.0, 1.0, 0.0], [s, 0.0, c]]
    return T


def matrix_of(loc: Location) -> Mat:
    """build123d의 자리(Location)를 4×4 행렬로 (옮김은 mm)."""
    o = (loc * Pos(0, 0, 0)).position
    T = np.eye(4)
    for col, unit in enumerate(((1, 0, 0), (0, 1, 0), (0, 0, 1))):
        v = (loc * Pos(*unit)).position - o
        T[:3, col] = (v.X, v.Y, v.Z)
    T[:3, 3] = (o.X, o.Y, o.Z)
    return T


def _vec(v: Mat) -> Vec3:
    return (float(v[0]), float(v[1]), float(v[2]))


def location_of(T: Mat) -> Location:
    """4×4 행렬(옮김은 mm)을 build123d의 자리로."""
    return Location(Plane(origin=_vec(T[:3, 3]), x_dir=_vec(T[:3, 0]), z_dir=_vec(T[:3, 2])))


# J1의 서보 좌표(parts/base.py: y가 위, x가 로봇 앞, 책상 윗면이 y = Y_DESK) → 베이스 좌표(z가 위, x가 작업대 쪽,
# 원점은 설치면 가운데): (x, y, z) → (x, -z, y - Y_DESK)
T_BASE_J1: Mat = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, -1.0, 0.0],
        [0.0, 1.0, 0.0, -base.Y_DESK],
        [0.0, 0.0, 0.0, 1.0],
    ]
)
# 관절 6의 출력 원판 좌표 → 플랜지 좌표: 원점은 원판 윗면의 가운데, z가 도구 쪽(원판의 +y), x가 멈춤 귀 쪽(+x)
T_J6OUT_FLANGE: Mat = np.array(
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, j.ATOP],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
)


@functools.cache
def child_matrices() -> tuple[Mat, ...]:
    """관절 k(1 ~ 5)의 자식 몸통 좌표("j(k+1)")를 관절 k의 서보 좌표로 옮기는 변환 (설계 각 0).

    부품 코드가 자식을 놓는 자리 그대로다: J1 몸통(base.child_at), 위팔(shoulder.child_at), 아래팔(위팔과 같은
    방식으로 STS 출력 원판에 앉는다), 손목 몸통 둘(wrist.from_parent의 거꾸로).
    """
    upper, fore = links.LINKS["upper_arm"][0], links.LINKS["forearm"][0]
    on_disc = wrist.from_parent().inverse()
    return (
        matrix_of(base.child_at()),
        matrix_of(shoulder.child_at(upper)),
        matrix_of(Pos(fore, j.ATOP + j.FRONT, 0) * Rot(180, 0, 0)),
        matrix_of(on_disc),
        matrix_of(on_disc),
    )


def frames_mm(q: ArrayLike) -> dict[str, Mat]:
    """관절 각 q [rad] (운동학의 각 여섯)에서 틀마다 T_base_틀. 옮김은 mm다 (부품 좌표가 mm라서)."""
    angles = np.asarray(q, dtype=float)
    if angles.shape != (JOINT_COUNT,):
        raise ValueError(f"관절 각은 {JOINT_COUNT}개여야 한다: {angles.shape}")
    children = child_matrices()
    out: dict[str, Mat] = {}
    T = T_BASE_J1
    for k in range(1, JOINT_COUNT + 1):
        angle = float(angles[k - 1])
        out[f"j{k}"] = T
        out[f"j{k}_out"] = T @ turn(disc_deg(k, angle))
        if k < JOINT_COUNT:
            T = T @ turn(body_deg(k, angle)) @ children[k - 1]
    out["flange"] = out[f"j{JOINT_COUNT}_out"] @ T_J6OUT_FLANGE
    return out


def link_of(frame: str) -> int:
    """틀이 붙은 링크 (0 베이스, k는 관절 k가 움직이는 링크)."""
    if frame == "flange":
        return JOINT_COUNT
    joint = int(frame[1])
    return joint if frame.endswith("_out") else joint - 1


@dataclass(frozen=True)
class ArmPart:
    """틀 하나에 붙은 부품. 형상은 그 틀의 좌표(mm)다."""

    name: str  # "묶음/부품" (예: "어깨/출력 원판")
    frame: str
    shape: Compound
    color: str = "body"  # 3D 그림의 색 구실 (parts/joint.py의 Part와 같다)
    hardware: bool = False  # 나사, 너트, 와셔

    @property
    def link(self) -> int:
        return link_of(self.frame)


def _from_joint(group: str, joint: int, parts: Sequence[j.Part]) -> list[ArmPart]:
    """관절 모듈의 부품 목록(서보 좌표, moving이면 출력 원판과 함께 돈다)을 틀에 붙인다. 영점 핀은 뺀다."""
    return [
        ArmPart(
            f"{group}/{p.name}",
            f"j{joint}_out" if p.moving else f"j{joint}",
            p.shape,
            p.color,
            p.density > 0,
        )
        for p in parts
        if "핀" not in p.name
    ]


def _base() -> list[ArmPart]:
    """베이스 몸통과 책상에 무는 판, J1 서보와 속 부품, 돌림판 (J1 몸통은 어깨 묶음에 있다)."""
    return _from_joint("베이스", 1, base.own_parts("STS3215") + base.hardware("STS3215"))


def _shoulder() -> list[ArmPart]:
    """J1 몸통(어깨 집과 발)과 어깨 서보, 속 부품, 출력 원판."""
    body = ArmPart("어깨/J1 몸통", "j2", base.j1_body())
    inner = shoulder.inner_parts(base.NECK_X) + shoulder.hardware()
    return [body, *_from_joint("어깨", 2, inner)]


def _link(group: str, key: str, joint: int, model: ServoModel) -> list[ArmPart]:
    """링크 하나: 앞 관절의 출력 원판에 앉은 링크와, 끝 집(관절 joint)의 서보와 속 부품."""
    body = ArmPart(f"{group}/{group}", f"j{joint}", links.body(key))
    inner = j.inner_parts(model, links.NECK_END, end_skirt=True) + j.inner_hardware(model)
    return [body, *_from_joint(group, joint, inner)]


def _upper_arm() -> list[ArmPart]:
    """위팔과 그 끝의 J3, 위팔을 어깨 출력 원판에 다는 나사."""
    length = links.LINKS["upper_arm"][0]
    screws = _from_joint("위팔", 2, shoulder.child_hardware(length))
    return _link("위팔", "upper_arm", 3, "STS3250") + screws


def _forearm() -> list[ArmPart]:
    """아래팔과 그 끝의 J4."""
    return _link("아래팔", "forearm", 4, "STS3215")


# 링크의 틀: 링크는 끝 집이 품는 서보의 좌표로 그려져 있다 (위팔은 J3, 아래팔은 J4)
LINK_FRAME = {"upper_arm": "j3", "forearm": "j4"}
LINK_NAME = {"upper_arm": "위팔", "forearm": "아래팔"}


def _letters() -> list[ArmPart]:
    """관 글자 (0019): 위팔의 BOMI와 아래팔의 HORI. 낱자마다 링크의 구멍에 꽂혀 있다."""
    return [
        ArmPart(f"글자/{links.LABELS[key]}", LINK_FRAME[name], shape, "cap")
        for name in links.LINKS
        for key, shape in links.letters(name).items()
    ]


def _d405_run() -> list[ArmPart]:
    """링크에 찍찍이 타이로 묶은 D405 선 (0031): 타이 둘과 그 사이의 선. 가정한 모양이고, 관절을 넘는 고리는 없다."""
    return [
        ArmPart(f"D405 선/{LINK_NAME[name]} {p.name}", LINK_FRAME[name], p.shape, p.color)
        for name in links.LINKS
        for p in links.d405_parts(name)
    ]


def _wrist1() -> list[ArmPart]:
    """손목 1 몸통과 그 속의 J5."""
    body = ArmPart("손목 1/손목 몸통", "j5", wrist.wrist_body())
    inner = j.inner_parts("STS3215", wrist.X_FOOT, end_skirt=False) + j.inner_hardware("STS3215")
    return [body, *_from_joint("손목 1", 5, inner)]


def _wrist2() -> list[ArmPart]:
    """손목 2 몸통과 그 속의 J6, 플랜지 원판 (도구와 D405는 아직 넣지 않았다)."""
    body = ArmPart("손목 2/손목 몸통", "j6", wrist.wrist_body())
    inner = [p for p in wrist.wrist2_inner("STS3215") if p.name != "출력 원판"]
    inner.append(j.Part("플랜지 원판", flange.flange_disc(), moving=True, color="cap"))
    return [body, *_from_joint("손목 2", 6, inner + j.inner_hardware("STS3215"))]


def _wrist_end() -> list[ArmPart]:
    """손목 끝: 손목 2 몸통에 단 D405와 받침(J6와 같이 돌지 않는다), 플랜지에 단 평행 그리퍼 (손가락을 닫은 모습)."""
    cam = [
        j.Part("D405 받침", camera_mount.mount(), color="cap"),
        j.Part("D405", camera_mount.camera(), color="servo"),
        j.Part("D405 플러그", camera_mount.plug(), color="plug"),
        *camera_mount.hardware(),
    ]
    return _from_joint("D405", 6, cam) + _from_joint("그리퍼", 6, gripper.gripper_parts(0.0))


def _rest_shelf() -> list[ArmPart]:
    """쉬는 자세의 선반 (0037): J1 몸통에 달려 J1과 같이 돈다."""
    return _from_joint("선반", 1, rest_shelf.parts())


def _wire_clamps() -> list[ArmPart]:
    """선 누르개 셋 (0031): 띠 밑(베이스), 귀와 왼쪽 면(J1 몸통)."""
    return _from_joint("누르개", 1, wire_clamps.base_parts()) + _from_joint(
        "누르개", 2, wire_clamps.shoulder_parts()
    )


def _pi() -> list[ArmPart]:
    """날개 위의 파이 판 둘, 파이 두 대와 덮개, 꽂힌 플러그, 서보 어댑터 (베이스에 서 있다. 자리만 그린 것이 섞여 있다)."""
    return _from_joint("파이", 1, pi_mount.parts())


# 묶음 이름 → 그 묶음의 부품을 만드는 함수. 새 부품(도구, 선)은 여기에 묶음을 더한다
GROUPS: dict[str, Callable[[], list[ArmPart]]] = {
    "base": _base,
    "shoulder": _shoulder,
    "upper_arm": _upper_arm,
    "forearm": _forearm,
    "wrist1": _wrist1,
    "wrist2": _wrist2,
    "wrist_end": _wrist_end,
    "rest_shelf": _rest_shelf,
    "wire_clamps": _wire_clamps,
    "pi": _pi,
    "letters": _letters,
    "d405_run": _d405_run,
}


@functools.cache
def desk_parts() -> tuple[ArmPart, ...]:
    """책상과 금속 클램프 둘 (팔의 부품이 아니다. 닿음을 볼 때만 더한다). 클램프의 일부는 사진으로 어림한 치수다."""
    return tuple(_from_joint("책상", 1, base.desk_parts()))


@functools.cache
def group(name: str) -> tuple[ArmPart, ...]:
    """묶음 하나의 부품 (형상은 처음 부를 때 한 번만 만든다)."""
    return tuple(GROUPS[name]())


def arm_parts(*names: str) -> list[ArmPart]:
    """묶음들의 부품. 이름을 주지 않으면 모든 묶음이다."""
    return [p for name in (names or tuple(GROUPS)) for p in group(name)]


def place(parts: Sequence[ArmPart], q: ArrayLike) -> list[tuple[ArmPart, Compound]]:
    """부품들을 관절 각 q [rad]의 자세로 베이스 좌표(mm)에 놓는다."""
    spots = {name: location_of(T) for name, T in frames_mm(q).items()}
    return [(p, solid(p.shape.moved(spots[p.frame]))) for p in parts]


def gaps(
    placed: Sequence[tuple[ArmPart, Compound]],
    limit: float = 2.0,
    link_apart: int = 1,
    hardware: bool = True,
) -> dict[tuple[str, str], float]:
    """놓은 부품 가운데 서로 다른 링크에 붙은 것끼리의 틈 [mm] (limit 아래만. 겹치면 0이다).

    link_apart는 견줄 두 링크의 번호 차이의 하한이다: 2면 이웃한 링크(관절 하나로 이어져 틈이 좁게 그려진
    부품들)를 빼고 본다. hardware가 거짓이면 나사와 너트를 뺀다.
    """
    items = [(p, s) for p, s in placed if hardware or not p.hardware]
    out: dict[tuple[str, str], float] = {}
    for i, (a, sa) in enumerate(items):
        for b, sb in items[i + 1 :]:
            if abs(a.link - b.link) < link_apart or far_apart(sa, sb, limit):
                continue
            d = sa.distance_to(sb)
            if d < limit:
                out[(a.name, b.name)] = d
    return out


def overlaps(
    placed: Sequence[tuple[ArmPart, Compound]], tol: float = 0.05
) -> dict[tuple[str, str], float]:
    """놓은 부품 가운데 서로 다른 링크에 붙은 것끼리 겹친 부피 [mm³] (tol 넘는 것만). 나사와 너트는 보지 않는다."""
    items = [(p, s) for p, s in placed if not p.hardware]
    out: dict[tuple[str, str], float] = {}
    for i, (a, sa) in enumerate(items):
        for b, sb in items[i + 1 :]:
            if a.link == b.link or far_apart(sa, sb, 0.0) or sa.distance_to(sb) > 1e-6:
                continue
            v = solid(sa & sb).volume
            if v > tol:
                out[(a.name, b.name)] = v
    return out


def main() -> None:
    q = np.radians([float(v) for v in sys.argv[1:]] or RANGE_MID_DEG)
    for name, T in frames_mm(q).items():
        x, y, z = T[:3, 3]
        ax = T[:3, 2] if name == "flange" else T[:3, 1]
        print(
            f"{name:7s} 원점 ({x:7.1f}, {y:7.1f}, {z:7.1f}) mm  "
            f"{'도구 쪽' if name == 'flange' else '출력 쪽'} ({ax[0]:5.2f}, {ax[1]:5.2f}, {ax[2]:5.2f})"
        )
    t0 = time.perf_counter()
    parts = arm_parts()
    t1 = time.perf_counter()
    placed = place(parts, q)
    t2 = time.perf_counter()
    print(
        f"부품 {len(parts)}개: 형상 만들기 {t1 - t0:.1f} s (처음 한 번), 자세 하나 놓기 {t2 - t1:.3f} s"
    )
    lo = min(s.bounding_box().min.Z for _, s in placed)
    hi = max(s.bounding_box().max.Z for _, s in placed)
    print(f"놓은 팔의 높이 {lo:.1f} ~ {hi:.1f} mm (베이스 좌표)")


if __name__ == "__main__":
    main()
