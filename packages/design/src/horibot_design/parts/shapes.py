"""부품 코드가 같이 쓰는 형상 도우미 (단위 mm).

상자와 원기둥을 좌표 범위로 만들고, 볼트 구멍, 너트 홈, 물방울 구멍을 낸다. 너트 홈 치수는 공차 시험편으로
고른 값(data/measured.toml)을 쓴다 (docs/decisions/0003-fastening-nut-traps.md).
"""

from __future__ import annotations

import math
from typing import cast

from build123d import (
    Align,
    Box,
    Circle,
    Compound,
    Cone,
    Cylinder,
    Location,
    Plane,
    Polygon,
    Pos,
    RegularPolygon,
    Rot,
    Sketch,
    Vector,
    extrude,
)

from horibot_design.printcheck import LAYER

Point = tuple[float, float, float]

# ISO 4032 M3 육각너트 두께. 홈 높이는 1차 시험편에서 고른 값을 쓴다.
M3_NUT_THICKNESS = 2.4


def solid(shape: object) -> Compound:
    """불 연산 결과를 Compound로 받는다. build123d의 연산자 타입은 선이나 면이 섞인 합집합이라 여기서 좁힌다."""
    if not isinstance(shape, Compound):
        raise TypeError(f"입체가 아니다: {type(shape).__name__}")
    return shape


def box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Compound:
    """좌표 범위로 만든 상자."""
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _up(radius: float, length: float) -> Compound:
    return Cylinder(radius, length, align=(Align.CENTER, Align.CENTER, Align.MIN))


def cyl_x(radius: float, x0: float, x1: float, y: float = 0.0, z: float = 0.0) -> Compound:
    """x 방향 원기둥 (x0 → x1)."""
    return Pos(x0, y, z) * (Rot(0, 90, 0) * _up(radius, x1 - x0))


def cyl_y(radius: float, y0: float, y1: float, x: float = 0.0, z: float = 0.0) -> Compound:
    """y 방향 원기둥 (y0 → y1)."""
    return Pos(x, y0, z) * (Rot(-90, 0, 0) * _up(radius, y1 - y0))


def cyl_z(radius: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0) -> Compound:
    """z 방향 원기둥 (z0 → z1)."""
    return Pos(x, y, z0) * _up(radius, z1 - z0)


def cone_x(r0: float, r1: float, x0: float, x1: float) -> Compound:
    """x 방향 원뿔대 (x0에서 반지름 r0, x1에서 r1)."""
    return Pos(x0, 0, 0) * (
        Rot(0, 90, 0) * Cone(r0, r1, x1 - x0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    )


def _pt(v: Vector) -> Point:
    return (v.X, v.Y, v.Z)


def rod(a: Point, b: Point, radius: float) -> Compound:
    """a에서 b까지 원기둥."""
    va, vb = Vector(*a), Vector(*b)
    return _up(radius, (vb - va).length).moved(Location(Plane(origin=va, z_dir=vb - va)))


def slot(a: Point, b: Point, radius: float, half_width: float, side: Point) -> Compound:
    """a에서 b까지 가는 납작한 길: 반지름 radius의 원기둥 둘을 side 방향으로 ±half_width 띄우고 그 사이를 채운다.

    5264 플러그처럼 납작한 것이 지나가는 길이다. side는 a→b와 직각인 단위 벡터다.
    """
    va, vb, vs = Vector(*a), Vector(*b), Vector(*side)
    d = vb - va
    ends = [rod(_pt(va + vs * s), _pt(vb + vs * s), radius) for s in (-half_width, half_width)]
    web = Box(2 * radius, 2 * half_width, d.length, align=(Align.CENTER, Align.CENTER, Align.MIN))
    plane = Plane(origin=va, x_dir=d.cross(vs), z_dir=d)
    return solid(ends[0] + ends[1] + web.moved(Location(plane)))


def nut_trap(
    center: Point, axis: Point, slide: Point, length: float, width: float, height: float
) -> Compound:
    """옆에서 밀어 넣는 너트 홈. 볼트는 axis 방향, 너트는 slide 쪽 입구에서 length만큼 밀려 들어온다.

    홈 끝은 육각이라 너트가 볼트 구멍 가운데에서 멈추고 돌지 않는다 (1차 시험편과 같은 모양).
    width는 맞변 거리(홈 폭), height는 볼트 방향 홈 높이다.
    """
    hexagon = extrude(
        RegularPolygon(width / 2, 6, major_radius=False), amount=height / 2, both=True
    )
    channel = Pos(length / 2, 0, 0) * Box(length, width, height)
    return solid((hexagon + channel).moved(Location(Plane(origin=center, x_dir=slide, z_dir=axis))))


def teardrop(origin: Point, axis: Point, length: float, diameter: float, roof: Point) -> Compound:
    """origin에서 axis 방향으로 length만큼 가는 가로 구멍. roof 방향(출력할 때 위, axis와 직각인 단위 벡터)으로
    45° 지붕을 내서 서포트 없이 뽑힌다."""
    r = diameter / 2
    c = r * math.cos(math.pi / 4)
    # 지붕 삼각형은 반시계 방향으로 준다. 시계 방향이면 면이 뒤집혀 원과 반대쪽으로 밀린다
    profile = Circle(r) + Polygon((-c, c), (c, c), (0.0, r * math.sqrt(2)), align=None)
    body = extrude(cast(Sketch, profile), amount=length)  # 지붕은 지역 +Y, 민 방향은 지역 +Z
    va, vr = Vector(*axis), Vector(*roof)
    # 지역 z = axis, 지역 y = roof가 되도록 지역 x = roof × axis (y = z × x)
    plane = Plane(origin=origin, x_dir=vr.cross(va), z_dir=va)
    return solid(body.moved(Location(plane)))


def teardrop_y(diameter: float, y0: float, y1: float, x: float, z: float, roof: Point) -> Compound:
    """y 방향 가로 구멍. roof 방향(출력할 때 위, xz 평면의 단위 벡터)으로 45° 지붕을 낸다.

    서포트 없이 뽑히고, 지붕 쪽으로만 틈이 생긴다. 영점 핀처럼 한 방향만 정확하면 되는 구멍은 지붕이
    그 방향과 직각이 되게 둔다.
    """
    r = diameter / 2
    c = r * math.cos(math.pi / 4)
    profile = Circle(r) + Polygon((-c, c), (c, c), (0.0, r * math.sqrt(2)), align=None)
    body = extrude(cast(Sketch, profile), amount=y1 - y0)  # 지붕은 +Y, 민 방향은 +Z
    rx, _, rz = roof
    x_dir = (-rz, 0.0, rx)  # roof × (0, 1, 0): 이 틀에서 지역 y가 roof, 지역 z가 세계 y다
    return solid(body.moved(Location(Plane(origin=(x, y0, z), x_dir=x_dir, z_dir=(0.0, 1.0, 0.0)))))


# ---- 바닥 쪽 머리 자리의 천장 ------------------------------------------------------------------------------
#
# 바닥에 닿는 면에서 판 나사 머리 자리(너트 자리)는 그 천장이 허공에 뽑힌다. 천장에 나사 구멍이 뚫려 있으면
# 슬라이서가 구멍을 둘러싼 벽 고리를 공중에 뽑고, 폭이 좁은 자리에서는 윤곽을 따라 도는 벽 선이 공중에서 꺾여
# 돈다. 둘 다 처진다 (docs/decisions/0035-slicer-automation.md의 공중 벽 선 검사). 천장 첫 층의 선이 모두 곧고
# 양 끝이 붙게 하는 따냄 둘이다. 스튜디오 2.9.1로 잘라 G-code에서 확인했다 (Strength 벽 6겹, Standard 벽 2겹):
# - slot_ceiling: 구멍 폭의 긴 구멍 한 층 + 네모 구멍 한 층. 뚫을 것이 없다. 머리는 긴 구멍 양옆의 두 띠에만 앉는다
#   (M3 둥근머리 5.7, 구멍 3.4면 고리 16.4 mm² 중 7.4 mm²). 힘을 받지 않는 덮개 나사에 쓴다.
# - comb_ceiling: 천장 첫 층이 구멍까지 덮는 막이고, 그 막을 좁은 틈으로 갈라 빗살로 만든다. 머리가 고리 전체에
#   앉는다 (틈만큼만 빠진다). 조립할 때 구멍 자리의 막을 뚫는다. 힘을 받는 머리 자리에 쓰고, 벽 띠(Strength
#   2.45 mm)보다 윤곽에 가까운 좁은 자리에서도 선이 곧다. 윤곽에서 벽 띠보다 먼 넓은 자리는 틈 없이 막만 두어도
#   다리 선으로 덮인다 (Standard 덮개에서 확인).
# 빗살의 틈: 폭 0.25 mm면 슬라이서가 틈으로 보고(닫는 반지름 0.049 mm) 틈 양옆에 곧은 벽 선을 낸다. 간격 1.7 mm면
# 빗살 폭이 1.45 mm라 선 서넛이 든다. 빗살 폭이 2.2 mm면 빗살 끝에서 벽 선이 다시 꺾여 돈다 (출력 원판 귀에 틈
# 둘을 ±1.0에 두고 잘라 본 것)
COMB_SLIT = 0.25
COMB_PITCH = 1.7


def slot_ceiling(
    seat: Point, up: Point, along: Point, hole_d: float, pocket_d: float, layer: float = LAYER
) -> Compound:
    """구멍 뚫린 머리 자리의 천장 위 두 층에서 따낼 것: 첫 층은 구멍 폭의 긴 구멍(along 방향, 머리 자리 지름까지),
    둘째 층은 구멍 폭의 네모다. 벽 선이 첫 층에서는 긴 구멍을 따라, 둘째 층에서는 그 위를 가로질러 곧게 건넌다.

    seat는 천장(머리가 앉는 면)의 구멍 가운데, up은 뽑을 때 위, along은 긴 구멍의 방향이다 (up과 직각인 단위 벡터).
    둥근 구멍은 부르는 쪽이 seat에서 두 층 위부터 낸다. seat는 층 경계에 둔다.
    """
    over = 0.01  # 아래 것과 겹쳐 한 덩어리로 따낸다
    first = Pos(0, 0, (layer - over) / 2) * Box(pocket_d, hole_d, layer + over)
    first &= Pos(0, 0, -over) * _up(pocket_d / 2, layer + over)
    second = Pos(0, 0, layer + (layer - over) / 2) * Box(hole_d, hole_d, layer + over)
    return solid((first + second).moved(Location(Plane(origin=seat, x_dir=along, z_dir=up))))


def comb_ceiling(
    seat: Point,
    up: Point,
    along: Point,
    pocket_d: float,
    back: float,
    front: float,
    layer: float = LAYER,
) -> Compound:
    """머리 자리의 천장 첫 층(구멍까지 덮는 막)을 빗살로 가르는 틈들: along 방향으로 나란하고 가운데 틈이 구멍
    가운데를 지난다. 벽 선이 틈을 따라 곧게 놓여, 양 끝(머리 자리 밖의 살)에 붙은 채로 머리 자리를 건넌다.

    seat는 천장의 구멍 가운데, up은 뽑을 때 위다. 틈은 seat에서 along 반대쪽으로 back, along 쪽으로 front까지다.
    둘 다 머리 자리 벽을 0.5 mm 넘게 지나야 선이 살 위에서 돈다. 구멍은 부르는 쪽이 seat에서 한 층 위부터 낸다.
    """
    over = 0.01
    n = int((pocket_d / 2 - 0.6) / COMB_PITCH)
    slits: Compound | None = None
    for k in range(-n, n + 1):
        one = Pos((front - back) / 2, k * COMB_PITCH, (layer - over) / 2) * Box(
            front + back, COMB_SLIT, layer + over
        )
        slits = solid(one) if slits is None else solid(slits + one)
    assert slits is not None
    return slits.moved(Location(Plane(origin=seat, x_dir=along, z_dir=up)))


# 바깥 상자는 빠른 어림이라 곡면에서 실제보다 조금 작을 수 있다. 이만큼 더 떨어진 쌍만 건너뛴다
BOX_MARGIN = 1.0


def far_apart(a: Compound, b: Compound, limit: float) -> bool:
    """두 형상의 바깥 상자가 limit + BOX_MARGIN보다 멀리 떨어져 있다.

    두 형상 사이의 실제 틈은 바깥 상자 사이 거리보다 작지 않아서, 그런 쌍은 틈이 limit보다 크다. 틈 검사에서 먼 쌍의
    거리 계산(부품 하나에 0.1 ~ 0.5초)을 건너뛰는 데 쓴다. 결과는 거르지 않을 때와 같다.
    """
    ba, bb = a.bounding_box(optimal=False), b.bounding_box(optimal=False)
    dx = max(0.0, bb.min.X - ba.max.X, ba.min.X - bb.max.X)
    dy = max(0.0, bb.min.Y - ba.max.Y, ba.min.Y - bb.max.Y)
    dz = max(0.0, bb.min.Z - ba.max.Z, ba.min.Z - bb.max.Z)
    return math.sqrt(dx * dx + dy * dy + dz * dz) > limit + BOX_MARGIN
