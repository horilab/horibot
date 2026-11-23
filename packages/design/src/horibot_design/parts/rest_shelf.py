"""쉬는 자세의 선반 (0037의 A): 아래팔 끝(J4의 집)을 받치는 흰색 받침. J1 몸통의 목 앞벽에 M3 × 12 둘로 단다.

쉬는 자세는 토크 없이 팔이 받쳐지는 접은 자세다. 어깨(J2)와 팔꿈치(J3)가 풀려도 어깨 축, 팔꿈치 축, 선반에 앉은
J4의 집이 삼각형으로 굳는다. 선반은 J1과 같이 돌아서 J1이 어느 각에 있어도 팔이 그대로 얹혀 있다.

- 쉬는 자세의 관절 각(REST_DEG)은 운동학의 각이다. J2와 J3가 선반의 자리를 정한다 (J4 축이 로봇 앞 94.5 mm,
  책상에서 133.7 mm). J4는 손목이 늘어져 서는 각이고(studies/rest_pose.py가 팔 전체 모델의 무게로 푼다), J1은
  늘어진 손목이 로봇 앞 가운데로 오게 돌린 각이다 (집는 자리와 파이 덮개를 비킨다).
- 받침은 J4의 집(지름 47.2 mm 원통)을 밑과 뒤(몸통 쪽)에서 받는다. 팔의 무게가 선반을 누르는 힘은 수직에서
  몸통 쪽으로 12°쯤 기울어 있어서(어림), 밑 받침과 뒤 받침이 다 눌린다. 앞에는 낮은 턱만 있어 집이 위와 앞에서
  내려앉는다. 받침 셋은 집 옆의 벽 나사 머리(밑에서 몸통 쪽으로 26°, 앞으로 64°, 뒤 위로 116°)를 비킨 자리다.
- 뿌리 판은 J1 몸통의 앞 귀 윗면에 앉고 목 앞벽에 붙는다 (위치는 두 면이 잡는다). 팔 무게의 모멘트는 위 나사가
  당기고 아래쪽이 벽을 누른다. 너트는 J1 몸통의 목 속 살에 있다 (parts/base.py의 SHELF_*).
- 옆으로 눕혀 뽑는다 (옆모습이 바닥에 그려지고, 층이 받침에서 뿌리까지 이어진다).

좌표(mm)는 parts/base.py와 같다 (J1의 서보 좌표: x 앞, y 위, z 오른쪽). J1 각 0의 자리로 그리고, 돌림판과 같이 돈다.

    uv run python -m horibot_design.parts.rest_shelf
"""

from __future__ import annotations

import math

from build123d import Compound, Polygon, Pos, extrude

from horibot_design.parts import base as b
from horibot_design.parts import joint as j
from horibot_design.parts import links
from horibot_design.parts import shoulder as sh
from horibot_design.parts.shapes import cyl_x, cyl_z, solid
from horibot_design.printcheck import air_wall_lines, mid_layer_levels, overhangs, printed_mass_g

Flat = tuple[float, float]

# ---- 쉬는 자세 (운동학의 관절 각 [°]) ----
# J2 + J3 = 206°: 아래팔이 수직에서 26° 뒤로 누워 내려온다. J3는 한계(160°)에서 6° 안쪽이라 멈춤에 기대지 않는다.
REST_J2, REST_J3 = 52.0, 154.0
# 늘어진 손목이 서는 J4 (평행 그리퍼를 달았을 때 325.2°, 도구가 없으면 319.2°: studies/rest_pose.py). 한계는 340°다
REST_J4 = 325.0
# J5는 0°: 이 자세에서 J5 축이 거의 수직이라 토크 없이도 돌지 않고, 손목이 늘어지는 각이 범위 안에 든다
REST_J5, REST_J6 = 0.0, 0.0
# J1: 늘어진 손목과 그리퍼가 로봇 앞 가운데로 온다 (studies/rest_pose.py가 닿음을 본다)
REST_J1 = -55.0
REST_DEG = (REST_J1, REST_J2, REST_J3, REST_J4, REST_J5, REST_J6)

# ---- 받침이 받는 집 (아래팔 끝의 J4 집) ----
HOUSE_R = j.R_DRUM  # 집의 바깥 반지름 23.6
# 집의 앞 끝(출력 쪽)의 옆 자리: 로봇 좌표 y (왼쪽이 +). 위팔의 납작한 면(41.5)에서 두 링크 사이 층(5.8)만큼 안쪽이다
HOUSE_FRONT = sh.ATOP - sh.J1_AXIS_Y - (j.ATOP - j.FRONT)
# 뒷덮개의 테가 시작하는 자리 (그 뒤는 반지름이 2 mm 크다)
HOUSE_SKIRT = HOUSE_FRONT - (j.FRONT - (j.BACK_END + j.COVER_SKIRT))
SIDE = (HOUSE_SKIRT + 2.6, HOUSE_FRONT - 7.7)  # 선반의 옆 범위 (로봇 좌표 y): -10.0 ~ 28.0

# ---- 옆모습 (x, 책상에서의 높이) ----
PAD_GAP = 0.3  # 받침과 집 사이 (쉬는 자세의 각으로 놓았을 때)
RELIEF = 3.0  # 받침 사이에서 물러난 깊이 (벽 나사 머리 1.65가 든다)
SHELL = 5.0  # 받침 몸통의 두께
# 받침 셋: 밑에서 잰 각 [°] (몸통 쪽이 +). 앞 턱, 밑 받침, 뒤 받침
PADS = ((-35.0, -24.0), (-14.0, 8.0), (55.0, 80.0))
ARC = (-38.0, 85.0)  # 받침 몸통이 도는 범위
ROOT_T = 5.0  # 뿌리 판의 두께
ROOT_TOP = b.SHELF_NUT_UP[1] + 7.0  # 뿌리 판의 위 끝 (위 나사 위로 7)
BEAM_T = 7.3  # 밑 보의 두께 (앞 귀 윗면에서)
TIE = (
    131.0,
    6.1,
)  # 위 띠의 윗면 높이(뿌리 쪽)와 두께. 위 나사는 그 위에 있어 앞에서 렌치가 곧게 닿는다
WINDOW_X = 58.0  # 뿌리 판과 받침 사이를 비운 창의 앞 끝
WALL_X = sh.NECK_HW  # 목 앞벽의 바깥 면 (x)
EAR_TOP = b.TOP_H + j.EAR_T  # 앞 귀의 윗면 (책상에서 101.2)

LABELS = {"rest_shelf": "쉬는 자세 선반"}


def j4_axis(q2_deg: float = REST_J2, q3_deg: float = REST_J3) -> Flat:
    """관절 각에서 J4 축의 자리: (로봇 앞 x, 책상에서의 높이). J1 각 0의 팔 평면에서 본다."""
    upper, fore = links.LINKS["upper_arm"][0], links.LINKS["forearm"][0]
    a, s = math.radians(q2_deg), math.radians(q2_deg + q3_deg)
    return (
        upper * math.sin(a) + fore * math.sin(s),
        b.SHOULDER_H + upper * math.cos(a) + fore * math.cos(s),
    )


def screw_angles(q2_deg: float = REST_J2, q3_deg: float = REST_J3) -> tuple[float, float, float]:
    """쉬는 자세에서 집 옆의 벽 나사 머리가 오는 각 [°] (밑에서, 몸통 쪽이 +): 서보의 가까운 끝 쪽 하나와 양옆.

    집의 가까운 끝 쪽(서보 좌표 +x)은 아래팔이 온 방향의 반대다: 아래팔이 수직에서 (q2 + q3 - 180)° 누웠으면
    그만큼 몸통 쪽으로 돈 자리다.
    """
    lean = q2_deg + q3_deg - 180.0
    return lean, lean - 90.0, lean + 90.0


def _at(angle_deg: float, radius: float) -> Flat:
    """받침 가운데(J4 축)에서 밑으로 잰 각 angle(몸통 쪽이 +), 반지름 radius인 점."""
    cx, cz = j4_axis()
    a = math.radians(angle_deg)
    return cx - radius * math.sin(a), cz - radius * math.cos(a)


def _prism(pts: list[Flat]) -> Compound:
    """옆모습의 다각형 (x, 책상에서의 높이)을 선반의 옆 범위만큼 민 기둥."""
    z0, z1 = -SIDE[1], -SIDE[0]
    flat = [(x, up + b.Y_DESK) for x, up in pts]
    area = sum(p[0] * q[1] - q[0] * p[1] for p, q in zip(flat, flat[1:] + flat[:1], strict=True))
    if area < 0:  # 시계 방향이면 면이 뒤집혀 반대쪽으로 밀린다
        flat.reverse()
    return solid(Pos(0, 0, z0) * extrude(Polygon(*flat, align=None), amount=z1 - z0))


def _sector(a0: float, a1: float, reach: float = 70.0) -> Compound:
    """받침 가운데에서 각 a0 ~ a1로 펼친 부채꼴 기둥 (반지름 reach·cos 15° 넘게 덮는다)."""
    n = max(1, math.ceil((a1 - a0) / 30.0))
    rim = [_at(a0 + (a1 - a0) * k / n, reach) for k in range(n + 1)]
    return _prism([j4_axis(), *rim])


def _ring(r_out: float, r_in: float) -> Compound:
    cx, cz = j4_axis()
    z0, z1 = -SIDE[1], -SIDE[0]
    cy = cz + b.Y_DESK
    return solid(cyl_z(r_out, z0, z1, cx, cy) - cyl_z(r_in, z0 - 1, z1 + 1, cx, cy))


def screw_xyz() -> list[tuple[float, float, float]]:
    """뿌리 판의 나사 둘 (J1의 서보 좌표): 머리가 앉는 면 위의 점."""
    return [(WALL_X + ROOT_T, up + b.Y_DESK, -b.SHELF_NUT_Y) for up in b.SHELF_NUT_UP]


def shelf() -> Compound:
    """쉬는 자세의 선반. 옆면(로봇 오른쪽 면, +z)을 바닥에 두고 눕혀 뽑는다."""
    r_pad = HOUSE_R + PAD_GAP
    r_rel = HOUSE_R + RELIEF
    r_out = r_rel + SHELL
    tie_top, tie_t = TIE
    back = _at(ARC[1], r_out)
    outline = [
        (WALL_X, EAR_TOP),
        (40.0, EAR_TOP),
        _at(0.0, r_out),
        j4_axis(),
        back,
        (WALL_X + ROOT_T, tie_top),
        (WALL_X + ROOT_T, ROOT_TOP),
        (WALL_X, ROOT_TOP),
    ]
    s = solid(_prism(outline) + solid(_ring(r_out, r_rel) & _sector(*ARC)))
    cx, cz = j4_axis()
    z0, z1 = -SIDE[1], -SIDE[0]
    s = solid(s - cyl_z(r_rel, z0 - 1, z1 + 1, cx, cz + b.Y_DESK))  # 집이 앉는 자리
    for a0, a1 in PADS:
        s = solid(s + solid(_ring(r_rel + 0.01, r_pad) & _sector(a0, a1)))
    # 뿌리 판과 받침 사이의 창: 무게를 덜고, 아래 나사의 머리와 렌치가 든다. 위 띠의 밑변과 나란하다
    x0 = WALL_X + ROOT_T
    slope = (back[1] - tie_top) / (back[0] - x0)
    under = tie_top - tie_t
    window = [
        (x0, EAR_TOP + BEAM_T),
        (WINDOW_X, EAR_TOP + BEAM_T),
        (WINDOW_X, under + slope * (WINDOW_X - x0)),
        (x0, under),
    ]
    s = solid(s - _prism(window))
    for x, y, z in screw_xyz():
        s = solid(s - cyl_x(j.M3_FREE_H / 2, WALL_X - 1, x + 1, y, z))
    if not s.is_valid or len(s.solids()) != 1:
        raise RuntimeError("선반 형상이 깨졌다")
    return s


def house_dummy() -> Compound:
    """쉬는 자세에 앉은 J4의 집 (원통만. 받침과의 틈을 보는 데 쓴다)."""
    cx, cz = j4_axis()
    return cyl_z(HOUSE_R, -HOUSE_FRONT, -HOUSE_SKIRT, cx, cz + b.Y_DESK)


def hardware() -> list[j.Part]:
    """선반 나사 M3 × 12 둘과 목 속의 너트 둘 (J1의 서보 좌표, 돌림판과 같이 돈다)."""
    out: list[j.Part] = []
    for k, (x, y, z) in enumerate(screw_xyz(), start=1):
        bolt = cyl_x(1.5, x - b.SHELF_SCREW, x, y, z) + cyl_x(2.85, x, x + 1.65, y, z)
        out.append(j.Part(f"선반 나사 M3x12 {k}", solid(bolt), j.STEEL, True, "metal"))
        x1 = WALL_X - b.SHELF_WALL  # 너트가 기대는 면
        nut = j.hex_x(y, z, x1 - j.NUT_T, x1, 5.4) - cyl_x(1.5, x1 - j.NUT_T - 1, x1 + 1, y, z)
        out.append(j.Part(f"너트 (선반) {k}", solid(nut), j.STEEL, True, "metal"))
    return out


def parts() -> list[j.Part]:
    """선반과 그 나사 (J1 몸통과 같이 도는 부품)."""
    return [j.Part("쉬는 자세 선반", shelf(), moving=True, color="body"), *hardware()]


def screw_checks() -> dict[str, float]:
    """나사가 물린 길이와 여유 [mm]."""
    x_head = WALL_X + ROOT_T
    tip = x_head - b.SHELF_SCREW
    nut_face = WALL_X - b.SHELF_WALL
    lower = b.SHELF_NUT_UP[0]
    return {
        "선반 나사가 너트(2.4)에 물린 길이": min(nut_face - tip, j.NUT_T),
        "선반 나사 끝이 너트를 지난 길이": (nut_face - j.NUT_T) - tip,
        "선반 나사 끝과 어깨 서보 옆면 사이": tip - sh.HW,
        "아래 나사 머리 밑과 밑 보 윗면 사이": lower - 2.85 - (EAR_TOP + BEAM_T),
        "위 나사 머리와 위 띠 윗면 사이": b.SHELF_NUT_UP[1] - 2.85 - TIE[0],
        "위 나사 위에 남은 뿌리 판": ROOT_TOP - b.SHELF_NUT_UP[1] - 2.85,
        "창에서 아래 나사에 렌치의 짧은 쪽이 들어갈 길이": WINDOW_X - (x_head + 1.65),
    }


def pad_margins() -> dict[str, float]:
    """받침과 벽 나사 머리 사이의 각 여유 [°] (머리의 반각 7°를 뺀 값). 받침마다 가장 가까운 머리와."""
    half = math.degrees(math.asin(2.85 / HOUSE_R))
    out: dict[str, float] = {}
    for name, (a0, a1) in zip(("앞 턱", "밑 받침", "뒤 받침"), PADS, strict=True):
        gaps = [max(a0 - s, s - a1) - half for s in screw_angles()]
        out[name] = min(gaps)
    return out


def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    return {"rest_shelf": (shelf(), (0.0, 0.0, -1.0), "white", "strength")}


def report() -> None:
    cx, cz = j4_axis()
    print(
        f"쉬는 자세 (운동학 J1 ~ J6 [°]): {REST_DEG}. J4 축은 앞으로 {cx:.1f}, 책상에서 {cz:.1f} mm, "
        f"집의 밑 {cz - HOUSE_R:.1f} mm"
    )
    print(
        f"선반의 옆 범위 (로봇 y) {SIDE[0]:.1f} ~ {SIDE[1]:.1f}: 집의 앞 끝 {HOUSE_FRONT:.1f}, "
        f"뒷덮개 테 {HOUSE_SKIRT:.1f}"
    )
    made = printed()
    for name, (shape, up, color, preset) in made.items():
        bb = shape.bounding_box()
        over = overhangs(shape, up)
        print(
            f"  {name} {color} {bb.size.X:.1f} × {bb.size.Y:.1f} × {bb.size.Z:.1f} mm  "
            f"{printed_mass_g(shape, up, preset):.1f} g  걸침 {over.total:.0f} mm² "
            f"(가장 넓은 한 자리 {over.largest:.0f})"
        )
        print(f"  층 가운데에 걸친 수평면: {mid_layer_levels(shape, up) or '없음'}")
    print("\n".join(air_wall_lines(made)))
    for name, value in screw_checks().items():
        print(f"  {name}: {value:.2f}")
    print(
        "벽 나사 머리가 오는 각 [°]: "
        + ", ".join(f"{a:.0f}" for a in screw_angles())
        + " / 받침과의 여유 [°]: "
        + ", ".join(f"{k} {v:.1f}" for k, v in pad_margins().items())
    )
    s = made["rest_shelf"][0]
    house = house_dummy()
    body = b.as_child(b.j1_body())
    print(
        f"틈 [mm]: 받침과 쉬는 자세의 집 {s.distance_to(house):.2f}, 선반과 J1 몸통 {s.distance_to(body):.2f} "
        f"(닿아 앉는다), 겹침 {solid(s & body).volume if s.distance_to(body) < 1e-6 else 0.0:.2f} mm³, "
        f"선반과 돌림판 {s.distance_to(b.turntable()):.2f}"
    )


if __name__ == "__main__":
    report()
