"""선 누르개 셋 (0031의 J1과 J1 몸통): J1을 넘는 서보 선 둘을 M3 × 8 하나씩으로 누르는 작은 판.

- 띠 밑 누르개 (베이스 몸통, 서 있다): 몸통 뒤 왼쪽(방위각 208°)의 띠 밑에서 선 둘을 몸통의 둥근 면에 누른다.
  선은 나사 양옆에 하나씩 세로로 지나 아래 턱을 튼 자리로 띠에 들어간다.
- 귀 누르개 (J1 몸통의 뒤쪽 귀 윗면, J1과 같이 돈다): 띠에서 돌림판과 귀의 가장자리를 넘어온 선 둘을 귀 윗면에 누른다.
  목 뒷벽에 닿는 모서리는 잘라 냈다. 선은 여기서 목 뒷벽 밑을 따라 왼쪽으로 가 뒤 왼쪽 모서리를 돈다.
- 왼쪽 면 누르개 (J1 몸통의 왼쪽 면, 어깨 앞면): 나사는 바닥판의 아래쪽에 있고, 위로 뻗은 팔이 그 위를 앞뒤로 지나는
  선 둘을 누른다 (아래 선은 J3로 가는 선, 위 선은 선 창으로 들어가는 J2 선). 왼쪽 면과 위팔의 납작한 면 사이가
  5.8 mm라 나사 머리까지 3.8 mm 안에 든다 (0031).

누르개는 선을 0.2 ~ 0.3 mm만 누른다 (선 두께 1.5에 자리 깊이 1.2 ~ 1.3). 나사를 끝까지 조여도 누르개가 몸통에 닿아 더
눌리지 않는다. 셋 다 검정이다. 선이 놓이는 면이 위를 보게 뽑는다 (왼쪽 면 누르개는 옆으로 눕힌다).

    uv run python -m horibot_design.parts.wire_clamps
"""

from __future__ import annotations

import math

from build123d import Compound, Polygon, Pos, extrude

from horibot_design.parts import base as b
from horibot_design.parts import joint as j
from horibot_design.parts import shoulder as sh
from horibot_design.parts.shapes import box, cyl_x, cyl_y, solid
from horibot_design.printcheck import air_wall_lines, mid_layer_levels, overhangs, printed_mass_g

SCREW = 8.0  # M3 × 8 둥근머리
HEAD = (2.85, 1.65)  # 머리 반지름과 높이
LANE = (1.9, 7.4)  # 나사 가운데에서 선 자리까지 (선 폭 4.85에 0.65 여유)
GROOVE = j.WIRE_T - 0.3  # 선 자리의 깊이 1.2: 선을 0.3 누른다
PLATE_T = 3.0  # 띠 밑 누르개와 귀 누르개의 두께
HALF = (4.0, 7.6)  # 그 둘의 반높이(선 방향)와 반폭
EAR_HALF = 3.5  # 귀 누르개의 선 방향 반길이
WALL_GAP = 0.6  # 귀 누르개와 목 뒷벽 사이
# 왼쪽 면 누르개의 옆모습: (바닥판 밑에서 위로, 왼쪽 면에서 밖으로). 나사 자리(두께 1.6)에서 한 단 올라가 선 위를 덮는다
STRIP_LAND = 1.6
STRIP_STANDOFF = j.WIRE_T - 0.2  # 팔 밑의 높이 1.3: 선을 0.2 누른다
STRIP_ARM = 1.4
STRIP_REACH = 17.0  # 팔의 위 끝 (바닥판 밑에서. 바닥판 위 벽의 선까지 덮는다)
STRIP_HW = 4.0  # 반폭 (선 방향)
STRIP_LIMIT = 3.8  # 왼쪽 면에서 나올 수 있는 높이 (0031)

LABELS = {
    "wire_clamp_band": "띠 밑 누르개",
    "wire_clamp_ear": "귀 누르개",
    "wire_clamp_strip": "왼쪽 면 누르개",
}


def _lanes(sign_axis: str, lo: float, hi: float, a0: float, a1: float) -> list[Compound]:
    """나사 양옆의 선 자리 둘이 지나는 상자 (z 쪽으로 LANE)."""
    out = []
    for sg in (1, -1):
        z0, z1 = sorted((sg * LANE[0], sg * LANE[1]))
        if sign_axis == "x":
            out.append(box(lo, hi, a0, a1, z0, z1))
        else:
            out.append(box(a0, a1, lo, hi, z0, z1))
    return out


def band_clamp_flat() -> Compound:
    """띠 밑 누르개를 방위각 0°(몸통의 +x 쪽)에 놓은 것: 안쪽 면이 몸통의 둥근 면에 맞고, 선 자리 둘이 세로로 지난다."""
    y_c = b.FRONT - b.BAND_H - b.CLAMP_DROP
    r0 = b.R_CORE
    hy, hz = HALF
    body = box(r0 - 2.0, r0 + PLATE_T, y_c - hy, y_c + hy, -hz, hz)
    body = solid(body - cyl_y(r0, y_c - hy - 1, y_c + hy + 1))
    groove = j.ring(r0 + GROOVE, r0 - 1, y_c - hy - 1, y_c + hy + 1)
    for lane in _lanes("x", 0.0, r0 + 10, y_c - hy - 1, y_c + hy + 1):
        body = solid(body - solid(groove & lane))
    return solid(body - cyl_x(j.K_M3_FREE / 2, r0 - 3, r0 + PLATE_T + 1, y_c, 0.0))


def band_clamp() -> Compound:
    """띠 밑 누르개 (베이스 좌표, 제자리: 방위각 WIRE_AZ)."""
    return j.turned(band_clamp_flat(), b.WIRE_AZ)


def band_screw() -> Compound:
    y_c = b.FRONT - b.BAND_H - b.CLAMP_DROP
    r1 = b.R_CORE + PLATE_T
    s = cyl_x(1.5, r1 - SCREW, r1, y_c, 0.0) + cyl_x(HEAD[0], r1, r1 + HEAD[1], y_c, 0.0)
    return j.turned(solid(s), b.WIRE_AZ)


def ear_clamp() -> Compound:
    """귀 누르개 (베이스 좌표, J1 각 0의 자리). 선 자리 둘이 반지름 쪽으로 지난다."""
    cx, cz = b.EAR_CLAMP
    az = math.degrees(math.atan2(cz, cx))
    y0 = b.ATOP + j.EAR_T
    hz = HALF[1]
    body = box(-EAR_HALF, EAR_HALF, y0, y0 + PLATE_T, -hz, hz)
    for lane in _lanes("x", -EAR_HALF - 1, EAR_HALF + 1, y0 - 1, y0 + GROOVE):
        body = solid(body - lane)
    body = solid(body - cyl_y(j.K_M3_FREE / 2, y0 - 1, y0 + PLATE_T + 1))
    placed = solid(Pos(cx, 0, cz) * j.turned(body, az))
    wall = -(sh.NECK_HW + WALL_GAP)
    return solid(placed - box(wall, 0.0, y0 - 1, y0 + PLATE_T + 1, -60.0, 60.0))


def ear_screw() -> Compound:
    cx, cz = b.EAR_CLAMP
    y1 = b.ATOP + j.EAR_T + PLATE_T
    return solid(cyl_y(1.5, y1 - SCREW, y1, cx, cz) + cyl_y(HEAD[0], y1, y1 + HEAD[1], cx, cz))


def _strip_profile() -> list[tuple[float, float]]:
    a_s = b.STRIP_SCREW_UP - b.TOP_H  # 나사의 높이 (바닥판 밑에서 2.9)
    step = a_s + HEAD[0] + 0.25  # 머리 옆에서 한 단 오른다
    top = STRIP_STANDOFF + STRIP_ARM
    return [
        (0.2, 0.0),
        (step, 0.0),
        (step + 0.8, STRIP_STANDOFF),
        (STRIP_REACH, STRIP_STANDOFF),
        (STRIP_REACH, top),
        (step, top),
        (step, STRIP_LAND),
        (0.2, STRIP_LAND),
    ]


def strip_clamp() -> Compound:
    """왼쪽 면 누르개 (어깨의 서보 좌표, 제자리). 옆면을 바닥에 두고 눕혀 뽑는다."""
    zc = -b.STRIP_SCREW_X
    pts = [(b.NECK_X + a, sh.FRONT + out) for a, out in _strip_profile()]
    area = sum(p[0] * q[1] - q[0] * p[1] for p, q in zip(pts, pts[1:] + pts[:1], strict=True))
    if area < 0:
        pts.reverse()
    body = Pos(0, 0, zc - STRIP_HW) * extrude(Polygon(*pts, align=None), amount=2 * STRIP_HW)
    x_s = b.STRIP_SCREW_UP - b.SHOULDER_H
    return solid(body - cyl_y(j.K_M3_FREE_H / 2, sh.FRONT - 1, sh.FRONT + 5, x_s, zc))


def strip_screw() -> Compound:
    x_s, zc = b.STRIP_SCREW_UP - b.SHOULDER_H, -b.STRIP_SCREW_X
    y1 = sh.FRONT + STRIP_LAND
    return solid(cyl_y(1.5, y1 - SCREW, y1, x_s, zc) + cyl_y(HEAD[0], y1, y1 + HEAD[1], x_s, zc))


def strip_height() -> float:
    """왼쪽 면에서 누르개와 나사 머리가 나온 높이 [mm]."""
    return max(STRIP_LAND + HEAD[1], STRIP_STANDOFF + STRIP_ARM)


def base_parts() -> list[j.Part]:
    """베이스 좌표의 누르개 둘과 나사: 띠 밑 누르개는 서 있고, 귀 누르개는 J1과 같이 돈다."""
    return [
        j.Part("띠 밑 누르개", band_clamp(), color="cap"),
        j.Part("누르개 나사 M3x8 (띠 밑)", band_screw(), j.STEEL, False, "metal"),
        j.Part("귀 누르개", ear_clamp(), moving=True, color="cap"),
        j.Part("누르개 나사 M3x8 (귀)", ear_screw(), j.STEEL, True, "metal"),
    ]


def shoulder_parts() -> list[j.Part]:
    """어깨의 서보 좌표(J1 몸통)의 왼쪽 면 누르개와 나사."""
    return [
        j.Part("왼쪽 면 누르개", strip_clamp(), color="cap"),
        j.Part("누르개 나사 M3x8 (왼쪽 면)", strip_screw(), j.STEEL, False, "metal"),
    ]


def screw_checks() -> dict[str, float]:
    """나사가 구멍에 물린 길이와 여유 [mm]."""
    return {
        "띠 밑: 조여 넣는 구멍에 물린 길이 (구멍 깊이 8)": SCREW - PLATE_T,
        "귀: 귀(6.2)에 물린 길이": SCREW - PLATE_T,
        "귀: 나사 끝과 돌림판 윗면 사이": j.EAR_T - (SCREW - PLATE_T),
        "왼쪽 면: 조여 넣는 구멍에 물린 길이": SCREW - STRIP_LAND,
        "왼쪽 면: 나사 끝과 구멍 바닥 사이": b.STRIP_TAP - (SCREW - STRIP_LAND),
        "왼쪽 면: 누르개와 머리가 나온 높이 (한도 3.8)": strip_height(),
        "왼쪽 면: 위팔의 납작한 면까지 남는 틈": (sh.ATOP - sh.FRONT) - strip_height(),
    }


def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    return {
        # 띠 밑 누르개: 바깥 면이 바닥, 선 자리가 위 (방위각 0°에 놓은 형상이라 위가 -x다)
        "wire_clamp_band": (band_clamp_flat(), (-1.0, 0.0, 0.0), "black", "standard"),
        "wire_clamp_ear": (ear_clamp(), (0.0, -1.0, 0.0), "black", "standard"),
        "wire_clamp_strip": (strip_clamp(), (0.0, 0.0, 1.0), "black", "standard"),
    }


def report() -> None:
    made = printed()
    for name, (shape, up, color, preset) in made.items():
        bb = shape.bounding_box()
        over = overhangs(shape, up)
        print(
            f"  {name:17s} {color} {bb.size.X:.1f} × {bb.size.Y:.1f} × {bb.size.Z:.1f} mm  "
            f"{printed_mass_g(shape, up, preset):.2f} g  걸침 {over.total:.0f} mm²  "
            f"층 가운데 수평면 {mid_layer_levels(shape, up) or '없음'}"
        )
    print("\n".join(air_wall_lines(made)))
    for name, value in screw_checks().items():
        print(f"  {name}: {value:.2f}")
    core, body = b.core(), b.j1_body()
    table, moved = b.turntable(), b.as_child(b.j1_body())
    band, ear, strip = band_clamp(), ear_clamp(), strip_clamp()
    print(
        f"겹침 [mm³]: 띠 밑 누르개와 몸통 {solid(band & core).volume:.3f}, 귀 누르개와 J1 몸통 "
        f"{solid(ear & moved).volume:.3f}, 왼쪽 면 누르개와 J1 몸통 {solid(strip & body).volume:.3f}"
    )
    print(
        f"틈 [mm]: 귀 누르개와 목 뒷벽·귀 {ear.distance_to(moved):.2f} (귀 윗면에 앉는다), 귀 누르개와 돌림판 "
        f"{ear.distance_to(table):.2f}, 띠 밑 누르개와 몸통 {band.distance_to(core):.2f}"
    )


if __name__ == "__main__":
    report()
