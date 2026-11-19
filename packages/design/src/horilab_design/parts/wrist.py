"""손목 몸통: 앞 관절의 출력 원판에 앉는 발 + 목 + 다음 관절의 집. 손목 1과 손목 2에 같은 부품을 쓴다 (0029).

좌표는 이 몸통이 품는 서보의 좌표다 (parts/joint.py). 앞 관절의 축은 x 방향이고 (y, z) = (AX_Y, 0)을 지난다.
발 바닥(앞 관절 출력 원판의 윗면)은 x = X_FOOT이다. 서보 먼 끝이 앞 관절 쪽을 본다.

첫 관절(0028: 실제 팔 부품으로 뽑아 맞춰 본다)은 이 몸통 둘로 조립한다. 하나는 집, 하나는 자식이다.

    uv run python -m horilab_design.parts.wrist
"""

from __future__ import annotations

import math
from collections.abc import Callable

from build123d import Compound, Location, Polygon, Pos, Rot, extrude

from horilab_design.parts import joint as j
from horilab_design.parts.shapes import box, cyl_x, cyl_y, cyl_z, far_apart, nut_trap, solid
from horilab_design.printcheck import (
    air_wall_lines,
    mid_layer_levels,
    overhangs,
    printed_mass_g,
)
from horilab_design.values import ServoModel

# 서보 먼 끝과 앞 관절 출력 원판 사이: 혼 볼트 머리(1.65)가 들고, 그 양옆으로 선 둘(두께 1.5)이 지나간다
HEAD = 2.6
X_FOOT = -(j.FAR + HEAD)  # 발 바닥
AX_Y = 3.68  # 앞 관절 축이 지나는 y: 뒷덮개 바깥 면 ~ 원통 앞 끝의 가운데
PIN_Z = j.R_PIN  # 영점 핀 귀 (+z 쪽)
PIN_POSE_DEG = -90.0  # 영점 핀을 꽂는 관절 각: 자식의 핀 귀가 부모의 살 위에 온다
# 앞벽의 발 쪽 끝에 낸 선 홈 (0031). 홈은 멈춤 기둥과 귀가 도는 반지름(32.8까지) 밖에 있고, 다음 관절로 나가는
# 선이 여기서 그 관절의 축과 나란히 곧게 나온다. 손목 1: 들어오는 선(J4에서)은 -z 쪽 옆 홈, 나가는 선(J5로)은
# +z 쪽 옆 홈이다. 옆 홈은 가운데 홈보다 J5 축에서 멀어서, 손목 2의 영점 핀 귀가 쓸고 가는 자리(33.4)와 2.6 mm
# 뜬다 (가운데 홈은 1.9, studies/wiring.py의 strand_gap). 손목 2: 들어오는 선은 뒷덮개의 창으로 들어오고
# (wrist2_cover), 나가는 선(J6로)도 +z 쪽 옆 홈이다: 플랜지 원판의 영점 핀 귀(33.4)와 같은 2.6 mm가 뜬다
EXIT_HW = 3.0  # 가운데 홈의 반폭 (선 폭 4.85에 여유 1.15. 링크의 선 창과 같은 폭이다)
# 서보 먼 끝 밑(앞 관절 원판 위)에서 선이 지나는 길: 혼 볼트 머리(|z| 7.8까지)와 목 안쪽 벽(12.7) 사이의 4.9 mm다.
# 선 폭이 4.75 mm라, 선은 둥근머리의 낮은 가장자리에 조금 걸쳐 놓인다 (머리와 몸통은 같이 돌아 서로 쓸리지 않는다)
SIDE_LANE = (7.8 + j.NECK_IN) / 2
NOTCH = (7.0, j.NECK_IN)  # 옆 홈의 |z| 범위: 그 길의 입구라 목 안쪽 벽까지 튼다 (폭 5.7)
NOTCH_H = 2.4  # 홈의 높이 (발 바닥에서). 이 몸통이 품은 관절의 축에서 35.3 ~ 37.7 mm다
PIN_EAR_R = (
    4.4  # 영점 핀 귀 끝의 둥근 자리의 반지름: 귀는 앞 관절 축에서 PIN_Z + 4.4 = 33.4 mm까지 온다
)
HEAD_H = 1.65  # 발 나사(M3 둥근머리)의 머리가 귀 윗면 위로 나온 높이
# ---- 손목 2의 뒷덮개 (J5를 넘어온 선이 나오는 자리, 0031) ----
# 손목 몸통의 뒷덮개는 발 바닥까지 덮어서, 선이 덮개 밑으로 발까지 눕혀 내려와 원판 가장자리와 덮개 사이로
# 아래를 보고 나왔다. 그 자리는 영점 핀 귀가 닿는 반지름 안이고 귀와 같은 높이라, 남는 선이 U자로 꺾여 두 몸통
# 사이로 처졌다 (2026-11-19 시험). 손목 2의 덮개는 선을 발 위 EXIT_H에서 덮개 밖으로 곧게(축에서 멀어지는 쪽으로)
# 내보낸다: 귀 윗면(6.2)과 발 나사 머리(7.85) 위, D405 받침의 발(20.7)과 클립(11.3, 축에서 27 mm 밖) 아래다.
# 눕힌 선은 덮개 속 턱의 둥근 면(반지름 EXIT_R)을 타고 밖으로 돌고, 창의 위 가장자리(둥근 코)가 그 위를 덮는다.
# 턱이 선의 아래를 받치고 덮개 밑으로 더 내려가는 길을 막아서, 선은 턱보다 낮은 데서 나올 수 없다
EXIT_H = 13.0  # 턱의 높이 (발 바닥에서). 선은 턱에 얹혀 그 위 0 ~ 1.5 mm로 나온다 (귀 윗면에서 6.8, 나사 머리에서 5.15)
EXIT_R = j.PLUG_TOP - j.BACK_OUT  # 턱 둥근 면의 반지름 4.0: 눕힌 선의 밑면에서 덮개 바깥 면까지
EXIT_GAP = 2.2  # 턱과 코 사이 (선 두께 1.5에 0.7)
EXIT_RAMP_DEG = (
    42.0  # 턱은 이 각까지만 둥글고 그 위는 곧은 비탈이다 (덮개를 뽑을 때 걸치지 않는 각)
)
# 턱의 z 범위: -z 쪽 커넥터(가운데 -4.95)의 선이 얹힌다. 목 안쪽 벽(-12.7)과 0.5, 그리퍼로 가는 선(+2.5부터)과
# 4.4 띄운다. 창은 여기서 덮개의 -z 가장자리까지 트여 있어, 선을 옆에서 눕힌 채 밀어 넣는다 (플러그를 꿰지
# 않는다). 덮개를 덮으면 목의 옆벽이 덮개 속의 선을 막아 선이 그 틈으로 되나가지 못한다
EXIT_Z = (-(j.NECK_IN - 0.5), -1.9)
EXIT_PLUG_Z = -4.95  # 선을 꽂는 커넥터 (J6 서보 뒷면의 -z 쪽, 영점 핀 귀의 반대쪽)
# 원통 옆(-z 쪽, 영점 핀 귀의 반대쪽)의 돋움 둘: 손목 2에서는 D405 받침(parts/camera_mount.py)을, 손목 1에서는
# D405 선의 클립을 M3 나사 둘로 단다. 너트는 뒤(-y, 뽑을 때 위)에서 떨어뜨려 넣는다
MOUNT_X = 12.0  # 돋움 가운데의 |x|
MOUNT_HW = 4.5  # 돋움의 반폭
# 돋움의 y 범위: 뒷덮개 테(-17.8) 앞. 서보를 잠그는 벽 나사(-3.5)는 두 돋움 사이에 있다. 뒤 끝 -16.9는 몸통을
# 뽑을 때 층 경계(바닥에서 47.4)에 온다
MOUNT_Y = (-16.9, -7.0)
MOUNT_Z = 27.6  # 돋움 면의 |z| (원통 옆면에서 4.0)
MOUNT_SCREW_Y = -12.0
# 너트 홈의 폭과 두께(흰색 fit1_nut_drops 값: 저절로 내려가고 조여도 안 돈다), 돋움 면에서 홈까지의 살
_DROP = j._WHITE
assert _DROP.m3_nut_drop_width_mm is not None and _DROP.m3_nut_drop_thickness_mm is not None
MOUNT_NUT = (_DROP.m3_nut_drop_width_mm, _DROP.m3_nut_drop_thickness_mm, 1.6)
MOUNT_HOLE = 6.0  # 나사 구멍의 깊이 (돋움 면에서. 원통 벽에는 닿지 않는다)


def _ear_outline(side: int, pin: bool) -> list[tuple[float, float]]:
    """발의 귀 윤곽 (y, z). 앞 관절 출력 원판 반지름 안, 목 옆면 바깥. 바닥 쪽(+y) 아랫변은 45°로 선다."""
    r, wall = j.R_DRUM, j.NECK_HW - 0.01
    half = math.sqrt(r * r - wall * wall)
    pts: list[tuple[float, float]] = []
    if not pin:
        pts.append((AX_Y + half, side * wall))
        u = 2.66  # 벽에서 45°로 올라가다 원호를 만나는 자리
        pts.append((AX_Y + half - u, side * (wall + u)))
        a0 = math.atan2(half - u, wall + u)
        a1 = -math.atan2(half, wall)
        pts += [(AX_Y + r * math.sin(a), side * r * math.cos(a)) for a in _steps(a0, a1, 24)]
        return pts
    # 핀 귀: 핀 둘레 원(반지름 4.4)과 원판을 잇고, 아랫변은 핀 원에서 벽까지 45°
    pr = PIN_EAR_R
    t = pr / math.sqrt(2)
    pts.append((AX_Y + t + (PIN_Z + t - wall), side * wall))
    for a in _steps(math.radians(45.0), math.radians(-131.5), 12, first=True):
        pts.append((AX_Y + pr * math.sin(a), side * (PIN_Z + pr * math.cos(a))))
    phi = math.asin((r - pr) / PIN_Z)
    pts.append((AX_Y - r * math.cos(phi), side * r * math.sin(phi)))  # 원판 위쪽 접점
    for a in _steps(phi, math.atan2(wall, half), 8):
        pts.append((AX_Y - r * math.cos(a), side * r * math.sin(a)))
    return pts


def ear_section(pin: bool) -> list[tuple[float, float]]:
    """발의 귀를 앞 관절 축에서 본 윤곽 (u, v) = (y - AX_Y, z). pin이면 영점 핀 귀(+z), 아니면 반대쪽 귀다."""
    return [(y - AX_Y, z) for y, z in _ear_outline(1 if pin else -1, pin)]


def _steps(a0: float, a1: float, n: int, first: bool = False) -> list[float]:
    return [a0 + (a1 - a0) * k / n for k in range(0 if first else 1, n + 1)]


def mount_xy() -> list[tuple[float, float]]:
    """돋움 둘의 나사 자리 (x, y)."""
    return [(MOUNT_X, MOUNT_SCREW_Y), (-MOUNT_X, MOUNT_SCREW_Y)]


def _mount_bosses() -> Compound:
    """원통 옆의 돋움 둘. 원통 밖에만 살을 더한다. 뽑을 때 아래를 보는 면(+y 쪽)은 45°보다 조금 서게 깎는다."""
    y0, y1 = MOUNT_Y
    deep = 15.0  # 원통 속까지 넉넉히 그린 뒤 원통 바깥 면으로 자른다
    ramp = 1.1 * (MOUNT_Z - deep)
    out: Compound | None = None
    for x, _y in mount_xy():
        b = box(x - MOUNT_HW, x + MOUNT_HW, y0, y1, -MOUNT_Z, -deep)
        b += j.prism_x(
            [(y1, -MOUNT_Z), (y1 + ramp, -deep), (y1, -deep)], x - MOUNT_HW, x + MOUNT_HW
        )
        out = solid(b) if out is None else solid(out + b)
    assert out is not None
    return solid(out - cyl_y(j.R_DRUM - 0.01, y0 - 1, y1 + ramp + 1))


def _mount_cuts() -> Compound:
    """돋움의 너트 홈(뒤에서 떨어뜨려 넣는다)과 나사 구멍."""
    w, t, roof = MOUNT_NUT
    z_nut = -(MOUNT_Z - roof - t / 2)
    out: Compound | None = None
    for x, y in mount_xy():
        trap = nut_trap((x, y, z_nut), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0), y - MOUNT_Y[0] + 1, w, t)
        cut = trap + cyl_z(j.M3_FREE_H / 2, -MOUNT_Z - 1, -MOUNT_Z + MOUNT_HOLE, x, y)
        out = solid(cut) if out is None else solid(out + cut)
    assert out is not None
    return out


def wrist_body() -> Compound:
    """손목 몸통 (흰색). 원통 앞 끝(y = FRONT)을 바닥에 두고 뽑는다."""
    h = solid(j.drum_shell(X_FOOT) - j.drum_cavity(X_FOOT))
    h = solid(solid(h + _mount_bosses()) - _mount_cuts())
    # 발: 양옆 귀 (+z 쪽에 영점 핀 귀). 뽑을 때 선 판이다
    h = solid(h + j.prism_x(_ear_outline(1, True), X_FOOT, X_FOOT + j.EAR_T))
    h = solid(h + j.prism_x(_ear_outline(-1, False), X_FOOT, X_FOOT + j.EAR_T))
    for x, z in j.RIM:
        h = solid(h - cyl_x(j.M3_FREE_H / 2, X_FOOT - 1, X_FOOT + j.EAR_T + 1, AX_Y + x, z))
    h = solid(h - cyl_x(j.PIN_HOLE_H / 2, X_FOOT - 1, X_FOOT + j.EAR_T + 1, AX_Y, PIN_Z))
    # 선 홈: 앞벽의 발 쪽 끝. 가운데는 다음 관절로 나가는 선, 양옆은 앞 관절에서 들어오는 선
    spans = [(-EXIT_HW, EXIT_HW), (NOTCH[0], NOTCH[1]), (-NOTCH[1], -NOTCH[0])]
    for z0, z1 in spans:
        h = solid(h - box(X_FOOT - 1, X_FOOT + NOTCH_H, j.NECK_FRONT_IN - 1, j.FRONT + 1, z0, z1))
    return j.cut_joint_features(h, window=False)


def wrist_cover() -> Compound:
    """손목 몸통의 뒷덮개 (검정). 발 쪽 끝은 발 바닥에서 끝난다."""
    return j.back_cover(X_FOOT, end_skirt=False)


def _prism_z(pts: list[tuple[float, float]], z0: float, z1: float) -> Compound:
    """(x, y) 다각형을 z0 ~ z1로 뽑은 기둥."""
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1], strict=True))
    if area < 0:
        pts = pts[::-1]
    return solid(Pos(0, 0, z0) * extrude(Polygon(*pts, align=None), amount=z1 - z0))


def _exit_xy(s: float, d: float) -> tuple[float, float]:
    """선이 나오는 자리의 단면 좌표 (s: 턱에서 플러그 쪽으로, d: 덮개 바깥 면에서 서보 쪽으로) → (x, y)."""
    return X_FOOT + EXIT_H + s, j.BACK_OUT + d


def wrist2_cover() -> Compound:
    """손목 2의 뒷덮개 (검정): 손목 몸통의 뒷덮개에 J5를 넘어온 선이 나오는 창과 턱을 낸 것.

    창은 발 위 EXIT_H ~ EXIT_H + EXIT_GAP이고 덮개의 -z 가장자리까지 트여 있다. 턱은 덮개 안쪽 면에서 눕힌 선의
    밑면 높이까지 솟아, 덮개 밑으로 발 쪽에 가는 길을 막는다. 바깥 면을 바닥에 두고 뽑는다: 턱의 둥근 면은 위를
    보며 EXIT_RAMP_DEG까지만 기울고, 코는 위로 갈수록 물러난다.
    """
    c = j.back_cover(X_FOOT, end_skirt=False)
    edge = -(j.NECK_HW + 2.0)  # 덮개의 -z 가장자리
    x0, y0 = _exit_xy(0.0, 0.0)
    cx, cy = _exit_xy(EXIT_R, 0.0)  # 둥근 면의 가운데: 덮개 바깥 면 위, 턱에서 EXIT_R
    y_skirt = j.BACK_END + j.COVER_SKIRT
    c = solid(c - box(x0, cx, y0 - 1, y_skirt + 1, edge - 1, EXIT_Z[1]))
    # 코: 창의 위 가장자리. 둥근 면이 선의 안쪽을 (EXIT_GAP - 선 두께)만큼 띄우고 따라 돈다
    # 목의 옆벽과 덮개 테 위(턱 밖)에서는 판 두께까지만 둔다: 옆벽의 뒤 끝과 덮개 안쪽 면 사이는 0.7 mm뿐이다
    nose = cyl_z(EXIT_R - EXIT_GAP, edge, EXIT_Z[1], cx, cy) & box(
        x0, cx + EXIT_R, y0, y0 + EXIT_R, edge - 1, EXIT_Z[1] + 1
    )
    nose = solid(nose) - box(x0 - 1, cx + EXIT_R, j.BACK_IN, y0 + EXIT_R + 1, edge - 1, EXIT_Z[0])
    c = solid(c + solid(nose))
    # 턱: 둥근 면(가운데가 코와 같다)과 그 위의 곧은 비탈, 눕힌 선의 밑면 높이의 윗면
    a_max = math.radians(EXIT_RAMP_DEG)
    arc = [
        _exit_xy(EXIT_R * (1 - math.cos(a)), EXIT_R * math.sin(a))
        for a in _steps(0.0, a_max, 8, first=True)
    ]
    s1, d1 = EXIT_R * (1 - math.cos(a_max)), EXIT_R * math.sin(a_max)
    top = [
        _exit_xy(s1 + (EXIT_R - d1) * math.tan(a_max), EXIT_R),
        _exit_xy(-3.0, EXIT_R),
        _exit_xy(-3.0, 0.5),
        _exit_xy(0.0, 0.5),
    ]
    c = solid(c + _prism_z(arc + top, *EXIT_Z))
    # 바깥 면의 두 모서리를 조금 깎는다 (선이 위아래로 당겨질 때 닿는 모서리. 첫 층이 퍼져 창이 좁아지는 몫이기도 하다)
    lip = [_exit_xy(-0.3, -0.1), _exit_xy(0.1, -0.1), _exit_xy(0.1, 0.5), _exit_xy(0.0, 0.5)]
    nose_cut = [
        _exit_xy(EXIT_GAP - 0.1, -0.1),
        _exit_xy(EXIT_GAP + 0.3, -0.1),
        _exit_xy(EXIT_GAP - 0.1, 0.55),
    ]
    for pts in (lip, nose_cut):
        c = solid(c - _prism_z(pts, edge - 1, EXIT_Z[1]))
    if not c.is_valid or len(c.solids()) != 1:
        raise RuntimeError("손목 2 뒷덮개 형상이 깨졌다")
    return c


def wrist2_wire() -> list[tuple[float, float, float]]:
    """손목 2 속에서 J5를 넘어온 선의 가운데 (서보 좌표의 꺾은선): 덮개 밖 끝 → 창 → 턱의 둥근 면 → 눕힌 선 → 플러그.

    덮개 밖 6 mm는 창이 정한 방향(축에서 멀어지는 쪽)으로 곧게 그린다. 그 뒤는 자유로운 고리다.
    """
    rc = EXIT_R - j.WIRE_T / 2
    cx, cy = _exit_xy(EXIT_R, 0.0)
    z = EXIT_PLUG_Z
    pts = [(cx - rc, cy - 6.0, z)]
    pts += [
        (cx - rc * math.cos(a), cy + rc * math.sin(a), z)
        for a in _steps(0.0, math.pi / 2, 8, first=True)
    ]
    y_lie = (j.WIRE_TOP + j.PLUG_TOP) / 2
    return [*pts, (-14.2 - 3.3, y_lie, z), (-14.2, j.PLUG_TOP, z)]


def wrist2_wire_shape() -> Compound:
    """그 선이 차지하는 자리 (겹침 검사와 그림용): 폭 WIRE_W, 두께 WIRE_T. 플러그 바로 뒤의 굽은 자리는 뺀다."""
    cx, cy = _exit_xy(EXIT_R, 0.0)
    z0, z1 = EXIT_PLUG_Z - j.WIRE_W / 2, EXIT_PLUG_Z + j.WIRE_W / 2
    r0, r1 = EXIT_R - j.WIRE_T, EXIT_R
    bend = (cyl_z(r1, z0, z1, cx, cy) - cyl_z(r0, z0 - 1, z1 + 1, cx, cy)) & box(
        cx - r1, cx, cy, cy + r1, z0, z1
    )
    out = box(cx - r1, cx - r0, cy - 6.0, cy, z0, z1)  # 덮개 밖
    lie = box(cx, -16.6, j.WIRE_TOP, j.PLUG_TOP, z0, z1)  # 플러그 밑까지 눕힌 선
    return solid(solid(bend) + out + lie)


def wrist2_inner(model: ServoModel = "STS3215") -> list[j.Part]:
    """손목 2 속 (J6의 집): 원통 속 부품과 서보, 손목 2의 뒷덮개, J5를 넘어온 선. 몸통은 뺀다 (서보 좌표)."""
    dy = j.servo_dy(model)
    parts = [p for p in j.inner_parts(model, X_FOOT, False, wrist2_cover()) if p.name != "선 자리"]
    # 그리퍼로 가는 선은 +z 쪽 커넥터에서 발까지 눕는다 (서보 모형의 선 자리에서 +z 쪽 절반)
    out = box(-36.0, -11.85, j.WIRE_TOP, j.PLUG_TOP, 2.5, 8.5)
    parts.append(j.Part("선 (그리퍼로)", solid(Pos(0, dy, 0) * out), color="wire"))
    parts.append(j.Part("선 (J5에서)", solid(Pos(0, dy, 0) * wrist2_wire_shape()), color="wire"))
    return parts


def as_child(shape: Compound, q_deg: float = 0.0) -> Compound:
    """자식 손목 몸통의 좌표를 부모 관절의 서보 좌표로 옮긴다.

    자식의 x → 부모의 +y(출력축), 자식의 y → 부모의 +x (관절 각 0에서 자식의 앞이 부모의 가까운 끝 쪽), z → -z.
    자식의 발 바닥이 부모 출력 원판 윗면(ATOP)에 앉고, 자식의 (y, z) = (AX_Y, 0) 선이 부모 축이다.
    """
    return j.turned(solid(_child_at() * shape), q_deg)


def _child_at() -> Location:
    """자식 손목 몸통의 좌표를 부모 관절의 서보 좌표로 옮기는 자리 (관절 각 0)."""
    return Pos(-AX_Y, j.ATOP - X_FOOT, 0) * (Rot(0, 180, 0) * Rot(0, 0, 90))


def parent_disc(model: ServoModel = "STS3215") -> list[j.Part]:
    """이 몸통이 앉은 앞 관절의 출력 원판과 멈춤 나사, 너트를 이 몸통의 서보 좌표로 옮긴 것 (이 몸통과 함께 서 있다).

    앞 관절의 멈춤 귀는 이 몸통의 출력 쪽(+y)을 보고, 그 끝이 이 몸통의 출력 원판 윗면 바로 밑(0.6 mm)까지 온다
    (손목 2에 단 도구가 J6를 따라 돌 때 그 위를 지나간다). J4의 원판은 손목 1에 반 바퀴 돌려 달아서 귀가 반대쪽이다.
    """
    back = _child_at().inverse()
    parts = [j.Part("앞 관절의 출력 원판", solid(back * j.out_disc()), color="cap")]
    for p in j.inner_hardware(model):
        if p.name.startswith(("멈춤 나사", "너트 (멈춤")):
            parts.append(
                j.Part(f"앞 관절의 {p.name}", solid(back * p.shape), j.STEEL, False, "metal")
            )
    return parts


def from_parent(flip: bool = False) -> Location:
    """앞 관절의 서보 좌표를 이 몸통의 서보 좌표로 옮기는 자리 (앞 관절 각 0, 범위 가운데).

    flip이면 앞 관절 축 둘레로 반 바퀴 돌려 단 것이다: J4의 원판을 손목 1에 달 때처럼 멈춤 귀가 이 몸통의 출력 쪽
    반대를 본다.
    """
    back = _child_at().inverse()
    if flip:
        back = Pos(0, AX_Y, 0) * Rot(180, 0, 0) * Pos(0, -AX_Y, 0) * back
    return back


def foot_mount(model: ServoModel = "STS3215", flip: bool = False) -> list[j.Part]:
    """이 몸통의 발에 단 앞 관절의 출력 원판과 그 밑면의 너트 넷, 발 나사(몸통 나사 M3 × 10) 넷 (이 몸통의 서보 좌표).

    작업 안내가 발에 원판을 다는 단계를 그릴 때 쓴다 (형상은 parent_disc, child_hardware와 같다). flip은
    from_parent와 같다.
    """
    back = from_parent(flip)
    parts = [j.Part("앞 관절의 출력 원판", solid(back * j.out_disc()), color="cap")]
    for p in j.inner_hardware(model):
        if p.name.startswith("너트 (출력 원판)"):
            parts.append(
                j.Part(f"앞 관절의 {p.name}", solid(back * p.shape), j.STEEL, False, "metal")
            )
    x = X_FOOT + j.EAR_T
    for k, (dy, z) in enumerate(j.RIM):
        s = cyl_x(1.5, x - j.BODY_SCREW, x, AX_Y + dy, z) + cyl_x(2.85, x, x + 1.65, AX_Y + dy, z)
        parts.append(j.Part(f"발 나사 M3x10 {k + 1}", solid(s), j.STEEL, False, "metal"))
    return parts


def child_hardware() -> list[j.Part]:
    """자식 몸통 나사 M3 × 10 넷과 영점 핀 (핀은 PIN_POSE_DEG에 꽂은 자리)."""
    out: list[j.Part] = []
    x = X_FOOT + j.EAR_T
    for k, (dy, z) in enumerate(j.RIM):
        s = cyl_x(1.5, x - j.BODY_SCREW, x, AX_Y + dy, z) + cyl_x(2.85, x, x + 1.65, AX_Y + dy, z)
        out.append(j.Part(f"몸통 나사 M3x10 {k + 1}", as_child(solid(s)), j.STEEL, True, "metal"))
    pin = cyl_x(1.5, x + 3.0 - 20.0, x + 3.0, AX_Y, PIN_Z)
    out.append(j.Part("영점 핀 MSH3-20", as_child(solid(pin), PIN_POSE_DEG), j.STEEL, False, "pin"))
    return out


def first_joint(model: ServoModel = "STS3215") -> list[j.Part]:
    """첫 관절 조립: 손목 몸통(부모, 집)과 속 부품, 출력 원판, 손목 몸통(자식), 나사와 핀."""
    body = wrist_body()
    parts = [j.Part("손목 몸통 (부모)", body, color="body")]
    parts += j.inner_parts(model, X_FOOT, end_skirt=False)
    parts.append(j.Part("손목 몸통 (자식)", as_child(body), moving=True, color="body"))
    return parts + j.inner_hardware(model) + child_hardware()


def clearances(model: ServoModel, q_deg: float, limit: float = 2.5) -> dict[tuple[str, str], float]:
    """관절 각 q에서 도는 부품과 서 있는 부품 사이 틈 [mm] (limit 아래만). 영점 핀은 뺀다."""
    parts = first_joint(model)
    static = [p for p in parts if not p.moving and p.name != "베어링 6806" and "핀" not in p.name]
    static = [p for p in static if not p.name.startswith("벽 나사")]
    out: dict[tuple[str, str], float] = {}
    for a in (p for p in parts if p.moving):
        name = a.name.rstrip("0123456789 ").strip()
        shape = j.turned(a.shape, q_deg)
        for b in static:
            if b.name == "서보 몸통" and (name.startswith("혼") or name.startswith("허브")):
                continue  # 혼과 혼 볼트 끝은 서보 앞면과 0.3 (서보 자체의 틈), 허브 판은 앞면 판과 1.7 ~ 1.8
            if far_apart(shape, b.shape, limit):
                continue
            d = shape.distance_to(b.shape)
            if d < limit:
                out[(name, b.name)] = min(d, out.get((name, b.name), limit))
    return out


# 뽑는 부품의 한글 이름 (출력 안내에 쓴다, horilab_design.slicing)
LABELS = {
    "wrist_body": "손목 몸통",
    "out_disc": "출력 원판",
    "hub_plate": "허브 판",
    "retainer": "누름 틀",
    "carrier": "서보 받침",
    "wrist_cover": "뒷덮개",
    "wrist2_cover": "뒷덮개 (손목 2)",
}


# 뽑는 부품: 이름 → (형상을 만드는 함수, 뽑을 때 위를 보는 방향, 색, 프리셋). 작업 안내는 형상을 만들지 않고
# 이름, 색, 파일 이름만 읽는다 (horilab_design.guide)
PRINT_SPEC: dict[str, tuple[Callable[[], Compound], tuple[float, float, float], str, str]] = {
    "wrist_body": (wrist_body, (0.0, -1.0, 0.0), "white", "strength"),
    "out_disc": (j.out_disc, (0.0, -1.0, 0.0), "black", "strength"),
    "hub_plate": (j.hub_plate, (0.0, -1.0, 0.0), "black", "strength"),
    "retainer": (j.retainer, (0.0, -1.0, 0.0), "black", "strength"),
    "carrier": (j.carrier, (0.0, 1.0, 0.0), "black", "strength"),
    "wrist_cover": (wrist_cover, (0.0, 1.0, 0.0), "black", "standard"),
    "wrist2_cover": (wrist2_cover, (0.0, 1.0, 0.0), "black", "standard"),
}


def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    """뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋)."""
    return {
        key: (make(), up, color, preset) for key, (make, up, color, preset) in PRINT_SPEC.items()
    }


def report(model: ServoModel = "STS3215") -> None:
    parts = first_joint(model)
    print(
        f"첫 관절 (손목 몸통 둘, {model}): 발 바닥 x {X_FOOT:.2f}, 앞 관절 축 y {AX_Y}, 귀 두께 {j.EAR_T}"
    )
    print(f"임시 값: {', '.join(j.PROVISIONAL) or '없음'}")
    print("\n뽑는 부품 (무게는 뽑아서 잰 부품에 맞춘 어림, 걸침은 바닥에 닿지 않고 아래를 보는 면)")
    for name, (shape, up, color, preset) in printed().items():
        bb = shape.bounding_box()
        over = overhangs(shape, up)
        print(
            f"  {name:12s} {color:5s} {bb.size.X:5.1f} × {bb.size.Y:5.1f} × {bb.size.Z:5.1f} mm  "
            f"{printed_mass_g(shape, up, preset):5.1f} g  걸침 {over.total:5.0f} mm² (거의 수평 {over.flat:.0f}, "
            f"가장 넓은 한 자리 {over.largest:.0f})"
        )
    mid = {n: mid_layer_levels(s, up) for n, (s, up, _c, _p) in printed().items()}
    print(
        "층 가운데에 걸친 수평면 (한 층이 더 뽑힐지가 정해지지 않는다): "
        + (", ".join(f"{n} {z}" for n, z in mid.items() if z) or "없음")
    )
    print("\n".join(air_wall_lines(printed())))
    paths = {name: j.nut_entry_overlap(part, c, a, s) for name, part, c, a, s in j.side_nuts()}
    print(
        "옆 홈에 너트를 곧게 밀어 넣을 때 겹침 [mm³]: "
        + (", ".join(f"{n} {v:.2f}" for n, v in paths.items() if v > 0.01) or "없음")
    )
    hits = j.overlaps([p for p in parts if "핀" not in p.name])
    print(
        "\n겹침 [mm³]: " + (", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in hits.items()) or "없음")
    )
    print("도는 부품과 서 있는 부품 사이 틈 [mm] (2.5 아래만)")
    for q in (0.0, 90.0, j.SOFT_DEG, j.STOP_DEG, -90.0, -j.SOFT_DEG, -j.STOP_DEG):
        gaps = sorted((d, a, b) for (a, b), d in clearances(model, q).items())
        print(f"  {q:7.1f}°: " + ", ".join(f"{a}↔{b} {d:.2f}" for d, a, b in gaps[:5]))
    body = wrist_body()
    pin = next(p for p in parts if "핀" in p.name)
    print(
        f"영점 핀 (관절 각 {PIN_POSE_DEG:.0f}°): 부모 몸통과 틈 {pin.shape.distance_to(body):.2f}, "
        f"자식 몸통과 틈 {pin.shape.distance_to(as_child(body, PIN_POSE_DEG)):.2f} (구멍 틈 0.15 ~ 0.18)"
    )


if __name__ == "__main__":
    report()
