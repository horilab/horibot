"""손목 D405 받침: 손목 2 몸통(J6의 집) 옆의 돋움 둘에 다는 꺾인 판 (0018, 0029의 "D405 받침").

D405는 J6 원통 옆(-z 쪽, 영점 핀 귀의 반대쪽)에 있고, 광축을 도구 축 쪽으로 20° 기울인다 (0017). 카메라는
광축 둘레로 90° 돌려 단다: 커넥터가 있는 옆면이 J6 원통 쪽을 보고, 카메라의 윗면이 +x(J5 반대쪽), 바닥면(1/4
나사 구멍)이 J5 쪽을 본다. 그래서 얇은 USB 2 선의 ㄱ자 플러그(0031)가 카메라와 원통 사이에 들고, 선이 손목
몸통을 따라 J5 쪽으로 나온다. 카메라 뒷면의 M3 구멍 둘(간격 20 mm)은 원통에서 멀어지는 쪽으로 나란히 놓이고,
받침의 뒤에서 나사를 조인다. 받침의 발은 손목 몸통의 돋움 둘(parts/wrist.py)에 M3 나사 둘로 단다.

카메라의 치수는 공식 사양서와 CAD에서 읽은 값이고(docs/hardware.md의 D405), 플러그는 잰 값이다
(data/measured.toml의 d405_usb2_plug). 몸통이 옆면의 어디에 붙는지도 잰 값이다 (PLUG_TOP, 2026-11-19).

플러그의 고무 끝 바로 밑에서 선을 무는 클립을 그렸다 (0031의 "D405 선": 선의 고리가 흔들리거나 당겨져도 플러그에
힘이 가지 않게 한다). 판의 앞면에서 솟은 가운데 받침과 두 갈래이고, 선을 앞(도구 쪽)에서 눌러 끼운다. 클립의
자리는 플러그의 값(PLUG_TOP 등)을 따라간다. 그런데 잰 자리에서는 고무 끝이 받침의 J5 쪽 면보다 2.0 mm 밖이라,
클립이 판에서 솟지 못하고 판 밖에 뜬다 (clip_rooted). 그래서 받침에는 클립이 없다. 플러그 밑에서 곧게 내려오는
D405 선은 모델에서 J5를 넘는 서보 선이 손목 2 겉을 도는 자리(발 위 11 ~ 16 mm)를 가로지르지만(studies/wiring.py),
첫 관절에 달아 손으로 돌려 보니 두 선이 만나지 않아 이 배치와 클립 없는 받침으로 간다 (2026-11-19). 플러그 밑에서
선을 잡는 방법은 따로 정한다 (PROVISIONAL).

좌표는 joint.py의 서보 좌표다 (y가 J6 축, 플랜지 면이 y = ATOP, -x가 J5 쪽).

    uv run python -m horilab_design.parts.camera_mount
"""

from __future__ import annotations

import math

from build123d import Compound, Location, Plane, Polygon, Pos, Rot, extrude

from horilab_design.parts import flange, wrist
from horilab_design.parts import joint as j
from horilab_design.parts.shapes import box, cyl_x, cyl_z, far_apart, solid
from horilab_design.printcheck import (
    LAYER,
    air_wall_lines,
    mid_layer_levels,
    overhangs,
    printed_mass_g,
)
from horilab_design.values import ServoModel, load_measured

_MEASURED = load_measured()
assert _MEASURED.d405_usb2_plug is not None and _MEASURED.d405_usb2_cable is not None
_PLUG, _CABLE = _MEASURED.d405_usb2_plug, _MEASURED.d405_usb2_cable
assert _CABLE.diameter_mm is not None

# ---- D405 (docs/hardware.md: 공식 사양서와 CAD) ----
# 카메라 자신의 가로(커넥터가 있는 옆면까지) × 세로 × 깊이. 90° 돌려 달아서 가로가 원통에서 멀어지는 쪽이다
CAM_W, CAM_H, CAM_D = 42.0, 42.0, 23.0
CAM_HOLE_X = 10.0  # 뒷면의 M3 구멍 둘: 카메라의 가로 가운데에서 ±10, 세로 가운데
CAM_HOLE_DEPTH = 4.0  # 나사가 들어갈 수 있는 깊이
CONN_BACK = 6.82  # 옆면 커넥터의 가운데: 뒷면에서
# ---- 플러그: 얇은 USB 2 선의 ㄱ자 플러그 (잰 값, 2026-11-15) ----
PLUG_OUT = _PLUG.protrusion_mm  # 커넥터가 있는 옆면에서 플러그의 가장 바깥까지
PLUG_LEN = _PLUG.body_length_mm  # 몸통의 길이: 카메라의 세로 방향이고, 선은 바닥면 쪽 끝에서 나온다
PLUG_BODY = _PLUG.body_out_mm  # 꽂는 방향의 몸통 두께 (선은 그 가운데에서 나온다)
PLUG_ACROSS = _PLUG.body_across_mm  # 카메라의 깊이 방향 폭
if _PLUG.top_mm is None:
    raise ValueError("D405 플러그 몸통의 자리(d405_usb2_plug.top_mm)를 재지 않았다")
# 몸통 위끝이 카메라 윗면에서 내려온 거리 (잰 값, 2026-11-19). 위끝은 비스듬히 깎여 옆면 쪽이 가장 높고, 잰 값은 그
# 가장 높은 곳이다. 상자로 그린 몸통은 그보다 크게 잡는 쪽이다
PLUG_TOP = _PLUG.top_mm
WIRE_D = _CABLE.diameter_mm  # 그 선의 지름
PLUG_GAP = 1.0  # 플러그와 손목 몸통(원통, 벽 나사 머리) 사이에 남길 틈

# ---- 카메라 자리 ----
TILT = 20.0  # 광축을 도구 축 쪽으로 기울인 각 [°] (0017)
# 카메라의 안쪽 앞 모서리와 원통 옆면 사이. 커넥터 면과 원통 사이에 플러그(PLUG_OUT)가 들고 PLUG_GAP이 남는
# 값이다 (plug_clearance). 가장 가까운 곳은 플러그의 앞 안쪽 모서리와 원통이다
DRUM_GAP = 6.0
# 렌즈 면의 가운데가 플랜지 면보다 뒤에 있는 거리. J6 멈춤 기둥의 나사 끝(이음매에서 8.0 아래, 축에서 26.5 ~ 29.5)이
# 카메라 앞을 2 mm 넘게 띄우고 지나가는 값이다 (8.0이면 0.9 mm)
LENS_BACK = 9.2
_T = math.radians(TILT)
AXIS = (0.0, math.cos(_T), math.sin(_T))  # 광축 (도구 쪽, J6 축 쪽으로 기운다)
INWARD = (0.0, -math.sin(_T), math.cos(_T))  # 카메라의 세로 방향 가운데 J6 축을 보는 쪽
LENS = (
    0.0,
    j.ATOP - LENS_BACK,
    -(j.R_DRUM + DRUM_GAP + CAM_H / 2 * math.cos(_T)),
)  # 렌즈 면의 가운데
# 카메라 좌표: 원점이 렌즈 면 가운데, x가 서보의 +x(카메라의 윗면 쪽), y가 J6 축에서 멀어지는 쪽(카메라의
# 가로. 커넥터가 있는 옆면은 -y), z가 광축
FRAME = Location(Plane(origin=LENS, x_dir=(1.0, 0.0, 0.0), z_dir=AXIS))

# ---- 받침 ----
# 카메라 뒤의 판. 카메라 무게에 판이 휘어 카메라가 기우는 양이 화소 반 칸(1280 화소에서 0.034°)보다 작은 두께다
# (tilt_under_weight). 카메라를 플러그만큼 밖으로 옮겨 팔이 길어져서 4.5로는 0.036°다
PLATE_T = 5.0
MOUNT_W = 34.0  # 받침의 폭 (발 나사 ±12를 품는다. 카메라 폭 42보다 좁혀 무게를 줄였다)
HEAD_REC = 2.0  # 카메라 나사 머리 자리의 깊이: 남은 살 3.0에 M3 × 6이 카메라에 3.0 들어간다
CAM_SCREW = 6.0
FOOT_T = 3.0  # 돋움에 앉는 발
FOOT_Y0 = wrist.MOUNT_Y[0] - 0.5  # 발의 뒤 끝
FOOT_SCREW = 8.0  # M3 × 8: 발 3.0 + 돋움 속 5.0 (너트를 0.6 지나간다)
FILLET = 1.5  # 발과 판이 만나는 안쪽 모서리를 메운 길이
HEAD_RELIEF_R = (
    3.3  # 발 나사 머리(반지름 2.85)가 앉는 자리의 반지름: 판의 뿌리와 메운 자리를 이만큼 판다
)
# ---- 클립: 플러그의 고무 끝 바로 밑에서 선을 문다 (0031) ----
# 단면은 카메라의 (가로 y, 광축 z) 평면에 그리고 선 방향(카메라의 세로 x)으로 뽑은 기둥이다. 받침을 눕혀 뽑을 때 이
# 단면이 층마다 그려져서 치수가 정확하고, 갈래가 휘는 힘이 층을 따라 걸린다. 높이 p는 판 앞면에서 잰다 (선의
# 가운데가 p = CONN_BACK). 판 앞면에서 솟은 가운데 받침이 선의 밑을 받치고, 받침과 0.6 mm 틈으로 갈라진 두 갈래가
# 선을 양옆에서 물며, 갈래 끝의 턱이 선을 위에서 막는다. 갈래는 판 가까이에서 갈라져 길고(선의 가운데까지 5.4 mm)
# 얇아서, 선을 누르면 갈래가 벌어진다 (선이 가늘어 세게 누르면 상한다)
CLIP_PINCH = 0.15  # 갈래가 선을 무는 양 (한쪽). 두 갈래 사이가 선 지름보다 0.3 좁다 (지름의 10%)
CLIP_T = 1.2  # 갈래의 두께
CLIP_POST_HW = 0.75  # 가운데 받침의 반폭
CLIP_ROOT = 1.4  # 갈래가 갈라지기 시작하는 높이 (틈의 바닥). 그 밑은 한 덩어리다
# 덩어리가 판 앞면 뒤로 들어간 깊이 (판 밖으로 나온 구간에서 덩어리의 두께가 2.0이다)
CLIP_BACK = 0.6
CLIP_LIP = 0.3  # 턱이 선의 반지름보다 안으로 들어온 양 (입구의 폭이 선 지름 - 0.6)
CLIP_LIP_DEG = 30.0  # 턱이 선에 닿는 자리: 선의 가장 넓은 곳에서 위로 잰 각
# 갈래 끝이 선의 가운데보다 높은 양. 갈래 끝 사이가 3.3으로 선 지름보다 넓어서 선이 저절로 자리를 잡는다
CLIP_TIP = 1.6
CLIP_LEN = 5.0  # 단면이 다 있는 길이 (선 방향)
# 플러그 몸통의 J5 쪽 아래 모서리와 클립의 비스듬한 면 사이 (PLUG_GAP보다 넓다)
CLIP_PLUG_GAP = 1.2
PROVISIONAL = (
    "플러그 밑에서 D405 선을 잡는 방법 (클립은 잰 플러그 자리에서 받침 판 밖에 떠서 뺐고, 받침은 클립 없이 쓴다)",
)


def _yz(p: tuple[float, float, float]) -> tuple[float, float]:
    return (p[1], p[2])


def _back_edge() -> tuple[float, float]:
    """카메라 뒷면의 안쪽 모서리 (y, z)."""
    return (
        LENS[1] - CAM_D * AXIS[1] + CAM_H / 2 * INWARD[1],
        LENS[2] - CAM_D * AXIS[2] + CAM_H / 2 * INWARD[2],
    )


def camera() -> Compound:
    """D405 몸통 (틈 검사와 그림에 쓴다)."""
    body = box(-CAM_W / 2, CAM_W / 2, -CAM_H / 2, CAM_H / 2, -CAM_D, 0.0)
    return solid(body.moved(FRAME))


def plug() -> Compound:
    """커넥터에 꽂은 ㄱ자 플러그가 차지하는 자리: 커넥터가 있는 옆면(-y)에 세로로 붙은 몸통.

    옆면에서 PLUG_OUT까지를 통째로 넣는다 (몸통 두께 PLUG_BODY에 꽂는 목까지). 선은 -x 끝에서 나온다.
    """
    x1 = CAM_H / 2 - PLUG_TOP
    z = -CAM_D + CONN_BACK
    body = box(
        x1 - PLUG_LEN,
        x1,
        -CAM_W / 2 - PLUG_OUT,
        -CAM_W / 2,
        z - PLUG_ACROSS / 2,
        z + PLUG_ACROSS / 2,
    )
    return solid(body.moved(FRAME))


def wire_exit() -> tuple[float, float, float]:
    """플러그에서 선이 나오는 곳 (카메라 좌표): 몸통의 -x 끝, 몸통 두께의 가운데."""
    return (
        CAM_H / 2 - PLUG_TOP - PLUG_LEN,
        -CAM_W / 2 - PLUG_OUT + PLUG_BODY / 2,
        -CAM_D + CONN_BACK,
    )


def to_servo(p: tuple[float, float, float]) -> tuple[float, float, float]:
    """카메라 좌표의 점을 서보 좌표로 옮긴다."""
    x, y, z = p
    oy, oz = -INWARD[1], -INWARD[2]
    return (LENS[0] + x, LENS[1] + y * oy + z * AXIS[1], LENS[2] + y * oz + z * AXIS[2])


# 선이 손목 2 몸통을 따라가는 높이: 받침의 발(x -MOUNT_W / 2까지) 바로 밑이다
RUN_X = -MOUNT_W / 2 - WIRE_D / 2 - 3.0
LEAD_Y = j.BACK_OUT - WIRE_D / 2  # 뒷덮개 바깥 면에 붙어 내려가는 선의 가운데


def wire_run() -> list[tuple[float, float, float]]:
    """플러그에서 나온 선이 손목 2 몸통을 따라 뒷덮개 가운데까지 가는 길 (서보 좌표의 꺾은선, 어림).

    플러그 끝에서 J5 쪽으로 곧게 나와 받침의 발 밑에서 뒤로 돌고, 목의 뒤 모서리를 돌아 뒷덮개의 가운데에서
    끝난다. 그 밑으로는 J5를 넘는 느슨한 구간이다 (studies/wiring.py).

    선을 두 몸통 사이의 층으로 넘기던 때의 길이다. 배선 계산(studies/wiring.py)과 팔 전체 모델(studies/ur_arm.py)이
    아직 이 길을 쓴다. 지금은 선이 관절 밖으로 넘고(0031), 플러그 밑에서는 받침의 클립이 문다 (clip, wire_stub).
    클립 밑으로 가는 길은 그 둘을 고칠 때 그린다.
    """
    ex, ey, ez = to_servo(wire_exit())
    side = -(j.NECK_HW + WIRE_D / 2 + 4.0)  # 목의 옆벽에서 조금 뜬 자리
    return [
        (ex, ey, ez),
        (ex - 5.0, ey, ez),
        (RUN_X, ey - 9.0, (ez + side) / 2),
        (RUN_X, j.BACK_OUT + 6.0, side),
        (RUN_X, LEAD_Y - 1.5, -(j.NECK_HW - 3.0)),  # 뒤 모서리를 돈다
        (RUN_X, LEAD_Y, 0.0),
    ]


def clip_span() -> tuple[float, float, float]:
    """클립의 선 방향 자리 (카메라의 x): (솟기 시작하는 곳, 단면이 다 차는 곳, 끝).

    클립은 판 앞면에서 J5 쪽으로 가며 45°로 솟는다 (받침을 눕혀 뽑을 때 서포트 없이 뽑히는 각). 그 비스듬한 면이
    플러그 몸통의 J5 쪽 아래 모서리와 CLIP_PLUG_GAP을 띄우고 지나간다. 그래서 고무 끝을 지나야 선에 닿고, 갈래
    끝까지 다 솟은 곳부터 CLIP_LEN만큼이 온 단면이다. 끝 면은 뽑을 때 층 경계에 오게 맞춘다. 플러그의
    자리(PLUG_TOP)가 바뀌면 그대로 따라간다.
    """
    plug_end = CAM_H / 2 - PLUG_TOP - PLUG_LEN
    start = plug_end + (CONN_BACK - PLUG_ACROSS / 2) - CLIP_PLUG_GAP * math.sqrt(2)
    full = start - (CONN_BACK + CLIP_TIP)
    bed = MOUNT_W / 2  # 뽑을 때 바닥에 닿는 면 (+x 쪽)
    end = bed - math.ceil((bed - (full - CLIP_LEN)) / LAYER - 1e-6) * LAYER
    return start, full, end


def clip_rooted() -> bool:
    """클립이 판의 앞면에서 솟는지. 솟기 시작하는 곳이 받침의 J5 쪽 면보다 안(+x)이어야 판에 뿌리가 있다."""
    return clip_span()[0] + CLIP_BACK > -MOUNT_W / 2 + CLIP_T


def clip_outline() -> list[tuple[float, float]]:
    """클립의 단면 (q, p): q는 선의 가운데에서 카메라의 가로(y) 쪽, p는 판 앞면에서 광축 쪽. q = 0에 대칭이다.

    가운데 받침의 윗면 → 틈(바닥이 둥글다) → 갈래의 안쪽 면 → 턱 → 들어가는 비탈 → 갈래 끝 → 갈래의 바깥 면 차례다.
    받침의 윗면과 턱은 선(반지름 r)에 닿는 자리에서 0.02 띄워 그린다. 갈래의 안쪽 면만 선을 CLIP_PINCH씩 문다.
    """
    r = WIRE_D / 2
    touch = r + 0.02
    p_w = CONN_BACK
    q_in = r - CLIP_PINCH
    q_out = q_in + CLIP_T
    slit = q_in - CLIP_POST_HW
    half: list[tuple[float, float]] = []
    for k in range(7):  # 받침의 윗면: 선의 밑을 따라 오목하다
        q = CLIP_POST_HW * k / 6
        half.append((q, p_w - math.sqrt(touch * touch - q * q)))
    cq, cp = CLIP_POST_HW + slit / 2, CLIP_ROOT + slit / 2
    for k in range(9):  # 틈의 둥근 바닥
        a = math.pi * (1 + k / 8)
        half.append((cq + slit / 2 * math.cos(a), cp + slit / 2 * math.sin(a)))
    t = math.radians(CLIP_LIP_DEG)
    tq, tp = touch * math.cos(t), p_w + touch * math.sin(t)  # 턱이 선에 닿는 곳
    q_lip = r - CLIP_LIP
    p_lip = tp + (tq - q_lip) / math.tan(t)
    p_tip = p_w + CLIP_TIP
    half += [
        (q_in, tp - (q_in - tq) / math.tan(t)),  # 턱의 밑 (선의 둘레에 접하는 비탈)
        (q_lip, p_lip),
        (q_lip, p_lip + 0.2),
        (q_lip + (p_tip - p_lip - 0.2), p_tip),  # 들어가는 비탈 45°
        (q_out, p_tip),
        (q_out, -CLIP_BACK),
    ]
    return half + [(-q, p) for q, p in reversed(half[1:])]


def _prism_y(pts: list[tuple[float, float]], y0: float, y1: float) -> Compound:
    """(x, z) 다각형을 y0 ~ y1로 뽑은 기둥."""
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1], strict=True))
    if area < 0:
        pts = pts[::-1]  # 반시계 방향으로 맞춘다
    body = extrude(Polygon(*pts, align=None), amount=y1 - y0)
    return solid(Pos(0, y1, 0) * (Rot(90, 0, 0) * body))


def clip() -> Compound:
    """선을 무는 클립 (받침의 한 부분, 서보 좌표). 단면 기둥을 45° 면으로 깎은 것이다 (clip_span)."""
    start, _full, end = clip_span()
    _x, yw, _z = wire_exit()
    pts = [(yw + q, -CAM_D + p) for q, p in clip_outline()]
    body = j.prism_x(pts, end, start + CLIP_BACK)
    # 45° 면의 위쪽을 깎는다: 판 앞면에서 잰 높이 p가 start - x보다 높은 곳
    far = start - end + 1.0
    over = [
        (start + 1.0, -CAM_D - 1.0),
        (start + 1.0, -CAM_D + far + 20.0),
        (end - 1.0, -CAM_D + far + 20.0),
        (end - 1.0, -CAM_D + far),
    ]
    half = WIRE_D / 2 + CLIP_T + 1.0
    body = solid(body - _prism_y(over, yw - half, yw + half))
    return solid(body.moved(FRAME))


def wire_stub() -> Compound:
    """플러그의 고무 끝에서 J5 쪽(-x)으로 곧게 나와 클립의 끝까지 가는 선 (틈 검사와 그림에 쓴다).

    클립 속에서는 두 갈래에 눌려 양옆이 CLIP_PINCH씩 납작하다. 클립 밑으로 관절 밖의 고리로 가는 길은 그리지 않는다.
    """
    x, y, z = wire_exit()
    start, _full, end = clip_span()
    r = WIRE_D / 2
    wire = cyl_x(r, end, x, y, z)
    for s in (1, -1):
        ya, yb = sorted((y + s * (r - CLIP_PINCH), y + s * (r + 1.0)))
        wire = solid(wire - box(end - 1.0, start - CONN_BACK, ya, yb, z - r - 1.0, z + r + 1.0))
    return solid(wire.moved(FRAME))


def profile() -> list[tuple[float, float]]:
    """받침의 꺾인 단면 (y, z). 발의 안쪽 뒤 모서리에서 시작해 카메라 뒷면을 따라 바깥 끝까지 갔다가 돌아온다:
    발 안쪽 뒤, 판 앞면이 돋움 면과 만나는 곳, 판 앞면의 바깥 끝, 판 뒷면의 바깥 끝, 메운 모서리 둘, 발 바깥 뒤."""
    ay, az = AXIS[1], AXIS[2]
    oy, oz = -INWARD[1], -INWARD[2]  # 판을 따라 바깥으로 가는 방향 (y, z)
    by, bz = _back_edge()
    z_in, z_out = -wrist.MOUNT_Z, -(wrist.MOUNT_Z + FOOT_T)
    s_in = (z_in - bz) / (-oz)  # 카메라 뒷면이 돋움 면과 만나는 곳까지 (안쪽으로)
    front_root = (by - s_in * oy, z_in)
    front_end = (by + CAM_H * oy, bz + CAM_H * oz)
    back_end = (front_end[0] - PLATE_T * ay, front_end[1] - PLATE_T * az)
    qy, qz = by - PLATE_T * ay, bz - PLATE_T * az  # 판 뒷면 위의 한 점
    s_out = (z_out - qz) / (-oz)
    back_root = (qy - s_out * oy, z_out)
    return [
        (FOOT_Y0, z_in),
        front_root,
        front_end,
        back_end,
        (back_root[0] + 2 * FILLET * oy, back_root[1] + 2 * FILLET * oz),
        (back_root[0] - FILLET, z_out),
        (FOOT_Y0, z_out),
    ]


def mount() -> Compound:
    """D405 받침 (검정). 옆으로 눕혀 뽑는다: 꺾인 단면이 바닥에 그려져 각도가 정확하고, 층이 판의 길이
    방향으로 이어져 카메라 무게가 층을 떼는 쪽으로 걸리지 않는다. +x 쪽 옆면을 바닥에 두어, J5 쪽(-x)으로 나온
    클립이 맨 위에서 뽑힌다. 클립은 판에 뿌리가 있을 때만 붙인다 (clip_rooted. 잰 플러그 자리에서는 없다)."""
    z_in, z_out = -wrist.MOUNT_Z, -(wrist.MOUNT_Z + FOOT_T)
    m = j.prism_x(profile(), -MOUNT_W / 2, MOUNT_W / 2)
    for x, y in wrist.mount_xy():  # 발 나사
        m = solid(m - cyl_z(j.K_M3_FREE_H / 2, z_out - 1, z_in + 1, x, y))
        # 머리 자리: 카메라를 플러그만큼 밖으로 옮겨 판의 뿌리가 나사 쪽으로 왔다. 머리 둘레만 판과 메운 자리를 판다
        m = solid(m - cyl_z(HEAD_RELIEF_R, z_out - 2.5, z_out, x, y))
    r = j.K_M3_FREE_H / 2
    for sy in (1, -1):  # 카메라 나사와 머리 자리 (카메라 좌표: 원통에서 멀어지는 쪽으로 나란하다)
        hole = cyl_z(r, -CAM_D - PLATE_T - 1, -CAM_D + 1, 0.0, sy * CAM_HOLE_X)
        hole += cyl_z(3.1, -CAM_D - PLATE_T - 1, -CAM_D - PLATE_T + HEAD_REC, 0.0, sy * CAM_HOLE_X)
        m = solid(m - solid(hole).moved(FRAME))
    if clip_rooted():
        m = solid(m + clip())
    if not m.is_valid or len(m.solids()) != 1:
        raise RuntimeError("D405 받침 형상이 깨졌다")
    return m


def hardware() -> list[j.Part]:
    """카메라 나사 M3 × 6 둘, 발 나사 M3 × 8 둘과 돋움 속 너트."""
    out: list[j.Part] = []
    seat = -CAM_D - PLATE_T + HEAD_REC  # 카메라 좌표에서 머리 밑면
    for k, sy in enumerate((1, -1), start=1):
        s = cyl_z(1.5, seat, seat + CAM_SCREW, 0.0, sy * CAM_HOLE_X)
        s += cyl_z(2.85, seat - 1.65, seat, 0.0, sy * CAM_HOLE_X)
        out.append(
            j.Part(f"카메라 나사 M3x6 {k}", solid(solid(s).moved(FRAME)), j.STEEL, False, "metal")
        )
    z_out = -(wrist.MOUNT_Z + FOOT_T)
    roof = wrist.MOUNT_NUT[2]
    for k, (x, y) in enumerate(wrist.mount_xy(), start=1):
        s = cyl_z(1.5, z_out, z_out + FOOT_SCREW, x, y) + cyl_z(2.85, z_out - 1.65, z_out, x, y)
        out.append(j.Part(f"받침 나사 M3x8 {k}", solid(s), j.STEEL, False, "metal"))
        z0 = -(wrist.MOUNT_Z - roof)
        nut = j.hex_z(x, y, z0, z0 + j.NUT_T, 5.4) - cyl_z(1.5, z0 - 1, z0 + j.NUT_T + 1, x, y)
        out.append(j.Part(f"너트 (돋움) {k}", solid(nut), j.STEEL, False, "metal"))
    return out


def assembly(model: ServoModel = "STS3215") -> list[j.Part]:
    """J6 조립(parts/flange.py)에 D405와 받침, 얇은 USB 2 선의 ㄱ자 플러그와 클립까지의 선을 더한 것."""
    parts = flange.assembly(model)
    parts += [
        j.Part("D405 받침", mount(), color="cap"),
        j.Part("D405", camera(), color="servo"),
        j.Part("D405 플러그", plug(), color="plug"),
        j.Part("D405 선", wire_stub(), color="plug"),
    ]
    return parts + hardware()


def plug_clearance(parts: list[j.Part]) -> dict[str, float]:
    """플러그와 클립까지의 선에서 손목 몸통, 벽 나사, 받침까지의 틈 [mm] (가까운 것만).

    받침과의 틈은 플러그에서만 잰다 (선은 받침의 클립이 물고 있다).
    """
    mine = [p for p in parts if p.name in ("D405 플러그", "D405 선")]
    out: dict[str, float] = {}
    for other in parts:
        if not other.name.startswith(("손목 몸통", "벽 나사", "D405 받침", "받침 나사", "뒷덮개")):
            continue
        key = other.name.rstrip("0123456789 ").strip()
        for p in mine:
            if other.name == "D405 받침" and p.name == "D405 선":
                continue
            if far_apart(p.shape, other.shape, 8.0):
                continue
            out[key] = min(out.get(key, 8.0), p.shape.distance_to(other.shape))
    return out


def clip_clearance(parts: list[j.Part]) -> dict[str, float]:
    """클립에서 둘레의 것(손목 몸통, 벽 나사, 뒷덮개, 받침 나사, 카메라, 플러그)까지의 틈 [mm] (10 안쪽만)."""
    shape = clip()
    out: dict[str, float] = {}
    for other in parts:
        if (
            not other.name.startswith(
                ("손목 몸통", "벽 나사", "뒷덮개", "받침 나사", "D405 플러그")
            )
            and other.name != "D405"
        ):
            continue
        if far_apart(shape, other.shape, 10.0):
            continue
        key = other.name if other.name == "D405" else other.name.rstrip("0123456789 ").strip()
        d = shape.distance_to(other.shape)
        if d < out.get(key, 10.0):
            out[key] = d
    return out


def clip_grip(e_gpa: float = 2.0) -> dict[str, float]:
    """클립이 선을 무는 정도 (어림). 갈래를 틈의 바닥에 박힌 외팔보로 본다. 선은 눌리지 않는다고 보아서, 실제 힘은
    이보다 작다 (선의 겉이 먼저 눌린다). 탄성 계수는 PLA의 낮은 쪽 값이 기본이다 (0029의 범위 2.0 ~ 2.75 GPa)."""
    start, full, end = clip_span()
    r = WIRE_D / 2
    e = e_gpa * 1000
    length = (
        full - end + CLIP_TIP
    )  # 갈래가 선의 옆에 닿는 길이 (45°로 솟는 구간에서 선의 가운데 높이부터)
    inertia = length * CLIP_T**3 / 12
    arm_w = CONN_BACK - CLIP_ROOT  # 틈의 바닥에서 선의 가운데까지
    arm_lip = arm_w + r * math.sin(math.radians(CLIP_LIP_DEG))  # 턱까지
    k_w = 3 * e * inertia / arm_w**3
    k_lip = 3 * e * (full - end) * CLIP_T**3 / 12 / arm_lip**3
    return {
        "갈래가 선의 옆에 닿는 길이 [mm]": length,
        "고무 끝에서 받침이 선의 밑에 닿는 곳까지 [mm]": (CAM_H / 2 - PLUG_TOP - PLUG_LEN)
        - (start - (CONN_BACK - r)),
        "고무 끝에서 클립 끝까지 [mm]": (CAM_H / 2 - PLUG_TOP - PLUG_LEN) - end,
        "갈래 하나가 선을 무는 힘 [N]": k_w * CLIP_PINCH,
        "끼울 때 갈래가 가장 많이 벌어지는 양 (한쪽) [mm]": CLIP_LIP,
        "그때 갈래 뿌리의 변형률 [%]": 150 * CLIP_T * CLIP_LIP / arm_lip**2,
        "그때 갈래 하나를 미는 힘 [N]": k_lip * CLIP_LIP,
    }


def screw_checks() -> dict[str, float]:
    """나사가 물린 길이와 여유 [mm]."""
    roof = wrist.MOUNT_NUT[2]
    foot_tip = FOOT_SCREW - FOOT_T  # 돋움 면에서 나사 끝까지
    return {
        "카메라 나사가 카메라에 들어간 길이": CAM_SCREW - (PLATE_T - HEAD_REC),
        "카메라 나사 끝과 구멍 바닥 사이": CAM_HOLE_DEPTH - (CAM_SCREW - (PLATE_T - HEAD_REC)),
        "받침 나사가 너트(2.4)에 물린 길이": min(foot_tip - roof, j.NUT_T),
        "받침 나사 끝과 나사 구멍 바닥 사이": wrist.MOUNT_HOLE - foot_tip,
        "돋움 나사 구멍 바닥과 원통 옆면 사이": math.hypot(
            wrist.MOUNT_X, wrist.MOUNT_Z - wrist.MOUNT_HOLE
        )
        - j.R_DRUM,
    }


CAM_LOAD_N = (
    0.65  # 카메라 58 ~ 60 g에 플러그와 선의 몫 6 g을 더한 무게 [N] (얇은 선 전체가 31.3 g이다)
)
PIXEL_DEG = 87.0 / 1280  # 가장 촘촘한 영상(가로 1280 화소)의 한 화소


def tilt_under_weight(e_gpa: float = 2.0) -> float:
    """카메라 무게가 판에 직각으로 걸릴 때 카메라가 기우는 각 [°]. 판을 발에서 카메라 나사까지 뻗은 외팔보로 본다.

    탄성 계수는 PLA의 낮은 쪽 값(2.0 GPa)을 기본으로 쓴다 (0029의 허브 판 계산과 같은 범위 2.0 ~ 2.75).
    """
    bz = _back_edge()[1]
    root = (-wrist.MOUNT_Z - bz) / math.cos(_T)  # 카메라 안쪽 모서리에서 발까지 (판을 따라)
    lever = CAM_H / 2 + root
    inertia = MOUNT_W * PLATE_T**3 / 12
    return math.degrees(CAM_LOAD_N * lever**2 / (2 * e_gpa * 1000 * inertia))


def sweep_gaps(parts: list[j.Part]) -> dict[str, float]:
    """J6를 멈춤 각까지 돌릴 때, 도는 부품과 D405·받침·플러그 사이의 가장 좁은 틈 [mm]."""
    fixed = [p for p in parts if p.name in ("D405", "D405 받침", "D405 플러그", "D405 선")]
    moving = [p for p in parts if p.moving]
    out: dict[str, float] = {}
    for q in range(-165, 166, 15):
        for a in moving:
            shape = j.turned(a.shape, float(max(-j.STOP_DEG, min(j.STOP_DEG, q))))
            for b in fixed:
                key = f"{a.name.rstrip('0123456789 ').strip()} ↔ {b.name}"
                if far_apart(shape, b.shape, out.get(key, 6.0)):
                    continue
                d = shape.distance_to(b.shape)
                if d < out.get(key, 6.0):
                    out[key] = d
    return out


# 뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋)
LABELS = {"d405_mount": "D405 받침"}  # 출력 안내의 한글 이름


def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    # +x 쪽 옆면이 바닥이다: 클립이 J5 쪽(-x)으로 받침 밖까지 나와 있어 맨 위에서 뽑힌다
    return {"d405_mount": (mount(), (-1.0, 0.0, 0.0), "black", "strength")}


def report(model: ServoModel = "STS3215") -> None:
    parts = assembly(model)
    by, bz = _back_edge()
    print(
        f"D405 받침 ({model}): 렌즈 면 가운데 y {LENS[1]:.1f} (플랜지 면보다 {LENS_BACK} 뒤), 축에서 {-LENS[2]:.1f}, "
        f"기울기 {TILT:.0f}°, 카메라 뒷면 안쪽 모서리 (y {by:.1f}, z {bz:.1f})"
    )
    cam = next(p for p in parts if p.name == "D405")
    bb = cam.shape.bounding_box()
    print(
        f"카메라가 차지하는 범위: y {bb.min.Y:.1f} ~ {bb.max.Y:.1f}, 축에서 {-bb.max.Z:.1f} ~ {-bb.min.Z:.1f} mm"
    )
    print(f"임시 값: {', '.join(PROVISIONAL)}")
    print("\n뽑는 부품")
    for name, (shape, up, color, preset) in printed().items():
        size = shape.bounding_box().size
        over = overhangs(shape, up)
        print(
            f"  {name:11s} {color:5s} {size.X:5.1f} × {size.Y:5.1f} × {size.Z:5.1f} mm  "
            f"{printed_mass_g(shape, up, preset):5.1f} g  걸침 {over.total:5.0f} mm² (거의 수평 {over.flat:.0f})"
        )
    print("\n".join(air_wall_lines(printed())))
    mid = {n: mid_layer_levels(s, up) for n, (s, up, _c, _p) in printed().items()}
    print(
        "층 가운데에 걸친 수평면 [mm]: "
        + (", ".join(f"{n} {v}" for n, v in mid.items() if v) or "없음")
    )
    start, full, end = clip_span()
    clip_g = printed_mass_g(clip(), (-1.0, 0.0, 0.0))
    if not clip_rooted():
        print(
            f"클립은 받침에 없다: 고무 끝이 카메라의 세로로 {CAM_H / 2 - PLUG_TOP - PLUG_LEN:.2f}, 받침의 J5 쪽 면이 "
            f"{-MOUNT_W / 2:.1f}이라 클립이 판 밖에 뜬다. 아래 클립 숫자는 플러그를 따라 그렸을 때의 값이다"
        )
    print(
        f"클립: 카메라의 세로로 {start:.2f}에서 솟기 시작해 {full:.2f}부터 {end:.2f}까지 온 단면 "
        f"(받침의 J5 쪽 면 {-MOUNT_W / 2:.1f}, 카메라의 바닥면 {-CAM_H / 2:.1f}), 무게 약 {clip_g:.2f} g"
    )
    for name, value in clip_grip().items():
        print(f"  {name}: {value:.2f}")
    for name, value in screw_checks().items():
        print(f"  {name}: {value:.2f}")
    print(
        f"  카메라 무게에 기우는 각: {tilt_under_weight(2.75):.3f} ~ {tilt_under_weight(2.0):.3f}° "
        f"(화소 반 칸 {PIXEL_DEG / 2:.3f}°)"
    )
    near = plug_clearance(parts)
    print(
        "  플러그와 선에서 가까운 것까지: "
        + ", ".join(f"{k} {v:.2f}" for k, v in sorted(near.items(), key=lambda kv: kv[1]))
    )
    around = clip_clearance(parts)
    print(
        "  클립에서 가까운 것까지: "
        + ", ".join(f"{k} {v:.2f}" for k, v in sorted(around.items(), key=lambda kv: kv[1]))
    )
    solids = [p for p in parts if "핀" not in p.name and "손잡이" not in p.name]
    hits = j.overlaps(solids)
    print(
        "\n겹침 [mm³]: " + (", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in hits.items()) or "없음")
    )
    body = next(p for p in parts if p.name == "손목 몸통")
    print(f"카메라와 손목 몸통 사이 {cam.shape.distance_to(body.shape):.2f} mm")
    print("J6를 돌릴 때 도는 부품과의 가장 좁은 틈 [mm] (6 아래만)")
    for key, d in sorted(sweep_gaps(parts).items(), key=lambda kv: kv[1]):
        print(f"  {key}: {d:.2f}")


if __name__ == "__main__":
    report()
