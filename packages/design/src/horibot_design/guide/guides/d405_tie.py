"""D405 선 묶기: 카메라 밑면에 테이프, 아래팔과 위팔에 찍찍이 타이 넷, 왼쪽 파이 판에 하나, 남는 선 뭉치 (0031의 D405 선).

정한 것 (2026-11-28):
- 카메라 쪽에서 선을 잡는 방법: 절연 테이프로 카메라 밑면(1/4 나사 구멍이 있는 면, J5 쪽)에 붙인다. 찍찍이는 감을 자리가
  없다: 카메라와 받침을 옆으로 감으면 받침의 발(손목 2의 돋움에 붙어 있다)에 막히고, 위아래로 감으면 렌즈 창을 덮는다.
  받침의 클립은 잰 플러그 자리에서 판 밖에 떠서 뽑지 않았다 (parts/camera_mount.py). 테이프는 공짜이고 되돌리기 쉽다.
  테이프의 바깥 끝은 J5 축에서 63 mm로 설계의 잡는 자리(62 mm, 카메라 밑면 밑의 공중)와 반지름이 같고, 높이는 밑면이라
  손목 2의 발 바닥에서 15.2 mm다. 그래서 J4·J5 고리는 이 자리로 다시 셌다 (studies/wiring.py의 d405_wrist_loop).
- 묶는 자세: J2와 J3 영점 핀으로 팔을 곧게 세운다 (손목 잇기의 J4 영점과 같은 자세). 타이 자리에 손이 다 닿고, 카메라
  밑면이 위를 본다. 묶은 뒤 J1, J4, J5를 손으로 끝까지 돌려 본다. J2와 J3는 핀으로 잡혀 있어 처음 전원 날 서보로 본다.
- 고리 길이: 카메라 쪽에서 베이스 쪽으로 가며, 앞 자리에서 선을 따라 줄자로 잰 길이가 다음 타이의 가운데에 오게 묶는다.
  길이는 설계 값(관절이 범위 끝에 있을 때 당기지 않는 길이에 8 mm)을 5 mm 단위로 올린 값이다. 남는 선(약 37 cm)은 파이
  쪽 끝에서 감아 묶어 책상 뒤에 둔다.
- 고리와 남는 선 뭉치의 모양은 그림을 위한 어림이다 (선이 실제로 늘어지는 모양은 선의 뻣뻣함과 무게에 달렸다, 0031).

장면 좌표는 J1의 서보 좌표다 (arm_common). 둘째 관절은 쓰지 않는다.
"""

from __future__ import annotations

import functools
import itertools
import math
from collections.abc import Sequence
from dataclasses import replace

import numpy as np
from build123d import Compound

from horibot_design import placement as pl
from horibot_design.guide.guides import arm_common as ac
from horibot_design.guide.guides import upper_arm as ua
from horibot_design.guide.guides import wrist_join as wj
from horibot_design.guide.model import Guide, Item, Label, Move, ScenePart, Step, Use, Vec, View
from horibot_design.parts import base as bp
from horibot_design.parts import camera_mount as cmt
from horibot_design.parts import links as lp
from horibot_design.parts import pi_mount as pm
from horibot_design.parts import wrist as wp
from horibot_design.parts.shapes import box, cyl_z, rod, solid
from horibot_design.studies import wiring as w

HAVE = ua.HAVE
W = wj.W
REST_PREFIX = wj.REST_PREFIX
D = "D405/"  # 이 안내에서 다는 것 (곧게 선 자세)
DR = "D405 쉬는 자세/"  # 같은 것의 쉬는 자세 모습 (마지막 단계)
NO_AXIS2 = 99  # 둘째 관절을 쓰지 않는다: 어느 부품도 둘째 관절과 같이 돌지 않는다
R_CABLE = lp.CABLE_D / 2

# ---- 카메라 쪽: 선을 밑면에 눕혀 테이프로 붙인다 (카메라 좌표: x 세로(밑면 -x), y 가로(커넥터 쪽 -y), z 깊이) ----
X_ON = -cmt.CAM_H / 2 - R_CABLE  # 밑면에 붙은 선의 가운데
BEND_R = 7.0  # 고무 끝에서 밑면 쪽으로 눕히는 굽은 반지름 (손가락 굵기)
TAPE_Y = (-12.0, 8.0)  # 테이프가 덮는 자리 (가로). 바깥 끝은 J5 축에서 63 mm
TAPE_Z = (
    -cmt.CAM_D + 0.5,
    -cmt.CAM_D + 19.5,
)  # 테이프의 폭 (19 mm, 흔한 절연 테이프): 밑면 안에 든다
HOLD_H = X_ON - wp.X_FOOT  # 테이프 자리의 높이: 손목 2의 발 바닥에서 선 가운데까지


def _hold_servo() -> Vec:
    """테이프의 바깥 끝: 여기서 J4·J5 고리가 시작한다 (손목 2의 서보 좌표)."""
    return cmt.to_servo((X_ON, TAPE_Y[1], cmt.wire_exit()[2]))


def hold_radius() -> float:
    """테이프의 바깥 끝이 J5 축에서 떨어진 거리 [mm]."""
    _x, y, z = _hold_servo()
    return math.hypot(y - wp.AX_Y, z)


def _up5(v: float) -> int:
    return int(math.ceil(v / 5.0 - 1e-9) * 5)


@functools.cache
def loop_mm() -> dict[str, int]:
    """묶는 자리 사이의 선 길이 [mm]: 설계 값을 5 mm 단위로 올린 것 (J4·J5는 테이프 자리로 다시 센 값)."""
    return {
        "J4·J5": _up5(w.d405_wrist_loop(hold_radius(), HOLD_H).length),
        "J3": _up5(w.d405_elbow_loop().length),
        "J1·J2": _up5(w.d405_base_loop().length),
    }


# 찍찍이 도막의 길이 [cm]: 관을 한 바퀴 감은 둘레(납작한 면과 선을 품는다)에 겹침 3.5 cm쯤. 베이스와 뭉치는 넉넉히 8 cm
TIE_CM = {"forearm": 15, "upper_arm": 16, "base": 8, "coil": 8}

DONE = Item(
    "assembled_arm",
    "조립해 둔 팔",
    "bought",
    "베이스부터 손목 끝까지. D405 선은 카메라와 카메라 파이의 파란 단자에 꽂혀 있다",
)
CABLE = Item(
    "d405_cable", "D405 선", "bought", "카메라와 파란 단자에 꽂혀 있는 얇은 USB 2 선", icon="wire"
)
TIE = Item("velcro_tie", "찍찍이 타이", "bought", "가진 롤(폭 10 mm)에서 잘라 쓴다", icon="band")
OTHER: dict[str, Item] = {
    it.key: it
    for it in (
        DONE,
        CABLE,
        TIE,
        Item("tape_measure", "줄자", "tool", "30 cm 자도 된다", icon="ruler"),
        Item("scissors", "가위", "tool", "찍찍이와 테이프를 자른다", icon="knife"),
        Item("tape", "절연 테이프", "tool", "선을 카메라 밑면에 붙인다", icon="tape"),
        Item("longnose", "롱노즈", "tool", "핀을 곧게 당겨 뺀다", icon="tweezers"),
        Item("phone", "휴대폰", "tool", "사진을 찍어 대화창에 보낸다", icon="phone"),
    )
}

# ------------------------------------------------------------------------------------------------
# 선의 모양


def _cable(pts: Sequence[Vec]) -> Compound:
    """장면 좌표 꺾은선의 D405 선 (지름 3.0). 그림이라 막대를 합치지 않고 묶음으로 모은다 (굽은 고리를 합치면 겹친
    막대 끝에서 형상이 깨진다)."""
    rods = [
        rod(a, b, R_CABLE)
        for a, b in itertools.pairwise(pts)
        if np.linalg.norm(np.subtract(b, a)) >= 0.05
    ]
    if not rods:
        raise ValueError("선이 비었다")
    return Compound(children=rods)


def _bezier(p0: Vec, t0: Vec, p3: Vec, t3: Vec, k: float, bend: Vec, n: int) -> np.ndarray:
    a, b = np.array(p0), np.array(p3)
    c0 = a + k * (np.array(t0) + np.array(bend))
    c1 = b - k * (np.array(t3) - np.array(bend))
    s = np.linspace(0.0, 1.0, n)[:, None]
    return (1 - s) ** 3 * a + 3 * (1 - s) ** 2 * s * c0 + 3 * (1 - s) * s**2 * c1 + s**3 * b


def loop_points(
    p0: Vec, t0: Vec, p3: Vec, t3: Vec, length: float, bend: Vec, n: int = 28
) -> list[Vec]:
    """p0에서 t0 쪽으로 나가 p3에 t3 쪽으로 들어오는 고리 (3차 베지에). bend 쪽으로 불룩하고, 길이가 length다."""

    def size(k: float) -> float:
        pts = _bezier(p0, t0, p3, t3, k, bend, 200)
        return float(np.linalg.norm(np.diff(pts, axis=0), axis=1).sum())

    lo, hi = 0.0, 2000.0
    if size(lo) > length:
        raise ValueError(f"고리 {length:.0f} mm가 두 자리 사이보다 짧다")
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if size(mid) < length else (lo, mid)
    pts = _bezier(p0, t0, p3, t3, lo, bend, n)
    return [(round(float(x), 3), round(float(y), 3), round(float(z), 3)) for x, y, z in pts]


def _unit(v: Sequence[float] | np.ndarray) -> Vec:
    a = np.array(v, dtype=float)
    a /= np.linalg.norm(a)
    return (float(a[0]), float(a[1]), float(a[2]))


def _neg(v: Vec) -> Vec:
    return (-v[0], -v[1], -v[2])


def _turn_j1(p: Vec, deg: float) -> Vec:
    """장면의 첫째 관절(J1, 장면 y축)을 deg만큼 돌린 자리 (joint.turned와 같은 뜻: y축 둘레로 -deg)."""
    a = math.radians(-deg)
    x, y, z = p
    return (x * math.cos(a) + z * math.sin(a), y, -x * math.sin(a) + z * math.cos(a))


def _cam_servo() -> list[Vec]:
    """플러그 고무 끝에서 나온 선이 밑면 쪽으로 둥글게 눕혀져 테이프의 바깥 끝까지 가는 길 (손목 2의 서보 좌표)."""
    ex, ey, ez = cmt.wire_exit()
    arc = [
        (ex - BEND_R * math.sin(a), ey + BEND_R * (1.0 - math.cos(a)), ez)
        for a in (math.radians(d) for d in range(0, 91, 15))
    ]
    rise = (X_ON, TAPE_Y[0] - 1.0, ez)  # 밑면에 닿는 곳 (테이프 바로 앞)
    return [cmt.to_servo(p) for p in (*arc, rise, (X_ON, TAPE_Y[1], ez))]


def _tape_shape() -> Compound:
    """선 위에 붙인 테이프 (손목 2의 서보 좌표). 선을 덮는 얇은 판으로 그린다."""
    top = X_ON - R_CABLE
    shape = box(top - 0.3, top, TAPE_Y[0], TAPE_Y[1], TAPE_Z[0], TAPE_Z[1])
    return solid(shape.moved(cmt.FRAME))


def _out_dir(q: tuple[float, ...]) -> Vec:
    """카메라 밑면에서 손목에서 멀어지는 쪽 (장면 좌표): 테이프에서 선이 나가는 쪽."""
    return ac.vec("j6", (0.0, -cmt.INWARD[1], -cmt.INWARD[2]), q)


def _link_end(name: str, end: str, q: tuple[float, ...]) -> tuple[Vec, Vec, Vec]:
    """링크에 묶인 선의 끝(타이 바깥 모서리), 거기서 선이 관을 따라 나가는 쪽, 관 바깥쪽(선이 있는 옆) (장면 좌표).
    end는 "foot"이나 "house"."""
    frame = pl.LINK_FRAME[name]
    y, z = lp.cable_yz(name)
    x0, x1 = lp.tie_x(name)
    out = ac.vec(frame, (0.0, 0.0, lp.CABLE_SIDE[name]), q)
    if end == "foot":
        return ac.pt(frame, (x0 - lp.TIE_W / 2, y, z), q), ac.vec(frame, (-1.0, 0.0, 0.0), q), out
    return ac.pt(frame, (x1 + lp.TIE_W / 2, y, z), q), ac.vec(frame, (1.0, 0.0, 0.0), q), out


# ---- 베이스: 왼쪽(카메라) 파이 판의 몸통 쪽 긴 변의 타이 구멍, 파란 단자의 플러그 (장면 좌표 = parts/base.py) ----
# 선은 판 가장자리 바깥 옆면에 붙여 판 두께의 가운데 높이로 뒤(-x)로 보낸다: 판 위의 살(4 mm)에는 덮개 기둥(가장자리
# 쪽 6 mm)이 서 있어 선이 지나가지 못하고, 덮개 판이 가장자리 위를 덮는다. 찍찍이는 구멍에 꿰어 그 살과 선을 같이 감는다
_SLOT = pm.tie_slots()[1]  # 옆 -쪽 긴 변 = 왼쪽 판의 몸통 쪽 (wiring.d405_base_tie와 같은 구멍)


def base_tie() -> dict[str, float]:
    """타이를 감는 자리 (장면 좌표): x 범위, 감는 살의 z 범위(구멍의 몸통 쪽 벽 ~ 판 가장자리), 판의 위아래."""
    return {
        "x0": bp.PI_CX - lp.TIE_W / 2,
        "x1": bp.PI_CX + lp.TIE_W / 2,
        "z0": -(bp.PI_CZ + _SLOT[2]),
        "z1": -(bp.PI_CZ - bp.TRAY_HZ),
        "y0": pm.Y_TRAY,
        "y1": pm.Y_TOP,
    }


def _base_run() -> tuple[Vec, Vec]:
    """판 가장자리 옆에 붙은 선: (타이의 앞 끝, 판의 뒤 끝 = 책상 뒤 가장자리 조금 밖). 선은 뒤(-x)로 간다."""
    t = base_tie()
    y, z = (t["y0"] + t["y1"]) / 2, t["z1"] + R_CABLE
    return (t["x1"], y, z), (bp.PI_CX - bp.TRAY_HX - 4.0, y, z)


def _base_tie_shape() -> Compound:
    """구멍에 꿰어 판 가장자리의 살과 옆의 선을 같이 감은 찍찍이 (한 겹 0.93)."""
    t = base_tie()
    g = lp.TIE_T
    y0, y1, z0, z1 = t["y0"], t["y1"], t["z0"], t["z1"] + 2 * R_CABLE
    outer = box(t["x0"], t["x1"], y0 - g, y1 + g, z0 - g, z1 + g)
    inner = box(t["x0"] - 1, t["x1"] + 1, y0, y1, z0, z1)
    return solid(outer - inner)


# J1·J2 고리가 파이 판으로 내려가는 틈: 카메라 파이 덮개 판의 안쪽 가장자리(z -50.6)와 J1 몸통·어깨 목(축에서 47.5)
# 사이, 덮개 판보다 높은 곳. 선은 여기서 곧게 내려가 덮개 밑으로 들어가 판 가장자리 옆의 타이에 묶인다
GAP_AT: Vec = (8.0, 40.0, -49.0)
COIL_R = 28.0  # 남는 선 뭉치의 반지름 (지름 6 cm쯤)
COIL_TURNS = 2
COIL_AT = (
    -100.0,
    bp.Y_DESK - 45.0,
    -80.0,
)  # 뭉치의 가운데: 책상 뒤 가장자리에서 40 mm 뒤(클램프 뒤 끝 -69 밖), 책상 밑


@functools.cache
def _plug_box() -> tuple[Vec, Vec]:
    """파란 단자에 꽂힌 D405 선의 USB-A 플러그 (장면 좌표 상자)."""
    part = next(p for p in pl.group("pi") if p.name == f"파이/{PI_PLUG}")
    bb = part.shape.bounding_box()
    return (bb.min.X, bb.min.Y, bb.min.Z), (bb.max.X, bb.max.Y, bb.max.Z)


def _coil() -> list[Vec]:
    """책상 뒤에 감은 남는 선: 책상과 나란한 면(y-z)에 두 바퀴. 꼭대기에서 -z 쪽(플러그 쪽)으로 감기고, 감을수록 뒤로
    조금씩 물러난다."""
    cx, cy, cz = COIL_AT
    return [
        (
            cx - 3.5 * k / 24,
            cy + COIL_R * math.cos(2 * math.pi * k / 24),
            cz - COIL_R * math.sin(2 * math.pi * k / 24),
        )
        for k in range(24 * COIL_TURNS + 1)
    ]


def _gap(a: Vec, b: Vec) -> float:
    return float(np.linalg.norm(np.subtract(b, a)))


def _tail_points() -> list[Vec]:
    """베이스 타이 → 판 가장자리 옆을 따라 뒤로 → 책상 뒤에 감은 뭉치 → 파란 단자의 플러그 (장면 좌표)."""
    start, back = _base_run()
    clear = (
        COIL_AT[0] + 22.0,
        back[1],
        back[2],
    )  # 책상 높이 위로 클램프 뒤 끝(-69)을 지난 뒤 내려간다
    lo, hi = _plug_box()
    exit_ = (
        lo[0],
        (lo[1] + hi[1]) / 2,
        (lo[2] + hi[2]) / 2,
    )  # 플러그에서 선이 나오는 곳 (-x로 나온다)
    coil = _coil()
    down = (0.0, 0.0, -1.0)  # 뭉치 꼭대기에서 선이 가는 쪽
    lead_in = loop_points(
        clear, (-1.0, 0.0, 0.0), coil[0], down, _gap(clear, coil[0]) * 1.3, (-0.2, 0.0, 0.0), 12
    )
    lead_out = loop_points(
        coil[-1], down, exit_, (1.0, 0.0, 0.0), _gap(coil[-1], exit_) * 1.5, (-0.5, 0.3, 0.0), 14
    )
    return [start, back, *lead_in, *coil[1:], *lead_out[1:]]


def _coil_tie_shape() -> Compound:
    """뭉치 밑(반 바퀴 돈 자리, 들고 나는 선이 없는 곳)에서 두 바퀴를 같이 감은 찍찍이: 축이 z인 고리."""
    cx, cy, cz = COIL_AT
    px, py = cx - 3.5, cy - COIL_R  # 두 바퀴가 3.5 mm 떨어져 나란히 지나는 가운데
    r_in, g, h = 4.0, lp.TIE_T, lp.TIE_W / 2
    return solid(
        cyl_z(r_in + g, cz - h, cz + h, px, py) - cyl_z(r_in, cz - h - 1, cz + h + 1, px, py)
    )


# ------------------------------------------------------------------------------------------------
# 장면 부품

CAM_RUN, TAPE = f"{D}카메라 쪽 선", f"{D}테이프"
LOOP45, LOOP3, LOOP12 = f"{D}J4·J5 고리", f"{D}J3 고리", f"{D}J1·J2 고리"
B = "D405 베이스/"  # 베이스에 서 있는 것 (쉬는 자세에도 그대로 보인다)
BASE_TIE, TAIL, COIL_TIE = f"{B}파이 판 타이", f"{B}파이 쪽 선", f"{B}뭉치 타이"
PI_PLUG = "카메라 파이/D405 USB 플러그"
CAM_PLUG = f"{W}D405 플러그"
LINK_KO = {"forearm": "아래팔", "upper_arm": "위팔"}


def _link_names(prefix: str, name: str) -> tuple[str, str, str]:
    ko = LINK_KO[name]
    return (f"{prefix}{ko} 타이 (끝 집 쪽)", f"{prefix}{ko} 타이 (발 쪽)", f"{prefix}{ko} 묶인 선")


FORE_HOUSE, FORE_FOOT, FORE_RUN = _link_names(D, "forearm")
UPPER_HOUSE, UPPER_FOOT, UPPER_RUN = _link_names(D, "upper_arm")

# 고리마다 불룩하게 미는 쪽 (장면 좌표): 곧게 선 자세와 쉬는 자세. 그림을 위한 어림이다
BEND_UP = {"J4·J5": (-0.5, -0.8, -0.1), "J3": (-1.0, 0.0, 0.0), "J1·J2": (-0.3, 0.0, -0.9)}
BEND_REST = {"J4·J5": (0.0, -0.8, -0.5), "J3": (0.4, 0.8, 0.0), "J1·J2": (-0.4, 0.3, -0.8)}
OUT_TILT = 1.0  # 타이에서 고리로 나갈 때 관 바깥쪽으로 기우는 정도 (끝 집과 발을 비킨다)


def _tilted(along: Vec, out: Vec, k: float = OUT_TILT) -> Vec:
    return _unit(np.array(along) + k * np.array(out))


def _d405_pieces(
    q: tuple[float, ...], prefix: str, j1_deg: float, bends: dict[str, Vec]
) -> list[ac.Piece]:
    """자세 q의 D405 선: 카메라 쪽 선과 테이프, 링크의 타이와 묶인 선, 고리 셋. J1·J2 고리는 베이스에 서 있는 것으로
    그리므로, 위팔 쪽 끝을 장면의 첫째 관절(J1)을 j1_deg만큼 돌린 자리로 옮긴다."""
    lengths = loop_mm()
    out: list[ac.Piece] = []
    cam = [ac.pt("j6", p, q) for p in _cam_servo()]
    out.append(ac.Piece(f"{prefix}카메라 쪽 선", _cable(cam), CABLE.key, "wire", "j6", False))
    out.append(
        ac.Piece(f"{prefix}테이프", ac.put(_tape_shape(), "j6", q), "tape", "cap", "j6", False)
    )
    # 링크의 타이와 묶인 선 (placement의 묶음 "d405_run")
    for p in ac.group_pieces("d405_run", CABLE.key, q):
        rest = p.name.split("/", 1)[1]  # "아래팔 타이 (발 쪽)" 같은 이름
        is_tie = "타이" in rest
        out.append(
            replace(
                p,
                name=f"{prefix}{rest}",
                key=TIE.key if is_tie else CABLE.key,
                color="frame" if is_tie else "wire",
                count=is_tie and prefix == D,
            )
        )
    half = lp.TIE_W / 2
    fh, fh_d, f_out = _link_end("forearm", "house", q)
    ff, ff_d, _ = _link_end("forearm", "foot", q)
    uh, uh_d, u_out = _link_end("upper_arm", "house", q)
    uf, uf_d, _ = _link_end("upper_arm", "foot", q)
    # J4·J5: 아래팔 끝 집 쪽 타이에서 나가(끝 집을 비켜 바깥으로) 테이프의 바깥 끝으로 들어온다 (밑면을 따라 안쪽으로)
    into = _neg(_out_dir(q))
    t45 = _tilted(fh_d, f_out)
    p45 = loop_points(fh, t45, cam[-1], into, lengths["J4·J5"] - half, bends["J4·J5"])
    out.append(ac.Piece(f"{prefix}J4·J5 고리", _cable(p45), CABLE.key, "wire", "j4", False))
    # J3: 위팔 끝 집 쪽 타이에서 나가 팔꿈치 바깥을 돌아 아래팔 발 쪽 타이로 들어온다
    into_f = _neg(_tilted(ff_d, f_out))
    t3 = _tilted(uh_d, u_out)
    p3 = loop_points(uh, t3, ff, into_f, lengths["J3"] - 2 * half, bends["J3"])
    out.append(ac.Piece(f"{prefix}J3 고리", _cable(p3), CABLE.key, "wire", "j3", False))
    # J1·J2: 위팔 발 쪽 타이에서 내려와, 덮개 판 안쪽 가장자리와 J1 몸통 사이 틈으로 내려가 파이 판 가장자리의
    # 타이로 뒤를 보며 들어온다
    a = _turn_j1(uf, j1_deg)
    da = _turn_j1(_tilted(uf_d, u_out), j1_deg)
    start, _back = _base_run()
    down = (0.0, -1.0, 0.0)
    tuck = loop_points(
        GAP_AT, down, start, (-1.0, 0.0, 0.0), _gap(GAP_AT, start) * 1.15, (0.3, 0.0, 0.0)
    )
    rest = lengths["J1·J2"] - 2 * half - _length(tuck)
    p12 = loop_points(a, da, GAP_AT, down, rest, bends["J1·J2"])
    out.append(
        ac.Piece(f"{prefix}J1·J2 고리", _cable([*p12, *tuck[1:]]), CABLE.key, "wire", "j1", False)
    )
    return out


def _length(pts: Sequence[Vec]) -> float:
    return float(np.linalg.norm(np.diff(np.array(pts), axis=0), axis=1).sum())


def _base_pieces() -> list[ac.Piece]:
    lo, hi = _plug_box()
    return [
        ac.Piece(
            f"{HAVE}{PI_PLUG}",
            box(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]),
            DONE.key,
            "plug",
            "j1",
            False,
        ),
        ac.Piece(BASE_TIE, _base_tie_shape(), TIE.key, "frame", "j1"),
        ac.Piece(TAIL, _cable(_tail_points()), CABLE.key, "wire", "j1", False),
        ac.Piece(COIL_TIE, _coil_tie_shape(), TIE.key, "frame", "j1"),
    ]


def _arm_pieces() -> list[ac.Piece]:
    """곧게 선 팔 (조립해 둔 것): 위팔, 아래팔, 손목 끝과 D405 플러그, 손목 발 나사, J4 → J5 선. 영점 핀 둘은 센다."""
    keep = ("J5 선 (관절 밖)", "J5 선 (손목 1 속)", "J5 선 플러그 (J5)", wj.FEET)
    out = [replace(p, key=DONE.key, count=False) for p in (*wj._have_pieces(), *wj._wrist_pieces())]
    out.append(ac.Piece(CAM_PLUG, ac.put(cmt.plug(), "j6"), DONE.key, "plug", "j6", False))
    for p in wj.pieces():
        if p.name.startswith(keep):
            out.append(replace(p, name=f"{HAVE}{p.name}", key=DONE.key, count=False))
        elif p.name in (PIN2, PIN3):
            out.append(replace(p, count=True))
    return out


def _rest_pieces() -> list[ac.Piece]:
    """쉬는 자세의 팔 (wrist_join과 같은 것)과 D405 플러그."""
    out = [replace(p, key=DONE.key, count=False) for p in wj.rest_pieces()]
    plug = ac.put(cmt.plug(), "j6", ac.REST)
    out.append(ac.Piece(f"{REST_PREFIX}D405 플러그", plug, DONE.key, "plug", "j6", False))
    return out


def scene() -> list[ScenePart]:
    have = [replace(p, key=DONE.key, moving2=False, count=False) for p in ua._have()]
    pieces = (
        _arm_pieces()
        + _base_pieces()
        + _d405_pieces(ac.POSE, D, 0.0, BEND_UP)
        + _rest_pieces()
        + _d405_pieces(ac.REST, DR, ac.REST_J1, BEND_REST)
    )
    return have + ac.scene_of(pieces, NO_AXIS2)


# ------------------------------------------------------------------------------------------------
# 단계

PIN2, PIN3 = wj.PIN2, wj.PIN3
# 쉬는 자세 단계에서 숨길 것: 곧게 선 팔(위팔과 그 너머, J2와 같이 도는 어깨 부품)과 거기 묶은 D405 선, 영점 핀
UPRIGHT = (
    f"{HAVE}위팔",
    f"{HAVE}J3 ",
    f"{HAVE}J4 ",
    f"{HAVE}아래팔",
    f"{HAVE}J5 선",
    f"{HAVE}손목 발 나사",
    W,
    *wj.SHOULDER_TURNING,
    D,
    PIN2,
    PIN3,
)


def _mid_tie(name: str) -> Vec:
    x0, x1 = lp.tie_x(name)
    return ac.pt(pl.LINK_FRAME[name], ((x0 + x1) / 2, *lp.cable_yz(name)))


_CAM_AT = ac.pt("j6", cmt.to_servo((X_ON, sum(TAPE_Y) / 2, cmt.wire_exit()[2])))
_FORE_AT = _mid_tie("forearm")
_UPPER_AT = _mid_tie("upper_arm")
_ELBOW_AT = ac.pt("j3", (lp.NECK_END, *lp.cable_yz("upper_arm")))
BACK_LEFT = View((0, 1, 0), (-1.0, 0, -0.8), 12, 0)
CAM_VIEW = View((0, 1, 0), (-0.7, 0, -0.7), 55, 0)
BASE_VIEW = View(
    (0, 1, 0), (1.0, 0, -0.6), 45, 0
)  # 앞 왼쪽 위에서 J1 몸통과 카메라 파이 사이 틈을 내려다본다
CAM_PI_COVER = tuple(f"{HAVE}카메라 파이/{n}" for n in ("덮개 판", "팬", "철망"))
TAIL_VIEW = View((0, 1, 0), (-1.0, 0, -0.5), 25, 0)
L = loop_mm()

PREP = Step(
    "prep",
    "준비물",
    "준비물 꺼내 놓기",
    kind="prep",
    do=(
        "서보 전원이 꺼져 있는지 봅니다. 팔은 쉬는 자세로 선반에 얹혀 있고, D405 선은 카메라와 카메라 파이의 파란 단자에 "
        "꽂힌 채로 둡니다.",
        f"찍찍이 롤에서 여섯 도막을 잘라 둡니다: {TIE_CM['upper_arm']} cm 둘(위팔), {TIE_CM['forearm']} cm 둘(아래팔), "
        f"{TIE_CM['base']} cm 둘(파이 판과 남는 선 뭉치).",
        "절연 테이프 2 ~ 3 cm 한 도막, 줄자(30 cm 자도 됩니다), 영점 핀 둘(J2, J3).",
    ),
    check=("빨강 플러그가 빠져 있다.", "찍찍이 6도막, 테이프, 줄자, 핀 2"),
    stuck="찍찍이가 모자라면 대화창에 알려 주세요.",
    uses=(Use("scissors"),),
)

STEPS_WORK: tuple[Step, ...] = (
    Step(
        "upright",
        "팔 세우기",
        "팔을 J2·J3 핀으로 곧게 세우기 (전원 없이)",
        do=(
            "팔을 선반에서 들어 곧게 세우고, J2 핀(위팔 발의 귀)과 J3 핀(아래팔 발의 귀)을 꽂아 팔을 곧게 잡아 둡니다. J4 "
            "영점을 잴 때와 같은 자세입니다.",
            "손목은 그림처럼 둡니다: 카메라의 삼각대 구멍이 있는 면(밑면)이 위를 봅니다.",
            "D405 선은 아직 아무 데도 묶지 않고 늘어뜨려 둡니다.",
        ),
        warn=("핀을 꽂기 전에는 팔을 놓지 않습니다.",),
        check=("핀 둘이 꽂혀 팔이 혼자 서 있다.", "카메라의 삼각대 구멍 면이 위를 본다."),
        stuck=ac.PHOTO,
        adds=(HAVE, W, PIN2, PIN3),
        still=(HAVE, W),
        moves=(Move((PIN2,), (0.0, 0.0, -24.0)), Move((PIN3,), (0.0, 0.0, 24.0), 0.8)),
        view=BACK_LEFT,
        labels=(Label(PIN2, "J2 핀"), Label(PIN3, "J3 핀")),
    ),
    Step(
        "camera",
        "카메라 쪽",
        "선을 카메라 밑면에 테이프로 붙이기",
        do=(
            "플러그 고무 끝에서 나온 선을 카메라 밑면(위를 보는 면) 쪽으로 둥글게 눕혀, 밑면을 가로질러 손목에서 먼 쪽으로 "
            "보냅니다 (그림).",
            "고무 끝 바로 옆에서 접듯이 꺾지 말고, 손가락 굵기쯤 둥글게 돌립니다.",
            "테이프를 선 위로 길게 붙여 그림 자리(밑면 가운데에서 바깥쪽)에 선을 눌러 둡니다.",
        ),
        warn=(
            "플러그를 당기거나 비틀지 않습니다.",
            "테이프가 렌즈가 있는 앞면으로 넘어가지 않게 합니다.",
        ),
        check=("선이 밑면에 붙어 있다.", "고무 끝 옆에서 선이 접히지 않았다."),
        stuck=ac.PHOTO,
        adds=(CAM_RUN, TAPE),
        uses=(Use("tape", note="2 ~ 3 cm"),),
        view=CAM_VIEW,
        zoom=8.0,
        center=_CAM_AT,
        labels=(Label(TAPE, "테이프 (선 위로)"), Label(CAM_PLUG, "플러그")),
    ),
    Step(
        "fore-house",
        "아래팔 위",
        f"J4·J5 고리 {L['J4·J5']} mm 두고 아래팔 위쪽에 묶기",
        do=(
            f"테이프의 바깥 끝에서 선을 따라 줄자로 {L['J4·J5']} mm 되는 곳을 손가락으로 잡습니다.",
            f"그 곳을 아래팔 관의 뒤쪽 가장자리(로봇 뒤를 보는 쪽)에 대고, 끝 집(J4)으로 넓어지는 목에서 2 cm쯤 아래에 "
            f"{TIE_CM['forearm']} cm 찍찍이로 감습니다 (그림 ①).",
            "찍찍이는 선이 관에 붙어 있을 만큼만 조입니다. 겹친 끝은 선이 있는 쪽에 둡니다.",
        ),
        warn=("세게 조이지 않습니다. 선이 눌리면 상합니다.",),
        check=(
            f"테이프에서 이 타이까지 선이 {L['J4·J5']} mm쯤이다.",
            "타이가 목에서 2 cm쯤 아래에 있다.",
        ),
        stuck=ac.PHOTO,
        adds=(LOOP45, f"{FORE_HOUSE}$"),
        uses=(Use("tape_measure"),),
        view=BACK_LEFT,
        zoom=4.0,
        center=_FORE_AT,
        labels=(
            Label(f"{FORE_HOUSE}$", f"① {TIE_CM['forearm']} cm 찍찍이"),
            Label(LOOP45, f"J4·J5 고리 {L['J4·J5']} mm"),
        ),
    ),
    Step(
        "fore-foot",
        "아래팔 아래",
        "선을 아래팔 뒤쪽을 따라 내려 발 쪽에 묶기",
        do=(
            "선을 아래팔 관의 뒤쪽 가장자리를 따라 곧게 내립니다. 처지지 않을 만큼만 살짝 당깁니다.",
            f"발(J3)에서 관이 시작하는 곳에서 1.5 cm쯤 위에 {TIE_CM['forearm']} cm 찍찍이로 감습니다 (그림 ②).",
        ),
        check=(
            "두 타이 사이의 선이 관에 붙어 곧다.",
            "아래 타이가 J3 원판의 귀(멈춤 기둥)에 닿지 않는다.",
        ),
        stuck=ac.PHOTO,
        adds=(f"{FORE_RUN}$", f"{FORE_FOOT}$"),
        view=BACK_LEFT,
        zoom=4.0,
        center=_FORE_AT,
        labels=(Label(f"{FORE_FOOT}$", f"② {TIE_CM['forearm']} cm 찍찍이"),),
    ),
    Step(
        "upper-house",
        "위팔 위",
        f"J3 고리 {L['J3']} mm 두고 위팔 위쪽에 묶기",
        do=(
            f"아래팔 아래 타이에서 선을 따라 {L['J3']} mm 되는 곳을 잡습니다.",
            f"그 곳을 위팔 관의 뒤쪽 가장자리, 끝 집(J3)으로 넓어지는 목에서 2 cm쯤 아래에 {TIE_CM['upper_arm']} cm "
            "찍찍이로 감습니다 (그림 ③).",
            "고리는 팔꿈치 뒤로 둡니다. 팔꿈치 옆의 서보 선 고리와 엉키지 않게 합니다.",
        ),
        check=(
            f"아래팔 아래 타이에서 이 타이까지 선이 {L['J3']} mm쯤이다.",
            "고리가 팔꿈치 뒤에 있고 서보 선과 감기지 않았다.",
        ),
        stuck=ac.PHOTO,
        adds=(LOOP3, f"{UPPER_HOUSE}$"),
        uses=(Use("tape_measure"),),
        view=BACK_LEFT,
        zoom=3.2,
        center=_ELBOW_AT,
        labels=(
            Label(f"{UPPER_HOUSE}$", f"③ {TIE_CM['upper_arm']} cm 찍찍이"),
            Label(LOOP3, f"J3 고리 {L['J3']} mm"),
        ),
    ),
    Step(
        "upper-foot",
        "위팔 아래",
        "선을 위팔 뒤쪽을 따라 내려 발 쪽에 묶기",
        do=(
            "선을 위팔 관의 뒤쪽 가장자리를 따라 곧게 내립니다.",
            f"발(어깨 원판 위)에서 관이 시작하는 곳에서 1.5 cm쯤 위에 {TIE_CM['upper_arm']} cm 찍찍이로 감습니다 "
            "(그림 ④).",
        ),
        check=(
            "두 타이 사이의 선이 관에 붙어 곧다.",
            "아래 타이가 어깨 원판의 귀(멈춤 기둥)에 닿지 않는다.",
        ),
        stuck=ac.PHOTO,
        adds=(f"{UPPER_RUN}$", f"{UPPER_FOOT}$"),
        view=BACK_LEFT,
        zoom=3.2,
        center=_UPPER_AT,
        labels=(Label(f"{UPPER_FOOT}$", f"④ {TIE_CM['upper_arm']} cm 찍찍이"),),
    ),
    Step(
        "base",
        "파이 판",
        f"J1·J2 고리 {L['J1·J2']} mm 두고 파이 판 구멍에 묶기",
        do=(
            f"위팔 아래 타이에서 선을 따라 {L['J1·J2']} mm 되는 곳을 잡습니다.",
            "그 곳을 왼쪽 파이(카메라 파이) 판에서 베이스 몸통을 보는 가장자리 옆에 대고, 가장자리 가운데의 긴 구멍(덮개 "
            f"밑)에 {TIE_CM['base']} cm 찍찍이를 꿰어 판 가장자리와 선을 같이 감습니다 (그림 ⑤).",
            "선은 판 가장자리 옆을 따라 뒤쪽(책상 뒤)으로 보냅니다.",
        ),
        check=(
            f"위팔 아래 타이에서 이 타이까지 선이 {L['J1·J2']} mm쯤이다.",
            "고리가 파이 덮개 위에 얹히지 않는다.",
        ),
        stuck=ac.PHOTO,
        adds=(LOOP12, BASE_TIE),
        uses=(Use("tape_measure"),),
        shell=CAM_PI_COVER,
        ghost=True,
        view=BASE_VIEW,
        zoom=4.0,
        center=_base_run()[0],
        labels=(
            Label(BASE_TIE, f"⑤ {TIE_CM['base']} cm 찍찍이 (구멍에 꿰어)"),
            Label(LOOP12, f"J1·J2 고리 {L['J1·J2']} mm"),
        ),
    ),
    Step(
        "tail",
        "남는 선",
        "파이 쪽 남는 선 감아 묶기",
        do=(
            "파이 판 타이에서 선을 판 가장자리 옆을 따라 뒤로 보내, 책상 뒤 가장자리를 넘깁니다.",
            f"남는 선(약 37 cm)은 책상 뒤에서 둥글게 두 바퀴 감아(지름 6 cm쯤) {TIE_CM['coil']} cm 찍찍이로 묶고, 책상 "
            "뒤로 늘어뜨립니다.",
            "플러그에서 나온 선은 당기지 않게 느슨하게 둡니다.",
        ),
        check=("남는 선이 뭉치로 묶여 책상 뒤에 있다.", "플러그가 파란 단자에 끝까지 꽂혀 있다."),
        stuck=ac.PHOTO,
        adds=(TAIL, COIL_TIE),
        view=TAIL_VIEW,
        zoom=2.5,
        center=_base_run()[1],
        labels=(
            Label(COIL_TIE, f"{TIE_CM['coil']} cm 찍찍이 (뭉치)"),
            Label(f"{HAVE}{PI_PLUG}", "파란 단자의 플러그"),
        ),
    ),
    Step(
        "turn",
        "돌려 보기",
        "J1, J4, J5를 손으로 끝까지 돌려 D405 선 보기",
        do=(
            "J1: 위팔을 받쳐 팔 전체를 천천히 한쪽 끝까지, 다시 반대쪽 끝까지 돌립니다. 파이 판과 위팔 사이 고리가 파이 "
            "덮개나 어깨에 걸리거나 팽팽해지지 않는지 봅니다.",
            "J4: 손목을 양 끝까지 돌리며 카메라와 아래팔 사이 고리를 봅니다.",
            "J5: 카메라가 달린 손목 끝을 양 끝까지 돌리며, 선이 손목 2 발과 손목 1 사이 틈으로 끌려 들어가지 않는지 봅니다 "
            "(11-22에 선이 낀 자리).",
            "다 보면 손목과 J1을 그림 자리로 되돌립니다. J2와 J3는 처음 전원 날 서보로 천천히 돌리며 봅니다.",
        ),
        warn=(
            "걸리거나 팽팽하면 더 돌리지 말고 그 자세를 사진으로 보내 주세요.",
            "끝에서 멈춤 기둥이 닿으면 더 밀지 않습니다.",
        ),
        check=(
            "J1, J4, J5 양 끝에서 선이 걸리거나 팽팽하지 않았다.",
            "테이프와 타이가 밀리지 않았다.",
        ),
        stuck="걸린 자세를 사진으로 보내 주세요.",
        uses=(Use("phone", note="걸리면 사진", count=False),),
        view=BACK_LEFT,
        labels=(
            Label(f"{HAVE}J1 베이스/J1 몸통", "① J1: 팔 전체를 양 끝까지"),
            Label(f"{W}손목 1/손목 몸통", "② J4: 손목째 양 끝까지"),
            Label(f"{W}손목 2/손목 몸통", "③ J5: 카메라 쪽 손목 끝을 양 끝까지"),
        ),
    ),
)


def _rest(sid: str, kind: str) -> Step:
    work = kind == "work"
    return Step(
        sid,
        "쉬는 자세",
        "핀 빼고 쉬는 자세로 두기",
        kind=kind,
        do=(
            "팔을 두 손으로 받친 채 J3 핀, J2 핀을 차례로 곧게 당겨 뺍니다.",
            "J1 몸통을 위에서 볼 때 시계 방향으로 55°쯤 돌리고, 위팔을 숙이며 아래팔을 접어 아래팔 끝 집을 선반에 얹습니다. "
            "손목은 아래로 늘어지게 둡니다.",
            "접을 때 D405 고리가 팔꿈치 사이나 선반에 끼지 않는지 봅니다.",
            "다 된 모습을 사진으로 보내 주세요.",
        )
        if work
        else (
            "D405 선을 묶었습니다: 카메라 밑면 테이프, 아래팔과 위팔의 타이 넷, 파이 판 타이, 남는 선 뭉치.",
            "다음은 처음 전원입니다. J2와 J3도 서보로 천천히 돌리며 D405 고리와 서보 선(J3 고리 포함)을 봅니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요.",
        ),
        warn=("핀을 빼면 팔이 넘어갑니다. 두 손으로 받칩니다.",) if work else (),
        check=(
            "아래팔 끝이 선반에 얹혀 팔이 혼자 서 있다.",
            "D405 선이 끼거나 팽팽한 곳이 없다.",
        )
        if work
        else (),
        adds=(REST_PREFIX, DR) if work else (),
        still=(REST_PREFIX, DR) if work else (),
        uses=(Use("longnose"), Use("phone", note="다 된 모습", count=False)) if work else (),
        hide=UPRIGHT,
        joint=ac.REST_J1,
        view=wj.REST_VIEW,
    )


STEPS: tuple[Step, ...] = (PREP, *STEPS_WORK, _rest("rest", "work"), _rest("done", "end"))

GUIDE = Guide(
    id="d405-tie",
    kind="wiring",
    title="D405 선 묶기",
    sub=(
        f"카메라 밑면 테이프 · 찍찍이 타이 여섯 · 고리 {L['J4·J5']} · {L['J3']} · {L['J1·J2']} mm · "
        "J1, J4, J5 손으로 돌려 보기"
    ),
    scene=scene,
    steps=STEPS,
    items=OTHER,
)
