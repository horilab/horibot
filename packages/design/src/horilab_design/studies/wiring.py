"""배선 검토 (0031): 선이 관절을 넘는 길이, 구간마다 쓸 서보 선, D405 선이 넘는 자리와 길이.

선은 부모의 살(멈춤 홈 가운데, 방위각 180°) 쪽에서 나와 출력 원판의 테에 감기며 자식으로 넘어간다 (0029).
범위 가운데에서 나가는 자리와 들어가는 자리가 같은 방위각에 오고, 관절이 돌면 그 사이의 선이 테에 감긴다.

- 관절마다: 나가는 자리와 들어가는 자리 사이에 필요한 선 길이(관절 각의 함수), 가장 길 때, 범위 가운데에서 남는 길이.
- 선마다: 몸통 속 길(부품 좌표의 꺾은선)과 관절을 넘는 길이를 더해, 가진 선(20 cm, 300 mm, 900 mm)으로 닿는지.
- 살 위 통로: 멈춤 기둥과 귀가 지나가지 않는 폭.
- D405 선(얇은 USB 2 선): 서보 선과 같은 자리(살 위, 두 몸통 사이 층)로 넘는다. 서보 선 위에 겹쳐 테에
  감기고, 몸통과 링크에서는 겉을 따라간다. 층에 드는 굵기인지, 감기는 반지름과 범위 가운데에서 남는 길이,
  멈춤 각에서 귀와의 틈, 카메라에서 베이스의 띠 밑 누르개까지의 길이를 본다 (0031).

좌표는 부품 코드와 같다 (parts/joint.py의 서보 좌표, mm). 몸통 속 길의 점은 그 부품의 치수에서 낸다.
서보 선의 폭과 두께, D405 선의 길이·지름·무게는 잰 값이다 (data/measured.toml).

    uv run python -m horilab_design.studies.wiring
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass

from horilab_design.parts import base as bp
from horilab_design.parts import camera_mount as cmt
from horilab_design.parts import flange as fp
from horilab_design.parts import gripper as gp
from horilab_design.parts import joint as j
from horilab_design.parts import links as lp
from horilab_design.parts import shoulder as sp
from horilab_design.parts import wrist as wp
from horilab_design.values import load_measured

Point = tuple[float, float, float]

# ---- 선 (가정과 실측) ----
PLUG = (9.58, 9.01, 4.75)  # 서보 선 플러그: 꽂는 방향 길이, 폭, 두께 (실측, docs/hardware.md)
# 가진 선의 길이는 커넥터 끝에서 끝까지다 (docs/hardware.md). 선이 플러그 뒤에서 나오는 곳 사이는 플러그 둘만큼 짧다
CABLES_MM = (200.0, 300.0, 900.0)
SPARE = 15.0  # 선마다 남겨 둘 길이: 조립 오차, 꺾이는 자리의 둥근 몫, 커넥터를 당기지 않을 여유
PLUG_X = -14.2  # 꽂은 플러그의 가운데 (서보 좌표 x)
PLUG_Z = 4.95  # 커넥터 둘의 가운데 |z|
PLUG_Y = j.PLUG_TOP  # 플러그 뒤 끝 (선이 나오는 곳)
BACK_Y = (j.WIRE_TOP + PLUG_Y) / 2  # 뒷면에 눕힌 선의 가운데
_MEASURED = load_measured()
_D405, _D405_OLD = _MEASURED.d405_usb2_cable, _MEASURED.d405_cable
assert _D405 is not None and _D405_OLD is not None
assert _D405.diameter_mm is not None and _D405.mass_g is not None
# D405 선: 얇은 USB 2 마이크로 USB 선 (잰 값: 지름 3.0, 플러그 끝에서 끝까지 1,550 mm, 선 전체 31.3 g)
D405_D = _D405.diameter_mm
D405_STOCK_MM = _D405.length_mm
# 1 m의 무게: 양쪽 플러그까지 길이로 고르게 편 값이라, 팔에 실리는 몫을 선 자체보다 무겁게 센다
D405_KG_M = _D405.mass_g / _D405.length_mm
D405_OLD_D = _D405_OLD.diameter_mm  # 쓰지 않기로 한 기본 선(USB 3, 1 m)의 지름 (잰 값, 5.12)
LAYER_GAP = 2.0  # 두 몸통 사이 층에서 선 위에 남길 틈 (0017의 움직이는 틈)
# D405 선이 굽는 반지름의 기준 (선 지름의 배수, docs/design.md의 "D405 케이블"): 테에 감기는 자리와, 감기는 쪽이
# 바뀌며 꺾이는 자리(살 위에서 테로 들어가는 곳, 자식 몸통에서 층으로 내려오는 곳)
WRAP_RATIO = 8.0
FLEX_RATIO = 4.0
# 링크의 납작한 면에서 D405 선을 잡는 자리: 관절 축에서 관 쪽으로 (살 위를 지나 테로 들어가는 선이 이 길이만큼
# 풀려 있어 완만하게 굽는다)
LINK_HOLD_R = 50.0
HEEL_RISE = 18.0  # 링크 발의 끝벽을 타고 올라가 잡히는 높이 (링크를 그릴 때 확정한다)
# 관절마다 D405 선에 더 두는 길이: 멈춤 각에서도 선이 팽팽해지지 않는다. 손목 1에서는 두 관절의 선이 같은 쪽으로
# 감길 때 목 앞면 위에서 되돌아 굽는데, 팽팽하면 그 반지름이 작아진다 (d405_wrist1_turn)
D405_SPARE = 8.0
# 무게: 20 cm 서보 선이 3.5 g이다 (실측). 플러그 하나를 0.45 g으로 어림하면 선은 1 m에 14.4 g이다
SERVO_WIRE_KG_M = (3.5 - 2 * 0.45) / (200.0 - 2 * PLUG[0])
PLUG_KG = 0.45e-3


def length(points: list[Point]) -> float:
    """꺾은선의 길이 [mm]."""
    return sum(math.dist(a, b) for a, b in itertools.pairwise(points))


def usable(cable_mm: float) -> float:
    """플러그 뒤에서 플러그 뒤까지 쓸 수 있는 선 길이."""
    return cable_mm - 2 * PLUG[0]


@dataclass(frozen=True)
class Crossing:
    """관절 하나를 넘는 선. 방위각은 부모의 서보 좌표다 (살이 180°). 들어가는 자리는 범위 가운데에서의 값이다."""

    name: str
    drum_r: float  # 선이 감기는 면의 반지름 (출력 원판의 테. J1은 베이스 몸통의 띠)
    exit_r: float  # 부모에서 나가는 자리
    exit_az: float
    entry_r: float  # 자식으로 들어가는 자리
    entry_az: float
    rise: float  # 두 자리의 축 방향 차이
    extra: float = 0.0  # 감기는 면 밖에서 늘 드는 길이 (J1: 띠에서 귀 위 누르개까지)
    stop_deg: float = j.STOP_DEG

    def length(self, q_deg: float, wire_d: float = j.WIRE_T) -> float:
        """관절 각 q에서 두 자리 사이에 필요한 선 길이. 선의 가운데가 감기는 면에서 반 두께만큼 뜬다."""
        r = self.drum_r + wire_d / 2
        re, ra = max(self.exit_r, r), max(self.entry_r, r)
        delta = abs(math.radians(self.entry_az + q_deg - self.exit_az))
        wrapped = delta - math.acos(r / re) - math.acos(r / ra)
        if wrapped > 0:
            plane = math.sqrt(re * re - r * r) + r * wrapped + math.sqrt(ra * ra - r * r)
        else:  # 감기지 않고 곧게 간다
            plane = math.sqrt(max(re * re + ra * ra - 2 * re * ra * math.cos(delta), 0.0))
        return math.hypot(plane, self.rise) + self.extra

    def longest(self, wire_d: float = j.WIRE_T) -> float:
        return max(self.length(s * self.stop_deg, wire_d) for s in (1, -1))

    def slack(self, q_deg: float = 0.0, wire_d: float = j.WIRE_T) -> float:
        """가장 길 때에 맞춘 선이 관절 각 q에서 남는 길이 (느슨한 고리)."""
        return self.longest(wire_d) - self.length(q_deg, wire_d)


def _az(x: float, z: float) -> tuple[float, float]:
    """(x, z) → (반지름, 방위각 [°])."""
    return math.hypot(x, z), math.degrees(math.atan2(z, x)) % 360.0


def crossings() -> dict[str, Crossing]:
    """관절마다 서보 선이 넘는 자리."""
    out: dict[str, Crossing] = {}
    win = sum(j.WIN_X) / 2  # 링크의 선 창 가운데
    notch_x = wp.X_FOOT + 1.2  # 손목 몸통의 가운데 홈
    rise = j.ATOP + 1.5 - j.FRONT  # 앞면에서 자식 발 바닥 조금 위까지
    slot_z = sum(lp.FOOT_SLOT_Z) / 2

    def foot_entry(foot: lp.Foot) -> tuple[float, float]:
        return _az(-(foot.hx - lp.WALL / 2), slot_z)  # 발 끝벽 밑의 구멍

    # J1: 몸통의 띠에 감긴다. 띠에서 J1 몸통 귀 위 누르개까지는 늘 드는 길이다 (돌림판과 귀를 넘어 올라간다)
    ear_r, _ = _az(*bp.EAR_CLAMP)
    up = (bp.ATOP + j.EAR_T) - (bp.FRONT - bp.BAND_H / 2)
    out["J1"] = Crossing(
        "J1", bp.R_CORE, bp.R_CORE, 180.0, bp.R_CORE, 180.0, 0.0, up + (bp.R_CORE - ear_r)
    )
    # J2: J1 몸통 왼쪽 면(어깨 앞면)의 바닥판 띠에 단 누르개에서 살 위로 올라온다
    r2, a2 = foot_entry(lp.SHOULDER_FOOT)
    sh_rise = sp.ATOP + 1.5 - sp.FRONT
    out["J2"] = Crossing("J2", sp.R_DRUM, STRIP_R, 180.0, r2, a2, sh_rise)
    r3, a3 = foot_entry(lp.STS_FOOT)
    out["J3"] = Crossing("J3", j.R_DRUM, -win, 180.0, r3, a3, rise)
    # J4: 손목 1은 반 바퀴 돌려 달아서 그 앞벽의 옆 홈이 살 쪽에 온다
    r4, a4 = _az(-(j.NECK_FRONT_IN + 1.2 - wp.AX_Y), wp.SIDE_LANE)
    out["J4"] = Crossing("J4", j.R_DRUM, -win, 180.0, r4, a4, rise)
    # J5: 손목 2는 뒷덮개 쪽 모서리(앞 관절 원판 가장자리와 덮개 사이)로 들어온다
    r5, a5 = _az(j.BACK_IN + 1.5 - wp.AX_Y, wp.SIDE_LANE)
    out["J5"] = Crossing("J5", j.R_DRUM, -notch_x, 180.0, r5, a5, rise)
    # J6: 그리퍼 몸통의 판을 튼 자리로 들어가 서보 뒷면 커넥터에 꽂힌다. 튼 자리는 폭이 21 mm라 선이 그 안에서
    # 짧은 쪽으로 온다. 가운데로 들어온다고 보면 어느 쪽으로 감겨도 실제보다 길게 센다
    r6, a6 = _az(-fp.ADAPTER_R, 0.0)
    out["J6"] = Crossing("J6", j.R_DRUM, -notch_x, 180.0, r6, a6, 2.0)
    return out


# J1 몸통 왼쪽 면에서 선을 잡는 누르개: 어깨 축에서의 거리 (바닥판 높이의 가운데. 그 밑은 속이 찬 바닥판이다)
STRIP_R = -(bp.NECK_X + bp.FLOOR_T / 2)


# ------------------------------------------------------------------------------------------------
# 몸통 속 길 (꺾은선)


def _plug(side: int) -> list[Point]:
    """커넥터에서 뒷면에 눕기까지 (side는 꽂는 커넥터의 z 쪽)."""
    return [(PLUG_X, PLUG_Y, side * PLUG_Z), (PLUG_X - 3.3, BACK_Y, side * PLUG_Z)]


def wrist_out() -> list[Point]:
    """손목 몸통: 커넥터 → 서보 먼 끝 밑(앞 관절 원판 위의 틈) → 앞벽 가운데 홈."""
    xg = (wp.X_FOOT - j.FAR) / 2  # 서보 먼 끝과 발 바닥 사이 틈의 가운데
    lane = wp.SIDE_LANE
    return [
        *_plug(1),
        (-33.5, BACK_Y, lane),
        (xg, j.L1 - 2.0, lane),
        (xg, j.L0, lane),
        (xg, 12.0, lane),  # 혼 볼트 머리를 지난 뒤 가운데로 온다
        (xg, j.NECK_FRONT_IN - 1.5, 0.0),
        (xg, j.FRONT, 0.0),
    ]


def wrist_in(front: bool) -> list[Point]:
    """손목 몸통으로 들어오는 선 → 커넥터. front면 앞벽 옆 홈(손목 1), 아니면 뒷덮개 쪽 모서리(손목 2)."""
    xg = (wp.X_FOOT - j.FAR) / 2
    lane = -wp.SIDE_LANE
    if not front:
        return [(wp.X_FOOT - 0.8, j.BACK_IN + 1.5, lane), (-35.5, BACK_Y, lane), *_plug(-1)[::-1]]
    return [
        (wp.X_FOOT - 0.8, j.NECK_FRONT_IN - 1.0, lane),
        (xg, j.NECK_FRONT_IN - 4.0, lane),
        (xg, j.L0, lane),
        (xg, j.L1 - 2.0, lane),
        (-33.5, BACK_Y, lane),
        *_plug(-1)[::-1],
    ]


def _far_gap() -> float:
    """링크 끝 집: 서보 먼 끝과 목 끝 벽 사이 틈의 가운데 (선이 뒷면에서 관 쪽으로 넘어가는 자리)."""
    return (-j.FAR + lp.NECK_END + lp.WALL) / 2


def link_out() -> list[Point]:
    """링크 끝 집: 커넥터 → 서보 먼 끝을 돌아 → 앞면의 선 창."""
    xf, win = _far_gap(), sum(j.WIN_X) / 2
    return [
        *_plug(1),
        (-33.5, BACK_Y, 3.0),
        (xf, j.L1 - 2.5, 3.0),
        (xf, j.LABEL + 4.0, 0.0),
        (win, j.NECK_FRONT_IN - 1.0, 0.0),
        (win, j.FRONT, 0.0),
    ]


def link_in(name: str) -> list[Point]:
    """링크: 발 끝벽 밑의 구멍 → 발 바닥의 긴 틈 → 관 속 → 끝 집의 서보 먼 끝을 돌아 → 커넥터."""
    span, tube_r, foot = lp.LINKS[name]
    xa, xf = -span, _far_gap()
    yc = j.FRONT - (tube_r - lp.EMBED)  # 관 가운데
    z = sum(lp.FOOT_SLOT_Z) / 2
    reach = lp.slot_reach(foot)
    return [
        (xa - foot.hx, j.FRONT - 1.3, z),
        (xa - reach - 2.0, j.FRONT - 1.3, z),  # 긴 틈을 따라 발 속으로
        (xa - reach + 6.0, j.FRONT - lp.WALL - 3.0, z),
        (xa + foot.hx, yc, 0.0),
        (lp.NECK_END - 4.0, yc, 0.0),
        (xf, j.LABEL + 2.0, -3.0),
        (xf, j.L1 - 2.5, -3.0),
        (-33.5, BACK_Y, -3.0),
        *_plug(-1)[::-1],
    ]


def core_run() -> list[Point]:
    """베이스 몸통: J1 서보 커넥터 → 몸통 뒤 아래의 선 구멍 → 몸통 밖을 따라 띠 밑의 누르개까지."""
    az = math.radians(bp.WIRE_AZ)
    y_hole = bp.Y_FLOOR + bp.WIRE_EXIT[1] / 2
    clamp = (
        bp.R_CORE * math.cos(az),
        bp.FRONT - bp.BAND_H - bp.CLAMP_DROP,
        bp.R_CORE * math.sin(az),
    )
    return [
        *_plug(1),
        (-j.FAR, BACK_Y, 0.0),
        (-bp.R_CORE - 1.0, y_hole, 0.0),
        (-bp.R_CORE - 1.0, y_hole + 12.0, 0.0),
        clamp,
    ]


# J1 몸통의 길 (어깨 좌표: x 위, y 왼쪽, z 뒤). 귀 위 누르개에서 뒤 왼쪽 모서리를 돌아 왼쪽 면의 누르개까지
def j1_strip() -> list[Point]:
    cx, cz = bp.EAR_CLAMP
    x = bp.NECK_X + j.EAR_T + 1.0  # 귀 윗면 바로 위
    return [
        (x, sp.J1_AXIS_Y - cz, -cx),
        (x, sp.FRONT + 1.0, sp.NECK_HW + 1.0),  # 뒤 왼쪽 모서리
        (-STRIP_R, sp.FRONT + 1.0, 0.0),  # 왼쪽 면의 누르개 (살의 아래)
    ]


def j1_into_shoulder() -> list[Point]:
    """왼쪽 면의 누르개 → 선 창 → 어깨 서보 앞의 빈자리 → 서보 옆 틈 → 옆면 커넥터 (뒤쪽 것)."""
    win = sum(sp.WIN_X) / 2
    side = (sp.HW + sp.NECK_IN) / 2  # 서보 옆면과 목 안쪽 벽 사이 틈의 가운데
    cx, cy = sum(sp.CONN_X) / 2, sum(sp.CONN_Y) / 2
    above_pad = sp.PAD_X[1] + 4.0
    return [
        (-STRIP_R, sp.FRONT + 1.0, 0.0),
        (win, sp.FRONT + 1.0, 0.0),
        (win, sp.NECK_FRONT_IN - 4.0, 0.0),
        (above_pad, sp.NECK_FRONT_IN - 8.0, 8.0),  # 턱 위로 올라온다
        (above_pad, sp.HT + 6.0, side),
        (above_pad, cy, side),
        (cx, cy, side),
        (cx, cy, sp.HW + sp.PLUG_OUT),
    ]


def gripper_tail() -> list[Point]:
    """그리퍼: 판을 튼 자리의 가장자리 → 서보 뒷면 커넥터 (도구 좌표를 J6의 서보 좌표로: 플랜지 면이 ATOP)."""
    y = j.ATOP
    return [
        (-fp.ADAPTER_R, y + 2.0, 0.0),
        (PLUG_X - 2.0, y + 2.0, PLUG_Z),
        (PLUG_X, y + gp.H_PLUG, PLUG_Z),
    ]


# ------------------------------------------------------------------------------------------------
# 선마다의 길이


@dataclass(frozen=True)
class Wire:
    name: str
    parts: tuple[tuple[str, float], ...]  # (구간, 길이)

    @property
    def need(self) -> float:
        return sum(v for _, v in self.parts)

    @property
    def cable(self) -> float:
        """이 길이에 SPARE를 더해도 닿는 가장 짧은 선."""
        for c in CABLES_MM:
            if usable(c) >= self.need + SPARE:
                return c
        raise ValueError(f"{self.name}: 가진 선으로 닿지 않는다 ({self.need:.0f} mm)")

    @property
    def left(self) -> float:
        return usable(self.cable) - self.need


def wires() -> list[Wire]:
    """서보 선 일곱 가닥 (0016의 두 줄: 단자 1은 J1 → J2, 단자 2는 J3 → J4 → J5 → J6 → 그리퍼)."""
    x = crossings()
    over = {k: c.longest() for k, c in x.items()}
    strip = length(j1_strip())
    return [
        Wire(
            "J1 → J2",
            (
                ("베이스 몸통", length(core_run())),
                ("J1 넘기", over["J1"]),
                ("J1 몸통 겉", strip),
                ("J1 몸통 속", length(j1_into_shoulder())),
            ),
        ),
        Wire(
            "띠 밑 누르개 → J3",
            (
                ("J1 넘기", over["J1"]),
                ("J1 몸통 겉", strip),
                ("J2 넘기", over["J2"]),
                ("위팔 속", length(link_in("upper_arm"))),
            ),
        ),
        Wire(
            "J3 → J4",
            (
                ("위팔 끝 집", length(link_out())),
                ("J3 넘기", over["J3"]),
                ("아래팔 속", length(link_in("forearm"))),
            ),
        ),
        Wire(
            "J4 → J5",
            (
                ("아래팔 끝 집", length(link_out())),
                ("J4 넘기", over["J4"]),
                ("손목 1 속", length(wrist_in(front=True))),
            ),
        ),
        Wire(
            "J5 → J6",
            (
                ("손목 1 속", length(wrist_out())),
                ("J5 넘기", over["J5"]),
                ("손목 2 속", length(wrist_in(front=False))),
            ),
        ),
        Wire(
            "J6 → 그리퍼",
            (
                ("손목 2 속", length(wrist_out())),
                ("J6 넘기", over["J6"]),
                ("그리퍼", length(gripper_tail())),
            ),
        ),
    ]


# ------------------------------------------------------------------------------------------------
# 살 위 통로와 D405 선


def corridor(
    drum_r: float, post_r: float, q_deg: float, ear_hw: float = j.EAR_HW
) -> tuple[float, float]:
    """관절 각 q에서 살 가운데 선(180°)과 (멈춤 기둥, 귀의 옆면) 사이 거리 [mm]. 귀는 테의 면에서 잰다.

    선은 이 폭 안에서 살을 넘는다. 기둥은 살 윗면부터 테 밑까지, 귀(반폭 ear_hw)는 테 높이를 지나간다.
    """
    d = math.radians(180.0 - abs(q_deg))
    post = post_r * math.sin(d) - j.BOSS_R
    ear = drum_r * math.sin(d) - ear_hw * math.cos(d)
    return post, ear


def exit_to_ear(z_exit: float, q_deg: float) -> float:
    """손목 몸통 앞벽의 홈(살 가운데에서 z_exit)에서 나와 팽팽히 감긴 선의 가운데와 멈춤 귀 끝 사이 [mm].

    귀가 홈과 같은 쪽으로 다가온 관절 각(q > 0)에서 잰다. 이때 선은 귀의 반대쪽으로 감겨 있다.
    """
    r = j.R_DRUM + j.WIRE_T / 2
    ex, ez = wp.X_FOOT + 1.2, z_exit
    r_e, az_e = math.hypot(ex, ez), math.atan2(ez, ex)
    az_t = az_e + math.acos(r / r_e)  # 선이 테에 닿는 곳
    tx, tz = r * math.cos(az_t), r * math.sin(az_t)
    q = math.radians(q_deg)
    cx, cz = j.R_POST * math.cos(q), j.R_POST * math.sin(q)  # 귀 끝의 둥근 자리 (반지름 EAR_HW)
    dx, dz = tx - ex, tz - ez
    t = max(0.0, min(1.0, ((cx - ex) * dx + (cz - ez) * dz) / (dx * dx + dz * dz)))
    return math.hypot(ex + t * dx - cx, ez + t * dz - cz) - j.EAR_HW


def layer_height() -> float:
    """부모의 앞면과 자식의 발 바닥 사이 층의 높이 (선이 감기는 테의 둘레)."""
    return j.ATOP - j.FRONT


def d405_max_diameter() -> float:
    """두 몸통 사이 층에 감길 수 있는 선의 지름 한도 (위에 LAYER_GAP을 남긴다)."""
    return layer_height() - LAYER_GAP


def loop_radius(chord: float, arc: float) -> float:
    """두 끝 사이(chord)에 길이 arc인 선이 둥근 호 하나로 놓일 때의 반지름. 호가 가장 완만한 모양이다."""
    if arc <= chord:
        return math.inf
    lo, hi = (
        arc / (2 * math.pi),
        1e6,
    )  # 2R·sin(arc / 2R) = chord. 반지름이 클수록 호가 펴져 현이 길다
    for _ in range(80):
        mid = (lo + hi) / 2
        if 2 * mid * math.sin(arc / (2 * mid)) < chord:
            lo = mid
        else:
            hi = mid
    return hi


# D405 선은 서보 선 위에 겹쳐 감긴다: 선의 가운데가 테에서 (서보 선 두께 + 반지름)만큼 뜬다.
# Crossing.length는 가운데가 wire_d / 2만큼 뜬다고 보므로, 그 값이 되는 두께를 넘긴다
D405_OVER = 2 * j.WIRE_T + D405_D


def d405_crossings() -> dict[str, Crossing]:
    """관절마다 D405 선이 넘는 자리. 방위각은 서보 선과 같다 (부모는 살 쪽, 자식은 멈춤 귀의 맞은편).

    - J5: 자식(손목 2)은 뒷덮개 가운데의 클립에서 층으로 내려온다 (parts/camera_mount.py의 wire_run). 부모(손목 1)는
      목 앞면이 짧아 선을 잡지 않는다: 선이 목 앞면 위에서 그대로 J4의 층으로 넘어간다. 길이는 목 앞면의
      가운데를 경계로 나눠 센다.
    - J4: 자식은 손목 1의 목 앞면(위와 같은 선), 부모는 아래팔의 납작한 면.
    - J3, J2: 자식은 링크 발의 끝벽을 타고 올라가 잡히고, 부모는 링크의 납작한 면(J2는 J1 몸통 왼쪽 면의 누르개).
    - J1: 서보 선과 같은 띠에 나란히 감긴다 (겹치지 않는다).
    """
    x = crossings()
    lift = j.WIRE_T + D405_D / 2  # 서보 선 위에 놓인 선의 가운데 높이
    neck_mid = (j.R_DRUM - wp.X_FOOT) / 2  # 손목 1 목 앞면의 가운데: J5 축에서
    clip_r = wp.AX_Y - cmt.LEAD_Y  # 손목 2 뒷덮개의 클립: J5 축에서
    clip_rise = (j.ATOP + cmt.RUN_X - wp.X_FOOT) - (j.FRONT + lift)
    top_r = j.FRONT - wp.AX_Y + lift  # 손목 1 목 앞면 위의 선: J4 축에서
    heel_u, heel_f = lp.SHOULDER_FOOT.hx + D405_D, lp.STS_FOOT.hx + D405_D
    return {
        "J1": x["J1"],
        "J2": Crossing("J2", sp.R_DRUM, STRIP_R, 180.0, heel_u, 180.0, HEEL_RISE),
        "J3": Crossing("J3", j.R_DRUM, LINK_HOLD_R, 180.0, heel_f, 180.0, HEEL_RISE),
        "J4": Crossing("J4", j.R_DRUM, LINK_HOLD_R, 180.0, top_r, 180.0, -wp.X_FOOT - neck_mid),
        "J5": Crossing("J5", j.R_DRUM, neck_mid, 180.0, clip_r, 180.0, clip_rise),
    }


def _d405_wire_d(name: str) -> float:
    """Crossing.length에 넘길 두께: J1은 띠에 바로 놓이고, 나머지는 서보 선 위에 겹친다."""
    return D405_D if name == "J1" else D405_OVER


def d405_wrap_radius(name: str) -> float:
    """D405 선의 가운데가 감기는 반지름 [mm]."""
    return d405_crossings()[name].drum_r + _d405_wire_d(name) / 2


def d405_lead_radius(name: str) -> float:
    """부모에서 살 위를 지나 테로 들어가는 선이 굽는 반지름 [mm].

    선이 잡힌 자리(축에서 exit_r)에서 축을 보고 오다가, 감기는 쪽으로 둥글게 꺾여 테에 닿는다. 그 호의 반지름은
    (exit_r² - r²) / 2r이다 (r은 감기는 반지름). 잡힌 자리가 멀수록 완만하다.
    """
    c, r = d405_crossings()[name], d405_wrap_radius(name)
    return (c.exit_r**2 - r**2) / (2 * r)


def d405_child_lead(name: str) -> float | None:
    """자식 몸통에서 잡힌 자리부터 층까지 내려오는 길이 [mm]: 선이 감기는 쪽으로 꺾이는 반지름이 이쯤이다.

    손목 1(J4의 자식)은 선을 잡지 않으므로 없다.
    """
    rise = d405_crossings()[name].rise
    return None if name in ("J1", "J4") else rise


def d405_wrist1_gap() -> float:
    """손목 1의 목 앞면 위에서, J5의 테에 감긴 선과 J4의 테에 감긴 선이 가장 가까운 두 접선 사이 [mm].

    두 관절의 선이 반대쪽으로 감기면 선이 이 사이를 비스듬히 건너고(완만하다), 같은 쪽으로 감기면 되돌아 굽는다.
    되돌아 굽는 반지름은 선이 팽팽할 때 이 거리의 절반쯤에서 시작하고, 남는 길이가 있으면 커진다.
    """
    r = d405_wrap_radius("J5")
    return (-wp.X_FOOT + layer_height() / 2) - r


def d405_ear_gap(drum_r: float, ear_hw: float, q_deg: float = j.STOP_DEG) -> float:
    """멈춤 각에서, 살 가운데를 넘는 D405 선의 옆과 멈춤 귀의 옆면 사이 [mm]. 감기는 반지름에서 잰다."""
    r = drum_r + j.WIRE_T + D405_D / 2
    d = math.radians(180.0 - abs(q_deg))
    return r * math.sin(d) - ear_hw * math.cos(d) - D405_D / 2


def d405_path() -> list[tuple[str, float]]:
    """D405 선이 카메라의 플러그에서 베이스의 띠 밑 누르개까지 가는 길 (관절은 가장 길 때)."""
    x = d405_crossings()
    over = {k: c.longest(_d405_wire_d(k)) + D405_SPARE for k, c in x.items()}
    span_u, _, foot_u = lp.LINKS["upper_arm"]
    span_f, _, foot_f = lp.LINKS["forearm"]
    return [
        ("손목 2 겉", length(cmt.wire_run())),
        ("J5 넘기", over["J5"]),
        ("J4 넘기", over["J4"]),
        ("아래팔 겉", span_f - LINK_HOLD_R + foot_f.hx + D405_D),
        ("J3 넘기", over["J3"]),
        ("위팔 겉", span_u - LINK_HOLD_R + foot_u.hx + D405_D),
        ("J2 넘기", over["J2"]),
        ("J1 몸통 겉", length(j1_strip())),
        ("J1 넘기", over["J1"]),
    ]


def wire_masses() -> dict[str, float]:
    """팔 전체 모델(studies/ur_arm.py)에 넣을 선의 무게 [kg]: 위팔과 아래팔은 (가운데, 끝 집), 손목 몸통 둘.

    서보 선은 고른 선의 길이 전체(남는 길이는 그 선이 지나는 관이나 몸통 속에 접어 둔다)와 플러그, D405 선은
    d405_path의 길이다 (1 m의 무게는 플러그까지 고르게 편 값). J6 → 그리퍼 선은 그리퍼의 무게에 들어 있다.
    J2보다 아래(J1 몸통, 베이스)의 선은 넣지 않는다.
    """
    by = {w.name: w for w in wires()}

    def mm(name: str, *keys: str) -> float:
        parts = dict(by[name].parts)
        return sum(parts[k] for k in keys)

    def servo(length_mm: float, plugs: int) -> float:
        return length_mm * 1e-3 * SERVO_WIRE_KG_M + plugs * PLUG_KG

    d = dict(d405_path())

    def cam(length_mm: float) -> float:
        return length_mm * 1e-3 * D405_KG_M

    to_j3, j34, j45, j56 = "띠 밑 누르개 → J3", "J3 → J4", "J4 → J5", "J5 → J6"
    return {
        "upper_mid": servo(mm(to_j3, "위팔 속") + mm(to_j3, "J2 넘기") / 2, 1)
        + cam(d["위팔 겉"] + (d["J2 넘기"] + d["J3 넘기"]) / 2),
        # J3 → J4 선의 남는 길이는 위팔 끝 집 쪽(J3 서보 먼 끝 뒤의 관 속)에 접어 둔다: 아래팔에 두는 것보다 어깨가 덜 든다
        "upper_end": servo(mm(j34, "위팔 끝 집") + by[j34].left, 1),
        "fore_mid": servo(mm(j34, "J3 넘기", "아래팔 속"), 1)
        + cam(d["아래팔 겉"] + (d["J3 넘기"] + d["J4 넘기"]) / 2),
        "fore_end": servo(mm(j45, "아래팔 끝 집"), 1),
        "wrist1": servo(mm(j45, "J4 넘기", "손목 1 속") + by[j45].left + mm(j56, "손목 1 속"), 2)
        + cam((d["J4 넘기"] + d["J5 넘기"]) / 2),
        "wrist2": servo(mm(j56, "J5 넘기", "손목 2 속") + by[j56].left, 1)
        + cam(d["손목 2 겉"] + d["J5 넘기"] / 2),
    }


def report() -> None:
    x = crossings()
    print(
        "## 관절을 넘는 서보 선 (가장 길 때는 멈춤 각 ±163°, 남는 길이는 그 길이에 맞춘 선이 느슨해지는 양)"
    )
    print(
        "| 관절 | 감기는 반지름 | 가장 길 때 | 범위 가운데 | 가운데에서 남는 길이 | 한쪽 끝 / 다른 쪽 끝 |"
    )
    print("|---|---|---|---|---|---|")
    for k, c in x.items():
        ends = " / ".join(f"{c.length(s * c.stop_deg):.0f}" for s in (1, -1))
        print(
            f"| {k} | {c.drum_r + j.WIRE_T / 2:.1f} | {c.longest():.0f} | {c.length(0.0):.0f} | "
            f"{c.slack(0.0):.0f} | {ends} |"
        )
    print("\n## 서보 선 (길이는 플러그 뒤에서 플러그 뒤까지, mm)")
    for w in wires():
        parts = " + ".join(f"{n} {v:.0f}" for n, v in w.parts)
        print(f"- {w.name}: {w.need:.0f} = {parts} → {w.cable:.0f} mm 선 (남는 길이 {w.left:.0f})")
    print(
        f"  (선마다 {SPARE:.0f} mm를 남기고 고른다. 쓸 수 있는 길이: "
        + ", ".join(f"{c:.0f} → {usable(c):.0f}" for c in CABLES_MM)
        + ")"
    )
    print("\n## 살 위 통로: 살 가운데 선에서 (멈춤 기둥, 귀)까지 [mm]")
    for name, r, post, ear in (
        ("STS 관절", j.R_DRUM, j.R_POST, j.EAR_HW),
        ("어깨", sp.R_DRUM, sp.R_POST, sp.EAR_HW),
    ):
        soft, hard = corridor(r, post, j.SOFT_DEG, ear), corridor(r, post, j.STOP_DEG, ear)
        print(
            f"- {name}: 프로그램 한계에서 {soft[0]:.1f}, {soft[1]:.1f} / 멈춤 각에서 {hard[0]:.1f}, {hard[1]:.1f} "
            f"(서보 선 반폭 {j.WIRE_W / 2:.2f})"
        )
    side = sum(wp.NOTCH) / 2
    print(
        "- 손목 몸통의 홈에서 나와 팽팽히 감긴 선의 가운데와 멈춤 귀 끝 사이 (프로그램 한계 / 멈춤 각): "
        f"옆 홈 {exit_to_ear(side, j.SOFT_DEG):.1f} / {exit_to_ear(side, j.STOP_DEG):.1f}, "
        f"가운데 홈 {exit_to_ear(0.0, j.SOFT_DEG):.1f} / {exit_to_ear(0.0, j.STOP_DEG):.1f}"
    )
    dx = d405_crossings()
    print(
        f"\n## D405 선 (얇은 USB 2 선, 지름 {D405_D} mm, {D405_STOCK_MM:.0f} mm, 1 m에 "
        f"{D405_KG_M * 1000:.1f} g): 서보 선 위에 겹쳐 테에 감긴다"
    )
    print(
        f"- 굵기: 두 몸통 사이 층 {layer_height():.1f} mm에 감길 수 있는 지름 한도 {d405_max_diameter():.1f} mm "
        f"(선 위에 {LAYER_GAP:.0f} mm를 남긴다). 쓰지 않기로 한 기본 선은 {D405_OLD_D} mm다"
    )
    print(
        "| 관절 | 감기는 반지름 (지름의 배수) | 부모 쪽에서 굽는 반지름 | 자식 쪽에서 굽는 반지름 | 선 길이 | "
        "가운데에서 남는 길이 | 프로그램 한계에서 남는 길이 |"
    )
    print("|---|---|---|---|---|---|---|")
    for k, c in dx.items():
        wd, r = _d405_wire_d(k), d405_wrap_radius(k)
        full = c.longest(wd) + D405_SPARE
        parent = "띠" if k == "J1" else "손목 1" if k == "J5" else f"{d405_lead_radius(k):.0f}"
        lead = d405_child_lead(k)
        child = "띠" if k == "J1" else "손목 1" if lead is None else f"{lead:.0f}"
        soft = full - max(c.length(sg * j.SOFT_DEG, wd) for sg in (1, -1))
        print(
            f"| {k} | {r:.1f} ({r / D405_D:.1f}) | {parent} | {child} | {full:.0f} | "
            f"{full - c.length(0.0, wd):.0f} | {soft:.0f} |"
        )
    print(
        f"  (기준: 감기는 반지름은 지름의 {WRAP_RATIO:.0f}배 {WRAP_RATIO * D405_D:.0f} mm, 꺾이는 자리는 "
        f"{FLEX_RATIO:.0f}배 {FLEX_RATIO * D405_D:.0f} mm 이상. 선 길이는 멈춤 각에서 {D405_SPARE:.0f} mm가 남는 값이다)"
    )
    print(
        f"- 손목 1: 선을 잡지 않는다. 목 앞면 위에서 J5 쪽 접선과 J4 쪽 접선 사이가 {d405_wrist1_gap():.1f} mm다 "
        "(같은 쪽으로 감길 때 선이 이 사이에서 되돌아 굽는다)"
    )
    print(
        f"- 멈춤 각에서 선의 옆과 멈춤 귀 사이: STS 관절 {d405_ear_gap(j.R_DRUM, j.EAR_HW):.1f}, "
        f"어깨 {d405_ear_gap(sp.R_DRUM, sp.EAR_HW):.1f} mm"
    )
    path = d405_path()
    total = sum(v for _, v in path)
    print(
        f"- 길이: 카메라의 플러그에서 띠 밑 누르개까지 {total:.0f} mm, 그 뒤로 {D405_STOCK_MM - total:.0f} mm가 남는다 "
        "(플러그 끝에서 끝까지 잰 길이에서 뺀 값)"
    )
    print("  " + ", ".join(f"{n} {v:.0f}" for n, v in path))
    masses = wire_masses()
    print()
    print(
        f"## 팔에 실리는 선의 무게 (J2 위쪽): {sum(masses.values()) * 1000:.0f} g = "
        + ", ".join(f"{k} {v * 1000:.1f}" for k, v in masses.items())
    )


if __name__ == "__main__":
    report()
