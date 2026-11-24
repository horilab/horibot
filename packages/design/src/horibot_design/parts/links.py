"""위팔·아래팔 링크: 앞 관절의 출력 원판에 앉는 발 + 납작한 관 + 다음 관절의 집. 한 부품이다 (0029).

좌표는 링크 끝의 집이 품는 서보의 좌표다 (parts/joint.py). 앞 관절 축은 x = -length에서 y와 나란하다.
링크의 납작한 면(뽑을 때 바닥)은 y = FRONT이고, 원통 앞 끝과 한 평면이다. 그래서 두 관절 축이 나란한 정도가
한 부품 안에서, 바닥에 닿아 뽑힌 면으로 정해진다. 앞 관절의 출력 원판은 이 평면 바깥(+y)에서 발을 받친다.
발은 앞 관절의 출력 원판에 맞춘다 (Foot). 아래팔의 발은 손목 몸통과 같고(나사 넷, 축에서 ±5, 폭 ±19.4),
위팔의 발은 어깨 출력 원판(지름 57.2, 혼 볼트 여덟, 나사 여섯)에 맞춰 넓다.

링크에 더한 것:
- 발의 선 자리 (0031): 끝벽 밑의 낮은 구멍과 발 바닥의 긴 틈(테에 감는 길), 끝벽의 높은 창(발의 겉으로 도는 길).
  두 길을 다 남긴다 (첫 출력에서 돌려 보고 고른다).
- 관 글자 (0019): 위팔의 납작한 면에 BOMI, 아래팔의 둥근 면에 HORI. 글자는 낱자마다 검정으로 따로 뽑아, 관에 낸 둥근
  구멍 둘에 핀으로 꽂는다 (letter, letter_holes). 팔을 곧게 세우면 어깨 → 팔꿈치 → 손목으로 읽히고 글자 윗변이 로봇
  뒤쪽이다.
- 아래팔 끝의 나사 자리 둘 (2026-11-22): D405 선을 잡거나 틈을 막는 부품을 나중에 덧달 자리다 (SEAT_*).
- D405 선을 묶는 찍찍이 타이의 자리 (0031): 링크에는 아무것도 내지 않는다. 타이와 묶인 선의 모양은 닿음과 가림을
  보려고 그린 것이다 (tie_band, cable_run. 타이의 폭과 두께는 가정이다).

    uv run python -m horibot_design.parts.links
"""

from __future__ import annotations

import functools
import math
from dataclasses import dataclass

import numpy as np
import shapely
from build123d import (
    Align,
    Compound,
    Cone,
    Face,
    Location,
    Plane,
    Pos,
    RegularPolygon,
    Rot,
    Text,
    Wire,
    extrude,
)
from shapely.geometry import MultiPoint, Point
from shapely.geometry import Polygon as Flat

from horibot_design.parts import joint as j
from horibot_design.parts import shoulder as sh
from horibot_design.parts.shapes import box, cyl_x, cyl_y, cyl_z, nut_trap, rod, solid
from horibot_design.printcheck import (
    air_wall_lines,
    mid_layer_levels,
    overhangs,
    printed_mass_g,
)
from horibot_design.values import REPO_ROOT

Vec = tuple[float, float, float]

WALL = 2.6  # 0.20mm Strength의 벽 6겹
EMBED = 6.0  # 관을 납작하게 깎는 깊이: 바닥에서 45° 안으로 선다
NECK_END = -40.0  # 집의 목이 관으로 이어지는 x
BED = 256.0  # 출력 범위 [mm]
UP: Vec = (0.0, -1.0, 0.0)  # 링크를 뽑을 때 위: 납작한 면이 바닥이다
# 발의 선 통로 (0031): 발 끝벽 밑의 구멍과, 거기서 혼 볼트 머리 위 구멍까지 발 바닥을 뚫은 긴 틈이다.
# 플러그는 혼 볼트 머리 위 구멍으로 넣고, 선은 링크를 원판에 얹기 전에 이 틈으로 밀어 끝벽 밑으로 뺀다
# (플러그를 좁은 구멍에 꿰지 않는다). 영점 핀 귀(|z| 4.4까지)를 비켜 한쪽에 둔다. 폭 5.7은 선 폭 4.75에 여유 0.95다
FOOT_SLOT_Z = (7.0, 12.7)
FOOT_SLOT_H = 2.6  # 끝벽 밑 구멍이 폭 전체로 열린 높이 (선 두께 1.5. 그 위는 세로 틈으로 좁아진다)
FOOT_SLOT_OPEN = (
    2.5  # 긴 틈이 혼 볼트 머리 위 구멍 속으로 들어온 길이: 선(두께 1.5)을 세워서 옆으로 민다
)
# 끝벽의 높은 창 (0031의 J2·J3: 남는 선을 발의 겉으로 돌리는 길, studies/wiring.py의 over_link). 손목 2 뒷덮개의
# 창처럼 선이 발 바닥 위 13.0 ~ 14.5에서 축에서 멀어지는 쪽으로 나온다: 영점 핀 귀(6.2)와 그 위로 나온 핀(9.2)보다
# 높다. 높이 범위는 뽑을 때의 층 경계다. 낮은 구멍과는 세로 틈으로 이어져서, 낮은 구멍으로 뺀 선을 세워 밀어 올린 뒤
# 창에 눕힌다 (플러그를 꿰지 않는다. 눕힌 선은 폭이 틈보다 넓어 되내려가지 않는다). 낮은 구멍에 그대로 두면 테에
# 감는 길이다
FOOT_WINDOW = (12.2, 15.2)
FOOT_SLIT_W = 2.0  # 세로 틈의 폭 (선 두께 1.5)


@dataclass(frozen=True)
class Foot:
    """앞 관절의 출력 원판에 앉는 발의 치수."""

    disc_r: float  # 앞 관절 출력 원판의 반지름 (귀의 바깥)
    hx: float  # 발 상자의 반 길이 (관 방향). 원판 둘레에 감긴 선이 끝벽의 홈으로 들어오게 원판보다 3.2 길다
    hw: float  # 발 상자의 바깥 반폭
    rim: tuple[tuple[float, float], ...]  # 몸통 나사 (축에서 관 방향, 폭 방향)
    head_r: float  # 혼 볼트 머리 위 구멍의 반지름
    pin_r: float  # 영점 핀
    chamfer: float = (
        8.0  # 발 상자 윗모서리(뽑을 때 위)를 45°로 깎는 깊이. 속은 1.5 덜 깎아 벽 두께를 지킨다
    )


STS_FOOT = Foot(j.R_DRUM, j.R_DRUM + 3.2, j.NECK_HW, j.RIM, 11.0, j.R_PIN)
# 어깨: 혼 볼트 머리가 반지름 14.85까지 온다. 발 상자 안쪽 반폭(15.6)이 그 구멍(15.4)보다 넓어야 옆벽이 남는다
# 윗모서리를 12 깎으면 발 상자가 관(반지름 20)의 둥근 윤곽 안에 거의 든다 (접은 자세에서 손목과 덜 닿는다)
SHOULDER_FOOT = Foot(sh.R_DRUM, sh.R_DRUM + 3.2, 18.2, sh.RIM, 15.4, sh.R_PIN, chamfer=12.0)

# 링크마다 (두 관절 축 사이 길이, 관 반지름, 발)
LINKS: dict[str, tuple[float, float, Foot]] = {
    "upper_arm": (220.0, 20.0, SHOULDER_FOOT),
    "forearm": (180.0, 18.0, STS_FOOT),
}

# ---- 관 글자 (0019) ----
FONT = REPO_ROOT / "data" / "fonts" / "montserrat" / "Montserrat-ExtraBold.ttf"
FONT_SIZE = 17.0  # 대문자 높이 11.9 mm (둥근 글자는 위아래로 0.24씩 더 나와 12.4)
# 링크마다 (글자, 다는 면): "flat"은 납작한 면(뽑을 때 바닥), "round"는 그 맞은편의 둥근 면(뽑을 때 맨 위).
# 로봇 오른쪽(-y)에서 보면 위팔의 납작한 면과 아래팔의 둥근 면이 보인다
WORDS: dict[str, tuple[str, str]] = {"upper_arm": ("BOMI", "flat"), "forearm": ("HORI", "round")}
# 글자 두께. 납작한 면의 글자는 한 두께이고, 둥근 면의 글자는 앞면이 평평하고 뒷면이 관의 곡면이라 관 꼭대기 위가
# 가장 얇다 (대문자 높이 12 mm의 가장자리에서 1.03 mm 더 두껍다). 둘 다 앞면을 바닥에 두고 뽑아서 층 경계의 값이다.
# BOMI는 위팔과 아래팔 사이, 위팔과 J1 몸통 왼쪽 면 사이의 5.8 mm 층에 이만큼 나온다
LETTER_T = {"flat": 1.4, "round": 1.0}
# 관에 내는 구멍의 지름 (그린 값). 흰색 세로 구멍은 그린 값보다 0.3쯤 작게 뽑혔다 (M3 구멍과 핀 구멍의 시험편,
# data/measured.toml). 그대로면 실제 지름은 2.3쯤이다 (가정: 이 지름의 시험편은 없다)
LETTER_HOLE = 2.6
# 구멍 가장자리와 글자 윤곽 사이에 남기는 살 (그린 값): 구멍이 글자 뒤에 숨는다. 획 굵기가 3.37 mm(I, O)라 구멍을
# 더 키울 수 없다
LETTER_COVER = 0.35
# 핀의 지름. 검정 핀 시험편(coupons.py의 fit4_letter_pegs: 2.1 ~ 2.6)을 뽑은 흰 링크의 실제 구멍에 꽂아, 살짝 똑
# 소리를 내며 끼워지는 칸으로 골랐다 (2026-11-23, 작업 안내 "글자 핀 맞춰 보기"). 납작한 면의 구멍(뽑을 때 바닥 면,
# 입구가 첫 층)과 둥근 면의 구멍(뽑을 때 관 꼭대기의 막힌 구멍)은 그린 지름이 같아도 맞는 핀이 달라서 따로 골랐다
PEG_D = {"flat": 2.3, "round": 2.4}
PEG_LEAD = (0.3, 0.4)  # 핀 끝을 깎은 양 (반지름, 길이): 구멍에 들어가기 시작하는 자리
# 납작한 면의 핀 길이: 벽(2.6)을 다 뚫은 구멍에 0.4 덜 들어가서 관 속으로 나오지 않는다 (관 속에 서보 선이 지나간다)
PEG_LEN_FLAT = 2.2
# 둥근 면의 구멍 깊이 (관 꼭대기에서). 막힌 구멍이라 관 속 천장의 다리 층(꼭대기에서 4.0)을 뚫지 않고, 바닥이 뽑을 때의
# 층 경계에 온다. 핀 끝은 바닥에서 0.4 뜬다
ROUND_HOLE_DEPTH = 2.6
PEG_FLOOR_GAP = 0.4
# 낱자마다 핀 둘을 둘 곳: 글자 윤곽의 바깥 상자 가운데에서 (읽는 방향, 글자 위쪽) [mm]. 핀은 이 점 둘레
# PEG_SNAP 안에서 윤곽과 가장 먼 점(획의 한가운데)에 선다 (peg_points). 획이 3.4 mm뿐이라 가운데에서 0.05 mm만
# 벗어나도 살이 LETTER_COVER보다 얇아진다.
# - 낱자마다 핀 사이 간격이 0.5 mm 넘게 달라서 다른 낱자의 구멍에는 꽂히지 않는다 (6.0 ~ 11.1 mm). O와 I는 두 글자에
#   다 있고 뒷면이 달라서(납작한 면, 둥근 면) 서로 다른 간격으로 했다.
# - 핀 둘이 같은 굵기라 반 바퀴 돌려 꽂는 것은 막지 못한다. PEG_MIRRORED의 낱자(H, I, 위팔의 O)는 둘째 핀이 첫째
#   핀의 가운데 대칭이라 돌려도 같은 자리에 같은 모양이다. B, M, R은 거꾸로 선 글자가 되고, 아래팔의 O(핀 둘이
#   위쪽)는 3.4 mm 비켜 앉아 눈에 띈다
PEGS: dict[str, tuple[tuple[float, float], tuple[float, float]]] = {
    "letter_bomi_b": ((-3.9, 0.3), (3.9, -3.0)),  # 세로 획의 가운데, 아래 배의 오른쪽
    "letter_bomi_o": ((-4.91, 0.0), (4.91, 0.0)),  # 양옆
    # M: 두 세로 획의 위쪽 바깥 (비스듬한 획과 만나 굵은 곳)
    "letter_bomi_m": ((-5.3, 3.0), (5.3, 3.0)),
    "letter_bomi_i": ((0.0, 3.5), (0.0, -3.5)),
    "letter_hori_h": ((-3.98, 3.9), (3.98, -3.9)),  # 왼쪽 획의 위, 오른쪽 획의 아래
    "letter_hori_o": ((-4.62, 1.75), (4.62, 1.75)),  # 양옆의 위쪽
    "letter_hori_r": ((-3.7, -1.0), (3.5, 2.0)),  # 세로 획의 아래쪽, 배의 오른쪽
    "letter_hori_i": ((0.0, 3.0), (0.0, -3.0)),
}
PEG_MIRRORED = frozenset({"letter_bomi_o", "letter_bomi_i", "letter_hori_h", "letter_hori_i"})
PEG_SNAP = 0.1  # 핀을 획의 한가운데로 옮길 때 찾는 반지름
PEG_APART = 0.5  # 낱자끼리 핀 사이 간격이 달라야 하는 양

# ---- 아래팔 끝의 나사 자리 (2026-11-22) ----
# D405 고리가 J4·J5 틈을 못 피하면 선을 잡거나 틈을 막는 부품을 여기에 덧단다 (0031의 "안 될 때"). 목의 양옆에 선
# 돋움 둘이고, 돋움마다 M3 나사 하나가 옆(±z)에서 들어와 너트에 물린다. 손목 몸통의 빈 돋움과 같은 너트 홈이다
# (흰색 fit1_nut_drops 값: 뒤(-y, 뽑을 때 위)에서 떨어뜨려 넣는다). 손목 몸통과 달리:
# - 돋움이 납작한 면(바닥)까지 내려와 있어 밑을 받칠 비탈이 없고, 살을 줄였다 (하나 0.26 g. 손목 몸통의 돋움은
#   0.8 g쯤이다): 어깨의 버팀 토크가 한계(정격의 0.8배)에 닿아 있어서, J4 집에 더하는 1 g이 0.0035 N·m다.
# - 너트 홈의 입구가 뒷덮개에서 39 mm 떨어진 돋움 윗면에 있어, 팔을 다 조립한 뒤에도 너트가 들어간다.
# 자리는 이음매(납작한 면) 바로 뒤, J4 축에서 28.8 ~ 37.2 mm다: 멈춤 귀와 영점 핀 귀가 쓸고 가는 자리(33.4)는 납작한
# 면 너머이고, 돋움은 그 면을 넘지 않는다. 멈춤 홈(반지름 32.8)에 걸친 안쪽 모서리는 홈과 같이 따인다. 쉬는 자세의
# 선반은 집의 맞은편(서보 가까운 끝 쪽)에 있다
SEAT_X = -33.0  # 나사의 x. 너트가 내려가는 길이 관의 둥근 옆(x ≤ -36)을 비킨다
SEAT_HW = 4.2  # 돋움의 반폭 (너트 홈 5.6에 양옆 살 1.4)
# 돋움의 높이 (납작한 면에서. 윗면이 뽑을 때의 층 경계다). 앉은 너트의 위 꼭짓점(8.03)보다 0.57 높다
SEAT_H = 8.6
SEAT_UP = 4.8  # 나사의 높이 (납작한 면에서). 너트 홈의 뾰족한 밑이 바닥에서 1.57 뜬다
_DROP = j._WHITE
assert _DROP.m3_nut_drop_width_mm is not None and _DROP.m3_nut_drop_thickness_mm is not None
SEAT_NUT = (_DROP.m3_nut_drop_width_mm, _DROP.m3_nut_drop_thickness_mm)  # 너트 홈의 폭과 두께
# 돋움 면에서 너트 홈까지의 살: 조인 너트가 붙는 벽이다 (뽑을 때 선 벽이라 벽 선 석 줄이다)
SEAT_ROOF = 1.2
# 너트 홈 안쪽의 살: 나사 구멍이 목의 벽을 파지 않고 돋움 안에서 끝난다 (목 속은 선이 지나가는 빈자리다)
SEAT_SKIN = 1.4
SEAT_FACE = j.NECK_HW + SEAT_SKIN + SEAT_NUT[1] + SEAT_ROOF  # 돋움 면의 |z| (20.7)
# 나사 구멍의 깊이 (5.4). 두께 3 mm 부품과 M3 × 8이면 나사 끝이 너트를 1.0 지나고 구멍 바닥과 0.4 뜬다
SEAT_HOLE = SEAT_FACE - j.NECK_HW

# ---- D405 선을 묶는 찍찍이 타이 (0031: 관 양 끝에 감는다. 링크에는 자리를 내지 않는다) ----
# 폭과 한 겹 두께는 data/measured.toml의 velcro_tie다 (폭 10은 상품 규격, 두께 0.93은 잰 값). 무게 0.6 g은 가정이다
# (폭 12, 두께 1.3으로 어림했던 값이라 넉넉한 쪽이다)
_TIE = j._MEASURED.velcro_tie
assert _TIE is not None
TIE_W, TIE_T, TIE_G = _TIE.width_mm, _TIE.thickness_mm, 0.6
# 관 끝에서 타이 가운데까지: (발 상자의 끝에서, 끝 집의 목에서).
# - 발 쪽 타이는 앞 관절 출력 원판의 멈춤 귀 끝(관의 납작한 면 밑, 발 끝에서 5.2 ~ 5.8 mm)과 2 mm 넘게 뜬다.
# - 끝 집 쪽 타이는 선 창과 거기서 올라오는 서보 선(목 끝에서 5.6 mm)을 비킨다. 그리고 팔꿈치에서 마주 보는 두 타이
#   (위팔의 끝 집 쪽, 아래팔의 발 쪽)가 아래팔이 다 접혀도 위에서 볼 때 겹치지 않는다: 아래팔 타이의 가장 먼 모서리가
#   J3 축에서 48.0 mm, 위팔 타이의 가까운 끝이 54.0 mm다 (손으로 감는 자리라 3 mm 넘게 띄운다). 겹치면 두 타이가
#   5.8 mm 층에서 마주 봐서, 겹쳐 감은 끝(두 겹)이 둘 다 납작한 면에 올 때 틈이 2.1 mm가 된다
TIE_IN = (14.0, 19.0)
# 선이 가는 관의 가장자리: 팔꿈치 바깥(팔을 곧게 세우면 로봇 뒤)이다. 위팔은 -z, 아래팔은 +z 쪽이다
CABLE_SIDE = {"upper_arm": -1.0, "forearm": 1.0}
_CABLE = j._MEASURED.d405_usb2_cable
assert _CABLE is not None and _CABLE.diameter_mm is not None
CABLE_D = _CABLE.diameter_mm  # D405 선의 지름 (잰 값, 3.0)


def slot_reach(foot: Foot) -> float:
    """발 바닥의 긴 틈이 끝나는 곳 (앞 관절 축에서 끝벽 쪽으로). 혼 볼트 머리 위 구멍 속으로 FOOT_SLOT_OPEN 들어온다."""
    return math.sqrt(foot.head_r**2 - FOOT_SLOT_Z[0] ** 2) - FOOT_SLOT_OPEN


def _chamfer_x(x0: float, x1: float, y_top: float, hz: float, c: float) -> Compound:
    """x 방향 상자의 뒷면 쪽(-y) 두 모서리를 45°로 깎는 쐐기 둘 (y_top이 상자 윗면, 반폭 hz)."""
    out = None
    for sg in (1, -1):
        pts = [
            (y_top - 0.01, sg * (hz + 0.01)),
            (y_top + c, sg * (hz + 0.01)),
            (y_top - 0.01, sg * (hz - c)),
        ]
        w = j.prism_x(pts, x0, x1)
        out = w if out is None else solid(out + w)
    assert out is not None
    return out


def _tube_cavity(r_in: float, yc: float) -> list[tuple[float, float]]:
    """관 속의 단면 (y, z). 누워서 뽑히므로 윗부분을 45° 면 둘과 평평한 다리로 잇는다 (둥근 천장은 처진다).

    바닥(납작한 벽의 안쪽 면)에서 45° 접점까지는 반지름 r_in의 원호(3°씩 나눈 꺾은선)이고, 그 위는 접선 둘과 다리다.
    한 다각형으로 그린다: 원기둥과 지붕을 따로 그려 합치면 접점 높이에 폭 0.01 mm의 수평 턱이 남아, 층 가운데에
    걸친 수평면으로 잡혔다.
    """
    u_top = r_in * 0.91
    v_top = r_in * math.sqrt(2) - u_top
    a_floor = math.asin((j.FRONT - WALL - yc) / r_in)  # 원호가 바닥과 만나는 각 (관 가운데에서)
    n = math.ceil(math.degrees(a_floor + math.pi / 4) / 3.0)
    angles = [a_floor - (a_floor + math.pi / 4) * k / n for k in range(n + 1)]
    side = [(yc + r_in * math.sin(a), r_in * math.cos(a)) for a in angles]
    roof = [(yc - u_top, v_top), (yc - u_top, -v_top)]
    return [*side, *roof, *[(y, -z) for y, z in reversed(side)]]


def _wall_opening() -> list[tuple[float, float]]:
    """발 끝벽에 낸 선 자리의 윤곽 (y, z): 밑의 낮은 구멍, 세로 틈, 높은 창이 하나로 이어진다.

    낮은 구멍의 윗변은 45°보다 조금 세운 비탈로 세로 틈에 이어져서 뽑을 때 걸치는 면이 없다. 걸치는 것은 창의
    윗변(폭 5.7 mm의 다리)뿐이다.
    """
    z0, z1 = FOOT_SLOT_Z
    zc, s = (z0 + z1) / 2, FOOT_SLIT_W / 2
    neck = FOOT_SLOT_H + 1.1 * (z1 - zc - s)  # 비탈이 세로 틈에 닿는 높이
    w0, w1 = FOOT_WINDOW
    outline = [
        (-1.0, z0),
        (-1.0, z1),
        (FOOT_SLOT_H, z1),
        (neck, zc + s),
        (w0, zc + s),
        (w0, z1),
        (w1, z1),
        (w1, z0),
        (w0, z0),
        (w0, zc - s),
        (neck, zc - s),
        (FOOT_SLOT_H, z0),
    ]
    return [(j.FRONT - h, z) for h, z in outline]


# ------------------------------------------------------------------------------------------------
# 관 글자


def letter_keys(name: str) -> list[str]:
    """링크에 다는 낱자의 이름 (읽는 차례): letter_bomi_b …"""
    word = WORDS[name][0]
    return [f"letter_{word.lower()}_{ch.lower()}" for ch in word]


def _link_of(key: str) -> tuple[str, int]:
    """낱자의 이름 → (링크, 글자 안의 차례)."""
    for name in WORDS:
        keys = letter_keys(name)
        if key in keys:
            return name, keys.index(key)
    raise KeyError(key)


@functools.cache
def _glyphs(word: str) -> tuple[Face, ...]:
    """글자의 낱자 윤곽 (읽는 차례). 스케치 좌표는 x가 읽는 방향, y가 글자 위쪽이고 글자 전체의 가운데가 원점이다."""
    text = Text(word, font_size=FONT_SIZE, font_path=str(FONT), align=(Align.CENTER, Align.CENTER))
    faces = tuple(sorted(text.faces(), key=lambda f: f.center().X))
    if len(faces) != len(word):
        raise RuntimeError(f"{word}: 낱자 윤곽이 {len(faces)}개다 (글꼴 파일을 확인한다)")
    return faces


@functools.cache
def glyph_outline(key: str) -> Flat:
    """낱자의 윤곽 (스케치 좌표의 다각형. 곡선은 잘게 나눈다)."""
    name, k = _link_of(key)
    face = _glyphs(WORDS[name][0])[k]

    def ring(wire: Wire) -> list[tuple[float, float]]:
        pts: list[tuple[float, float]] = []
        for edge in wire.order_edges():
            for i in range(24):
                p = edge.position_at(i / 24)
                pts.append((p.X, p.Y))
        return pts

    shape = Flat(ring(face.outer_wire()), [ring(w) for w in face.inner_wires()])
    return shape if shape.is_valid else shape.buffer(0)


@functools.cache
def peg_points(key: str) -> tuple[tuple[float, float], ...]:
    """낱자의 핀 둘의 자리 (스케치 좌표): PEGS의 점 둘레에서 윤곽과 가장 먼 점 (0.02 mm 격자).

    곧은 획에서는 가운데 선 위의 점이 모두 같은 거리라, 그 가운데 PEGS의 점에 가장 가까운 것을 고른다.
    """
    outline = glyph_outline(key)
    edge = outline.boundary
    x0, y0, x1, y1 = outline.bounds
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    step = 0.02
    n = round(PEG_SNAP / step)
    grid = np.arange(-n, n + 1) * step
    da, db = (v.ravel() for v in np.meshgrid(grid, grid, indexing="ij"))
    off = np.hypot(da, db)
    out: list[tuple[float, float]] = []
    for ds, dt in PEGS[key]:
        if out and key in PEG_MIRRORED:
            out.append((round(2 * cx - out[0][0], 2), round(2 * cy - out[0][1], 2)))
            break
        xs, ys = cx + ds + da, cy + dt + db
        ok = (off <= PEG_SNAP + 1e-9) & shapely.contains_xy(outline, xs, ys)
        if not ok.any():
            raise RuntimeError(f"{key}: 핀 자리가 글자 밖이다")
        far = np.asarray(shapely.distance(edge, shapely.points(xs, ys)), dtype=float)
        clear = np.where(ok, np.round(far, 2), -1.0)
        near = np.where(clear == clear.max(), off, np.inf)
        k = int(near.argmin())
        out.append((round(float(xs[k]), 2), round(float(ys[k]), 2)))
    return tuple(out)


def peg_spacing(key: str) -> float:
    """낱자의 핀 둘 사이 간격 [mm]."""
    a, b = peg_points(key)
    return math.dist(a, b)


def peg_cover(key: str) -> float:
    """구멍 가장자리와 글자 윤곽 사이에 남는 살의 가장 작은 값 [mm] (그린 값). 음수면 구멍이 글자 밖으로 보인다."""
    edge = glyph_outline(key).boundary
    return min(edge.distance(Point(p)) for p in peg_points(key)) - LETTER_HOLE / 2


def letter_mid(name: str) -> float:
    """글자 가운데의 x: 관에 감는 두 타이의 가운데 (관의 쓸 수 있는 길이의 가운데에서 발 쪽으로 2.5 mm다)."""
    foot_x, house_x = tie_x(name)
    return (foot_x + house_x) / 2


def _tube_top(name: str) -> float:
    """관의 둥근 면 꼭대기의 y (뽑을 때 맨 위)."""
    _span, tube_r, _foot = LINKS[name]
    return j.FRONT - (2 * tube_r - EMBED)


def _letter_plane(name: str) -> Plane:
    """글자의 스케치 좌표를 링크 좌표로 옮기는 면: 원점은 글자 가운데이고 글자가 관에 닿는 높이, 법선은 관 밖이다.

    읽는 방향은 +x(앞 관절 → 끝 관절)다. 관 밖에서 볼 때 글자가 뒤집히지 않으려면 글자 위쪽이 납작한 면에서는 -z,
    둥근 면에서는 +z다. 둘 다 팔을 곧게 세웠을 때 로봇 뒤쪽이다 (placement의 틀: 위팔의 +z는 앞, 아래팔의 +z는 뒤).
    """
    x = letter_mid(name)
    if WORDS[name][1] == "flat":
        return Plane(origin=(x, j.FRONT, 0.0), x_dir=(1.0, 0.0, 0.0), z_dir=(0.0, 1.0, 0.0))
    return Plane(origin=(x, _tube_top(name), 0.0), x_dir=(1.0, 0.0, 0.0), z_dir=(0.0, -1.0, 0.0))


def letter_point(name: str, s: float, t: float, lift: float = 0.0) -> Vec:
    """글자의 스케치 좌표 (s, t)의 점을 링크 좌표로. lift는 관에 닿는 높이에서 관 밖으로 띄운 거리다."""
    plane = _letter_plane(name)
    p = plane.origin + plane.x_dir * s + plane.y_dir * t + plane.z_dir * lift
    return (p.X, p.Y, p.Z)


def peg_spots(name: str) -> list[tuple[str, float, float]]:
    """링크에 낼 글자 구멍의 자리: (낱자, x, z) (링크 좌표)."""
    out: list[tuple[str, float, float]] = []
    for key in letter_keys(name):
        for s, t in peg_points(key):
            x, _y, z = letter_point(name, s, t)
            out.append((key, x, z))
    return out


def letter_holes(name: str) -> Compound:
    """링크에서 따낼 글자 구멍. 납작한 면은 벽을 뚫고, 둥근 면은 꼭대기에서 ROUND_HOLE_DEPTH까지 막힌 구멍이다."""
    r = LETTER_HOLE / 2
    flat = WORDS[name][1] == "flat"
    top = _tube_top(name)
    out: Compound | None = None
    for _key, x, z in peg_spots(name):
        if flat:
            one = cyl_y(r, j.FRONT - WALL - 1.0, j.FRONT + 1.0, x, z)
        else:
            one = cyl_y(r, top - 1.0, top + ROUND_HOLE_DEPTH, x, z)
        out = one if out is None else solid(out + one)
    assert out is not None
    return out


def _peg(x: float, z: float, y_root: float, y_tip: float, d: float) -> Compound:
    """y 방향 핀: y_root에서 y_tip까지. 끝은 PEG_LEAD만큼 깎는다."""
    cut, lead = PEG_LEAD
    sg = 1.0 if y_tip > y_root else -1.0
    y_mid = y_tip - sg * lead
    lo, hi = sorted((y_root, y_mid))
    body = cyl_y(d / 2, lo, hi, x, z)
    over = 0.01  # 몸통과 겹치는 길이 (맞닿게만 두면 합칠 때 형상이 깨질 수 있다)
    tip = Cone(d / 2, d / 2 - cut, lead + over, align=(Align.CENTER, Align.CENTER, Align.MIN))
    # 원뿔의 축(지역 +z)을 핀이 뻗는 쪽(±y)으로 돌린다
    turned = Rot(-90.0 if sg > 0 else 90.0, 0.0, 0.0) * tip
    return solid(body + Pos(x, y_mid - sg * over, z) * turned)


@functools.cache
def letter(key: str) -> Compound:
    """낱자 하나 (검정). 링크에 꽂힌 자리의 링크 좌표다. 앞면을 바닥에 두고 뽑는다 (letter_up)."""
    name, k = _link_of(key)
    word, side = WORDS[name]
    plane = _letter_plane(name)
    face = _glyphs(word)[k].moved(Location(plane))
    thick, d = LETTER_T[side], PEG_D[side]
    spots = [(x, z) for kk, x, z in peg_spots(name) if kk == key]
    if side == "flat":
        part = solid(extrude(face, amount=thick))
        for x, z in spots:
            part = solid(part + _peg(x, z, j.FRONT + 0.2, j.FRONT - PEG_LEN_FLAT, d))
    else:
        span, tube_r, _foot = LINKS[name]
        top = _tube_top(name)
        # 앞면(꼭대기 위 thick)에서 관 속으로 넉넉히 밀고 관의 겉(원기둥)으로 자른다: 뒷면이 관의 곡면이 된다
        front = face.moved(Location((0.0, -thick, 0.0)))
        raw = solid(extrude(front, amount=-(thick + 4.0)))
        part = solid(raw - cyl_x(tube_r, -span, 0.0, j.FRONT - (tube_r - EMBED), 0.0))
        tip = top + ROUND_HOLE_DEPTH - PEG_FLOOR_GAP
        for x, z in spots:
            part = solid(part + _peg(x, z, top - thick + 0.2, tip, d))
    if not part.is_valid or len(part.solids()) != 1:
        raise RuntimeError(f"{key}: 글자 형상이 깨졌다")
    return part


def letter_up(key: str) -> Vec:
    """낱자를 뽑을 때 위를 보는 방향 (링크 좌표): 앞면이 바닥이고 핀이 위로 선다."""
    return (0.0, -1.0, 0.0) if WORDS[_link_of(key)[0]][1] == "flat" else (0.0, 1.0, 0.0)


def letters(name: str) -> dict[str, Compound]:
    """링크의 낱자들 (링크 좌표)."""
    return {key: letter(key) for key in letter_keys(name)}


# ------------------------------------------------------------------------------------------------
# 아래팔 끝의 나사 자리


def _seat_blocks() -> Compound:
    """목 양옆의 돋움 둘 (겉). 납작한 면에서 SEAT_H까지 선 상자다. 목의 벽 속으로 0.5 겹쳐 한 덩어리가 된다."""
    out: Compound | None = None
    for sg in (1, -1):
        z0, z1 = sorted((sg * (j.NECK_HW - 0.5), sg * SEAT_FACE))
        one = box(SEAT_X - SEAT_HW, SEAT_X + SEAT_HW, j.FRONT - SEAT_H, j.FRONT, z0, z1)
        out = one if out is None else solid(out + one)
    assert out is not None
    return out


def seat_nuts() -> list[tuple[str, tuple[float, float, float], Vec, Vec]]:
    """나사 자리의 너트 둘: (이름, 조인 너트가 앉는 홈의 가운데, 볼트 방향, 입구 쪽 방향)."""
    z_nut = SEAT_FACE - SEAT_ROOF - SEAT_NUT[1] / 2
    y = j.FRONT - SEAT_UP
    return [
        (
            f"나사 자리 너트 ({'+' if sg > 0 else '-'}z)",
            (SEAT_X, y, sg * z_nut),
            (0.0, 0.0, 1.0),
            UP,
        )
        for sg in (1, -1)
    ]


def _seat_cuts() -> Compound:
    """돋움의 너트 홈(뒤에서 떨어뜨려 넣는다)과 나사 구멍. 구멍은 돋움 안에서 끝난다 (목의 벽은 파지 않는다)."""
    w, t = SEAT_NUT
    length = SEAT_H - SEAT_UP + 1.0
    out: Compound | None = None
    for _name, (x, y, z), axis, slide in seat_nuts():
        cut = nut_trap((x, y, z), axis, slide, length, w, t)
        z0, z1 = (j.NECK_HW, SEAT_FACE + 1.0) if z > 0 else (-SEAT_FACE - 1.0, -j.NECK_HW)
        cut = solid(cut + cyl_z(j.M3_FREE_H / 2, z0, z1, x, y))
        out = cut if out is None else solid(out + cut)
    assert out is not None
    return out


def seat_screws(length: float = 8.0, plate: float = 3.0) -> list[j.Part]:
    """나사 자리에 두께 plate인 부품을 M3 × length로 달았을 때의 나사와 너트 (검사용. 지금은 달 것이 없다)."""
    out: list[j.Part] = []
    for name, (x, y, z), _axis, _slide in seat_nuts():
        sg = 1.0 if z > 0 else -1.0
        head = sg * (SEAT_FACE + plate)
        a, b = sorted((head, head - sg * length))
        bolt = cyl_z(1.5, a, b, x, y)
        h0, h1 = sorted((head, head + sg * 1.65))
        bolt = solid(bolt + cyl_z(2.85, h0, h1, x, y))
        out.append(j.Part(name.replace("너트", "나사"), bolt, j.STEEL, False, "metal"))
        roof = sg * (SEAT_FACE - SEAT_ROOF)  # 조인 너트가 붙는 면
        n0, n1 = sorted((roof, roof - sg * j.NUT_T))
        # 너트는 홈과 같은 방향이다: 꼭짓점이 홈의 입구 쪽(뽑을 때 위)을 본다
        hexagon = extrude(RegularPolygon(5.4 / 2, 6, major_radius=False), amount=j.NUT_T)
        nut = hexagon.moved(Location(Plane(origin=(x, y, n0), x_dir=UP, z_dir=(0.0, 0.0, 1.0))))
        nut = solid(nut) - cyl_z(1.5, n0 - 1, n1 + 1, x, y)
        out.append(j.Part(name, solid(nut), j.STEEL, False, "metal"))
    return out


# ------------------------------------------------------------------------------------------------
# 링크


def link(
    length: float,
    tube_r: float,
    foot: Foot = STS_FOOT,
    *,
    holes: Compound | None = None,
    seats: bool = False,
    win_x: tuple[float, float] = j.WIN_X,
) -> Compound:
    """링크 하나 (흰색). 납작한 면(y = FRONT)을 바닥에 두고 뽑는다.

    holes는 따낼 글자 구멍, seats면 끝 집의 목 양옆에 나사 자리를 낸다. 이름으로 다 갖춘 링크를 만드는 것은 body다.
    """
    xa = -length  # 앞 관절 축
    h_t = 2 * tube_r - EMBED  # 관과 발 상자의 높이
    y_top = j.FRONT - h_t
    yc = j.FRONT - (tube_r - EMBED)  # 관 가운데
    hx, hw_out, hw = foot.hx, foot.hw, foot.hw - WALL  # 발 상자의 반 길이, 바깥 반폭, 안쪽 반폭
    # ---- 겉: 집, 관, 발 상자, 귀, 핀 귀
    tube = cyl_x(tube_r, xa + hx - 1, NECK_END + 4, yc, 0.0)
    tube &= box(xa, 0, y_top - 1, j.FRONT, -tube_r - 1, tube_r + 1)
    foot_box = box(xa - hx, xa + hx, y_top, j.FRONT, -hw_out, hw_out)
    foot_box -= _chamfer_x(xa - hx - 1, xa + hx + 1, y_top, hw_out, foot.chamfer)
    ears = cyl_y(foot.disc_r, j.FRONT - j.EAR_T, j.FRONT, xa, 0.0)
    ears -= box(
        xa - foot.disc_r - 5,
        xa + foot.disc_r + 5,
        j.FRONT - j.EAR_T - 1,
        j.FRONT + 1,
        -hw_out + 0.01,
        hw_out - 0.01,
    )
    tab = box(xa - foot.pin_r, xa - hx + 0.01, j.FRONT - j.EAR_T, j.FRONT, -4.4, 4.4)
    tab += cyl_y(4.4, j.FRONT - j.EAR_T, j.FRONT, xa - foot.pin_r, 0.0)
    body = solid(j.drum_shell(NECK_END) + tube + foot_box + ears + tab)
    if seats:
        body = solid(body + _seat_blocks())
    # ---- 속: 목은 관 높이까지 관 속으로 이어지고, 그 위(서보 뒷면 쪽)는 목 끝 벽에서 막힌다
    inner = j.drum_cavity(NECK_END, roof_y=y_top + WALL, roof_x=NECK_END + WALL)
    tube_in = j.prism_x(_tube_cavity(tube_r - WALL, yc), xa + hx - 4, NECK_END + 6)
    foot_in = box(xa - hx + WALL, xa + hx + 2, y_top + WALL, j.FRONT - WALL, -hw, hw)
    foot_in -= _chamfer_x(xa - hx, xa + hx + 3, y_top + WALL, hw, foot.chamfer - 1.5)
    h = solid(body - solid(inner + tube_in + foot_in))
    # ---- 발: 혼 볼트 머리 위 구멍, 몸통 나사 구멍, 영점 핀 구멍, 선이 들어오는 자리
    h = solid(h - cyl_y(foot.head_r, j.FRONT - WALL - 1, j.FRONT + 1, xa, 0.0))
    for dx, z in foot.rim:
        h = solid(h - cyl_y(j.M3_FREE / 2, j.FRONT - j.EAR_T - 1, j.FRONT + 1, xa + dx, z))
    h = solid(h - cyl_y(j.PIN_HOLE / 2, j.FRONT - j.EAR_T - 1, j.FRONT + 1, xa - foot.pin_r, 0.0))
    z0, z1 = FOOT_SLOT_Z
    # 끝벽: 밑의 낮은 구멍, 세로 틈, 높은 창
    h = solid(h - j.prism_x(_wall_opening(), xa - hx - 1, xa - hx + WALL + 0.01))
    # 긴 틈은 혼 볼트 머리 위 구멍과 겹쳐서, 구멍에서 끝벽까지 선을 옆으로 밀어 넣을 수 있다
    reach = slot_reach(foot)
    h = solid(h - box(xa - hx + WALL, xa - reach, j.FRONT - WALL - 1, j.FRONT + 1, z0, z1))
    if holes is not None:
        h = solid(h - holes)
    if seats:
        h = solid(h - _seat_cuts())
    return j.cut_joint_features(h, window=True, win_x=win_x)


@functools.cache
def body(name: str) -> Compound:
    """다 갖춘 링크 (흰색): 글자 구멍과, 아래팔이면 끝의 나사 자리까지. 뽑는 것은 이 모양이다."""
    length, tube_r, foot = LINKS[name]
    part = link(length, tube_r, foot, holes=letter_holes(name), seats=name == "forearm")
    if not part.is_valid or len(part.solids()) != 1:
        raise RuntimeError(f"{name}: 링크 형상이 깨졌다")
    return part


def link_cover() -> Compound:
    """링크 끝 집의 뒷덮개 (검정). 목이 관으로 내려가는 자리까지 덮는다."""
    return j.back_cover(NECK_END, end_skirt=True)


# ------------------------------------------------------------------------------------------------
# D405 선의 타이와 묶인 선 (닿음과 가림을 보는 형상. 링크의 부분이 아니다)


def tie_x(name: str) -> tuple[float, float]:
    """링크에 감는 타이 둘의 가운데 x: (발 쪽, 끝 집 쪽)."""
    span, _tube_r, foot = LINKS[name]
    return (-span + foot.hx + TIE_IN[0], NECK_END - TIE_IN[1])


def cable_yz(name: str) -> tuple[float, float]:
    """관 가장자리에 놓인 D405 선의 가운데 (y, z): 관 가운데 높이, 팔꿈치 바깥쪽 옆."""
    _span, tube_r, _foot = LINKS[name]
    return (j.FRONT - (tube_r - EMBED), CABLE_SIDE[name] * (tube_r + CABLE_D / 2))


def _tie_section(name: str) -> Flat:
    """타이가 감싸는 단면 (y, z): 납작하게 깎인 관과 그 옆의 선을 팽팽히 두른 볼록한 둘레."""
    _span, tube_r, _foot = LINKS[name]
    yc = j.FRONT - (tube_r - EMBED)
    half = math.sqrt(tube_r**2 - (tube_r - EMBED) ** 2)
    pts = [
        (yc + tube_r * math.sin(a), tube_r * math.cos(a))
        for a in (2 * math.pi * k / 96 for k in range(96))
        if yc + tube_r * math.sin(a) <= j.FRONT
    ]
    pts += [(j.FRONT, half), (j.FRONT, -half)]
    cy, cz = cable_yz(name)
    pts += [
        (cy + CABLE_D / 2 * math.sin(a), cz + CABLE_D / 2 * math.cos(a))
        for a in (2 * math.pi * k / 24 for k in range(24))
    ]
    hull = MultiPoint(pts).convex_hull
    assert isinstance(hull, Flat)
    return hull


def tie_band(name: str, x: float) -> Compound:
    """x에 감은 타이 한 겹 (가정한 폭과 두께). 겹친 끝은 둥근 면 쪽에 둔다고 본다 (납작한 면은 한 겹이다)."""
    inner = _tie_section(name)
    outer = inner.buffer(TIE_T, quad_segs=3)
    x0, x1 = x - TIE_W / 2, x + TIE_W / 2

    def yz(shape: Flat) -> list[tuple[float, float]]:
        return [(float(y), float(z)) for y, z in list(shape.exterior.coords)[:-1]]

    return solid(j.prism_x(yz(outer), x0, x1) - j.prism_x(yz(inner), x0 - 1, x1 + 1))


def cable_run(name: str) -> Compound:
    """두 타이 사이에 묶여 관 가장자리를 따라가는 D405 선 (손으로 옮길 수 없는 구간)."""
    x0, x1 = tie_x(name)
    y, z = cable_yz(name)
    return rod((x0 - TIE_W / 2, y, z), (x1 + TIE_W / 2, y, z), CABLE_D / 2)


def d405_parts(name: str) -> list[j.Part]:
    """링크에 묶인 D405 선과 타이 둘 (링크 좌표, 가정한 모양)."""
    foot_x, house_x = tie_x(name)
    return [
        j.Part("타이 (발 쪽)", tie_band(name, foot_x), color="wire"),
        j.Part("타이 (끝 집 쪽)", tie_band(name, house_x), color="wire"),
        j.Part("묶인 선", cable_run(name), color="wire"),
    ]


def fits_bed(shape: Compound) -> tuple[bool, float]:
    """출력 판(256 × 256)에 들어가는지와, 대각선으로 놓았을 때 필요한 한 변 [mm]."""
    bb = shape.bounding_box()
    diagonal = (bb.size.X + bb.size.Z) / math.sqrt(2)
    return max(bb.size.X, bb.size.Z) <= BED or diagonal <= BED, diagonal


# 뽑는 부품의 한글 이름 (출력 안내에 쓴다, horibot_design.slicing)
LABELS = {
    "upper_arm": "위팔",
    "forearm": "아래팔",
    "link_cover": "링크 뒷덮개",
    "letter_bomi_b": "글자 B (위팔)",
    "letter_bomi_o": "글자 O (위팔)",
    "letter_bomi_m": "글자 M (위팔)",
    "letter_bomi_i": "글자 I (위팔)",
    "letter_hori_h": "글자 H (아래팔)",
    "letter_hori_o": "글자 O (아래팔)",
    "letter_hori_r": "글자 R (아래팔)",
    "letter_hori_i": "글자 I (아래팔)",
}


def printed() -> dict[str, tuple[Compound, Vec, str, str, int]]:
    """뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋, 개수).

    글자는 흰 링크를 뽑아 핀 시험편으로 PEG_D를 고른 뒤에 뽑는다 (그 전의 글자는 가정한 핀 지름이다).
    """
    out: dict[str, tuple[Compound, Vec, str, str, int]] = {
        name: (body(name), UP, "white", "strength", 1) for name in LINKS
    }
    out["link_cover"] = (link_cover(), (0.0, 1.0, 0.0), "black", "standard", 2)
    for name in LINKS:
        for key in letter_keys(name):
            out[key] = (letter(key), letter_up(key), "black", "strength", 1)
    return out


def seat_checks() -> dict[str, float]:
    """아래팔 끝의 나사 자리: 너트가 들어가는 길과 둘레의 여유 [mm, mm³]."""
    part = body("forearm")
    entry = max(j.nut_entry_overlap(part, c, a, s) for _n, c, a, s in seat_nuts())
    reach = math.hypot(SEAT_X + SEAT_HW, j.NECK_HW)  # 돋움에서 J4 축에 가장 가까운 모서리
    cover_end = j.BACK_END + j.COVER_SKIRT  # 뒷덮개 테의 끝 (y)
    return {
        "너트를 뒤에서 곧게 떨어뜨릴 때 링크와 겹침 [mm³]": entry,
        "너트 홈의 입구와 뒷덮개 테 사이 (y)": (j.FRONT - SEAT_H) - cover_end,
        "돋움 면이 관의 옆(반지름 18)보다 나온 길이": SEAT_FACE - LINKS["forearm"][1],
        "돋움 면과 집 원통의 옆(반지름 23.6) 사이": j.R_DRUM - SEAT_FACE,
        "돋움에서 J4 축에 가장 가까운 모서리 (멈춤 홈의 바깥 32.8)": reach,
        "돋움과 선 창 사이 (z)": j.NECK_HW - j.WIN_Z,
        "돋움과 끝 집 쪽 타이 사이 (x)": (SEAT_X - SEAT_HW) - (tie_x("forearm")[1] + TIE_W / 2),
        "나사 구멍의 깊이 (돋움 면에서)": SEAT_HOLE,
    }


def layout(name: str) -> dict[str, float]:
    """관 위의 자리 (x, 링크 좌표): 타이 둘, 글자의 두 끝, 그 사이의 틈."""
    foot_x, house_x = tie_x(name)
    edges = [glyph_outline(k).bounds for k in letter_keys(name)]
    mid = letter_mid(name)
    lo, hi = mid + min(b[0] for b in edges), mid + max(b[2] for b in edges)
    return {
        "발 쪽 타이": foot_x,
        "글자 시작": lo,
        "글자 끝": hi,
        "끝 집 쪽 타이": house_x,
        "발 쪽 타이와 글자 사이": lo - (foot_x + TIE_W / 2),
        "글자와 끝 집 쪽 타이 사이": (house_x - TIE_W / 2) - hi,
    }


def report() -> None:
    made = printed()
    for name, (length, tube_r, _foot) in LINKS.items():
        part = made[name][0]
        bb = part.bounding_box()
        c = part.center()
        over = overhangs(part, UP)
        ok, diagonal = fits_bed(part)
        word, side = WORDS[name]
        print(f"{name}: 길이 {length:.0f}, 관 Ø{2 * tube_r:.0f}, 글자 {word} ({side})")
        print(
            f"  크기 {bb.size.X:.1f} × {bb.size.Y:.1f} × {bb.size.Z:.1f} mm, 무게 {printed_mass_g(part, UP):.1f} g, "
            f"부피 중심 x {c.X:.1f} y {c.Y:.1f}"
        )
        fit = (
            "곧게 들어간다" if bb.size.X <= BED else f"대각선으로 놓는다 (한 변 {diagonal:.0f} mm)"
        )
        print(f"  출력 판: {fit if ok else '들어가지 않는다'}")
        print(
            f"  걸침 {over.total:.0f} mm² (거의 수평 {over.flat:.0f}, 가장 넓은 한 자리 {over.largest:.0f})"
        )
        for a, hgt, u, v, flat in over.spots[:8]:
            print(
                f"    높이 {hgt:4.0f}  자리 ({u:6.1f}, {v:5.1f})  넓이 {a:6.1f}  {'수평' if flat else '비스듬'}"
            )
        print(f"  층 가운데에 걸친 수평면: {mid_layer_levels(part, UP) or '없음'}")
        print("  관 위의 자리 (x): " + ", ".join(f"{k} {v:.1f}" for k, v in layout(name).items()))
    print("\n".join(air_wall_lines({k: made[k] for k in LINKS})))
    print("\n글자 (검정, 앞면을 바닥에 두고 뽑는다)")
    print(
        f"  구멍 Ø{LETTER_HOLE} (그린 값), 핀 Ø{PEG_D['flat']} · Ø{PEG_D['round']} (가정: 핀 시험편으로 고른다), "
        f"납작한 면의 핀 길이 {PEG_LEN_FLAT}, 둥근 면의 구멍 깊이 {ROUND_HOLE_DEPTH}"
    )
    for name in LINKS:
        for key in letter_keys(name):
            shape, up, _color, preset, _n = made[key]
            bb = shape.bounding_box()
            over = overhangs(shape, up)
            print(
                f"  {key}: {bb.size.X:.1f} × {bb.size.Y:.1f} × {bb.size.Z:.1f} mm, "
                f"{printed_mass_g(shape, up, preset):.2f} g, 걸침 {over.total:.1f} mm², "
                f"층 가운데 수평면 {mid_layer_levels(shape, up) or '없음'}, 핀 사이 "
                f"{peg_spacing(key):.2f} mm, 구멍을 덮는 살 {peg_cover(key):.2f} mm"
            )
    print("\n".join(air_wall_lines({k: v for k, v in made.items() if k.startswith("letter_")})))
    print("\n아래팔 끝의 나사 자리 (M3 둘, 목 양옆)")
    for label, value in seat_checks().items():
        print(f"  {label}: {value:.2f}")
    hits = j.overlaps(
        [j.Part("아래팔", made["forearm"][0]), *[p for p in seat_screws() if p.density == 0]]
    )
    print(f"  겹침: {hits or '없음'}")
    for name in LINKS:
        link_shape = made[name][0]
        worst = max(solid(link_shape & made[k][0]).volume for k in letter_keys(name))
        print(f"{name}: 글자와 링크의 겹침 {worst:.3f} mm³ (가장 큰 낱자)")


if __name__ == "__main__":
    report()
