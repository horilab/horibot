"""파이 자리: 날개 위의 기둥에 얹는 파이 판(작은 판)과, 그 위아래에 놓이는 것들의 자리 (docs/decisions/0038-base-layout.md).

책상에 무는 판(parts/base.py의 plate)의 날개가 곧 파이 받침이다. 날개의 기둥 넷 위에 파이 판을 M3 × 8 넷으로 달고,
파이 판에 파이(기판만)와 덮개 기둥 넷을 단다. 덮개는 가진 적층 아크릴 케이스의 팬 구멍 판과 팬, 뽑은 철망이다. 파이 판 밑에는
클램프의 위 턱이 든다. 파이와 덮개에 맞춘 것은 모두 이 작은 판에 있어서, 가정한 값이 틀리면 이 판만 다시 뽑는다.

서보 어댑터는 몸통 바로 뒤 가운데, 책상 뒤 가장자리 밖으로 나온 뒤 판에 눕힌다 (parts/base.py의 "서보 어댑터 자리").
위에서 꽂는 서보 플러그와 선, 전원선, 파이로 가는 USB 선을 실제 크기로 그리고, 그 위에 얹는 덮개(adapter_cover)를 뽑는다.

- 두 파이는 같은 방향이다 (USB·랜이 뒤, 전원이 로봇 왼쪽을 보는 긴 변). 그래서 파이 판 둘이 같은 부품이다.
- 컨트롤러 파이(hori1)가 오른쪽, 카메라 파이(hori2)가 왼쪽이다. D405 선의 J1·J2 고리는 팔이 J1을 따라 ±160° 도는
  가운데 방위(로봇 왼쪽 앞)에서 베이스에 묶어야 가장 짧고, 거기서 왼쪽 파이의 USB 단자까지가 가깝다
  (studies/wiring.py의 D405 선). 서보 선은 어느 쪽이든 닿는다.
- 파이: M2.5 × 12 접시머리 나사(케이스의 것)가 기판, 흰 나일론 링(높이 3.17), 파이 판의 돋움을 지나 판 밑의
  M2.5 너트에 물린다. 가정: 케이스에 M2.5 너트가 여덟 개 있다. 구멍은 맞춤 없이 지나가는 구멍이다.
- 덮개 기둥(육각 황동, 몸통 22.93, 한 끝 M3 수나사 6.2): 수나사가 파이 판의 긴 구멍을 지나 판 밑의 M3 너트에 물린다.
  긴 구멍은 x 쪽으로 ±1 mm 길다 (판이 기판 가운데에 맞는지 모른다: 가정).
- 조립 순서: 손에 든 파이 판에 덮개 기둥 넷(밑에서 너트)과 파이(밑에서 너트) → 파이 판을 기둥에 얹고 M3 × 8 넷
  → 젠더와 플러그 → 팬과 철망을 단 아크릴 판 (팬 나사 넷이 철망, 판, 팬을 묶는다). 클램프는 파이 판을 떼지 않고 뒤에서
  끼우고 뺀다.
- 서보 어댑터 (파이와 따로, 언제든 손이 닿는다): 뒤 판의 돋움 넷에 M2 × 5 넷 → 전원선을 나사 단자에 위에서 조인다
  → 서보 플러그 둘(왼쪽 단자에 J3 줄, 가운데 단자에 J1 줄)과 USB-C → 덮개를 위에서 얹는다. 덮개는 선을 뽑지 않고
  들어 뗀다.

좌표(mm)는 parts/base.py와 같다 (J1의 서보 좌표: x 앞, y 위, z 오른쪽 = 로봇 좌표의 -y).

    uv run python -m horibot_design.parts.pi_mount
"""

from __future__ import annotations

import functools
import itertools
import math
from dataclasses import dataclass

from build123d import Compound, Pos, Rot, Sphere

from horibot_design.parts import base as b
from horibot_design.parts import joint as j
from horibot_design.parts.shapes import Point, box, cyl_y, far_apart, rod, solid
from horibot_design.printcheck import air_wall_lines, mid_layer_levels, overhangs, printed_mass_g

_M = j._MEASURED
_CASE = _M.pi_case
assert _CASE is not None
_PLUG = _M.base_plug

# ---- 누가 어느 쪽인가 (z의 부호: +1이 로봇 오른쪽) ----
CONTROLLER, CAMERA = 1, -1

# ---- 파이 판 ----
PI_HOLES = (
    (39.0, 24.5),
    (39.0, -24.5),
    (-19.0, 24.5),
    (-19.0, -24.5),
)  # 기판 가운데에서 (x, 옆). 공식 도면
PI_HOLE_D = 3.0  # M2.5가 지나가는 구멍 (맞춤 없이 넉넉히)
PI_BOSS = (3.1, 1.6)  # 나일론 링이 앉는 돋움의 반지름과 높이: 젠더와 판 나사 머리 사이를 띄운다
POST_SLOT = (3.6, 1.0)  # 덮개 기둥 수나사의 긴 구멍: 폭, x 쪽으로 더 긴 반길이
WINDOW = ((-12.0, 32.0), 17.0)  # 판 가운데의 창: x 범위, 옆 반폭 (바람이 통하고 선이 지난다)
# 찍찍이 타이 구멍 (가정: 타이 폭 12, 두께 1.3. 2026-11-24에 샀고 아직 재지 않았다): 긴 변마다 하나, 앞 변에 둘
TIE = (14.0, 3.0)
TIE_EDGE = 4.0  # 구멍과 판 가장자리 사이의 살
TIE_FRONT_V = (7.5, 20.5)  # 앞 변 구멍의 옆 범위 (SD 카드와 파이 돋움 사이)

# ---- 파이와 덮개 (자리만 그린다) ----
BOARD_T = 1.5  # 기판 두께 (가정: 도면에 없다)
RING_H = _CASE.pi_spacer_mm[0]  # 흰 나일론 링의 높이 3.17 (잰 값)
POST_BODY = _CASE.post_body_mm  # 덮개 기둥의 몸통 22.93 (잰 값)
POST_STUD = _CASE.post_stud_mm  # 수나사 길이 6.2 (잰 값)
COVER = (*_CASE.plate_mm, _CASE.plate_thickness_mm)  # 아크릴 판 97.84 × 72.52 × 2.86 (잰 값)
FAN = _CASE.fan_mm  # 30 × 30 × 7 (상품 이름의 값)
SPACER_R = _CASE.pi_spacer_mm[1] / 2  # 나일론 링의 바깥 반지름 (잰 값)
PI_SCREW = _CASE.pi_screw_mm[1]  # 파이를 다는 M2.5 접시머리 나사의 전체 길이 11.93 (잰 값)
# 기판 위 단자: (앞 끝에서 x0, x1, 전원 쪽 긴 변에서 y0, y1, 높이). 공식 도면에서 읽었다 (hardware.md)
PORTS = {
    "랜": (67.0, 88.0, 37.75, 53.75, 13.5),
    "USB 2.0": (70.0, 88.0, 20.0, 34.0, 16.0),
    "USB 3.0": (70.0, 88.0, 2.0, 16.0, 16.0),
    "USB-C 전원": (7.6, 14.8, -b.USBC_OUT, 6.3, 3.2),
    "GPIO": (7.0, 58.0, 50.0, 55.0, 8.5),
}
DONGLE_OUT = 30.0  # 게임패드 동글이 단자 밖으로 나온 길이 (가정: 재지 않았다)
UM2_OUT = 30.0  # 파이와 어댑터를 잇는 USB A–C 선의 플러그가 나온 길이 (가정: 재지 않았다)

# ---- 덮개 철망 (0038의 파이 덮개, 2026-11-24) ----
# 케이스의 금속 철망은 하나뿐이라 두 덮개에 같은 철망을 뽑는다. 금속 철망 자리 그대로 아크릴 판 위에 얹고, 팬 나사
# 넷이 철망, 판, 팬을 같이 묶는다. 매끈하게 볼록한 동심원이다: 고리와 살을 판 윗면에서 곧게 세우고 윗면을 한 구면으로
# 깎는다. 밑면이 모두 판 윗면 한 높이라 테를 바닥에 두고 서포트 없이 뽑고, 가운데는 나사 넷이 맞춘다.
# 팬은 판 가운데에 있다고 본다 (위 PROVISIONAL의 아크릴 판 가정과 같다)
FAN_HOLE_PITCH = _CASE.fan_hole_pitch_mm  # 팬 나사 구멍 가운데 사이 24.5 (잰 값)
FAN_OPENING = _CASE.fan_opening_mm  # 판의 둥근 팬 구멍 지름 28.12 (잰 값)
GRILLE_R = 13.75  # 고리 묶음의 바깥 반지름 (판의 팬 구멍 안 0.31)
GRILLE_BAR = 0.6  # 고리와 살의 굵기
GRILLE_HUB_R = 2.2  # 가운데 기둥
GRILLE_GAPS = 5  # 가운데 기둥에서 바깥 고리까지 고리 사이 틈의 수 (고리는 바깥 고리까지 다섯)
GRILLE_SPOKES = 8
GRILLE_RIM = (16.5, 2.0)  # 둥근 테의 반지름과 두께 (판 윗면에서)
GRILLE_EAR_R = 4.2  # 나사 귀의 반지름
GRILLE_DOME = 4.5  # 테 윗면에서 꼭대기까지
# 팬 나사 머리가 앉는 높이 (판 윗면에서). 가정: 금속 철망이 있던 자리라 그 두께만큼이면 지금 나사가 그대로 맞는다.
# 팬 나사 길이와 너트, 금속 철망 두께를 재서 정한다. 층 경계(0.2 mm의 배수)에 둔다
GRILLE_HEAD_SEAT = 1.0
GRILLE_HEAD_BORE = 3.1  # 머리 자리 반지름 (가정: 머리 지름 5.7 아래)
GRILLE_SCREW_R = 1.7  # 나사가 지나는 구멍 반지름

# ---- 서보 어댑터 (parts/base.py의 "서보 어댑터 자리": 몸통 바로 뒤 가운데의 뒤 판) ----
# 기판 위 부품의 자리: (u0, u1, v0, v1, 기판 위 높이). u는 서보 단자가 있는 긴 변을 따라가고(DC 잭 쪽 끝이 0),
# v는 그 변에서 전원 쪽 긴 변으로 간다. Waveshare 치수 그림(docs.waveshare.com/Bus_Servo_Adapter_A)에서 읽은
# 값이라 ±0.5 mm쯤이다. 높이는 DC 잭만 잰 값이고 나머지는 흔한 부품 값이다 (가정). 놓인 방향: u가 로봇 오른쪽(+z),
# v가 뒤(-x)다. 그래서 DC 잭이 왼쪽 뒤, USB-C가 오른쪽 뒤(컨트롤러 파이 쪽)에 온다
AD_ITEMS = {
    "서보 단자 1": (5.3, 15.8, 0.0, 5.0, 6.0),
    "서보 단자 2": (17.4, 28.0, 0.0, 5.0, 6.0),
    "UART 핀": (29.6, 37.5, 0.3, 2.3, 8.5),
    "점퍼": (33.2, 39.4, 10.8, 18.4, 8.5),
    "DC 잭": (4.9, 14.4, 19.0, 34.1, b.ADAPTER_JACK_TOP - b.ADAPTER_T),
    "나사 단자": (17.1, 24.9, 26.6, 33.7, 10.0),
    "USB-C 단자": (28.0, 37.2, 25.8, 33.8, 3.2),
}
# 서보 단자에 꽂는 줄 (0016의 두 줄. 두 단자는 기판에서 이어져 있어 바꿔 꽂아도 된다): 왼쪽 단자(1)에는 띠 밑
# 누르개에서 내려온 J3 줄(누르개가 몸통 뒤 왼쪽이다), 가운데 단자(2)에는 몸통 뒤 아래의 선 구멍에서 나온 J1 줄
assert _M.servo_plug is not None and _M.desk is not None
AD_PLUG_W = _M.servo_plug.width_mm  # 꽂은 플러그의 폭 (단자가 늘어선 쪽, 잰 값 9.01)
# 플러그 뒤 끝(선이 나오는 곳)의 높이 (기판 밑면에서, 잰 값). 이 높이에서 플러그 위의 고리가 시작한다
AD_PLUG_BACK = b.ADAPTER_PLUG_BACK
# 서보 선이 플러그에서 나와 가는 길 (2026-11-22, 꺾임이 첫 조건이다. 0031). 단자가 위를 봐서 선은 위로 나왔다가
# 내려와야 한다. 바짝 꺾지 않게, 플러그 위로 곧게 올린 뒤 넉넉한 반지름으로 앞(몸통 쪽)으로 반 바퀴 돌려 선 창으로
# 내린다. 서보 선은 납작해서(4.85 × 1.5) 앞뒤·위아래로만 굽는다: 길 전체가 한 평면(앞뒤-위아래)에 있어 비틀리지 않는다.
# 남는 선은 선 창 밑에서 고리로 늘어진다. 반지름은 선의 가운데 값이다 (안쪽 반지름은 0.75 작다)
AD_STRAIGHT = 6.0  # 플러그 위로 곧게 두는 길이 (기준 5 넘게)
# 플러그 위에서 반 바퀴 도는 고리 (안쪽 6.25: 기준 5 넘게). 내려가는 줄이 선 창 안(앞 변에서 4.5, 뒤 변에서 9.5)에 온다
AD_LOOP_R = 7.0
# 창 밑에서 다시 올라오는 줄의 x (창의 앞 변에서 1.5: 책상 뒷면과 2.5 뜬다)
AD_LEG_X = b.DECK_WINDOW[0][1] - 1.5
# 선 창 밑에 늘어진 고리의 반지름 (설계 값: 이만큼은 되게 선을 고른다. 남는 선만큼 더 커진다)
AD_BIGHT_R = 12.0
# J1 줄이 창에서 올라와 몸통의 선 구멍 쪽으로 눕는 반지름 (창의 앞 변을 0.4 넘게 비킨다)
AD_HOLE_R = 8.0
AD_HOLE_UP = 4.5  # 그 줄이 선 구멍으로 들어가는 높이 (뒤 판 바닥의 윗면에서. 구멍은 0 ~ 7)
AD_RISE = (20.0, 40.0)  # J3 줄이 창에서 올라와 몸통 뒷면으로 옮겨 가는 두 굽이의 반지름과 각 [°]
AD_CLAMP_RUN = 15.0  # 띠 밑 누르개 아래로 몸통에 붙어 곧게 서는 길이
AD_POWER_OUT = 20.0  # 나사 단자에서 뒤로 곧게 나온 전원선을 그린 길이
AD_Y = b.Y_FLOOR + b.ADAPTER_SEAT  # 어댑터 기판의 밑면
# 어댑터 덮개 (검정, 나사 없이 위에서 얹고 들어 뗀다): 떨어진 나사가 12 V 나사 단자에 걸치지 않게 하고 먼지를 막는다
# (0038의 파이 덮개와 같은 까닭). 지붕과 옆벽 둘, 뒷벽이다. 앞(몸통 쪽)은 벽이 없어 서보 선 둘이 지붕 밑으로 나가고,
# 뒷벽은 USB 플러그와 전원선 자리를 밑이 트인 홈으로 냈다. 그래서 선을 뽑지 않고 덮개만 들어 뗀다.
# 옆벽은 뒤 판의 옆 턱 밖에 앉아 옆 자리를 잡고, 앞 끝은 날개 뒷면에, 뒷벽은 뒤 판의 끝에 걸려 앞뒤 자리를 잡는다.
# 지붕은 선 창 위까지 덮는다 (창으로 내려간 선의 고리 윗부분이 덮개 속이다)
CAP_WALL = 2.0
CAP_ROOF = 2.0
# 덮개 천장의 높이 (책상에서). 플러그 뒤 끝을 15.3 mm로 어림해, 그 위 고리의 꼭대기(36.05)에서 2.55 mm 띄워 그렸다.
# 잰 값(12.63)은 더 낮아 고리와 천장 사이가 넓어지기만 해서, 덮개는 그린 높이 그대로 뽑는다 (2026-11-23).
# 천장은 책상에서 38.6이라 덮개를 뽑을 때 벽 끝이 층 경계다
_DRAWN_PLUG_BACK = 15.3
CAP_CEIL = AD_Y - b.Y_DESK + _DRAWN_PLUG_BACK + AD_STRAIGHT + AD_LOOP_R + j.WIRE_T / 2 + 2.55
CAP_SIDE_GAP = 0.3  # 옆벽과 옆 턱 사이
CAP_END_GAP = 0.4  # 앞 끝과 날개 뒷면, 뒷벽과 뒤 판 끝 사이
CAP_BACK_DROP = 3.0  # 뒷벽이 뒤 판 바닥의 윗면보다 내려온 길이 (바닥 두께 4 가운데 3에 걸린다)
CAP_NOTCH = (
    (-6.0, 21.0),
    18.0,
)  # 뒷벽의 홈: z 범위(전원선 ±3.9, USB 플러그 4.6 ~ 18.6), 윗변의 높이 (책상에서)

LABELS = {"pi_tray": "파이 판", "adapter_cover": "어댑터 덮개", "fan_grille": "파이 덮개 철망"}
PROVISIONAL = (
    "아크릴 판의 팬 구멍과 파이 고정 구멍의 자리 (판이 기판 가운데에 맞고 팬이 판 가운데에 있다고 봤다)",
    "철망에서 팬 나사 머리가 앉는 높이와 머리 자리 (GRILLE_HEAD_SEAT, GRILLE_HEAD_BORE: 팬 나사와 금속 철망을 재서 "
    "정한다)",
    "케이스의 M2.5 너트 (여덟 개가 있다고 봤다)",
    "기판 두께 BOARD_T, 게임패드 동글 DONGLE_OUT, USB A–C 플러그 UM2_OUT",
    "찍찍이 타이의 폭과 두께 (TIE)",
    "서보 어댑터의 부품 자리와 높이 (AD_ITEMS: 치수 그림에서 읽은 값. DC 잭 높이, 꽂은 서보 플러그의 윗면, 바짝 꺾은 서보 선의 꼭대기만 쟀다)",
)

Y_TRAY = b.Y_DESK + b.TRAY_UNDER  # 파이 판 밑면
Y_TOP = Y_TRAY + b.TRAY_T  # 파이 판 윗면
Y_BOARD = Y_TOP + PI_BOSS[1] + RING_H  # 기판 밑면
Y_COVER = Y_TOP + POST_BODY  # 아크릴 판 밑면


def post_uv() -> list[tuple[float, float]]:
    """덮개 기둥 넷 (기판 가운데에서 x, 옆)."""
    hx, hv = b.COVER_PITCH[0] / 2, b.COVER_PITCH[1] / 2
    return [(sx * hx, sv * hv) for sx in (1, -1) for sv in (1, -1)]


def screw_uv() -> list[tuple[float, float]]:
    """파이 판을 날개의 기둥에 다는 나사 넷 (기판 가운데에서 x, 옆)."""
    return [(sx * b.POST_DX, sv * b.POST_DZ) for sx in (1, -1) for sv in (1, -1)]


def tie_slots() -> list[tuple[float, float, float, float]]:
    """타이 구멍 넷: (x0, x1, 옆 0, 옆 1). 긴 변 둘의 가운데와 앞 변의 둘."""
    long_, wide = TIE
    v1 = b.TRAY_HZ - TIE_EDGE
    u1 = b.TRAY_HX - TIE_EDGE
    a, c = TIE_FRONT_V
    return [
        (-long_ / 2, long_ / 2, v1 - wide, v1),
        (-long_ / 2, long_ / 2, -v1, -v1 + wide),
        (u1 - wide, u1, a, c),
        (u1 - wide, u1, -c, -a),
    ]


def tray_local() -> Compound:
    """파이 판 (기판 가운데가 x = 0, z = 0인 자리로 그린 것). 밑면을 바닥에 두고 뽑는다.

    전원 쪽 긴 변(옆 +)이 -z다. 두 파이가 같은 방향이라 이 판 하나를 양쪽에 쓴다.
    """
    hx, hz = b.TRAY_HX, b.TRAY_HZ
    t = box(-hx, hx, Y_TRAY, Y_TOP, -hz, hz)
    (u0, u1), wv = WINDOW
    t = solid(t - box(u0, u1, Y_TRAY - 1, Y_TOP + 1, -wv, wv))
    r, h = PI_BOSS
    for u, v in PI_HOLES:
        t = solid(t + cyl_y(r, Y_TOP - 0.01, Y_TOP + h, u, -v))
        t = solid(t - cyl_y(PI_HOLE_D / 2, Y_TRAY - 1, Y_TOP + h + 1, u, -v))
    w, d = POST_SLOT
    for u, v in post_uv():
        slot = cyl_y(w / 2, Y_TRAY - 1, Y_TOP + 1, u - d, -v) + cyl_y(
            w / 2, Y_TRAY - 1, Y_TOP + 1, u + d, -v
        )
        slot += box(u - d, u + d, Y_TRAY - 1, Y_TOP + 1, -v - w / 2, -v + w / 2)
        t = solid(t - solid(slot))
    for u, v in screw_uv():
        t = solid(t - cyl_y(j.K_M3_FREE / 2, Y_TRAY - 1, Y_TOP + 1, u, -v))
    for a0, a1, v0, v1 in tie_slots():
        t = solid(t - box(a0, a1, Y_TRAY - 1, Y_TOP + 1, -v1, -v0))
    if not t.is_valid or len(t.solids()) != 1:
        raise RuntimeError("파이 판 형상이 깨졌다")
    return t


def grille_gap() -> float:
    """철망의 고리 사이 틈 [mm] (가운데 기둥과 첫 고리 사이도 같다)."""
    inner = GRILLE_R - GRILLE_BAR - GRILLE_HUB_R
    return (inner - (GRILLE_GAPS - 1) * GRILLE_BAR) / GRILLE_GAPS


@functools.cache
def fan_grille_local() -> Compound:
    """덮개 철망 (기판 가운데가 x = 0, z = 0인 자리, 아크릴 판 위). 테 밑면을 바닥에 두고 뽑는다."""
    y0 = Y_COVER + COVER[2]
    rim_r, rim_t = GRILLE_RIM
    rim_top = y0 + rim_t
    apex = rim_top + GRILLE_DOME
    # 윗면의 구: 바깥 고리 자리에서 테 윗면에 닿고 가운데가 꼭대기다
    rs = (GRILLE_R**2 + GRILLE_DOME**2) / (2 * GRILLE_DOME)
    tall = apex + 1.0
    g = j.ring(GRILLE_R, GRILLE_R - GRILLE_BAR, y0, tall)
    gap = grille_gap()
    r = GRILLE_HUB_R + gap
    for _ in range(GRILLE_GAPS - 1):
        g = solid(g + j.ring(r + GRILLE_BAR, r, y0, tall))
        r += GRILLE_BAR + gap
    g = solid(g + cyl_y(GRILLE_HUB_R, y0, tall))
    for k in range(GRILLE_SPOKES):
        spoke = box(GRILLE_HUB_R - 0.2, GRILLE_R - 0.1, y0, tall, -GRILLE_BAR / 2, GRILLE_BAR / 2)
        # 살은 나사 귀(대각선) 사이로 간다
        g = solid(g + Rot(0, (k + 0.5) * 360 / GRILLE_SPOKES, 0) * spoke)
    g = solid(g & (Pos(0, apex - rs, 0) * Sphere(rs)))
    rim = solid(cyl_y(rim_r, y0, rim_top) - cyl_y(GRILLE_R, y0 - 1, rim_top + 1))
    h = FAN_HOLE_PITCH / 2
    holes = [(sx * h, sz * h) for sx in (1, -1) for sz in (1, -1)]
    for x, z in holes:
        rim = solid(rim + cyl_y(GRILLE_EAR_R, y0, rim_top, x, z))
    g = solid(g + rim)
    for x, z in holes:
        g = solid(g - cyl_y(GRILLE_SCREW_R, y0 - 1, rim_top + 1, x, z))
        g = solid(g - cyl_y(GRILLE_HEAD_BORE, y0 + GRILLE_HEAD_SEAT, rim_top + 1, x, z))
    if not g.is_valid or len(g.solids()) != 1:
        raise RuntimeError("철망 형상이 깨졌다")
    return g


def at(side: int) -> Pos:
    """기판 가운데가 원점인 자리를 side 쪽 파이의 자리로 옮긴다 (side: +1 오른쪽, -1 왼쪽)."""
    return Pos(b.PI_CX, 0.0, side * b.PI_CZ)


def tray(side: int) -> Compound:
    return solid(at(side) * tray_local())


def _rbox(x0: float, x1: float, v0: float, v1: float, y0: float, y1: float) -> Compound:
    """기판 가운데에서 (x, 옆) 범위와 높이로 만든 상자 (옆 +가 -z)."""
    return box(x0, x1, y0, y1, -v1, -v0)


def pi_parts(side: int) -> list[j.Part]:
    """파이 한 대와 덮개, 꽂힌 플러그 (자리만 그린 상자와 원기둥). side 쪽 자리다.

    기판에서 전원 쪽 긴 변이 옆 +(로봇 왼쪽), USB·랜이 뒤다.
    """
    hx, hv = b.PI_BOARD[0] / 2, b.PI_BOARD[1] / 2
    top = Y_BOARD + BOARD_T
    who = "컨트롤러 파이" if side == CONTROLLER else "카메라 파이"
    out: list[tuple[str, Compound, str]] = [
        ("기판", _rbox(-hx, hx, -hv, hv, Y_BOARD, top), "servo")
    ]

    def port(key: str) -> tuple[float, float, float, float, float]:
        x0, x1, y0, y1, h = PORTS[key]
        return hx - x1, hx - x0, hv - y1, hv - y0, h

    for key in PORTS:
        x0, x1, v0, v1, h = port(key)
        out.append((key, _rbox(x0, x1, v0, v1, top, top + h), "metal"))
    # ㄱ자 젠더 (ML-CCSG3, 도면 값): 전원 단자에 꽂혀 옆으로 나오고, 소켓이 뒤를 본다. 전원 플러그가 거기서 뒤로 간다
    x0, x1, _v0, v1, h = port("USB-C 전원")
    xc, yc = (x0 + x1) / 2, top + h / 2
    g_len, g_deep, g_t = b.GENDER_BODY
    g0, g1 = v1, v1 + b.GENDER_OUT
    g_front = xc + b.GENDER_END  # 닫힌 끝 (앞쪽)
    out.append(
        ("젠더", _rbox(g_front - g_len, g_front, g0, g1, yc - g_t / 2, yc + g_t / 2), "plug")
    )
    pw = _PLUG["pi_power"]
    pv = g1 - g_deep / 2  # 소켓은 몸통 폭의 가운데다
    px1 = g_front - g_len
    out.append(
        (
            "전원 플러그",
            _rbox(
                px1 - pw.out_mm,
                px1,
                pv - pw.width_mm / 2,
                pv + pw.width_mm / 2,
                yc - pw.thickness_mm / 2,
                yc + pw.thickness_mm / 2,
            ),
            "plug",
        )
    )
    x0, _x1, v0, v1, h = port("랜")
    lan = _PLUG["lan"]
    vm = (v0 + v1) / 2
    out.append(
        (
            "랜 플러그",
            _rbox(
                x0 - lan.out_mm,
                x0,
                vm - lan.width_mm / 2,
                vm + lan.width_mm / 2,
                top + 1,
                top + 1 + lan.thickness_mm,
            ),
            "plug",
        )
    )
    usb = _PLUG["d405_usb_a"]
    if side == CAMERA:  # D405 선의 USB-A
        x0, _x1, v0, v1, h = port("USB 3.0")
        vm = (v0 + v1) / 2
        out.append(
            (
                "D405 USB 플러그",
                _rbox(
                    x0 - usb.out_mm,
                    x0,
                    vm - usb.width_mm / 2,
                    vm + usb.width_mm / 2,
                    top + 8.5,
                    top + 8.5 + usb.thickness_mm,
                ),
                "plug",
            )
        )
    else:  # 게임패드 동글과 서보 어댑터로 가는 USB 선 (둘 다 USB 2.0 단자: 가정한 크기)
        x0, _x1, v0, v1, h = port("USB 2.0")
        vm = (v0 + v1) / 2
        out.append(
            (
                "게임패드 동글",
                _rbox(x0 - DONGLE_OUT, x0, vm - 7.0, vm + 7.0, top + 8.5, top + 8.5 + 6.5),
                "plug",
            )
        )
        out.append(
            (
                "어댑터 USB 플러그",
                _rbox(x0 - UM2_OUT, x0, vm - 7.0, vm + 7.0, top + 1.0, top + 1.0 + 6.5),
                "plug",
            )
        )
    for k, (u, v) in enumerate(PI_HOLES, start=1):
        ring = cyl_y(SPACER_R, Y_TOP + PI_BOSS[1], Y_BOARD, u, -v)
        out.append((f"나일론 링 {k}", ring, "body"))
    for k, (u, v) in enumerate(post_uv(), start=1):
        out.append((f"덮개 기둥 {k}", cyl_y(3.0, Y_TOP, Y_COVER, u, -v), "metal"))
        stud = cyl_y(1.5, Y_TOP - POST_STUD, Y_TOP, u, -v) + j.hex_y(
            u, -v, Y_TRAY - j.NUT_T, Y_TRAY, 5.4
        )
        out.append((f"덮개 기둥 수나사와 너트 {k}", solid(stud), "metal"))
    cl, cw, ct = COVER
    out.append(("덮개 판", _rbox(-cl / 2, cl / 2, -cw / 2, cw / 2, Y_COVER, Y_COVER + ct), "cap"))
    fs, ft = FAN
    out.append(("팬", _rbox(-fs / 2, fs / 2, -fs / 2, fs / 2, Y_COVER - ft, Y_COVER), "servo"))
    out.append(("철망", fan_grille_local(), "cap"))
    for k, (u, v) in enumerate(screw_uv(), start=1):
        s = cyl_y(1.5, Y_TOP - 8.0, Y_TOP, u, -v) + cyl_y(2.85, Y_TOP, Y_TOP + 1.65, u, -v)
        out.append((f"파이 판 나사 M3x8 {k}", solid(s), "metal"))
    move = at(side)
    return [j.Part(f"{who}/{n}", solid(move * s), color=c) for n, s, c in out]


def _ad_box(u0: float, u1: float, v0: float, v1: float, h0: float, h1: float) -> Compound:
    """어댑터 기판의 (u, v) 범위와 기판 밑면에서의 높이로 만든 상자 (AD_ITEMS의 좌표)."""
    half = b.ADAPTER[0] / 2
    x = b.ADAPTER_EDGE_X
    return box(x - v1, x - v0, AD_Y + h0, AD_Y + h1, u0 - half, u1 - half)


def _ad_center(key: str) -> tuple[float, float]:
    """기판 위 부품의 가운데 (x, z)."""
    u0, u1, v0, v1, _h = AD_ITEMS[key]
    return b.ADAPTER_EDGE_X - (v0 + v1) / 2, (u0 + u1) / 2 - b.ADAPTER[0] / 2


@dataclass(frozen=True)
class Bend:
    """서보 선이 굽는 곳 하나: 어디서, 선 가운데의 반지름 [mm], 도는 각 [°]."""

    where: str
    radius: float
    angle: float


class _Trace:
    """앞뒤(x)-위아래(책상에서의 높이 h) 평면에서 곧은 구간과 원호를 이어 그리는 펜. 각은 +x에서 위쪽으로 잰다."""

    def __init__(self, x: float, h: float, heading_deg: float) -> None:
        self.x, self.h, self.heading = x, h, heading_deg
        self.points: list[tuple[float, float]] = [(x, h)]
        self.bends: list[Bend] = []

    def line(self, length: float) -> None:
        if length < -1e-9:
            raise ValueError(f"곧은 구간의 길이가 음수다: {length:.2f}")
        a = math.radians(self.heading)
        self.x, self.h = self.x + length * math.cos(a), self.h + length * math.sin(a)
        self.points.append((self.x, self.h))

    def arc(self, radius: float, turn_deg: float, where: str) -> None:
        """turn_deg만큼 돈다 (+가 반시계: 앞을 보고 가다 위로 굽는 쪽)."""
        side = 1.0 if turn_deg > 0 else -1.0
        a = math.radians(self.heading)
        cx, ch = self.x - side * radius * math.sin(a), self.h + side * radius * math.cos(a)
        steps = max(2, math.ceil(abs(turn_deg) / 10.0))
        for k in range(1, steps + 1):
            t = math.radians(self.heading + turn_deg * k / steps)
            self.points.append((cx + side * radius * math.sin(t), ch - side * radius * math.cos(t)))
        self.x, self.h = self.points[-1]
        self.heading += turn_deg
        self.bends.append(Bend(where, radius, abs(turn_deg)))


def _lead_start(key: str) -> _Trace:
    """플러그 뒤 끝에서 위로 곧게, 앞으로 반 바퀴, 선 창을 지나 내려가 창 밑의 고리를 돌아 다시 올라오는 줄까지.

    창 밑의 고리: 내려가는 줄과 올라오는 줄이 창 안에서 가까워서(AD_LEG_X까지 3 mm), 고리는 뒤(벽 쪽)로 배가 나온
    물방울 모양이다. 내려가는 줄이 뒤로 한 번 굽고(각 β), 큰 원을 180° + β 돌아 올라온다. 책상 뒷면이 앞을 막아
    앞으로는 나오지 않는다.
    """
    x, _z = _ad_center(key)
    t = _Trace(x, AD_Y - b.Y_DESK + AD_PLUG_BACK, 90.0)
    t.line(AD_STRAIGHT)
    t.arc(AD_LOOP_R, -180.0, "플러그 위의 고리 (앞으로 반 바퀴)")
    t.line(t.h)  # 뒤 판의 밑면(책상 윗면 높이)까지 곧게 내려간다
    beta = math.degrees(math.acos((AD_LEG_X - t.x) / (2 * AD_BIGHT_R)))
    t.arc(AD_BIGHT_R, -beta, "창 밑: 고리로 들어가며 뒤로 굽는 곳")
    t.arc(AD_BIGHT_R, 180.0 + beta, "창 밑의 고리")
    return t


def _lead_traces() -> dict[str, tuple[_Trace, float, float]]:
    """서보 선 둘의 길: 이름 → (펜, 플러그의 z, 끝의 z)."""
    out: dict[str, tuple[_Trace, float, float]] = {}
    # J1 줄: 가운데 단자(2) → 창 → 몸통 뒤 아래의 선 구멍 (구멍과 같은 z라 옆으로 가지 않는다)
    t = _lead_start("서보 단자 2")
    h_hole = b.PLATE_T + AD_HOLE_UP
    t.line(h_hole - AD_HOLE_R - t.h)
    t.arc(AD_HOLE_R, -90.0, "창에서 올라와 선 구멍 쪽으로 눕는 곳")
    t.line(-b.R_CORE - 1.0 - t.x)
    out["어댑터 → J1"] = (t, _ad_center("서보 단자 2")[1], 0.0)
    # J3 줄: 왼쪽 단자(1) → 창 → 몸통 뒷면을 따라 띠 밑의 누르개 (누르개가 몸통 뒤 왼쪽이라 왼쪽 단자가 가깝다)
    t = _lead_start("서보 단자 1")
    az = math.radians(b.WIRE_AZ)
    r_wire = b.R_CORE + j.WIRE_T / 2
    x_end, z_end = r_wire * math.cos(az), r_wire * math.sin(az)
    radius, turn = AD_RISE
    a = math.radians(turn)
    run = (x_end - t.x - 2 * radius * (1 - math.cos(a))) / math.sin(a)
    rise = 2 * radius * math.sin(a) + run * math.cos(a)
    h_clamp = b.FRONT - b.BAND_H - b.CLAMP_DROP - b.Y_DESK
    t.line(h_clamp - AD_CLAMP_RUN - rise - t.h)
    t.arc(radius, -turn, "창에서 올라와 몸통 쪽으로 기우는 곳")
    t.line(run)
    t.arc(radius, turn, "몸통 뒷면에 붙어 다시 서는 곳")
    t.line(AD_CLAMP_RUN)
    out["어댑터 → 띠 밑 누르개"] = (t, _ad_center("서보 단자 1")[1], z_end)
    return out


def servo_leads() -> dict[str, list[Point]]:
    """어댑터의 서보 단자에서 몸통까지 가는 서보 선 둘의 길 (플러그 뒤 끝부터, 선 가운데의 꺾은선. 원호는 10°씩 끊었다).

    선은 플러그 위로 곧게 올라와 앞으로 반 바퀴 돌고, 플러그 바로 앞의 선 창(parts/base.py의 DECK_WINDOW)으로 내려가
    책상 뒤에서 고리로 늘어졌다가 창으로 다시 올라온다. J1 줄은 거기서 몸통 뒤 아래의 선 구멍으로 눕고, J3 줄은 몸통
    뒷면으로 옮겨 가 띠 밑의 누르개로 올라간다. 창 밑의 고리는 설계 반지름(AD_BIGHT_R)으로 그렸다: 고른 선의 남는
    길이만큼 더 크게 늘어진다 (studies/wiring.py가 길이를 센다). J3 줄은 창에서 누르개까지 가며 옆으로 9 mm 옮겨 간다.
    """
    out: dict[str, list[Point]] = {}
    for name, (t, z0, z1) in _lead_traces().items():
        # 옆으로 옮겨 가는 것은 창에서 다시 올라온 뒤다: 거기서 끝까지 간 길이에 고르게 나눈다
        start = next(k for k, (x, h) in enumerate(t.points) if abs(x - AD_LEG_X) < 1e-6 and k > 2)
        run = [0.0]
        for p, q in itertools.pairwise(t.points[start:]):
            run.append(run[-1] + math.dist(p, q))
        pts: list[Point] = []
        for k, (x, h) in enumerate(t.points):
            f = 0.0 if k <= start else run[k - start] / run[-1]
            pts.append((x, b.Y_DESK + h, z0 + (z1 - z0) * f))
        out[name] = pts
    return out


def lead_bends() -> dict[str, list[Bend]]:
    """서보 선 둘이 굽는 곳 (플러그에서 몸통 쪽으로 차례대로)."""
    return {name: t.bends for name, (t, _z0, _z1) in _lead_traces().items()}


def loop_top() -> float:
    """플러그 위 고리의 꼭대기 (선의 바깥 면, 책상에서) [mm]."""
    return AD_Y - b.Y_DESK + AD_PLUG_BACK + AD_STRAIGHT + AD_LOOP_R + j.WIRE_T / 2


def cover_extent() -> tuple[float, float, float, float]:
    """어댑터 덮개의 겉 크기: 뒤 끝 x, 앞 끝 x, 반폭, 윗면의 높이 (책상에서) [mm]."""
    x_out = b.DECK_END - CAP_END_GAP - CAP_WALL
    hw_out = b.DECK_RIB[1] + CAP_SIDE_GAP + CAP_WALL
    return x_out, -b.EDGE - CAP_END_GAP, hw_out, CAP_CEIL + CAP_ROOF


def adapter_cover() -> Compound:
    """어댑터 덮개 (조립된 자리로 그린 것). 지붕을 바닥에 두고 뒤집어 뽑는다: 벽이 위로 서서 걸치는 곳이 없다."""
    y_floor = b.Y_FLOOR
    x_out, x_front, hw_out, top = cover_extent()
    y_ceil = b.Y_DESK + top - CAP_ROOF
    x_in = x_out + CAP_WALL
    hw_in = hw_out - CAP_WALL
    c = box(x_out, x_front, y_ceil - 0.01, y_ceil + CAP_ROOF, -hw_out, hw_out)
    for sg in (1, -1):
        z0, z1 = sorted((sg * hw_in, sg * hw_out))
        c += box(x_out, x_front, y_floor, y_ceil, z0, z1)
    c += box(x_out, x_in, y_floor - CAP_BACK_DROP, y_ceil, -hw_out, hw_out)
    (n0, n1), n_top = CAP_NOTCH
    c = solid(
        solid(c) - box(x_out - 1, x_in + 1, y_floor - CAP_BACK_DROP - 1, b.Y_DESK + n_top, n0, n1)
    )
    if not c.is_valid or len(c.solids()) != 1:
        raise RuntimeError("어댑터 덮개 형상이 깨졌다")
    return c


def adapter_parts() -> list[j.Part]:
    """서보 어댑터와 꽂힌 것들, 덮개: 몸통 바로 뒤 가운데의 뒤 판 위.

    서보 플러그는 위에서 꽂힌다 (뒤 끝의 높이는 가정이다). USB 플러그와 전원선은 전원 쪽 긴 변에서 뒤(벽 쪽)로
    나간다. 서보 선 둘은 servo_leads의 길을 선 두께(1.5 mm)의 둥근 줄로 그린다 (폭 4.85는 옆 쪽이다).
    """
    t = b.ADAPTER_T
    out: list[tuple[str, Compound, str]] = [
        ("", _ad_box(0.0, b.ADAPTER[0], 0.0, b.ADAPTER[1], 0.0, t), "servo")
    ]
    for key, (u0, u1, v0, v1, h) in AD_ITEMS.items():
        out.append((f"/{key}", _ad_box(u0, u1, v0, v1, t, t + h), "metal"))
    for k in (1, 2):  # 꽂은 서보 플러그 (선은 그 뒤 끝에서 위로 나온다)
        u0, u1, v0, v1, _h = AD_ITEMS[f"서보 단자 {k}"]
        uc = (u0 + u1) / 2
        plug = _ad_box(uc - AD_PLUG_W / 2, uc + AD_PLUG_W / 2, v0, v1, t + 0.01, AD_PLUG_BACK)
        out.append((f"/서보 플러그 {k}", plug, "plug"))
    u0, u1, _v0, v1, h = AD_ITEMS["USB-C 단자"]
    uc = (u0 + u1) / 2
    usb = _ad_box(uc - 7.0, uc + 7.0, v1, v1 + UM2_OUT, t + h / 2 - 3.25, t + h / 2 + 3.25)
    out.append(("/USB 플러그", usb, "plug"))
    u0, u1, _v0, v1, _h = AD_ITEMS["나사 단자"]
    out.append(("/전원선 자리", _ad_box(u0, u1, v1, v1 + AD_POWER_OUT, t + 2.0, t + 6.0), "plug"))
    for k, (x, z) in enumerate(b.adapter_xz(), start=1):
        out.append((f"/M2 나사 {k}", cyl_y(1.9, AD_Y + t, AD_Y + t + 1.5, x, z), "metal"))
    for name, pts in servo_leads().items():
        r = j.WIRE_T / 2
        line = rod(pts[0], pts[1], r)
        for p, q in itertools.pairwise(pts[1:]):
            line = line + rod(p, q, r)
        out.append((f"/서보 선 ({name})", solid(line), "wire"))
    parts = [j.Part(f"서보 어댑터{n}", s, color=c) for n, s, c in out]
    return [*parts, j.Part("어댑터 덮개", adapter_cover(), color="frame")]


def parts() -> list[j.Part]:
    """파이 판 둘과 파이 두 대, 덮개, 플러그, 서보 어댑터와 그 덮개 (베이스에 서 있는 것들)."""
    out = [
        j.Part("파이 판 (오른쪽)", tray(1), color="frame"),
        j.Part("파이 판 (왼쪽)", tray(-1), color="frame"),
    ]
    return out + pi_parts(1) + pi_parts(-1) + adapter_parts()


def clearances(limit: float = 6.0) -> dict[tuple[str, str], float]:
    """파이 쪽 것들, 서보 어댑터와 베이스 몸통·책상에 무는 판·클램프 사이의 틈 [mm] (limit 아래만. 앉은 것은 닿는다)."""
    mine = [
        p for p in parts() if "서보 선 (" not in p.name
    ]  # 서보 선은 몸통 뒷면을 따라간다 (닿는다)
    fixed = [
        j.Part("베이스 몸통", b.core()),
        j.Part("책상에 무는 판", b.plate()),
        *[p for p in b.desk_parts() if p.name.startswith("클램프")],
    ]
    out: dict[tuple[str, str], float] = {}
    for a in mine:
        for other in fixed:
            if far_apart(a.shape, other.shape, limit):
                continue
            d = a.shape.distance_to(other.shape)
            if d < limit:
                out[(a.name, other.name)] = d
    return out


def stack() -> dict[str, float]:
    """높이 쌓임 (책상에서) [mm]와 여유."""
    desk = b.Y_DESK
    top = Y_BOARD + BOARD_T
    jaw_top = b.WING_T + b.JAW_T
    return {
        "클램프 위 턱의 윗면": jaw_top,
        "서보 어댑터 기판 밑면": AD_Y - desk,
        "서보 선을 바짝 꺾었을 때의 꼭대기 (잰 값)": AD_Y + b.ADAPTER_PLUG_TOP - desk,
        "플러그 위 고리의 꼭대기 (곧게 6, 반지름 7로 돌린 선)": loop_top(),
        "어댑터 덮개 윗면": cover_extent()[3],
        "파이 판 밑면": Y_TRAY - desk,
        "파이 기판 밑면": Y_BOARD - desk,
        "USB 단자 윗면": top + 16.0 - desk,
        "팬 밑면": Y_COVER - FAN[1] - desk,
        "아크릴 판 밑면": Y_COVER - desk,
        "아크릴 판 윗면": Y_COVER + COVER[2] - desk,
        "철망 꼭대기": Y_COVER + COVER[2] + GRILLE_RIM[1] + GRILLE_DOME - desk,
        "띠 밑 (선의 고리가 생기는 높이)": b.FRONT - b.BAND_H - desk,
        "파이 나사 M2.5 × 12가 판 밑으로 나온 길이 (너트 2.0)": PI_SCREW
        - (BOARD_T + RING_H + PI_BOSS[1] + b.TRAY_T),
        "덮개 기둥 수나사가 판 밑으로 나온 길이 (너트 2.4)": POST_STUD - b.TRAY_T,
        "젠더 밑과 파이 판 나사 머리 사이": (top + 1.6 - b.GENDER_BODY[2] / 2) - (Y_TOP + 1.65),
    }


def cover_gaps() -> dict[str, float]:
    """어댑터 덮개와 그 속의 것들(기판, 부품, 꽂힌 플러그, 선) 사이의 틈 [mm]."""
    made = adapter_parts()
    cover = made[-1].shape
    return {p.name: cover.distance_to(p.shape) for p in made[:-1]}


def adapter_room() -> dict[str, float]:
    """서보 어댑터 둘레의 자리 [mm]: 플러그를 잡는 자리, 덮개 속 여유, 벽과 클램프까지."""
    assert _M.desk is not None
    wall = -b.EDGE - _M.desk.back_gap_mm
    usb_end = b.ADAPTER_EDGE_X - AD_ITEMS["USB-C 단자"][3] - UM2_OUT
    jaw_in = b.CLAMP_Z - b.JAW_HW
    return {
        "서보 플러그 앞: 몸통 뒷면까지 (선 창과 선이 나가는 바닥)": -b.R_CORE - b.ADAPTER_EDGE_X,
        "서보 플러그 뒤: DC 잭까지 (그 사이는 낮은 부품뿐이다)": AD_ITEMS["DC 잭"][2]
        - AD_ITEMS["서보 단자 1"][3],
        "DC 잭 꼭대기에서 플러그 뒤 끝까지 (잭 위로 잡는 높이)": AD_PLUG_BACK - b.ADAPTER_JACK_TOP,
        "기판 윗면이 옆 턱 윗면보다 낮은 만큼 (플러그 몸통은 턱 위로 올라온다)": (
            b.Y_FLOOR + b.RING_H
        )
        - (AD_Y + b.ADAPTER_T),
        "플러그 위 고리의 꼭대기와 덮개 천장 사이": CAP_CEIL - loop_top(),
        "창 밑에서 올라오는 줄과 책상 뒷면 사이": -b.EDGE - AD_LEG_X - j.WIRE_T / 2,
        "USB 플러그 끝(가정한 길이)에서 벽까지": usb_end - wall,
        "뒤 판과 클램프 등 사이 (클램프를 뒤에서 끼우는 길)": jaw_in - b.DECK_HW,
    }


def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str, int]]:
    return {
        "pi_tray": (tray_local(), (0.0, 1.0, 0.0), "black", "strength", 2),
        # 프리셋을 책상에 무는 판, 파이 판과 같게 해서 한 판에 같이 뽑는다
        "adapter_cover": (adapter_cover(), (0.0, -1.0, 0.0), "black", "strength", 1),
        # 두 덮개에 하나씩. 테 밑면(판에 닿는 면)을 바닥에 둔다
        "fan_grille": (fan_grille_local(), (0.0, 1.0, 0.0), "black", "strength", 2),
    }


def report() -> None:
    print(f"가정한 값: {'; '.join(PROVISIONAL)}")
    for name, (shape, up, color, preset, count) in printed().items():
        bb = shape.bounding_box()
        over = overhangs(shape, up)
        print(
            f"  {name} × {count} {color} {bb.size.X:.1f} × {bb.size.Y:.1f} × {bb.size.Z:.1f} mm  "
            f"{printed_mass_g(shape, up, preset):.1f} g  걸침 {over.total:.0f} mm²"
        )
        print(f"  층 가운데에 걸친 수평면: {mid_layer_levels(shape, up) or '없음'}")
    print("\n".join(air_wall_lines({k: v[:4] for k, v in printed().items()})))
    print("높이 (책상에서)")
    for name, value in stack().items():
        print(f"  {name}: {value:.2f}")
    print("서보 어댑터 둘레의 자리 [mm]")
    for name, value in adapter_room().items():
        print(f"  {name}: {value:.2f}")
    print("서보 선이 굽는 곳 (플러그에서 몸통 쪽으로. 반지름은 선 가운데, 안쪽은 0.75 작다)")
    for name, bends in lead_bends().items():
        total = sum(v.angle for v in bends)
        print(f"  {name}: 플러그 위로 곧게 {AD_STRAIGHT:.0f} mm, 굽는 각의 합 {total:.0f}°")
        for v in bends:
            print(f"    {v.where}: 반지름 {v.radius:.1f} mm, {v.angle:.0f}°")
    print("어댑터 덮개와 속의 것 사이 틈 [mm] (좁은 것부터 다섯)")
    for name, d in sorted(cover_gaps().items(), key=lambda kv: kv[1])[:5]:
        print(f"  {name}: {d:.2f}")
    print("파이 쪽 것들과 몸통·판·클램프 사이 틈 [mm] (6 아래)")
    for (a, other), d in sorted(clearances().items(), key=lambda kv: kv[1]):
        print(f"  {a} ↔ {other}: {d:.2f}")


if __name__ == "__main__":
    report()
