"""J1 관절: 베이스 몸통과 돌림판, 그 위에 앉는 J1 몸통의 발 (0028의 J1, 0029의 "J1 관절").

STS 관절(parts/joint.py)을 세워 놓고 베어링을 둘로 늘린 짜임이다. 서보, 누름 틀, 서보 받침은 STS 관절과 같다.
다른 점:
- 6806 둘을 가운데 사이 30 mm로 띄운다. 돌림판의 저널 하나가 안쪽 링 둘을 지나서, 두 링이 한 원기둥에 앉는다.
- 허브는 혼에 짧은 나사 넷으로 먼저 달고(책상 위에서), 돌림판에서 내려온 긴 볼트 넷(가진 M3 × 45)을 허브 옆
  홈의 너트에 조인다. 볼트는 돌림판 단 - 위 안쪽 링 - 사이 고리 - 아래 안쪽 링 - 허브를 한 덩어리로 문다. 저널
  끝은 허브에 닿지 않는다. 혼의 얇은 나사산(2.5 mm)에 긴 볼트를 걸지 않고, 볼트 길이가 혼 높이에 매이지 않는다.
  볼트 머리와 와셔는 돌림판 윗면에서 5.6 mm 내려간 자리에 앉는다 (위 안쪽 링의 윗면보다 2.4 mm 위).
- 바깥 링은 위쪽이 몸통의 턱, 아래쪽이 누름 틀이다. 누름 틀을 밀어 붙인 채 벽 나사를 잠근다.
- 몸통은 두꺼운 원기둥이다 (무게가 문제 되지 않는 자리라 베어링 둘레를 두껍게 했다). 벽 나사는 굴 속에 앉는다.
- 기계 멈춤 기둥은 돌림판 밑면에, 홈과 살은 몸통 윗면에 있다. 영점 핀은 J1 몸통의 뒤쪽 귀에서 그 살에 꽂는다.
- J1 몸통은 돌림판에 축 가까이(반지름 21 mm)의 나사 넷으로 단다. 나사는 돌림판 밑에서 올라와 J1 몸통의 두꺼운
  바닥판 속 너트에 물린다 (책상 위에서 먼저 조립한다). 돌림판의 얇은 테가 팔의 기울이는 힘에 휘지 않게 하려는 것이다.
  너트는 J1 몸통을 뽑는 방향(어깨 원통 앞 끝이 바닥)에서 위로 열린 홈에 떨어뜨려 넣는다 (흰색 fit1_nut_drops 값).
  어깨 뒷덮개 쪽의 둘은 바닥판의 뒤 가장자리에서, 위팔 쪽의 둘은 바닥판 가운데 구멍 속에서 넣는다.
- 베어링 구멍은 두 베어링이 앉는 자리만 시험편 값(42.2)이고, 그 아래와 사이는 0.2 넓다. 위 베어링은 아래 자리(6.6 mm)만
  눌러 지나가면 사이를 그냥 내려간다.
- 선은 몸통 윗부분의 띠(높이 16 mm, 반지름 40.5 mm)에 감긴다 (0031). 몸통 쪽 누르개와 J1 몸통 뒤쪽 귀 위의
  누르개가 범위 가운데에서 같은 방위각(뒤에서 왼쪽으로 28°)에 온다.

좌표(mm)는 joint.py의 서보 좌표다: y가 J1 축(위), x는 서보의 가까운 끝 쪽(로봇 앞), z = x × y (로봇 오른쪽).
서보 먼 끝(-x)이 로봇 뒤(책상 가장자리 쪽)를 본다. 책상 윗면은 y = Y_DESK다.
J1 몸통은 어깨 집(parts/shoulder.py)에 발을 단 한 부품이고, 어깨의 서보 좌표로 그린 뒤 as_child로 옮긴다.

책상에 무는 판(날개)은 몸통 밑에 깔리고, 책상 뒤 가장자리에서 금속 클램프 둘로 문다. 책상 두께(40 mm)와 가장자리에서
J1 축까지(60 mm)는 문서의 값이다(0014, 0018). 금속 클램프는 다 벌린 틈과 위 턱을 쟀고 나머지 치수는 사진으로
어림했다 (hardware.md).

    uv run python -m horibot_design.parts.base
"""

from __future__ import annotations

import math

from build123d import Compound, Cone, Location, Plane, Pos, Rot

from horibot_design.parts import joint as j
from horibot_design.parts import shoulder as sh
from horibot_design.parts.shapes import (
    Point,
    box,
    cyl_x,
    cyl_y,
    cyl_z,
    far_apart,
    nut_trap,
    rod,
    solid,
    teardrop,
)
from horibot_design.printcheck import (
    LAYER,
    air_wall_lines,
    mid_layer_levels,
    overhangs,
    printed_mass_g,
)
from horibot_design.values import ServoModel

# ---- 축 방향 쌓임 (혼 윗면 j.SEAT에서 위로) ----
LABEL_GAP = 1.2  # 허브 밑면과 서보 앞면 라벨 판 사이에 둘 틈
# 허브 밑면을 혼 바깥에서 들어 올리는 높이. 혼이 라벨 판보다 LABEL_GAP 넘게 높으면 0이다 (혼 높이 값에 달렸다)
HUB_RELIEF = max(0.0, LABEL_GAP - (j.SEAT - j.LABEL - 0.17))
HUB_BODY = 6.0  # 혼 바깥에서의 허브 두께: 너트 위 살 1.6 + 옆 홈 2.8 + 밑 살 1.6
HUB_T = HUB_BODY + HUB_RELIEF  # 혼 윗면 ~ 아래 베어링 밑면
SPAN = 30.0  # 두 베어링의 가운데 사이 (0028)
SPACER = SPAN - j.BRG_B  # 사이 고리의 길이 23
B0 = j.SEAT + HUB_T  # 아래 베어링 밑면 = 허브 윗면
B1 = B0 + j.BRG_B + SPACER  # 위 베어링 밑면
FRONT = B1 + j.BRG_B + j.LIP_T  # 몸통 윗면 (뽑을 때 바닥)
SEAM = FRONT + j.GAP  # 돌림판 테의 밑면
WASHER_T = (
    0.5  # 조임 볼트 머리 밑의 평와셔 (M3 × 8 × 0.5): 머리가 PLA를 누르는 넓이를 2.7배로 넓힌다
)
# 돌림판 테. J1 몸통 나사의 머리 자리(1.8) 위로 살 2.4가 남는다. 두꺼워지면 돌림판 윗면이 올라가 어깨 서보
# 밑의 자리(5.0)가 준다
BRIM_T = 4.2
ATOP = SEAM + BRIM_T  # 돌림판 윗면 = J1 몸통이 앉는 면
JOURNAL_GAP = 0.6  # 저널 끝과 허브 사이. 베어링 폭과 출력 오차를 다 더해도 닿지 않는다
PLATE_T = 4.0  # 몸통 밑에 깔리는 판의 바닥 두께
Y_FLOOR = j.WIRE_TOP - j.COVER_GAP  # 몸통 아래 끝 = 판 윗면. 꽂아 눕힌 선과 1.0 띄운다
Y_DESK = Y_FLOOR - PLATE_T  # 책상 윗면
TOP_H = ATOP - Y_DESK  # 돌림판 윗면의 높이 (책상에서)
SHOULDER_H = 160.0  # 어깨 축 높이 (0017의 골격)
NECK_X = -(SHOULDER_H - TOP_H)  # 어깨 좌표에서 J1 몸통의 발 바닥 (목의 아래 끝)

# ---- 반지름 ----
R_CORE = 40.5  # 몸통 바깥. 서보 먼 끝 모서리(37.2) 밖으로 벽 2.4가 남는다
R_TABLE = 39.0  # 돌림판 테와 J1 몸통 발의 바깥 = 선이 감기는 반지름
R_CLAMP = 11.6  # 조임 볼트 넷 (저널 벽 1.75가 남고, 볼트가 혼의 바깥(반지름 9.98)을 지난다)
R_HORN = 7.0  # 혼 나사 넷 (혼의 M3 나사 구멍)
HEAD_POCKET = j.HEAD_H + 0.6  # 허브 밑면의 혼 가운데 나사 머리 자리 (머리는 혼 윗면 위로 2.0까지)
# 허브 윗면의 혼 나사 머리 자리 (머리가 0.05 나오고, 저널 끝과는 0.55 뜬다). 허브를 뽑을 때 자리 바닥이 층 경계다
HEAD_RECESS = 1.6
HORN_SCREW = 8.0  # 혼 나사 M3 × 8
HORN_FLANGE = 2.5  # 혼의 테 두께 (M3 나사 구멍이 뚫려 있다, docs/hardware.md)
NUT_ROOF = 1.6  # 허브 옆 홈의 너트 위에 남는 살
# 검정 부품(베이스 몸통, 허브)의 옆에서 밀어 넣는 너트 홈: 너트가 눕는다 (검정 fit1_nut_slots 값 5.9 × 2.8)
SLOT_W, SLOT_H = j.K_NUT_LYING
CLAMP_LEN = 45.0  # 조임 볼트: 가진 유두 렌치볼트 M3 × 45
CLAMP_HEAD_R, CLAMP_HEAD_H = 2.75, 3.0  # 그 머리 (지름 5.5, 높이 3.0, 2.5 mm 육각렌치)
# 조임 볼트의 머리 자리: 돌림판 윗면에서 SINK만큼 내려간 곳에 와셔가 앉는다. 볼트 끝이 허브의 너트를 1.1 지나고
# 허브 밑면보다 0.9 위에서 끝나는 깊이다 (4.5 ~ 6.5이면 너트에 다 물리고 끝이 허브 속에서 끝난다).
# 자리의 천장은 뽑을 때 허공에 걸친다 (0.2 mm 층으로 28층 위). 막 한 층(SKIN)이 볼트 구멍까지 덮어서 줄이
# 끊기지 않고 걸친다. 막은 조립 전에 볼트 구멍 자리만 머리 자리 쪽에서 뚫는다
SINK = 5.6
SINK_R = 4.3  # 머리 자리의 반지름: 와셔(지름 8)에 틈 0.3
SKIN = 0.2
# 머리 자리가 단 속으로 1.4 내려가므로, 그 둘레에 살 1.6이 남게 단을 넓힌다. 위 베어링 윗면에서 0.8 위부터라
# 베어링에 닿지 않고, 몸통의 턱 구멍(반지름 19.5)과는 2.0 뜬다
STEP_WIDE_R = 17.5
STEP_WIDE_UP = 0.8
SPOT_R = j.SPOT_R  # 벽 나사 머리가 앉는 면. STS 관절과 같아서 누름 틀과 서보 받침을 그대로 쓴다
SPACER_R = j.R_BORE - 0.5  # 사이 고리의 몸통: 저널이 들어오기 전에 구멍 가운데 가까이 있게 한다
JOURNAL_TIP = 0.8  # 저널 끝의 모따기 (사이 고리가 0.5 비켜 있어도 들어간다)
SPACER_BORE = j.BRG_D / 2 + 0.15
SEAT_IN = (
    j.SEAT_Y - j.B0
)  # 베어링 자리(좁은 구멍)가 베어링 밑면보다 늦게 시작하는 길이 0.4 (STS 관절과 같다)

# ---- 기계 멈춤, 영점 핀, J1 몸통 나사 ----
R_POST = 33.0  # 멈춤 기둥 (돌림판 밑면, 앞쪽)
POST_TIP = SEAM - 4.6  # 기둥 돋움의 끝 (돌림판을 뽑을 때 바닥에서 8.8). 그 아래 너트 2.4
CH_R = (R_POST - 4.4, R_POST + 4.4)  # 몸통 윗면의 홈
CH_Y = FRONT - 7.0  # 홈 바닥
WEB_HALF = 180.0 - j.STOP_DEG - math.degrees(math.asin(j.BOSS_R / R_POST))  # 살의 반각 11.3°
R_PIN = 33.0  # 영점 핀 (뒤쪽 살의 가운데)
# J1 몸통 나사 M3 × 12 넷 (x, z). 돌림판의 단(반지름 16.5)에서 4.5 mm 밖이라 테가 휘는 길이가 짧다.
# 가장자리(반지름 35)에 두면 테가 팔의 기울이는 힘(약 3 N·m)에 0.05° 넘게 휜다 (어림)
R_FOOT = 21.0
# 방위각: 오른쪽(어깨 뒷덮개 쪽, +z)의 둘은 45°와 135°, 왼쪽(위팔 쪽)의 둘은 240°와 300°다. 왼쪽 둘의 너트 홈은
# J1 몸통 바닥판의 가운데 구멍 속으로 열려야 해서, 홈(폭 5.6)이 구멍(반지름 16) 안에 다 들게 x를 ±10.5로 좁혔다.
# 넷이 반 바퀴 대칭이 아니라서 J1 몸통이 돌림판에 한 방향으로만 달린다
FOOT_DEG = (45.0, 135.0, 240.0, 300.0)
FOOT = tuple(j.polar(R_FOOT, a) for a in FOOT_DEG)
FOOT_SCREW = 12.0
FOOT_RECESS = 1.8  # 돌림판 밑면의 나사 머리 자리
# J1 몸통 바닥판의 너트 홈: 뽑을 때 위로 열려 너트를 떨어뜨려 넣는 홈이다 (볼트는 가로). 흰색은 너트가 서는 홈의
# 값이 이 방향(fit1_nut_drops: 폭 5.6, 두께 2.8)뿐이다. 너트 밑(돌림판 쪽)에 살 FOOT_SEAT, 위에 살 1.0이 남는다
FOOT_SEAT = 6.0
FOOT_NUT_SLOT = (
    j._fit(j._WHITE.m3_nut_drop_width_mm),
    j._fit(j._WHITE.m3_nut_drop_thickness_mm),
)
# J1 몸통의 바닥판 두께 (목 안쪽). 돌림판 밑에서 올라온 M3 × 12가 바닥판 속 너트에 다 물린다.
# 목 앞뒤로 나온 귀는 j.EAR_T(6.2)로 얇아서 영점 핀이 살 속에 4.6 들어간다
FLOOR_T = 9.8
POCKET_TOP = j.LABEL + 1.4  # 서보 먼 끝 자리의 천장
FAR_GAP = 1.2  # 서보 먼 끝(눕힌 선은 0.9 더 나온다)과 벽 사이
WIRE_EXIT = (
    6.0,
    7.0,
)  # 몸통 뒤 아래의 선 구멍: 반폭, 높이 (플러그 9.6 × 5.4가 지나간다). J1 서보의 선 둘이 여기로 나온다
FLARE = 1.3  # 돌림판 테의 아래 가장자리를 45°로 넓힌 턱: 감긴 선이 몸통과의 틈으로 빠지지 않는다
STRING_D = 1.0  # 돌림판 테 둘레의 스프링 줄 홈 깊이 (V 모양이라 뽑을 때 걸치지 않는다)
STRING_Y = SEAM + 2.5  # 줄 홈의 가운데 높이
M2_TAP = 1.7  # 줄을 묶는 M2 나사를 조여 넣는 구멍 (임시: 시험편 값이 없다)
FOOT_HOLE = 16.0  # J1 몸통 바닥판 가운데 구멍의 반지름 (조임 볼트와 와셔를 넣고 렌치가 드나든다)
STRING_DEG = 10.0  # 줄 끝을 묶는 나사의 방위각 (앞쪽. 멈춤 기둥을 비킨다)

# ---- 선 (0031) ----
# J1을 넘는 선(서보 선 둘, D405 선)은 몸통 윗부분의 띠에 감긴다. 띠는 서 있는 몸통의 둥근 면이라, J1 몸통의
# 모서리(축에서 47.5 mm까지 나온다)와 상관없이 선이 한 원에 감긴다. 멈춤 기둥은 돌림판 밑에 숨어 있어 선과 만나지 않는다.
# 선은 몸통 쪽 누르개(방위각 WIRE_AZ)와 J1 몸통 뒤쪽 귀 위의 누르개 사이에서 감겼다 풀린다. 범위 가운데에서 두
# 누르개가 같은 방위각에 온다
BAND_H = 16.0  # 띠의 높이 (몸통 윗면에서 아래로)
BAND_LIP = 1.5  # 띠 위아래의 턱 (45°): 감긴 선이 띠 밖으로 넘어가지 않는다
WIRE_AZ = 208.0  # 선이 띠로 올라오는 방위각 (뒤에서 왼쪽으로 28°)
BAND_NOTCH = 8.0  # 아래 턱을 튼 자리의 반폭 (선이 밑에서 띠로 올라온다)
CLAMP_DROP = 6.0  # 아래 턱에서 몸통 쪽 누르개 나사 구멍까지
CLAMP_DEPTH = 8.0  # 누르개 나사(M3 × 8)를 조여 넣는 구멍의 깊이
EAR_CLAMP = (-31.5, -17.0)  # J1 몸통 뒤쪽 귀의 누르개 나사 자리 (x, z). 방위각이 WIRE_AZ다
TABLE_LIP = (
    38.6,
    39.8,
)  # 돌림판 밑의 테두리 (반지름 범위): 몸통 윗면과의 틈을 joint.LIP_GAP으로 좁힌다

# ---- 책상에 무는 판 ----
EDGE = 60.0  # 책상 뒤 가장자리에서 J1 축까지 (0018의 검토에 쓴 값)
DESK_T = 40.0  # 책상 상판 두께 (0014)
DESK_W, DESK_D = (
    550.0,
    340.0,
)  # 책상 가로(로봇을 무는 긴 변)와 앞뒤 (0014). 로봇은 긴 변의 가운데에 있다고 본다 (가정)
SEAT_GAP = 0.2  # 몸통과 판의 자리 사이
RING_H = 6.0  # 몸통 둘레의 턱과 날개가 바닥 위로 올라온 높이 (날개 두께는 바닥까지 10)
RING_W = 5.5  # 턱의 폭
WING_IN = 30.0  # 날개가 시작하는 |z| (그 사이는 선이 나가는 바닥이다)
WING_X = -15.0  # 날개의 두꺼운 띠(클램프가 무는 띠)의 앞 끝
WING_T = PLATE_T + RING_H  # 날개의 두께 (책상에서 윗면까지 10)

# ---- 파이 자리 (날개 위, 몸통 양옆. 2026-11-22 사용자 결정) ----
# 두 파이는 같은 방향으로 놓는다: USB·랜 단자가 뒤(-x, 책상 뒤 가장자리 쪽), 전원 단자(USB-C)가 로봇 왼쪽을 보는
# 긴 변에 있다 (hardware.md의 라즈베리파이 4). 그래서 왼쪽 파이는 전원이 바깥을, 오른쪽 파이는 몸통을 본다.
# 전원은 ㄱ자 USB-C 젠더로 꺾어 파이 가장자리를 따라 뒤로 보낸다 (두 파이 모두). 겉모양은 좌우 대칭이다.
_CASE = j._MEASURED.pi_case
assert _CASE is not None
PI_BOARD = (85.0, 56.0)  # 기판 (x 쪽, 옆 쪽). 공식 도면
PI_PORTS = 3.0  # USB·랜 단자가 기판 뒤 끝 밖으로 나온 길이 (도면의 축척으로 어림, hardware.md)
USBC_OUT = 1.2  # USB-C 단자가 기판 긴 변 밖으로 나온 길이 (가정: 도면에 치수가 없다)
# ㄱ자 USB-C 젠더: 마하링크 ML-CCSG3 (판매 페이지의 도면 값이다. 몸통 쪽 틈 BODY_GAP이 3 mm라 받아도 재지 않는다, 2026-11-22).
# 몸통은 단자 면에 수직으로 12.30, 꺾인 쪽으로 20.0(18.00에 소켓 쪽 끝의 TPE 2.00), 두께 6.70이다. 플러그(6.75 ± 0.2)는
# 두께 가운데에서 나오고, 플러그가 나오는 면에 TPE 깃이 약 1.35 있다. 플러그 가운데는 닫힌 끝에서 약 5.2, 소켓 면에서
# 약 14.8이다 (깃과 플러그 자리는 치수가 없어 도면의 비례로 읽은 값). 소켓은 뒤(책상 가장자리 쪽)를 본다.
# 단자 면에서 나온 길이 GENDER_OUT = 몸통 12.30 + 깃 1.35 + 단자 밖에 남는 플러그 약 0.5. 파이 자리가 이 값을 따라간다
GENDER_BODY = (
    20.0,
    12.3,
    6.7,
)  # 꺾인 쪽 길이 (파이 가장자리와 나란하다), 단자 면에 수직인 길이, 두께
GENDER_COLLAR = 1.35
GENDER_PLUG_LEFT = 0.5
GENDER_END = 5.2  # 플러그 가운데에서 닫힌 끝까지
GENDER_OUT = GENDER_BODY[1] + GENDER_COLLAR + GENDER_PLUG_LEFT  # 14.15
BODY_GAP = 3.0  # 오른쪽 파이의 젠더와 베이스 몸통 사이에 둘 틈
PI_IN = R_CORE + BODY_GAP + GENDER_OUT + USBC_OUT  # 기판 안쪽 긴 변의 |z| (58.85)
# 기판 앞 끝의 x. 파이 덮개 기둥(기판 가운데에서 ±44.1)이 책상 뒤 가장자리 안에 선다
PI_FRONT = 30.5
PI_CX = PI_FRONT - PI_BOARD[0] / 2  # 기판 가운데의 x (-12.0)
PI_CZ = PI_IN + PI_BOARD[1] / 2  # 기판 가운데의 |z| (86.85)
COVER_PITCH = _CASE.hole_pitch_mm  # 덮개(아크릴 판) 귀퉁이 기둥의 간격 88.2 × 62.7 (잰 값)
TRAY_HX = (
    COVER_PITCH[0] / 2 + 3.9
)  # 파이 판(작은 판)의 반길이 48.0: 덮개 기둥의 육각 몸통을 다 받친다
TRAY_HZ = COVER_PITCH[1] / 2 + 4.15  # 반폭 35.5
TRAY_T = 3.0
# 날개 위의 기둥 넷이 파이 판을 받친다 (판 나사 M3 × 8을 기둥에 조여 넣는다). 기둥은 기판 긴 변의 밖, 덮개 기둥과
# 같은 줄에 선다: 파이를 단 채로 위에서 드라이버가 닿는다. 기둥의 무늬가 기판 가운데에 대해 앞뒤·좌우 대칭이라
# 두 파이 판이 같은 부품이고 두 날개가 거울상이다
POST_DX, POST_DZ = 30.0, COVER_PITCH[1] / 2
POST_R = 4.0
# 파이 판 밑면의 높이 (책상에서). 그 밑에 클램프의 위 턱(윗면 17.93)이 든다. 서보 어댑터는 이 밑에서 몸통 뒤로
# 옮겼지만(아래 "서보 어댑터 자리") 높이는 그대로 둔다: 낮추면 위 턱을 뒤에서 끼우고 빼는 틈(파이 판 밑으로 나온
# 너트까지 5.9)이 준다
TRAY_UNDER = 27.0
POST_TAP = 10.0  # 기둥 속 조여 넣는 구멍의 깊이
WING_Z = PI_CZ + TRAY_HZ  # 판의 반폭 (122.35)
PAD_X = PI_CX + POST_DX + POST_R + 1.0  # 앞쪽 기둥을 받치는 얇은 바닥의 앞 끝 (17.0)
PAD_IN = R_CORE + SEAT_GAP + RING_W + 4.3  # 그 바닥이 시작하는 |z| (몸통 둘레의 턱 밖)
# 클램프가 무는 자리: 파이 판 밑, 안쪽 기둥 줄(|z| 55.5)의 바깥이다 (가장자리에서 35 mm까지 비운 폭 20 mm의 띠).
# 파이 밑에서 물려 위 턱이 파이 판에 덮인다. 몸통에서 멀어진 만큼(뒤쪽 판 나사에서 49 mm) 날개가 더 휘므로,
# 몸통과 클램프 사이의 날개를 STIFF_H만큼 더 두껍게 한다 (wing_bend)
CLAMP_Z = 70.0
CLAMP_HW = 10.0
CLAMP_IN = 25.0  # 책상 가장자리에서 띠의 가운데까지 (design.md: 2 ~ 3 cm 안쪽)
THICK_OUT = CLAMP_Z + 12.0  # 두께 WING_T인 띠의 바깥 끝. 그 밖은 얇은 바닥(PLATE_T)이다
STIFF_OUT = 62.0  # 더 두꺼운 띠의 바깥 끝 (클램프 위 턱 65.1과 3 mm 띄운다)
STIFF_H = 6.0  # 그 띠가 날개 윗면보다 높은 만큼 (책상에서 16)
# ---- 서보 어댑터 자리 (몸통 바로 뒤 가운데, 2026-11-22. docs/decisions/0038-base-layout.md) ----
# Waveshare Bus Servo Adapter (A): 기판 42 × 33, 구멍 37 × 28 (지름 2.5). 서보 단자 둘은 한 긴 변에서 위로 꽂고,
# DC 잭·나사 단자·USB-C는 맞은편 긴 변에 있다 (Waveshare 치수 그림. 부품 자리는 parts/pi_mount.py).
# 꽂은 서보 플러그와 위로 나와 꺾이는 선이 기판 밑면에서 23.3 mm까지 올라오고, 꽉 끼는 플러그를 잡고 흔들어 빼려면
# 그 위와 앞뒤가 열려 있어야 한다. 그래서 파이 밑(파이 판까지 14 mm)에서 꺼내, 두 날개 사이에서 책상 뒤 가장자리
# 밖으로 낸 판(뒤 판)에 눕힌다. 서보 단자가 있는 긴 변이 몸통을, 전원·USB 쪽 긴 변이 벽을 본다.
# - 견준 안: 파이 판을 13 mm 올려 그대로 파이 밑에 (플러그 위가 막혀 파이를 떼야 뽑는다. 덮개 윗면이 띠 밑에 닿는다),
#   SO-101처럼 세워 달기 (받침 부품이 하나 늘고, 플러그가 벽 쪽을 봐서 보지 않고 뽑는다. 꽂는 힘이 받침을 층을 떼는
#   쪽으로 민다), 날개 바깥 옆면 (판 폭 244.7에 자리가 없고 한쪽에만 달린다. J1 선이 300 mm 선으로 는다).
# - 고른 까닭: 위가 열려 플러그를 위로 뽑고 나사 단자를 위에서 조인다. 몸통 뒤 아래의 선 구멍과 띠 밑 누르개가 바로
#   옆이다. 좌우 가운데라 대칭이 그대로이고 새 부품은 덮개 하나다. 덮개 윗면(40.6)이 파이 덮개(55.8)보다 낮다
# - 기판의 방향: 서보 단자의 변이 몸통을 본다. 서보 선은 납작해서 단자가 늘어선 쪽으로는 굽지 않으므로, 이 방향이어야
#   플러그 위의 고리, 선 창, 몸통의 선 구멍과 띠 밑 누르개가 한 평면에 놓여 선이 비틀리지 않는다. 변이 옆을 보면
#   (90° 돌리면) 몸통으로 가는 길에서 선을 90° 비틀어야 하고, 벽을 보면(180°) USB 플러그가 몸통을 본다.
#   눕힌 기판에서는 선이 위로 나와 반 바퀴 돌아야 한다 (parts/pi_mount.py의 AD_LOOP_R: 곧게 6, 반지름 7)
ADAPTER = (42.0, 33.0)
ADAPTER_HOLES = (37.0, 28.0)
ADAPTER_T = 1.6  # 기판 두께 (가정: 재지 않았다)
ADAPTER_JACK_TOP = 12.69  # DC 잭 꼭대기 (기판 밑면에서, 2026-11-22 캘리퍼스)
ADAPTER_PLUG_TOP = (
    23.3  # 꽂은 서보 플러그와 위로 나와 꺾이는 선의 꼭대기 (기판 밑면에서, 2026-11-22 캘리퍼스)
)
# 꽂은 서보 플러그의 윗면(선이 나오는 면) (기판 밑면에서, 2026-11-23 캘리퍼스, 작업 안내 "검정 판 3 전 재기")
ADAPTER_PLUG_BACK = 12.63
ADAPTER_TALL = 11.0  # DC 잭 꼭대기를 기판 위로 어림한 값 (잰 값은 11.09). 재기 안내의 그림(guides/arm_measure.py)이 쓴다
ADAPTER_SEAT = (
    3.0  # 돋움의 높이 (뒤 판 바닥 위): 기판 밑으로 나온 납땜 다리(사진 어림 2 mm)를 띄운다
)
# 뒤 판: 책상에 무는 판의 바닥(PLATE_T)을 책상 뒤 가장자리 밖으로 늘린 것. 벽까지 151 mm다 (data/measured.toml의 desk)
# 선 창: 책상 가장자리 바로 밖에서 바닥을 뚫은 창. 남는 서보 선(J1 → J2 선 482, J3 줄 220, J1 줄 100 mm쯤)을 고리째
# 이 창으로 내려 책상 뒤로 늘어뜨린다. 서보 선은 납작해서 단자가 늘어선 쪽(옆)으로는 잘 굽지 않고 앞뒤·위아래로
# 굽는다. 창이 플러그 바로 앞이라 선이 굽기 쉬운 쪽으로만 굽어 내려가고, 고리가 제 무게로 아래에 머문다.
# 창의 길이 14는 플러그 위에서 반 바퀴 돈 줄(플러그 가운데에서 앞으로 14)과 다시 올라오는 줄이 3 mm 띄워 지나가는
# 길이다. 두 줄이 가까워 창 밑의 고리는 뒤로 배가 나온 물방울 모양이 된다 (되굽는 곳 83°, 반지름 12). 창을 17 mm
# 늘리면 되굽지 않는 반 바퀴가 되지만 뒤 판이 70 mm 나오고 USB 플러그 끝에서 벽까지가 54 mm로 준다 (플러그 길이는
# 가정이다). 되굽는 곳도 반지름 12라 늘리지 않았다
# 창이 없으면 남는 선이 몸통 뒤에서 위로 서는 고리가 되어, J1이 돌 때 선이 감겼다 풀리는 띠(책상에서 72.8 ~ 88.8)에
# 닿을 수 있다 (0031: 꺾임과 끼임이 첫 조건이다. 선의 뻣뻣함에 맡기지 않는다). 창만큼 뒤 판이 15 mm 길어진다
DECK_WINDOW = (
    (-75.0, -61.0),
    14.0,
)  # 선 창의 x 범위, 반폭 (서보 선 셋이 고리로 드나들고 플러그 9.6 × 5.4도 지나간다)
# 찍찍이 타이 구멍 둘 (파이 판의 구멍과 같은 14 × 3): 선 창의 양옆. 창과 구멍 사이의 살(폭 4)에 타이를 감아, 창으로
# 내려가는 선을 묶는다. 서 있는 선만 묶고 남는 선은 그 밑으로 느슨한 고리로 둔다 (접어 꺾지 않는다)
DECK_TIE = (18.0, 21.0)  # 타이 구멍의 |z| 범위 (x 범위는 선 창과 같다)
# 앞 턱: 선 창의 뒤 변을 따라 기판 윗면 높이까지 올라와, 떨어진 나사가 기판 밑(납땜 다리)으로 굴러들지 못한다.
# 두께, 기판 변과의 틈
DECK_SILL = (1.2, 0.8)
# 서보 단자가 있는 긴 변의 x (-77.0). 그 앞은 선 창과 선이 나가는 바닥이라 몸통 뒷면(-40.5)까지 36.5 mm가 비고,
# 손가락이나 롱노즈가 플러그의 넓은 면(앞뒤)을 잡는다. 플러그 뒤쪽(기판 위)은 DC 잭까지 14 mm가 낮은 부품뿐이다
ADAPTER_EDGE_X = DECK_WINDOW[0][0] - sum(DECK_SILL)
ADAPTER_CX = ADAPTER_EDGE_X - ADAPTER[1] / 2  # 어댑터 가운데의 x (-93.5). 옆으로는 가운데(z = 0)다
DECK_END = (
    ADAPTER_EDGE_X - ADAPTER[1] - 3.0
)  # 뒤 끝 (-113.0). DC 잭이 변 밖으로 1 mm쯤 나오고 덮개 뒷벽이 그 뒤다
DECK_HW = 32.5  # 반폭: 옆 턱 밖으로 덮개 옆벽이 앉는 2.5 mm가 남는다
# 옆 턱 둘 (|z| 범위): 덮개의 옆 자리를 잡고 뒤 판을 굳힌다 (선 창 옆은 바닥이 좁아 턱이 굽힘을 받는다). 높이는 날개의
# 더 두꺼운 띠(책상에서 16)와 같고, 앞으로는 그 띠의 안쪽 면을 따라 DECK_RIB_FRONT까지 들어와 띠와 한 몸이다
# (뒤 판의 뿌리가 책상 위에서 잡힌다)
DECK_RIB = (27.6, 30.0)
DECK_RIB_FRONT = -48.0
M2_BOSS_R = (
    2.5  # 구멍(1.7) 둘레 살 1.65. 뒤쪽 돋움이 기판 변 밖으로 나오지 않아 USB 플러그 밑을 비킨다
)
# 가진 금속 클램프 (다 벌린 틈과 위 턱은 캘리퍼스, 나머지는 사진으로 어림한 값, hardware.md). 등을 책상 가장자리에
# 붙이면 위 턱이 가장자리에서 JAW_REACH까지 날개를 누르고, 나사가 그 밑(JAW_AXIS)에서 받침으로 책상 밑면을 민다
JAW_REACH = 28.67  # 위 턱이 등 안쪽 면에서 나온 길이 (2026-11-21 캘리퍼스)
JAW_AXIS = 22.0  # 등 안쪽 면에서 나사 축까지
JAW_HW = 4.88  # 각재의 반폭 (위 턱의 폭 9.76, 2026-11-21 캘리퍼스)
JAW_T, SPINE_T = 7.93, 9.0  # 위 턱의 두께 (2026-11-21 캘리퍼스), 등과 아래 턱의 두께
PAD_R, PAD_T = 20.0, 10.7  # 나사 끝 받침의 반지름, 받침의 높이 (나사를 다 풀었을 때 아래 턱 위로)
CLAMP_OPEN = 55.88  # 금속 클램프를 다 벌린 틈 (2026-11-13 캘리퍼스, hardware.md)
R_PLATE = (
    33.0  # 몸통을 판에 묶는 나사 넷 (45° 대각선: 서보 먼 끝 자리와 선 구멍, 벽 나사 굴을 비킨다)
)
PLATE_SCREW = 10.0  # M3 × 10
PLATE_RECESS = 2.2  # 판 밑면의 머리 자리: 둥근머리 1.65 + 와셔 0.5 (머리가 밑면보다 0.05 안에 있다)
PLATE_NUT_UP = 4.2  # 몸통 밑면에서 너트 홈까지의 살
PLATE_HOLE = 8.8  # 몸통 밑면에서 나사 구멍의 깊이 (나사 끝 7.7에 1.1 여유. 몸통을 뽑을 때 구멍 끝이 층 경계다)
# 부속 자리 (스프링 기둥, 선 정리 부품): 날개마다 M3 둘. 더 두꺼운 띠의 뒤쪽, 파이 판과 클램프 밖이다
ACC_X = -52.0
ACC_Z = (35.0, 47.0)
ACC_NUT = 2.6  # 그 밑면의 너트 자리 깊이

# ---- 쉬는 자세의 선반을 다는 자리 (0037): J1 몸통의 목 앞벽(로봇 앞) 안쪽에 낸 너트 둘 ----
# 선반의 뿌리 판이 앞 귀 윗면에 앉고 목 앞벽에 붙는다. M3 × 12 둘이 벽을 지나 목 속 살의 너트에 물린다.
# 너트는 J1 몸통을 뽑는 방향에서 위로 열린 홈(뒷덮개가 입구를 가린다)에 떨어뜨리고, 자리의 마지막 SHELF_GRIP은
# 흰색 fit1_nut_drops의 t2.6 칸(폭 5.7 × 두께 2.6: 살짝 밀어야 내려가고 뒤집어도 안 나온다)이라 선반을 떼어도
# 너트가 제자리에 남는다. 아래 너트는 서보 먼 끝을 잡는 턱 속, 위 너트는 새로 낸 살 속이다
SHELF_NUT_UP = (117.5, 136.0)  # 너트 둘의 높이 (책상에서)
SHELF_NUT_Y = -2.0  # 너트의 옆 자리 (로봇 좌표 y: 왼쪽이 +)
SHELF_WALL = 3.4  # 너트가 기대는 벽 (목 앞벽 2.6 + 살 0.8)
SHELF_SEAT = (5.7, 2.6)  # 너트 자리: 폭, 두께
SHELF_GRIP = 6.1  # 꼭 끼는 자리의 길이. 그 끝이 J1 몸통을 뽑을 때 층 경계에 온다 (바닥에서 43.8)
SHELF_SKIN = 1.6  # 홈 안쪽에 남는 살
SHELF_SCREW = 12.0
# ---- J1 몸통 왼쪽 면의 누르개 나사 자리 (0031) ----
# 바닥판의 왼쪽 면(어깨 좌표 y = FRONT, J1 몸통을 뽑을 때 바닥)에 조여 넣는 구멍. 바닥판 높이의 아래쪽에 두어,
# 누르개의 팔이 그 위를 지나는 선 둘(바닥판 윗부분과 그 위의 벽)을 누른다. 뒤 왼쪽 모서리와 선 창 사이다
STRIP_SCREW_UP = 97.9  # 나사 구멍의 높이 (책상에서. 바닥판은 95.0 ~ 104.8)
STRIP_SCREW_X = -14.0  # 나사 구멍의 앞뒤 자리 (로봇 좌표 x)
STRIP_TAP = 9.0

PROVISIONAL = (
    "ㄱ자 USB-C 젠더 ML-CCSG3의 크기 (GENDER_*: 판매 페이지의 도면 값이고 깃과 플러그 자리는 도면의 비례다. 몸통 쪽 틈이 3 mm라 재지 않는다)",
    "서보 어댑터의 기판 두께 (ADAPTER_T)와 기판 밑 납땜 다리 (사진 어림 2 mm), 단자 자리 (치수 그림에서 읽어 ±0.5 mm쯤)",
    "USB-C 단자가 파이 기판 밖으로 나온 길이 (USBC_OUT)",
    "M2_TAP (스프링 줄과 서보 어댑터의 M2 나사를 조여 넣는 구멍. 시험편 값이 없다)",
)


def clamp_xz() -> list[tuple[float, float]]:
    """조임 볼트 넷."""
    return [j.polar(R_CLAMP, a) for a in (0, 90, 180, 270)]


# ------------------------------------------------------------------------------------------------
# 베이스 몸통 (검정)


def _tunnel(axis: str, sign: int, y: float, half_y: float, half_w: float) -> Compound:
    """벽 나사 머리가 앉는 면에서 몸통 밖까지 뚫은 네모 굴 (나사와 렌치가 드나든다)."""
    lo, hi = SPOT_R, R_CORE + 1
    if axis == "x":
        return box(lo, hi, y - half_y, y + half_y, -half_w, half_w)
    z0, z1 = (lo, hi) if sign > 0 else (-hi, -lo)
    return box(-half_w, half_w, y - half_y, y + half_y, z0, z1)


def core() -> Compound:
    """베이스 몸통. 윗면(y = FRONT)을 바닥에 두고 뽑는다. 속은 아래(뽑을 때 위)로 열려 있다."""
    h = solid(Rot(0, 45, 0) * cyl_y(R_CORE, Y_FLOOR, FRONT))
    # 베어링 구멍: 두 베어링이 앉는 자리(각 6.6 mm)만 R_BORE이고, 그 아래와 사이는 0.2 넓다 (R_ENTRY). 자리의
    # 끝은 몸통을 뽑을 때 층 경계에 온다 (바닥에서 1.8, 8.4, 31.8, 38.4)
    cav = cyl_y(j.R_ENTRY, Y_FLOOR - 1, B0 + SEAT_IN + 0.01)
    cav += cyl_y(j.R_BORE, B0 + SEAT_IN, B0 + j.BRG_B)
    cav += cyl_y(j.R_ENTRY, B0 + j.BRG_B - 0.01, B1 + SEAT_IN + 0.01)
    cav += cyl_y(j.R_BORE, B1 + SEAT_IN, B1 + j.BRG_B)
    cav += cyl_y(j.R_LIP, B1, FRONT + 1)
    cav += box(-(j.FAR + FAR_GAP), -12.0, Y_FLOOR - 1, POCKET_TOP, -j.NECK_IN, j.NECK_IN)
    h = solid(h - solid(cav))
    hw, ht = WIRE_EXIT
    h = solid(h - box(-R_CORE - 1, -j.FAR, Y_FLOOR - 1, Y_FLOOR + ht, -hw, hw))
    # 멈춤 홈: 윗면을 한 바퀴 돌고, 뒤쪽(180°)에 살만 남긴다
    band = j.ring(CH_R[1], CH_R[0], CH_Y, FRONT + 1)
    web = band & j.wedge(180 - WEB_HALF, 180 + WEB_HALF, CH_R[1] + 5, CH_Y - 1, FRONT + 2)
    h = solid(h - solid(band - web))
    h = solid(h - cyl_y(j.K_PIN_HOLE / 2, CH_Y - 2.0, FRONT + 1, -R_PIN, 0.0))
    # 벽 나사: 누름 틀 셋 (양옆과 앞), 서보 받침 둘 (양옆, 축 방향으로 긴 구멍)
    r = j.K_M3_FREE_H / 2
    for s in (1, -1):
        z0, z1 = (15.0, SPOT_R + 0.01) if s > 0 else (-SPOT_R - 0.01, -15.0)
        h = solid(h - cyl_z(r, z0, z1, 0.0, j.RET_SCREW_Y))
        h = solid(h - _tunnel("z", s, j.RET_SCREW_Y, 3.4, 3.4))
        slot = cyl_z(r, z0, z1, 0.0, j.CAR_SCREW_Y - 1.0) + cyl_z(
            r, z0, z1, 0.0, j.CAR_SCREW_Y + 1.0
        )
        slot += box(-r, r, j.CAR_SCREW_Y - 1.0, j.CAR_SCREW_Y + 1.0, z0, z1)
        h = solid(h - solid(slot))
        h = solid(h - _tunnel("z", s, j.CAR_SCREW_Y, 5.2, 4.2))
    h = solid(h - cyl_x(r, 15.0, SPOT_R + 0.01, j.RET_SCREW_Y, 0.0))
    h = solid(h - _tunnel("x", 1, j.RET_SCREW_Y, 3.4, 3.4))
    # 책상에 무는 판의 나사 넷: 밑에서 올라와 옆에서 밀어 넣은 너트에 물린다. 조인 너트가 붙는 홈의 면(몸통을 뽑을
    # 때 홈의 천장)은 첫 층이 나사 구멍까지 덮는 막이다. 구멍이 뚫린 천장은 구멍을 둘러싼 벽 고리가 공중에 뽑힌다.
    # 판을 달기 전에 밑면 쪽에서 구멍 자리의 막을 뚫는다
    y_nut = Y_FLOOR + PLATE_NUT_UP + SLOT_H / 2
    for a, (x, z) in zip((45.0, 135.0, 225.0, 315.0), plate_xz(), strict=True):
        h = solid(h - cyl_y(j.K_M3_FREE / 2, Y_FLOOR - 1, Y_FLOOR + PLATE_NUT_UP - SKIN, x, z))
        h = solid(h - cyl_y(j.K_M3_FREE / 2, Y_FLOOR + PLATE_NUT_UP, Y_FLOOR + PLATE_HOLE, x, z))
        out = (math.cos(math.radians(a)), 0.0, math.sin(math.radians(a)))
        trap = nut_trap((x, y_nut, z), (0.0, 1.0, 0.0), out, R_CORE - R_PLATE + 1, SLOT_W, SLOT_H)
        h = solid(h - trap)
    h = solid(h + _band_lips())
    # 몸통 쪽 누르개 나사 구멍 (아래 턱 밑, 옆에서 조여 넣는다)
    y_clamp = FRONT - BAND_H - CLAMP_DROP
    hole = cyl_x(j.M3_TAP / 2, R_CORE - CLAMP_DEPTH, R_CORE + 1, y_clamp, 0.0)
    h = solid(h - j.turned(solid(hole), WIRE_AZ))
    if not h.is_valid or h.volume < 1000:
        raise RuntimeError(f"베이스 몸통 형상이 깨졌다: 부피 {h.volume:.1f}")
    return h


def _band_lips() -> Compound:
    """선이 감기는 띠의 위아래 턱. 위 턱은 몸통 윗면(뽑을 때 바닥)에서 가장 넓고, 아래 턱은 양쪽이 45°보다 조금 선다.

    아래 턱은 WIRE_AZ에서 터 있다 (선이 밑에서 띠로 올라오는 자리).
    """
    lip, c = BAND_LIP, 1.1 * BAND_LIP
    top = Pos(0, FRONT - c / 2, 0) * (Rot(-90, 0, 0) * Cone(R_CORE - 0.01, R_CORE + lip, c))
    y_rib = FRONT - BAND_H
    rib = Pos(0, y_rib - c / 2, 0) * (Rot(-90, 0, 0) * Cone(R_CORE - 0.01, R_CORE + lip, c))
    rib += Pos(0, y_rib + c / 2, 0) * (Rot(-90, 0, 0) * Cone(R_CORE + lip, R_CORE - 0.01, c))
    notch = box(
        R_CORE - 1.0, R_CORE + lip + 1, y_rib - c - 1, y_rib + c + 1, -BAND_NOTCH, BAND_NOTCH
    )
    rib = solid(solid(Rot(0, 45, 0) * solid(rib)) - j.turned(notch, WIRE_AZ))
    # 원뿔의 안쪽은 몸통과 겹친다. 몸통 밖으로 나온 고리만 남긴다
    lips = solid(solid(Rot(0, 45, 0) * solid(top)) + rib)
    return solid(lips - cyl_y(R_CORE - 0.5, Y_FLOOR - 1, FRONT + 1))


# ------------------------------------------------------------------------------------------------
# 책상에 무는 판 (검정): 몸통이 앉는 자리 + 책상 뒤 가장자리까지 나온 날개 둘


def plate_xz() -> list[tuple[float, float]]:
    """몸통을 판에 묶는 나사 넷."""
    return [j.polar(R_PLATE, a) for a in (45.0, 135.0, 225.0, 315.0)]


def mount_xz() -> list[tuple[float, float]]:
    """날개의 부속 자리 넷 (x, z)."""
    return [(ACC_X, s * z) for s in (1, -1) for z in ACC_Z]


def post_xz() -> list[tuple[float, float]]:
    """파이 판을 받치는 기둥 여덟 (x, z): 날개마다 넷."""
    return [
        (PI_CX + sx * POST_DX, s * (PI_CZ + sz * POST_DZ))
        for s in (1, -1)
        for sx in (1, -1)
        for sz in (1, -1)
    ]


def adapter_xz() -> list[tuple[float, float]]:
    """서보 어댑터의 돋움 넷 (x, z): 뒤 판 위 (어댑터의 고정 구멍 37 × 28, 긴 변이 옆 쪽)."""
    hx, hz = ADAPTER_HOLES[1] / 2, ADAPTER_HOLES[0] / 2
    return [(ADAPTER_CX + sx * hx, sz * hz) for sx in (1, -1) for sz in (1, -1)]


def _side(x0: float, x1: float, y0: float, y1: float, s0: float, s1: float, sg: int) -> Compound:
    """|z|가 s0 ~ s1인 상자를 sg 쪽에 (판은 z = 0에 대해 거울상이다)."""
    z0, z1 = sorted((sg * s0, sg * s1))
    return box(x0, x1, y0, y1, z0, z1)


def plate() -> Compound:
    """책상에 무는 판 (날개가 곧 파이 받침이다). 밑면(책상에 닿는 면)을 바닥에 두고 뽑는다.

    바닥(PLATE_T) 위로 몸통 둘레의 턱과 날개가 RING_H만큼 올라와 있다. 몸통은 턱 안에 앉아 자리를 잡고,
    밑에서 올라온 나사 넷이 몸통 속 너트에 물린다. 클램프는 파이 판 밑에서 날개 윗면을 책상 가장자리에서 문다.
    날개에서 클램프가 무는 띠까지는 두께 WING_T이고(몸통 쪽은 STIFF_H만큼 더 두껍다), 그 밖과 앞은 얇은 바닥이다.
    얇은 바닥 위로 파이 판을 받치는 기둥 넷이 선다. 두 날개 사이에서는 바닥이 책상 뒤 가장자리 밖으로 나와(뒤 판)
    서보 어댑터의 돋움 넷과 덮개의 자리를 잡는 턱을 받친다.
    """
    r_seat = R_CORE + SEAT_GAP
    r_out = r_seat + RING_W
    y0, y1, y2 = Y_DESK, Y_FLOOR, Y_FLOOR + RING_H
    y3 = y2 + STIFF_H
    y_tray = Y_DESK + TRAY_UNDER
    floor = cyl_y(r_out, y0, y1) + box(-EDGE, WING_X, y0, y1, -WING_Z, WING_Z)
    raised = j.ring(r_out, r_seat, y1 - 0.01, y2)
    for sg in (1, -1):
        floor += _side(WING_X - 0.01, PAD_X, y0, y1, PAD_IN, WING_Z, sg)
        wing = _side(-EDGE, WING_X, y1 - 0.01, y2, WING_IN, THICK_OUT, sg)
        wing += _side(-EDGE, WING_X, y2 - 0.01, y3, WING_IN, STIFF_OUT, sg)
        raised += solid(solid(wing) - cyl_y(r_seat, y1 - 1, y3 + 1))
    p = solid(solid(floor) + solid(raised))
    # 선이 나가는 길: 몸통 뒤 아래의 선 구멍 앞에서 턱을 튼다
    hw = WIRE_EXIT[0] + 1.0
    p = solid(p - box(-r_out - 1, -r_seat + 1, y1, y2 + 1, -hw, hw))
    for x, z in post_xz():  # 파이 판의 기둥: 밑동을 원뿔로 넓히고, 위에서 M3 × 8을 조여 넣는다
        post = cyl_y(POST_R, y1 - 0.01, y_tray, x, z)
        if abs(z) > THICK_OUT or x > WING_X:  # 얇은 바닥에 선 기둥
            foot = Pos(x, y1 + 1.5, z) * (Rot(-90, 0, 0) * Cone(POST_R + 2.5, POST_R, 3.0))
            # 판 밖으로 나가는 밑동은 잘라 낸다 (바닥 없이 뜨지 않게)
            foot = solid(foot) & box(-EDGE, PAD_X, y1 - 1, y1 + 4, -WING_Z, WING_Z)
            post = solid(post + solid(foot))
        p = solid(p + post)
        p = solid(p - cyl_y(j.M3_TAP / 2, y_tray - POST_TAP, y_tray + 1, x, z))
    # 뒤 판: 책상 뒤 가장자리 밖으로 나온 바닥, 옆 턱 둘, 앞 턱. 선 창과 타이 구멍 둘을 뚫는다
    deck = box(DECK_END, -EDGE + 0.01, y0, y1, -DECK_HW, DECK_HW)
    rib0, rib1 = DECK_RIB
    for sg in (1, -1):
        deck += _side(DECK_END, DECK_RIB_FRONT, y1 - 0.01, y3, rib0, rib1, sg)
    sill_t, sill_gap = DECK_SILL
    x_sill = ADAPTER_EDGE_X + sill_gap
    y_ad = y1 + ADAPTER_SEAT
    deck += box(x_sill, x_sill + sill_t, y1 - 0.01, y_ad + ADAPTER_T, -rib0 - 0.01, rib0 + 0.01)
    p = solid(p + solid(deck))
    (wx0, wx1), w_hw = DECK_WINDOW
    p = solid(p - box(wx0, wx1, y0 - 1, y1 + 1, -w_hw, w_hw))
    for sg in (1, -1):
        p = solid(p - _side(wx0, wx1, y0 - 1, y1 + 1, *DECK_TIE, sg))
    for (
        x,
        z,
    ) in adapter_xz():  # 서보 어댑터의 돋움: M2 나사를 조여 넣는다 (구멍 밑에 바닥 1 mm가 남는다)
        p = solid(p + cyl_y(M2_BOSS_R, y1 - 0.01, y_ad, x, z))
        p = solid(p - cyl_y(M2_TAP / 2, y_ad - 6.0, y_ad + 1, x, z))
    for x, z in plate_xz():  # 나사 구멍과 밑면의 머리 자리. 그 사이에 막 한 층이 남는다
        p = solid(p - cyl_y(j.K_M3_FREE / 2, y0 + PLATE_RECESS + SKIN, y1 + 1, x, z))
        p = solid(p - cyl_y(SINK_R, y0 - 1, y0 + PLATE_RECESS, x, z))
    for x, z in mount_xz():  # 부속 자리: 위에서 조이는 M3 나사와 밑면의 너트 자리
        p = solid(p - cyl_y(j.K_M3_FREE / 2, y0 + ACC_NUT + SKIN, y3 + 1, x, z))
        p = solid(p - j.hex_y(x, z, y0 - 1, y0 + ACC_NUT, j.K_NUT_AF))
    if not p.is_valid or len(p.solids()) != 1:
        raise RuntimeError("책상에 무는 판 형상이 깨졌다")
    return p


def wing_bend(force_n: float = 70.0, e_gpa: float = 2.5) -> dict[str, float]:
    """클램프가 누르는 힘에 날개가 휘는 양 [mm] (어림: 뒤쪽 판 나사에서 클램프까지 뻗은 외팔보).

    0029의 어림과 같은 식이다: 팔이 뒤로 뻗을 때 클램프 하나가 약 70 N을 버틴다. 두꺼운 띠(STIFF_OUT까지)와
    그 밖(클램프까지 두께 WING_T)을 이어 붙인 보로 셈한다. 견줄 값은 클램프가 몸통 바로 옆(|z| 50)에 있고 날개가
    한 두께(WING_T)이던 앞선 배치다.
    """
    e = e_gpa * 1e3
    width = WING_X + EDGE  # 날개의 앞뒤 폭 45
    x_c = -EDGE + JAW_AXIS
    px, pz = plate_xz()[1]  # 뒤쪽 판 나사 (-23.3, 23.3)

    def tip(z_clamp: float, thick_to: float, thick: float) -> float:
        # 뿌리에서 a까지 두께 thick, 그 밖은 WING_T인 외팔보의 끝 처짐
        length = math.hypot(x_c - px, z_clamp - pz)
        a = min(length, max(0.0, (thick_to - pz) / max(z_clamp - pz, 1e-9) * length))
        i1, i2 = width * thick**3 / 12, width * WING_T**3 / 12
        b = length - a
        # 끝에 힘 F: 굵은 구간의 끝 처짐과 기울기, 가는 구간의 처짐을 더한다
        d1 = force_n * a**2 * (3 * length - a) / (6 * e * i1)
        th1 = force_n * a * (2 * length - a) / (2 * e * i1)
        return d1 + th1 * b + force_n * b**3 / (3 * e * i2)

    return {
        "지금 (클램프 |z| 70, 몸통 쪽 띠를 두껍게)": tip(CLAMP_Z, STIFF_OUT, WING_T + STIFF_H),
        "두껍게 하지 않으면": tip(CLAMP_Z, 0.0, WING_T),
        "앞선 배치 (클램프 |z| 50, 한 두께)": tip(50.0, 0.0, WING_T),
    }


def deck_bend(force_n: float = 20.0, e_gpa: float = 2.5) -> dict[str, float]:
    """뒤 판(책상 밖으로 나온 판)이 플러그를 꽂고 빼는 힘에 휘는 양 [mm] (어림: 책상 가장자리에서 뻗은 외팔보).

    단면은 가장 약한 곳(선 창과 타이 구멍 옆: 바닥은 살과 양옆 띠만 남는다)의 것을 길이 전체에 쓴다. 꽉 끼는 서보
    플러그를 누르고 당기는 힘을 20 N쯤으로 본다. 가장 먼 것은 뒤쪽 긴 변의 나사 단자(드라이버로 누른다)다.
    """
    e = e_gpa * 1e3
    rib_w, rib_h = DECK_RIB[1] - DECK_RIB[0], RING_H + STIFF_H
    floor_w = 2 * (DECK_HW - DECK_TIE[1] + DECK_TIE[0] - DECK_WINDOW[1])

    def inertia(ribs: bool) -> float:
        a_floor, a_rib = floor_w * PLATE_T, 2 * rib_w * rib_h * ribs
        c_floor, c_rib = PLATE_T / 2, PLATE_T + rib_h / 2
        c = (a_floor * c_floor + a_rib * c_rib) / (a_floor + a_rib)
        return (
            floor_w * PLATE_T**3 / 12
            + a_floor * (c - c_floor) ** 2
            + ribs * 2 * rib_w * rib_h**3 / 12
            + a_rib * (c_rib - c) ** 2
        )

    def tip(arm: float, ribs: bool = True) -> float:
        return force_n * arm**3 / (3 * e * inertia(ribs))

    servo = -EDGE - (ADAPTER_EDGE_X - 2.5)  # 서보 단자의 가운데 (변에서 2.5)
    far = -EDGE - (ADAPTER_EDGE_X - ADAPTER[1])  # 전원 쪽 긴 변
    return {
        "서보 단자에서": tip(servo),
        "전원 쪽 긴 변에서": tip(far),
        "턱이 없으면 전원 쪽 긴 변에서": tip(far, ribs=False),
    }


def desk_parts() -> list[j.Part]:
    """책상과 금속 클램프 둘 (그림에만 쓴다). 클램프는 사진으로 어림한 치수다. 등을 책상 가장자리에 붙여 그린다."""
    top = Y_FLOOR + RING_H
    under = Y_DESK - DESK_T  # 책상 밑면
    desk = box(-EDGE, DESK_D - EDGE, under, Y_DESK, -DESK_W / 2, DESK_W / 2)
    out = [j.Part("책상", desk, color="desk")]
    arm_top = top - (CLAMP_OPEN + PAD_T)  # 아래 턱의 윗면
    x_back, x_tip, x_axis = -EDGE - SPINE_T, -EDGE + JAW_REACH, -EDGE + JAW_AXIS
    for k, s in enumerate((1, -1), start=1):
        z0, z1 = sorted((s * (CLAMP_Z - JAW_HW), s * (CLAMP_Z + JAW_HW)))
        jaw = box(x_back, x_tip, top, top + JAW_T, z0, z1)
        spine = box(x_back, -EDGE, arm_top - SPINE_T, top + JAW_T, z0, z1)
        low = box(x_back, x_tip, arm_top - SPINE_T, arm_top, z0, z1)
        screw = cyl_y(4.0, arm_top - SPINE_T - 45.0, under - 4.0, x_axis, s * CLAMP_Z)
        knob = cyl_y(9.0, arm_top - SPINE_T - 57.0, arm_top - SPINE_T - 45.0, x_axis, s * CLAMP_Z)
        pad = cyl_y(PAD_R, under - 4.0, under, x_axis, s * CLAMP_Z)
        shape = solid(jaw + spine + low + screw + knob + pad)
        out.append(j.Part(f"클램프 {k}", shape, color="metal"))
    return out


# ------------------------------------------------------------------------------------------------
# 돌림판, 허브, 사이 고리 (검정, 서보와 함께 돈다)


def bearings() -> tuple[Compound, Compound]:
    # 저널은 시험편으로 고른 지름이라 그린 값이 안지름 30보다 크다. 그린 값끼리 겹치지 않게 안쪽을 저널에 맞춘다
    r_in = max(j.BRG_D / 2, j.R_JOURNAL)
    lower = j.ring(j.BRG_OD / 2, r_in, B0, B0 + j.BRG_B)
    upper = j.ring(j.BRG_OD / 2, r_in, B1, B1 + j.BRG_B)
    return lower, upper


def hub() -> Compound:
    """허브: 혼 윗면에 M3 × 8 넷으로 달고, 넓은 윗면이 아래 안쪽 링을 받친다. 혼 쪽 면을 바닥에 두고 뽑는다.

    조임 볼트의 너트는 옆면에서 밀어 넣는다. 볼트 끝은 허브 속에서 끝나서 혼에 닿지 않는다.
    """
    h = solid(Rot(0, 22.5, 0) * cyl_y(j.R_STEP, j.SEAT, B0))
    if HUB_RELIEF > 0:  # 혼 바깥에서 밑면을 들어 라벨 판 위로 띄운다
        h = solid(h - j.ring(j.R_STEP + 1, j.HUB_SEAT_R, j.SEAT - 1, j.SEAT + HUB_RELIEF))
    h = solid(h - cyl_y(3.6, j.SEAT - 1, j.SEAT + HEAD_POCKET))  # 혼 가운데 나사 머리
    for x, z in j.bolt_xz():  # 혼 나사 (45°부터)
        h = solid(h - cyl_y(j.K_M3_FREE / 2, j.SEAT - 1, B0 + 1, x, z))
        h = solid(h - cyl_y(3.1, B0 - HEAD_RECESS, B0 + 1, x, z))
    # 조임 볼트의 너트 홈. 조인 너트가 붙는 홈의 천장(B0 - NUT_ROOF)은 첫 층이 볼트 구멍까지 덮는 막이다 (구멍이
    # 뚫린 천장은 구멍을 둘러싼 벽 고리가 공중에 뽑힌다). 허브를 혼에 달기 전에 윗면 쪽에서 구멍 자리의 막을 뚫는다
    y_nut = B0 - NUT_ROOF - SLOT_H / 2
    roof = B0 - NUT_ROOF
    for a, (x, z) in zip((0, 90, 180, 270), clamp_xz(), strict=True):
        h = solid(h - cyl_y(j.K_M3_FREE / 2, j.SEAT - 1, roof, x, z))
        h = solid(h - cyl_y(j.K_M3_FREE / 2, roof + SKIN, B0 + 1, x, z))
        out = (math.cos(math.radians(a)), 0.0, math.sin(math.radians(a)))
        trap = nut_trap((x, y_nut, z), (0.0, 1.0, 0.0), out, j.R_STEP - R_CLAMP + 1, SLOT_W, SLOT_H)
        h = solid(h - trap)
    return h


def spacer() -> Compound:
    """사이 고리: 두 안쪽 링 사이. 끝의 좁은 면(반지름 16.5까지)만 안쪽 링에 닿고, 몸통은 45°로 물러난다."""
    y0, y1 = B0 + j.BRG_B, B1
    c = 1.1 * (SPACER_R - j.R_STEP)  # 뽑을 때 45°보다 조금 서게
    s = cyl_y(j.R_STEP, y0, y1) + cyl_y(SPACER_R, y0 + c, y1 - c)
    lower = Pos(0, y0 + c / 2, 0) * (Rot(-90, 0, 0) * Cone(j.R_STEP, SPACER_R, c))
    upper = Pos(0, y1 - c / 2, 0) * (Rot(-90, 0, 0) * Cone(SPACER_R, j.R_STEP, c))
    s = solid(s + lower + upper)
    return solid(s - cyl_y(SPACER_BORE, y0 - 1, y1 + 1))


def turntable() -> Compound:
    """돌림판: 저널(안쪽 링 둘과 사이 고리 속) + 단(위 안쪽 링 윗면) + 테 + 멈춤 기둥. 테 윗면을 바닥에 두고 뽑는다.

    조임 볼트의 머리와 와셔는 테 윗면에서 SINK만큼 내려간 자리에 앉는다. 자리의 천장은 허공에 걸쳐 뽑히고,
    막 한 층이 볼트 구멍을 덮는다.
    """
    y_tip = B0 + JOURNAL_GAP
    a = cyl_y(j.R_JOURNAL, y_tip + JOURNAL_TIP, B1 + j.BRG_B + 0.01)
    a += Pos(0, y_tip + JOURNAL_TIP / 2, 0) * (
        Rot(-90, 0, 0) * Cone(j.R_JOURNAL - JOURNAL_TIP, j.R_JOURNAL, JOURNAL_TIP + 0.01)
    )
    a += cyl_y(j.R_STEP, B1 + j.BRG_B, SEAM + 0.01)
    a += cyl_y(STEP_WIDE_R, B1 + j.BRG_B + STEP_WIDE_UP, SEAM + 0.01)
    brim = cyl_y(R_TABLE, SEAM, ATOP) + Pos(0, SEAM + FLARE / 2, 0) * (
        Rot(-90, 0, 0) * Cone(R_TABLE + FLARE, R_TABLE, FLARE)
    )
    a = solid(a + solid(Rot(0, 45, 0) * solid(brim)))
    # 테 밑의 테두리: 몸통 윗면과의 틈을 좁혀, 띠에서 올라오는 선이 돌림판 밑으로 들어가지 않게 한다
    lip = j.ring(TABLE_LIP[1], TABLE_LIP[0], FRONT + j.LIP_GAP, SEAM + 0.01)
    a = solid(a + solid(Rot(0, 45, 0) * lip))
    a = solid(a + cyl_y(j.BOSS_R, POST_TIP, SEAM + 0.01, R_POST, 0.0))  # 멈춤 기둥 (속에 M3 나사)
    # 기둥 나사 머리 자리: 천장 첫 층이 구멍까지 덮는 막이다 (STS 관절의 출력 원판과 같다). 멈춤 나사를 꽂기 전에
    # 구멍 자리의 막을 뚫는다
    seat = ATOP - j.POST_HEAD_SINK
    a = solid(a - cyl_y(j.K_M3_FREE / 2, POST_TIP - 1, seat - LAYER, R_POST, 0.0))
    a = solid(a - cyl_y(j.HEAD_POCKET_R, seat, ATOP + 1, R_POST, 0.0))
    for x, z in clamp_xz():  # 볼트 구멍과 머리 자리. 그 사이에 막 한 층이 남는다
        a = solid(a - cyl_y(j.K_M3_FREE / 2, B0, ATOP - SINK - SKIN, x, z))
        a = solid(a - cyl_y(SINK_R, ATOP - SINK, ATOP + 1, x, z))
    for (
        x,
        z,
    ) in FOOT:  # J1 몸통 나사: 머리가 밑면의 자리에 앉는다 (뽑을 때 위라 자리 바닥이 평평하다)
        a = solid(a - cyl_y(j.K_M3_FREE / 2, SEAM - 1, ATOP + 1, x, z))
        a = solid(a - cyl_y(3.1, SEAM - 1, SEAM + FOOT_RECESS, x, z))
    a = solid(a - cyl_y(2.0, SEAM - 1, ATOP + 1, -R_PIN, 0.0))  # 영점 핀이 지나가는 구멍
    # 스프링 자리 (0011): 테 둘레의 줄 홈과, 줄 끝을 묶는 M2 나사 구멍 (옆에서 조여 넣는다). 스프링이나 추는
    # 다 만든 팔의 반복 정밀도가 모자랄 때만 단다
    d, hh = STRING_D, 1.1 * STRING_D  # 홈의 두 면을 45°보다 조금 세운다
    groove = j.ring(R_TABLE + 0.01, R_TABLE - d, STRING_Y - hh, STRING_Y + hh)
    groove -= Pos(0, STRING_Y - hh / 2, 0) * (Rot(-90, 0, 0) * Cone(R_TABLE, R_TABLE - d, hh))
    groove -= Pos(0, STRING_Y + hh / 2, 0) * (Rot(-90, 0, 0) * Cone(R_TABLE - d, R_TABLE, hh))
    a = solid(a - solid(Rot(0, 45, 0) * solid(groove)))
    sx, sz = j.polar(1.0, STRING_DEG)
    r0, r1 = R_TABLE - 5.5, R_TABLE + 1
    a = solid(a - rod((sx * r0, STRING_Y, sz * r0), (sx * r1, STRING_Y, sz * r1), M2_TAP / 2))
    if not a.is_valid:
        raise RuntimeError("돌림판 형상이 깨졌다")
    return a


def retainer() -> Compound:
    """누름 틀: STS 관절의 것에서 고리만 길다 (허브가 두껍다)."""
    return j.retainer(B0)


# ---- 베어링을 미는 도구 ----
# 베이스 몸통은 뒤집어 놓고 밑(열린 쪽)에서 베어링을 넣는다. 위 베어링은 아래 자리(깊이 46.4 mm부터 6.6 mm)를
# 눌러 지나 위 자리(깊이 76.4 mm부터)의 턱까지, 아래 베어링은 아래 자리까지 들어간다. 손가락이 닿지 않아
# 바깥 링만 미는 통으로 민다. 통의 끝이 턱에 앉은 위 베어링에 닿았을 때 손잡이 테가 몸통 밑면에서 PUSH_STOP만큼 뜬다:
# 테가 밑면에 닿으려 하면 다 들어간 것이다 (테는 멈춤이 아니라 눈금이다. 베어링은 몸통의 턱이 세운다)
PUSH_R = j.R_BORE - 0.3  # 통의 바깥 반지름 20.8: 좁은 자리(반지름 21.1)를 0.3 틈으로 지난다
# 통의 안 반지름: 바깥 링만 밀고 옆 덮개(ZZ)에는 닿지 않는다. 가진 6806의 바깥 링 안쪽 지름은 39.05다 (2026-11-22,
# 캘리퍼스 안쪽 턱). 세워 뽑은 구멍이 지름으로 0.2쯤 작게 나와도(6806 자리 시험편) 링 가장자리에서 0.1 넘게 남는다
PUSH_BORE = 19.8
PUSH_STOP = 0.4
PUSH_GRIP = (24.0, 3.0)  # 손잡이 테의 반지름과 두께


def bearing_pusher() -> Compound:
    """베어링을 미는 도구. 손잡이 테를 바닥에 두고 세워 뽑는다 (미는 끝이 맨 위 층이다).

    좌표는 도구의 것이다: 축이 y, 미는 끝이 y = 0, 손잡이 테의 안쪽 면이 y = -(깊이)다.
    """
    reach = (B1 - Y_FLOOR) + PUSH_STOP  # 몸통 밑면에서 턱에 앉은 위 베어링의 밑면까지 76.0
    grip_r, grip_t = PUSH_GRIP
    tool = j.ring(PUSH_R, PUSH_BORE, -reach - 0.01, 0.0) + cyl_y(grip_r, -reach - grip_t, -reach)
    tool = solid(tool - cyl_y(4.0, -reach - grip_t - 1, -reach + 1))  # 공기가 빠지는 구멍
    if not tool.is_valid:
        raise RuntimeError("베어링을 미는 도구 형상이 깨졌다")
    return solid(tool)


# ------------------------------------------------------------------------------------------------
# J1 몸통 (흰색): 어깨 집 + 목 + 돌림판에 앉는 발. 어깨의 서보 좌표로 그린다


def j1_body() -> Compound:
    """J1 몸통. 어깨 원통 앞 끝(어깨 좌표의 y = FRONT)을 바닥에 두고 뽑는다.

    목의 아래 끝을 두꺼운 바닥판(FLOOR_T)이 막고, 목 앞뒤로 둥근 귀(돌림판과 같은 반지름, 두께 6.2)가 나온다.
    바닥판은 돌림판 밑에서 올라온 나사 넷으로 달리고, 뒤쪽 귀에 영점 핀 구멍이 있다.
    돌림판의 테와 귀의 둘레가 선이 감기는 면이다.
    """
    h = sh.house(NECK_X)
    x0 = NECK_X
    x_ear, x_floor = x0 + j.EAR_T, x0 + FLOOR_T
    lim = R_TABLE + 5
    foot = cyl_x(R_TABLE, x0, x_ear, sh.J1_AXIS_Y, 0.0) & box(
        x0 - 1, x_ear + 1, -lim, sh.FRONT, -lim, lim
    )
    wall = sh.NECK_HW - 0.01
    # 뽑을 때 귀의 아랫변이 벽에서 45°로 서게 깎는다 (원호는 벽 가까이에서 44°로 조금 눕는다)
    y_w = sh.J1_AXIS_Y + math.sqrt(R_TABLE**2 - sh.NECK_HW**2) - 0.3
    for sg in (1, -1):
        cut = j.prism_x(
            [
                (y_w, sg * wall),
                (y_w - 20.0, sg * (wall + 20.0)),
                (y_w + 25.0, sg * (wall + 20.0)),
                (y_w + 25.0, sg * wall),
            ],
            x0 - 1,
            x_ear + 1,
        )
        foot = solid(foot - cut)
    # 목 안쪽의 두꺼운 바닥판: 목의 단면 전체를 막는다 (뽑을 때 선 벽이다)
    foot = solid(foot + box(x0, x_floor, sh.BACK_END, sh.FRONT, -sh.NECK_HW, sh.NECK_HW))
    h = solid(h + foot)
    # 가운데 구멍: 누워서 뽑히는 큰 가로 구멍이라 위를 45° 지붕으로 낸다
    hole = teardrop(
        (x0 - 1, sh.J1_AXIS_Y, 0.0), (1.0, 0.0, 0.0), FLOOR_T + 2, 2 * FOOT_HOLE, (0.0, -1.0, 0.0)
    )
    h = solid(h - hole)
    for (x_nut, y_h, z_h), slide, length in foot_nut_slots():
        h = solid(h - cyl_x(j.M3_FREE_H / 2, x0 - 1, x_floor + 1, y_h, z_h))
        trap = nut_trap((x_nut, y_h, z_h), (1.0, 0.0, 0.0), slide, length, *FOOT_NUT_SLOT)
        h = solid(h - trap)
    h = solid(h - cyl_x(j.PIN_HOLE_H / 2, x0 - 1, x_ear + 1, sh.J1_AXIS_Y, R_PIN))
    # 뒤쪽 귀의 누르개 나사 구멍 (0031): J1을 넘는 선을 여기서 잡는다. 베이스의 (x, z) → 어깨의 (y, z)
    cx, cz = EAR_CLAMP
    h = solid(h - cyl_x(j.M3_TAP / 2, x0 - 1, x_ear + 1, sh.J1_AXIS_Y - cz, -cx))
    # 왼쪽 면의 누르개 나사 구멍 (0031): 왼쪽 면에서 바닥판 속으로 조여 넣는다 (뽑을 때 세로 구멍)
    h = solid(
        h
        - cyl_y(
            j.M3_TAP / 2,
            sh.FRONT - STRIP_TAP,
            sh.FRONT + 1,
            STRIP_SCREW_UP - SHOULDER_H,
            -STRIP_SCREW_X,
        )
    )
    # 쉬는 자세의 선반을 다는 너트 둘 (0037): 목 앞벽 안쪽의 살과, 뒤 끝에서 떨어뜨리는 홈
    z_wall, z_in = -(sh.NECK_IN + 0.01), _SHELF_RIB_IN
    z_out = -(sh.NECK_HW - SHELF_WALL)  # 너트가 기대는 면 (홈의 벽 쪽 면)
    lower, upper = shelf_nut_seats()
    y_n = lower[1]
    rib = j.prism_x(
        [
            (sh.BACK_END, z_wall),
            (sh.BACK_END, z_in),
            (sh.PAD_Y[0] + 0.01, z_in),
            (sh.PAD_Y[0] + 0.01, z_wall),
        ],
        sh.PAD_X[0],
        sh.PAD_X[1],
    )
    h = solid(h + rib)
    # 위 너트의 살: 앞쪽(+y, 뽑을 때 아래)은 벽에서 45°로 선다 (서보 먼 끝을 잡는 턱과 같다)
    y_end = y_n + 4.0
    rib = j.prism_x(
        [
            (sh.BACK_END, z_wall),
            (sh.BACK_END, z_in),
            (y_end, z_in),
            (y_end + (z_in - z_wall), z_wall),
        ],
        upper[0] - _SHELF_RIB_HX,
        upper[0] + _SHELF_RIB_HX,
    )
    h = solid(h + rib)
    half = FOOT_NUT_SLOT[0] / 2
    for x_n, _y, z_c in (lower, upper):
        seat = nut_trap((x_n, y_n, z_c), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0), SHELF_GRIP, *SHELF_SEAT)
        h = solid(h - seat)
        drop = box(
            x_n - half,
            x_n + half,
            sh.BACK_END - 1,
            y_n - SHELF_GRIP + 0.6,  # 꼭 끼는 자리와 0.6 겹친다 (끝이 층 경계: 바닥에서 43.2)
            z_out,
            z_out + FOOT_NUT_SLOT[1],
        )
        h = solid(h - drop)
        h = solid(h - cyl_z(j.M3_FREE_H / 2, -sh.NECK_HW - 1, z_in + 1, x_n, y_n))
    if not h.is_valid or len(h.solids()) != 1:
        raise RuntimeError("J1 몸통 형상이 깨졌다")
    return h


# 선반 너트의 살: 홈 안쪽 면에서 SHELF_SKIN만큼 더 들어온다 (어깨 좌표 z). 어깨 서보의 옆면(15.0)과는 3.8 뜬다
_SHELF_RIB_IN = -(sh.NECK_HW - SHELF_WALL - FOOT_NUT_SLOT[1] - SHELF_SKIN)
_SHELF_RIB_HX = 4.4  # 위 너트의 살의 반높이 (너트 홈 폭 5.6에 위아래 살 1.6)


def shelf_nut_seats() -> list[Point]:
    """선반 너트 둘의 가운데 (어깨의 서보 좌표): (아래, 위). 너트는 목 앞벽(-z)의 안쪽 살에 든다."""
    z_c = -(sh.NECK_HW - SHELF_WALL - SHELF_SEAT[1] / 2)
    return [(up - SHOULDER_H, SHELF_NUT_Y + sh.J1_AXIS_Y, z_c) for up in SHELF_NUT_UP]


def j1_cover() -> Compound:
    """J1 뒷덮개 (검정): 어깨 집의 뒷덮개를 목의 아래 끝까지 내린 것. 어깨의 서보 좌표다.

    J1 몸통의 둥근 귀는 목의 뒤 끝보다 1.5 mm 더 나와서(돌림판과 같은 반지름), 덮개 판의 안쪽 면과 0.6 mm 겹친다.
    그 자리의 안쪽 면을 귀의 둥근 면대로 얕게 따낸다 (판 2 mm에 깊이 0.9 mm, 틈 0.3 mm. 뽑을 때 윗면이라 걸치지 않는다).
    """
    cover = sh.back_cover(NECK_X)
    ear = cyl_x(R_TABLE + 0.3, NECK_X - 1, NECK_X + j.EAR_T + 0.3, sh.J1_AXIS_Y, 0.0)
    out = solid(cover - ear)
    if not out.is_valid or len(out.solids()) != 1:
        raise RuntimeError("J1 뒷덮개 형상이 깨졌다")
    return out


def foot_nut_slots() -> list[tuple[Point, Point, float]]:
    """J1 몸통 바닥판의 너트 홈 넷: (너트 가운데, 입구 쪽 방향, 홈의 길이). 어깨의 서보 좌표다.

    입구는 모두 J1 몸통을 뽑을 때 위(-y, 어깨 뒷덮개 쪽)다. 뒷덮개 쪽의 둘은 바닥판의 뒤 가장자리까지 열리고
    (뒷덮개의 테가 입구를 가린다), 위팔 쪽의 둘은 바닥판 가운데 구멍 속으로 열린다.
    """
    x_nut = NECK_X + FOOT_SEAT + FOOT_NUT_SLOT[1] / 2
    out: list[tuple[Point, Point, float]] = []
    for x, z in FOOT:  # 베이스의 (x, z) → 어깨의 (y, z) = (J1_AXIS_Y - z, -x)
        y_h, z_h = sh.J1_AXIS_Y - z, -x
        # 가운데 구멍 속으로 열리는 홈은 구멍 가운데에서 4 mm 앞까지 판다 (그 끝은 구멍 속이다)
        length = y_h - sh.BACK_END + 1.0 if z > 0 else -z - 4.0
        out.append(((x_nut, y_h, z_h), (0.0, -1.0, 0.0), length))
    return out


def child_at() -> Location:
    """어깨의 서보 좌표를 베이스 좌표(J1의 서보 좌표)로 옮기는 자리 (J1 각 0).

    어깨의 x(위) → 베이스의 y, 어깨의 y(어깨 축, 출력 쪽) → 베이스의 -z(로봇 왼쪽), 어깨의 z → 베이스의 -x(뒤).
    """
    plane = Plane(
        origin=(0.0, Y_DESK + SHOULDER_H, sh.J1_AXIS_Y),
        x_dir=(0.0, 1.0, 0.0),
        z_dir=(-1.0, 0.0, 0.0),
    )
    return Location(plane)


def as_child(shape: Compound, q_deg: float = 0.0) -> Compound:
    """어깨의 서보 좌표로 그린 것을 베이스 좌표로 옮긴다 (child_at). q_deg는 설계 각이다 (joint.turned)."""
    return j.turned(solid(shape.moved(child_at())), q_deg)


# ------------------------------------------------------------------------------------------------
# 조립 검사에 쓰는 부품 목록


def _screw_down(
    x: float, z: float, seat: float, length: float, head: float = 1.65, head_r: float = 2.85
) -> Compound:
    """머리가 위(+y)에 있는 M3 볼트. 머리 밑면이 seat. 기본 머리는 둥근머리 렌치볼트다."""
    return solid(cyl_y(1.5, seat - length, seat, x, z) + cyl_y(head_r, seat, seat + head, x, z))


def hardware(model: ServoModel) -> list[j.Part]:
    """조임 볼트, 혼 나사, 너트, 벽 나사, 멈춤 나사, J1 몸통 나사, 영점 핀, 서보 뒷면 나사."""
    dy = j.servo_dy(model)
    out: list[j.Part] = []
    pocket = ATOP - SINK  # 머리 자리의 바닥 (와셔가 앉는 면)
    for k, (x, z) in enumerate(clamp_xz()):
        bolt = _screw_down(x, z, pocket + WASHER_T, CLAMP_LEN, CLAMP_HEAD_H, CLAMP_HEAD_R)
        out.append(j.Part(f"조임 볼트 M3x45 {k + 1}", bolt, j.STEEL, True, "metal"))
        washer = j.ring(4.0, 1.6, pocket, pocket + WASHER_T)
        out.append(
            j.Part(
                f"와셔 (조임 볼트) {k + 1}", solid(Pos(x, 0, z) * washer), j.STEEL, True, "metal"
            )
        )
        nut_top = B0 - NUT_ROOF
        nut = j.hex_y(x, z, nut_top - j.NUT_T, nut_top, 5.4) - cyl_y(
            1.5, nut_top - 3, nut_top + 1, x, z
        )
        out.append(j.Part(f"너트 (허브) {k + 1}", solid(nut), j.STEEL, True, "metal"))
    for k, (x, z) in enumerate(j.bolt_xz()):
        seat = B0 - HEAD_RECESS
        out.append(
            j.Part(
                f"혼 나사 M3x8 {k + 1}", _screw_down(x, z, seat, HORN_SCREW), j.STEEL, True, "metal"
            )
        )
    seat = ATOP - j.POST_HEAD_SINK
    out.append(
        j.Part("멈춤 나사 M3x10", _screw_down(R_POST, 0.0, seat, 10.0), j.STEEL, True, "metal")
    )
    nut = j.hex_y(R_POST, 0.0, POST_TIP - j.NUT_T, POST_TIP, 5.4) - cyl_y(
        1.5, POST_TIP - 3, POST_TIP + 1, R_POST, 0.0
    )
    out.append(j.Part("너트 (멈춤 기둥)", solid(nut), j.STEEL, True, "metal"))
    top = ATOP + j.EAR_T
    seat = SEAM + FOOT_RECESS
    for k, (x, z) in enumerate(FOOT):  # 돌림판 밑에서 올라오는 나사와 바닥판 속 너트
        shape = cyl_y(1.5, seat, seat + FOOT_SCREW, x, z) + cyl_y(2.85, seat - 1.65, seat, x, z)
        out.append(j.Part(f"J1 몸통 나사 M3x12 {k + 1}", solid(shape), j.STEEL, True, "metal"))
        y0 = ATOP + FOOT_SEAT
        nut = j.hex_y(x, z, y0, y0 + j.NUT_T, 5.4) - cyl_y(1.5, y0 - 1, y0 + j.NUT_T + 1, x, z)
        out.append(j.Part(f"너트 (J1 몸통) {k + 1}", solid(nut), j.STEEL, True, "metal"))
    pin = cyl_y(1.5, top + 3.0 - 20.0, top + 3.0, -R_PIN, 0.0)
    out.append(j.Part("영점 핀 MSH3-20", solid(pin), j.STEEL, False, "pin"))
    head = Y_DESK + PLATE_RECESS - WASHER_T  # 판 나사 머리의 윗면 (와셔 밑)
    for k, (x, z) in enumerate(plate_xz(), start=1):
        s = cyl_y(1.5, head, head + PLATE_SCREW, x, z) + cyl_y(2.85, head - 1.65, head, x, z)
        out.append(j.Part(f"판 나사 M3x10 {k}", solid(s), j.STEEL, False, "metal"))
        washer = j.ring(4.0, 1.6, head, head + WASHER_T)
        out.append(
            j.Part(f"와셔 (판 나사) {k}", solid(Pos(x, 0, z) * washer), j.STEEL, False, "metal")
        )
        y0 = Y_FLOOR + PLATE_NUT_UP
        nut = j.hex_y(x, z, y0, y0 + j.NUT_T, 5.4) - cyl_y(1.5, y0 - 1, y0 + j.NUT_T + 1, x, z)
        out.append(j.Part(f"너트 (판) {k}", solid(nut), j.STEEL, False, "metal"))
    out += j.wall_screws()
    side = j.NUT_Z + j.K_NUT_SLOT[1] / 2  # 조인 너트가 붙는 옆 홈의 바깥 면 (STS 관절과 같다)
    for sg in (1, -1):
        z0, z1 = (side - j.NUT_T, side) if sg > 0 else (-side, -side + j.NUT_T)
        nut_r = j.hex_z(0.0, j.RET_SCREW_Y, z0, z1, 5.4)
        out.append(j.Part("너트 (누름 틀)", nut_r, j.STEEL * 0.72, False, "metal"))
        nut_c = solid(Pos(0, dy, 0) * j.hex_z(0.0, j.CAR_SCREW_Y, z0, z1, 5.4))
        out.append(j.Part("너트 (서보 받침)", nut_c, j.STEEL * 0.72, False, "metal"))
    nut_x = j.hex_x(j.RET_SCREW_Y, 0.0, side - j.NUT_T, side, 5.4)
    out.append(j.Part("너트 (누름 틀)", nut_x, j.STEEL * 0.72, False, "metal"))
    for k, (x, z) in enumerate(j.BACK_HOLES):
        y = j.L1 - j.PLATE_T + dy  # 머리 밑면 (받침 판의 뒷면)
        shape = cyl_y(1.0, y, y + j.M2_SCREW, x, z) + cyl_y(2.0, y - 1.6, y, x, z)
        out.append(j.Part(f"서보 뒷면 나사 {k + 1}", solid(shape), j.STEEL, False, "metal"))
    return out


def own_parts(model: ServoModel = "STS3215") -> list[j.Part]:
    """J1 관절의 부품: 베이스 몸통과 책상에 무는 판, 속 부품, J1 서보, 돌림판 (J1 몸통과 나사는 빼고)."""
    dy = j.servo_dy(model)
    sv = j.servo(model)
    lower, upper = bearings()
    return [
        j.Part("베이스 몸통", core(), color="cap"),
        j.Part("책상에 무는 판", plate(), color="frame"),
        j.Part("베어링 6806 (아래)", lower, color="metal"),
        j.Part("베어링 6806 (위)", upper, color="metal"),
        j.Part("사이 고리", spacer(), moving=True, color="frame"),
        j.Part("허브", hub(), moving=True, color="cap"),
        j.Part("돌림판", turntable(), moving=True, color="cap"),
        j.Part("누름 틀", retainer(), color="frame"),
        j.Part("서보 받침", solid(Pos(0, dy, 0) * j.carrier()), color="frame"),
        j.Part("서보 몸통", sv["body"], color="servo"),
        j.Part("혼", sv["horn"], moving=True, color="metal"),
        j.Part("플러그", sv["plugs"], color="plug"),
        j.Part("선 자리", sv["wires"], color="wire"),
    ]


def assembly(model: ServoModel = "STS3215") -> list[j.Part]:
    """J1 조립: 베이스 몸통과 속 부품, 돌림판, J1 몸통, 어깨 서보(틈 검사용), 나사와 핀."""
    over = [
        j.Part("J1 몸통", as_child(j1_body()), moving=True, color="body"),
        j.Part("어깨 서보", as_child(sh.servo()["body"]), moving=True, color="servo"),
    ]
    return own_parts(model) + over + hardware(model)


# 서로 닿게 그린 쌍: 안쪽 링을 무는 부품들, 혼에 앉는 허브
_TOUCHING = {
    frozenset(p)
    for p in (
        ("서보 몸통", "혼"),
        ("플러그", "서보 몸통"),
        ("플러그", "선 자리"),
    )
}


def overlaps(parts: list[j.Part], tol: float = 0.05) -> dict[tuple[str, str], float]:
    """출력 부품과 서보끼리 겹친 부피 [mm³] (tol 넘는 것만). 나사와 너트는 구멍 속에 있어 보지 않는다."""
    solids = [p for p in parts if p.density == 0.0]
    out: dict[tuple[str, str], float] = {}
    for i, a in enumerate(solids):
        for b in solids[i + 1 :]:
            if frozenset((a.name, b.name)) in _TOUCHING:
                continue
            if far_apart(a.shape, b.shape, 0.0):
                continue
            v = solid(a.shape & b.shape).volume if a.shape.distance_to(b.shape) < 1e-6 else 0.0
            if v > tol:
                out[(a.name, b.name)] = v
    return out


def clearances(
    parts: list[j.Part], q_deg: float, limit: float = 2.5
) -> dict[tuple[str, str], float]:
    """J1 각 q에서 도는 부품과 서 있는 부품 사이 틈 [mm] (limit 아래만). 영점 핀, 벽 나사, 베어링은 뺀다."""
    static = [
        p
        for p in parts
        if not p.moving
        and "베어링" not in p.name
        and "핀" not in p.name
        and "벽 나사" not in p.name
    ]
    out: dict[tuple[str, str], float] = {}
    for a in (p for p in parts if p.moving):
        name = a.name.rstrip("0123456789 ").strip()
        shape = j.turned(a.shape, q_deg)
        for b in static:
            if b.name == "서보 몸통" and name.startswith("혼"):
                continue  # 혼과 서보는 한 몸이다
            if far_apart(shape, b.shape, limit):
                continue
            d = shape.distance_to(b.shape)
            if d < limit:
                out[(name, b.name)] = min(d, out.get((name, b.name), limit))
    return out


def screw_checks(model: ServoModel) -> dict[str, float]:
    """나사가 물린 길이와 끝의 여유 [mm]."""
    dy = j.servo_dy(model)
    horn_tip = B0 - HEAD_RECESS - HORN_SCREW  # 혼 나사의 끝
    nut_top = B0 - NUT_ROOF
    clamp_seat = ATOP - SINK + WASHER_T  # 조임 볼트 머리의 밑면
    clamp_tip = clamp_seat - CLAMP_LEN
    foot_tip = SEAM + FOOT_RECESS + FOOT_SCREW
    foot_nut = ATOP + FOOT_SEAT  # 바닥판 속 너트의 밑면
    plate_tip = Y_DESK + PLATE_RECESS - WASHER_T + PLATE_SCREW  # 판 나사의 끝
    return {
        "혼 나사가 혼 테(2.5)에 물린 길이": min(j.SEAT - horn_tip, HORN_FLANGE),
        "혼 나사 끝과 서보 앞면 사이": horn_tip - (j.F29 + dy),
        "조임 볼트가 너트(2.4)에 물린 길이": min(nut_top - clamp_tip, j.NUT_T),
        "조임 볼트 끝과 허브 밑면 사이": clamp_tip - (j.SEAT + HUB_RELIEF),
        "조임 볼트 머리 윗면과 돌림판 윗면 사이": ATOP - (clamp_seat + CLAMP_HEAD_H),
        "조임 볼트 머리 자리와 위 안쪽 링 윗면 사이의 살": (ATOP - SINK) - (B1 + j.BRG_B),
        "J1 몸통 나사가 너트(2.4)에 물린 길이": min(foot_tip - foot_nut, j.NUT_T),
        "J1 몸통 나사 끝과 바닥판 윗면 사이": (ATOP + FLOOR_T) - foot_tip,
        "J1 몸통 바닥판과 어깨 서보 먼 끝 사이": (Y_DESK + SHOULDER_H - sh.FAR) - (ATOP + FLOOR_T),
        "판 나사가 너트(2.4)에 물린 길이": min(plate_tip - (Y_FLOOR + PLATE_NUT_UP), j.NUT_T),
        "판 나사 끝과 나사 구멍 바닥 사이": (Y_FLOOR + PLATE_HOLE) - plate_tip,
        "판 나사 머리 자리 위에 남은 판의 살": PLATE_T - PLATE_RECESS,
        "클램프가 벌어지는 틈에서 책상과 날개를 뺀 나머지": CLAMP_OPEN
        - DESK_T
        - (PLATE_T + RING_H),
    }


# 뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋). J1 몸통과 그 뒷덮개(어깨 원통의 뒷면)는 어깨의 서보 좌표다
LABELS = {  # 뽑는 부품의 한글 이름 (출력 안내에 쓴다, horibot_design.slicing)
    "base_core": "베이스 몸통",
    "base_plate": "책상에 무는 판",
    "base_turntable": "돌림판",
    "base_hub": "베이스 허브",
    "base_spacer": "사이 고리",
    "base_retainer": "베이스 누름 틀",
    "j1_body": "J1 몸통",
    "j1_cover": "J1 뒷덮개",
    "base_bearing_pusher": "베어링을 미는 도구",
}


def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    return {
        "base_core": (core(), (0.0, -1.0, 0.0), "black", "strength"),
        "base_plate": (plate(), (0.0, 1.0, 0.0), "black", "strength"),
        "base_turntable": (turntable(), (0.0, -1.0, 0.0), "black", "strength"),
        # 허브: 밑면이 평평하면 혼 쪽 면이 바닥이다. 혼 바깥을 들어 올린 모양이면 뒤집어 뽑는다
        "base_hub": (hub(), (0.0, 1.0 if HUB_RELIEF == 0 else -1.0, 0.0), "black", "strength"),
        "base_spacer": (spacer(), (0.0, 1.0, 0.0), "black", "strength"),
        "base_retainer": (retainer(), (0.0, -1.0, 0.0), "black", "strength"),
        "j1_body": (j1_body(), (0.0, -1.0, 0.0), "white", "strength"),
        "j1_cover": (j1_cover(), (0.0, 1.0, 0.0), "black", "standard"),
        # 팔에 달리지 않는 조립 도구다. 검정 판에 같이 뽑는다 (색은 상관없다)
        "base_bearing_pusher": (bearing_pusher(), (0.0, 1.0, 0.0), "black", "standard"),
    }


def report(model: ServoModel = "STS3215") -> None:
    parts = assembly(model)
    print(
        f"J1 관절 ({model}, 6806 둘): 책상 {Y_DESK:.2f}, 몸통 {Y_FLOOR:.2f} ~ {FRONT:.2f}, 베어링 {B0:.2f} ~ "
        f"{B0 + j.BRG_B:.2f} · {B1:.2f} ~ {B1 + j.BRG_B:.2f}, 돌림판 {SEAM:.2f} ~ {ATOP:.2f}"
    )
    print(
        f"책상에서: 몸통 윗면 {FRONT - Y_DESK:.1f}, 돌림판 윗면 {TOP_H:.1f}, 어깨 축 {SHOULDER_H:.0f} "
        f"(어깨 서보 먼 끝 {SHOULDER_H - sh.FAR:.1f}), 목 아래 끝 x {NECK_X:.1f}"
    )
    print(f"임시 값: {'; '.join(PROVISIONAL)}")
    print("\n뽑는 부품 (무게는 뽑아서 잰 부품에 맞춘 어림, 걸침은 바닥에 닿지 않고 아래를 보는 면)")
    made = printed()
    for name, (shape, up, color, preset) in made.items():
        bb = shape.bounding_box()
        over = overhangs(shape, up)
        print(
            f"  {name:15s} {color:5s} {bb.size.X:5.1f} × {bb.size.Y:5.1f} × {bb.size.Z:5.1f} mm  "
            f"{printed_mass_g(shape, up, preset):5.1f} g  걸침 {over.total:5.0f} mm² (거의 수평 {over.flat:.0f}, "
            f"가장 넓은 한 자리 {over.largest:.0f})"
        )
    mid = {n: mid_layer_levels(s, up) for n, (s, up, _c, _p) in made.items()}
    print(
        "층 가운데에 걸친 수평면 (한 층이 더 뽑힐지가 정해지지 않는다): "
        + (", ".join(f"{n} {z}" for n, z in mid.items() if z) or "없음")
    )
    print("\n".join(air_wall_lines(made)))
    body = made["j1_body"][0]
    drops = [
        j.nut_entry_overlap(body, c, (1.0, 0.0, 0.0), s, reach=n) for c, s, n in foot_nut_slots()
    ]
    print(
        "J1 몸통 바닥판의 홈에 너트를 곧게 떨어뜨릴 때 겹침 [mm³]: "
        + ", ".join(f"{v:.2f}" for v in drops)
    )
    metal = sum(p.mass_g for p in parts if p.density > 0 and "핀" not in p.name)
    print(f"나사와 너트 {metal:.1f} g")
    for name, value in screw_checks(model).items():
        print(f"  {name}: {value:.2f}")
    bend = wing_bend()
    print(
        "클램프 하나가 70 N을 버틸 때 날개가 휘는 양 [mm] (어림): "
        + ", ".join(f"{k} {v:.3f}" for k, v in bend.items())
    )
    plate_box = made["base_plate"][0].bounding_box()
    print(
        f"책상에 무는 판 {plate_box.size.X:.1f} × {plate_box.size.Z:.1f} mm (출력 판 256 mm), 파이 기판 안쪽 변 |z| "
        f"{PI_IN:.1f}, 파이 판 밑면 높이 {TRAY_UNDER:.1f}"
    )
    print(
        f"뒤 판 (서보 어댑터 자리): 책상 뒤 가장자리 밖으로 {-EDGE - DECK_END:.1f} mm, 폭 {2 * DECK_HW:.0f} mm, "
        f"선 창 {DECK_WINDOW[0][1] - DECK_WINDOW[0][0]:.0f} × {2 * DECK_WINDOW[1]:.0f} mm, 서보 단자의 변과 몸통 "
        f"뒷면 사이 {-R_CORE - ADAPTER_EDGE_X:.1f} mm. "
        "20 N에 휘는 양 [mm] (어림): " + ", ".join(f"{k} {v:.3f}" for k, v in deck_bend().items())
    )
    drops = [
        j.nut_entry_overlap(
            body, c, (0.0, 0.0, 1.0), (0.0, -1.0, 0.0), reach=c[1] - sh.BACK_END + 2.0
        )
        for c in shelf_nut_seats()
    ]
    print(
        "선반 너트 둘을 뒤 끝에서 자리까지 곧게 밀 때 겹침 [mm³]: "
        + ", ".join(f"{v:.2f}" for v in drops)
    )
    hits = overlaps([p for p in parts if "핀" not in p.name])
    print(
        "\n겹침 [mm³]: " + (", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in hits.items()) or "없음")
    )
    print("도는 부품과 서 있는 부품 사이 틈 [mm] (2.5 아래만)")
    for q in (0.0, 90.0, j.SOFT_DEG, j.STOP_DEG, -90.0, -j.SOFT_DEG, -j.STOP_DEG):
        gaps = sorted((d, a, b) for (a, b), d in clearances(parts, q).items())
        print(f"  {q:7.1f}°: " + ", ".join(f"{a}↔{b} {d:.2f}" for d, a, b in gaps[:6]))
    body = next(p for p in parts if p.name == "베이스 몸통")
    arm = next(p for p in parts if p.name == "J1 몸통")
    pin = next(p for p in parts if "핀" in p.name)
    print(
        f"영점 핀 (J1 각 0°): 몸통과 틈 {pin.shape.distance_to(body.shape):.2f}, "
        f"J1 몸통과 틈 {pin.shape.distance_to(arm.shape):.2f} (구멍 틈 0.15 ~ 0.18)"
    )


if __name__ == "__main__":
    report()
