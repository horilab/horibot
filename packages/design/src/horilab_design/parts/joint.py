"""STS 관절(J3~J6) 하나의 부품 (0028의 UR식 관절을 부품으로 나눈 것, 0029).

서보 좌표계(mm)로 그린다: x는 출력축에서 서보의 가까운 끝 쪽, y는 출력축 방향(출력 혼 쪽이 +), z = x × y.
원점은 출력축 위, 몸통 두께의 가운데다: y = -14.5가 출력축 둘레의 낮은 뒷면(두께 29), -16.0이 두께 32인
뒷면이다 (docs/hardware.md의 "기구 치수"). 출력 혼 윗면은 그 낮은 뒷면에서 잰 값이다 ("혼 높이").
방위각은 x-z 평면에서 +x가 0°, +z가 90°다. 집의 목(서보 먼 끝)은 180° 쪽이다.

구조:
- 부모: 집(원통과 목). 원통 앞 끝의 턱이 한 몸이고, 베어링(6806)은 뒤에서 넣는다. 누름 틀이 바깥 링 뒷면을
  받치고 벽 나사 셋으로 잠긴다. 서보는 뒷면에 단 서보 받침의 날개를 벽 나사 둘로 맨 나중에 잠근다. 벽 구멍이
  축 방향으로 길어서, 베어링이 정한 자리 그대로 잠긴다 (0028: 베어링이 축을 정한다).
- 자식: 허브 판(혼 윗면에 앉고 안쪽 링 뒤를 받침)과 출력 원판(안쪽 링 속 저널, 원통 앞 끝 위의 테)이 혼 볼트
  넷으로 안쪽 링을 물고 혼에 묶인다. 자식 몸통은 출력 원판 윗면에 M3 × 10 넷으로 단다 (parts/wrist.py, parts/links.py).
- 기계 멈춤: 출력 원판 귀의 기둥(속에 M3 나사)이 목 앞면의 홈을 돌다가 가운데 살에 닿는다 (±STOP_DEG).
- 영점 핀: 자식 몸통의 귀와 그 살의 구멍에 꽂는다.

부품을 품는 몸통(손목 몸통, 링크)은 이 모듈의 원통과 목, 홈을 가져다 쓴다.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from build123d import (
    Compound,
    Cone,
    Location,
    Plane,
    Polygon,
    Pos,
    RegularPolygon,
    Rot,
    extrude,
)

from horilab_design.parts.shapes import (
    Point,
    box,
    comb_ceiling,
    cyl_x,
    cyl_y,
    cyl_z,
    far_apart,
    nut_trap,
    slot_ceiling,
    solid,
)
from horilab_design.printcheck import LAYER, layer_snap
from horilab_design.values import ServoModel, load_measured

PLA = 1.24e-3  # [g/mm³] 나사와 너트의 무게를 어림할 때만 쓴다. 출력 무게는 printcheck.printed_mass_g가 낸다
STEEL = 7.85e-3

_MEASURED = load_measured()

# ---- 서보 (STS3215와 STS3250은 도면이 같다. 공식 2D 도면의 옆모습 29 / 32 / 35와 실측) ----
HW = 12.36  # 반폭
NEAR, FAR = 10.11, 35.11
L0, F29 = -14.5, 14.5  # 출력축 둘레의 낮은 면 (두께 29)
L1, F32 = -16.0, 16.0  # 두께 32인 뒷면과 앞면 (고정 구멍과 커넥터가 이 면에 있다)
STEP = -6.0  # 29 → 32 단이 끝나는 x
LABEL = F32 + 1.1  # 앞면 라벨 판 (x -34.1 ~ -9.2, z ±7.0)
PLATEAU = L1 - 1.9  # 뒷면 볼록한 판 (x -29.7 ~ -17.33, z ±9.24)
BOSS_TIP = L0 - 4.1  # 뒷면 보스 끝 (도면. STS3215의 실측 혼 윗면 ~ 보스 끝 37.32와 맞는다)
HORN_R = 9.975
STS_MODELS: tuple[ServoModel, ...] = ("STS3215", "STS3250")
# 출력 혼 윗면: 낮은 뒷면에서 잰 값이다. 혼이 앞면 라벨 판보다 1.6 ~ 1.8 높다 (깊이 막대로 잰 1.5 ~ 1.75와 맞는다)
HORN_TOP: dict[ServoModel, float] = {
    m: L0 + _MEASURED.servo_body[m].horn_to_back_mm for m in STS_MODELS
}
assert _MEASURED.servo_depth is not None
# 혼 가운데 나사의 머리: 지름과, 혼 윗면 위로 나온 높이 (1.7 ~ 2.0으로 오르내린 값 중 큰 값)
HEAD_D = _MEASURED.servo_depth.horn_screw_head_diameter_mm
HEAD_H = max(_MEASURED.servo_depth.horn_screw_head_mm)
_WIRE = _MEASURED.servo_body["STS3250"]
assert _WIRE.horn_to_wire_mm is not None
WIRE_TOP = HORN_TOP["STS3250"] - _WIRE.horn_to_wire_mm  # 꽂아 눕힌 선 (혼 윗면에서 잰 값, -20.12)
BACK_HOLES = ((-8.30, 10.25), (-8.30, -10.25), (-32.75, 10.25), (-32.75, -10.25))

# ---- 공차 (공차 시험편으로 고른 값: data/measured.toml, 고르는 규칙은 docs/manufacturing.md의 "공차") ----
# 구멍과 홈은 부품의 색과 뽑히는 방향이 같은 시험편의 값을 쓴다. 몸통(손목 몸통, 링크)은 흰색이고, 속 부품(출력
# 원판, 허브 판, 누름 틀, 서보 받침, 뒷덮개)은 검정이다. 검정 값은 K_로 시작한다
_WHITE = _MEASURED.fit["pla_basic_white"]
_BLACK = _MEASURED.fit["pla_basic_black"]
HOLE_SLACK = 0.1  # 볼트가 지나가는 구멍에 더하는 구멍 자리 오차 몫


def _fit(value: float | None) -> float:
    if value is None:
        raise ValueError("공차 시험편 값이 없다 (data/measured.toml)")
    return value


PIN_HOLE = _fit(_WHITE.pin_hole_mm)  # 세로 구멍 (몸통의 살)
PIN_HOLE_H = _fit(_WHITE.pin_hole_horizontal_mm)  # 가로 구멍 (손목 몸통의 귀는 뽑을 때 눕는다)
# 검정, 세로 핀 구멍 (흰색과 같은 3.35): 플랜지 원판의 영점 핀과 어댑터 판의 도구 핀
K_PIN_HOLE = _fit(_BLACK.pin_hole_mm)
# 검정, 도구 핀을 눌러 박는 세로 구멍 (3.30): 손끝으로 들어가고 손으로 안 빠진다
K_PIN_PRESS = _fit(_BLACK.pin_press_mm)
M3_FREE = _fit(_WHITE.m3_hole_mm) + HOLE_SLACK  # 흰색, 볼트가 지나가는 세로 구멍 (3.4)
M3_FREE_H = _fit(_WHITE.m3_hole_horizontal_mm) + HOLE_SLACK  # 흰색, 가로 구멍 (3.4)
K_M3_FREE = _fit(_BLACK.m3_hole_mm) + HOLE_SLACK  # 검정, 세로 구멍 (3.4)
K_M3_FREE_H = _fit(_BLACK.m3_hole_horizontal_mm) + HOLE_SLACK  # 검정, 가로 구멍 (3.3)
M3_TAP = _fit(_BLACK.m3_tap_hole_mm)  # 조여 넣는 구멍 (뒷덮개 나사). 흰색도 같은 값이다 (2.8)
NUT_T = 2.4  # M3 너트 두께
NUT_AF = _fit(_WHITE.m3_nut_pocket_mm)  # 흰색, 위에서 눌러 끼우는 육각 홈의 맞변 거리 (5.6)
# 검정, 위에서 눌러 끼우는 육각 홈 (5.5): 엄지로 눌러 들어가고 뒤집어도 안 빠진다 (출력 원판 밑면의 너트)
K_NUT_AF = _fit(_BLACK.m3_nut_pocket_mm)
# 검정, 너트가 서서 옆에서 밀려 들어가는 홈 (세로 폭 5.8, 두께 2.8): 누름 틀과 서보 받침의 너트 자리.
# 뽑을 때 너트의 축이 가로이고 홈 천장이 허공에 걸쳐서, 위에서 끼우는 홈의 값이 옮겨지지 않는다
K_NUT_SLOT = (
    _fit(_BLACK.m3_nut_upright_width_mm),
    _fit(_BLACK.m3_nut_upright_thickness_mm),
)
# 검정, 너트가 누워 옆에서 밀려 들어가는 홈 (폭 5.9, 높이 2.8): 뽑을 때 너트의 축이 세로다
K_NUT_LYING = (
    _fit(_BLACK.m3_nut_slot_width_mm),
    _fit(_BLACK.m3_nut_slot_height_mm),
)
# 6806 바깥 링 자리: 엄지로 꾹 눌러 들어가고 놀지 않는다. 흰색과 검정이 같은 값이다 (42.2)
BEARING_BORE = _fit(_WHITE.bearing_6806_bore_mm)
assert BEARING_BORE == _fit(_BLACK.bearing_6806_bore_mm)
# 베어링이 자리까지 지나가는 구멍과 속 부품이 드나드는 구멍: 자리보다 0.2 넓다 (42.3이 저절로 들어가는 칸이다)
BEARING_ENTRY = BEARING_BORE + 0.2
JOURNAL_D = _fit(_BLACK.bearing_6806_shaft_mm)  # 검정, 안쪽 링 속 저널 (30.1)
# 다른 부품 코드가 이어 쓰는 임시 값의 목록 (이 모듈의 값은 모두 시험편 값이다)
PROVISIONAL: tuple[str, ...] = ()

# ---- 베어링과 축 방향 쌓임 ----
BRG_D, BRG_OD, BRG_B = 30.0, 42.0, 7.0
SEAT = max(
    HORN_TOP.values()
)  # 허브 판이 혼 윗면에 앉는 높이 (STS3250 기준. STS3215는 몸통이 0.17 앞으로 온다)
# 허브 판이 혼 위에서 두꺼운 높이 (혼 바깥에서는 밑면이 이만큼 들려 있다). 혼 가운데 나사 머리(혼 윗면 위로
# HEAD_H)가 허브 판 속에 다 들어가고, 저널 끝과 0.8 뜬다. 앞면 라벨 판과는 3.2 ~ 3.4 뜬다
RELIEF = 1.6
HUB_SEAT_R = 9.8  # 허브 판이 혼 윗면에 앉는 돋움의 반지름 (테는 이 가장자리에서 휜다)
# 허브 판의 테 (안쪽 링 뒤). 혼 볼트를 조이면 이 테가 CLAMP_GAP만큼 휘면서 안쪽 링을 누르고, 그 뒤에는
# 저널 끝이 허브 판에 닿아 볼트 힘이 저널로 간다. 테가 스프링이라 온도와 크리프에도 누르는 힘이 남는다.
# 두께 1.2, 틈 0.2면 누르는 힘 80 ~ 110 N, 휨 응력 28 ~ 38 MPa다 (탄성 계수 2.0 ~ 2.75 GPa, 고리 판 계산:
# studies/clamp_plate.py). 두께 2.0, 틈 0.3이면 69 ~ 104 MPa로 PLA의 굽힘 강도(약 76 MPa)에 닿는다
FLANGE_T = 1.2
# 조이기 전에 저널 끝과 허브 판 사이에 남는 틈. 출력 원판을 뽑을 때 저널 끝이 층에 맞는 값이다
CLAMP_GAP = 0.2
B0 = SEAT + RELIEF + FLANGE_T  # 베어링 뒷면
LIP_T = 1.8
FRONT = B0 + BRG_B + LIP_T  # 원통 앞 끝 (몸통을 뽑을 때 바닥)
GAP = 2.0  # 원통 앞 끝과 출력 원판 테 사이 (0017의 움직이는 틈)
SEAM = FRONT + GAP  # 출력 원판 테의 밑면
BRIM_T = 3.8  # M3 × 20 혼 볼트 머리가 테 윗면(뽑을 때 바닥 면)에 바로 앉고 혼에 2.6 들어가는 두께
ATOP = SEAM + BRIM_T  # 출력 원판 윗면 = 자식 몸통이 앉는 면
R_BORE = BEARING_BORE / 2
R_ENTRY = BEARING_ENTRY / 2
R_DRUM = 23.6  # 원통 바깥 반지름. 벽은 베어링 자리에서 2.5, 그 뒤에서 2.4 mm다
WALL = R_DRUM - R_BORE
# 베어링 자리(좁은 구멍)가 시작하는 곳: 누름 틀의 고리가 좁은 구멍에 들지 않게 0.4 앞이다. 몸통을 뽑을 때
# 이 단이 층 경계(바닥에서 8.4)에 온다
SEAT_Y = B0 + 0.4
R_SLIDE = R_ENTRY - 0.3  # 원통 속에서 드나드는 부품(누름 틀, 서보 받침)의 바깥 반지름 (20.9)
R_LIP = 19.5
R_STEP = 16.5  # 허브 판의 테와 출력 원판의 단: 안쪽 링 앞뒤 면만 누른다
R_JOURNAL = JOURNAL_D / 2
COVER_GAP = (
    1.0  # 꽂아 눕힌 선과 그 위를 덮는 면 사이의 기준 틈 (어깨의 뒷덮개와 베이스의 판도 쓴다)
)
# 뒷덮개: 바깥 면은 눕힌 선 위 2.5 mm다 (관절의 길이 ATOP - BACK_OUT = 58.9를 정한다). 판은 1.4 mm(7층)이고
# 선과 안쪽 면 사이가 1.1 mm다. 판이 1.5면 안쪽 면이 덮개를 뽑을 때 층의 한가운데에 온다
COVER_T = 1.4
BACK_OUT = WIRE_TOP - COVER_GAP - 1.5  # 뒷덮개 바깥 면
BACK_IN = BACK_OUT + COVER_T  # 뒷덮개 안쪽 면
# 원통 뒤 끝 (몸통을 뽑을 때 맨 위, 바닥에서 50.8). 덮개 안쪽 면과 0.9 띄운다: 덮개는 서보 받침을 따라 앉는다
BACK_END = BACK_OUT + 2.3
COVER_SKIRT = 2.5  # 덮개 테가 원통 뒤 끝을 겹쳐 가리는 길이
COVER_R = R_DRUM + 0.4 + 1.6
# 바닥에 닿는 면에서 판 M3 둥근머리(지름 5.7, 높이 1.65) 자리의 반지름과 깊이. 천장(머리가 앉는 면)은 허공에
# 뽑히므로 첫 층의 선이 곧게 건너도록 따낸다 (parts/shapes.py의 slot_ceiling, comb_ceiling)
HEAD_POCKET_R = 3.3
POST_HEAD_SINK = (
    1.8  # 멈춤 나사: 머리가 출력 원판 윗면 밑으로 다 든다 (그 위로 자식 몸통의 목이 지나간다)
)
COVER_HEAD_SINK = 1.2  # 뒷덮개 나사: 머리가 덮개 바깥 면 위로 0.45 나온다
# 뒷덮개의 나사 기둥. 머리 자리(3.3)보다 굵어서 기둥이 머리 자리 둘레의 판에 0.7 mm 폭으로 얹힌다. 3.2였을 때는
# 기둥과 판이 머리 자리 천장 한 층(0.2 mm)으로만 이어졌다. 받침의 날개(반지름 20.5) 안에 앉는다 (16.3 + 4.0)
COVER_BOSS_R = 4.0

# ---- 원통 속 부품 ----
RET_Y = (5.0, 18.7)  # 누름 틀 다리의 범위
RET_SCREW_Y = 12.0
CAR_Y = (L1, L1 + 17.8)  # 서보 받침 날개의 범위
CAR_SCREW_Y = L1 + 12.5  # 서보를 잠그는 벽 나사 (받침 판에서 12.5)
LEAF_T = 1.4  # 날개 두께: 반지름 쪽으로 무르고 돌리는 쪽으로 단단하다
# 서보 받침의 판: 한 두께이고, 그 두께가 서보 뒷면 나사의 머리 밑 살이다. 뒷면 전체가 뽑을 때 바닥에 닿아 머리가
# 바닥 면에 바로 앉는다. 3.0 mm 판에 머리 자리(깊이 0.8)와 선 홈(0.8)을 팠을 때는 먼 가로대가 통째로 바닥에서 떠서,
# 그 천장의 벽 선이 받칠 살 없이 공중에 뽑혔다 (머리 자리가 판 끝까지 터져 모서리에 살이 없었다)
PLATE_T = 2.2
# 서보 밖(옆면에서 0.5)의 날개는 윗면을 올려 이 두께를 지킨다: 날개 벽과 돋움이 서는 자리다
WING_T = 3.0
WING_IN = HW + 0.54  # 12.9
COVER_BOSS = ((3.0, 16.0), (3.0, -16.0))  # 뒷덮개 나사 자리 (x, z)
COVER_SCREW = 12.0  # 뒷덮개 나사 M3 × 12
# 서보 뒷면 나사: STS3215는 2 mm 자가탭(가진 것은 예전 SO-101에서 뺀 둥근머리, 전체 7.16 mm), STS3250은 M2다.
# 구멍은 흔한 M2·ST2.2 둥근머리(머리 지름 4.0, 높이 1.6, 나사산 2.2까지)가 검정 세로 구멍의 줄어듦(M3에서 약 0.3)까지
# 넣어 걸리지 않게 낸다. 머리 밑의 살(PLATE_T)이 2.2 mm라, 가진 자가탭 나사(머리 밑 5.5 ~ 6.0으로 본다)가 서보에
# 3.3 ~ 3.8 mm 물린다 (살이 1.2면 4.3 ~ 4.8 mm라 구멍 바닥에 먼저 닿을 수 있다). 머리는 판 뒷면 위로 1.6 나오고,
# 뒷덮개 안쪽 면(판에서 3.0)과 1.4 mm 뜬다. 꽂아 눕힌 선은 밑이 판 뒷면에서 0.4 떠서 판 위 어디로든 지나간다
# (나사 머리 사이는 |z| 8.25까지다)
M2_HOLE_R = 1.35  # 나사가 지나가는 구멍의 반지름
M2_SCREW = 5.0  # STS3250: M2 × 5가 서보에 2.8 mm 물린다
# 옛 받침(판 3.0)의 먼 가로대에 있던 선 홈의 x 범위. 지금 받침에는 홈이 없다. 작업 안내의 첫 관절 조립이 이미 뽑은
# 받침에서 실을 걷을 자리로 가리킨다 (guide/guides/first_joint.py). 안내를 고치면 지운다
WIRE_GROOVE_X = (-36.0, -29.2)
SPOT_R = R_DRUM - 0.6  # 벽 나사 머리가 앉는 면
SCREW_LEN = 8.0  # 벽 나사 M3 × 8
# 벽 나사의 너트가 드는 홈의 가운데 (축에서). 홈은 15.9 ~ 18.7이고, 조이면 너트가 바깥 면에 붙는다 (16.3 ~ 18.7).
# 나사 끝(15.0)이 너트를 1.3 지나간다. 홈이 서보 받침의 날개 안쪽 면(19.5)보다 안에 있어 날개를 파지 않는다
NUT_Z = 17.3
# 서보 받침의 날개를 따내기 시작하는 x: 돋움의 옆면이다. 날개 안쪽 면은 x 5.53부터 홈의 바깥 면(18.7)보다
# 안으로 들어온다 (√(19.5² - 18.7²))
NUT_GATE_X = 4.6

# ---- 목 (서보 먼 끝을 품는다) ----
NECK_HW = HW + 0.34 + 2.6  # 목 바깥 반폭 15.3
NECK_IN = HW + 0.34  # 목 안쪽 반폭
NECK_FRONT_IN = FRONT - 2.4  # 목 앞벽의 안쪽 면

# ---- 기계 멈춤, 영점 핀, 자식 몸통 나사 ----
R_POST = 28.0  # 멈춤 기둥 중심
BOSS_R = 3.25
# 멈춤 기둥 귀의 반폭. 귀의 옆면은 멈춤 각에서 살 가운데(선이 넘는 자리) 가까이 온다. 4.0이면 테의 면에서
# 살 가운데까지 3.1 mm가 남아 선(반폭 2.4)과 0.7 mm 뜬다 (4.6이면 2.5 mm가 남아 0.1 mm다)
EAR_HW = 4.0
POST_TIP = SEAM - 4.6  # 기둥 돋움의 끝 (원판을 뽑을 때 바닥에서 8.4). 그 아래 너트 2.4
STOP_DEG = 163.0  # 기둥이 살에 닿는 각. 프로그램 한계는 SOFT_DEG
SOFT_DEG = 160.0
WEB_HALF = 180.0 - STOP_DEG - math.degrees(math.asin(BOSS_R / R_POST))  # 살의 반각 10.3°
CH_R1 = 32.8  # 홈 바깥 반지름
CH_Y = 23.5  # 홈 바닥 (원통 앞 끝에서 7.0). 멈춤 나사 끝(24.5)과 1.0 띄운다
TUB_Y = CH_Y - 1.6  # 홈 밑을 막는 살의 밑면. 서보 앞면의 라벨 판(17.3)과 4.6 띄운다
TUB_X = -34.6  # 그 살이 끝나는 x
R_PIN = 29.0  # 영점 핀
# 자식 몸통 나사 (x, z): 축에서 19.5 mm. 출력 원판 밑면의 너트 홈(맞변이 반지름 쪽을 본다)이 안쪽 링을 누르는
# 단(R_STEP)과 테 밑의 테두리(R_DRUM - LIP_W) 사이에 든다. z가 ±19.4(축에서 20.0)일 때는 테두리가 홈 입구를 0.9 mm 막았다
RIM = ((5.0, 18.85), (-5.0, 18.85), (5.0, -18.85), (-5.0, -18.85))
BODY_SCREW = 10.0  # M3 × 10
# 자식 몸통 귀의 두께: 나사 10 = 귀 6.2 + 출력 원판 살 1.2 + 너트 2.4 + 0.2 (끝이 테 밑면과 같은 높이에 온다)
EAR_T = 6.2
NUT_POCKET = NUT_T + 0.2  # 출력 원판 밑면의 너트 홈 깊이: 홈 바닥이 원판을 뽑을 때의 층 경계에 온다
# 링크의 선 창 (플러그 9.01 × 4.75가 지나간다: 길이 9.6, 폭 6.0). 축 쪽 끝이 축에서 36.0 mm라, 창에서 곧게 올라오는
# 선이 자식 발의 영점 핀 귀가 쓸고 가는 자리(33.4)와 2.6 mm 뜬다 (0031. 35.4였을 때는 2.01 mm였다). 이미 뽑은 가짜
# 아래팔 끝(parts/fake_forearm.py)은 옛 자리 그대로다
WIN_X = (-45.6, -36.0)
WIN_Z = 3.0

# ---- 선 (0031) ----
# 서보 선 한 줄이 차지하는 자리: 세 가닥을 나란히 놓은 폭과 두께. 폭은 쓰는 선 가운데 넓은 쪽이다 (딸려 온
# 20 cm 선 4.75, 손목 구간과 긴 구간에 쓰는 Waveshare 선 4.85). 두께는 둘 다 1.5다
assert _MEASURED.servo_wire is not None
_BOUGHT_W = (
    _MEASURED.waveshare_servo_wire.width_mm if _MEASURED.waveshare_servo_wire is not None else None
)
WIRE_W = max(_MEASURED.servo_wire.width_mm, _BOUGHT_W or 0.0)
WIRE_T = _MEASURED.servo_wire.thickness_mm
# 꽂은 플러그의 뒤 끝 (선이 나오는 곳): 눕힌 선의 밑이다. 커넥터가 있는 뒷면(L1)에서 2.6 나온다
PLUG_TOP = WIRE_TOP + WIRE_T
# 선은 살(180°) 위에서 출력 원판의 테로 건너가 테 둘레에 감긴다. 테 밑의 틈(GAP)으로 선이 빠지면 도는 원판과
# 서 있는 원통 사이에 낀다. 테 밑면 가장자리에 테두리를 내려 틈을 LIP_GAP으로 좁힌다 (선 두께보다 좁다)
# 0.8이면 테두리 끝이 원판을 뽑을 때의 층 경계(바닥에서 5.0 mm)에 온다. 0.7이면 층의 한가운데라 한 층이
# 더 뽑힐지가 정해지지 않는다
LIP_GAP = 0.8
LIP_W = 1.2


# ------------------------------------------------------------------------------------------------
# 형상 도우미


def ring(r_out: float, r_in: float, y0: float, y1: float) -> Compound:
    return solid(cyl_y(r_out, y0, y1) - cyl_y(r_in, y0 - 0.01, y1 + 0.01))


def hex_z(x: float, y: float, z0: float, z1: float, af: float = NUT_AF) -> Compound:
    """z 방향 육각 기둥 (너트 자리)."""
    h = extrude(RegularPolygon(af / 2, 6, major_radius=False), amount=z1 - z0)
    return solid(Pos(x, y, z0) * h)


def hex_x(y: float, z: float, x0: float, x1: float, af: float = NUT_AF) -> Compound:
    h = extrude(RegularPolygon(af / 2, 6, major_radius=False), amount=x1 - x0)
    return solid(Pos(x0, y, z) * (Rot(0, 90, 0) * h))


def hex_y(x: float, z: float, y0: float, y1: float, af: float = NUT_AF) -> Compound:
    h = extrude(RegularPolygon(af / 2, 6, major_radius=False), amount=y1 - y0)
    return solid(Pos(x, y1, z) * (Rot(90, 0, 0) * h))


def hex_y_radial(x: float, z: float, y0: float, y1: float, af: float = NUT_AF) -> Compound:
    """y 방향 육각 기둥인데, 맞변 한 쌍이 축에서 (x, z)로 가는 반지름과 직각이다 (원판 둘레의 너트 자리)."""
    h = extrude(RegularPolygon(af / 2, 6, major_radius=False), amount=y1 - y0)
    turned_h = Rot(0, 0, math.degrees(math.atan2(z, x)) - 90.0) * h
    return solid(Pos(x, y1, z) * (Rot(90, 0, 0) * turned_h))


def polar(r: float, a: float) -> tuple[float, float]:
    return r * math.cos(math.radians(a)), r * math.sin(math.radians(a))


def wedge(a0: float, a1: float, r: float, y0: float, y1: float) -> Compound:
    """방위각 a0 ~ a1 [°], 반지름 r까지의 부채꼴 기둥 (y0 ~ y1)."""
    n = max(2, int(abs(a1 - a0) / 6) + 1)
    pts = [(0.0, 0.0)] + [polar(r, a0 + (a1 - a0) * k / n) for k in range(n + 1)]
    prism = extrude(Polygon(*pts, align=None), amount=y1 - y0)
    return solid(Pos(0, y1, 0) * (Rot(90, 0, 0) * prism))


def sector(r0: float, r1: float, a0: float, a1: float, y0: float, y1: float) -> Compound:
    """방위각 a0 ~ a1, 반지름 r0 ~ r1의 고리 조각."""
    band = cyl_y(r1, y0, y1) - cyl_y(r0, y0 - 0.01, y1 + 0.01)
    return solid(band & wedge(a0, a1, r1 + 5, y0 - 0.02, y1 + 0.02))


def prism_x(pts: list[tuple[float, float]], x0: float, x1: float) -> Compound:
    """(y, z) 다각형을 x0 ~ x1로 뽑은 기둥."""
    area = sum(a[0] * b[1] - b[0] * a[1] for a, b in zip(pts, pts[1:] + pts[:1], strict=True))
    if area < 0:
        pts = pts[::-1]  # 반시계 방향으로 맞춘다 (면이 뽑는 쪽을 보게)
    body = extrude(Polygon(*pts, align=None), amount=x1 - x0)
    # 스케치 (u, v, w) → (x, y, z) = (w, u, v): z축 둘레 90°, 이어서 y축 둘레 90°
    return solid(Pos(x0, 0, 0) * (Rot(0, 90, 0) * (Rot(0, 0, 90) * body)))


def turned(shape: Compound, q_deg: float) -> Compound:
    """자식 쪽 부품을 관절 각 q만큼 돌린다 (+y 축 둘레, 방위각이 커지는 쪽)."""
    return solid(Rot(0, -q_deg, 0) * shape)


# ------------------------------------------------------------------------------------------------
# 검사용 서보 외형 (출력하지 않는다)


def servo(model: ServoModel = "STS3250") -> dict[str, Compound]:
    """서보 몸통, 혼, 꽂은 플러그, 눕힌 선이 차지하는 자리. 몸통은 혼 윗면이 SEAT에 오게 옮긴다."""
    dy = SEAT - HORN_TOP[model]
    top = HORN_TOP[model]
    body = box(-FAR, NEAR, L0, F29, -HW, HW)  # 두께 29
    body += box(-34.5, STEP, L1, F32, -HW, HW)  # 두께 32 (앞뒤로 1.5씩)
    body += box(-34.1, -9.2, F32, LABEL, -7.0, 7.0)  # 앞면 라벨 판
    body += box(-29.7, -17.33, PLATEAU, L1, -9.24, 9.24)  # 뒷면 볼록한 판
    body -= cyl_y(10.8, F29, LABEL + 1)  # 혼 둘레의 둥근 자리
    body += cyl_y(3.1, BOSS_TIP, L0)  # 뒷면 보스
    horn = cyl_y(HORN_R, F29 + 0.3, top)
    horn += cyl_y(HEAD_D / 2, top, top + HEAD_H)  # 혼과 가운데 나사 머리
    plugs = box(-16.6, -11.85, PLUG_TOP, L1 + 0.5, 0.45, 9.46)
    plugs += box(-16.6, -11.85, PLUG_TOP, L1 + 0.5, -9.46, -0.45)
    wires = box(-36.0, -11.85, WIRE_TOP, PLUG_TOP, -8.5, 8.5)  # 먼 끝 쪽으로 눕힌 선
    out = {"body": solid(body), "horn": solid(horn), "plugs": solid(plugs), "wires": solid(wires)}
    return {k: solid(Pos(0, dy, 0) * v) for k, v in out.items()}


def servo_dy(model: ServoModel) -> float:
    """서보 몸통과 거기 달린 부품(서보 받침, 뒷덮개)이 축 방향으로 옮겨지는 값."""
    return SEAT - HORN_TOP[model]


# ------------------------------------------------------------------------------------------------
# 원통과 목 (몸통이 가져다 쓰는 조각)


def drum_shell(neck_x: float) -> Compound:
    """원통과 목의 겉. 목은 x = neck_x까지다."""
    return solid(
        cyl_y(R_DRUM, BACK_END, FRONT) + box(neck_x, -12.0, BACK_END, FRONT, -NECK_HW, NECK_HW)
    )


def drum_cavity(
    neck_x: float, roof_y: float | None = None, roof_x: float | None = None
) -> Compound:
    """원통과 목의 속. 서보 앞면 위(TUB_Y부터)는 멈춤 홈 밑을 막는 살로 채우고 x < TUB_X만 비운다.

    roof_y를 주면 목의 뒷면 쪽(y < roof_y)은 x = roof_x에서 막힌다 (링크: 목이 낮은 관으로 이어질 때).
    """
    # 베어링 자리(R_BORE)는 앞 끝의 턱 바로 뒤 6.5 mm뿐이고, 그 뒤는 0.1 넓다 (R_ENTRY): 베어링이 자리까지
    # 걸리지 않고 내려가고, 누름 틀과 서보 받침이 드나든다
    c = cyl_y(R_ENTRY, BACK_END - 1, SEAT_Y) + cyl_y(R_BORE, SEAT_Y - 0.01, B0 + BRG_B)
    c += cyl_y(R_LIP, B0, FRONT + 1)
    if roof_y is None or roof_x is None:
        c += box(neck_x - 1, -12.0, BACK_END - 1, TUB_Y, -NECK_IN, NECK_IN)
    else:
        c += box(neck_x - 1, -12.0, roof_y, TUB_Y, -NECK_IN, NECK_IN)
        c += box(roof_x, -12.0, BACK_END - 1, roof_y + 0.01, -NECK_IN, NECK_IN)
    c += box(neck_x - 1, TUB_X, TUB_Y - 0.5, NECK_FRONT_IN, -NECK_IN, NECK_IN)
    return solid(c)


def cut_joint_features(h: Compound, window: bool, win_x: tuple[float, float] = WIN_X) -> Compound:
    """몸통에 이 관절의 멈춤 홈과 살, 영점 핀 구멍, 벽 나사 구멍을 낸다. window면 링크의 선 창(x 범위 win_x)도 낸다.

    불 연산을 하나씩 하고 그때마다 입체로 좁힌다. 한꺼번에 빼면 형상 계산이 깨지는 일이 있었다.
    """
    # 멈춤 홈: 원통 벽 0.4 밖부터 CH_R1까지, 목 폭 전체. 가운데 살만 남긴다
    h = solid(h - sector(R_DRUM + 0.4, CH_R1, 180 + WEB_HALF, 240, CH_Y, FRONT + 1))
    h = solid(h - sector(R_DRUM + 0.4, CH_R1, 120, 180 - WEB_HALF, CH_Y, FRONT + 1))
    h = solid(h - cyl_y(PIN_HOLE / 2, TUB_Y - 1, FRONT + 1, -R_PIN, 0.0))
    if window:
        h = solid(h - box(win_x[0], win_x[1], NECK_FRONT_IN - 1, FRONT + 1, -WIN_Z, WIN_Z))
    r = M3_FREE_H / 2
    for s in (1, -1):
        z0, z1 = (15.0, R_DRUM + 1) if s > 0 else (-R_DRUM - 1, -15.0)
        zs0, zs1 = (SPOT_R, R_DRUM + 1) if s > 0 else (-R_DRUM - 1, -SPOT_R)
        h = solid(h - cyl_z(r, z0, z1, 0.0, RET_SCREW_Y))
        # 머리 자리의 위아래는 ±3.5다 (±3.4면 몸통을 뽑을 때 층 가운데에서 0.02 mm라 한 층이 오갈 수 있다)
        h = solid(h - box(-3.4, 3.4, RET_SCREW_Y - 3.5, RET_SCREW_Y + 3.5, zs0, zs1))
        # 서보 받침 나사 구멍은 축 방향으로 길다 (서보가 찾은 자리 그대로 잠근다)
        slot = cyl_z(r, z0, z1, 0.0, CAR_SCREW_Y - 1.0) + cyl_z(r, z0, z1, 0.0, CAR_SCREW_Y + 1.0)
        slot += box(-r, r, CAR_SCREW_Y - 1.0, CAR_SCREW_Y + 1.0, z0, z1)
        h = solid(h - solid(slot))
        h = solid(h - box(-4.2, 4.2, CAR_SCREW_Y - 5.2, CAR_SCREW_Y + 5.2, zs0, zs1))
    h = solid(h - cyl_x(r, 15, R_DRUM + 1, RET_SCREW_Y, 0.0))
    h = solid(h - box(SPOT_R, R_DRUM + 1, RET_SCREW_Y - 3.5, RET_SCREW_Y + 3.5, -3.4, 3.4))
    if not h.is_valid or h.volume < 1000:
        raise RuntimeError(f"몸통 형상이 깨졌다: 부피 {h.volume:.1f}")
    return h


# ------------------------------------------------------------------------------------------------
# 원통 속 부품 (검정)


def bearing() -> Compound:
    """검사용 베어링. 저널은 시험편으로 고른 지름(뽑으면 줄어 안쪽 링에 맞는다)이라 그린 값이 안지름 30보다
    크다. 그린 값끼리 겹치지 않게 안쪽을 저널에 맞춘다."""
    return ring(BRG_OD / 2, max(BRG_D / 2, R_JOURNAL), B0, B0 + BRG_B)


def bolt_xz() -> list[tuple[float, float]]:
    """혼 볼트 넷. 자식 몸통 속을 지나는 선(|z| 8 ~ 12)과 머리가 비키게 45° 돌려 둔다."""
    return [polar(7.0, a) for a in (45, 135, 225, 315)]


def hub_plate() -> Compound:
    """허브 판: 혼 윗면에 앉고(가운데 r 9.8), 넓은 테가 안쪽 링 뒤를 받친다. 테 윗면을 바닥에 두고 뽑는다.

    가운데 구멍은 혼 가운데 나사 머리(지름 5.15, 높이 2.0까지)의 자리다. 판을 뚫어서, 머리가 판 두께(2.8)
    안에 들고 그 위의 저널 끝과 0.8 뜬다.
    """
    h = cyl_y(HUB_SEAT_R, SEAT, SEAT + RELIEF + 0.01) + cyl_y(R_STEP, SEAT + RELIEF, B0)
    h -= cyl_y(3.6, SEAT - 1, B0 + 1)  # 혼 가운데 나사 머리
    for x, z in bolt_xz():
        h -= cyl_y(K_M3_FREE / 2, SEAT - 1, B0 + 1, x, z)
    return solid(h)


def out_disc() -> Compound:
    """출력 원판: 저널(안쪽 링 속) + 단(안쪽 링 앞면) + 테(원통 앞 끝 위) + 멈춤 기둥 귀.

    테 윗면을 바닥에 두고 뽑는다. 혼 볼트 머리는 그 면(바닥에 닿아 평평한 면)에 앉는다.
    테 밑면 가장자리의 테두리는 원통 앞 끝과 LIP_GAP만 띄워, 테에 감기는 선이 틈으로 빠지지 않게 한다.
    """
    # 허브 판과 CLAMP_GAP만큼 띄운다 (안쪽 링을 사이에 두고 조인다)
    a = cyl_y(R_JOURNAL, B0 + CLAMP_GAP, B0 + BRG_B + 0.01)
    a += cyl_y(R_STEP, B0 + BRG_B, SEAM + 0.01)
    a += cyl_y(R_DRUM, SEAM, ATOP)
    a += ring(
        R_DRUM, R_DRUM - LIP_W, FRONT + LIP_GAP, SEAM + 0.01
    )  # 선이 틈으로 빠지지 않게 막는 테두리
    a += box(20.0, R_POST, SEAM, ATOP, -EAR_HW, EAR_HW)
    a += cyl_y(EAR_HW, SEAM, ATOP, R_POST, 0.0)  # 귀
    a += cyl_y(BOSS_R, POST_TIP, SEAM + 0.01, R_POST, 0.0)  # 멈춤 기둥 (속에 M3 나사)
    # 기둥 나사 머리 자리. 귀가 좁아서(폭 8.0, 머리 자리 6.6) 천장 첫 층이 모두 벽 선이다. 그 층이 구멍까지 덮는
    # 막이고, 반지름 쪽 틈 셋으로 갈라 선이 귀의 뿌리에서 끝까지 곧게 건넌다. 멈춤 나사를 꽂기 전에 구멍 자리의
    # 막을 뚫는다. 나사가 기둥을 눌러 두는 힘을 머리가 받으므로, 머리가 두 띠에만 앉는 긴 구멍은 쓰지 않는다
    seat = ATOP - POST_HEAD_SINK
    a -= cyl_y(K_M3_FREE / 2, POST_TIP - 1, seat - LAYER, R_POST, 0.0)
    a -= cyl_y(HEAD_POCKET_R, seat, ATOP + 1, R_POST, 0.0)
    a -= comb_ceiling(
        (R_POST, seat, 0.0),
        (0.0, -1.0, 0.0),
        (1.0, 0.0, 0.0),
        2 * HEAD_POCKET_R,
        back=HEAD_POCKET_R + 0.6,
        front=EAR_HW + 1.0,
    )
    for x, z in bolt_xz():
        a -= cyl_y(K_M3_FREE / 2, B0, ATOP + 1, x, z)
    a = solid(a)
    for x, z in RIM:
        a = solid(a - cyl_y(K_M3_FREE / 2, SEAM - 1, ATOP + 1, x, z))
        # 밑면에서 눌러 끼우는 너트 홈. 테 밑면에서만 판다 (테두리와 단은 건드리지 않는다)
        a = solid(a - hex_y_radial(x, z, SEAM, SEAM + NUT_POCKET, K_NUT_AF))
    return a


def retainer_nut_y(b0: float = B0) -> float:
    """누름 틀 너트 홈의 가운데 y. 홈의 위아래(세로 폭)가 뽑을 때의 층 경계에 오게 맞춘다 (고리 앞면 b0이
    바닥이다). 나사 축(RET_SCREW_Y)에서 0.1 안으로만 옮겨진다."""
    w = K_NUT_SLOT[0]
    return b0 - layer_snap(b0 - RET_SCREW_Y - w / 2) - w / 2


def retainer(b0: float = B0) -> Compound:
    """누름 틀: 바깥 링 뒷면을 받치는 고리와 다리 셋 (벽 나사의 너트를 품는다). 고리를 바닥에 두고 뽑는다.

    b0은 고리가 받치는 베어링 뒷면이다 (J1은 허브가 두꺼워 고리가 더 길다, parts/base.py).
    """
    r_out = R_SLIDE
    leg_in = SPOT_R - SCREW_LEN  # 나사 끝이 닿는 면 = 다리 안쪽 면 (15.0)
    y_end = b0 - layer_snap(
        b0 - RET_Y[0]
    )  # 다리 끝: 뽑을 때 층 경계에 오게 맞춘다 (0.1 안에서 옮겨진다)
    # 원기둥 면의 이음선(방위각 0°)을 나사 구멍 자리에서 45° 돌려 둔다. 이음선 위에 구멍을 내면 형상이 깨진다
    part = solid(Rot(0, 45, 0) * ring(r_out, R_STEP + 1.3, RET_Y[1], b0))
    seg = solid(Rot(0, 45, 0) * cyl_y(r_out, y_end, RET_Y[1] + 0.01))
    legs = box(-8.0, 8.0, y_end, RET_Y[1] + 0.01, leg_in, 30)
    legs += box(-8.0, 8.0, y_end, RET_Y[1] + 0.01, -30, -leg_in)
    legs += box(leg_in, 30, y_end, RET_Y[1] + 0.01, -7.0, 7.0)
    part += seg & legs
    r = K_M3_FREE_H / 2
    w, t = K_NUT_SLOT
    # 너트 홈: 다리의 옆면에서 너트를 세워 밀어 넣는다 (시험편과 같은 모양). 홈이 다리 속에 있어 너트가 안쪽으로
    # 빠지지 않고, 벽 나사가 밀어도 뒤가 받친다
    y_nut = retainer_nut_y(b0)
    for s in (1, -1):
        part -= nut_trap((0.0, y_nut, s * NUT_Z), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 8.02, w, t)
        part -= (
            cyl_z(r, 10, 25, 0.0, RET_SCREW_Y) if s > 0 else cyl_z(r, -25, -10, 0.0, RET_SCREW_Y)
        )
    part -= nut_trap((NUT_Z, y_nut, 0.0), (1.0, 0.0, 0.0), (0.0, 0.0, 1.0), 7.02, w, t)
    part -= cyl_x(r, 10, 25, RET_SCREW_Y, 0.0)
    # 앞면 라벨 판 위를 0.75 띄운다 (STS3215는 몸통이 0.17 앞이다). 고리(RET_Y[1]부터)는 그보다 위라 깎이지 않는다
    part -= box(-30, -9.0, 0, LABEL + 0.17 + 0.75, -7.7, 7.7)
    # 다리가 고리 안쪽으로 2.8 걸치지 않게, 다리 밑을 45° 원뿔로 깎는다.
    # 원뿔은 다리 안쪽 면보다 0.3 안에서 시작하고(면에 접하면 형상이 깨진다), 고리 속으로 0.05 올라간다
    r_ring = R_STEP + 1.3
    y_lo, y_hi = RET_Y[1] - (r_ring - leg_in) - 0.3, RET_Y[1] + 0.05
    cone = Cone(leg_in - 0.3, r_ring + 0.05, y_hi - y_lo)
    cut = Pos(0, (y_lo + y_hi) / 2, 0) * (Rot(-90, 0, 0) * cone)
    return solid(solid(part) - cut)


def carrier() -> Compound:
    """서보 받침: 서보 뒷면 M2 넷에 다는 틀 모양 판과 벽을 따라 선 날개 둘. 판 뒷면을 바닥에 두고 뽑는다.

    판은 뒷면의 볼록한 판과 커넥터를 비킨다. 가까운 쪽 가로대의 가운데는 터서, 플러그 옆면을 잡을 수 있다.
    판은 한 두께(PLATE_T)이고 뒷면에 파인 자리가 없다. M2 나사의 머리는 뒷면에 바로 앉고, 선은 판 위를 어디로든
    지나간다.
    """
    y0, y1 = L1 - PLATE_T, L1
    r_out = R_SLIDE
    hw = HW - 0.06  # 목 안쪽 벽과 0.4
    plate = box(-35.0, -30.2, y0, y1, -hw, hw)  # 먼 가로대
    for sg in (1, -1):
        # 나사 자리. 서보의 두께 32인 면(x -6.0까지)보다 +x로 1 mm 더 나와, 날개와 5.3 mm 폭으로 이어진다
        plate += box(-10.4, -5.1, y0, y1, min(sg * 7.5, sg * hw), max(sg * 7.5, sg * hw))
        plate += box(-35.0, -6.1, y0, y1, min(sg * 10.0, sg * hw), max(sg * 10.0, sg * hw))  # 옆 살
        wing = box(-10.4, 10.0, y0, y1, min(sg * hw, sg * 30), max(sg * hw, sg * 30))
        plate += wing & cyl_y(r_out - 0.4, y0, y1)
        # 서보 밖의 날개는 윗면을 올려 WING_T로 둔다 (윗면이 바닥에서 3.0이라 층 경계다)
        top = y0 + WING_T
        rise = box(
            -10.4, 10.0, y1 - 0.01, top, min(sg * WING_IN, sg * 30), max(sg * WING_IN, sg * 30)
        )
        plate += rise & cyl_y(r_out - 0.4, y1 - 0.01, top)
    part = plate
    leaf_in = r_out - LEAF_T
    w, t = K_NUT_SLOT
    # 돋움의 안쪽 면: 너트 홈 안쪽에 살 1.0이 남는다. 서보를 넣은 뒤에는 너트에 손이 닿지 않으므로, 벽 나사가
    # 밀어도 너트가 빠지지 않게 홈을 돋움 속에 둔다. 서보 옆면(HW)과는 2.5 mm 뜬다
    boss_in = NUT_Z - t / 2 - 1.0
    r = K_M3_FREE_H / 2
    for sg in (1, -1):
        side = box(-10.0, 10.0, y1 - 0.01, CAR_Y[1], min(sg * 10, sg * 30), max(sg * 10, sg * 30))
        part += ring(r_out, leaf_in, y1 - 0.01, CAR_Y[1]) & side
        # 돋움의 위아래: 너트 홈(세로 5.8) 위아래에 살 1.8. 판 뒷면에서 10.0, 19.4라 층 경계에 온다
        y_lo, y_hi = CAR_SCREW_Y - 4.7, CAR_SCREW_Y + 4.7
        bz = box(-4.6, 4.6, y_lo, y_hi, min(sg * boss_in, sg * 30), max(sg * boss_in, sg * 30))
        part += cyl_y(r_out, y_lo, y_hi) & bz
        # 판 뒷면을 바닥에 두고 뽑을 때 돋움 밑이 걸치지 않게 45°로 받친다
        dz = leaf_in - boss_in
        under = [
            (y_lo - dz, sg * leaf_in),
            (y_lo + 0.01, sg * leaf_in),
            (y_lo + 0.01, sg * boss_in),
        ]
        part += prism_x(under, -4.6, 4.6)
        # 너트 홈: 돋움의 +x 옆면에서 너트를 세워 밀어 넣는다 (두 돋움의 입구가 같은 쪽이다)
        part -= nut_trap(
            (0.0, CAR_SCREW_Y, sg * NUT_Z), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 4.62, w, t
        )
        # 날개는 둥글어서 입구 앞(+x)으로 갈수록 안쪽 면이 홈의 바깥 면(18.7)보다 안으로 들어온다. 너트(길이
        # 6.35)가 곧게 들어가도록, 입구 앞의 날개를 홈 바닥 높이부터 따낸다 (돋움 밑의 날개는 폭 그대로다)
        part -= box(
            NUT_GATE_X,
            10.1,
            CAR_SCREW_Y - w / 2,
            CAR_Y[1] + 1,
            min(sg * (NUT_Z - t / 2), sg * 30),
            max(sg * (NUT_Z - t / 2), sg * 30),
        )
        part -= (
            cyl_z(r, 10, 25, 0.0, CAR_SCREW_Y) if sg > 0 else cyl_z(r, -25, -10, 0.0, CAR_SCREW_Y)
        )
    # 뒷덮개 나사가 물리는 기둥: 나사 끝(덮개의 머리 자리에서 COVER_SCREW)보다 0.4 길다. 구멍은 기둥 윗면에서
    # 끝난다. 그 위의 돋움까지 파면 구멍이 돋움의 안쪽 모서리를 0.7 mm 조각으로 떼어 놓아, 그 조각이 허공에서
    # 뽑히기 시작한다 (슬라이서가 떠 있는 영역이라고 경고한다)
    pillar = BACK_OUT + COVER_HEAD_SINK + COVER_SCREW + 0.4
    for x, z in COVER_BOSS:
        part += cyl_y(3.2, y1 - 0.01, pillar, x, z)
        part -= cyl_y(M3_TAP / 2, y0 - 1, pillar + 0.01, x, z)
    for x, z in BACK_HOLES:
        part -= cyl_y(M2_HOLE_R, y0 - 1, y1 + 1, x, z)
    return solid(part)


def back_cover(neck_x: float, end_skirt: bool) -> Compound:
    """뒷덮개: 판 + 원통 뒤 끝을 겹쳐 가리는 테 + 서보 받침에 닿는 나사 기둥 둘. 바깥 면을 바닥에 두고 뽑는다.

    서보 받침에 조이므로 받침을 따라 축 방향으로 조금 다르게 앉는다. 테가 그 틈을 가린다.
    end_skirt가 거짓이면 목 끝 쪽 테가 없다 (손목 몸통: 앞 관절의 출력 원판을 넘지 않는다).
    """
    y_skirt = BACK_END + COVER_SKIRT
    x_out = neck_x - 2.0 if end_skirt else neck_x
    x_in = neck_x - 0.4 if end_skirt else neck_x - 1.0
    outline = cyl_y(COVER_R, BACK_OUT, y_skirt)
    outline += box(x_out, -12.0, BACK_OUT, y_skirt, -NECK_HW - 2.0, NECK_HW + 2.0)
    inner = cyl_y(R_DRUM + 0.4, BACK_IN, y_skirt + 1)
    inner += box(x_in, -12.0, BACK_IN, y_skirt + 1, -NECK_HW - 0.4, NECK_HW + 0.4)
    c = solid(outline - inner)
    # 나사 머리 자리의 천장: 긴 구멍 한 층과 네모 한 층으로 벽 선이 곧게 건넌다 (뚫을 것이 없다). 덮개 나사는 덮개만
    # 잡으므로 머리가 긴 구멍 양옆의 두 띠에 앉아도 된다
    seat = BACK_OUT + COVER_HEAD_SINK
    for x, z in COVER_BOSS:
        c = solid(c + cyl_y(COVER_BOSS_R, BACK_IN - 0.01, L1 - PLATE_T, x, z))
        c = solid(c - cyl_y(HEAD_POCKET_R, BACK_OUT - 0.01, seat, x, z))
        c = solid(
            c
            - slot_ceiling(
                (x, seat, z), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0), K_M3_FREE, 2 * HEAD_POCKET_R
            )
        )
        c = solid(c - cyl_y(K_M3_FREE / 2, seat + 2 * LAYER - 0.01, L1 - PLATE_T + 1, x, z))
    return c


# ------------------------------------------------------------------------------------------------
# 조립 검사에 쓰는 부품 목록


@dataclass(frozen=True)
class Part:
    name: str
    shape: Compound
    density: float = 0.0  # [g/mm³] 나사와 너트만 준다
    moving: bool = False  # 자식과 함께 도는 부품
    color: str = "body"  # 3D 페이지의 색 구실

    @property
    def mass_g(self) -> float:
        return self.shape.volume * self.density


def _screw_y(x: float, z: float, seat: float, length: float) -> Compound:
    """머리가 +y 쪽에 있는 M3 둥근머리 볼트 (머리 밑면이 seat)."""
    return solid(cyl_y(1.5, seat - length, seat, x, z) + cyl_y(2.85, seat, seat + 1.65, x, z))


def wall_screws() -> list[Part]:
    """벽 나사 M3 × 8 다섯: 1·3·5번은 누름 틀(앞줄), 2·4번은 서보 받침(뒷줄)."""
    out: list[Part] = []

    def one(axis: str, s: int, y: float) -> None:
        if axis == "z":
            if s > 0:
                shape = cyl_z(1.5, SPOT_R - SCREW_LEN, SPOT_R, 0.0, y) + cyl_z(
                    2.85, SPOT_R, SPOT_R + 1.65, 0.0, y
                )
            else:
                shape = cyl_z(1.5, -SPOT_R, -(SPOT_R - SCREW_LEN), 0.0, y)
                shape += cyl_z(2.85, -SPOT_R - 1.65, -SPOT_R, 0.0, y)
        else:
            shape = cyl_x(1.5, SPOT_R - SCREW_LEN, SPOT_R, y, 0.0) + cyl_x(
                2.85, SPOT_R, SPOT_R + 1.65, y, 0.0
            )
        out.append(Part(f"벽 나사 {len(out) + 1}", solid(shape), STEEL, color="metal"))

    for s in (1, -1):
        one("z", s, RET_SCREW_Y)
        one("z", s, CAR_SCREW_Y)
    one("x", 1, RET_SCREW_Y)
    return out


def inner_hardware(model: ServoModel) -> list[Part]:
    """혼 볼트, 멈춤 나사, 너트, 서보 뒷면 나사, 덮개 나사 (검사와 3D 페이지용)."""
    dy = servo_dy(model)
    out: list[Part] = []
    for k, (x, z) in enumerate(bolt_xz()):
        out.append(Part(f"혼 볼트 M3x20 {k + 1}", _screw_y(x, z, ATOP, 20.0), STEEL, True, "metal"))
    out.append(
        Part(
            "멈춤 나사 M3x10",
            _screw_y(R_POST, 0.0, ATOP - POST_HEAD_SINK, 10.0),
            STEEL,
            True,
            "metal",
        )
    )
    nut = hex_y(R_POST, 0.0, POST_TIP - NUT_T, POST_TIP, 5.4) - cyl_y(
        1.5, POST_TIP - 3, POST_TIP + 1, R_POST, 0.0
    )
    out.append(Part("너트 (멈춤 기둥)", solid(nut), STEEL, True, "metal"))
    for k, (x, z) in enumerate(RIM):
        nut = hex_y(x, z, SEAM, SEAM + NUT_T, 5.4) - cyl_y(1.5, SEAM - 1, SEAM + NUT_T + 1, x, z)
        out.append(Part(f"너트 (출력 원판) {k + 1}", solid(nut), STEEL, True, "metal"))
    for k, (x, z) in enumerate(BACK_HOLES):
        y = L1 - PLATE_T + dy  # 머리 밑면 (판의 뒷면)
        shape = cyl_y(1.0, y, y + M2_SCREW, x, z) + cyl_y(2.0, y - 1.6, y, x, z)
        out.append(Part(f"서보 뒷면 나사 {k + 1}", solid(shape), STEEL, False, "metal"))
    for k, (x, z) in enumerate(COVER_BOSS):
        y = BACK_OUT + COVER_HEAD_SINK + dy
        shape = cyl_y(1.5, y, y + COVER_SCREW, x, z) + cyl_y(2.85, y - 1.65, y, x, z)
        out.append(Part(f"덮개 나사 M3x12 {k + 1}", solid(shape), STEEL, False, "metal"))
    seat = NUT_Z + K_NUT_SLOT[1] / 2  # 조인 너트가 붙는 홈의 바깥 면
    for sg in (1, -1):
        z0, z1 = (seat - NUT_T, seat) if sg > 0 else (-seat, -seat + NUT_T)
        out.append(
            Part(
                "너트 (누름 틀)", hex_z(0.0, RET_SCREW_Y, z0, z1, 5.4), STEEL * 0.72, False, "metal"
            )
        )
        nut_c = solid(Pos(0, dy, 0) * hex_z(0.0, CAR_SCREW_Y, z0, z1, 5.4))
        out.append(Part("너트 (서보 받침)", nut_c, STEEL * 0.72, False, "metal"))
    nut_x = hex_x(RET_SCREW_Y, 0.0, seat - NUT_T, seat, 5.4)
    out.append(Part("너트 (누름 틀)", nut_x, STEEL * 0.72, False, "metal"))
    return out


def inner_parts(
    model: ServoModel, neck_x: float, end_skirt: bool, cover: Compound | None = None
) -> list[Part]:
    """원통 속 부품과 서보, 출력 원판, 벽 나사 (몸통과 자식 몸통은 빼고).

    cover를 주면 뒷덮개를 그것으로 바꾼다 (손목 2: 선이 나오는 자리가 다른 덮개, parts/wrist.py).
    """
    dy = servo_dy(model)
    sv = servo(model)
    lid = back_cover(neck_x, end_skirt) if cover is None else cover
    parts = [
        Part("베어링 6806", bearing(), color="metal"),
        Part("허브 판", hub_plate(), moving=True, color="cap"),
        Part("출력 원판", out_disc(), moving=True, color="cap"),
        Part("누름 틀", retainer(), color="frame"),
        Part("서보 받침", solid(Pos(0, dy, 0) * carrier()), color="frame"),
        Part("뒷덮개", solid(Pos(0, dy, 0) * lid), color="cap"),
        Part("서보 몸통", sv["body"], color="servo"),
        Part("혼", sv["horn"], moving=True, color="metal"),
        Part("플러그", sv["plugs"], color="plug"),
        Part("선 자리", sv["wires"], color="wire"),
    ]
    return parts + wall_screws()


def side_nut_slots() -> list[tuple[str, Point, Point, Point]]:
    """옆 홈에 세워 넣는 너트 다섯: (이름, 너트 가운데, 볼트 방향, 입구 쪽 방향). 좌표는 부품 자신의 것이다
    (서보 받침은 조립하면 servo_dy만큼 옮겨진다). 형상은 만들지 않는다 (작업 안내가 너트를 미는 방향으로 쓴다)."""
    y_ret = retainer_nut_y()
    x_dir, z_dir = (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)
    return [
        ("누름 틀 +z", (0.0, y_ret, NUT_Z), z_dir, x_dir),
        ("누름 틀 -z", (0.0, y_ret, -NUT_Z), z_dir, x_dir),
        ("누름 틀 +x", (NUT_Z, y_ret, 0.0), x_dir, z_dir),
        ("서보 받침 +z", (0.0, CAR_SCREW_Y, NUT_Z), z_dir, x_dir),
        ("서보 받침 -z", (0.0, CAR_SCREW_Y, -NUT_Z), z_dir, x_dir),
    ]


def side_nuts() -> list[tuple[str, Compound, Point, Point, Point]]:
    """옆 홈에 세워 넣는 너트 다섯: (이름, 홈이 있는 부품, 너트 가운데, 볼트 방향, 입구 쪽 방향)."""
    ret, car = retainer(), carrier()
    return [
        (name, ret if name.startswith("누름 틀") else car, c, a, s)
        for name, c, a, s in side_nut_slots()
    ]


def nut_entry_overlap(
    part: Compound, center: Point, axis: Point, slide: Point, reach: float = 14.0, step: float = 0.5
) -> float:
    """M3 너트(맞변 5.5, 모서리 6.35, 두께 2.4)를 입구 밖 reach에서 자리까지 곧게 밀 때 부품과 겹치는 가장 큰
    부피 [mm³]. 0이면 너트가 기울이지 않고 들어간다."""
    nut = extrude(RegularPolygon(5.5 / 2, 6, major_radius=False), amount=NUT_T / 2, both=True)
    worst = 0.0
    for k in range(int(reach / step) + 1):
        at = (
            center[0] + slide[0] * k * step,
            center[1] + slide[1] * k * step,
            center[2] + slide[2] * k * step,
        )
        placed = nut.moved(Location(Plane(origin=at, x_dir=slide, z_dir=axis)))
        worst = max(worst, solid(part & placed).volume)
    return worst


# 서로 닿거나 겹쳐 그린 것으로 치는 쌍 (서보 안의 조각끼리)
_SAME_BODY = {
    frozenset(p) for p in (("서보 몸통", "혼"), ("플러그", "서보 몸통"), ("플러그", "선 자리"))
}


def overlaps(parts: list[Part], tol: float = 0.05) -> dict[tuple[str, str], float]:
    """출력 부품과 서보끼리 겹친 부피 [mm³] (tol 넘는 것만). 나사와 너트는 구멍 속에 있어 보지 않는다."""
    solids = [p for p in parts if p.density == 0.0]
    out: dict[tuple[str, str], float] = {}
    for i, a in enumerate(solids):
        for b in solids[i + 1 :]:
            if frozenset((a.name, b.name)) in _SAME_BODY:
                continue
            if far_apart(a.shape, b.shape, 0.0):
                continue
            v = solid(a.shape & b.shape).volume if a.shape.distance_to(b.shape) < 1e-6 else 0.0
            if v > tol:
                out[(a.name, b.name)] = v
    return out
