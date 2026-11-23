"""J1 관절(베이스 몸통, 돌림판, J1 몸통): 뽑을 수 있는 형상인지, 조립했을 때 겹치지 않는지, 멈춤과 영점 핀이 맞는지."""

from __future__ import annotations

import pytest

from horibot_design.parts import base as b
from horibot_design.parts import joint as j
from horibot_design.parts import shoulder as sh
from horibot_design.printcheck import (
    air_walls,
    mid_layer_levels,
    overhangs,
    printed_mass_g,
    wall_settings,
)
from horibot_design.studies import ur_arm
from horibot_design.studies.packaging import MM
from horibot_design.values import ServoModel, load_measured

BED = 256.0  # 출력 범위 [mm]
HORN_MASS_G = 2.0  # STS 서보의 출력 혼과 가운데 나사 (실측)


@pytest.fixture(scope="module")
def parts() -> list[j.Part]:
    return b.assembly("STS3215")


@pytest.mark.slow
def test_printed_parts_are_single_valid_solids_without_steep_overhang() -> None:
    printed = b.printed()
    assert set(b.LABELS) == set(printed)  # 출력 안내의 한글 이름
    for name, (shape, up, _color, _preset) in printed.items():
        assert shape.is_valid, name
        assert len(shape.solids()) == 1, name
        bb = shape.bounding_box()
        assert max(bb.size.X, bb.size.Y, bb.size.Z) < BED, name
        # 바닥에 닿지 않고 아래를 보는 면은 양쪽이 붙은 다리와 작은 가로 구멍 윗변뿐이어야 한다
        over = overhangs(shape, up)
        # (J1 몸통은 작은 가로 구멍이 많다: 돌림판 나사 넷, 벽 나사, 선반 나사 둘의 윗변이 90 mm²쯤이다)
        assert over.total - over.flat < 100.0, (name, over.total, over.flat)


@pytest.mark.slow
def test_base_parts_do_not_overlap(parts: list[j.Part]) -> None:
    assert b.overlaps([p for p in parts if "핀" not in p.name]) == {}


@pytest.mark.slow
def test_sts3250_fits_the_same_seat() -> None:
    # J1 서보 자리는 STS3250도 들어간다 (0028: 정밀도가 모자라면 바꿀 수 있게)
    parts = b.assembly("STS3250")
    assert b.overlaps([p for p in parts if "핀" not in p.name]) == {}


@pytest.mark.slow
def test_turntable_turns_freely_and_stops_on_the_web(parts: list[j.Part]) -> None:
    # 프로그램 한계까지는 도는 부품이 어디에도 닿지 않고, 멈춤 각에서 기둥이 살에 닿는다
    for q in (0.0, 90.0, j.SOFT_DEG, -90.0, -j.SOFT_DEG):
        gaps = b.clearances(parts, q)
        assert min(gaps.values()) > 0.4, (q, gaps)
    for q in (j.STOP_DEG, -j.STOP_DEG):
        gaps = b.clearances(parts, q)
        assert gaps[("돌림판", "베이스 몸통")] < 0.05, (q, gaps)


@pytest.mark.slow
def test_zero_pin_sits_in_both_holes(parts: list[j.Part]) -> None:
    core = next(p for p in parts if p.name == "베이스 몸통")
    body = next(p for p in parts if p.name == "J1 몸통")
    table = next(p for p in parts if p.name == "돌림판")
    pin = next(p for p in parts if "핀" in p.name)
    # 핀(지름 3.0)이 구멍(세로 3.35, 가로 3.30) 속에 있으면 벽과의 틈이 0.15 ~ 0.18이다
    assert 0.1 < pin.shape.distance_to(core.shape) < 0.25
    assert 0.1 < pin.shape.distance_to(body.shape) < 0.25
    # 돌림판은 핀을 잡지 않는다 (영점은 J1 몸통과 베이스 몸통의 구멍이 정한다)
    assert pin.shape.distance_to(table.shape) > 0.4


@pytest.mark.parametrize("model", ["STS3215", "STS3250"])
def test_screws_engage_fully_and_do_not_bottom(model: ServoModel) -> None:
    checks = b.screw_checks(model)
    # 혼 나사 M3 × 8: 혼의 테(2.5)에 거의 다 물리고, 끝이 서보 앞면에 닿지 않는다
    assert checks["혼 나사가 혼 테(2.5)에 물린 길이"] >= 2.4
    assert checks["혼 나사 끝과 서보 앞면 사이"] > 0.25
    # 조임 볼트 M3 × 45: 너트에 다 물리고, 끝이 허브 속에서 끝난다 (혼에 닿지 않는다)
    assert checks["조임 볼트가 너트(2.4)에 물린 길이"] == pytest.approx(j.NUT_T)
    assert checks["조임 볼트 끝과 허브 밑면 사이"] > 0.5
    # 머리가 돌림판 윗면 아래에 있고, 머리 자리 밑에 위 안쪽 링까지 살이 2 mm 넘게 남는다
    assert checks["조임 볼트 머리 윗면과 돌림판 윗면 사이"] > 1.0
    assert checks["조임 볼트 머리 자리와 위 안쪽 링 윗면 사이의 살"] >= 2.0
    # J1 몸통 나사 M3 × 12: 바닥판 속 너트에 다 물리고, 끝이 바닥판 위로 나오지 않는다
    assert checks["J1 몸통 나사가 너트(2.4)에 물린 길이"] == pytest.approx(j.NUT_T)
    assert checks["J1 몸통 나사 끝과 바닥판 윗면 사이"] >= 0.0
    assert checks["J1 몸통 바닥판과 어깨 서보 먼 끝 사이"] > 2.0


def test_stack_matches_the_skeleton() -> None:
    # 돌림판 윗면이 어깨 서보의 먼 끝과 J1 몸통의 바닥판 아래에 있다
    assert b.NECK_X == pytest.approx(-(b.SHOULDER_H - b.TOP_H))
    assert (b.SHOULDER_H - sh.FAR) - (b.TOP_H + b.FLOOR_T) > 2.0
    # 두 베어링의 가운데 사이가 30 mm다 (0028)
    assert (b.B1 - b.B0) == pytest.approx(30.0)
    # 저널 끝이 허브에 닿지 않는다: 베어링 폭 공차(둘 합쳐 0.24)와 출력 오차를 넘는 틈
    assert b.JOURNAL_GAP >= 0.5
    # 멈춤 나사 끝이 홈 바닥에 닿지 않고, 영점 핀이 살 속에 3.5 넘게 들어간다
    assert (b.ATOP - 1.8 - 10.0) - b.CH_Y > 0.8
    assert b.FRONT - (b.ATOP + j.EAR_T + 3.0 - 20.0) > 3.5


def test_bearing_bore_is_tight_only_at_the_two_seats() -> None:
    # 자리 둘(각 6.6 mm)만 시험편 값이고 그 아래와 사이는 0.2 넓다. 속 부품(누름 틀, 서보 받침)은 그보다 0.3 작다
    assert j.BEARING_ENTRY - j.BEARING_BORE == pytest.approx(0.2)
    assert b.SEAT_IN == pytest.approx(0.4)
    core = b.core().solids()[0]
    for y, r in (
        (b.Y_FLOOR + 5.0, j.R_ENTRY),
        (b.B0 + 3.5, j.R_BORE),
        ((b.B0 + j.BRG_B + b.B1) / 2, j.R_ENTRY),
        (b.B1 + 3.5, j.R_BORE),
    ):
        # 서보 먼 끝 자리(-x)와 벽 나사 구멍(±z, +x)을 피한 방위각에서 본다
        x, z = j.polar(r, 45.0)
        assert not core.is_inside((x * (1 - 0.03 / r), y, z * (1 - 0.03 / r))), y
        assert core.is_inside((x * (1 + 0.03 / r), y, z * (1 + 0.03 / r))), y


def test_black_parts_use_the_black_coupon_values() -> None:
    assert (b.SLOT_W, b.SLOT_H) == j.K_NUT_LYING == (5.9, 2.8)
    # 허브: 너트 위 살 1.6 + 옆 홈 2.8 + 밑 살 1.6
    assert b.HUB_BODY - b.NUT_ROOF - b.SLOT_H == pytest.approx(1.6)


def test_j1_body_nuts_drop_in_from_above_as_printed() -> None:
    # 흰색은 너트가 서는 홈의 값이 떨어뜨려 넣는 방향뿐이다 (fit1_nut_drops 5.6 × 2.8)
    white = load_measured().fit["pla_basic_white"]
    assert (white.m3_nut_drop_width_mm, white.m3_nut_drop_thickness_mm) == b.FOOT_NUT_SLOT
    w, t = b.FOOT_NUT_SLOT
    # 너트 밑에 살 6.0, 위에 살 1.0. 돌림판 밑에서 올라온 M3 × 12가 너트를 1 넘게 지나고 바닥판 속에서 끝난다
    assert b.FLOOR_T - b.FOOT_SEAT - t == pytest.approx(1.0)
    tip = b.BRIM_T - b.FOOT_RECESS - b.FOOT_SCREW  # 돌림판 윗면에서 나사 끝까지 (음수: 위로)
    assert -tip - (b.FOOT_SEAT + j.NUT_T) >= 1.0
    assert -tip <= b.FLOOR_T
    for (_x, y, z), slide, length in b.foot_nut_slots():
        assert slide == (0.0, -1.0, 0.0)  # J1 몸통을 뽑을 때 위
        r = (z * z + (y - sh.J1_AXIS_Y) ** 2) ** 0.5
        assert r == pytest.approx(b.R_FOOT)
        if y < sh.J1_AXIS_Y:  # 뒷덮개 쪽: 바닥판의 뒤 가장자리까지 열린다
            assert y - length < sh.BACK_END
        else:  # 위팔 쪽: 홈이 가운데 구멍(반지름 16) 속으로 다 열린다
            assert abs(z) + w / 2 < b.FOOT_HOLE - 2.0
            end = y - length - sh.J1_AXIS_Y
            assert (end**2 + (abs(z) + w / 2) ** 2) ** 0.5 < b.FOOT_HOLE
    # 나사 넷이 반 바퀴 대칭이 아니다: J1 몸통이 돌림판에 한 방향으로만 달린다
    spots = {(round(x, 3), round(z, 3)) for x, z in b.FOOT}
    assert {(-x, -z) for x, z in spots} != spots


@pytest.mark.slow
def test_j1_body_nuts_fall_into_their_slots() -> None:
    body = b.j1_body()
    for center, slide, length in b.foot_nut_slots():
        assert j.nut_entry_overlap(body, center, (1.0, 0.0, 0.0), slide, reach=length) < 0.01
    # 홈 위(어깨 서보 쪽)에 살이 남아 너트가 목 속으로 빠지지 않는다
    solid_body = body.solids()[0]
    for (x, y, z), _slide, _length in b.foot_nut_slots():
        assert solid_body.is_inside((x + b.FOOT_NUT_SLOT[1] / 2 + 0.5, y + 2.0, z + 2.2))


@pytest.mark.slow
def test_no_air_walls_or_mid_layer_faces_block_the_printed_parts() -> None:
    for name, (shape, up, _color, preset) in b.printed().items():
        found = [a for a in air_walls(shape, up, wall_settings(preset)) if a.blocking]
        assert found == [], (name, [a.describe() for a in found])
        assert mid_layer_levels(shape, up) == [], name


@pytest.mark.slow
def test_nut_seat_ceilings_are_closed_by_one_layer(parts: list[j.Part]) -> None:
    # 조인 너트가 붙는 홈의 천장은 막 한 층이 나사 구멍까지 덮는다 (구멍이 뚫린 천장은 벽 고리가 공중에 뽑힌다)
    core = next(p for p in parts if p.name == "베이스 몸통").shape.solids()[0]
    for x, z in b.plate_xz():
        roof = b.Y_FLOOR + b.PLATE_NUT_UP
        assert core.is_inside((x, roof - b.SKIN / 2, z))
        assert not core.is_inside((x, roof - b.SKIN - 0.5, z))
        assert not core.is_inside((x, roof + 0.5, z))
    hub = next(p for p in parts if p.name == "허브").shape.solids()[0]
    for x, z in b.clamp_xz():
        roof = b.B0 - b.NUT_ROOF
        assert hub.is_inside((x, roof + b.SKIN / 2, z))
        assert not hub.is_inside((x, roof + b.SKIN + 0.5, z))
        assert not hub.is_inside((x, roof - 0.5, z))


def test_j1_body_screws_sit_close_to_the_journal() -> None:
    # J1 몸통 나사가 돌림판의 단(반지름 16.5)에서 5 mm 안에 있다: 얇은 테가 휘는 길이가 짧다
    assert b.R_FOOT - j.R_STEP <= 5.0
    # 나사 머리 자리가 단과 겹치지 않고, 조임 볼트의 와셔(반지름 4)와 바닥판 가운데 구멍을 비킨다
    assert b.R_FOOT - 3.1 > j.R_STEP
    assert b.R_FOOT - 2.85 > b.FOOT_HOLE
    assert b.R_CLAMP + 4.0 < b.FOOT_HOLE


def test_clamp_bolt_heads_sit_in_pockets() -> None:
    # 가진 M3 × 45로 문다: 머리 자리 깊이가 볼트 끝이 너트를 지나고 허브 속에서 끝나는 범위(4.5 ~ 6.5)의 가운데다
    tip = b.ATOP - b.SINK + b.WASHER_T - b.CLAMP_LEN
    assert (b.B0 - b.NUT_ROOF - j.NUT_T) - tip >= 0.9  # 너트를 지나간 길이
    assert tip - (b.B0 - b.HUB_BODY) >= 0.9  # 허브 밑면까지 남은 길이
    # 천장이 0.2 mm 층의 경계에 온다 (막 한 층이 고르게 걸친다)
    assert b.SINK / 0.2 == pytest.approx(round(b.SINK / 0.2))
    assert pytest.approx(0.2) == b.SKIN
    # 머리 자리가 J1 몸통 바닥판의 가운데 구멍 안에 있고, 넓힌 단에 살 1.5 넘게 싸인다
    assert b.R_CLAMP + b.SINK_R < b.FOOT_HOLE
    assert b.STEP_WIDE_R - (b.R_CLAMP + b.SINK_R) >= 1.5
    # 넓힌 단은 몸통의 턱 구멍과 1.5 넘게 뜨고, 볼트는 혼의 바깥을 지난다
    assert j.R_LIP - b.STEP_WIDE_R >= 1.5
    assert b.R_CLAMP - 1.5 > j.HORN_R
    # 머리 자리끼리 떨어져 있다
    assert b.R_CLAMP * 2**0.5 - 2 * b.SINK_R > 5.0


@pytest.mark.slow
def test_clamp_bolt_holes_are_closed_by_one_layer(parts: list[j.Part]) -> None:
    # 머리 자리의 천장은 막 한 층이 볼트 구멍까지 덮는다 (뽑을 때 줄이 끊기지 않고 걸친다)
    table = next(p for p in parts if p.name == "돌림판").shape.solids()[0]
    for x, z in b.clamp_xz():
        assert table.is_inside((x, b.ATOP - b.SINK - b.SKIN / 2, z))
        assert not table.is_inside((x, b.ATOP - b.SINK + 0.5, z))
        assert not table.is_inside((x, b.ATOP - b.SINK - b.SKIN - 0.5, z))


def test_plate_bolts_and_clamp_zones() -> None:
    checks = b.screw_checks("STS3215")
    # 판 나사 M3 × 10: 몸통 속 너트에 다 물리고, 구멍 바닥에 닿지 않는다
    assert checks["판 나사가 너트(2.4)에 물린 길이"] == pytest.approx(j.NUT_T)
    assert checks["판 나사 끝과 나사 구멍 바닥 사이"] >= 0.5
    assert checks["판 나사 머리 자리 위에 남은 판의 살"] >= 1.5
    # 머리와 와셔가 판 밑면 밖으로 나오지 않는다 (책상에 닿지 않는다)
    assert b.PLATE_RECESS >= 1.65 + b.WASHER_T
    # 가진 금속 클램프(다 벌려 55.88 mm)가 책상(40 mm)과 날개를 같이 문다: 조일 틈이 5 mm 넘게 남는다
    assert checks["클램프가 벌어지는 틈에서 책상과 날개를 뺀 나머지"] >= 5.0
    # 판 나사는 서보 먼 끝 자리(|z| 12.7 안)와 베어링 구멍(반지름 21) 밖의 두꺼운 벽에 있다
    for x, z in b.plate_xz():
        assert abs(z) - 1.6 > j.NECK_IN + 3.0
        assert (x * x + z * z) ** 0.5 - 1.6 > j.R_BORE + 3.0
    # 클램프가 무는 자리는 몸통을 비키고 날개의 두께 10 mm인 띠 안에 있다 (가장자리에서 25 mm 들어온 곳까지)
    x_in = -b.EDGE + b.CLAMP_IN + b.CLAMP_HW
    near = (x_in**2 + (b.CLAMP_Z - b.CLAMP_HW) ** 2) ** 0.5
    assert near > b.R_CORE + b.SEAT_GAP + 2.0
    assert b.WING_IN <= b.CLAMP_Z - b.CLAMP_HW
    assert x_in <= b.WING_X
    # 위 턱은 더 두꺼운 띠(몸통 쪽), 안쪽 기둥과 2 mm 넘게 떨어져 있어 뒤에서 끼우고 뺀다. 책상 밖으로 나온 뒤 판
    # (서보 어댑터 자리)은 클램프의 등에서 30 mm 넘게 안쪽이다
    jaw = (b.CLAMP_Z - b.JAW_HW, b.CLAMP_Z + b.JAW_HW)
    assert jaw[0] - b.STIFF_OUT >= 2.0
    assert jaw[0] - (b.PI_CZ - b.POST_DZ + b.POST_R) >= 2.0
    assert jaw[0] - b.DECK_HW >= 30.0
    assert jaw[1] + 2.0 <= b.THICK_OUT
    # 위 턱은 파이 기판 밑에 있다 (파이 판이 덮는다)
    assert b.PI_IN <= jaw[0] and jaw[1] <= b.PI_IN + b.PI_BOARD[1]
    assert b.WING_T + b.JAW_T + 2.0 <= b.TRAY_UNDER
    # 부속 자리는 더 두꺼운 띠 위, 파이 판과 클램프 밖이다
    assert max(b.ACC_Z) + 3.2 <= min(b.STIFF_OUT, b.PI_CZ - b.TRAY_HZ)
    assert min(b.ACC_Z) - 3.2 >= b.WING_IN
    # 가진 금속 클램프(사진으로 어림한 치수)는 그 띠 안에 든다: 위 턱의 끝과 폭, 나사 축
    assert b.JAW_REACH <= b.CLAMP_IN + b.CLAMP_HW - 2.0
    assert b.JAW_HW <= b.CLAMP_HW - 2.0
    assert b.JAW_AXIS < b.JAW_REACH
    # 몸통이 앉는 턱은 책상 가장자리 안에 있다
    assert b.EDGE >= b.R_CORE + b.SEAT_GAP + b.RING_W


@pytest.mark.slow
def test_plate_seats_the_core(parts: list[j.Part]) -> None:
    core = next(p for p in parts if p.name == "베이스 몸통")
    plate = next(p for p in parts if p.name == "책상에 무는 판")
    # 몸통이 판의 바닥에 앉는다 (닿는다). 옆으로는 턱과 0.2 mm 틈이다
    assert core.shape.distance_to(plate.shape) < 0.01
    solid = plate.shape.solids()[0]
    for x, z in b.plate_xz():  # 머리 자리와 나사 구멍 사이에 막 한 층이 있다
        assert solid.is_inside((x, b.Y_DESK + b.PLATE_RECESS + b.SKIN / 2, z))
        assert not solid.is_inside((x, b.Y_DESK + b.PLATE_RECESS - 0.5, z))
    # 선이 나가는 길이 턱에 뚫려 있다
    assert not solid.is_inside((-(b.R_CORE + 3.0), b.Y_FLOOR + 3.0, 0.0))


def test_wires_wrap_at_the_d405_bend_radius() -> None:
    # 선이 감기는 면의 반지름: 지름 3.2 mm D405 선의 가운데가 40 mm 넘는 곳을 돈다 (design.md의 D405 케이블)
    assert b.R_TABLE + 1.6 >= 40.0
    # 감기는 면의 높이 (몸통과의 틈 + 돌림판 테 + J1 몸통 바닥판)에 D405 선과 서보 선 둘이 나란히 든다
    assert j.GAP + b.BRIM_T + j.EAR_T >= 4.5 + 2 * 3.0


@pytest.mark.slow
def test_arm_model_uses_the_j1_stack_and_masses(parts: list[j.Part]) -> None:
    # 팔 전체 모델의 돌림판 높이와 J1 쪽 무게가 이 CAD의 값이다
    assert ur_arm.TABLE_TOP == pytest.approx(b.TOP_H * MM)
    assert ur_arm.shoulder_room(ur_arm.Concept()) > 2.0 * MM
    printed = {
        name: printed_mass_g(shape, up, preset)
        for name, (shape, up, _color, preset) in b.printed().items()
    }
    moving = sum(p.mass_g for p in parts if p.density > 0 and p.moving)
    table = sum(printed[f"base_{n}"] for n in ("turntable", "hub", "spacer")) + moving + HORN_MASS_G
    assert table == pytest.approx(ur_arm.J1_TABLE_MASS * 1000, abs=2.0)
    house = printed_mass_g(sh.house(), (0.0, -1.0, 0.0))
    assert printed["j1_body"] - house == pytest.approx(ur_arm.J1_FOOT_MASS * 1000, abs=2.0)


def test_wings_carry_the_pi_trays_inside_the_bed_and_the_desk() -> None:
    plate = b.plate()
    bb = plate.bounding_box()
    # 판은 좌우 대칭이고 출력 판(256 mm)에 든다
    assert bb.max.Z == pytest.approx(-bb.min.Z, abs=1e-6) == pytest.approx(b.WING_Z)
    assert 2 * b.WING_Z < BED - 4.0 and bb.size.X < BED - 4.0
    # 날개는 책상 뒤 가장자리까지고, 두 날개 사이의 뒤 판(서보 어댑터 자리)만 그 밖으로 나온다
    assert bb.min.X == pytest.approx(b.DECK_END)
    solid = plate.solids()[0]
    assert not solid.is_inside((-b.EDGE - 1.0, b.Y_DESK + 1.0, b.DECK_HW + 5.0))
    # 오른쪽 파이의 젠더(가정한 크기)와 몸통 사이에 3 mm가 남는다
    assert b.PI_IN - b.USBC_OUT - b.GENDER_OUT - b.R_CORE == pytest.approx(b.BODY_GAP)
    # 기둥 여덟은 날개마다 넷, 기판 가운데에 대해 대칭이다. 속의 조여 넣는 구멍이 위로 열려 있다
    posts = b.post_xz()
    assert len(posts) == 8 and {(x, -z) for x, z in posts} == set(posts)
    top = b.Y_DESK + b.TRAY_UNDER
    for x, z in posts:
        assert not solid.is_inside((x, top - 1.0, z))
        assert solid.is_inside((x + b.POST_R - 0.6, top - 1.0, z))
        assert solid.is_inside((x, top - b.POST_TAP - 1.0, z)) or top - b.POST_TAP - 1.0 < b.Y_FLOOR
    # 몸통 쪽을 두껍게 해서, 클램프를 파이 밑으로 옮기고도 날개가 앞선 배치만큼만 휜다 (어림)
    bend = b.wing_bend()
    now, thin, before = bend.values()
    assert now < 1.1 * before and thin > 3.0 * now


def test_back_deck_carries_the_adapter_behind_the_core() -> None:
    plate = b.plate().solids()[0]
    # 돋움 넷은 어댑터의 고정 구멍 자리다 (긴 변이 옆 쪽, 좌우 대칭). 조여 넣는 구멍이 위로 열려 있고 밑은 막혀 있다
    holes = b.adapter_xz()
    assert len(holes) == 4 and {(x, -z) for x, z in holes} == set(holes)
    xs, zs = sorted({x for x, _ in holes}), sorted({z for _, z in holes})
    assert (zs[1] - zs[0], xs[1] - xs[0]) == pytest.approx(b.ADAPTER_HOLES)
    top = b.Y_FLOOR + b.ADAPTER_SEAT
    for x, z in holes:
        assert not plate.is_inside((x, top - 1.0, z))
        assert plate.is_inside((x + b.M2_BOSS_R - 0.3, top - 1.0, z))
        assert plate.is_inside((x, b.Y_DESK + 0.5, z))
        assert abs(z) + b.M2_BOSS_R < b.DECK_RIB[0]
    # 어댑터는 책상 뒤 가장자리 밖이고, 서보 단자의 변 바로 앞이 선 창이다. 창은 바닥을 뚫었다
    (wx0, wx1), hw = b.DECK_WINDOW
    assert wx1 < -b.EDGE and b.ADAPTER_EDGE_X < wx0 and b.ADAPTER_EDGE_X - b.ADAPTER[1] > b.DECK_END
    mid_x, mid_y = (wx0 + wx1) / 2, b.Y_DESK + b.PLATE_T / 2
    assert not plate.is_inside((mid_x, mid_y, 0.0))
    # 타이 구멍 둘은 파이 판의 구멍과 같은 14 × 3이고, 창과 구멍 사이에 타이를 감는 살 4 mm가 있다
    assert (wx1 - wx0, b.DECK_TIE[1] - b.DECK_TIE[0]) == pytest.approx((14.0, 3.0))
    assert b.DECK_TIE[0] - hw == pytest.approx(4.0)
    for sg in (1, -1):
        assert not plate.is_inside((mid_x, mid_y, sg * sum(b.DECK_TIE) / 2))
        assert plate.is_inside((mid_x, mid_y, sg * (hw + b.DECK_TIE[0]) / 2))
    # 플러그의 넓은 면을 잡을 자리: 서보 단자의 변 앞으로 몸통 뒷면까지 20 mm 넘게 비어 있다
    assert -b.R_CORE - b.ADAPTER_EDGE_X >= 20.0
    # 앞 턱이 기판 앞에서 기판 윗면 높이까지 막는다 (떨어진 나사가 기판 밑으로 굴러들지 못한다)
    x_sill = b.ADAPTER_EDGE_X + b.DECK_SILL[1] + b.DECK_SILL[0] / 2
    assert plate.is_inside((x_sill, top + b.ADAPTER_T - 0.2, 0.0))
    assert b.DECK_SILL[1] < 1.0
    # 옆 턱은 날개의 더 두꺼운 띠와 같은 높이고, 책상 위까지 들어와 띠에 붙어 있다
    rib = sum(b.DECK_RIB) / 2
    assert plate.is_inside((b.DECK_END + 5.0, b.Y_FLOOR + b.RING_H + b.STIFF_H - 0.5, rib))
    assert plate.is_inside((-b.EDGE + 5.0, b.Y_FLOOR + b.RING_H + b.STIFF_H - 0.5, rib))
    assert b.DECK_RIB[1] == pytest.approx(b.WING_IN)
    # 플러그를 누르고 당기는 힘 20 N에 뒤 판이 0.2 mm도 휘지 않는다 (어림). 턱이 없으면 2 mm쯤 휜다
    bend = b.deck_bend()
    assert bend["서보 단자에서"] < 0.02 and bend["전원 쪽 긴 변에서"] < 0.2
    assert bend["턱이 없으면 전원 쪽 긴 변에서"] > 1.0


@pytest.mark.slow
def test_j1_body_holds_the_shelf_nuts_and_the_strip_clamp_screw() -> None:
    body = b.j1_body().solids()[0]
    lower, upper = b.shelf_nut_seats()
    for x, y, z in (lower, upper):
        # 너트 자리는 비어 있고, 벽 쪽(선반 쪽)에 살 SHELF_WALL, 안쪽에 살 SHELF_SKIN이 있다
        assert not body.is_inside((x, y, z))
        # (나사 구멍을 비켜 너트의 모서리 쪽에서 본다)
        assert body.is_inside((x + 2.4, y, z - b.SHELF_SEAT[1] / 2 - 0.5))
        assert body.is_inside((x + 2.4, y, z + b.SHELF_SEAT[1] / 2 + 0.5))
        # 뒤 끝(뽑을 때 위)에서 자리까지 너트가 곧게 내려간다
        reach = y - sh.BACK_END + 2.0
        assert (
            j.nut_entry_overlap(
                b.j1_body(), (x, y, z), (0.0, 0.0, 1.0), (0.0, -1.0, 0.0), reach=reach
            )
            < 0.01
        )
    # 살은 어깨 서보의 옆면에서 3 mm 넘게 떨어져 있다 (서보가 힘 없이 드나든다)
    assert -b._SHELF_RIB_IN - sh.HW >= 3.0
    servo = sh.servo()["body"]
    assert servo.distance_to(b.j1_body()) > 0.3
    # 왼쪽 면의 누르개 나사 구멍은 바닥판의 왼쪽 면에서 깊이 STRIP_TAP까지다
    x, z = b.STRIP_SCREW_UP - b.SHOULDER_H, -b.STRIP_SCREW_X
    assert not body.is_inside((x, sh.FRONT - 1.0, z))
    assert body.is_inside((x, sh.FRONT - b.STRIP_TAP - 1.0, z))
    assert b.NECK_X + 1.4 + 1.2 <= x <= b.NECK_X + b.FLOOR_T - 1.4 - 1.2


def test_bearing_pusher_reaches_the_upper_seat() -> None:
    tool = b.bearing_pusher()
    bb = tool.bounding_box()
    reach = b.B1 - b.Y_FLOOR
    # 통은 좁은 자리를 지나고 바깥 링만 민다. 손잡이 테는 위 베어링이 턱에 앉았을 때 몸통 밑면에서 0.4 뜬다
    assert b.PUSH_R < j.R_BORE and b.PUSH_BORE > j.BRG_D / 2 + 3.0
    assert -bb.min.Y == pytest.approx(reach + b.PUSH_STOP + b.PUSH_GRIP[1])
    assert reach > 46.4 + j.BRG_B  # 아래 자리(깊이 46.4 mm부터)를 지나 위 자리까지


@pytest.mark.slow
def test_j1_cover_clears_the_round_ear_of_the_j1_body() -> None:
    # 뒷덮개는 J1 몸통과 겹치지 않는다 (둥근 귀가 목의 뒤 끝보다 더 나온 자리를 덮개에서 따냈다. 서보 받침에 달린다)
    cover, body = b.j1_cover(), b.j1_body()
    assert cover.distance_to(body) >= 0.25
