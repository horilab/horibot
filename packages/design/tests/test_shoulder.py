"""어깨 관절(ST-3120) 부품: 뽑을 수 있는 형상인지, 조립했을 때 겹치지 않는지, 멈춤과 영점 핀이 맞는지."""

from __future__ import annotations

import math

import pytest

from horibot_design.parts import joint as j
from horibot_design.parts import links
from horibot_design.parts import shoulder as s
from horibot_design.printcheck import (
    LAYER,
    air_walls,
    mid_layer_levels,
    overhangs,
    printed_mass_g,
    wall_settings,
)
from horibot_design.studies import clamp_plate, ur_arm
from horibot_design.studies.packaging import MM
from horibot_design.values import load_measured

BED = 256.0  # 출력 범위 [mm]
# 혼 FT110M과 가운데 나사 (부피로 어림한 값. 출력 혼은 떼어 재지 않았다, data/measured.toml)
HORN_MASS_G = load_measured().servo_horn_mass_g["ST-3120"]


@pytest.fixture(scope="module")
def parts() -> list[j.Part]:
    return s.assembly(links.body("upper_arm"), links.LINKS["upper_arm"][0])


@pytest.mark.slow
def test_printed_parts_are_single_valid_solids_without_steep_overhang() -> None:
    printed = s.printed()
    assert set(s.LABELS) == set(printed)  # 출력 안내의 한글 이름
    for name, (shape, up, _color, _preset) in printed.items():
        assert shape.is_valid, name
        assert len(shape.solids()) == 1, name
        bb = shape.bounding_box()
        assert max(bb.size.X, bb.size.Y, bb.size.Z) < BED, name
        # 바닥에 닿지 않고 아래를 보는 면은 양쪽이 붙은 다리와 작은 가로 구멍 윗변뿐이어야 한다
        over = overhangs(shape, up)
        assert over.total - over.flat < 80.0, (name, over.total, over.flat)


@pytest.mark.slow
def test_shoulder_parts_do_not_overlap(parts: list[j.Part]) -> None:
    assert j.overlaps([p for p in parts if "핀" not in p.name]) == {}


@pytest.mark.slow
def test_upper_arm_turns_freely_and_stops_on_the_web(parts: list[j.Part]) -> None:
    # 프로그램 한계까지는 도는 부품이 어디에도 닿지 않고, 멈춤 각에서 기둥이 살에 닿는다
    for q in (0.0, 90.0, j.SOFT_DEG, -90.0, -j.SOFT_DEG):
        gaps = s.clearances(parts, q)
        assert min(gaps.values()) > 0.5, (q, gaps)
    for q in (j.STOP_DEG, -j.STOP_DEG):
        gaps = s.clearances(parts, q)
        assert gaps[("출력 원판", "어깨 집")] < 0.05, (q, gaps)


@pytest.mark.slow
def test_zero_pin_sits_in_both_holes(parts: list[j.Part]) -> None:
    body = next(p for p in parts if p.name == "어깨 집")
    arm = next(p for p in parts if p.name == "위팔")
    pin = next(p for p in parts if "핀" in p.name)
    # 핀(지름 3.0)이 구멍(3.35) 속에 있으면 벽과의 틈이 0.15 ~ 0.18이다
    assert 0.1 < pin.shape.distance_to(body.shape) < 0.25
    assert 0.1 < pin.shape.distance_to(arm.shape) < 0.25


def test_screw_lengths_match_the_stack() -> None:
    # 혼 볼트 M3 × 20: 혼의 테(두께 2.5)를 다 지나고 0.3 넘게 나오지 않는다. 테 뒤는 허브 둘레의 빈자리다
    engaged = 20.0 - (s.ATOP - s.SEAT)
    assert s.HORN_T <= engaged <= s.HORN_T + 0.3
    # 위팔 나사 M3 × 10: 너트를 끝까지 지나가고, 원통 앞 끝 쪽 틈으로 0.2 넘게 나오지 않는다
    tip = s.ATOP + j.EAR_T - j.BODY_SCREW
    assert s.SEAM - 0.2 <= tip <= s.SEAM + 0.3
    # 영점 핀 20 mm: 위팔 귀 위로 3 나오고, 살 속에 4 넘게 들어간다
    assert s.FRONT - (s.ATOP + j.EAR_T + 3.0 - 20.0) > 4.0
    # 멈춤 나사 끝이 홈 바닥에 닿지 않는다
    assert (s.ATOP - 1.8 - 10.0) - s.CH_Y > 0.8
    # 덮개 나사 M3 × 12 끝이 서보 받침의 기둥(판 위로 5.6) 안에서 멈춘다
    tip = s.BACK_OUT + j.COVER_HEAD_SINK + 12.0
    assert -s.HT < tip < -s.HT + s.COVER_PILLAR


def test_servo_follows_the_measured_st3120() -> None:
    # 잰 값 (2026-11-15): 두께 48.07, 혼 테의 윗면이 앞면에서 8.42, 가운데 돋움이 테보다 2.12, 나사 머리가 3.84 높다
    assert 2 * s.HT == pytest.approx(48.07)
    assert s.HORN_TOP - s.HT == pytest.approx(8.42)
    assert s.HORN_BOSS_H == pytest.approx(2.12)
    assert s.HORN_HEAD_H == pytest.approx(3.84)
    # 뒷면: 보스에 남은 베어링(지름 11.17, 뒷면에서 4.24)을 받침 판이 0.5 넘게 비키고, 뒷덮개와 1.0 넘게 뜬다
    assert (2 * s.BOSS_R, -s.HT - s.BOSS_TIP) == pytest.approx((11.17, 4.24))
    assert s.BOSS_TIP - s.BACK_IN >= j.COVER_GAP
    # 꽂은 플러그: 먼 쪽 면이 축에서 17.69, 앞쪽 면이 뒷면에서 8.94. 받침의 옆 날개와 돋움은 플러그보다 가까운 끝 쪽이다
    assert s.CONN_X[0] == pytest.approx(-17.69)
    assert s.CONN_Y[1] + s.HT == pytest.approx(8.94)
    assert s.LEAF_X[0] - s.CONN_X[1] > 3.0


def test_horn_boss_and_screw_head_have_room() -> None:
    # 허브 판은 테에만 앉는다: 가운데가 돋움보다 넓게 뚫려 있고, 혼 볼트 구멍(반지름 12)이 앉는 고리 안에 있다
    assert s.SEAT_R0 - s.HORN_BOSS_R == pytest.approx(0.5)
    assert s.SEAT_R0 + 0.8 <= s.BOLT_R - j.K_M3_FREE / 2
    assert s.BOLT_R + j.K_M3_FREE / 2 + 0.6 <= s.SEAT_R1 <= s.HORN_R
    # 돋움은 허브 판 두께 속에 든다 (조인 뒤의 저널 끝은 허브 판 윗면에 닿는다)
    assert s.B0 - (s.HORN_TOP + s.HORN_BOSS_H) >= 0.5
    # 가운데 나사 머리는 저널 끝의 자리에 든다: 조인 뒤에 머리 위로 0.8 넘게, 옆으로 0.8 넘게 남는다
    assert (s.B0 + s.HEAD_POCKET) - (s.HORN_TOP + s.HORN_HEAD_H) > 0.8
    assert s.HEAD_HOLE_R - s.HORN_HEAD_R >= 0.8
    assert s.HEAD_HOLE_R + 2.0 < s.BOLT_R - j.K_M3_FREE / 2  # 자리와 혼 볼트 구멍 사이의 살
    # 자리의 바닥과 저널 끝은 원판을 뽑을 때 층 경계에 온다
    for depth in (s.ATOP - s.B0 - j.CLAMP_GAP, s.HEAD_POCKET):
        assert depth / LAYER == pytest.approx(round(depth / LAYER))


def test_wall_nuts_sit_in_side_slots_like_the_sts_joint() -> None:
    w, t = j.K_NUT_SLOT
    tip = s.SPOT_R - j.SCREW_LEN  # 벽 나사 M3 × 8의 끝
    # 홈 안쪽에 살 0.9가 남고, 조인 너트(바깥 면에 붙는다)를 나사 끝이 1 넘게 지나간다
    assert (s.NUT_Z - t / 2) - tip == pytest.approx(0.9)
    assert (s.NUT_Z + t / 2 - j.NUT_T) - tip >= 1.0
    # 홈 바깥의 살: 누름 틀과 서보 받침의 바깥 면(R_SLIDE)까지 2 넘게
    assert s.R_SLIDE - (s.NUT_Z + t / 2) >= 2.0
    # 누름 틀(고리 앞면이 바닥)과 서보 받침(판 뒷면이 바닥)의 홈 위아래가 층 경계에 온다
    for bed, y in ((s.B0, s.RET_SCREW_Y), (-s.HT - s.PLATE_T, s.CAR_SCREW_Y)):
        for edge in (abs(bed - y) - w / 2, abs(bed - y) + w / 2):
            assert edge / LAYER == pytest.approx(round(edge / LAYER))
    # 누름 틀의 다리는 서보 옆면보다 바깥이고, 받침의 돋움은 서보 옆면과 2 넘게 뜬다
    assert tip - s.HW >= 4.0
    assert (s.NUT_Z - t / 2 - 1.0) - s.HW >= 2.0


def test_bearing_seat_uses_the_coupon_values_with_an_entry_bore() -> None:
    fit = load_measured().fit
    assert pytest.approx(fit["pla_basic_white"].bearing_6808_bore_mm) == s.BEARING_BORE
    assert pytest.approx(fit["pla_basic_black"].bearing_6808_shaft_mm) == s.JOURNAL_D
    # 자리(6.6 mm) 뒤의 구멍은 0.2 넓고, 속 부품은 그보다 0.3 작다. 원통 벽은 2.3 넘게 남는다
    assert s.BEARING_ENTRY - s.BEARING_BORE == pytest.approx(0.2)
    assert s.B0 + s.BRG_B - s.SEAT_Y == pytest.approx(6.6)
    assert s.R_ENTRY - s.R_SLIDE == pytest.approx(0.3)
    assert s.R_DRUM - s.R_ENTRY >= 2.3
    # 출력 원판 밑면의 너트 홈(맞변 5.5가 반지름 쪽)이 단과 테두리 사이에 든다
    assert s.RIM_R - j.K_NUT_AF / 2 > s.R_STEP
    assert math.hypot(s.RIM_R + j.K_NUT_AF / 2, j.K_NUT_AF / 2 / math.sqrt(3)) < s.R_DRUM - j.LIP_W


@pytest.mark.slow
def test_no_air_walls_or_mid_layer_faces_block_the_printed_parts() -> None:
    for name, (shape, up, _color, preset) in s.printed().items():
        found = [a for a in air_walls(shape, up, wall_settings(preset)) if a.blocking]
        assert found == [], (name, [a.describe() for a in found])
        assert mid_layer_levels(shape, up) == [], name


@pytest.mark.slow
def test_nuts_slide_straight_into_their_seats() -> None:
    for name, part, center, axis, slide in s.side_nuts():
        assert j.nut_entry_overlap(part, center, axis, slide) < 0.01, name
    assert s.rim_nut_overlap(s.out_disc()) < 0.01


def test_side_plug_fits_inside_the_neck_but_cannot_be_plugged_there() -> None:
    # 옆면 커넥터에 꽂은 플러그와 꺾인 선이 목 안쪽 벽 안에 든다
    assert s.HW + s.WIRE_OUT < s.NECK_IN
    # 목 속 옆 틈이 플러그 길이(9.58)보다 좁다: 서보를 넣기 전에 꽂는다 (조립 순서)
    assert s.NECK_IN - s.HW < 9.58


def test_hub_plate_spring_stays_well_below_the_bending_strength() -> None:
    for plate in clamp_plate.plates():
        force_lo, _ = clamp_plate.press(plate, clamp_plate.E_PLA[0])
        _, stress_hi = clamp_plate.press(plate, clamp_plate.E_PLA[1])
        assert stress_hi < 0.55 * clamp_plate.BEND_STRENGTH, plate.name
        assert force_lo > 60.0, plate.name


def test_shoulder_keeps_the_skeleton_plane() -> None:
    c = ur_arm.Concept()
    st = c.st("ST-3120")
    assert c.ya2 == pytest.approx(ur_arm.SHOULDER_PLANE)
    # 어깨 원통(뒷덮개 ~ 출력 원판 윗면)이 J1 축을 가운데로 1.5 mm 안에서 대칭이다 (-40.4 ~ 41.5)
    assert abs((c.c2 + st.back) + (c.c2 + st.atop)) < 1.5 * MM


@pytest.mark.slow
def test_shoulder_mass_constants_match_the_cad(parts: list[j.Part]) -> None:
    printed = {
        name: printed_mass_g(shape, up, preset)
        for name, (shape, up, _color, preset) in s.printed().items()
    }
    screws = [p for p in parts if p.density > 0 and "핀" not in p.name]
    static = sum(p.mass_g for p in screws if not p.moving)
    moving = sum(p.mass_g for p in screws if p.moving)
    # 집과 뒷덮개는 어깨만 따로 본 모양이다 (뽑는 J1 몸통에서 더 든 몫은 ur_arm.J1_FOOT_MASS)
    house = printed_mass_g(s.house(), (0.0, -1.0, 0.0))
    cover = printed_mass_g(s.back_cover(), (0.0, 1.0, 0.0), "standard")
    house += printed["shoulder_retainer"] + printed["shoulder_carrier"] + cover + static
    assert house == pytest.approx(ur_arm.SHOULDER_HOUSE_MASS * 1000, abs=2.0)
    out = printed["shoulder_out_disc"] + printed["shoulder_hub_plate"] + moving + HORN_MASS_G
    assert out == pytest.approx(ur_arm.SHOULDER_OUT_MASS * 1000, abs=2.0)
