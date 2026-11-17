"""STS 관절, 손목 몸통, 링크 부품: 뽑을 수 있는 형상인지, 조립했을 때 겹치지 않는지, 멈춤과 영점 핀이 맞는지."""

from __future__ import annotations

import math

import pytest

from horilab_design.parts import joint as j
from horilab_design.parts import links, wrist
from horilab_design.parts.shapes import cyl_y
from horilab_design.printcheck import LAYER, air_walls, mid_layer_levels, overhangs, wall_settings
from horilab_design.values import ServoModel, load_measured

BED = 256.0  # 출력 범위 [mm]


@pytest.mark.slow
def test_printed_parts_are_single_valid_solids_that_fit_the_bed() -> None:
    printed = wrist.printed()
    assert set(wrist.LABELS) == set(printed)  # 출력 안내의 한글 이름
    for name, (shape, _up, _color, _preset) in printed.items():
        assert shape.is_valid, name
        assert len(shape.solids()) == 1, name
        bb = shape.bounding_box()
        assert max(bb.size.X, bb.size.Y, bb.size.Z) < BED, name
    for name, (length, tube_r, foot) in links.LINKS.items():
        part = links.link(length, tube_r, foot)
        assert part.is_valid, name
        assert len(part.solids()) == 1, name
        assert part.bounding_box().size.Y < BED, name
        assert links.fits_bed(part)[0], name


@pytest.mark.slow
def test_printed_parts_have_no_steep_overhang() -> None:
    # 바닥에 닿지 않고 아래를 보는 면은 양쪽이 붙은 다리(거의 수평)와 작은 가로 구멍 윗변뿐이어야 한다.
    # 비스듬히(45° ~ 70°) 걸친 면은 서포트 없이 처진다
    parts = {name: (shape, up) for name, (shape, up, _c, _p) in wrist.printed().items()}
    for name, (length, tube_r, foot) in links.LINKS.items():
        parts[name] = (links.link(length, tube_r, foot), (0.0, -1.0, 0.0))
    for name, (shape, up) in parts.items():
        over = overhangs(shape, up)
        # 45°로 깎은 면은 삼각형으로 나눌 때 45°를 조금 넘는 조각이 생긴다 (누름 틀의 원뿔 면 70 mm²)
        assert over.total - over.flat < 80.0, (name, over.total, over.flat)


@pytest.mark.slow
def test_no_air_walls_block_the_first_joint_parts() -> None:
    # 첫 관절 판(2026-11-15)에서 실이 처진 자리는 모두 공중 벽 선 검사가 막던 자리였다: 출력 원판 귀의 나사 머리
    # 자리, 서보 받침의 M2 머리 자리와 먼 가로대 밑, 뒷덮개의 나사 머리 자리 둘. 그 자리들의 모양을 고쳐서 이제
    # 막는 자리가 없다: 귀는 빗살로 가른 막, 뒷덮개는 긴 구멍과 네모, 받침은 파인 자리 없는 한 두께의 판이다
    for name, (shape, up, _c, preset) in wrist.printed().items():
        found = air_walls(shape, up, wall_settings(preset))
        assert [a.describe() for a in found if a.blocking] == [], name
        if name == "carrier":
            assert found == (), name  # 뒷면 전체가 바닥에 닿아 참고로 나오는 자리도 없다


def test_head_pockets_keep_their_seat_and_thread() -> None:
    # 출력 원판 귀: 천장 첫 층이 구멍을 덮는 막이고(틈 몫만 빠진다), 머리(반지름 2.85)가 앉는 고리 16.4 mm² 가운데
    # 13 mm² 넘게 살이 있다. 긴 구멍이면 7.4 mm²다
    disc = j.out_disc()
    seat = j.ATOP - j.POST_HEAD_SINK
    lo, hi = seat - LAYER + 0.02, seat - 0.02
    hole = cyl_y(j.K_M3_FREE / 2, lo, hi, j.R_POST, 0.0)
    assert (disc & hole).volume / hole.volume > 0.85
    ring = cyl_y(2.85, lo, hi, j.R_POST, 0.0) - cyl_y(
        j.K_M3_FREE / 2, lo - 1, hi + 1, j.R_POST, 0.0
    )
    assert (disc & ring).volume / (hi - lo) > 13.0
    # 뒷덮개: 나사 기둥이 머리 자리보다 굵어 둘레의 판에 얹히고, 받침의 날개(반지름 R_SLIDE - 0.4) 안에 앉는다
    assert j.COVER_BOSS_R - j.HEAD_POCKET_R >= 0.6
    assert math.hypot(*j.COVER_BOSS[0]) + j.COVER_BOSS_R <= j.R_SLIDE - 0.4
    # 뒷덮개 나사 M3 × 12가 받침에 물리는 길이 (받침 판의 뒷면에서 나사 끝까지)
    tip = j.BACK_OUT + j.COVER_HEAD_SINK + j.COVER_SCREW
    assert tip - (j.L1 - j.PLATE_T) == pytest.approx(8.78, abs=0.01)


@pytest.mark.slow
@pytest.mark.parametrize("model", j.STS_MODELS)
def test_first_joint_parts_do_not_overlap(model: ServoModel) -> None:
    parts = [p for p in wrist.first_joint(model) if "핀" not in p.name]
    assert j.overlaps(parts) == {}


@pytest.mark.slow
def test_child_turns_freely_and_stops_on_the_web() -> None:
    # 프로그램 한계까지는 도는 부품이 어디에도 닿지 않고, 멈춤 각에서 기둥이 살에 닿는다
    for q in (0.0, 90.0, j.SOFT_DEG, -90.0, -j.SOFT_DEG):
        gaps = wrist.clearances("STS3215", q)
        assert min(gaps.values()) > 0.5, (q, gaps)
    for q in (j.STOP_DEG, -j.STOP_DEG):
        gaps = wrist.clearances("STS3215", q)
        assert gaps[("출력 원판", "손목 몸통 (부모)")] < 0.05, (q, gaps)


@pytest.mark.slow
def test_zero_pin_sits_in_both_holes_at_the_pin_pose() -> None:
    body = wrist.wrist_body()
    pin = next(p for p in wrist.child_hardware() if "핀" in p.name)
    child = wrist.as_child(body, wrist.PIN_POSE_DEG)
    # 핀(지름 3.0)이 구멍(3.30, 3.35) 속에 있으면 벽과의 틈이 0.15 ~ 0.18이다
    assert 0.1 < pin.shape.distance_to(body) < 0.25
    assert 0.1 < pin.shape.distance_to(child) < 0.25


def test_servo_sits_where_the_measured_horn_height_puts_it() -> None:
    measured = load_measured()
    depth = measured.servo_depth
    assert depth is not None
    # 혼 윗면을 낮은 뒷면(두께 29)에서 잰 것으로 읽으면, 혼이 앞면 라벨 판보다 높은 양이 깊이 막대로 잰 값과 맞는다
    low, high = depth.horn_above_label_mm
    for model in j.STS_MODELS:
        assert low - 0.15 <= j.HORN_TOP[model] - j.LABEL <= high + 0.15, model
    # 도면: 혼 윗면은 낮은 앞면에서 약 4.3 (축 3.4 + 혼의 남는 살 0.9), 보스 끝은 낮은 뒷면에서 4.1
    for model in j.STS_MODELS:
        assert j.HORN_TOP[model] - j.F29 == pytest.approx(4.3, abs=0.15), model
    boss = measured.servo_body["STS3215"].horn_to_boss_mm
    assert boss is not None
    assert j.HORN_TOP["STS3215"] - boss == pytest.approx(j.BOSS_TIP, abs=0.05)
    # 몸통은 두께의 가운데가 원점이고, 두께 32인 면이 낮은 면보다 앞뒤로 1.5씩 높다
    assert (j.L0, j.F29, j.L1, j.F32) == (-14.5, 14.5, -16.0, 16.0)
    # 관절의 길이는 혼 윗면 ~ 꽂아 눕힌 선(39.00)에서 나와서 그대로다
    assert j.ATOP - j.BACK_OUT == pytest.approx(58.9, abs=0.01)


def test_horn_screw_head_and_back_side_fit() -> None:
    # 혼 가운데 나사 머리(지름 5.15, 혼 윗면 위로 2.0까지)가 허브 판 속에 들고, 조인 뒤의 저널 끝과 0.5 넘게 뜬다
    assert (j.HEAD_D, j.HEAD_H) == (5.15, 2.0)
    assert j.B0 - (j.SEAT + j.HEAD_H) >= 0.5
    assert 3.6 - j.HEAD_D / 2 >= 0.8  # 허브 판 가운데 구멍과 머리 사이
    # 뒷면: 꽂은 플러그는 커넥터가 있는 뒷면에서 2.6 나오고, 눕힌 선의 윗면은 잰 값이다
    assert j.L1 - j.PLUG_TOP == pytest.approx(2.62, abs=0.01)
    assert j.PLUG_TOP - j.WIRE_TOP == pytest.approx(j.WIRE_T)
    # 서보 받침: 판 뒷면이 눕힌 선 밑보다 낮아서 선이 홈 없이 판 위를 지나간다
    back = j.L1 - j.PLATE_T
    assert back - j.PLUG_TOP >= 0.3
    # 서보 뒷면 나사: 둥근머리(높이 1.6)가 판 뒷면에 바로 앉고, 뒷덮개 안쪽 면과 1.0 넘게 뜬다
    assert (back - 1.6) - j.BACK_IN >= 1.0
    # 머리 밑의 살(판 두께) 2.2 mm: STS3250의 M2 × 5는 서보에 2.8 mm 물리고, STS3215의 자가탭 나사(전체 7.16 mm,
    # 머리 밑 5.5 ~ 6.0으로 본다)는 3.3 ~ 3.8 mm 물린다
    assert j.PLATE_T == pytest.approx(2.2)
    assert j.M2_SCREW - j.PLATE_T == pytest.approx(2.8)
    measured = load_measured()
    assert measured.servo_back_screw is not None
    assert measured.servo_back_screw.total_length_mm == pytest.approx(7.16)
    # 구멍: 흔한 M2·ST2.2 둥근머리(머리 지름 4.0, 나사산 2.2까지)의 나사산이 검정 세로 구멍의 줄어듦까지 넣어 지나간다.
    # 머리는 판 안에 앉는다 (먼 구멍의 머리 가장자리가 판 옆 끝보다 안이다)
    assert 2 * j.M2_HOLE_R - 2.2 >= 0.4
    assert max(abs(z) for _x, z in j.BACK_HOLES) + 2.0 <= j.HW - 0.06
    # 서보 밖의 날개는 3.0 mm이고 서보 옆면과 0.5 넘게 뜬다
    assert j.WING_T == pytest.approx(3.0) and j.WING_IN - j.HW >= 0.5
    # 판 위를 지나는 선(두께 1.5)과 뒷덮개 사이
    assert (back - j.WIRE_T) - j.BACK_IN >= 0.5
    # 뒷덮개 나사 M3 × 12의 끝이 받침의 기둥 속에서 끝난다
    tip = j.BACK_OUT + 1.2 + j.COVER_SCREW
    assert j.L1 + 6.0 < tip < j.L1 + 7.0


def test_screw_lengths_match_the_stack() -> None:
    # 혼 볼트 M3 × 20: 머리는 출력 원판 윗면, 끝은 혼(윗면 SEAT, 테 두께 2.5)에 2.4 ~ 2.9 들어간다.
    # 혼의 테 뒤로 서보 앞면까지는 1.7 mm가 비어 있다
    engaged = 20.0 - (j.ATOP - j.SEAT)
    assert 2.4 <= engaged <= 2.9
    for model in j.STS_MODELS:
        assert (j.SEAT - engaged) - (j.F29 + j.servo_dy(model)) > 1.0, model
    # 자식 몸통 나사 M3 × 10: 너트를 끝까지 지나가고, 원통 앞 끝 쪽 틈으로 0.2 넘게 나오지 않는다
    tip = j.ATOP + j.EAR_T - j.BODY_SCREW
    assert j.SEAM - 0.2 <= tip <= j.SEAM + 0.3
    # 영점 핀 20 mm: 자식 귀 위로 3 나오고, 살 속에 4 넘게 들어간다
    assert j.FRONT - (j.ATOP + j.EAR_T + 3.0 - 20.0) > 4.0
    # 멈춤 나사 끝이 홈 바닥에 닿지 않는다
    assert (j.ATOP - 1.8 - 10.0) - j.CH_Y > 0.8


def _on_layer(height: float) -> bool:
    return abs(height / LAYER - round(height / LAYER)) < 1e-6


@pytest.mark.slow
def test_horizontal_faces_do_not_sit_mid_layer() -> None:
    # 슬라이서는 층의 가운데 높이에서 자른다. 수평면이 그 높이에 있으면 한 층이 더 뽑힐지가 정해지지 않는다
    for name, (shape, up, _color, _preset) in wrist.printed().items():
        assert mid_layer_levels(shape, up) == [], name


def test_fits_land_on_layer_boundaries() -> None:
    # 시험편으로 고른 값이 그대로 뽑히려면 홈의 위아래와 맞닿는 면이 층 경계에 와야 한다
    w = j.K_NUT_SLOT[0]
    # 누름 틀 (고리 앞면이 바닥): 너트 홈의 바닥과 천장. 홈은 나사 축에서 0.1 안으로만 옮겨진다
    y = j.retainer_nut_y()
    assert _on_layer(j.B0 - (y + w / 2)) and _on_layer(j.B0 - (y - w / 2))
    assert abs(y - j.RET_SCREW_Y) <= 0.1
    # 서보 받침 (판 뒷면이 바닥)
    bed = j.L1 - j.PLATE_T
    assert _on_layer(j.CAR_SCREW_Y - w / 2 - bed) and _on_layer(j.CAR_SCREW_Y + w / 2 - bed)
    # 출력 원판 (테 윗면이 바닥): 너트 홈 바닥, 테두리 끝, 멈춤 기둥 끝, 저널 끝
    for y in (j.SEAM + j.NUT_POCKET, j.FRONT + j.LIP_GAP, j.POST_TIP, j.B0 + j.CLAMP_GAP):
        assert _on_layer(j.ATOP - y), y
    # 손목 몸통 (원통 앞 끝이 바닥): 턱, 베어링 자리의 끝, 원통 뒤 끝
    for y in (j.B0 + j.BRG_B, j.SEAT_Y, j.BACK_END):
        assert _on_layer(j.FRONT - y), y
    # 뒷덮개 (바깥 면이 바닥): 판 안쪽 면, 테 끝
    for y in (j.BACK_IN, j.BACK_END + j.COVER_SKIRT):
        assert _on_layer(y - j.BACK_OUT), y


def test_bearing_seat_is_stepped_for_entry() -> None:
    # 베어링 자리는 시험편 값이고, 그 뒤는 0.2 넓어서 베어링과 속 부품이 걸리지 않고 드나든다
    assert j.BEARING_ENTRY - j.BEARING_BORE == pytest.approx(0.2)
    assert j.R_ENTRY - j.R_SLIDE == pytest.approx(0.3)
    # 누름 틀의 고리(앞면이 베어링 뒷면 B0)는 좁은 구멍에 들지 않고, 자리는 베어링 폭 7 mm 중 6.5 mm를 넘게 문다
    assert j.B0 < j.SEAT_Y <= j.B0 + 0.5
    assert j.B0 + j.BRG_B - j.SEAT_Y >= 6.5


def test_nut_seats_use_the_coupon_values_for_their_colour_and_pose() -> None:
    fit = load_measured().fit
    black, white = fit["pla_basic_black"], fit["pla_basic_white"]
    assert j.K_NUT_SLOT == (black.m3_nut_upright_width_mm, black.m3_nut_upright_thickness_mm)
    assert j.K_NUT_AF == black.m3_nut_pocket_mm
    assert j.BEARING_BORE == white.bearing_6806_bore_mm
    assert j.JOURNAL_D == black.bearing_6806_shaft_mm
    assert j.PROVISIONAL == ()
    # 옆 홈: 안쪽에 살이 남아 벽 나사가 밀어도 너트가 빠지지 않고, 조인 너트를 나사가 1 mm 넘게 지나간다
    tip = j.SPOT_R - j.SCREW_LEN
    assert j.NUT_Z - j.K_NUT_SLOT[1] / 2 - tip >= 0.8
    assert (j.NUT_Z + j.K_NUT_SLOT[1] / 2 - j.NUT_T) - tip >= 1.0
    # 출력 원판 밑면의 너트 홈(맞변이 반지름 쪽을 본다)은 안쪽 링을 누르는 단과 테 밑의 테두리 사이에 든다
    r = math.hypot(*j.RIM[0])
    assert r - j.K_NUT_AF / 2 >= j.R_STEP + 0.2
    assert r + j.K_NUT_AF / 2 <= j.R_DRUM - j.LIP_W - 0.1


@pytest.mark.slow
def test_side_nuts_slide_straight_into_their_slots() -> None:
    # 누름 틀과 서보 받침의 너트 다섯은 옆 입구에서 기울이지 않고 자리까지 들어간다
    for name, part, center, axis, slide in j.side_nuts():
        assert j.nut_entry_overlap(part, center, axis, slide) < 0.01, name


def test_cover_screw_hole_stops_at_the_pillar_top() -> None:
    # 덮개 나사 M3 × 12의 끝은 기둥 윗면보다 0.4 mm 아래다. 구멍을 기둥 위의 돋움까지 파면 돋움의 안쪽 모서리가
    # 조각으로 떨어져 허공에서 뽑히기 시작한다 (슬라이서의 떠 있는 영역 경고)
    pillar = j.BACK_OUT + 1.2 + j.COVER_SCREW + 0.4
    part = j.carrier()
    x, z = j.COVER_BOSS[0]
    # 구멍 속이던 자리 가운데 돋움 밑의 살 안(축에서 먼 쪽)에 든 작은 기둥이 꽉 차 있다
    probe = cyl_y(0.4, pillar + 0.3, pillar + 0.8, x, z + 0.8)
    assert (part & probe).volume == pytest.approx(probe.volume, rel=1e-3)
