"""링크에 더한 것: 발의 선 창, 관 글자와 그 구멍, 아래팔 끝의 나사 자리, D405 타이의 자리, 홈 자세의 가림 검사."""

from __future__ import annotations

import itertools
import math

import numpy as np
import pytest

from horilab_design import placement as pl
from horilab_design.parts import fake_forearm as ff
from horilab_design.parts import joint as j
from horilab_design.parts import links
from horilab_design.parts import rest_shelf as rs
from horilab_design.parts import shoulder as sp
from horilab_design.parts.shapes import box, solid
from horilab_design.printcheck import (
    LAYER,
    air_walls,
    mid_layer_levels,
    overhangs,
    printed_mass_g,
    wall_settings,
)
from horilab_design.studies import home_view as hv
from horilab_design.studies import wiring as wr


def _on_layer(height: float) -> bool:
    return abs(height / LAYER - round(height / LAYER)) < 1e-6


@pytest.mark.slow
def test_link_bodies_print_without_trouble() -> None:
    made = links.printed()
    assert set(links.LABELS) == set(made)  # 출력 안내의 한글 이름
    for name in links.LINKS:
        part, up = made[name][0], made[name][1]
        assert part.is_valid and len(part.solids()) == 1, name
        assert links.fits_bed(part)[0], name
        over = overhangs(part, up)
        assert over.total - over.flat < 80.0, (name, over.total, over.flat)
        # 수평면이 층의 한가운데에 걸치면 출력 준비 도구가 막는다 (관 속 천장을 한 다각형으로 그려서 없다)
        assert mid_layer_levels(part, up) == [], name
        found = air_walls(part, up, wall_settings("strength"))
        assert [a.describe() for a in found if a.blocking] == [], name


def test_foot_wall_has_the_low_slot_the_slit_and_the_high_window() -> None:
    z0, z1 = links.FOOT_SLOT_Z
    w0, w1 = links.FOOT_WINDOW
    # 창은 선(폭 4.85, 두께 1.5)이 누워 지나가는 크기이고, 높이 범위가 뽑을 때의 층 경계다
    assert z1 - z0 > j.WIRE_W + 0.8 and w1 - w0 > j.WIRE_T + 1.0
    assert _on_layer(w0) and _on_layer(w1) and _on_layer(links.FOOT_SLOT_H)
    # 창으로 나온 선은 영점 핀 귀와 그 위로 3 mm 나온 핀보다 3 mm 넘게 높다
    assert (w0 + w1) / 2 - j.WIRE_T / 2 - (j.EAR_T + 3.0) > 3.0
    # 세로 틈은 선을 세워야 지나가고(두께 1.5보다 넓고 폭 4.85보다 좁다), 눕힌 선은 되내려가지 않는다
    assert j.WIRE_T + 0.4 <= links.FOOT_SLIT_W < j.WIRE_W - 2.0
    # 배선 검토가 보는 창의 자리가 링크의 값이다
    assert wr.LINK_WINDOW_H == pytest.approx((w0 + w1) / 2)
    assert wr.LINK_WINDOW == pytest.approx((z1 - z0, w1 - w0))
    assert wr.link_window_v() == pytest.approx((z0 + z1) / 2)


@pytest.mark.slow
def test_foot_wall_openings_are_open_in_the_printed_links() -> None:
    z0, z1 = links.FOOT_SLOT_Z
    zc = (z0 + z1) / 2
    w0, w1 = links.FOOT_WINDOW
    for name, (length, _tube_r, foot) in links.LINKS.items():
        part = links.body(name)
        x0, x1 = -length - foot.hx - 0.5, -length - foot.hx + links.WALL + 0.5
        window = box(x0, x1, j.FRONT - w1 + 0.2, j.FRONT - w0 - 0.2, z0 + 0.3, z1 - 0.3)
        slit = box(x0, x1, j.FRONT - w0, j.FRONT - 5.0, zc - 0.8, zc + 0.8)
        for probe in (window, slit):
            assert solid(part & probe).volume < 0.01, name
        # 창 밑의 벽은 세로 틈 양옆에 남아 있다 (눕힌 선을 받친다)
        ledge = box(x0 + 0.7, x1 - 0.7, j.FRONT - w0 + 0.3, j.FRONT - w0 + 2.0, z0 + 0.2, zc - 1.2)
        assert solid(part & ledge).volume == pytest.approx(ledge.volume, rel=0.01), name


@pytest.mark.slow
def test_wire_window_moved_out_but_the_printed_fake_forearm_kept_its_own() -> None:
    lo, hi = j.WIN_X
    assert (lo, hi) == pytest.approx((-45.6, -36.0))
    old_lo, old_hi = ff.WIN_X_PRINTED
    fake, real = ff.fake_forearm(), links.body("forearm")
    y0, y1 = j.NECK_FRONT_IN + 0.3, j.FRONT - 0.2
    # 축 쪽 끝: 가짜 아래팔 끝은 옛 자리(35.4)까지 뚫려 있고, 링크는 36.0에서 막힌다
    inner = box(hi + 0.1, old_hi - 0.1, y0, y1, -2.5, 2.5)
    assert solid(fake & inner).volume < 0.01
    assert solid(real & inner).volume == pytest.approx(inner.volume, rel=0.01)
    # 관 쪽 끝: 링크는 45.6까지 뚫려 플러그(폭 9.01)가 지나간다
    outer = box(lo + 0.1, old_lo - 0.1, y0, y1, -2.5, 2.5)
    assert solid(real & outer).volume < 0.01
    assert solid(fake & outer).volume == pytest.approx(outer.volume, rel=0.01)


def test_letters_read_from_the_previous_joint_with_tops_toward_the_back() -> None:
    frames = pl.frames_mm(np.zeros(6))  # 팔을 곧게 세운 자세
    for name, (word, side) in links.WORDS.items():
        plane = links._letter_plane(name)
        R = frames[pl.LINK_FRAME[name]][:3, :3]
        read, top, out = (
            R @ np.array((v.X, v.Y, v.Z)) for v in (plane.x_dir, plane.y_dir, plane.z_dir)
        )
        # 어깨 → 팔꿈치 → 손목(위)으로 읽고, 글자 윗변이 로봇 뒤(-x), 글자는 로봇 오른쪽(-y)을 본다
        assert read == pytest.approx((0.0, 0.0, 1.0), abs=1e-9), name
        assert top == pytest.approx((-1.0, 0.0, 0.0), abs=1e-9), name
        assert out == pytest.approx((0.0, -1.0, 0.0), abs=1e-9), name
        # 낱자는 읽는 차례로 놓인다
        centres = [links.glyph_outline(k).centroid.x for k in links.letter_keys(name)]
        assert centres == sorted(centres) and len(centres) == len(word), name
        assert side == ("flat" if name == "upper_arm" else "round")
    # 글꼴 크기 17에서 대문자 높이 11.9 mm, BOMI는 47.0 mm다 (0019)
    b = links.glyph_outline("letter_bomi_b").bounds
    assert b[3] - b[1] == pytest.approx(11.9, abs=0.05)
    lay = links.layout("upper_arm")
    assert lay["글자 끝"] - lay["글자 시작"] == pytest.approx(47.04, abs=0.05)


def test_pegs_hide_behind_the_strokes_and_differ_between_letters() -> None:
    keys = [k for name in links.LINKS for k in links.letter_keys(name)]
    assert set(keys) == set(links.PEGS)
    for key in keys:
        # 구멍(그린 지름)이 글자 윤곽 안에 LETTER_COVER 넘게 든다
        assert links.peg_cover(key) >= links.LETTER_COVER, key
        outline = links.glyph_outline(key)
        x0, y0, x1, y1 = outline.bounds
        (ax, ay), (bx, by) = links.peg_points(key)
        if key in links.PEG_MIRRORED:  # 돌려 꽂아도 같은 자리다
            assert (ax + bx, ay + by) == pytest.approx((x0 + x1, y0 + y1), abs=0.03), key
    # 낱자마다 핀 사이 간격이 달라서 다른 낱자의 구멍에는 꽂히지 않는다
    spacing = sorted(links.peg_spacing(k) for k in keys)
    assert min(b - a for a, b in itertools.pairwise(spacing)) >= links.PEG_APART
    assert spacing[0] > 5.0  # 핀 둘이 낱자의 방향을 잡을 만큼 떨어져 있다
    # 핀은 구멍보다 가늘게 그렸고(맞춤은 핀 시험편으로 고른다), 구멍은 획(3.37 mm)보다 가늘다
    assert all(0.0 < links.LETTER_HOLE - d <= 0.4 for d in links.PEG_D.values())
    assert links.LETTER_HOLE + 2 * links.LETTER_COVER <= 3.37


@pytest.mark.slow
def test_letters_are_printable_and_sit_in_their_holes() -> None:
    for name, (_word, side) in links.WORDS.items():
        part = links.body(name)
        for key, shape in links.letters(name).items():
            up = links.letter_up(key)
            assert shape.is_valid and len(shape.solids()) == 1, key
            assert overhangs(shape, up).total < 0.5, key  # 앞면이 바닥이고 핀이 위로 선다
            assert mid_layer_levels(shape, up) == [], key
            assert air_walls(shape, up, wall_settings("strength")) == (), key
            # 그린 값으로는 핀이 구멍 벽과 닿지 않고(0.1 mm), 글자는 관에 닿아 앉는다
            assert solid(part & shape).volume < 0.01, key
            assert shape.distance_to(part) < 0.01, key
            assert printed_mass_g(shape, up) < 0.25, key
        bb = [s.bounding_box() for s in links.letters(name).values()]
        top = links._tube_top(name)
        if side == "flat":
            # 납작한 면에서 LETTER_T만큼 나오고, 핀은 벽(2.6)을 넘지 않는다 (관 속에 선이 지나간다)
            assert max(b.max.Y for b in bb) == pytest.approx(j.FRONT + links.LETTER_T[side])
            assert min(b.min.Y for b in bb) >= j.FRONT - links.WALL + 0.3
        else:
            # 앞면은 관 꼭대기 위 LETTER_T의 평면이고, 핀 끝은 막힌 구멍의 바닥에서 뜬다
            assert min(b.min.Y for b in bb) == pytest.approx(top - links.LETTER_T[side])
            tip = max(b.max.Y for b in bb)
            assert top + links.ROUND_HOLE_DEPTH - tip == pytest.approx(links.PEG_FLOOR_GAP)
    # 둥근 면의 구멍: 바닥이 층 경계이고, 관 속 천장(다리 층)까지 1 mm 넘게 남는다
    _length, tube_r, _foot = links.LINKS["forearm"]
    top = links._tube_top("forearm")
    assert _on_layer(j.FRONT - (top + links.ROUND_HOLE_DEPTH))
    roof = j.FRONT - (tube_r - links.EMBED) - 0.91 * (tube_r - links.WALL)  # 관 속 천장의 y
    assert roof - (top + links.ROUND_HOLE_DEPTH) > 1.0
    # 글자의 높이도 층 경계다
    assert _on_layer(links.LETTER_T["flat"]) and _on_layer(links.LETTER_T["round"])
    assert _on_layer(links.LETTER_T["flat"] + links.PEG_LEN_FLAT)


def test_ties_clear_the_stop_ears_the_wire_windows_and_the_letters() -> None:
    half = links.TIE_W / 2
    for name, (length, _tube_r, foot) in links.LINKS.items():
        foot_x, house_x = links.tie_x(name)
        lay = links.layout(name)
        # 글자는 두 타이 사이에 있다 (10 mm 넘게 떨어져)
        assert lay["발 쪽 타이와 글자 사이"] > 10.0 and lay["글자와 끝 집 쪽 타이 사이"] > 10.0, (
            name
        )
        # 끝 집 쪽 타이는 선 창(관 쪽 끝 -45.6)과 2 mm 넘게 떨어져 있다
        assert j.WIN_X[0] - (house_x + half) >= 2.0, name
        # 발 쪽 타이는 앞 관절 출력 원판의 멈춤 귀 끝(납작한 면 밑)과 2 mm 넘게 떨어져 있다
        ear_tip = (j.R_POST + j.EAR_HW) if name == "forearm" else (sp.R_POST + sp.EAR_HW)
        assert (foot_x - half) - (-length + ear_tip) >= 2.0, name
        assert foot_x - half > -length + foot.hx  # 발 상자 밖의 관에 감는다
    # 선은 팔꿈치 바깥(팔을 세우면 로봇 뒤) 가장자리에 있다
    frames = pl.frames_mm(np.zeros(6))
    for name in links.LINKS:
        y, z = links.cable_yz(name)
        tube_mid = frames[pl.LINK_FRAME[name]] @ np.array([-100.0, y, 0.0, 1.0])
        cable = frames[pl.LINK_FRAME[name]] @ np.array([-100.0, y, z, 1.0])
        assert cable[0] - tube_mid[0] == pytest.approx(-abs(z)), name
    # 타이는 가정한 값이다 (아직 사지 않았다)
    assert (links.TIE_W, links.TIE_T, links.TIE_G) == (12.0, 1.3, 0.6)
    # 팔꿈치에서 마주 보는 두 타이는 아래팔이 다 접혀도 위에서 볼 때 겹치지 않는다 (J3 축에서의 거리)
    fore, tube_r, _foot = links.LINKS["forearm"]
    flat_hw = math.sqrt(tube_r**2 - (tube_r - links.EMBED) ** 2) + links.TIE_T
    far_corner = math.hypot(fore + links.tie_x("forearm")[0] + half, flat_hw)
    assert -links.tie_x("upper_arm")[1] - half - far_corner > 3.0  # 손으로 감는 자리의 몫


@pytest.mark.slow
def test_forearm_seats_take_nuts_after_assembly_and_stay_behind_the_seam() -> None:
    part = links.body("forearm")
    checks = links.seat_checks()
    # 너트는 뒤(뽑을 때 위)에서 곧게 내려가고, 입구는 뒷덮개에서 멀다 (덮개를 떼지 않는다)
    assert checks["너트를 뒤에서 곧게 떨어뜨릴 때 링크와 겹침 [mm³]"] < 0.01
    assert checks["너트 홈의 입구와 뒷덮개 테 사이 (y)"] > 30.0
    # 돋움은 납작한 면(이음매)을 넘지 않고, 집 원통의 옆보다 덜 나온다
    bb = part.bounding_box()
    assert bb.max.Y == pytest.approx(j.FRONT) and bb.max.Z == pytest.approx(j.R_DRUM, abs=0.05)
    assert links.SEAT_FACE < j.R_DRUM and _on_layer(links.SEAT_H)
    # 나사 구멍은 돋움 안에서 끝난다: 목의 벽(선이 지나가는 빈자리의 벽)의 겉을 파지 않는다
    for sg in (1, -1):
        z0, z1 = sorted((sg * (j.NECK_HW - 0.8), sg * (j.NECK_HW - 0.1)))
        wall = box(links.SEAT_X - 1.5, links.SEAT_X + 1.5, j.FRONT - 6.0, j.FRONT - 3.6, z0, z1)
        assert solid(part & wall).volume == pytest.approx(wall.volume, rel=0.01)
    # 두께 3 mm 부품을 M3 × 8로 달면 나사가 너트를 지나고 구멍 바닥에 닿지 않는다
    screws = links.seat_screws(8.0, 3.0)
    for p in screws:
        assert solid(part & p.shape).volume < 0.01, p.name
    tip = 3.0 + links.SEAT_HOLE - 8.0
    assert 0.3 <= tip and 8.0 - 3.0 - (links.SEAT_ROOF + j.NUT_T) >= 0.9
    # 쉬는 자세: 선반은 집의 맞은편이라 돋움과 멀다
    placed = pl.place(pl.arm_parts("forearm", "rest_shelf"), np.radians(rs.REST_DEG))
    seats = solid(links._seat_blocks()).moved(
        pl.location_of(pl.frames_mm(np.radians(rs.REST_DEG))["j4"])
    )
    shelf = next(s for p, s in placed if p.name == "선반/쉬는 자세 선반")
    assert seats.distance_to(shelf) > 10.0


@pytest.mark.slow
def test_both_words_show_in_one_photo_of_the_home_pose() -> None:
    assert hv.HOME_DEG == (0.0, 15.0, 110.0, 145.0, 90.0, 0.0)  # H3 (0019)
    found = hv.views()  # 카메라 18곳: -y 정면에서 앞뒤·위아래 ±15°, 거리 0.6과 1.2 m
    assert len(found) == 18
    for view in found:
        for key, share in view.seen.items():
            assert share == 1.0, (view.camera, key, view.blockers[key])
    # 검사가 가림을 잡는지: O 앞에 판을 세우면 그 낱자가 가려진다
    points = hv.letters_at(hv.HOME_DEG)
    c = points["letter_hori_o"].mean(axis=0)
    corners = np.array([(-9.0, -30.0, -9.0), (9.0, -30.0, -9.0), (0.0, -30.0, 12.0)])
    plate = hv.solid_of("판", (c + corners)[None, :, :])
    blocked = hv.views(tilts=(0.0,), dists=(900.0,), extra=(plate,))[0]
    assert blocked.seen["letter_hori_o"] < 0.5 and blocked.blockers["letter_hori_o"] == ("판",)
    assert blocked.seen["letter_bomi_m"] == 1.0
    # 고리는 손으로 비켜 놓을 자리가 있다 (2026-11-22): 고리마다 글자를 가리지 않는 방향이 절반 넘게 있다
    for name, (_slack, ways, _hang) in hv.loop_room().items():
        assert ways >= hv.LOOP_DIRS // 2, name


@pytest.mark.slow
def test_letters_and_ties_keep_the_moving_gap_to_the_neighbour_links() -> None:
    # 위팔의 납작한 면은 아래팔, J1 몸통 왼쪽 면과 5.8 mm 층을 이룬다. 글자(1.4)와 타이(1.3)가 그 층에 나와도
    # 관절을 범위 끝까지 돌릴 때 다른 링크의 부품과 2 mm 넘게 뜬다
    for name, (gap, angle, a, b) in hv.extra_gaps(step=20.0).items():
        assert gap >= hv.MOVING_GAP, (name, gap, angle, a, b)
    assert math.isclose(j.ATOP - j.FRONT, 5.8)
    assert links.LETTER_T["flat"] <= (j.ATOP - j.FRONT) - hv.MOVING_GAP - 2.0
