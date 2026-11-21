"""베이스 둘레의 부품 (베이스·어깨 묶음 2): 파이 판, 쉬는 자세의 선반, 선 누르개, 쉬는 자세의 검토."""

from __future__ import annotations

import math

import numpy as np
import pytest

from horilab_design import placement as pl
from horilab_design.parts import base as b
from horilab_design.parts import joint as j
from horilab_design.parts import pi_mount as pm
from horilab_design.parts import rest_shelf as rs
from horilab_design.parts import shoulder as sh
from horilab_design.parts import wire_clamps as wc
from horilab_design.parts.shapes import solid
from horilab_design.printcheck import air_walls, mid_layer_levels, overhangs, wall_settings
from horilab_design.studies import rest_pose as rp
from horilab_design.studies import wiring as wr


@pytest.mark.slow
def test_new_printed_parts_are_single_solids_that_print_without_support() -> None:
    made = {
        **{k: v[:4] for k, v in pm.printed().items()},
        **rs.printed(),
        **wc.printed(),
    }
    assert set(made) == set(pm.LABELS) | set(rs.LABELS) | set(wc.LABELS)
    for name, (shape, up, _color, preset) in made.items():
        assert shape.is_valid and len(shape.solids()) == 1, name
        over = overhangs(shape, up)
        assert over.total - over.flat < 30.0, (name, over.total, over.flat)
        assert mid_layer_levels(shape, up) == [], name
        found = [a for a in air_walls(shape, up, wall_settings(preset)) if a.blocking]
        assert found == [], (name, [a.describe() for a in found])


def test_pi_tray_matches_the_wing_posts_the_pi_and_the_cover() -> None:
    tray = pm.tray_local().solids()[0]
    mid = (pm.Y_TRAY + pm.Y_TOP) / 2
    # 판 나사 구멍은 날개의 기둥과 같은 자리다 (두 파이 판이 같은 부품이고 두 날개가 거울상이다)
    posts = {(round(x, 3), round(z, 3)) for x, z in b.post_xz()}
    for side in (1, -1):
        for u, v in pm.screw_uv():
            assert (round(b.PI_CX + u, 3), round(side * b.PI_CZ - v, 3)) in posts
    for u, v in (*pm.screw_uv(), *pm.post_uv(), *pm.PI_HOLES):
        assert not tray.is_inside((u, mid, -v))
    # 판 나사는 기판 밖(긴 변에서 머리 반지름 넘게)이고 덮개 기둥의 육각 몸통과 떨어져 있다
    for u, v in pm.screw_uv():
        assert abs(v) - 2.85 >= b.PI_BOARD[1] / 2
        assert min(math.dist((u, v), p) for p in pm.post_uv()) > 2.85 + 3.2 + 2.0
    # 타이 구멍 넷은 판 가장자리에서 4 mm 안쪽에 뚫려 있다
    for a0, a1, v0, v1 in pm.tie_slots():
        assert not tray.is_inside(((a0 + a1) / 2, mid, -(v0 + v1) / 2))
        assert min(b.TRAY_HX - a1, b.TRAY_HZ - max(abs(v0), abs(v1))) == pytest.approx(pm.TIE_EDGE)
    # 높이: 덮개가 USB 단자 위에 있고, 덮개 윗면은 선의 고리가 생기는 띠 밑보다 낮다
    stack = pm.stack()
    assert stack["아크릴 판 밑면"] > stack["USB 단자 윗면"]
    assert stack["아크릴 판 윗면"] + 3.0 < stack["띠 밑 (선의 고리가 생기는 높이)"]
    assert 2.0 <= stack["파이 나사 M2.5 × 12가 판 밑으로 나온 길이 (너트 2.0)"]
    assert 2.4 <= stack["덮개 기둥 수나사가 판 밑으로 나온 길이 (너트 2.4)"]
    assert stack["젠더 밑과 파이 판 나사 머리 사이"] >= 2.0


@pytest.mark.slow
def test_pi_side_parts_clear_the_core_the_plate_and_the_clamps() -> None:
    near = pm.clearances(limit=3.0)
    # 닿는 것은 기둥에 앉은 파이 판과 판 나사, 돋움에 앉은 서보 어댑터뿐이다. 어댑터의 플러그 자리(변 전체로 넉넉히
    # 그린 상자)는 기둥과 클램프에서 0.5 mm 넘게 떨어져 있다
    for (a, other), d in near.items():
        if "자리" in a:
            assert d >= 0.5, (a, other, d)
            continue
        if other.startswith(
            "클램프"
        ):  # 클램프의 위 턱은 서보 어댑터와 2 mm 넘게 떨어져 뒤에서 드나든다
            assert a == "서보 어댑터" and d >= 2.0, (a, other, d)
            continue
        seated = "파이 판" in a or a == "서보 어댑터"
        assert seated and other == "책상에 무는 판" and d < 0.01, (a, other, d)
    # 오른쪽 파이의 젠더와 전원 플러그는 몸통과 3 mm 넘게 떨어져 있다 (젠더는 도면 값)
    wide = pm.clearances(limit=6.0)
    gaps = [wide[(f"컨트롤러 파이/{name}", "베이스 몸통")] for name in ("젠더", "전원 플러그")]
    assert b.BODY_GAP <= min(gaps) < b.BODY_GAP + 1.0


def test_rest_pose_seats_the_forearm_end_on_the_shelf() -> None:
    x4, z4 = rs.j4_axis()
    frames = pl.frames_mm(np.radians((0.0, *rs.REST_DEG[1:])))
    assert (x4, z4) == pytest.approx((frames["j4"][0, 3], frames["j4"][2, 3]), abs=1e-6)
    # J3는 한계에서 5° 넘게 안쪽이다 (멈춤에 기대지 않는다)
    assert rs.REST_J3 <= j.SOFT_DEG - 5.0
    # 받침 셋은 집 옆의 벽 나사 머리를 10° 넘게 비킨다
    assert min(rs.pad_margins().values()) > 10.0
    checks = rs.screw_checks()
    assert checks["선반 나사가 너트(2.4)에 물린 길이"] == pytest.approx(j.NUT_T)
    assert checks["선반 나사 끝과 어깨 서보 옆면 사이"] > 3.0
    assert checks["아래 나사 머리 밑과 밑 보 윗면 사이"] > 1.0
    assert checks["위 나사 머리와 위 띠 윗면 사이"] > 1.0
    assert checks["창에서 아래 나사에 렌치의 짧은 쪽이 들어갈 길이"] > 20.0
    # 나사 자리는 J1 몸통의 너트와 같은 줄이다
    for (x, y, z), (xs, ys, _zs) in zip(rs.screw_xyz(), b.shelf_nut_seats(), strict=True):
        assert y - b.Y_DESK == pytest.approx(xs + b.SHOULDER_H) and -z == pytest.approx(
            ys - sh.J1_AXIS_Y
        )
        assert x == pytest.approx(sh.NECK_HW + rs.ROOT_T)


@pytest.mark.slow
def test_shelf_sits_on_the_j1_body_and_cradles_the_housing() -> None:
    shelf = rs.shelf()
    body = b.as_child(b.j1_body())
    assert shelf.distance_to(body) < 0.01 and solid(shelf & body).volume < 0.05
    assert shelf.distance_to(b.turntable()) > 2.0
    # 쉬는 자세의 집은 받침에서 PAD_GAP만큼 뜬다 (받침 셋에 고르게)
    assert shelf.distance_to(rs.house_dummy()) == pytest.approx(rs.PAD_GAP, abs=0.02)
    # 실제 아래팔을 쉬는 자세로 놓아도 같다 (벽 나사 머리는 받침 사이의 물러난 자리에 든다)
    parts = [p for p in pl.arm_parts("forearm", "rest_shelf")]
    placed = pl.place(parts, np.radians(rs.REST_DEG))
    seat = next(s for p, s in placed if p.name == "선반/쉬는 자세 선반")
    for p, s in placed:
        if p.name.startswith("아래팔/") and "선" not in p.name:
            d = seat.distance_to(s)
            assert d >= rs.PAD_GAP - 0.05, (p.name, d)


@pytest.mark.slow
def test_hanging_wrist_stays_inside_the_j4_range() -> None:
    for tool in (True, False):
        (angle,) = rp.hang_j4(tool)
        assert angle <= rp.J4_LIMIT - rp.J4_MARGIN, (tool, angle)
    assert rp.stable_sign_ok()
    (with_tool,) = rp.hang_j4(True)
    assert rs.REST_J4 == pytest.approx(with_tool, abs=1.0)
    # 선반이 받는 힘은 위로 10 N쯤이고 몸통 쪽으로 조금 기운다: 밑 받침과 뒤 받침이 다 눌린다
    force = rp.shelf_reaction()
    assert 8.0 < force["위로 받치는 힘 [N]"] < 14.0
    assert 3.0 < force["받치는 힘이 수직에서 기운 각 [°]"] < rs.PADS[2][0] / 2


@pytest.mark.slow
def test_rest_pose_clears_everything_around_the_base() -> None:
    gaps = rp.rest_gaps()
    seat = gaps.pop(("아래팔", "선반"))
    assert seat[0] == pytest.approx(rs.PAD_GAP, abs=0.1)
    for key, (d, a, f) in gaps.items():
        assert d >= rp.REST_GAP, (key, d, a, f)
    assert rp.lowest_rest_point()[0] >= rp.REST_GAP


def test_wire_clamps_fit_under_the_upper_arm_and_hold_two_wires() -> None:
    checks = wc.screw_checks()
    assert checks["왼쪽 면: 누르개와 머리가 나온 높이 (한도 3.8)"] <= wc.STRIP_LIMIT
    assert checks["왼쪽 면: 위팔의 납작한 면까지 남는 틈"] >= 2.0
    assert checks["왼쪽 면: 나사 끝과 구멍 바닥 사이"] >= 1.0
    assert checks["귀: 나사 끝과 돌림판 윗면 사이"] >= 1.0
    # 선 자리는 선을 0.2 ~ 0.3 mm 누르고, 폭은 선(4.85)보다 넓다
    assert 0.15 <= j.WIRE_T - wc.GROOVE <= 0.35 and 0.15 <= j.WIRE_T - wc.STRIP_STANDOFF <= 0.35
    assert wc.LANE[1] - wc.LANE[0] > j.WIRE_W + 0.4
    # 띠 밑 누르개는 아래 턱을 튼 자리(반폭 8)보다 좁고, 턱 밑에 있다
    assert wc.HALF[1] <= b.BAND_NOTCH and wc.HALF[0] < b.CLAMP_DROP - 1.1 * b.BAND_LIP
    # 왼쪽 면의 선 둘: 아래 선(J3로 가는 선)은 누르개의 팔 밑, 나사 머리 위 줄을 지난다
    lane = wr.STRIP_LANE - b.TOP_H
    assert lane - j.WIRE_W / 2 >= (b.STRIP_SCREW_UP - b.TOP_H) + 2.85 + 0.25 + 0.8 - 1e-9
    assert lane + j.WIRE_W / 2 + j.WIRE_W <= wc.STRIP_REACH + 0.5


@pytest.mark.slow
def test_wire_clamps_sit_on_their_bodies_without_overlap() -> None:
    core, body = b.core(), b.j1_body()
    moved = b.as_child(body)
    assert solid(wc.band_clamp() & core).volume < 0.01 and wc.band_clamp().distance_to(core) < 0.01
    assert solid(wc.ear_clamp() & moved).volume < 0.01 and wc.ear_clamp().distance_to(moved) < 0.01
    assert (
        solid(wc.strip_clamp() & body).volume < 0.01 and wc.strip_clamp().distance_to(body) < 0.01
    )
    # 누르개 나사는 조여 넣는 구멍 속에 있다 (살과 겹치는 것은 나사산 몫뿐이다)
    assert solid(wc.strip_screw() & body).volume < 6.0


def test_link_feet_can_take_the_wire_over_the_body_or_around_the_brim() -> None:
    for name, (brim, over) in wr.link_crossings().items():
        # 두 길 다 멈춤 각에서 가장 길고, 범위 가운데에서 5 cm 넘게 남는다
        assert over.longest() == pytest.approx(
            max(over.length(q) for q in (-j.STOP_DEG, j.STOP_DEG))
        )
        assert over.slack() > 50.0 and brim.slack() > 50.0, name
    by = {w.name: w for w in wr.wires()}
    parts = dict(by["띠 밑 누르개 → J3"].parts)
    # 선은 두 길 가운데 긴 쪽에 맞춰 골라서, 어느 길로 가도 닿는다
    assert parts["J2 넘기"] == pytest.approx(max(c.longest() for c in wr.link_crossings()["J2"]))
    assert by["J3 → J4"].left > 100.0 and by["띠 밑 누르개 → J3"].left > 100.0


@pytest.mark.slow
def test_d405_wire_reaches_the_camera_pi_outside_the_joints() -> None:
    path = dict(wr.d405_outer_path())
    assert set(path) >= {"J4·J5 고리", "J3 고리", "J1·J2 고리"}
    # 가진 선(1,550 mm)이 카메라 파이의 USB 단자까지 닿고 50 mm 넘게 남는다
    assert wr.d405_left() >= 50.0
    for loop in (wr.d405_wrist_loop(), wr.d405_elbow_loop(), wr.d405_base_loop()):
        assert loop.length > loop.middle > loop.shortest > 0.0, loop
    # 베이스의 타이는 카메라 파이(왼쪽) 판의 몸통 쪽 긴 변이다: 팔이 앞을 볼 때 위팔의 타이와 같은 쪽에 있다
    _x, y, z = wr.d405_base_tie()
    assert y > b.R_CORE + 10.0 and abs(y - (b.PI_CZ - b.TRAY_HZ)) < 8.0 and z == pytest.approx(30.0)
    # 팔에 실리는 선은 46 g쯤이다 (층으로 넘기던 길보다 2 g 는다)
    assert sum(wr.wire_masses().values()) == pytest.approx(0.046, abs=0.002)


@pytest.mark.slow
def test_taut_d405_wire_cannot_clear_the_wrist_seams_from_any_exit_point() -> None:
    # 관절 밖 고리가 팽팽해지면, 손목 2의 어디에서 선이 떠나든 J4·J5의 여러 각에서 선이 이음매 둘레(멈춤 기둥과 귀가
    # 쓸고 가는 자리)에 놓인다. 받침의 모양만으로는 못 지키는 조건이라는 것을 숫자로 남긴다 (2026-11-22)
    for label, point in wr.D405_EXITS.items():
        found = wr.d405_taut_scan(point, "J5", step=20.0)
        assert found["어기는 칸"] / found["본 칸"] > 0.1, (label, found)
        assert found["가장 나쁜 여유"] < -5.0, (label, found)
    # 식 확인: 가까이 마주 본 두 자리는 곧게 이어져 축에서 멀다. 반 바퀴 넘게 감기면 원통에 붙는다
    keep = wr.SEAM_SWEEP_R + wr.D405_D / 2 + wr.SEAM_KEEP[0]
    zone = (-9.0, 15.0)
    near, short = wr._taut_path((60.0, 5.0), (60.0, -5.0), 0.3, 25.1, zone)
    assert near == pytest.approx(60.0 * math.cos(0.15) - keep, abs=0.05)
    assert short == pytest.approx(math.hypot(2 * 60.0 * math.sin(0.15), 10.0))
    wound, long = wr._taut_path((60.0, 5.0), (60.0, -5.0), 4.0, 25.1, zone)
    assert wound == pytest.approx(25.1 - keep, abs=0.3) and long > 150.0
