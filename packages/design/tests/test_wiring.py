"""배선 검토 (0031): 관절을 넘는 선 길이, 구간마다 고른 선, 살 위 통로, 부품에 낸 선 자리."""

from __future__ import annotations

import math

import pytest
from build123d import Pos

from horilab_design.parts import base as bp
from horilab_design.parts import joint as j
from horilab_design.parts import links, wrist
from horilab_design.parts import shoulder as sp
from horilab_design.parts.shapes import box, solid
from horilab_design.studies import wiring as wr


def test_wire_is_longest_at_a_stop_and_slack_in_the_middle() -> None:
    for name, c in wr.crossings().items():
        longest = c.longest()
        assert longest == pytest.approx(max(c.length(q) for q in range(-163, 164)), abs=0.5), name
        # 범위 가운데에서 남는 길이는 테에 160° 안팎 감기는 길이다
        wrapped = math.radians(j.STOP_DEG) * (c.drum_r + j.WIRE_T / 2)
        assert c.slack(0.0) == pytest.approx(wrapped, rel=0.2), name
        assert c.slack(0.0) > c.slack(90.0) > c.slack(c.stop_deg) > -1e-9, name
    # J1은 두 누르개가 범위 가운데에서 같은 방위각이라 양쪽이 같다
    j1 = wr.crossings()["J1"]
    assert j1.length(160.0) == pytest.approx(j1.length(-160.0))


def test_each_wire_reaches_with_a_cable_on_hand() -> None:
    chosen = {w.name: w for w in wr.wires()}
    assert [w.cable for w in chosen.values()] == [900.0, 900.0, 900.0, 300.0, 300.0, 200.0]
    for w in chosen.values():
        assert w.left >= wr.SPARE, (w.name, w.need, w.left)
    # 산 300 mm 선으로는 아래팔을 지나는 J3 → J4가 닿지 않는다 (0016의 어림과 다르다)
    assert chosen["J3 → J4"].need > wr.usable(300.0)
    # 띠 밑 누르개에서 어댑터까지 20 cm 넘게 남는다
    assert chosen["띠 밑 누르개 → J3"].left > 200.0


def test_wire_crosses_the_web_where_the_stop_never_sweeps() -> None:
    for drum_r, post_r, ear in (
        (j.R_DRUM, j.R_POST, j.EAR_HW),
        (sp.R_DRUM, sp.R_POST, sp.EAR_HW),
    ):
        post_soft, ear_soft = wr.corridor(drum_r, post_r, j.SOFT_DEG, ear)
        post_hard, ear_hard = wr.corridor(drum_r, post_r, j.STOP_DEG, ear)
        # 프로그램 한계에서는 선 옆으로 1.5 mm 넘게 남고, 멈춤 각에서도 선에 닿지 않는다
        assert min(post_soft, ear_soft) - j.WIRE_W / 2 > 1.5
        assert min(post_hard, ear_hard) - j.WIRE_W / 2 > 0.4
    # 기둥이 살에 닿는 각에서 기둥 가장자리가 살의 옆면 가까이 있다 (둥근 기둥이 비스듬한 옆면에 닿는다)
    web = j.R_POST * math.sin(math.radians(j.WEB_HALF))
    assert wr.corridor(j.R_DRUM, j.R_POST, j.STOP_DEG)[0] == pytest.approx(web, abs=0.15)
    # 손목 몸통의 옆 홈에서 나온 선은 멈춤 귀 끝 가까이를 지나고, 가운데 홈에서 나온 선은 멀리 지난다
    side = sum(wrist.NOTCH) / 2
    assert wr.exit_to_ear(side, j.STOP_DEG) - j.WIRE_W / 2 < 1.5
    assert wr.exit_to_ear(0.0, j.STOP_DEG) - j.WIRE_W / 2 > 5.0
    # 손목 몸통의 가운데 홈과 링크의 선 창은 선 폭보다 넓고, 플러그는 창을 지나간다
    assert 2 * wrist.EXIT_HW > j.WIRE_W + 1.0
    # 손목 몸통의 옆 홈과 링크 발의 선 틈(5.7)도 선 폭(넓은 쪽, Waveshare 선 4.85)보다 0.8 넘게 넓다. 긴 틈은 선을
    # 세워 밀어 넣을 만큼 구멍과 겹친다
    assert wrist.NOTCH[1] - wrist.NOTCH[0] > j.WIRE_W + 0.8
    assert links.FOOT_SLOT_Z[1] - links.FOOT_SLOT_Z[0] > j.WIRE_W + 0.8
    assert links.FOOT_SLOT_OPEN > j.WIRE_T + 0.9 and links.FOOT_SLOT_H > j.WIRE_T + 1.0
    for foot in (links.STS_FOOT, links.SHOULDER_FOOT):
        assert (
            links.FOOT_SLOT_Z[1] <= foot.hw - links.WALL + 1e-9
        )  # 발 상자의 안쪽 벽을 넘지 않는다
    # 서보 먼 끝 밑의 길은 혼 볼트 머리와 목 안쪽 벽 사이이고, 선 폭이 겨우 든다
    head_edge = max(abs(z) for _x, z in j.bolt_xz()) + 2.85
    assert j.NECK_IN - head_edge > j.WIRE_W
    assert wrist.SIDE_LANE == pytest.approx((head_edge + j.NECK_IN) / 2, abs=0.05)
    assert wrist.HEAD > j.WIRE_T + 1.0
    assert 2 * j.WIN_Z >= wr.PLUG[2] + 1.0 and j.WIN_X[1] - j.WIN_X[0] >= wr.PLUG[1] + 0.5
    assert 2 * sp.WIN_Z >= wr.PLUG[2] + 1.0 and sp.WIN_X[1] - sp.WIN_X[0] >= wr.PLUG[1] + 0.5


def test_wrist2_wire_leaves_the_cover_above_the_pin_ear() -> None:
    c = wr.over_wrist2()
    # 선은 덮개에서 축에서 멀어지는 쪽으로 곧게 나온다 (아래를 보고 나와 U자로 꺾이지 않는다)
    assert c.face[1] == c.start[1] == wrist.EXIT_PLUG_Z and c.start[0] < c.face[0] < 0
    # 귀 윗면에서 6 mm, 발 나사 머리에서 5 mm 넘게 위에서 나오고, 부모의 앞면(영점 핀 구멍이 있는 살)에서 18 mm 넘게 뜬다
    low = c.h - j.WIRE_T / 2
    assert low == pytest.approx(wrist.EXIT_H)
    assert low - j.EAR_T > 6.0 and low - (j.EAR_T + wrist.HEAD_H) > 5.0
    assert low + j.ATOP - j.FRONT > 18.0
    # 몸통 둘레를 그 높이로 돌 때 선의 위 가장자리는 D405 받침의 돋움(발 위 21.2)보다 4 mm 넘게 아래다
    boss_low = -wrist.MOUNT_X - wrist.MOUNT_HW - wrist.X_FOOT
    assert boss_low - (c.h + j.WIRE_W / 2) > 4.0
    # 범위 가운데에서 나가는 자리와 들어가는 자리가 거의 같은 방위각이다 (양쪽으로 비슷하게 감긴다)
    assert abs(c.exit_az - c.entry_az) < 10.0
    assert c.length(c.stop_deg) == pytest.approx(c.length(-c.stop_deg), abs=5.0)
    # 손목 1의 홈에서 곧게 올라오는 선: 옆 홈이면 영점 핀 귀가 쓸고 가는 자리와 2.5 mm 넘게 뜨고, 가운데 홈이면 2 mm가 안 된다
    lumps = wr.wrist_lumps(True, False)
    reach = j.R_PIN + wrist.PIN_EAR_R
    assert max(math.hypot(u, v) for u, v in wrist.ear_section(True)) == pytest.approx(
        reach, abs=0.05
    )
    assert wr.strand_gap(wr.wrist1_strand(True), lumps, "영점 핀 귀") > 2.5
    assert wr.strand_gap(wr.wrist1_strand(False), lumps, "영점 핀 귀") < 2.0
    # J5 → J6 선: 손목 1에서는 +z 쪽 옆 홈으로 곧게 나가고, 손목 2 속은 창에서 플러그까지 20 mm가 안 된다
    out = wr.wrist_out(side=True)
    assert out[-1][2] == pytest.approx(wrist.SIDE_LANE) and out[-1][1] == j.FRONT
    assert wrist.NOTCH[0] + j.WIRE_W / 2 < wrist.SIDE_LANE < wrist.NOTCH[1] - j.WIRE_W / 2
    assert 10.0 < wr.length(wr.wrist_in(front=False)) < 20.0


@pytest.mark.slow
def test_wrist2_wire_has_a_clear_route_at_every_angle() -> None:
    c = wr.over_wrist2()
    strand = wr.wrist1_strand(True)
    routes = wr.clear_routes(c, wr.wrist_lumps(True, False), strand, (c.h,))
    assert [r.q for r in routes] == [float(q) for q in range(-163, 164)]
    # 멈춤 각에서 멈춤 각까지 1° 간격으로, 귀와 발 나사 머리, 앞 관절의 멈춤 귀를 2.5 mm 넘게 피하는 길이 있다
    assert min(r.gap for r in routes) > 2.5
    # 그 길은 300 mm 선에서 두 몸통 속의 길을 뺀 길이보다 짧다
    by = dict(next(w for w in wr.wires() if w.name == "J5 → J6").parts)
    assert (
        max(r.length for r in routes) < wr.usable(300.0) - by["손목 1 속"] - by["손목 2 속"] - 30.0
    )
    # 핀 자세(-90°)에서 귀는 부모 쪽 구간 바로 안쪽을 지난다: 그 각 근처의 가장 좁은 곳이 부모 쪽 구간과 귀 사이다
    pin = routes[163 + round(wrist.PIN_POSE_DEG)]
    assert pin.near == "영점 핀 귀" and pin.gap > 2.5
    # D405를 달면 닿지 않는 길은 있지만, +93 ~ +121°에서 받침의 클립 밑으로 지나느라 2 mm보다 좁다 (받침과 선을 달고
    # 돌려 볼 자리, 0031의 남은 확인). 클립은 상자로, 선은 지름이 폭인 둥근 선으로 본 값이라 실제보다 좁게 나온다
    cam = wr.clear_routes(c, wr.wrist_lumps(True, True), strand, (c.h,))
    assert min(r.gap for r in cam) > 0.0
    narrow = [r.q for r in cam if r.gap < 2.0]
    assert narrow and 90.0 <= narrow[0] and narrow[-1] <= 125.0
    assert all(r.gap > 2.5 for r in cam if r.q < 0)  # 귀 쪽(-)은 D405와 상관없다


@pytest.mark.slow
def test_wrist1_wire_leaves_at_ear_height_but_away_from_the_ears() -> None:
    # J4: 손목 1의 옆 홈(-z)은 귀와 같은 높이다 (앞벽의 그 위는 J5의 멈춤 홈이라 올릴 자리가 없다). 대신 축에서
    # 멀어지는 쪽으로 나오고, 축에서 본 영점 핀 귀까지 25 mm 넘게 떨어져 있다
    c = wr.over_wrist1()
    assert c.flip and c.h + j.WIRE_T / 2 < j.EAR_T
    assert c.face[1] == c.start[1] < 0 < c.face[0] < c.start[0]
    assert math.dist(c.face, c.start) == pytest.approx(6.0)
    ear = [(u, v) for u, v in wrist.ear_section(True)]
    assert min(math.dist(c.start, p) for p in ear) > 25.0
    # 홈 위로 J5의 멈춤 홈까지 남은 살(2.5 mm)을 다 써도 홈이 귀 윗면보다 낮다
    assert wrist.NOTCH_H + (-j.CH_R1 - (wrist.X_FOOT + wrist.NOTCH_H)) < j.EAR_T
    # 아래팔의 선 창에서 올라오는 선은 창의 축 쪽 끝에 놓여도 귀가 쓸고 가는 자리와 2 mm 뜬다
    lumps = wr.wrist_lumps(False, False)
    assert wr.strand_gap(wr.forearm_strand(), lumps, "영점 핀 귀") == pytest.approx(2.0, abs=0.05)
    # 발 높이로만 돌면 귀에 닿고, 홈에서 나와 귀와 나사 머리 위로 올라서 돌면 2 mm 띄우는 길이 있다
    low = wr.clear_routes(c, lumps, wr.forearm_strand(), (c.h,))
    assert min(r.gap for r in low) < 0.0
    over = wr.clear_routes(c, lumps, wr.forearm_strand(), (c.h, wr.WRAP_OVER))
    assert min(r.gap for r in over) > 1.95


def test_gap_under_the_brim_is_too_narrow_for_a_wire() -> None:
    assert j.LIP_GAP < j.WIRE_T - 0.4
    assert j.LIP_GAP >= 0.6  # 도는 원판과 서 있는 원통 사이의 축 방향 틈


def test_d405_wire_wraps_the_brims_over_the_servo_wire() -> None:
    # 두 몸통 사이 층(5.8 mm)에 감기려면 선이 3.8 mm보다 가늘어야 한다. 얇은 USB 2 선(3.0)은 들고, 기본 선(5.12)은 안 든다
    assert wr.layer_height() == pytest.approx(j.GAP + j.BRIM_T)
    assert wr.d405_max_diameter() == pytest.approx(3.8)
    assert wr.D405_D == pytest.approx(3.0) and wr.D405_OLD_D == pytest.approx(5.12)
    assert wr.D405_D + 0.5 < wr.d405_max_diameter() < wr.D405_OLD_D
    # 서보 선과 같은 방위각으로 넘는다 (부모는 살 쪽, 자식은 멈춤 귀의 맞은편)
    x = wr.d405_crossings()
    assert set(x) == {"J1", "J2", "J3", "J4", "J5"}
    for name, c in x.items():
        assert c.exit_az == c.entry_az == 180.0, name
        # 감기는 반지름은 기준(지름의 8배)보다 크다
        assert wr.d405_wrap_radius(name) >= wr.WRAP_RATIO * wr.D405_D, name
    # 서보 선 위에 겹치므로 서보 선보다 바깥에서 감긴다 (J1은 띠에 나란히 놓인다)
    assert wr.d405_wrap_radius("J5") == pytest.approx(j.R_DRUM + j.WIRE_T + wr.D405_D / 2)
    assert wr.d405_wrap_radius("J1") == pytest.approx(bp.R_CORE + wr.D405_D / 2)
    # 감기는 쪽이 바뀌며 꺾이는 자리는 기준(지름의 4배)보다 완만하다. 손목 1은 선을 잡지 않는다
    for name in ("J2", "J3", "J4"):
        assert wr.d405_lead_radius(name) >= wr.FLEX_RATIO * wr.D405_D, name
    for name in ("J2", "J3", "J5"):
        lead = wr.d405_child_lead(name)
        assert lead is not None and lead >= wr.FLEX_RATIO * wr.D405_D, name
    assert wr.d405_child_lead("J4") is None
    assert 2 * wr.FLEX_RATIO * wr.D405_D > wr.d405_wrist1_gap() > 10.0
    # 멈춤 각에서도 선이 팽팽하지 않고(D405_SPARE), 멈춤 귀가 선에서 2 mm 넘게 뜬다
    for name, c in x.items():
        wd = wr.D405_D if name == "J1" else wr.D405_OVER
        assert dict(wr.d405_path())[f"{name} 넘기"] - c.longest(wd) == pytest.approx(wr.D405_SPARE)
    assert wr.d405_ear_gap(j.R_DRUM, j.EAR_HW) > 2.0
    assert wr.d405_ear_gap(sp.R_DRUM, sp.EAR_HW) > 2.0
    # 고리 식: 반원(현 2R, 호 πR)이면 R이 나오고, 선이 두 끝 사이보다 짧으면 고리가 없다
    assert wr.loop_radius(40.0, math.pi * 20.0) == pytest.approx(20.0, rel=1e-6)
    assert wr.loop_radius(50.0, 40.0) == math.inf
    # 가진 선(1,550 mm)은 베이스의 띠 밑 누르개까지 가고 40 cm 넘게 남는다 (베이스에서 파이까지 갈 몫)
    total = sum(v for _, v in wr.d405_path())
    assert 950.0 < total < wr.D405_STOCK_MM - 400.0


def test_wire_masses_for_the_arm_model() -> None:
    masses = wr.wire_masses()
    assert set(masses) == {"upper_mid", "upper_end", "fore_mid", "fore_end", "wrist1", "wrist2"}
    assert sum(masses.values()) == pytest.approx(0.044, abs=0.004)
    # 20 cm 선의 무게(실측 3.5 g)가 식에서 다시 나온다
    again = wr.usable(200.0) * 1e-3 * wr.SERVO_WIRE_KG_M + 2 * wr.PLUG_KG
    assert again == pytest.approx(3.5e-3)


def test_base_clamps_line_up_in_the_middle_of_the_range() -> None:
    x, z = bp.EAR_CLAMP
    assert math.degrees(math.atan2(z, x)) % 360.0 == pytest.approx(bp.WIRE_AZ, abs=1.0)
    # 귀 위 누르개 나사는 귀 안에 있고(가장자리에서 3 mm 넘게), 영점 핀 구멍에서 멀다
    assert bp.R_TABLE - math.hypot(x, z) > 3.0
    assert math.dist((x, z), (-bp.R_PIN, 0.0)) > 10.0
    # 돌림판 밑의 테두리는 몸통 윗면의 바깥 벽 위에 있다 (멈춤 홈 밖)
    assert bp.CH_R[1] < bp.TABLE_LIP[0] and bp.TABLE_LIP[1] <= bp.R_TABLE + bp.FLARE


@pytest.mark.slow
def test_wire_openings_are_open_in_the_printed_bodies() -> None:
    # 링크 발: 끝벽 밑의 구멍과 발 바닥의 긴 틈이 발 속까지 뚫려 있다
    for name, (length, tube_r, foot) in links.LINKS.items():
        part = links.link(length, tube_r, foot)
        xa = -length
        z0, z1 = links.FOOT_SLOT_Z
        mouse = box(
            xa - foot.hx - 0.5,
            xa - foot.hx + links.WALL + 0.5,
            j.FRONT - 2.0,
            j.FRONT - 0.2,
            z0 + 0.5,
            z1 - 0.5,
        )
        floor = box(
            xa - foot.hx + links.WALL + 1.0,
            xa - foot.head_r,
            j.FRONT - links.WALL - 0.8,
            j.FRONT - 0.2,
            z0 + 0.5,
            z1 - 0.5,
        )
        for probe in (mouse, floor):
            assert solid(part & probe).volume < 0.01, name
    # 손목 몸통: 앞벽 발 쪽 끝의 가운데 홈과 옆 홈
    body = wrist.wrist_body()
    for zc in (0.0, sum(wrist.NOTCH) / 2, -sum(wrist.NOTCH) / 2):
        probe = box(
            wrist.X_FOOT + 0.2, wrist.X_FOOT + 2.0, j.NECK_FRONT_IN, j.FRONT, zc - 1.5, zc + 1.5
        )
        assert solid(body & probe).volume < 0.01, zc
    # 출력 원판의 테두리가 원통 앞 끝과 LIP_GAP만 띄운다
    disc = j.out_disc()
    lip = box(
        -j.R_DRUM, -j.R_DRUM + j.LIP_W - 0.2, j.FRONT + j.LIP_GAP + 0.05, j.SEAM - 0.05, -1.0, 1.0
    )
    assert solid(disc & lip).volume == pytest.approx(lip.volume, rel=0.1)


@pytest.mark.slow
def test_base_band_and_clamp_holes() -> None:
    core = bp.core()
    assert core.is_valid and len(core.solids()) == 1
    # 띠의 위 턱이 몸통보다 넓고, 누르개 나사 구멍이 띠 밑에 뚫려 있다
    assert core.bounding_box().size.X == pytest.approx(2 * (bp.R_CORE + bp.BAND_LIP), abs=0.3)
    az = math.radians(bp.WIRE_AZ)
    y = bp.FRONT - bp.BAND_H - bp.CLAMP_DROP
    r = bp.R_CORE - 3.0
    hole = solid(
        Pos(r * math.cos(az), 0, r * math.sin(az)) * box(-0.6, 0.6, y - 0.6, y + 0.6, -0.6, 0.6)
    )
    assert solid(core & hole).volume < 0.01
    # J1 몸통 뒤쪽 귀의 누르개 나사 구멍
    body = bp.as_child(bp.j1_body())
    x, z = bp.EAR_CLAMP
    probe = box(x - 0.6, x + 0.6, bp.ATOP + 1.0, bp.ATOP + j.EAR_T - 1.0, z - 0.6, z + 0.6)
    assert solid(body & probe).volume < 0.01
