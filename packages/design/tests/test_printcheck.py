"""출력 무게 어림식이 뽑아서 잰 부품과 맞는지, 층 가운데에 걸친 수평면과 공중에 뽑히는 벽 선을 찾는지."""

from __future__ import annotations

import pytest
from build123d import Box, Compound, Cylinder, Pos

from horibot_design.parts import joint_test as jt
from horibot_design.printcheck import (
    WALLS_ONLY_SPAN,
    air_walls,
    layer_snap,
    mid_layer_levels,
    printed_mass_g,
    wall_settings,
)
from horibot_design.values import load_measured

UP = (0.0, 0.0, 1.0)

# 0.20mm Strength로 뽑아 잰 부품. 게이지 받침(gauge_bracket)은 잰 값이 형상의 0.33배라 식과 맞지 않아 뺐다
# (뽑은 뒤 형상을 고쳤을 수 있다)
STRENGTH_PARTS = ("elbow_house", "elbow_disc_out", "elbow_disc_back", "test_link", "test_stand")


@pytest.mark.slow
def test_mass_estimate_matches_weighed_parts() -> None:
    weighed = load_measured().printed_mass_g
    for spec in jt.PARTS:
        if spec.name not in STRENGTH_PARTS:
            continue
        shape = jt.placed_for_print(spec)  # 뽑는 방향으로 놓은 형상 (z가 위)
        estimate = printed_mass_g(shape, (0.0, 0.0, 1.0))
        assert estimate == pytest.approx(weighed[spec.name], rel=0.03), spec.name


def test_mid_layer_levels_finds_faces_the_slicer_cannot_place() -> None:
    up = (0.0, 0.0, 1.0)
    # 바닥 판(두께 2.5) 위에 기둥: 판 윗면이 2.4 ~ 2.6 층의 한가운데다
    bad = Compound([Pos(0, 0, 1.25) * Box(20, 20, 2.5) + Pos(0, 0, 4.0) * Box(6, 6, 8.0)])
    assert mid_layer_levels(bad, up) == [pytest.approx(2.5)]
    # 0.02 mm 벗어난 면(2.52)은 뽑히는 층이 정해져 있다
    near = Compound([Pos(0, 0, 1.26) * Box(20, 20, 2.52) + Pos(0, 0, 4.0) * Box(6, 6, 8.0)])
    assert mid_layer_levels(near, up) == []
    good = Compound([Pos(0, 0, 1.2) * Box(20, 20, 2.4) + Pos(0, 0, 4.0) * Box(6, 6, 8.0)])
    assert mid_layer_levels(good, up) == []
    # 뽑는 방향을 뒤집으면 바닥이 바뀐다: 기둥 끝이 바닥이고 판의 밑면이 5.5, 윗면이 8.0에 온다
    assert mid_layer_levels(bad, (0.0, 0.0, -1.0)) == [pytest.approx(5.5)]
    assert layer_snap(2.52) == pytest.approx(2.6) and layer_snap(2.48) == pytest.approx(2.4)


def test_wall_settings_come_from_the_print_values() -> None:
    strength, standard = wall_settings("strength"), wall_settings("standard")
    assert (strength.loops, standard.loops) == (6, 2)
    # 바깥 벽 가운데 0.21, 둘째 0.60, 그다음은 0.407씩: 벽 6겹의 띠가 약 2.45 mm다
    assert strength.offsets()[:2] == [pytest.approx(0.21), pytest.approx(0.602, abs=1e-3)]
    assert strength.offsets()[-1] + strength.inner_w / 2 == pytest.approx(2.45, abs=0.01)


def test_hole_in_a_ceiling_leaves_the_ring_around_it_in_the_air() -> None:
    # 바닥 쪽 나사 머리 자리(지름 6.6, 깊이 1.8)에 구멍(3.4)이 뚫렸다: 첫 관절의 출력 원판 귀와 같다
    block = Pos(0, 0, 3) * Box(20, 20, 6) - Pos(0, 0, 0.9) * Cylinder(3.3, 1.8)
    block = block - Pos(0, 0, 3) * Cylinder(1.7, 6)
    for preset in ("strength", "standard"):
        found = air_walls(Compound([block]), UP, wall_settings(preset))
        assert [(a.kind, a.height) for a in found] == [("ring", pytest.approx(1.8))], preset
        assert found[0].where[:2] == pytest.approx((0.0, 0.0), abs=0.2)
        assert found[0].blocking
    # 구멍이 없는 머리 자리 천장은 다리 선으로 덮인다
    plain = Pos(0, 0, 3) * Box(20, 20, 6) - Pos(0, 0, 0.9) * Cylinder(3.3, 1.8)
    assert air_walls(Compound([plain]), UP, wall_settings("strength")) == ()


def test_narrow_bar_over_a_long_gap_is_walls_only() -> None:
    # 폭 4.8 가로대가 20 mm 틈을 건넌다 (양옆이 트였다): 벽 6겹의 띠 둘에 다 들어가 다리 선이 없다
    long = Pos(0, 0, 1.5) * Box(30, 4.8, 3) - Pos(0, 0, 0.5) * Box(20, 6, 1)
    found = air_walls(Compound([long]), UP, wall_settings("strength"))
    assert {a.kind for a in found} == {"span"}
    assert all(a.length > WALLS_ONLY_SPAN and a.height == pytest.approx(1.0) for a in found)
    # 8 mm 틈은 한도(10 mm) 안이다
    short = Pos(0, 0, 1.5) * Box(18, 4.8, 3) - Pos(0, 0, 0.5) * Box(8, 6, 1)
    assert air_walls(Compound([short]), UP, wall_settings("strength")) == ()


def test_wide_ceiling_is_bridged_and_its_edges_are_only_reported() -> None:
    # 닫힌 주머니의 천장은 윤곽에 닿지 않아 다리 선으로만 채워진다
    pocket = Pos(0, 0, 2) * Box(30, 30, 4) - Pos(0, 0, 0.5) * Box(15, 15, 1)
    assert air_walls(Compound([pocket]), UP, wall_settings("strength")) == ()
    # 양옆이 트인 넓은 홈: 가장자리 벽 선이 다리 선 옆으로 14 mm쯤 건넌다 (손목 몸통 멈춤 홈과 같은 모양)
    channel = Pos(0, 0, 1.5) * Box(30, 20, 3) - Pos(0, 0, 0.5) * Box(14, 22, 1)
    found = air_walls(Compound([channel]), UP, wall_settings("strength"))
    assert found and {a.kind for a in found} == {"edge"}
    assert not any(a.blocking for a in found)
