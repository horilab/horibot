"""출력 무게 어림식이 뽑아서 잰 부품과 맞는지, 층 가운데에 걸친 수평면을 찾는지."""

from __future__ import annotations

import pytest
from build123d import Box, Compound, Pos

from horilab_design.parts import joint_test as jt
from horilab_design.printcheck import layer_snap, mid_layer_levels, printed_mass_g
from horilab_design.values import load_measured

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
