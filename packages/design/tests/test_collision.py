"""부딪힘 검사의 형상 (horibot_design.collision, 0041): 자료가 지금 설계와 같은지, 조각이 겉을 감싸는지, 쉬는
자세를 떠나는 길이 선반을 파고들지 않는지."""

from __future__ import annotations

import itertools
import tomllib
from pathlib import Path

import numpy as np
import pytest
import trimesh

from horibot_design import collision as c
from horibot_design import placement as pl
from horibot_design.parts.shapes import far_apart

ROBOT_TOML = Path(__file__).resolve().parents[3] / "data" / "robot.toml"


def test_coverage_sees_a_piece_that_misses_the_surface() -> None:
    box = trimesh.creation.box(extents=(10.0, 10.0, 10.0))
    V, F = np.asarray(box.vertices), np.asarray(box.faces)
    assert c.coverage_mm(V, F, [V]) == 0.0
    # 한쪽 면을 0.3 mm 안으로 들인 조각은 그 면을 0.3 mm 못 감싼다
    short = V.copy()
    short[short[:, 0] > 0, 0] -= 0.3
    assert c.coverage_mm(V, F, [short]) == pytest.approx(0.3, abs=1e-6)


@pytest.mark.slow
def test_decompose_covers_a_bent_part() -> None:
    # ㄱ자 덩어리(오목하다)와 떨어진 작은 상자: 덩어리마다 나뉘고 겉을 감싼다
    a = trimesh.creation.box(extents=(40.0, 10.0, 10.0))
    b = trimesh.creation.box(extents=(10.0, 40.0, 10.0))
    b.apply_translation((15.0, 15.0, 0.0))
    bent = trimesh.boolean.union([a, b], engine="manifold")
    assert isinstance(bent, trimesh.Trimesh)
    far = trimesh.creation.box(extents=(5.0, 5.0, 5.0))
    far.apply_translation((100.0, 0.0, 0.0))
    mesh = trimesh.util.concatenate([bent, far])
    assert isinstance(mesh, trimesh.Trimesh)
    V, F = np.asarray(mesh.vertices), np.asarray(mesh.faces)
    pieces, how = c.decompose(V, F, hardware=False, force_hull=False)
    assert how.startswith("coacd(2덩어리)")
    assert len(pieces) >= 3  # ㄱ자는 둘 넘게, 상자는 하나
    _, how, cover = c.finish(V, F, pieces, how, comps_hull=False)
    assert cover <= c.COVERAGE_LIMIT_MM


@pytest.mark.slow
def test_collision_data_matches_the_design() -> None:
    # 설계를 고치고 build를 돌리지 않으면 여기서 걸린다: uv run python -m horibot_design.collision build
    assert c.check() == []


@pytest.mark.slow
def test_leaving_the_rest_pose_lifts_off_the_shelf() -> None:
    # 쉬는 자세에서 쉬는 자세 위로 가는 길(컨트롤러가 선반 닿음을 봐주는 유일한 길, 로봇 설정의 rest_contact)에서
    # 설계 형상의 아래팔과 선반은 겹치지 않고 점점 멀어진다
    poses = tomllib.loads(ROBOT_TOML.read_text(encoding="utf-8"))["poses"]
    rest, above = np.array(poses["rest_deg"]), np.array(poses["rest_above_deg"])
    shelf = [p for p in pl.arm_parts("rest_shelf")]
    fore = [p for p in pl.arm_parts("forearm")]
    distances = []
    for s in np.linspace(0.0, 1.0, 6):
        placed = pl.place(shelf + fore, np.radians(rest + (above - rest) * s))
        a, b = placed[: len(shelf)], placed[len(shelf) :]
        d = min(
            (sa.distance_to(sb) for _, sa in a for _, sb in b if not far_apart(sa, sb, 20.0)),
            default=20.0,
        )
        distances.append(d)
    assert distances[0] > 0.0, distances  # 쉬는 자세에서도 파고들지 않는다
    assert all(b >= a - 1e-3 for a, b in itertools.pairwise(distances)), distances
    assert distances[-1] > 8.0, distances  # 쉬는 자세 위에서는 여유(3 mm)보다 넉넉히 떨어진다
