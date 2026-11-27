"""보기 모델 (mjcf_view, 0033의 시뮬레이션 화면): 바디·관절이 시뮬레이션 모델과 같고, 부품이 그 틀의 자리에 붙는다.

모든 부품을 불러오는 쓰기(write)는 30초쯤 걸려서 여기서는 작은 상자로 본다.
"""

from __future__ import annotations

import tomllib
from pathlib import Path

import mujoco
import numpy as np
import pytest
from build123d import Box, Pos

from horibot_design import collision
from horibot_design import placement as pl
from horibot_design.mjcf import add_gripper, build_spec
from horibot_design.mjcf_view import (
    PART_GROUP,
    PIECE_GROUP,
    PieceSet,
    Plane,
    ViewPart,
    build_view_spec,
    settings_planes_and_rest,
)
from horibot_design.parts import gripper
from horibot_design.robot import Mat
from horibot_design.values import REPO_ROOT, load_robot

RP, OPEN = gripper.RP * 1e-3, gripper.OPEN_MAX * 1e-3
REST = [-55.0, 52.0, 154.0, 325.0, 0.0, 0.0]
JOINTS = [f"j{k}" for k in range(1, 7)]


@pytest.fixture(scope="module")
def frames() -> dict[str, tuple[int, Mat]]:
    return collision.frame_transforms()


def _view(
    frames: dict[str, tuple[int, Mat]],
    mesh_dir: Path,
    parts: list[ViewPart] | None = None,
    pieces: list[PieceSet] | None = None,
    planes: list[Plane] | None = None,
) -> mujoco.MjModel:
    spec = build_view_spec(
        load_robot(), frames, parts or [], pieces or [], planes or [], REST, mesh_dir, RP, OPEN
    )
    return spec.compile()


def _pose(model: mujoco.MjModel, q: np.ndarray) -> mujoco.MjData:
    data = mujoco.MjData(model)
    for name, angle in zip(JOINTS, q, strict=True):
        data.qpos[model.joint(name).qposadr[0]] = angle
    mujoco.mj_kinematics(model, data)
    return data


def test_bodies_and_joints_match_the_simulation_model(
    frames: dict[str, tuple[int, Mat]], tmp_path: Path
) -> None:
    robot = load_robot()
    spec = build_spec(robot)
    add_gripper(spec, robot, RP, OPEN)
    physics = spec.compile()
    view = _view(frames, tmp_path)
    for name in [*JOINTS, "gripper"]:
        a, b = physics.joint(name), view.joint(name)
        assert np.allclose(physics.jnt_axis[a.id], view.jnt_axis[b.id])
        assert np.allclose(physics.jnt_range[a.id], view.jnt_range[b.id])
        assert np.allclose(physics.body_pos[a.bodyid[0]], view.body_pos[b.bodyid[0]])
        assert np.allclose(physics.body_quat[a.bodyid[0]], view.body_quat[b.bodyid[0]])


def test_a_part_sits_in_its_frame(frames: dict[str, tuple[int, Mat]], tmp_path: Path) -> None:
    center = np.array([5.0, -10.0, 40.0])  # 틀 좌표 [mm]
    part = ViewPart("시험/상자", "j3_out", "body", None, Pos(*center) * Box(10, 20, 30))
    model = _view(frames, tmp_path, parts=[part])
    q = np.radians([30.0, -40.0, 70.0, 200.0, -50.0, 20.0])
    data = _pose(model, q)
    meshes = [g for g in range(model.ngeom) if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH]
    assert len(meshes) == 1 and model.geom_group[meshes[0]] == PART_GROUP
    expected = pl.frames_mm(q)["j3_out"] @ np.append(center, 1.0)
    assert np.allclose(data.geom_xpos[meshes[0]], expected[:3] / 1000, atol=1e-6)


def test_finger_parts_slide_along_the_opening(
    frames: dict[str, tuple[int, Mat]], tmp_path: Path
) -> None:
    slide = (0.5, 0.0, 0.0)  # 열림 폭 1 mm마다 틀 x로 0.5 mm
    part = ViewPart("시험/손가락", "flange", "metal", slide, Box(4, 4, 4))
    model = _view(frames, tmp_path, parts=[part])
    finger = model.joint("finger1")
    assert finger.range[1] == pytest.approx(OPEN * 0.5)
    q = np.radians([10.0, 20.0, 30.0, 170.0, 40.0, 50.0])
    closed = _pose(model, q).geom_xpos[-1].copy()
    data = mujoco.MjData(model)
    data.qpos[:] = _pose(model, q).qpos
    data.qpos[finger.qposadr[0]] = finger.range[1]
    mujoco.mj_kinematics(model, data)
    T = pl.frames_mm(q)["flange"]
    expected = T[:3, :3] @ np.array(slide) * gripper.OPEN_MAX / 1000
    assert np.allclose(data.geom_xpos[-1] - closed, expected, atol=1e-6)


def test_rest_key_planes_and_hidden_pieces(
    frames: dict[str, tuple[int, Mat]], tmp_path: Path
) -> None:
    planes = [Plane("책상", (0.0, 0.0, 1.0), 0.0), Plane("벽", (1.0, 0.0, 0.0), -0.211)]
    tetra = np.array([[0, 0, 0], [10, 0, 0], [0, 10, 0], [0, 0, 10]], dtype=float)
    pieces = [PieceSet("시험/조각", "j2_out", None, "k", (tetra,))]
    model = _view(frames, tmp_path, pieces=pieces, planes=planes)
    key = model.key("rest")
    for name, angle in zip(JOINTS, REST, strict=True):
        assert key.qpos[model.joint(name).qposadr[0]] == pytest.approx(np.radians(angle))
    # 벽은 반투명 판으로, 책상 면은 판을 그리지 않는다 (책상 판 형상이 보인다)
    wall = model.geom("plane 벽")
    assert wall.pos[0] == pytest.approx(-0.211 - 0.001)
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "plane 책상") == -1
    # 부딪힘 검사의 조각은 무리 3: MuJoCo 창은 처음에 무리 0 ~ 2만 그린다
    groups = {int(model.geom_group[g]) for g in range(model.ngeom)}
    assert PIECE_GROUP in groups
    assert mujoco.MjvOption().geomgroup[PIECE_GROUP] == 0


def test_settings_planes_and_rest_come_from_the_robot_settings() -> None:
    settings = tomllib.loads((REPO_ROOT / "data" / "robot.toml").read_text(encoding="utf-8"))
    planes, rest = settings_planes_and_rest()
    assert rest == pytest.approx(settings["poses"]["rest_deg"])
    assert [p.name for p in planes] == [p["name"] for p in settings["collision"]["planes"]]
