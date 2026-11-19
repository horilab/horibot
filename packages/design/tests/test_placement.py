"""팔 부품을 관절 각대로 놓는 코드: 부품의 쌓임으로 만든 틀이 막대 모델(studies/ur_arm.py)의 골격과 맞는지,
관절마다 설계 각과 운동학 각의 부호가 맞는지."""

from __future__ import annotations

import numpy as np
import pytest

from horilab_design import placement as pl
from horilab_design.parts import joint as j
from horilab_design.parts import wrist
from horilab_design.parts.shapes import box, solid
from horilab_design.robot import Robot, flange_pose, link_poses
from horilab_design.studies import ur_arm
from horilab_design.studies.packaging import MM

# 영점, 범위의 가운데, 관절마다 부호가 다르게 섞인 자세들 [°]. J4는 범위가 180° 둘레다
POSES_DEG = (
    (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    pl.RANGE_MID_DEG,
    (40.0, -55.0, 80.0, 150.0, -70.0, 25.0),
    (-130.0, 20.0, -95.0, 230.0, 110.0, -140.0),
    (15.0, 100.0, 35.0, 185.0, 5.0, 90.0),
)


@pytest.fixture(scope="module")
def robot() -> Robot:
    return ur_arm.robot_of(ur_arm.Concept())


def test_turn_is_the_same_rotation_as_the_part_code() -> None:
    # 설계 각의 뜻은 joint.turned 하나다: 구멍과 홈 자리도 그 함수가 정한다
    shape = solid(box(10.0, 12.0, 3.0, 5.0, -1.0, 1.0))
    for q in (30.0, -110.0):
        c = j.turned(shape, q).center()
        expected = pl.turn(q) @ np.array([11.0, 4.0, 0.0, 1.0])
        assert (c.X, c.Y, c.Z) == pytest.approx(tuple(expected[:3]), abs=1e-6)
    # 방위각(+x가 0°, +z가 90°)이 커지는 쪽이 +다
    assert (pl.turn(90.0) @ np.array([1.0, 0.0, 0.0, 1.0]))[:3] == pytest.approx((0.0, 0.0, 1.0))


@pytest.mark.parametrize("pose_deg", POSES_DEG)
def test_servo_axes_lie_on_the_joint_axes_of_the_rod_model(
    robot: Robot, pose_deg: tuple[float, ...]
) -> None:
    q = np.radians(pose_deg)
    frames = pl.frames_mm(q)
    poses = link_poses(robot, q)
    for k, joint in enumerate(robot.joints, start=1):
        T_base_link = poses[k - 1]
        axis = T_base_link[:3, :3] @ np.asarray(joint.axis)
        point = T_base_link[:3, 3]  # 관절 원점 [m]
        frame = frames[f"j{k}"]
        out = frame[:3, 1]  # 서보의 출력 쪽
        origin = frame[:3, 3] * MM
        # 서보의 축이 관절 축 위에 있다 (원점이 축 위에 있고 방향이 나란하다)
        off = (origin - point) - ((origin - point) @ axis) * axis
        assert np.linalg.norm(off) < 1e-9, (k, off)
        # 설계 각은 출력 쪽에서 볼 때 시계 방향이 +다: 출력이 관절 축의 + 쪽을 보면 운동학 각과 부호가 반대다
        assert float(out @ axis) == pytest.approx(-pl.DESIGN_SIGN[k - 1], abs=1e-9), k


@pytest.mark.parametrize("pose_deg", POSES_DEG)
def test_flange_matches_the_rod_model(robot: Robot, pose_deg: tuple[float, ...]) -> None:
    # 플랜지 면(J6 출력 원판의 윗면)의 자리와 방향: 손목 오프셋(79 · 70 · 33)과 J6의 부호까지 묶인다
    q = np.radians(pose_deg)
    T = pl.frames_mm(q)["flange"]
    expected = flange_pose(robot, q)
    assert T[:3, :3] == pytest.approx(expected[:3, :3], abs=1e-9)
    assert T[:3, 3] * MM == pytest.approx(expected[:3, 3], abs=1e-9)


def test_range_middle_is_the_design_zero_of_every_disc() -> None:
    q = np.radians(pl.RANGE_MID_DEG)
    for k in range(1, pl.JOINT_COUNT + 1):
        assert pl.disc_deg(k, float(q[k - 1])) == pytest.approx(0.0)
    frames = pl.frames_mm(q)
    for k in range(1, pl.JOINT_COUNT + 1):
        assert frames[f"j{k}_out"] == pytest.approx(frames[f"j{k}"])
    # J4의 가운데(180°)에서 손목 1 몸통은 출력 원판에 반 바퀴 돌려 달린 자리다 (wrist.from_parent의 flip)
    T_j5_j4 = np.linalg.inv(frames["j5"]) @ frames["j4"]
    assert T_j5_j4 == pytest.approx(pl.matrix_of(wrist.from_parent(flip=True)), abs=1e-9)
    # 다른 관절(J5)은 돌리지 않은 자리다
    T_j6_j5 = np.linalg.inv(frames["j6"]) @ frames["j5"]
    assert T_j6_j5 == pytest.approx(pl.matrix_of(wrist.from_parent()), abs=1e-9)


def test_frames_follow_the_links() -> None:
    assert [pl.link_of(f) for f in ("j1", "j1_out", "j2", "j4_out", "j6", "j6_out", "flange")] == [
        0,
        1,
        1,
        4,
        5,
        6,
        6,
    ]
    assert set(pl.frames_mm(np.zeros(6))) == set(pl.FRAMES)
    with pytest.raises(ValueError):
        pl.frames_mm(np.zeros(5))


@pytest.mark.slow
def test_placed_base_and_shoulder_do_not_collide_while_the_joints_turn() -> None:
    # 부호가 틀리면 위팔이나 J1 몸통이 엉뚱한 쪽으로 돌아 서 있는 부품에 박힌다
    parts = [p for p in pl.arm_parts("base", "shoulder", "upper_arm") if not p.hardware]
    assert {p.link for p in parts} == {0, 1, 2, 3}
    placed = pl.place(parts, np.radians((70.0, -100.0, 60.0, 180.0, 0.0, 0.0)))
    assert pl.overlaps(placed) == {}
    # 위팔은 J2가 +로 돌 때 앞(+x)으로 숙인다 (운동학의 각: 어깨 축 +y에 대한 오른손 법칙)
    arm = next(p for p in parts if p.name == "위팔/위팔")
    ((_, tilted),) = pl.place([arm], np.radians((0.0, 60.0, 0.0, 180.0, 0.0, 0.0)))
    ((_, upright),) = pl.place([arm], np.zeros(6))
    assert tilted.center().X - upright.center().X > 50.0
    # J1이 +90° 돌면 앞(+x)에 있던 것이 왼쪽(+y)으로 간다: (x, y) → (-y, x)
    ((_, swung),) = pl.place([arm], np.radians((90.0, 60.0, 0.0, 180.0, 0.0, 0.0)))
    a, b = tilted.center(), swung.center()
    assert (b.X, b.Y, b.Z) == pytest.approx((-a.Y, a.X, a.Z), abs=1e-6)
