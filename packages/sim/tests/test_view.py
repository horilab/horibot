"""시뮬레이션 화면: 세계의 관절 각을 이름으로 보기 모델에 옮긴다 (창 없이 본다, 0033)."""

from __future__ import annotations

from pathlib import Path

import pytest

from horibot_sim.view import View, status_text
from horibot_sim.world import World

# 시험용 물리 모델: 관절 j2, j3과 그리퍼 피니언
PHYSICS = """<mujoco model="test">
  <compiler angle="radian"/>
  <worldbody>
    <body name="link1" pos="0 0 0.1">
      <joint name="j2" type="hinge" axis="0 1 0" range="-2 2" limited="true"/>
      <geom type="capsule" fromto="0 0 0 0 0 0.22" size="0.012"/>
      <body name="link2" pos="0 0 0.22">
        <joint name="j3" type="hinge" axis="0 1 0" range="-2 2" limited="true"/>
        <geom type="capsule" fromto="0 0 0 0 0 0.18" size="0.012"/>
        <body name="gripper" pos="0 0 0.18">
          <joint name="gripper" type="hinge" axis="0 0 1" range="0 2.4" limited="true" armature="0.002"/>
          <inertial pos="0 0 0" mass="0.005" diaginertia="1e-7 1e-7 1e-7"/>
        </body>
      </body>
    </body>
  </worldbody>
</mujoco>
"""
# 보기 모델: 같은 바디와 관절에 손가락 미닫이 관절(그리퍼 2.4 rad에 30 mm)과 키프레임 rest
VIEW = """<mujoco model="view">
  <compiler angle="radian"/>
  <worldbody>
    <body name="link1" pos="0 0 0.1">
      <joint name="j2" type="hinge" axis="0 1 0" range="-2 2" limited="true"/>
      <geom type="box" size="0.01 0.01 0.1"/>
      <body name="link2" pos="0 0 0.22">
        <joint name="j3" type="hinge" axis="0 1 0" range="-2 2" limited="true"/>
        <geom type="box" size="0.01 0.01 0.09"/>
        <body name="gripper" pos="0 0 0.18">
          <joint name="gripper" type="hinge" axis="0 0 1" range="0 2.4" limited="true"/>
          <inertial pos="0 0 0" mass="0.005" diaginertia="1e-7 1e-7 1e-7"/>
        </body>
        <body name="finger1">
          <joint name="finger1" type="slide" axis="1 0 0" range="0 0.03" limited="true"/>
          <geom type="box" size="0.005 0.005 0.005"/>
        </body>
      </body>
    </body>
  </worldbody>
  <keyframe>
    <key name="rest" qpos="0.5 -0.5 0 0"/>
  </keyframe>
</mujoco>
"""
SERVO_SPECS = "[STS3250]\nstall_torque_kgcm = 50\n[STS3215]\nstall_torque_kgcm = 19.5\n"


@pytest.fixture
def files(tmp_path: Path) -> tuple[Path, Path, Path]:
    physics, view, specs = tmp_path / "arm.xml", tmp_path / "view.xml", tmp_path / "servos.toml"
    physics.write_text(PHYSICS, encoding="utf-8")
    view.write_text(VIEW, encoding="utf-8")
    specs.write_text(SERVO_SPECS, encoding="utf-8")
    return physics, view, specs


def _world(physics: Path, specs: Path) -> World:
    servos = [(2, "j2", 2825, 1, 0), (3, "j3", 2825, 1, 0), (11, "gripper", 777, 1, 0)]
    return World(physics, specs, 0.01, servos, {"j2": 0.3, "j3": -0.2, "gripper": 1.2})


def test_view_starts_at_rest_and_follows_the_world_by_name(files: tuple[Path, Path, Path]) -> None:
    physics, path, specs = files
    view = View(path)
    j2, j3, gripper, finger = (
        int(view.model.joint(n).qposadr[0]) for n in ("j2", "j3", "gripper", "finger1")
    )
    # 컨트롤러가 붙기 전에는 쉬는 자세(키프레임 rest)다
    assert view.data.qpos[j2] == pytest.approx(0.5)
    assert view.data.qpos[j3] == pytest.approx(-0.5)
    world = _world(physics, specs)
    view.apply(view.snapshot(world))
    assert view.data.qpos[j2] == pytest.approx(0.3)
    assert view.data.qpos[j3] == pytest.approx(-0.2)
    assert view.data.qpos[gripper] == pytest.approx(1.2)
    # 손가락은 그리퍼에 비례한다: 반 열면 30 mm의 반
    assert view.data.qpos[finger] == pytest.approx(0.015)


def test_status_text_shows_angles_loads_and_the_opening(files: tuple[Path, Path, Path]) -> None:
    physics, _, specs = files
    names, values = status_text(_world(physics, specs))
    assert names.splitlines() == ["time", "j2", "j3", "gripper"]
    lines = values.splitlines()
    assert lines[0] == "0.00 s"
    assert lines[1].startswith("17.2 deg") and "load" in lines[1]
    assert lines[3].startswith("open 50 %")
    # 창의 글꼴은 영문만 된다
    assert (names + values).isascii()


def test_a_missing_view_model_says_how_to_make_it(tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError, match="mjcf_view"):
        View(tmp_path / "none.xml")


def test_update_without_a_window_does_nothing(files: tuple[Path, Path, Path]) -> None:
    physics, path, specs = files
    view = View(path)
    view.update(_world(physics, specs), force=True)
    assert view.data.qpos[int(view.model.joint("j2").qposadr[0])] == pytest.approx(0.5)
    view.close()
