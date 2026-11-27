"""시뮬레이션 화면 (0033): 보기 모델에 세계의 관절 각을 옮겨 MuJoCo 창에 그린다.

보기 모델(build/sim/horibot_view.xml)은 설계가 쓴다 (`uv run python -m horibot_design.mjcf_view`): 시뮬레이션 모델과
바디·관절이 같고 부품 메시가 붙었다. 서버는 물리를 시뮬레이션 모델로 돌리고, 박자마다(초당 60번까지) 이름이 같은
관절의 각을 보기 모델로 넘긴다. 그래서 컨트롤러가 붙을 때마다 세계를 새로 만들어도 창은 그대로이고, 무거운 메시가
물리 계산과 시험에 끼지 않는다.
- 창에 넘기는 일(MuJoCo 창의 sync)은 따로 된 스레드가 한다: sync는 창이 한 장을 그리는 동안(약 16 ms) 기다려서,
  서버 스레드에서 부르면 박자가 주기(10 ms)를 넘었다 (2026-11-27 측정, 0033). 서버는 각과 글만 넘기고 바로 돌아간다.
- 손가락 관절(finger…)은 그리퍼 관절에 비례해 움직인다: 두 관절 범위의 끝끼리 맞는다.
- 창 왼쪽 위에 시뮬레이션 시간과 서보마다 관절 각, 부하(멈춤 토크의 %)를 쓴다. 창의 글꼴은 영문만 된다.
- 컨트롤러가 붙기 전에는 보기 모델의 키프레임 rest(쉬는 자세)로 보인다.
- 창을 닫아도 서버는 계속 돈다.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco

from horibot_sim.world import World

if TYPE_CHECKING:
    from mujoco.viewer import Handle

VIEW_MODEL = Path("build/sim/horibot_view.xml")
FINGER_PREFIX = "finger"
GRIPPER = "gripper"
FRAME_TIME = 1 / 60  # [s] 창에 넘기는 가장 짧은 간격
# 처음 카메라: 로봇 오른쪽 앞에서 팔과 책상이 다 보이게 (마우스로 돌리고 옮긴다)
LOOKAT = (0.03, 0.0, 0.2)
DISTANCE = 1.0
AZIMUTH = 115.0
ELEVATION = -15.0


def _qpos_of(model: mujoco.MjModel, name: str) -> int | None:
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return None if joint < 0 else int(model.jnt_qposadr[joint])


@dataclass(frozen=True)
class Snapshot:
    """창에 넘길 한 장: 보기 모델의 qpos 자리마다 값, 왼쪽 위 글 (이름 열, 값 열)."""

    angles: tuple[tuple[int, float], ...]
    names: str
    values: str


class View:
    """보기 모델과 그 창. `show`를 부르기 전에는 창 없이 각만 옮긴다 (시험)."""

    def __init__(self, path: Path = VIEW_MODEL) -> None:
        if not path.exists():
            raise FileNotFoundError(
                f"보기 모델이 없다: {path}. uv run python -m horibot_design.mjcf_view로 만든다"
            )
        self.model = mujoco.MjModel.from_xml_path(str(path))
        self.data = mujoco.MjData(self.model)
        rest = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "rest")
        if rest >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, rest)
        mujoco.mj_kinematics(self.model, self.data)
        self.handle: Handle | None = None
        self.taken = -math.inf  # 마지막으로 넘긴 때 (서버 스레드)
        self._world_model: mujoco.MjModel | None = None
        self._pairs: list[tuple[int, int]] = []  # (세계의 qpos 자리, 보기 모델의 qpos 자리)
        # 손가락: (보기 모델의 qpos 자리, 그리퍼 각 1 rad마다의 미닫이 [m])
        self._fingers: list[tuple[int, float]] = []
        self._gripper = _qpos_of(self.model, GRIPPER)
        if self._gripper is not None:
            top = float(self.model.jnt_range[self.model.joint(GRIPPER).id][1])
            for joint in range(self.model.njnt):
                name = self.model.joint(joint).name
                if name.startswith(FINGER_PREFIX) and top > 0:
                    ratio = float(self.model.jnt_range[joint][1]) / top
                    self._fingers.append((int(self.model.jnt_qposadr[joint]), ratio))
        self._pending: Snapshot | None = None
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._drawer: threading.Thread | None = None

    def show(self) -> None:
        """MuJoCo 창을 띄우고 그리는 스레드를 돌린다 (설정 판은 닫아 둔다. 창에서 Tab, Shift+Tab으로 연다)."""
        import mujoco.viewer

        handle = mujoco.viewer.launch_passive(
            self.model, self.data, show_left_ui=False, show_right_ui=False
        )
        with handle.lock():
            handle.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            handle.cam.lookat[:] = LOOKAT
            handle.cam.distance = DISTANCE
            handle.cam.azimuth = AZIMUTH
            handle.cam.elevation = ELEVATION
        self.handle = handle
        self._drawer = threading.Thread(target=self._draw, name="view", daemon=True)
        self._drawer.start()

    def snapshot(self, world: World) -> Snapshot:
        """세계에서 창에 넘길 것을 뽑는다: 이름이 같은 관절의 각과 왼쪽 위 글."""
        if self._world_model is not world.model:
            self._world_model = world.model
            self._pairs = []
            for joint in range(world.model.njnt):
                view = _qpos_of(self.model, world.model.joint(joint).name)
                if view is not None:
                    self._pairs.append((int(world.model.jnt_qposadr[joint]), view))
        angles = tuple((view, float(world.data.qpos[source])) for source, view in self._pairs)
        return Snapshot(angles, *status_text(world))

    def apply(self, snapshot: Snapshot) -> None:
        """한 장을 보기 모델에 옮긴다: 관절 각, 손가락은 그리퍼에 비례."""
        for target, angle in snapshot.angles:
            self.data.qpos[target] = angle
        if self._gripper is not None:
            opened = float(self.data.qpos[self._gripper])
            for target, ratio in self._fingers:
                self.data.qpos[target] = opened * ratio
        mujoco.mj_kinematics(self.model, self.data)

    def update(self, world: World, force: bool = False) -> None:
        """창이 떠 있으면 세계의 자세와 글을 그리는 스레드에 넘긴다 (force가 아니면 FRAME_TIME보다 자주 넘기지
        않는다). 서버 스레드에서 부르고, 기다리지 않는다."""
        handle = self.handle
        if handle is None or not handle.is_running():
            return
        now = time.monotonic()
        if not force and now - self.taken < FRAME_TIME:
            return
        self.taken = now
        snapshot = self.snapshot(world)
        with self._lock:
            self._pending = snapshot
        self._wake.set()

    def _draw(self) -> None:
        """그리는 스레드: 넘겨받은 마지막 한 장을 보기 모델에 옮기고 창에 넘긴다 (창이 닫히면 끝난다)."""
        while (handle := self.handle) is not None and handle.is_running():
            if not self._wake.wait(0.5):
                continue
            self._wake.clear()
            with self._lock:
                snapshot, self._pending = self._pending, None
            if snapshot is None:
                continue
            self.apply(snapshot)
            handle.set_texts((None, None, snapshot.names, snapshot.values))
            handle.sync()

    def close(self) -> None:
        handle, self.handle = self.handle, None
        if handle is not None:
            handle.close()
        self._wake.set()
        if self._drawer is not None:
            self._drawer.join(timeout=2)


def status_text(world: World) -> tuple[str, str]:
    """창 왼쪽 위에 쓸 글: (이름 열, 값 열). 서보마다 관절 각과 부하(멈춤 토크의 %), 그리퍼는 열린 정도다."""
    names = ["time"]
    values = [f"{world.time:.2f} s"]
    for servo in sorted(world.servos.values(), key=lambda s: s.id):
        load = abs(servo.regs.get("present_load", 0)) / 10
        angle = world.angle(servo.id)
        if servo.joint == GRIPPER:
            top = float(world.model.jnt_range[world.model.joint(GRIPPER).id][1])
            # 닫힌 채 조금 더 밀려 음수로 읽힐 때가 있다
            opened = max(0.0, 100 * angle / top) if top > 0 else 0.0
            shown = f"open {opened:.0f} %" if top > 0 else f"{math.degrees(angle):.1f} deg"
        else:
            shown = f"{math.degrees(angle):.1f} deg"
        names.append(servo.joint)
        values.append(f"{shown}   load {load:.0f} %")
    return "\n".join(names), "\n".join(values)
