"""로봇 기술로 MuJoCo 모델(MJCF)을 만든다: 막대 모양에 실물 무게를 단 시뮬레이션 모델.

MuJoCo의 MjSpec으로 코드에서 모델을 만든다 (docs/decisions/0006-data-and-pipeline.md).
바디 이름은 base, link1 … linkN, 플랜지는 사이트 flange다 (0010).

모양은 관절 사이를 잇는 막대다 (보기와 자리 확인용, 부딪힘은 시뮬레이션이 끈다). 시뮬레이션 모델(`write`)은 링크마다
팔 검토 모델(studies/ur_arm.py)의 무게, 무게 중심, 관성을 단다 (0033): 뽑아서 잰 부품에 맞춘 무게 어림이고, 서보,
베어링, 나사, 선, D405와 평행 그리퍼(짐 없이)가 들었다. 막대의 부피로 셈하면 링크마다 실물보다 2 ~ 7배 가볍다.

시뮬레이션 서버(packages/sim)는 설계를 불러오지 않고 이 모듈이 쓴 MJCF 파일만 읽는다 (0007, 0033):
uv run python -m horibot_design.mjcf [--out build/sim/horibot.xml]
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import ArrayLike

from horibot_design.robot import Mat, Robot
from horibot_design.values import REPO_ROOT, load_robot

SIM_MODEL = REPO_ROOT / "build" / "sim" / "horibot.xml"  # 시뮬레이션 서버가 읽는 모델
STICK_RADIUS = 0.012  # [m] 막대 모델의 링크 굵기. 모양을 보려는 값이라 정밀할 필요는 없다


@dataclass(frozen=True)
class Inertial:
    """링크 하나의 무게. 좌표는 링크 좌표계 (영점 자세에서 베이스와 같은 방향, 원점은 관절 점) [kg, m]."""

    mass: float
    center: Mat  # 무게 중심 (3)
    inertia: Mat  # 무게 중심에 대한 관성 텐서 (3 × 3) [kg·m²]


def build_spec(
    robot: Robot,
    stick_radius: float = STICK_RADIUS,
    inertials: Sequence[Inertial] | None = None,
) -> mujoco.MjSpec:
    """막대 모델 MjSpec을 만든다. 링크마다 다음 관절(또는 플랜지)까지 막대를 하나 둔다.

    inertials(링크 1 ~ N)를 주면 링크의 무게로 쓰고 막대는 모양만 남는다. 없으면 막대의 부피에서 셈한다.
    """
    if inertials is not None and len(inertials) != len(robot.joints):
        raise ValueError(f"링크 무게는 {len(robot.joints)}개여야 한다: {len(inertials)}")
    spec = mujoco.MjSpec()
    spec.modelname = robot.name
    # MJCF 각도의 기본 단위는 도다. 로봇 기술은 라디안이라 바꿔 둔다.
    spec.compiler.degree = False

    body = spec.worldbody.add_body(name="base")
    _add_stick(body, robot.joints[0].origin, stick_radius)
    for i, joint in enumerate(robot.joints):
        body = body.add_body(name=f"link{i + 1}", pos=list(joint.origin))
        body.add_joint(
            name=joint.name,
            type=mujoco.mjtJoint.mjJNT_HINGE,
            axis=list(joint.axis),
            range=[joint.lower, joint.upper],
            limited=mujoco.mjtLimited.mjLIMITED_TRUE,
        )
        if inertials is not None:
            _set_inertial(body, inertials[i])
        is_last = i + 1 == len(robot.joints)
        end = robot.T_last_flange[:3, 3] if is_last else robot.joints[i + 1].origin
        _add_stick(body, end, stick_radius)

    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, robot.T_last_flange[:3, :3].flatten())
    body.add_site(name="flange", pos=list(robot.T_last_flange[:3, 3]), quat=list(quat))
    return spec


def _set_inertial(body: mujoco.MjsBody, inertial: Inertial) -> None:
    t = inertial.inertia
    body.explicitinertial = True
    body.mass = inertial.mass
    body.ipos = list(inertial.center)
    body.fullinertia = [t[0, 0], t[1, 1], t[2, 2], t[0, 1], t[0, 2], t[1, 2]]


def _add_stick(body: mujoco.MjsBody, end: ArrayLike, radius: float) -> None:
    """바디 원점에서 end까지 캡슐을 단다. 길이가 0이면(축이 한 점에서 만나면) 구를 단다."""
    end = np.asarray(end, dtype=float)
    if np.linalg.norm(end) < 1e-9:
        body.add_geom(type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[radius, 0.0, 0.0])
    else:
        body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_CAPSULE,
            fromto=[0.0, 0.0, 0.0, *end],
            size=[radius, 0.0, 0.0],
        )


def _own_inertia(shape: str, size: tuple[float, float, float], mass: float) -> Mat:
    """형상 하나의 무게 중심에 대한 관성 (형상 좌표계, 속이 찬 것으로 본다). 볼록 껍질은 점 무게로 본다."""
    if shape == "box":
        a, b, c = size  # 반 크기
        return mass / 3 * np.diag([b * b + c * c, a * a + c * c, a * a + b * b])
    if shape in ("cyl", "ring"):
        outer, half, inner = size  # 축은 z
        r2 = outer * outer + inner * inner
        side = mass * (3 * r2 + 4 * half * half) / 12
        return np.diag([side, side, mass * r2 / 2])
    return np.zeros((3, 3))


def arm_inertials() -> list[Inertial]:
    """팔 검토 모델(studies/ur_arm.py, 평행 그리퍼를 단 지금 설계)의 링크 1 ~ 6 무게.

    무게 중심은 형상마다의 무게를 그 자리에 둔 점 무게의 가운데라, 검토 모델의 버팀 토크(statics)와 같은 무게
    분포다. 관성은 형상마다 속이 찬 상자, 원기둥, 고리로 보고 더한다 (어림). 도구 몸체(링크 7)는 링크 6에 넣는다.
    """
    from horibot_design.studies import ur_arm  # 부품 형상을 불러와 느려서 쓸 때만 읽는다

    _, prims = ur_arm.build(ur_arm.Concept(), ur_arm.GRIPPER)
    out: list[Inertial] = []
    for link in range(1, 7):
        own = [p for p in prims if min(p.link, 6) == link and p.mass > 0]
        mass = sum(p.mass for p in own)
        center = sum((p.mass * p.pos for p in own), np.zeros(3)) / mass
        inertia = np.zeros((3, 3))
        for p in own:
            d = p.pos - center
            inertia += p.rot @ _own_inertia(p.shape, p.size, p.mass) @ p.rot.T
            inertia += p.mass * (float(d @ d) * np.eye(3) - np.outer(d, d))
        out.append(Inertial(mass, center, inertia))
    return out


def write(path: Path = SIM_MODEL) -> Path:
    """설계 값(params/arm.toml)의 막대 모델에 팔 검토 모델의 링크 무게를 달아 MJCF 파일로 쓴다."""
    spec = build_spec(load_robot(), inertials=arm_inertials())
    spec.compile()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(spec.to_xml(), encoding="utf-8")
    return path


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="시뮬레이션용 MJCF를 쓴다 (0033)")
    parser.add_argument("--out", type=Path, default=SIM_MODEL)
    args = parser.parse_args(argv)
    print(write(args.out))


if __name__ == "__main__":
    main()
