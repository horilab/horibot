"""처음 전원 순서(data/scripts/first-power/)의 자세가 지금 설계와 맞는지 본다 (docs/decisions/0034의 처음 전원 순서).

순서를 읽고 부딪힘을 거르고 시뮬레이션에서 끝까지 돌리는 것은 컨트롤러 시험
(crates/controller/tests/scripts.rs)이 본다. 여기서는 어깨 버티기의 자세가 설계의 IK 해(도구가 아래,
작업 갈래 앞위+, 로봇 앞 오른쪽 대각선)와 같은지 본다: 링크 길이나 도구가 바뀌면 자세를 다시 내야 한다.
"""

import math
import tomllib
from pathlib import Path

import numpy as np

from horibot_design.ik import AnalyticIK
from horibot_design.studies import packaging as pk
from horibot_design.studies import ur_arm as ua

ROOT = Path(__file__).resolve().parents[1]
HOLD = ROOT / "data" / "scripts" / "first-power" / "1-shoulder-hold.toml"
RADII_M = (0.16, 0.20, 0.24, 0.26, 0.28, 0.30)
AZIMUTH = math.radians(-45.0)  # 책상이 가장 깊은 쪽
HEIGHT_M = 0.18  # 도구 끝 높이: 반지름 30 cm에서 2 cm와 부담이 거의 같아 쿠션을 비킨다 (0028)


def _front_up_plus(q: np.ndarray) -> bool:
    """작업 갈래 앞위+: J1이 앞쪽 반, 팔꿈치가 위(J3 > 0), J5 > 0."""
    front = abs((q[0] + math.pi) % (2 * math.pi) - math.pi) < math.pi / 2
    return front and q[2] > 0 and q[4] > 0


def test_shoulder_hold_poses_are_the_design_ik_solutions() -> None:
    steps = tomllib.loads(HOLD.read_text(encoding="utf-8"))["steps"]
    moves = [s["move"] for s in steps if isinstance(s.get("move"), dict) and len(s["move"]) == 6]
    assert len(moves) == len(RADII_M)
    robot = ua.robot_of(ua.Concept())
    ik = AnalyticIK(robot)
    for r, move in zip(RADII_M, moves, strict=True):
        script = np.radians([move[f"j{k}"] for k in range(1, 7)])
        target = (r * math.cos(AZIMUTH), r * math.sin(AZIMUTH), HEIGHT_M)
        pose = pk.tool_pose(target, 0.0, 0.0, 0.0, ua.GRIPPER.length)
        fitted = [ua.fit_ranges(q, robot.lower, robot.upper) for q in ik.solve(pose)]
        found = [q for q in fitted if q is not None and _front_up_plus(q)]
        assert found, f"반지름 {r} m: 앞위+ 해가 없다"
        best = min(found, key=lambda q: float(np.abs(q - script).max()))
        off = float(np.degrees(np.abs(best - script)).max())
        assert off < 0.01, (
            f"반지름 {r} m: 설계의 해 {np.degrees(best).round(2).tolist()}와 {off:.3f}° 다르다"
        )
