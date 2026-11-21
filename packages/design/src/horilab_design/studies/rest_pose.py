"""쉬는 자세와 베이스 둘레의 검토 (0037, 베이스·어깨 묶음 2).

- 늘어진 손목이 서는 J4 각: 팔 전체 모델(studies/ur_arm.py)의 무게로, 손목 관절에 토크가 없을 때 J4에 걸리는 중력
  토크가 0이 되는 각을 찾는다 (평행 그리퍼를 달았을 때와 도구가 없을 때). 한계(340°) 안쪽 10°가 기준이다.
- 선반이 받는 힘: 어깨와 팔꿈치에 토크가 없을 때 J4의 집을 받치는 힘 (크기와 기운 각).
- 쉬는 자세의 틈: 실제 부품(placement)을 쉬는 자세로 놓고, 팔(위팔부터 그리퍼까지)과 베이스 쪽 것들(선반, 파이와
  덮개와 플러그, 클램프, 책상, 베이스 몸통) 사이를 본다. 손목이 늘어지는 각의 어림이 틀릴 몫으로 J4를 ±10°, 토크 없는
  J6가 돌 몫으로 J6를 한 바퀴 돌려 본다. 기준은 10 mm다 (선반의 받침은 집을 받치므로 뺀다).
- 작업 자세의 틈: 박스 쌓기와 옮기는 길의 자세(작업 갈래)에서 팔과 선반·파이 사이. 기준은 2 mm다 (0017).
- 첫 시험의 추 (0028): 위팔까지만 조립하고 J1을 +90°로 돌려, 팔꿈치 집에 건 끈과 물병이 책상 뒤 가장자리와
  벽 사이로 늘어지는지.

    uv run python -m horilab_design.studies.rest_pose [work]
"""

from __future__ import annotations

import math
import sys
from collections.abc import Sequence

import numpy as np
from build123d import Compound

from horilab_design import placement as pl
from horilab_design.parts import base as bp
from horilab_design.parts import joint as j
from horilab_design.parts import links as lp
from horilab_design.parts import rest_shelf as rs
from horilab_design.parts.shapes import box, cyl_y, far_apart
from horilab_design.robot import Robot
from horilab_design.statics import PointMass, frame_poses, holding_torques
from horilab_design.studies import cameras as cm
from horilab_design.studies import ur_arm as ua

J4_LIMIT = 340.0  # J4의 프로그램 한계 [°]
J4_MARGIN = 10.0  # 늘어진 손목이 한계 안쪽으로 남겨야 하는 각
REST_GAP = 10.0  # 쉬는 자세에서 팔과 베이스 쪽 것들 사이 [mm]
WORK_GAP = 2.0  # 작업 자세에서 선반·파이와 팔 사이 [mm] (0017의 움직이는 틈)
ARM = (
    "위팔",
    "아래팔",
    "손목 1",
    "손목 2",
    "D405",
    "그리퍼",
)  # 어깨 축 너머의 묶음 (placement의 이름)
FIXED = ("선반", "파이", "책상", "베이스")
WALL_GAP = (
    j._MEASURED.desk.back_gap_mm if j._MEASURED.desk else 151.41
)  # 책상 뒤 가장자리 ~ 벽 (잰 값)
# 첫 시험의 추 (가정: 실물은 조립할 때 정한다): 팔꿈치 집의 책상 밖으로 나온 곳에 끈을 걸고, 지름 100 mm 물병이
# 상판 밑면보다 아래에 매달린다
BOTTLE_R = 50.0
STRING_R = 1.5
SWING_DEG = 5.0  # 끈이 흔들리는 몫 (가정)


def arm_masses(tool: bool = True) -> tuple[Robot, list[PointMass]]:
    """팔 전체 모델의 점 질량 (짐은 빼고). tool이 거짓이면 플랜지에 단 도구를 뺀다."""
    robot, prims = ua.build(ua.Concept(), ua.GRIPPER)
    out = [
        PointMass(p.name, min(p.link, 6), tuple(p.pos), p.mass)
        for p in prims
        if p.mass > 0 and (tool or p.group != "도구")
    ]
    return robot, out


def hang_j4(tool: bool = True, q: Sequence[float] = rs.REST_DEG) -> list[float]:
    """손목에 토크가 없을 때 J4가 서는 각 [°] (범위 20° ~ 340° 안의 안정한 평형)."""
    robot, masses = arm_masses(tool)

    def torque(a: float) -> float:
        qq = np.radians([q[0], q[1], q[2], a, q[4], q[5]])
        return float(holding_torques(robot, qq, masses)[3])

    out: list[float] = []
    step = 0.5
    grid = np.arange(20.0, J4_LIMIT + 1e-9, step)
    values = [torque(float(a)) for a in grid]
    for a, t0, t1 in zip(grid[:-1], values[:-1], values[1:], strict=True):
        # 버팀 토크가 -에서 +로 바뀌는 곳: 각이 커지면 중력이 되돌리는 쪽으로 미는 안정한 평형이다
        if t0 < 0.0 <= t1:
            out.append(float(a + step * (-t0) / (t1 - t0)))
    return out


def stable_sign_ok() -> bool:
    """평형의 부호 확인: 찾은 각에서 손목 2(J6 축)가 J4 축보다 아래에 있다 (매달린 쪽이다)."""
    robot, _ = arm_masses()
    (a,) = hang_j4() or [float("nan")]
    q = np.radians([0.0, rs.REST_J2, rs.REST_J3, a, rs.REST_J5, rs.REST_J6])
    frames = frame_poses(robot, q)
    return bool(frames[6][2, 3] < frames[4][2, 3])


def shelf_reaction() -> dict[str, float]:
    """쉬는 자세에서 선반이 J4의 집을 받치는 힘 (어깨와 팔꿈치에 토크가 없을 때, 팔 평면에서).

    관절 2와 3의 버팀 토크(중력)를 J4 축에 건 힘 하나가 대신 낸다고 놓고 푼다. 기운 각은 수직에서 몸통 쪽이 +다.
    """
    robot, masses = arm_masses()
    q = np.radians([0.0, *rs.REST_DEG[1:]])
    tau = holding_torques(robot, q, masses)
    frames = frame_poses(robot, q)
    p4 = frames[4][:3, 3]
    rows = []
    for k in (1, 2):  # 관절 2, 3
        origin = frames[k + 1][:3, 3]
        axis = frames[k + 1][:3, :3] @ np.asarray(robot.joints[k].axis)
        r = p4 - origin
        # 힘 (fx, 0, fz)가 관절에 내는 토크: axis · (r × f)
        rows.append(
            [float(axis @ np.cross(r, (1.0, 0.0, 0.0))), float(axis @ np.cross(r, (0.0, 0.0, 1.0)))]
        )
    fx, fz = np.linalg.solve(np.array(rows), tau[1:3])
    return {
        "위로 받치는 힘 [N]": float(fz),
        "앞으로 미는 힘 [N] (+면 집이 몸통 쪽으로 기댄다)": float(fx),
        "받치는 힘이 수직에서 기운 각 [°]": math.degrees(math.atan2(fx, fz)),
        "뿌리 판 밑에서 본 모멘트 [N·m]": float(fz) * (rs.j4_axis()[0] - rs.WALL_X) * 1e-3,
    }


def _group(name: str) -> str:
    return name.split("/")[0]


def scene() -> list[pl.ArmPart]:
    """팔의 모든 묶음과 책상·클램프 (나사는 뺀다)."""
    return [p for p in (*pl.arm_parts(), *pl.desk_parts()) if not p.hardware]


def group_gaps(
    parts: Sequence[pl.ArmPart], q_deg: Sequence[float], limit: float, fixed: Sequence[str] = FIXED
) -> dict[tuple[str, str], tuple[float, str, str]]:
    """자세 q에서 팔의 묶음과 베이스 쪽 묶음 사이의 가장 좁은 틈 [mm]: (팔 묶음, 베이스 쪽 묶음) → (틈, 부품, 부품).

    limit보다 넓은 쌍은 뺀다. 베이스 묶음에서는 서 있는 것(링크 0)만 본다.
    """
    placed = pl.place(parts, np.radians(q_deg))
    arm = [(p, s) for p, s in placed if _group(p.name) in ARM and p.link >= 2]
    rest = [
        (p, s)
        for p, s in placed
        if _group(p.name) in fixed and p.link <= 1 and (_group(p.name) != "베이스" or p.link == 0)
    ]
    out: dict[tuple[str, str], tuple[float, str, str]] = {}
    for a, sa in arm:
        for f, sf in rest:
            if far_apart(sa, sf, limit):
                continue
            d = sa.distance_to(sf)
            key = (_group(a.name), _group(f.name))
            if d < limit and d < out.get(key, (limit, "", ""))[0]:
                out[key] = (d, a.name, f.name)
    return out


def rest_gaps(
    parts: Sequence[pl.ArmPart] | None = None,
) -> dict[tuple[str, str], tuple[float, str, str]]:
    """쉬는 자세의 틈: J4를 늘어지는 각 둘레로 ±10°, J6를 여러 각으로 놓은 것 가운데 가장 좁은 값."""
    parts = scene() if parts is None else parts
    hang = hang_j4(True) + hang_j4(False)
    j4s = sorted({rs.REST_J4, *hang, min(hang) - 10.0, max(hang) + 10.0})
    out: dict[tuple[str, str], tuple[float, str, str]] = {}
    for a in j4s:
        for j6 in (0.0, 45.0, 90.0, 135.0):
            q = (rs.REST_J1, rs.REST_J2, rs.REST_J3, a, rs.REST_J5, j6)
            for key, hit in group_gaps(parts, q, 30.0).items():
                if hit[0] < out.get(key, (1e9, "", ""))[0]:
                    out[key] = hit
    return out


def lowest_rest_point(parts: Sequence[pl.ArmPart] | None = None) -> tuple[float, str]:
    """쉬는 자세(늘어진 손목, J6 한 바퀴)에서 팔의 가장 낮은 곳의 높이 [mm]와 그 부품."""
    parts = scene() if parts is None else parts
    arm = [p for p in parts if _group(p.name) in ARM]
    best = (1e9, "")
    for a in hang_j4(True):
        for j6 in (0.0, 45.0, 90.0, 135.0):
            q = (rs.REST_J1, rs.REST_J2, rs.REST_J3, a, rs.REST_J5, j6)
            for p, s in pl.place(arm, np.radians(q)):
                z = s.bounding_box().min.Z
                if z < best[0]:
                    best = (z, p.name)
    return best


def rest_footprint(parts: Sequence[pl.ArmPart] | None = None) -> dict[str, tuple[float, ...]]:
    """쉬는 자세에서 팔의 묶음마다 차지하는 자리 (로봇 좌표: x0, x1, y0, y1, z0, z1)."""
    parts = scene() if parts is None else parts
    arm = [p for p in parts if _group(p.name) in ARM]
    out: dict[str, list[float]] = {}
    for p, s in pl.place(arm, np.radians(rs.REST_DEG)):
        bb = s.bounding_box()
        got = out.setdefault(_group(p.name), [1e9, -1e9, 1e9, -1e9, 1e9, -1e9])
        for k, v in enumerate((bb.min.X, bb.max.X, bb.min.Y, bb.max.Y, bb.min.Z, bb.max.Z)):
            got[k] = min(got[k], v) if k % 2 == 0 else max(got[k], v)
    return {k: tuple(v) for k, v in out.items()}


def work_gaps() -> dict[str, tuple[float, str, str, str]]:
    """박스 쌓기와 옮기는 길의 자세(작업 갈래의 해)에서, 팔과 선반·파이 사이의 가장 좁은 틈: 묶음 → (틈, 자세, 부품, 부품)."""
    h = ua.hori(ua.Concept(), ua.GRIPPER)
    parts = [p for p in pl.arm_parts() if not p.hardware]
    poses = cm.placed_stacking_poses(ua.GRIPPER) + cm.transit_poses(ua.GRIPPER)
    out: dict[str, tuple[float, str, str, str]] = {}
    count = 0
    for pose in poses:
        sols = [q for q in ua.good_solutions(h, pose.T) if ua._branch(q) == "앞위+"]
        if not sols:
            continue
        count += 1
        q = [float(v) for v in np.degrees(sols[0])]
        for (_arm, fixed), (d, a, f) in group_gaps(parts, q, 60.0, ("선반", "파이")).items():
            if d < out.get(fixed, (1e9, "", "", ""))[0]:
                out[fixed] = (d, pose.label, a, f)
    out["본 자세 수"] = (float(count), f"{len(poses)}개 가운데", "", "")
    return out


def first_test_weight() -> dict[str, float]:
    """첫 시험의 추: J1 +90°, J2 +90°(위팔 수평)에서 팔꿈치 집에 건 끈과 물병의 틈 [mm] (가정한 물병).

    끈은 팔꿈치 집의 책상 밖으로 나온 곳의 가운데에 건다. 물병은 상판 밑면보다 아래에 매달려, 책상 가장자리와 벽
    사이에서 흔들린다.
    """
    parts = [
        p
        for p in (
            *pl.arm_parts("base", "shoulder", "upper_arm", "pi", "wire_clamps"),
            *pl.desk_parts(),
        )
        if not p.hardware
    ]
    q = (90.0, 90.0, 0.0, 180.0, 0.0, 0.0)
    placed = pl.place(parts, np.radians(q))
    arm = next(s for p, s in placed if p.name == "위팔/위팔")
    bb = arm.bounding_box()
    edge = -bp.EDGE
    out_len = edge - bb.min.X  # 팔꿈치 집이 책상 뒤 가장자리 밖으로 나온 길이
    x_s = (bb.min.X + edge) / 2  # 끈을 거는 자리
    frames = pl.frames_mm(np.radians(q))
    y_s, z_axis = float(frames["j3"][1, 3]), float(frames["j3"][2, 3])
    z_bottom = z_axis - j.R_DRUM
    desk_under = -bp.DESK_T

    def rbox(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Compound:
        return box(x0, x1, y0, y1, z0, z1)

    string = rbox(
        x_s - STRING_R, x_s + STRING_R, y_s - STRING_R, y_s + STRING_R, desk_under - 60.0, z_bottom
    )
    fixed = [(p.name, s) for p, s in placed if p.link == 0]
    # 책상은 뺀다 (끈과 책상 가장자리 사이는 위에서 따로 낸다)
    gaps = sorted((string.distance_to(s), n) for n, s in fixed if n != "책상/책상")
    swing = math.tan(math.radians(SWING_DEG)) * (z_bottom - desk_under)
    bottle_top = desk_under - 20.0
    return {
        "팔꿈치 집이 책상 뒤 가장자리 밖으로 나온 길이": out_len,
        "끈과 책상 가장자리 사이": edge - (x_s + STRING_R),
        "끈이 흔들릴 때 책상 가장자리 쪽으로 가는 길이 (상판 밑면 높이에서)": swing,
        "끈과 가장 가까운 베이스 쪽 것 (파이, 클램프, 판) 사이": gaps[0][0],
        "끈을 거는 자리의 옆 (로봇 y)": y_s,
        "물병이 벽에 닿지 않는 여유 (끈 밑에 곧게 매달릴 때)": WALL_GAP - (edge - x_s) - BOTTLE_R,
        "물병의 윗면 높이 (상판 밑면보다 20 mm 아래로 잡았다)": bottle_top,
        "끈 길이 (팔꿈치 집 밑에서 물병 윗면까지)": z_bottom - bottle_top,
        "팔꿈치 집 밑과 파이 덮개 위 사이 (위팔이 파이 위를 지난다)": min(
            arm.distance_to(s) for n, s in fixed if n.startswith("파이")
        ),
    }


def main() -> None:
    print(f"쉬는 자세 (운동학 J1 ~ J6 [°]): {rs.REST_DEG}")
    for label, tool in (("평행 그리퍼", True), ("도구 없이", False)):
        angles = hang_j4(tool)
        text = (
            ", ".join(f"{a:.1f}° (한계까지 {J4_LIMIT - a:.1f}°)" for a in angles)
            or "범위 안에 없다"
        )
        print(f"- 늘어진 손목이 서는 J4 ({label}): {text}")
    print(f"  매달린 쪽의 평형인가: {stable_sign_ok()}")
    for name, value in shelf_reaction().items():
        print(f"- {name}: {value:.2f}")
    parts = scene()
    print(
        f"- 쉬는 자세의 틈 [mm] (J4 ±10°, J6 한 바퀴 가운데 가장 좁은 값. 기준 {REST_GAP:.0f} mm, 30 mm 아래만)"
    )
    for (arm, fixed), (d, a, f) in sorted(rest_gaps(parts).items(), key=lambda kv: kv[1][0]):
        print(f"    {arm} ↔ {fixed}: {d:.1f}  ({a} ↔ {f})")
    z, name = lowest_rest_point(parts)
    print(f"- 쉬는 자세에서 팔의 가장 낮은 곳: 책상에서 {z:.1f} mm ({name})")
    for group, (x0, x1, y0, y1, z0, z1) in rest_footprint(parts).items():
        print(f"    {group}: x {x0:.0f} ~ {x1:.0f}, y {y0:.0f} ~ {y1:.0f}, z {z0:.0f} ~ {z1:.0f}")
    print("- 첫 시험의 추 (J1 +90°, J2 +90°)")
    for name, value in first_test_weight().items():
        print(f"    {name}: {value:.1f}")
    if "work" in sys.argv[1:]:
        print(f"- 작업 자세(박스 쌓기와 옮기는 길)의 틈 [mm] (기준 {WORK_GAP:.0f} mm)")
        for fixed, (d, pose, a, f) in work_gaps().items():
            print(f"    {fixed}: {d:.1f}  {pose}  ({a} ↔ {f})")
    _ = (cyl_y, lp)


if __name__ == "__main__":
    main()
