"""부딪힘 검사의 숫자를 다시 만든다 (docs/decisions/0041-collision-checking.md의 ② 볼록 조각).

    uv run python -m horibot_design.studies.collision poses
    cargo run --release -p horibot-controller --example collision_check -- build/collision/poses.json build/collision/pairs.csv
    uv run python -m horibot_design.studies.collision compare [build/collision/pairs.csv]

- poses: 견줄 자세 2,783개를 build/collision/poses.json에 쓴다. 쉬는 자세, 홈, 곧게 선 자세, 홈과 곧게 선 자세에서
  관절마다 소프트 한계 안을 5°씩 훑은 자세, 무작위 2,000개 (씨앗 20261127)다. 로봇 설정(data/robot.toml)의 값을 쓴다.
- 컨트롤러 예제(collision_check)가 자세마다 두 질문의 시간을 재고, 창(20 mm) 안의 짝과 조각 거리를 CSV로 쓴다.
  파이에서 잰 시간이 0041의 표다.
- compare: 같은 짝을 설계 형상의 메시(0.2 mm 현 오차)로 다시 잰다 (python-fcl의 BVH 거리, 개발용 의존성). 메시
  거리는 B-rep 거리와 0.115 mm 안에서 같다 (0041). 조각이 메시보다 멀게 잰 일(안전)과, 여유마다 조각으로만 막힌
  자세(헛경보)를 센다. 그리퍼 손가락은 컨트롤러가 닫은 자리부터 다 연 자리까지 쓸고 간 모양으로 두고, 메시는 닫은
  자리다. 평면은 메시 꼭짓점까지의 거리다.
"""

from __future__ import annotations

import csv
import json
import sys
import time
import tomllib
from multiprocessing import Pool
from pathlib import Path
from typing import Any

import numpy as np

from horibot_design import collision as col
from horibot_design.robot import link_poses
from horibot_design.values import load_robot

OUT = col.ROOT / "build" / "collision"
ROBOT_TOML = col.ROOT / "data" / "robot.toml"
SEED = 20261127
RANDOM = 2000
SWEEP_DEG = 5.0
MARGINS_MM = (1.0, 2.0, 3.0, 5.0)


def poses() -> list[dict[str, Any]]:
    """견줄 자세 (0041과 같은 묶음)."""
    robot = tomllib.loads(ROBOT_TOML.read_text(encoding="utf-8"))
    limits = [tuple(robot["joints"][f"j{k}"]["soft_limits_deg"]) for k in range(1, 7)]
    rest, home = robot["poses"]["rest_deg"], robot["poses"]["home_deg"]
    upright = [0.0, 0.0, 0.0, 180.0, 0.0, 0.0]
    out = [
        {"tag": "rest", "q_deg": list(map(float, rest))},
        {"tag": "home", "q_deg": list(map(float, home))},
        {"tag": "upright", "q_deg": upright},
    ]
    for name, base in (("home", home), ("upright", upright)):
        for k, (lo, hi) in enumerate(limits):
            for a in np.arange(lo, hi + 0.1, SWEEP_DEG):
                q = [float(v) for v in base]
                q[k] = float(a)
                out.append({"tag": f"sweep-{name}-j{k + 1}", "q_deg": q})
    rng = np.random.default_rng(SEED)
    for _ in range(RANDOM):
        out.append({"tag": "random", "q_deg": [float(rng.uniform(lo, hi)) for lo, hi in limits]})
    return out


_G: dict[str, Any] = {}


def _init() -> None:
    """작업 프로세스: 부품마다 링크 좌표의 메시(FCL BVH)를 만든다."""
    import fcl

    content = json.loads(col.MANIFEST.read_text(encoding="utf-8"))
    items = col.items()
    if [it.name for it in items] != [p["name"] for p in content["parts"]]:
        raise SystemExit("형상 자료가 지금 설계와 다르다 (collision build)")
    meshes, frames, links, points = [], content["frames"], [], []
    for it in items:
        V, F = col.mesh_of(it.shape)
        T = np.array(frames[it.frame]["T_link_frame"])
        Vl = V @ T[:3, :3].T + T[:3, 3]
        m = fcl.BVHModel()
        m.beginModel(len(Vl), len(F))
        m.addSubModel(Vl, F)
        m.endModel()
        meshes.append(fcl.CollisionObject(m, fcl.Transform()))
        links.append(frames[it.frame]["link"])
        points.append(Vl)
    robot = tomllib.loads(ROBOT_TOML.read_text(encoding="utf-8"))
    planes = [
        (np.array(p["normal"], float), float(p["offset_mm"])) for p in robot["collision"]["planes"]
    ]
    _G.update(fcl=fcl, meshes=meshes, links=links, points=points, planes=planes, robot=load_robot())


def _links_mm(q_deg: list[float]) -> list[np.ndarray]:
    out = [np.eye(4)]
    for T in link_poses(_G["robot"], np.radians(q_deg)):
        T = T.copy()
        T[:3, 3] *= 1000.0
        out.append(T)
    return out


def _mesh_rows(job: tuple[list[float], list[tuple[int, str, float]]]) -> list[tuple[float, float]]:
    """자세 하나의 짝마다 (조각 거리, 메시 거리)."""
    fcl = _G["fcl"]
    q_deg, rows = job
    L = _links_mm(q_deg)
    out = []
    for a, b, convex in rows:
        Ta = L[_G["links"][a]]
        if b.startswith("plane"):
            n, d = _G["planes"][int(b[5:])]
            P = _G["points"][a] @ Ta[:3, :3].T + Ta[:3, 3]
            mesh = max(float((P @ n).min() - d), 0.0)
        else:
            j = int(b)
            Tb = L[_G["links"][j]]
            ma, mb = _G["meshes"][a], _G["meshes"][j]
            ma.setTransform(fcl.Transform(Ta[:3, :3].copy(), Ta[:3, 3].copy()))
            mb.setTransform(fcl.Transform(Tb[:3, :3].copy(), Tb[:3, 3].copy()))
            # 결과는 부를 때마다 새로 만든다 (다시 쓰면 앞의 가장 작은 값이 남는다)
            mesh = max(
                float(fcl.distance(ma, mb, fcl.DistanceRequest(), fcl.DistanceResult())), 0.0
            )
        out.append((convex, mesh))
    return out


def compare(pairs_csv: Path) -> None:
    """조각 거리(컨트롤러 예제)와 메시 거리를 견준다: 안전과 여유마다의 헛경보."""
    t0 = time.perf_counter()
    all_poses = json.loads((OUT / "poses.json").read_text(encoding="utf-8"))
    by_pose: dict[int, list[tuple[int, str, float]]] = {}
    with pairs_csv.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):
            by_pose.setdefault(int(row["pose"]), []).append(
                (int(row["a"]), row["b"], float(row["distance_mm"]))
            )
    jobs = [(all_poses[i]["q_deg"], rows) for i, rows in sorted(by_pose.items())]
    with Pool(initializer=_init) as pool:
        results = pool.map(_mesh_rows, jobs, chunksize=8)
    worse = max(c - m for res in results for c, m in res)
    print(
        f"자세 {len(all_poses)}, 짝-자세 {sum(map(len, results))}, {time.perf_counter() - t0:.0f} s"
    )
    print(f"안전: 조각이 메시보다 멀게 잰 것 최대 {worse:+.4f} mm")
    named = [p["tag"] != "random" for p in all_poses]
    for m in MARGINS_MM:
        mesh_hit, convex_hit = set(), set()
        for (i, _), res in zip(sorted(by_pose.items()), results, strict=True):
            if any(mm < m for _, mm in res):
                mesh_hit.add(i)
            if any(c < m for c, _ in res):
                convex_hit.add(i)
        false = convex_hit - mesh_hit
        free_named = sum(1 for i, n in enumerate(named) if n and i not in mesh_hit)
        free_random = sum(1 for i, n in enumerate(named) if not n and i not in mesh_hit)
        fn = sum(1 for i in false if named[i])
        fr = len(false) - fn
        print(
            f"여유 {m:.0f} mm: 메시로 막힘 {len(mesh_hit)}, 조각으로 막힘 {len(convex_hit)}, 헛경보 "
            f"이름·훑기 {fn}/{free_named}, 무작위 {fr}/{free_random} ({fr / max(free_random, 1):.1%})"
        )


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args[:1] == ["poses"]:
        OUT.mkdir(parents=True, exist_ok=True)
        found = poses()
        (OUT / "poses.json").write_text(json.dumps(found), encoding="utf-8")
        print(f"{OUT / 'poses.json'}: 자세 {len(found)}")
        return 0
    if args[:1] == ["compare"]:
        compare(Path(args[1]) if len(args) > 1 else OUT / "pairs.csv")
        return 0
    print(__doc__)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
