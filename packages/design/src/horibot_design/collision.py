"""부딪힘 검사의 형상 (docs/decisions/0041-collision-checking.md): 설계 부품을 볼록 조각으로 나눠 data/collision/에 쓴다.

    uv run python -m horibot_design.collision build [--jobs N] [--timeout S]   # 바뀐 부품만 다시 나눈다
    uv run python -m horibot_design.collision check                            # 자료가 지금 설계와 같은지

컨트롤러는 이 자료만 읽어 부딪힘을 잰다 (crates/controller/src/collision.rs). 자료는 엔진과 상관없는 꼴이다:
- parts.json: 틀마다 붙은 링크와 링크 좌표계에서 본 자리(T_link_frame, mm), 부품마다 틀, 묶음, 나사·너트인지,
  도구, 그리퍼 손가락이 열림 폭을 따라 움직이는 양(slide), 조각 파일의 열쇠.
- pieces/<열쇠>.json: 부품 하나의 볼록 조각들. 조각마다 꼭짓점(부품이 그려진 틀의 좌표, mm)이다. 엔진은 이
  점들의 볼록 껍질을 쓴다.

부품: 팔의 모든 묶음(placement.arm_parts)과 책상 클램프 둘. 책상 판은 안전 평면으로 보므로 넣지 않는다 (0041).
그리퍼의 부품은 도구 "gripper"이고, 손가락처럼 열림 폭을 따라 곧게 움직이는 부품은 닫은 자리의 조각과 열림 폭
1 mm마다 옮겨 가는 양을 둔다.

나누기 (0041의 결정 1):
- 부품을 0.2 mm 현 오차로 메시로 만든다.
- 나사·너트와, 부피가 볼록 껍질의 98% 이상인 부품은 볼록 껍질 하나로 둔다.
- 나머지는 떨어진 덩어리마다 CoACD(문턱 0.05)로 나눈다. CoACD는 덩어리가 섞인 입력에서 끝나지 않았다
  (2026-11-27, 플러그). 거의 볼록한 덩어리는 볼록 껍질로 둔다.
- 부품마다 시간 제한을 두고, 넘으면 덩어리마다 볼록 껍질 하나로 둔다 (how에 남긴다).
- 조각이 메시의 겉을 0.05 mm 안에서 감싸는지 본다. 못 감싸면 덩어리마다 볼록 껍질로 둔다.

부품을 알아보는 열쇠는 형상의 B-rep 성질(부피, 겉넓이, 무게 중심, 관성, 상자, 면·모서리 수)과 메시·나누기
설정의 해시다. 메시는 같은 형상도 앞서 다른 설정으로 메시를 만든 적이 있으면 달라질 수 있어서 열쇠로 쓰지 않는다.
열쇠가 같은 조각 파일이 이미 있으면 다시 나누지 않는다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import sys
import tempfile
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import numpy as np

from horibot_design.robot import Mat

if TYPE_CHECKING:
    import trimesh
    from build123d import Compound

ROOT = Path(__file__).resolve().parents[4]
DATA = ROOT / "data" / "collision"
MANIFEST = DATA / "parts.json"
PIECES = DATA / "pieces"

MESH_TOLERANCE_MM = 0.2  # 현 오차
MESH_ANGLE_RAD = 0.3
COACD_THRESHOLD = 0.05  # CoACD의 오목함 문턱 (0041에서 견준 값)
CONVEX_ENOUGH = 0.98  # 부피가 볼록 껍질의 이만큼 이상이면 볼록 껍질 하나로 둔다
COVERAGE_LIMIT_MM = 0.05  # 조각 밖으로 나간 겉의 점이 이보다 멀면 못 감싼 것이다
DEFAULT_TIMEOUT_S = (
    2400.0  # 부품 하나를 나누는 시간 제한 (2026-11-27에 가장 오래 걸린 부품이 약 15분)
)
SMALL_TIMEOUT_S = 60.0  # 삼각형이 적은 부품의 시간 제한
SMALL_TRIANGLES = 200
SAMPLES = 10_000  # 감쌈을 볼 때 겉에 뿌리는 점 수 (메시 꼭짓점에 더한다)
DESK_BOARD = "책상/책상"  # 책상 판: 안전 평면으로 본다
GRIPPER_PREFIX = "그리퍼/"
FRAME_DRIFT_MM = (
    0.02  # 틀의 링크 좌표계 자리가 자세마다 이 안에서 같아야 한다 (값 파일은 0.01 mm까지 적는다)
)
ABOUT = (
    "부딪힘 검사의 형상 (docs/decisions/0041-collision-checking.md). "
    "uv run python -m horibot_design.collision build이 만든다. 손으로 고치지 않는다."
)


@dataclass(frozen=True)
class Item:
    """자료에 넣을 부품 하나. 형상은 그 틀의 좌표(mm)다."""

    name: str
    frame: str
    group: str
    hardware: bool
    tool: str  # "" (팔과 베이스), "gripper"
    slide: tuple[float, float, float] | None  # 열림 폭 1 mm마다 옮겨 가는 양 (틀 좌표, mm)
    shape: Compound


def items() -> list[Item]:
    """자료에 넣을 부품: 팔의 모든 묶음과 책상 클램프 (책상 판은 뺀다)."""
    from horibot_design import placement as pl
    from horibot_design.parts import gripper

    parts = [p for p in [*pl.arm_parts(), *pl.desk_parts()] if p.name != DESK_BOARD]
    # 그리퍼: 닫은 모습, 반 연 모습, 다 연 모습을 같은 순서로 견줘 곧게 옮겨 간 부품의 움직임을 얻는다
    # (placement는 핀을 뺀다)
    widths = (0.0, gripper.OPEN_MAX / 2, gripper.OPEN_MAX)
    shapes = [[p for p in gripper.gripper_parts(w) if "핀" not in p.name] for w in widths]
    closed = shapes[0]
    slides: list[tuple[float, float, float] | None] = []
    for c, h, o in zip(*shapes, strict=True):
        if not c.name == h.name == o.name:
            raise ValueError(f"그리퍼 부품의 순서가 다르다: {c.name} / {o.name}")
        slides.append(_slide(c.name, (c.shape, h.shape, o.shape), gripper.OPEN_MAX))
    tools = [p for p in parts if p.name.startswith(GRIPPER_PREFIX)]
    if [p.name for p in tools] != [GRIPPER_PREFIX + c.name for c in closed]:
        raise ValueError("배치의 그리퍼 부품이 gripper_parts(0)과 다르다")
    slide_of = {id(p): s for p, s in zip(tools, slides, strict=True)}
    return [
        Item(
            name=p.name,
            frame=p.frame,
            group=p.name.split("/", 1)[0],
            hardware=p.hardware,
            tool="gripper" if p.name.startswith(GRIPPER_PREFIX) else "",
            slide=slide_of.get(id(p)),
            shape=p.shape,
        )
        for p in parts
    ]


def _slide(
    name: str, shapes: tuple[Compound, Compound, Compound], width: float
) -> tuple[float, float, float] | None:
    """닫은 모습, 반 연 모습, 다 연 모습(width)에서 곧게 옮겨 갔으면 열림 폭 1 mm마다의 옮김 [mm], 아니면 None.

    피니언과 피니언 나사는 피니언 축 둘레로 돌아서 곧게 옮겨 가지 않는다. 랙 집 안에서 도는 부품이라 닫은 자리에
    둔다. 그 밖의 부품이 곧지 않게 움직이면 그리퍼 설계가 바뀐 것이라 멈춘다."""
    boxes = [s.bounding_box() for s in shapes]
    c = [np.array([b.center().X, b.center().Y, b.center().Z]) for b in boxes]
    shift = c[2] - c[0]
    if np.linalg.norm(shift) < 0.01:
        return None  # 자리가 그대로인 부품
    straight = np.linalg.norm(c[1] - (c[0] + c[2]) / 2) < 0.01
    sizes = [np.array([b.size.X, b.size.Y, b.size.Z]) for b in boxes]
    same = all(np.allclose(s, sizes[0], atol=1e-4) for s in sizes)
    if straight and same:
        v = shift / width
        return (float(v[0]), float(v[1]), float(v[2]))
    if "피니언" in name:
        return None
    raise ValueError(f"그리퍼 {name}: 열림 폭을 따라 곧게 옮겨 가지 않는다")


def frame_transforms() -> dict[str, tuple[int, Mat]]:
    """틀마다 (링크, T_link_틀 [mm]). 링크 좌표계는 로봇 기술(values.load_robot, 로봇 설정의 [kinematics]와 같다)의
    것이고, 0은 베이스다. 틀은 링크와 함께 움직이므로 자세와 상관없이 같아야 한다: 몇 자세에서 견줘 본다."""
    from horibot_design import placement as pl
    from horibot_design.robot import link_poses
    from horibot_design.values import load_robot

    robot = load_robot()

    def links_mm(q: np.ndarray) -> list[Mat]:
        out = [np.eye(4)]
        for T in link_poses(robot, q):
            T = T.copy()
            T[:3, 3] *= 1000.0
            out.append(T)
        return out

    poses = [
        np.zeros(6),
        *np.radians([(40, -55, 80, 150, -70, 25), (-130, 20, -95, 230, 110, -140)]),
    ]
    found: dict[str, tuple[int, Mat]] = {}
    for q in poses:
        L, F = links_mm(q), pl.frames_mm(q)
        for name in pl.FRAMES:
            k = pl.link_of(name)
            T = np.linalg.inv(L[k]) @ F[name]
            if name not in found:
                found[name] = (k, T)
                continue
            T0 = found[name][1]
            if not (
                np.allclose(T[:3, :3], T0[:3, :3], atol=1e-6)
                and np.allclose(T[:3, 3], T0[:3, 3], atol=FRAME_DRIFT_MM)
            ):
                raise ValueError(
                    f"틀 {name}이 링크 {k}와 같이 움직이지 않는다 (로봇 기술과 부품의 쌓임이 다르다)"
                )
    return found


def signature(shape: Compound) -> list[str]:
    """형상의 B-rep 성질: 부피, 겉넓이, 무게 중심, 관성, 상자, 면·모서리 수 (6자리로 반올림한 글자)."""
    # OCP의 형식 정보에 이 이름들이 없다 (실행에는 있다)
    from OCP.BRepGProp import BRepGProp  # pyright: ignore[reportAttributeAccessIssue]
    from OCP.GProp import GProp_GProps  # pyright: ignore[reportAttributeAccessIssue]

    volume, surface = GProp_GProps(), GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, volume)
    BRepGProp.SurfaceProperties_s(shape.wrapped, surface)
    c = volume.CentreOfMass()
    inertia = volume.MatrixOfInertia()
    box = shape.bounding_box()
    values = [
        volume.Mass(),
        surface.Mass(),
        c.X(),
        c.Y(),
        c.Z(),
        *(inertia.Value(r, s) for r in (1, 2, 3) for s in range(r, 4)),
        box.min.X,
        box.min.Y,
        box.min.Z,
        box.max.X,
        box.max.Y,
        box.max.Z,
    ]
    text = [_num(v) for v in values]
    return [*text, str(len(shape.faces())), str(len(shape.edges()))]


def _num(v: float) -> str:
    # 0 둘레의 잡음(-0.0, 1e-13)이 열쇠를 바꾸지 않게 0.0001 mm보다 작은 값은 0으로 본다
    return "0" if abs(v) < 1e-4 else f"{v:.6g}"


def key_of(item: Item) -> str:
    """조각 파일의 열쇠: 형상과 나누기 설정의 해시."""
    text = json.dumps(
        {
            "mesh": [MESH_TOLERANCE_MM, MESH_ANGLE_RAD],
            "coacd": COACD_THRESHOLD,
            "convex": CONVEX_ENOUGH,
            "hardware": item.hardware,
            "shape": signature(item.shape),
        },
        ensure_ascii=False,
    )
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:16]


def mesh_of(shape: Compound) -> tuple[np.ndarray, np.ndarray]:
    """형상을 틀 좌표의 삼각형 메시로: (꼭짓점 [mm], 삼각형)."""
    vs, ts = shape.tessellate(MESH_TOLERANCE_MM, MESH_ANGLE_RAD)
    V = np.array([(v.X, v.Y, v.Z) for v in vs], dtype=np.float64)
    F = np.array(ts, dtype=np.int64).reshape(-1, 3)
    return V, F


# --- 나누기 (따로 띄운 프로세스에서 돈다: CoACD가 끝나지 않으면 시간 제한으로 끊는다) ------------------------


def decompose(
    V: np.ndarray, F: np.ndarray, hardware: bool, force_hull: bool
) -> tuple[list[np.ndarray], str]:
    """메시를 볼록 조각으로 나눈다: (조각마다 꼭짓점, 나눈 방법)."""
    import trimesh

    m = trimesh.Trimesh(V, F, process=True)
    m.merge_vertices()
    comps = m.split(only_watertight=False)
    if force_hull:
        return [_hull_points(c) for c in comps], "hull(시간 넘음)"
    hull = m.convex_hull
    if hardware:
        return [np.asarray(hull.vertices)], "hull(나사)"
    if m.is_watertight and hull.volume > 0 and m.volume / hull.volume >= CONVEX_ENOUGH:
        return [np.asarray(hull.vertices)], "hull(볼록)"
    import coacd

    coacd.set_log_level("error")
    pieces: list[np.ndarray] = []
    for c in comps:
        ch = c.convex_hull
        if c.is_watertight and ch.volume > 0 and c.volume / ch.volume >= CONVEX_ENOUGH:
            pieces.append(np.asarray(ch.vertices))
            continue
        res = coacd.run_coacd(
            coacd.Mesh(np.asarray(c.vertices), np.asarray(c.faces)),
            threshold=COACD_THRESHOLD,
            preprocess_mode="off" if c.is_watertight else "on",
            preprocess_resolution=100,
            max_convex_hull=-1,
            seed=1,
        )
        pieces += [
            np.asarray(trimesh.Trimesh(v, f, process=True).convex_hull.vertices) for v, f in res
        ]
    return pieces, f"coacd({len(comps)}덩어리)" if len(comps) > 1 else "coacd"


def _hull_points(m: trimesh.Trimesh) -> np.ndarray:
    return np.asarray(m.convex_hull.vertices)


def hull_meshes(pieces: list[np.ndarray]) -> list[trimesh.Trimesh]:
    """조각의 꼭짓점에서 볼록 껍질 메시 (엔진이 쓰는 것과 같은 꼴)."""
    import trimesh

    return [trimesh.Trimesh(vertices=p).convex_hull for p in pieces]


def coverage_mm(V: np.ndarray, F: np.ndarray, pieces: list[np.ndarray], seed: int = 1) -> float:
    """메시의 겉(꼭짓점과 고르게 뿌린 점)이 조각 밖으로 가장 멀리 나간 거리 [mm]. 0이면 다 감쌌다."""
    import trimesh

    m = trimesh.Trimesh(V, F, process=False)
    rng = np.random.default_rng(seed)
    pts = np.vstack([V, trimesh.sample.sample_surface(m, SAMPLES, seed=rng)[0]])
    hulls = hull_meshes(pieces)
    inside = np.zeros(len(pts), dtype=bool)
    for h in hulls:
        A = h.face_normals
        b = np.einsum("ij,ij->i", A, h.triangles[:, 0])
        inside |= np.all(pts @ A.T - b <= 1e-6, axis=1)
    out = pts[~inside]
    if len(out) == 0:
        return 0.0
    best = np.full(len(out), np.inf)
    for h in hulls:
        _, dist, _ = trimesh.proximity.closest_point(h, out)
        best = np.minimum(best, dist)
    return float(best.max())


def finish(
    V: np.ndarray, F: np.ndarray, pieces: list[np.ndarray], how: str, comps_hull: bool
) -> tuple[list[np.ndarray], str, float]:
    """조각을 0.001 mm로 반올림하고 감쌈을 본다. 못 감싸면 덩어리마다 볼록 껍질로 바꾼다: (조각, 방법, 감쌈)."""
    rounded = [np.round(p, 3) for p in pieces]
    cover = coverage_mm(V, F, rounded)
    if cover > COVERAGE_LIMIT_MM and not comps_hull:
        import trimesh

        m = trimesh.Trimesh(V, F, process=True)
        m.merge_vertices()
        rounded = [np.round(_hull_points(c), 3) for c in m.split(only_watertight=False)]
        how = f"hull(못 감쌈 {cover:.2f} mm)"
        cover = coverage_mm(V, F, rounded)
    return rounded, how, cover


def piece_text(name: str, key: str, pieces: list[np.ndarray], how: str, cover: float) -> str:
    """조각 파일의 글: 사람이 볼 부품 이름과 나눈 방법, 감쌈, 조각마다 꼭짓점 [x, y, z, x, y, z, …] (mm)."""
    head = {"name": name, "key": key, "how": how, "coverage_mm": round(cover, 4)}
    rows = ",\n".join(json.dumps([round(float(x), 3) for x in p.ravel()]) for p in pieces)
    return json.dumps(head, ensure_ascii=False)[:-1] + ',\n"pieces": [\n' + rows + "\n]}\n"


def _work(args: list[str]) -> int:
    """따로 띄운 프로세스: <메시 npz> <출력 json> [hull]. 나누고 감쌈을 보고 조각 파일을 쓴다."""
    src, out = Path(args[0]), Path(args[1])
    force_hull = len(args) > 2 and args[2] == "hull"
    z = np.load(src)
    V, F = z["V"], z["F"]
    name, key, hardware = str(z["name"]), str(z["key"]), bool(z["hardware"])
    t0 = time.perf_counter()
    pieces, how = decompose(V, F, hardware, force_hull)
    pieces, how, cover = finish(V, F, pieces, how, comps_hull=how.startswith("hull"))
    how = f"{how}, {time.perf_counter() - t0:.0f} s" if how.startswith("coacd") else how
    out.write_text(piece_text(name, key, pieces, how, cover), encoding="utf-8", newline="\n")
    return 0


# --- 만들기와 검사 --------------------------------------------------------------------------------------


def read_piece_head(key: str) -> dict[str, Any]:
    """조각 파일의 머리 (이름, 방법, 감쌈, 조각 수)."""
    data = json.loads((PIECES / f"{key}.json").read_text(encoding="utf-8"))
    return {"how": data["how"], "coverage_mm": data["coverage_mm"], "pieces": len(data["pieces"])}


def manifest_of(
    all_items: list[Item], keys: list[str], frames: dict[str, tuple[int, Mat]]
) -> dict[str, Any]:
    """parts.json의 내용."""
    used = sorted({it.frame for it in all_items}, key=lambda f: list(frames).index(f))
    parts = []
    for it, key in zip(all_items, keys, strict=True):
        head = read_piece_head(key)
        parts.append(
            {
                "name": it.name,
                "frame": it.frame,
                "group": it.group,
                "hardware": it.hardware,
                "tool": it.tool,
                "slide": None if it.slide is None else [round(v, 9) for v in it.slide],
                "key": key,
                "how": head["how"],
                "pieces": head["pieces"],
                "coverage_mm": head["coverage_mm"],
            }
        )
    return {
        "about": ABOUT,
        "units": "mm",
        "mesh_tolerance_mm": MESH_TOLERANCE_MM,
        "mesh_angle_rad": MESH_ANGLE_RAD,
        "coacd_threshold": COACD_THRESHOLD,
        "frames": {
            name: {"link": frames[name][0], "T_link_frame": _rounded(frames[name][1])}
            for name in used
        },
        "parts": parts,
    }


def _rounded(T: Mat) -> list[list[float]]:
    out = np.round(T, 9)
    out[:3, 3] = np.round(T[:3, 3], 6)
    return [[float(v) + 0.0 for v in row] for row in out]


def write_manifest(content: dict[str, Any]) -> None:
    lines = [
        json.dumps({k: v for k, v in content.items() if k != "parts"}, ensure_ascii=False)[:-1]
        + ',\n"parts": ['
    ]
    lines.append(",\n".join(json.dumps(p, ensure_ascii=False) for p in content["parts"]))
    lines.append("]}\n")
    MANIFEST.write_text("\n".join(lines), encoding="utf-8", newline="\n")


def build(jobs: int, timeout: float) -> int:
    """바뀐 부품만 다시 나누고 parts.json을 쓴다. 쓰지 않는 조각 파일은 지운다."""
    t0 = time.perf_counter()
    all_items = items()
    frames = frame_transforms()
    keys = [key_of(it) for it in all_items]
    PIECES.mkdir(parents=True, exist_ok=True)
    todo: dict[str, Item] = {}
    for it, key in zip(all_items, keys, strict=True):
        if not (PIECES / f"{key}.json").exists():
            todo.setdefault(key, it)
    print(
        f"부품 {len(all_items)}개, 다시 나눌 것 {len(todo)}개 ({time.perf_counter() - t0:.0f} s)",
        flush=True,
    )
    failed = _run_workers(todo, jobs, timeout) if todo else []
    if failed:
        print("나누지 못한 부품:\n  " + "\n  ".join(failed))
        return 1
    write_manifest(manifest_of(all_items, keys, frames))
    wanted = set(keys)
    stale = [p for p in PIECES.glob("*.json") if p.stem not in wanted]
    for p in stale:
        p.unlink()
    heads = [read_piece_head(k) for k in dict.fromkeys(keys)]
    worst = max(h["coverage_mm"] for h in heads)
    print(
        f"{MANIFEST.relative_to(ROOT)}: 부품 {len(all_items)}, 조각 {sum(h['pieces'] for h in heads)}, "
        f"감쌈 최대 {worst:.3f} mm, 지운 조각 파일 {len(stale)}, {time.perf_counter() - t0:.0f} s"
    )
    return 0


def _run_workers(todo: dict[str, Item], jobs: int, timeout: float) -> list[str]:
    """부품마다 프로세스를 띄워 나눈다 (큰 것부터). 실패한 부품의 이름과 까닭을 낸다."""
    env = os.environ | {"OMP_NUM_THREADS": "2"}
    failed: list[str] = []
    with tempfile.TemporaryDirectory() as tmp:
        sizes: dict[str, int] = {}
        for key, it in todo.items():
            V, F = mesh_of(it.shape)
            np.savez(
                Path(tmp) / f"{key}.npz", V=V, F=F, name=it.name, key=key, hardware=it.hardware
            )
            sizes[key] = len(F)
        order = sorted(todo, key=lambda k: -sizes[k])
        t0 = time.perf_counter()

        def run(key: str) -> tuple[str, str]:
            cmd = [
                sys.executable,
                "-m",
                "horibot_design.collision",
                "piece",
                str(Path(tmp) / f"{key}.npz"),
                str(PIECES / f"{key}.json"),
            ]
            limit = timeout if sizes[key] > SMALL_TRIANGLES else SMALL_TIMEOUT_S
            try:
                subprocess.run(cmd, env=env, timeout=limit, check=True, capture_output=True)
                return key, "ok"
            except subprocess.TimeoutExpired:
                subprocess.run([*cmd, "hull"], env=env, check=True, capture_output=True)
                return key, "시간 넘음"
            except subprocess.CalledProcessError as e:
                return key, "오류: " + e.stderr.decode(errors="replace")[-400:]

        with ThreadPoolExecutor(jobs) as pool:
            for done, fut in enumerate(as_completed([pool.submit(run, k) for k in order]), start=1):
                key, status = fut.result()
                name = todo[key].name
                if status.startswith("오류"):
                    failed.append(f"{name}: {status}")
                    continue
                head = read_piece_head(key)
                print(
                    f"[{done}/{len(order)} {time.perf_counter() - t0:5.0f} s] {head['how']:24s} 조각 {head['pieces']:3d} "
                    f"감쌈 {head['coverage_mm']:.3f} mm  {name}",
                    flush=True,
                )
    return failed


def check(groups: list[str] | None = None) -> list[str]:
    """자료가 지금 설계와 같은지 본다: 부품과 순서, 틀의 자리, 열쇠(형상), 조각 파일, 감쌈. 문제를 모아 낸다.

    groups를 주면 그 묶음(부품 이름의 앞부분, 예: "어깨")의 부품만 열쇠를 다시 계산한다 (시험을 나눠 돌릴 때)."""
    content = json.loads(MANIFEST.read_text(encoding="utf-8"))
    problems: list[str] = []
    all_items = items()
    names = [p["name"] for p in content["parts"]]
    if names != [it.name for it in all_items]:
        problems.append("부품 목록이나 순서가 다르다 (build로 다시 만든다)")
        return problems
    frames = frame_transforms()
    for name, f in content["frames"].items():
        k, T = frames[name]
        if f["link"] != k or not np.allclose(np.array(f["T_link_frame"]), T, atol=1e-5):
            problems.append(f"틀 {name}의 자리가 다르다")
    for it, p in zip(all_items, content["parts"], strict=True):
        if groups is not None and it.group not in groups:
            continue
        want = {"frame": it.frame, "group": it.group, "hardware": it.hardware, "tool": it.tool}
        have = {k: p[k] for k in want}
        if have != want:
            problems.append(f"{it.name}: {have} ≠ {want}")
        slide = None if it.slide is None else list(it.slide)
        if (slide is None) != (p["slide"] is None) or (
            slide is not None and not np.allclose(slide, p["slide"], atol=1e-6)
        ):
            problems.append(f"{it.name}: 그리퍼 움직임이 다르다")
        key = key_of(it)
        if key != p["key"]:
            problems.append(f"{it.name}: 형상이 바뀌었다 (build로 다시 나눈다)")
        elif not (PIECES / f"{key}.json").exists():
            problems.append(f"{it.name}: 조각 파일이 없다 ({key})")
        elif p["coverage_mm"] > COVERAGE_LIMIT_MM:
            problems.append(f"{it.name}: 조각이 겉을 {p['coverage_mm']} mm 못 감싼다")
    return problems


def main(argv: list[str] | None = None) -> int:
    args = list(sys.argv[1:] if argv is None else argv)
    if args[:1] == ["piece"]:
        return _work(args[1:])
    parser = argparse.ArgumentParser(
        prog="python -m horibot_design.collision", description="부딪힘 검사의 형상 (0041)"
    )
    sub = parser.add_subparsers(dest="command", required=True)
    b = sub.add_parser("build", help="바뀐 부품만 다시 나눠 data/collision/에 쓴다")
    b.add_argument(
        "--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2), help="동시에 나눌 부품 수"
    )
    b.add_argument(
        "--timeout", type=float, default=DEFAULT_TIMEOUT_S, help="부품 하나의 시간 제한 [s]"
    )
    sub.add_parser("check", help="자료가 지금 설계와 같은지 본다")
    ns = parser.parse_args(args)
    if ns.command == "build":
        return build(ns.jobs, ns.timeout)
    problems = check()
    print("\n".join(problems) if problems else "자료가 지금 설계와 같다")
    return 1 if problems else 0


if __name__ == "__main__":
    raise SystemExit(main())
