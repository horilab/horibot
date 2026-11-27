"""MuJoCo 창에 그릴 보기 모델 (docs/decisions/0033의 시뮬레이션 화면): 설계 부품의 메시와 부딪힘 검사의 조각.

    uv run python -m horibot_design.mjcf_view [--out build/sim/horibot_view.xml]

시뮬레이션 서버를 `--viewer`로 띄우면 이 파일을 읽어 창에 그린다. 물리는 시뮬레이션 모델(mjcf.py, 막대에 실물
무게)이 하고, 보기 모델은 관절 각만 받아 그린다. 그래서 바디와 관절은 시뮬레이션 모델과 같다 (`build_spec`).
- 무리 0 (기본으로 보인다): 설계 부품을 부딪힘 검사와 같은 0.2 mm 메시로 만든 것. 부품과 자리는 부딪힘 검사의 자료와
  같고(collision.items), 색은 3D 페이지와 같은 구실별 색이다 (선만 실물처럼 어둡게).
- 무리 3 (창에서 숫자 3으로 켜고 끈다): 부딪힘 검사의 볼록 조각 (data/collision/). 컨트롤러의 검사가 보는 모양이다.
- 무리 4: 시뮬레이션 모델의 막대.
- 책상 판은 설계 형상으로, 책상 면이 아닌 안전 평면(벽)은 반투명 판으로 그린다 (data/robot.toml의 [collision]).
- 그리퍼에서 열림 폭을 따라 곧게 움직이는 부품(손가락과 그 받침)은 손가락 바디에 붙인다. 손가락 바디의 미닫이 관절
  (이름 finger…)은 시뮬레이션 서버가 그리퍼 관절에 비례해 움직인다 (범위의 끝끼리 맞는다).
- 키프레임 rest: 로봇 설정의 쉬는 자세. 컨트롤러가 붙기 전에 이 자세로 보인다.

부품 메시는 build/sim/meshes/<열쇠>.stl(mm)에 두고, 형상이 같으면 다시 만들지 않는다. 열쇠는 형상의 B-rep 성질
(collision.signature)과 메시 설정의 해시다. 부품을 모두 불러오므로 고친 부품이 없어도 30초쯤 걸린다.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import tomllib
from collections.abc import Sequence
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING

import mujoco
import numpy as np

from horibot_design.mjcf import add_gripper, build_spec
from horibot_design.robot import Mat, Robot
from horibot_design.values import REPO_ROOT, load_robot

if TYPE_CHECKING:
    from build123d import Compound

VIEW_MODEL = (
    REPO_ROOT / "build" / "sim" / "horibot_view.xml"
)  # 시뮬레이션 서버가 --viewer로 읽는 모델
ROBOT_SETTINGS = REPO_ROOT / "data" / "robot.toml"
MESH_DIR = "meshes"  # 보기 모델 파일 옆의 메시 폴더

PART_GROUP = 0
PIECE_GROUP = 3
STICK_GROUP = 4
FINGER_PREFIX = "finger"  # 손가락 바디와 그 미닫이 관절의 이름 앞부분

# 구실별 색: 3D 페이지(pages/joint_page.html)와 같다. 선은 페이지가 빨갛게 보이지만 여기서는 실물처럼 어둡게 둔다
COLORS = {
    "body": 0xF1F1EE,  # 흰색 PLA
    "cap": 0x2B2E33,  # 검정 PLA
    "frame": 0x3A4250,  # 검정 PLA
    "metal": 0xB7BEC7,
    "servo": 0x30343A,
    "plug": 0xF3EFE4,
    "wire": 0x26282B,
    "desk": 0xD9D2C5,
}
PIECE_RGBA = (1.0, 0.45, 0.1, 0.35)
PLANE_RGBA = (0.35, 0.6, 1.0, 0.12)
PLANE_THICKNESS = 0.002  # [m] 안전 평면을 그리는 판의 두께
PLANE_HALF = (0.45, 0.3)  # [m] 판의 반 크기 (책상 면을 따라, 위로)


@dataclass(frozen=True)
class ViewPart:
    """그릴 부품 하나. 형상은 그 틀의 좌표(mm)다."""

    name: str
    frame: str
    color: str
    slide: tuple[float, float, float] | None  # 열림 폭 1 mm마다 옮겨 가는 양 (틀 좌표, mm)
    shape: Compound


@dataclass(frozen=True)
class PieceSet:
    """부딪힘 검사 자료의 부품 하나의 볼록 조각들 (틀 좌표, mm)."""

    name: str
    frame: str
    slide: tuple[float, float, float] | None
    key: str
    pieces: tuple[np.ndarray, ...]


@dataclass(frozen=True)
class Plane:
    """안전 평면: 팔은 normal · x ≥ offset 쪽에 있다 (베이스 좌표계)."""

    name: str
    normal: tuple[float, float, float]
    offset: float  # [m]


def view_parts() -> list[ViewPart]:
    """그릴 부품: 부딪힘 검사의 부품(팔의 모든 묶음과 책상 클램프)과 책상 판."""
    from horibot_design import collision
    from horibot_design import placement as pl

    placed = [*pl.arm_parts(), *pl.desk_parts()]
    color = {p.name: p.color for p in placed}
    parts = [ViewPart(i.name, i.frame, color[i.name], i.slide, i.shape) for i in collision.items()]
    desk = [p for p in placed if p.name == collision.DESK_BOARD]
    return [*parts, *(ViewPart(p.name, p.frame, "desk", None, p.shape) for p in desk)]


def collision_pieces() -> list[PieceSet]:
    """부딪힘 검사 자료(data/collision/)의 부품마다 볼록 조각."""
    from horibot_design import collision

    manifest = json.loads(collision.MANIFEST.read_text(encoding="utf-8"))
    out: list[PieceSet] = []
    cache: dict[str, tuple[np.ndarray, ...]] = {}
    for part in manifest["parts"]:
        key = part["key"]
        if key not in cache:
            data = json.loads((collision.PIECES / f"{key}.json").read_text(encoding="utf-8"))
            cache[key] = tuple(np.reshape(p, (-1, 3)) for p in data["pieces"])
        slide = part["slide"]
        out.append(
            PieceSet(
                part["name"],
                part["frame"],
                None if slide is None else tuple(slide),
                key,
                cache[key],
            )
        )
    return out


def settings_planes_and_rest(path: Path = ROBOT_SETTINGS) -> tuple[list[Plane], list[float]]:
    """로봇 설정의 안전 평면과 쉬는 자세 [°]."""
    settings = tomllib.loads(path.read_text(encoding="utf-8"))
    planes = [
        Plane(p["name"], tuple(float(v) for v in p["normal"]), float(p["offset_mm"]) / 1000)  # type: ignore[arg-type]
        for p in settings.get("collision", {}).get("planes", [])
    ]
    return planes, [float(a) for a in settings["poses"]["rest_deg"]]


def mesh_key(shape: Compound) -> str:
    """메시 파일의 열쇠: 형상의 B-rep 성질과 메시 설정의 해시."""
    from horibot_design import collision

    text = json.dumps(
        {
            "view": [collision.MESH_TOLERANCE_MM, collision.MESH_ANGLE_RAD],
            "shape": collision.signature(shape),
        }
    )
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:16]


def cached_mesh(shape: Compound, mesh_dir: Path) -> str:
    """부품 메시를 mesh_dir/<열쇠>.stl(mm)로 두고 파일 이름을 낸다. 같은 열쇠의 파일이 있으면 다시 만들지 않는다."""
    import trimesh

    from horibot_design import collision

    name = f"{mesh_key(shape)}.stl"
    path = mesh_dir / name
    if not path.exists():
        V, F = collision.mesh_of(shape)
        mesh_dir.mkdir(parents=True, exist_ok=True)
        trimesh.Trimesh(V, F, process=False).export(path)
    return name


def _token_inertial(body: mujoco.MjsBody) -> None:
    """물리를 돌리지 않는 보기 모델의 바디 무게 (컴파일에만 쓴다). 파일에 쓰이려면 무게 중심 자리(ipos)까지 준다.
    이미 무게를 단 바디(그리퍼 피니언)는 그대로 둔다."""
    if body.explicitinertial:
        return
    body.explicitinertial = True
    body.mass = 0.01
    body.ipos = [0.0, 0.0, 0.0]
    body.fullinertia = [1e-6, 1e-6, 1e-6, 0.0, 0.0, 0.0]


def _rgba(color: str) -> list[float]:
    value = COLORS[color]
    return [((value >> 16) & 255) / 255, ((value >> 8) & 255) / 255, (value & 255) / 255, 1.0]


def _quat(R: Mat) -> list[float]:
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, np.ascontiguousarray(R, dtype=float).flatten())
    return quat.tolist()


def _finger_axis(
    frames: dict[str, tuple[int, Mat]], frame: str, slide: Sequence[float]
) -> tuple[int, np.ndarray, float]:
    """손가락 부품의 (링크, 링크 좌표의 단위 축, 열림 폭 1 mm마다 옮겨 가는 길이 [mm])."""
    link, T = frames[frame]
    v = T[:3, :3] @ np.asarray(slide, dtype=float)
    size = float(np.linalg.norm(v))
    return link, v / size, size


def build_view_spec(
    robot: Robot,
    frames: dict[str, tuple[int, Mat]],
    parts: Sequence[ViewPart],
    pieces: Sequence[PieceSet],
    planes: Sequence[Plane],
    rest_deg: Sequence[float],
    mesh_dir: Path,
    pitch_radius: float,
    max_width: float,
) -> mujoco.MjSpec:
    """보기 모델 MjSpec. frames는 틀마다 (링크, T_link_틀 [mm]), pitch_radius와 max_width는 그리퍼 [m]."""
    spec = build_spec(robot)
    for geom in spec.geoms:
        geom.group = STICK_GROUP
    add_gripper(spec, robot, pitch_radius, max_width)
    # 물리는 돌리지 않는다: 바디마다 작은 무게를 적고, 메시는 무게를 보태지 않는다
    spec.compiler.inertiafromgeom = mujoco.mjtInertiaFromGeom.mjINERTIAFROMGEOM_FALSE
    for body in spec.bodies[1:]:
        _token_inertial(body)
    spec.compiler.meshdir = str(mesh_dir)

    # 손가락 바디: 링크와 축이 같은 부품끼리 한 바디
    fingers: dict[tuple[int, tuple[float, ...]], mujoco.MjsBody] = {}

    def body_of(frame: str, slide: Sequence[float] | None) -> mujoco.MjsBody:
        if slide is None:
            link = frames[frame][0]
            return spec.body("base" if link == 0 else f"link{link}")
        link, axis, per_mm = _finger_axis(frames, frame, slide)
        key = (link, tuple(np.round(axis, 6)))
        if key not in fingers:
            name = f"{FINGER_PREFIX}{len(fingers) + 1}"
            body = spec.body(f"link{link}").add_body(name=name)
            _token_inertial(body)
            body.add_joint(
                name=name,
                type=mujoco.mjtJoint.mjJNT_SLIDE,
                axis=axis.tolist(),
                range=[0.0, max_width * per_mm],
                limited=mujoco.mjtLimited.mjLIMITED_TRUE,
            )
            fingers[key] = body
        return fingers[key]

    def place(
        body: mujoco.MjsBody, frame: str, mesh: str, group: int, rgba: Sequence[float]
    ) -> None:
        T = frames[frame][1]
        body.add_geom(
            type=mujoco.mjtGeom.mjGEOM_MESH,
            meshname=mesh,
            pos=(T[:3, 3] / 1000).tolist(),
            quat=_quat(T[:3, :3]),
            group=group,
            rgba=list(rgba),
            contype=0,
            conaffinity=0,
        )

    meshes: set[str] = set()
    for part in parts:
        if part.color == "desk":
            # 책상 판은 상자다. 삼각형 열둘인 메시는 MuJoCo가 모서리 꼭짓점의 법선을 평균 내어 윗면이 어둡게 그려져서
            # 상자 도형으로 그린다
            box = part.shape.bounding_box()
            center = np.array([box.center().X, box.center().Y, box.center().Z])
            link, T = frames[part.frame]
            spec.body("base" if link == 0 else f"link{link}").add_geom(
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=[box.size.X / 2000, box.size.Y / 2000, box.size.Z / 2000],
                pos=((T[:3, :3] @ center + T[:3, 3]) / 1000).tolist(),
                quat=_quat(T[:3, :3]),
                group=PART_GROUP,
                rgba=_rgba(part.color),
                contype=0,
                conaffinity=0,
            )
            continue
        file = cached_mesh(part.shape, mesh_dir)
        name = f"part_{file.removesuffix('.stl')}"
        if name not in meshes:
            spec.add_mesh(
                name=name,
                file=file,
                scale=[1e-3] * 3,
                inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL,
            )
            meshes.add(name)
        place(body_of(part.frame, part.slide), part.frame, name, PART_GROUP, _rgba(part.color))
    for item in pieces:
        for i, vertices in enumerate(item.pieces):
            name = f"piece_{item.key}_{i}"
            if name not in meshes:
                # 꼭짓점만 주면 MuJoCo가 볼록 껍질을 만든다 (컨트롤러가 쓰는 모양과 같다)
                spec.add_mesh(
                    name=name,
                    uservert=np.asarray(vertices, dtype=float).flatten().tolist(),
                    scale=[1e-3] * 3,
                    inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL,
                )
                meshes.add(name)
            place(body_of(item.frame, item.slide), item.frame, name, PIECE_GROUP, PIECE_RGBA)

    for plane in planes:
        n = np.asarray(plane.normal, dtype=float)
        if abs(n[2]) > 0.99:
            continue  # 책상 면은 책상 판 형상으로 보인다
        y = np.cross([0.0, 0.0, 1.0], n)
        y /= np.linalg.norm(y)
        z = np.cross(n, y)
        center = n * (plane.offset - PLANE_THICKNESS / 2) + z * PLANE_HALF[1]
        spec.worldbody.add_geom(
            name=f"plane {plane.name}",
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[PLANE_THICKNESS / 2, *PLANE_HALF],
            pos=center.tolist(),
            quat=_quat(np.column_stack([n, y, z])),
            rgba=list(PLANE_RGBA),
            contype=0,
            conaffinity=0,
        )

    # 보기: 밝은 배경과 위에서 비추는 빛
    spec.add_texture(
        name="sky",
        type=mujoco.mjtTexture.mjTEXTURE_SKYBOX,
        builtin=mujoco.mjtBuiltin.mjBUILTIN_GRADIENT,
        rgb1=[0.93, 0.95, 0.97],
        rgb2=[0.72, 0.77, 0.82],
        width=512,
        height=3072,
    )
    spec.worldbody.add_light(
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        pos=[0.0, 0.0, 2.0],
        dir=[0.25, -0.35, -1.0],
        diffuse=[0.55] * 3,
        specular=[0.1] * 3,
        castshadow=False,
    )
    spec.visual.headlight.ambient = [0.3] * 3
    spec.visual.headlight.diffuse = [0.4] * 3
    # 화면 밖 그림(확인용 사진)의 최대 크기
    spec.visual.global_.offwidth = 1920
    spec.visual.global_.offheight = 1080

    # 키프레임 rest: 쉬는 자세 (그리퍼와 손가락은 닫힘)
    model = spec.compile()
    qpos = np.array(model.qpos0)
    for k, angle in enumerate(rest_deg, start=1):
        qpos[model.jnt_qposadr[model.joint(f"j{k}").id]] = np.radians(angle)
    spec.add_key(name="rest", qpos=qpos.tolist())
    return spec


def write(path: Path = VIEW_MODEL) -> Path:
    """설계 부품과 부딪힘 검사의 조각으로 보기 모델을 쓴다. 메시는 path 옆의 meshes/에 둔다."""
    from horibot_design import collision
    from horibot_design.parts import gripper

    mesh_dir = (path.parent / MESH_DIR).resolve()
    planes, rest = settings_planes_and_rest()
    spec = build_view_spec(
        load_robot(),
        collision.frame_transforms(),
        view_parts(),
        collision_pieces(),
        planes,
        rest,
        mesh_dir,
        gripper.RP * 1e-3,
        gripper.OPEN_MAX * 1e-3,
    )
    spec.compile()
    # 메시 폴더는 파일 옆을 가리키게 바꿔 쓴다 (다른 PC나 다른 폴더에서도 읽힌다)
    text = spec.to_xml().replace(f'meshdir="{mesh_dir.as_posix()}', f'meshdir="{MESH_DIR}', 1)
    text = text.replace(f'meshdir="{mesh_dir}', f'meshdir="{MESH_DIR}', 1)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    mujoco.MjModel.from_xml_path(str(path))  # 쓴 파일이 그대로 읽히는지
    return path


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="MuJoCo 창에 그릴 보기 모델을 쓴다 (0033)")
    parser.add_argument("--out", type=Path, default=VIEW_MODEL)
    args = parser.parse_args(argv)
    print(write(args.out))


if __name__ == "__main__":
    main()
