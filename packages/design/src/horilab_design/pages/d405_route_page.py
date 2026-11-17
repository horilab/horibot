"""D405 선을 관절 밖으로 넘기는 모습: 팔 전체 3D 페이지 ("D405 선 길", 0031).

D405 선은 서보 선과 같은 틈에 넣지 않는다. 베이스 몸통, 위팔, 아래팔의 겉은 찍찍이 케이블 타이로 묶고, 플러그
바로 밑은 D405 받침의 클립으로 잡으며, 관절을 지나는 구간은 여유를 두고 공중에 띄운다. 생각을 맞춰 보려는
그림이라, 묶는 자리, 카메라 파이 자리, 공중 구간의 선 길이는 그림에 맞춰 고른 어림이다 (설계 값이 아니다).
실제 묶는 자리는 다 조립한 팔에서 묶어 보며 맞춘다.

틀 d405_route_page.html의 __DATA__ 자리에 팔(arm_page.page_data의 형상과 관절), 카메라 파이 스케치, 선을 잡는
자리(Clip: 타이와 받침 클립), 선이 지나는 자리, 작업 자세, 관절마다 돌릴 수 있는 범위를 넣는다. 선의 모양은
페이지가 푼다: 마디를 이은 끈에 무게, 굽힘을 펴는 힘, 팔 형상과 책상과의 부딪힘을 주고, 관절을 움직이면 잡는
자리가 링크와 같이 움직인다. 서보 선은 몸통 속과 두 몸통 사이 층으로 가서 그리지 않는다. 관절마다 돌릴 수 있는
범위는 작업 자세(박스 쌓기의 첫 자세)에서 그 관절만 돌려, 팔끼리나 책상, 카메라 파이에 닿기 전까지다.

    uv run python -m horilab_design.pages.d405_route_page          # build/pages/d405_route.html
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass

import numpy as np

from horilab_design.pages import HERE, OUT_DIR
from horilab_design.pages import arm_page as ap
from horilab_design.robot import Robot
from horilab_design.studies import packaging as pk
from horilab_design.studies import ur_arm
from horilab_design.studies import wiring as wr
from horilab_design.studies.packaging import MM

Vec = tuple[float, float, float]
DEG = math.pi / 180


@dataclass(frozen=True)
class Clip:
    """선을 잡는 자리 하나 (찍찍이 타이나 받침 클립). at은 선 가운데, along은 선이 가는 방향, out은 붙는 면에서
    선 쪽 (링크 좌표 mm)."""

    link: int
    at: Vec
    along: Vec
    out: Vec
    tag: str = ""


@dataclass(frozen=True)
class Run:
    """선이 한 링크에 붙어 곧게 가는 구간 (타이 사이, 플러그 끝)."""

    name: str
    link: int
    a: Vec
    b: Vec


@dataclass(frozen=True)
class Span:
    """두 Run 사이에서 선이 공중에 뜨는 구간. guide는 처음 모양이 지나는 점 (링크 번호, 자리 mm)이다."""

    key: str
    name: str
    joints: str
    guide: tuple[tuple[int, Vec], ...]


# ---- 카메라 파이 (스케치 어림): 왼쪽 날개를 받침으로 넓히고 기둥 18 mm 위에 판을 얹는다. 단자는 책상 뒤 쪽 ----
PI_BOARD = ((-60.0, 25.0), (50.0, 106.0), (28.0, 29.4))
PLATE = ((-60.0, 27.0), (30.0, 109.0), (0.0, 10.0))
PLUG_BOX = ((-84.0, -62.0), (59.5, 70.5), (32.5, 42.5))  # USB-A 플러그 몸통
PLUG_EXIT: Vec = (-84.0, 65.0, 37.5)  # 선이 플러그에서 나오는 곳 (-x로 나온다)

# ---- 잡는 자리 (링크 좌표 mm). 위팔과 아래팔은 관 양 끝 가까이에 하나씩, 관의 -x 면(작업 자세에서 팔꿈치 바깥)
BASE_AZ = 115.0 * DEG
BASE_R = 43.0  # 베이스 몸통(반지름 40.5) 겉에 묶은 선의 가운데
BASE_AT: Vec = (BASE_R * math.cos(BASE_AZ), BASE_R * math.sin(BASE_AZ), 55.0)
UPPER_X, FORE_X = -23.5, -21.5  # 관 -x 면(반지름 20, 18) 밖의 선 가운데
CAM_RUN: tuple[Vec, Vec] = (
    (29.9, -0.9, 42.0),
    (29.9, -0.9, 56.0),
)  # 받침 클립 → 플러그에서 선이 나오는 곳

CLIPS = (
    Clip(
        0, BASE_AT, (0.0, 0.0, 1.0), (math.cos(BASE_AZ), math.sin(BASE_AZ), 0.0), "① 베이스 (타이)"
    ),
    Clip(2, (UPPER_X, 55.5, 60.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), "② 위팔 (타이 2개)"),
    Clip(2, (UPPER_X, 55.5, 160.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)),
    Clip(3, (FORE_X, 23.7, 50.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), "③ 아래팔 (타이 2개)"),
    Clip(3, (FORE_X, 23.7, 123.0), (0.0, 0.0, 1.0), (-1.0, 0.0, 0.0)),
    Clip(5, (29.9, -0.9, 49.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0), "④ 받침 클립"),
)

# 선이 가는 차례: 카메라 파이에서 카메라 플러그까지. Run과 Span이 번갈아 온다
ROUTE: tuple[Run | Span, ...] = (
    Run("파이 플러그", 0, PLUG_EXIT, (-90.0, 65.0, 37.5)),
    Span(
        "S0",
        "파이 → ① 베이스",
        "책상 위",
        ((0, (-100.0, 60.0, 45.0)), (0, (-70.0, 48.0, 60.0)), (0, (-35.0, 42.0, 58.0))),
    ),
    Run("① 베이스", 0, (BASE_AT[0], BASE_AT[1], 50.0), (BASE_AT[0], BASE_AT[1], 60.0)),
    Span(
        "S1", "① 베이스 → ② 위팔", "J1, J2", ((1, (-36.0, 36.0, 110.0)), (1, (-36.0, 52.0, 165.0)))
    ),
    Run("② 위팔", 2, (UPPER_X, 55.5, 55.0), (UPPER_X, 55.5, 165.0)),
    Span(
        "S2",
        "② 위팔 → ③ 아래팔",
        "J3",
        ((2, (-32.0, 52.0, 210.0)), (2, (-10.0, 40.0, 252.0)), (2, (30.0, 28.0, 250.0))),
    ),
    Run("③ 아래팔", 3, (FORE_X, 23.7, 45.0), (FORE_X, 23.7, 128.0)),
    Span(
        "S3",
        "③ 아래팔 → ④ 받침",
        "J4, J5",
        ((4, (40.0, 45.0, 32.0)), (4, (38.0, 80.0, 34.0)), (4, (22.0, 104.0, 40.0))),
    ),
    Run("④ 받침", 5, CAM_RUN[0], CAM_RUN[1]),
)

# 공중 구간의 선 길이 [mm]: 그림을 그리려고 넉넉히 잡은 어림 (설계 값이 아니다)
SPAN_MM = {"S0": 130.0, "S1": 310.0, "S2": 190.0, "S3": 360.0}


def _box(
    name: str, x: tuple[float, float], y: tuple[float, float], z: tuple[float, float], role: str
) -> dict:
    return {
        "l": 0,
        "s": "box",
        "p": [
            round((x[0] + x[1]) / 2, 3),
            round((y[0] + y[1]) / 2, 3),
            round((z[0] + z[1]) / 2, 3),
        ],
        "r": [1, 0, 0, 0, 1, 0, 0, 0, 1],
        "z": [
            round((x[1] - x[0]) / 2, 3),
            round((y[1] - y[0]) / 2, 3),
            round((z[1] - z[0]) / 2, 3),
        ],
        "c": role,
        "n": name,
    }


Box = tuple[str, tuple[float, float], tuple[float, float], tuple[float, float], str]


def pi_boxes() -> list[Box]:
    """카메라 파이 스케치 (mm): 넓힌 날개(받침), 기둥 넷, 판, USB 둘과 랜 단자, D405 선의 USB-A 플러그."""
    (bx0, bx1), (by0, by1), (bz0, bz1) = PI_BOARD
    out: list[Box] = [
        ("pi_plate", *PLATE, "base"),
        ("pi_board", (bx0, bx1), (by0, by1), (bz0, bz1), "pi"),
    ]
    for i, hx in enumerate((bx0 + 3.5, bx0 + 61.5)):
        for k, hy in enumerate((by0 + 3.5, by0 + 52.5)):
            post = ((hx - 2.5, hx + 2.5), (hy - 2.5, hy + 2.5), (PLATE[2][1], bz0))
            out.append((f"pi_post{2 * i + k}", *post, "port"))
    for name, y0, y1, h in (
        ("pi_usb2", 57.0, 72.0, 16.0),
        ("pi_usb3", 74.5, 89.5, 16.0),
        ("pi_lan", 91.0, 106.0, 13.5),
    ):
        out.append((name, (bx0 - 2.0, bx0 + 17.0), (y0, y1), (bz1, bz1 + h), "port"))
    out.append(("d405_usb_plug", *PLUG_BOX, "plug"))
    return out


def pi_fixture() -> list[pk.Prim]:
    """부딪힘 검사에 넣을 카메라 파이 (책상에 고정된 물건). 팔이 파이 자리를 쓸고 지나가는 자세를 뺀다."""
    b = pk.Builder()
    for name, x, y, z, _ in pi_boxes():
        lo, hi = (x[0] * MM, y[0] * MM, z[0] * MM), (x[1] * MM, y[1] * MM, z[1] * MM)
        b.box(0, f"fixture/{name}", lo, hi, mass=0.0, group="카메라 파이")
    return b.prims


def free_limits(scene: pk.Scene, q0: np.ndarray, robot: Robot) -> list[tuple[float, float]]:
    """q0에서 관절 하나씩만 돌려, 팔끼리나 책상, 카메라 파이에 닿기 전까지의 범위 [rad] (기계 한계 안, 2° 걸음)."""
    out = []
    for k in range(6):
        ends = []
        for step, limit in ((-2 * DEG, robot.lower[k]), (2 * DEG, robot.upper[k])):
            q = q0.copy()
            angle = q0[k]
            while (step < 0 and angle + step >= limit - 1e-9) or (
                step > 0 and angle + step <= limit + 1e-9
            ):
                q[k] = angle + step
                if pk.collisions(pk.hits(scene, q)):
                    break
                angle += step
            ends.append(angle)
        out.append((ends[0], ends[1]))
    return out


def demo(work: np.ndarray, limits: list[tuple[float, float]]) -> list[dict]:
    """끝까지 돌려 보기: 작업 자세에서 J1부터 J5까지 하나씩 양 끝으로 갔다가 돌아온다 (J6은 카메라보다 끝이라 뺀다)."""
    out = []
    for k in range(5):
        for angle in limits[k]:
            q = work.copy()
            q[k] = angle
            out.append(
                {
                    "q": [round(float(a), 5) for a in q],
                    "hold": 1.0,
                    "j": k,
                    "deg": round(math.degrees(angle)),
                }
            )
        out.append({"q": [round(float(a), 5) for a in work], "hold": 0.6, "j": k, "deg": None})
    return out


def page_data() -> dict:
    c = ur_arm.Concept()
    arm = ap.page_data(c)
    v = arm["variants"][0]
    # 지금 설계의 D405 선(받침 밑을 돌아 J5 틈으로 가는 선)과 선 무게 자리는 이 안에 없다. 왼쪽 날개는 파이 받침으로 바꾼다
    drop = ("d405_wire", "wire_", "wing+1")
    prims = [p for p in v["prims"] if not p["n"].startswith(drop)]
    prims += [_box(name, x, y, z, role) for name, x, y, z, role in pi_boxes()]
    robot, design = ur_arm.build(c, ur_arm.GRIPPER)
    scene = pk.build_scene(robot, design, pi_fixture())
    pick = np.array(v["frames"][0]["q"], dtype=float)
    place = np.array(v["frames"][6]["q"], dtype=float)
    limits = free_limits(scene, pick, robot)
    route = []
    for item in ROUTE:
        if isinstance(item, Run):
            route.append(
                {
                    "kind": "run",
                    "name": item.name,
                    "l": item.link,
                    "a": list(item.a),
                    "b": list(item.b),
                }
            )
        else:
            route.append(
                {
                    "kind": "span",
                    "key": item.key,
                    "name": item.name,
                    "joints": item.joints,
                    "len": SPAN_MM[item.key],
                    "guide": [[link, list(p)] for link, p in item.guide],
                }
            )
    return {
        "arm": {
            "chain": v["chain"],
            "prims": prims,
            "labels": v["labels"],
            "tags": [t for t in v["tags"] if t["t"].startswith("J")]
            + [t for t in v["tags"] if t["t"] == "D405"],
            "flange": v["flange"],
            "tcp_mm": v["tcp_mm"],
        },
        "clips": [
            {"l": k.link, "p": list(k.at), "d": list(k.along), "o": list(k.out), "tag": k.tag}
            for k in CLIPS
        ],
        "route": route,
        "poses": {
            "pick": [round(float(a), 5) for a in pick],
            "place": [round(float(a), 5) for a in place],
        },
        "limits": [[round(lo, 5), round(hi, 5)] for lo, hi in limits],
        "demo": demo(pick, limits),
        "wire": {"d": wr.D405_D, "stock": wr.D405_STOCK_MM},
        "pi": {"at": [round((PI_BOARD[0][0] + PI_BOARD[0][1]) / 2, 1), 78.0, 30.0]},
    }


def render() -> str:
    template = (HERE / "d405_route_page.html").read_text(encoding="utf-8")
    data = json.dumps(page_data(), ensure_ascii=False, separators=(",", ":"))
    return template.replace("__DATA__", data)


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / "d405_route.html"
    out.write_text(render(), encoding="utf-8", newline="\n")
    print(f"썼다: {out} ({out.stat().st_size / 1000:.0f} kB)")


if __name__ == "__main__":
    main()
