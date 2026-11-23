"""관 글자와 D405 타이의 검토: 홈 자세의 가림 검사와, 관절이 도는 동안 이웃 링크와 이루는 틈.

홈 자세의 가림 검사 (0019): 로봇 오른쪽(-y)에서 찍은 사진 한 장에 관 글자 BOMI와 HORI가 다 보이는지.

팔의 모든 부품(placement의 묶음: 몸통, 나사, 선반, 파이, 찍찍이 타이와 두 타이 사이에 묶인 D405 선)과 책상을 홈 자세에
놓고, 낱자 앞면의 점마다 카메라까지 곧은 선이 다른 부품에 막히는지 본다. 카메라는 한 자리가 아니라 범위로 본다
(가정: -y 정면에서 위아래·앞뒤로 ±15°, 거리 0.6 ~ 1.2 m. 사진을 찍는 자리가 조금 달라도 되게).

관절을 넘는 고리(D405 선의 J1·J2, J3, J4·J5 고리와 J2·J3 서보 선의 남는 고리)는 모양이 정해지지 않는다. 사진을 찍기 전에
손으로 비켜 놓아도 되므로(2026-11-22), 고리마다 글자를 가리지 않게 놓을 자리가 있는지만 본다: 두 끝 사이에 고리 길이의
둥근 호를 여러 방향으로 놓아 보고, 어느 카메라에서도 글자를 가리지 않는 방향을 센다. 호가 팔의 부품을 지나가는지는
보지 않는다 (놓을 자리의 방향만 본다).

틈 (extra_gaps, pose_gaps): 글자(위팔의 BOMI는 납작한 면에서 1.4 mm 나온다)와 타이·묶인 선은 링크 겉에 덧붙은 것이라,
링크끼리 마주 보는 5.8 mm 층(위팔과 아래팔, 위팔과 J1 몸통 왼쪽 면)과 접힌 자세에서 이웃 링크의 부품과 가까워진다.
J2와 J3를 범위 끝까지 돌리며 다른 링크의 부품(나사 포함)과의 가장 좁은 틈을 본다. 기준은 움직이는 틈 2 mm다 (0017).

    uv run python -m horibot_design.studies.home_view [gaps]
"""

from __future__ import annotations

import functools
import itertools
import math
import sys
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np
import shapely
from build123d import Compound

from horibot_design import placement as pl
from horibot_design.parts import joint as j
from horibot_design.parts import links as lp
from horibot_design.parts import rest_shelf as rs
from horibot_design.parts import shoulder as sp
from horibot_design.parts.shapes import solid
from horibot_design.studies import wiring as wr

Vec3 = tuple[float, float, float]

# 홈 자세 H3 (0019, 2026-11-22): 운동학의 관절 각 [°]. 도구가 곧게 아래를 본다
HOME_DEG = (0.0, 15.0, 110.0, 145.0, 90.0, 0.0)
# 카메라의 범위 (가정): 로봇 오른쪽(-y) 정면에서 앞뒤(수직축 둘레)와 위아래로 기운 각, 글자까지의 거리
CAM_TILT_DEG = 15.0
CAM_DIST = (600.0, 1200.0)
SAMPLE = 0.6  # 낱자 앞면에 찍는 점의 간격 [mm]
EPS = 0.05  # 점을 앞면에서 띄우는 거리 (낱자 자신과 닿지 않게)
LOOP_DIRS = 12  # 고리를 놓아 보는 방향의 수
EXTRAS = ("글자", "D405 선")  # 링크 겉에 덧붙은 것의 묶음 (placement의 이름)
MOVING_GAP = 2.0  # 서로 움직이는 부품 사이에 둘 틈 [mm] (0017)
# J2를 보는 범위 [°]: 팔꿈치 집이 책상에 닿는 +128°와 뒤로 넘어가는 -120°까지 (studies/wiring.py의 BASE_LOOP_Q2)
SWEEP_J2 = wr.BASE_LOOP_Q2


@dataclass(frozen=True)
class Mesh:
    """부품 하나의 삼각형 (부품 좌표, mm)."""

    name: str
    frame: str
    tri: np.ndarray  # (n, 3, 3)


@functools.cache
def _meshes(desk: bool = True) -> tuple[Mesh, ...]:
    """가릴 수 있는 것들의 삼각형: 팔의 모든 묶음(글자는 빼고)과 책상."""
    parts = [p for p in pl.arm_parts() if not p.name.startswith("글자/")]
    if desk:
        parts += list(pl.desk_parts())
    out: list[Mesh] = []
    for p in parts:
        verts, tris = p.shape.tessellate(0.1, 0.4)
        if not tris:
            continue
        pts = np.array([(v.X, v.Y, v.Z) for v in verts])
        out.append(Mesh(p.name, p.frame, pts[np.array(tris)]))
    return tuple(out)


@functools.cache
def letter_points(key: str) -> np.ndarray:
    """낱자 앞면의 점들 (링크 좌표, (n, 3)): 윤곽 안에 SAMPLE 간격으로 찍고 앞면에서 EPS 띄운다."""
    name, _k = lp._link_of(key)
    outline = lp.glyph_outline(key)
    x0, y0, x1, y1 = outline.bounds
    xs, ys = np.meshgrid(np.arange(x0, x1 + SAMPLE, SAMPLE), np.arange(y0, y1 + SAMPLE, SAMPLE))
    inside = shapely.contains_xy(outline.buffer(-0.05), xs.ravel(), ys.ravel())
    lift = lp.LETTER_T[lp.WORDS[name][1]] + EPS
    return np.array(
        [
            lp.letter_point(name, float(s), float(t), lift)
            for s, t in zip(xs.ravel()[inside], ys.ravel()[inside], strict=True)
        ]
    )


def _to_base(T: np.ndarray, pts: np.ndarray) -> np.ndarray:
    return pts @ T[:3, :3].T + T[:3, 3]


def cameras(
    target: np.ndarray, tilts: Sequence[float] | None = None, dists: Sequence[float] = CAM_DIST
) -> list[tuple[str, np.ndarray]]:
    """카메라 자리들 (베이스 좌표, mm): target에서 -y 쪽으로, 앞뒤와 위아래로 tilts만큼 기운 방향의 거리 dists."""
    tilts = (-CAM_TILT_DEG, 0.0, CAM_TILT_DEG) if tilts is None else tilts
    out: list[tuple[str, np.ndarray]] = []
    for d in dists:
        for az in tilts:  # +면 로봇 앞쪽(+x)에서 본다
            for el in tilts:  # +면 위에서 본다
                a, e = math.radians(az), math.radians(el)
                way = np.array([math.sin(a) * math.cos(e), -math.cos(a) * math.cos(e), math.sin(e)])
                out.append((f"거리 {d:.0f}, 앞뒤 {az:+.1f}°, 위아래 {el:+.1f}°", target + d * way))
    return out


def _hits(points: np.ndarray, cam: np.ndarray, tri: np.ndarray) -> np.ndarray:
    """점마다 카메라까지의 선분이 삼각형들 가운데 하나에 막히는지 (Möller–Trumbore, 점 × 삼각형)."""
    if len(tri) == 0 or len(points) == 0:
        return np.zeros(len(points), dtype=bool)
    out = np.zeros(len(points), dtype=bool)
    v0, e1, e2 = tri[:, 0], tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]
    for lo in range(0, len(points), 256):
        o = points[lo : lo + 256]
        d = cam[None, :] - o  # (p, 3): 선분 전체 (t가 0 ~ 1)
        pvec = np.cross(d[:, None, :], e2[None, :, :])  # (p, n, 3)
        det = np.einsum("nk,pnk->pn", e1, pvec)
        ok = np.abs(det) > 1e-12
        inv = np.where(ok, 1.0 / np.where(ok, det, 1.0), 0.0)
        tvec = o[:, None, :] - v0[None, :, :]
        u = np.einsum("pnk,pnk->pn", tvec, pvec) * inv
        qvec = np.cross(tvec, e1[None, :, :])
        v = np.einsum("pk,pnk->pn", d, qvec) * inv
        t = np.einsum("nk,pnk->pn", e2, qvec) * inv
        hit = ok & (u >= 0) & (v >= 0) & (u + v <= 1) & (t > 1e-6) & (t < 1.0)
        out[lo : lo + 256] = hit.any(axis=1)
    return out


def _seg_dist(c: np.ndarray, a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """점들 c ((n, 3))에서 선분 a-b까지의 거리."""
    ab = b - a
    t = np.clip(((c - a) @ ab) / float(ab @ ab), 0.0, 1.0)
    return np.linalg.norm(c - (a + t[:, None] * ab), axis=1)


@dataclass(frozen=True)
class Solid:
    """가릴 수 있는 물체 하나 (베이스 좌표): 삼각형과, 빨리 거르는 데 쓰는 둘러싼 공."""

    name: str
    tri: np.ndarray  # (n, 3, 3)
    centre: np.ndarray
    radius: float
    tri_c: np.ndarray  # 삼각형마다 가운데 (n, 3)
    tri_r: np.ndarray  # 삼각형마다 가운데에서 가장 먼 꼭짓점까지 (n,)


def solid_of(name: str, tri: np.ndarray) -> Solid:
    flat = tri.reshape(-1, 3)
    centre = (flat.min(axis=0) + flat.max(axis=0)) / 2
    tri_c = tri.mean(axis=1)
    tri_r = np.linalg.norm(tri - tri_c[:, None, :], axis=2).max(axis=1)
    return Solid(
        name, tri, centre, float(np.linalg.norm(flat - centre, axis=1).max()), tri_c, tri_r
    )


def blocked(points: np.ndarray, cam: np.ndarray, body: Solid) -> np.ndarray:
    """점마다 카메라까지의 선분이 물체에 막히는지. 점들과 카메라를 잇는 선분 둘레에 닿는 삼각형만 따져 본다."""
    c = points.mean(axis=0)
    r = float(np.linalg.norm(points - c, axis=1).max())  # 모든 선분은 c-카메라 선분에서 r 안에 있다
    if _seg_dist(body.centre[None, :], c, cam)[0] > body.radius + r:
        return np.zeros(len(points), dtype=bool)
    keep = _seg_dist(body.tri_c, c, cam) <= body.tri_r + r
    return _hits(points, cam, body.tri[keep])


@dataclass(frozen=True)
class View:
    """카메라 하나에서 본 결과: 낱자마다 보이는 몫과, 가린 부품."""

    camera: str
    seen: dict[str, float]  # 낱자 → 보이는 점의 몫 (1.0이면 다 보인다)
    blockers: dict[str, tuple[str, ...]]  # 낱자 → 가린 부품들


def letters_at(q_deg: Sequence[float]) -> dict[str, np.ndarray]:
    """자세 q에서 낱자 앞면의 점들 (베이스 좌표)."""
    frames = pl.frames_mm(np.radians(q_deg))
    return {
        key: _to_base(frames[pl.LINK_FRAME[name]], letter_points(key))
        for name in lp.LINKS
        for key in lp.letter_keys(name)
    }


def target_of(points: dict[str, np.ndarray]) -> np.ndarray:
    """카메라가 겨누는 점: 두 글자의 가운데."""
    return np.concatenate(list(points.values())).mean(axis=0)


@functools.lru_cache(maxsize=4)
def scene(q_deg: tuple[float, ...]) -> tuple[Solid, ...]:
    """자세 q에 놓은 가릴 수 있는 것들 (베이스 좌표)."""
    frames = pl.frames_mm(np.radians(q_deg))
    return tuple(
        solid_of(m.name, _to_base(frames[m.frame], m.tri.reshape(-1, 3)).reshape(-1, 3, 3))
        for m in _meshes()
    )


def views(
    q_deg: Sequence[float] = HOME_DEG,
    tilts: Sequence[float] | None = None,
    dists: Sequence[float] = CAM_DIST,
    extra: Sequence[Solid] = (),
) -> list[View]:
    """자세 q에서 카메라마다 낱자가 보이는 몫. extra는 더 넣을 가림 물체(베이스 좌표)다."""
    points = letters_at(q_deg)
    solids = [*scene(tuple(float(v) for v in q_deg)), *extra]
    out: list[View] = []
    for label, cam in cameras(target_of(points), tilts, dists):
        seen: dict[str, float] = {}
        blockers: dict[str, tuple[str, ...]] = {}
        for key, pts in points.items():
            hidden = np.zeros(len(pts), dtype=bool)
            names: list[str] = []
            for body in solids:
                hit = blocked(pts, cam, body)
                if hit.any():
                    names.append(body.name)
                    hidden |= hit
            seen[key] = 1.0 - float(hidden.mean())
            blockers[key] = tuple(names)
        out.append(View(label, seen, blockers))
    return out


# ------------------------------------------------------------------------------------------------
# 고리: 놓을 자리가 있는지


@dataclass(frozen=True)
class FreeLoop:
    """관절을 넘는 고리 하나: 두 끝(베이스 좌표)과 길이 [mm]."""

    name: str
    a: np.ndarray
    b: np.ndarray
    length: float

    @property
    def slack(self) -> float:
        return self.length - float(np.linalg.norm(self.b - self.a))


def free_loops(q_deg: Sequence[float] = HOME_DEG) -> list[FreeLoop]:
    """자세 q에서 관절을 넘는 고리들: D405 선의 고리 셋과 J2·J3 서보 선의 남는 고리 (겉으로 도는 길)."""
    frames = pl.frames_mm(np.radians(q_deg))

    def at(frame: str, p: Vec3) -> np.ndarray:
        return (frames[frame] @ np.array([*p, 1.0]))[:3]

    half = lp.TIE_W / 2
    (uf, uh), (ff, fh) = lp.tie_x("upper_arm"), lp.tie_x("forearm")
    (uy, uz), (fy, fz) = lp.cable_yz("upper_arm"), lp.cable_yz("forearm")
    hold = wr.D405_EXITS["카메라 바닥면 밑 (앞선 가정, J5 축에서 62)"]
    out = [
        FreeLoop(
            "D405 J4·J5 고리",
            at("j4", (fh + half, fy, fz)),
            at("j6", hold),
            wr.d405_wrist_loop().length,
        ),
        FreeLoop(
            "D405 J3 고리",
            at("j3", (uh + half, uy, uz)),
            at("j4", (ff - half, fy, fz)),
            wr.d405_elbow_loop().length,
        ),
        FreeLoop(
            "D405 J1·J2 고리",
            at("j3", (uf - half, uy, uz)),
            np.array(wr.d405_base_tie()),
            wr.d405_base_loop().length,
        ),
    ]
    # 서보 선: 부모에서 나가는 자리(선 창, J1 몸통 왼쪽 면의 누르개 위)와 발 끝벽의 창 밖
    win = sum(j.WIN_X) / 2
    z_win = sum(lp.FOOT_SLOT_Z) / 2
    h_win = sum(lp.FOOT_WINDOW) / 2
    for name, link, parent, exit_p in (
        ("J3 서보 선의 고리", "forearm", "j3", (win, j.FRONT, 0.0)),
        ("J2 서보 선의 고리", "upper_arm", "j2", (-wr.STRIP_R, sp.FRONT, 0.0)),
    ):
        span, _r, foot = lp.LINKS[link]
        entry = (-span - foot.hx - wr.LINK_WINDOW_OUT, j.FRONT - h_win, z_win)
        joint = "J3" if link == "forearm" else "J2"
        out.append(
            FreeLoop(
                name,
                at(parent, exit_p),
                at(pl.LINK_FRAME[link], entry),
                wr.over_link(joint).longest(),
            )
        )
    return out


def _arc(loop: FreeLoop, way: np.ndarray, n: int = 40) -> np.ndarray:
    """두 끝 사이에 길이 loop.length인 둥근 호를 way 쪽으로 부풀려 놓은 점들 ((n + 1, 3))."""
    chord = loop.b - loop.a
    c = float(np.linalg.norm(chord))
    u = chord / c
    w = way - (way @ u) * u
    w /= np.linalg.norm(w)
    if loop.length <= c * 1.0005:
        return np.linspace(loop.a, loop.b, n + 1)
    r = wr.loop_radius(c, loop.length)
    theta = loop.length / r  # 호의 각
    mid = (loop.a + loop.b) / 2
    centre = mid - w * (r * math.cos(theta / 2))
    angles = np.linspace(-theta / 2, theta / 2, n + 1)
    return centre[None, :] + r * (
        np.sin(angles)[:, None] * u[None, :] + np.cos(angles)[:, None] * w[None, :]
    )


def _tube(points: np.ndarray, radius: float, sides: int = 6) -> np.ndarray:
    """꺾은선을 감싸는 관의 삼각형 ((m, 3, 3))."""
    tris: list[np.ndarray] = []
    rings: list[np.ndarray] = []
    for k, p in enumerate(points):
        t = points[min(k + 1, len(points) - 1)] - points[max(k - 1, 0)]
        t = t / np.linalg.norm(t)
        a = np.array([0.0, 0.0, 1.0]) if abs(t[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        e1 = np.cross(t, a)
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(t, e1)
        ang = np.linspace(0.0, 2 * math.pi, sides, endpoint=False)
        rings.append(p + radius * (np.cos(ang)[:, None] * e1 + np.sin(ang)[:, None] * e2))
    for r0, r1 in itertools.pairwise(rings):
        for i in range(sides):
            k = (i + 1) % sides
            tris.append(np.array([r0[i], r0[k], r1[i]]))
            tris.append(np.array([r0[k], r1[k], r1[i]]))
    return np.array(tris)


def loop_room(
    q_deg: Sequence[float] = HOME_DEG, tilts: Sequence[float] | None = None
) -> dict[str, tuple[float, int, bool]]:
    """고리마다 (남는 길이 [mm], 글자를 어느 카메라에서도 가리지 않는 방향의 수, 아래로 늘어뜨려도 안 가리는지).

    방향은 두 끝을 잇는 선에 직각인 LOOP_DIRS개다. 남는 길이가 없는 고리는 곧은 선 하나로 본다.
    """
    points = letters_at(q_deg)
    cams = cameras(target_of(points), tilts)
    out: dict[str, tuple[float, int, bool]] = {}
    for loop in free_loops(q_deg):
        u = (loop.b - loop.a) / np.linalg.norm(loop.b - loop.a)
        a = np.array([0.0, 0.0, 1.0]) if abs(u[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        e1 = np.cross(u, a)
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(u, e1)

        def clear(way: np.ndarray, loop: FreeLoop = loop) -> bool:
            body = solid_of(loop.name, _tube(_arc(loop, way), max(lp.CABLE_D, j.WIRE_W) / 2))
            return not any(
                blocked(pts, cam, body).any() for _label, cam in cams for pts in points.values()
            )

        ways = [
            math.cos(2 * math.pi * k / LOOP_DIRS) * e1 + math.sin(2 * math.pi * k / LOOP_DIRS) * e2
            for k in range(LOOP_DIRS)
        ]
        down = np.array([0.0, 0.0, -1.0])
        hang = clear(down) if abs(down @ u) < 0.98 else True
        out[loop.name] = (loop.slack, sum(clear(w) for w in ways), hang)
    return out


# ------------------------------------------------------------------------------------------------
# 틈: 글자, 타이, 묶인 선과 다른 링크의 부품 사이


def _group(name: str) -> str:
    return name.split("/")[0]


@dataclass(frozen=True)
class _Shell:
    """부품 하나의 거르기용 자료 (부품 좌표): 바깥 상자와, 덧붙은 것이면 겉의 점들."""

    part: pl.ArmPart
    lo: np.ndarray
    hi: np.ndarray
    points: np.ndarray  # (n, 3). 덧붙은 것(EXTRAS)만 채운다


SAMPLE_GAP = (
    4.0  # 덧붙은 것의 겉에 찍은 점 사이가 이보다 넓지 않다고 보고 상자를 이만큼 더 키워 거른다
)


@functools.cache
def _shells() -> tuple[_Shell, ...]:
    out: list[_Shell] = []
    for part in pl.arm_parts():
        bb = part.shape.bounding_box()
        lo, hi = np.array([bb.min.X, bb.min.Y, bb.min.Z]), np.array([bb.max.X, bb.max.Y, bb.max.Z])
        pts = np.zeros((0, 3))
        if _group(part.name) in EXTRAS:
            verts, tris = part.shape.tessellate(0.05, 0.2)
            v = np.array([(q.X, q.Y, q.Z) for q in verts])
            tri = v[np.array(tris)]
            # 꼭짓점, 삼각형의 가운데, 변의 가운데
            mids = (tri + np.roll(tri, 1, axis=1)).reshape(-1, 3) / 2
            pts = np.concatenate([v, tri.mean(axis=1), mids])
        out.append(_Shell(part, lo, hi, pts))
    return tuple(out)


def extra_gap(
    q_deg: Sequence[float],
    limit: float = 6.0,
    own: Sequence[int] = (2, 3),
    others: Sequence[int] = (0, 1, 2, 3, 4, 5, 6),
) -> tuple[float, str, str] | None:
    """자세 q에서 링크 겉에 덧붙은 것(EXTRAS)과 다른 링크의 부품 사이의 가장 좁은 틈: (틈, 덧붙은 것, 부품).

    limit보다 넓으면 None이다. 나사와 너트도 본다. 덧붙은 것끼리(위팔의 타이와 아래팔의 타이)도 본다.
    own은 볼 덧붙은 것이 붙은 링크(2 위팔, 3 아래팔), others는 견줄 부품의 링크다.
    형상끼리의 거리는 덧붙은 것의 겉의 점이 상대 부품의 바깥 상자(limit만큼 키운 것) 안에 드는 쌍만 잰다.
    """
    frames = pl.frames_mm(np.radians(q_deg))
    spots = {name: pl.location_of(T) for name, T in frames.items()}
    back = {name: np.linalg.inv(T) for name, T in frames.items()}
    moved: dict[str, Compound] = {}

    def at(part: pl.ArmPart) -> Compound:
        if part.name not in moved:
            moved[part.name] = solid(part.shape.moved(spots[part.frame]))
        return moved[part.name]

    shells = _shells()
    grow = limit + SAMPLE_GAP
    best: tuple[float, str, str] | None = None
    for mine in shells:
        a = mine.part
        if _group(a.name) not in EXTRAS or a.link not in own:
            continue
        world = _to_base(frames[a.frame], mine.points)
        for other in shells:
            b = other.part
            if a.link == b.link or b.link not in others:
                continue
            if _group(b.name) in EXTRAS and b.link in own and b.name < a.name:
                continue  # 이미 잰 쌍
            local = _to_base(back[b.frame], world)
            if not ((local >= other.lo - grow) & (local <= other.hi + grow)).all(axis=1).any():
                continue
            d = at(a).distance_to(at(b))
            if d < limit and (best is None or d < best[0]):
                best = (d, a.name, b.name)
    return best


def extra_gaps(step: float = 4.0) -> dict[str, tuple[float, float, str, str]]:
    """관절 하나씩을 범위 끝까지 돌릴 때의 가장 좁은 틈: 돌린 관절 → (틈, 그 관절의 각 [°], 덧붙은 것, 부품).

    - J2: SWEEP_J2 범위. 위팔의 글자와 타이·묶인 선이 J1 몸통 왼쪽 면과 누르개, 어깨 집, 베이스 쪽 것들 위를 지난다
      (링크 0과 1의 부품만 견준다).
    - J3: 프로그램 한계 ±160°와 멈춤 각 ±163°. 접힌 아래팔이 위팔의 납작한 면 위를 지난다 (위팔과 아래팔에 덧붙은
      것을 서로의 링크, J1 몸통과 견준다. 손목이 관에 닿는 자세는 팔 전체 모델 studies/ur_arm.py가 본다).
    """
    lo, hi = SWEEP_J2
    j3 = (-j.STOP_DEG, *np.arange(-j.SOFT_DEG, j.SOFT_DEG, step), j.SOFT_DEG, j.STOP_DEG)
    sweeps: dict[
        str, tuple[list[tuple[float, tuple[float, ...]]], tuple[int, ...], tuple[int, ...]]
    ]
    sweeps = {
        "J2": (
            [
                (float(q), (0.0, float(q), 0.0, 180.0, 0.0, 0.0))
                for q in (*np.arange(lo, hi, step), hi)
            ],
            (2,),
            (0, 1),
        ),
        "J3": (
            [(float(q), (0.0, 0.0, float(q), 180.0, 0.0, 0.0)) for q in j3],
            (2, 3),
            (1, 2, 3),
        ),
    }
    out: dict[str, tuple[float, float, str, str]] = {}
    for name, (poses, own, others) in sweeps.items():
        worst: tuple[float, float, str, str] | None = None
        for angle, q in poses:
            found = extra_gap(q, own=own, others=others)
            if found is not None and (worst is None or found[0] < worst[0]):
                worst = (found[0], angle, found[1], found[2])
        out[name] = worst if worst is not None else (math.inf, 0.0, "", "")
    return out


def pose_gaps(
    q_deg: Sequence[float], limit: float = MOVING_GAP
) -> tuple[dict[tuple[str, str], float], dict[tuple[str, str], float]]:
    """자세 하나에서 팔 전체의 (틈, 겹침).

    - 틈 [mm]: 관절 둘 넘게 떨어진 링크의 부품끼리 limit보다 좁은 쌍 (나사 포함). 이웃한 링크는 관절 하나로 이어져
      좁게 그린 틈이 많아서 뺀다 (그쪽은 관절 모듈의 검사와 extra_gaps가 본다).
    - 겹침 [mm³]: 링크 몸통(위팔, 아래팔)이나 거기 덧붙은 것이, 다른 링크의 부품과 0.05 mm³ 넘게 겹친 쌍 (나사와
      너트는 빼고). 그 밖의 부품끼리의 겹침은 부품 모듈마다의 조립 검사가 본다.
    placement의 gaps, overlaps와 같은 뜻이다. 형상끼리의 거리를 재기 전에, 한 부품의 바깥 상자를 다른 부품의 좌표로
    옮겨 그 부품의 바깥 상자와 견주어 먼 쌍을 거른다 (기운 긴 링크의 상자가 헐거워서 두 쪽으로 다 본다).
    """
    frames = pl.frames_mm(np.radians(q_deg))
    spots = {name: pl.location_of(T) for name, T in frames.items()}
    back = {name: np.linalg.inv(T) for name, T in frames.items()}
    shells = _shells()
    unit = np.array([(x, y, z) for x in (0, 1) for y in (0, 1) for z in (0, 1)], dtype=float)
    world = [_to_base(frames[sh.part.frame], sh.lo + unit * (sh.hi - sh.lo)) for sh in shells]
    moved: dict[int, Compound] = {}

    def at(k: int) -> Compound:
        if k not in moved:
            part = shells[k].part
            moved[k] = solid(part.shape.moved(spots[part.frame]))
        return moved[k]

    def apart(i: int, k: int) -> float:
        """부품 i의 바깥 상자(여덟 모서리)를 부품 k의 좌표로 옮겨, k의 바깥 상자와 떨어진 거리 (실제 틈의 아래 값)."""
        local = _to_base(back[shells[k].part.frame], world[i])
        gap = np.maximum(
            np.maximum(local.min(axis=0) - shells[k].hi, shells[k].lo - local.max(axis=0)), 0.0
        )
        return float(np.linalg.norm(gap))

    gaps: dict[tuple[str, str], float] = {}
    over: dict[tuple[str, str], float] = {}
    bodies = {sh.part.name for sh in shells if _group(sh.part.name) in EXTRAS}
    bodies |= {f"{name}/{name}" for name in pl.LINK_NAME.values()}
    n = len(shells)
    for i in range(n):
        a = shells[i].part
        for k in range(i + 1, n):
            b = shells[k].part
            if a.link == b.link:
                continue
            far = abs(a.link - b.link) >= 2
            soft = not (a.hardware or b.hardware) and (a.name in bodies or b.name in bodies)
            if not far and not soft:
                continue
            reach = limit if far else 0.0
            if apart(i, k) > reach or apart(k, i) > reach:
                continue
            d = at(i).distance_to(at(k))
            if far and d < limit:
                gaps[(a.name, b.name)] = d
            if soft and d <= 1e-6:
                v = solid(at(i) & at(k)).volume
                if v > 0.05:
                    over[(a.name, b.name)] = v
    return gaps, over


def report_gaps() -> None:
    print(f"글자·타이·묶인 선과 다른 링크의 부품 사이 (기준 {MOVING_GAP:.0f} mm)")
    for name, (gap, angle, a, b) in extra_gaps().items():
        print(f"- {name}를 돌릴 때 가장 좁은 틈: {gap:.2f} mm ({name} {angle:+.0f}°, {a} ↔ {b})")
    for label, q in (("홈 자세 H3", HOME_DEG), ("쉬는 자세", rs.REST_DEG)):
        gaps, over = pose_gaps(q)
        near = extra_gap(q, 40.0)
        print(
            f"- {label}: 관절 둘 넘게 떨어진 링크 사이에 {MOVING_GAP:.0f} mm보다 좁은 쌍 "
            + (", ".join(f"{a} ↔ {b} {d:.2f}" for (a, b), d in gaps.items()) or "없음")
        )
        print(
            "    겹침 [mm³]: "
            + (", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in over.items()) or "없음")
        )
        if near is not None:
            print(f"    글자·타이·묶인 선의 가장 좁은 틈: {near[0]:.2f} mm ({near[1]} ↔ {near[2]})")


def report() -> None:
    print(f"홈 자세 H3 (운동학 J1 ~ J6 [°]): {HOME_DEG}")
    points = letters_at(HOME_DEG)
    t = target_of(points)
    print(
        f"카메라 (가정): 로봇 오른쪽(-y)에서 글자 가운데 ({t[0]:.0f}, {t[1]:.0f}, {t[2]:.0f})를 겨눈다. "
        f"앞뒤·위아래 ±{CAM_TILT_DEG:.0f}°, 거리 {CAM_DIST[0]:.0f} ~ {CAM_DIST[1]:.0f} mm"
    )
    tilts = (-CAM_TILT_DEG, -CAM_TILT_DEG / 2, 0.0, CAM_TILT_DEG / 2, CAM_TILT_DEG)
    found = views(HOME_DEG, tilts)
    worst = {key: min(v.seen[key] for v in found) for key in points}
    print(f"- 카메라 {len(found)}곳에서 낱자마다 보이는 몫의 가장 작은 값 (점 {SAMPLE} mm 간격):")
    print("    " + ", ".join(f"{lp.LABELS[k]} {100 * s:.0f}%" for k, s in worst.items()))
    bad = [
        (v.camera, k, v.seen[k], v.blockers[k]) for v in found for k in points if v.seen[k] < 1.0
    ]
    if not bad:
        print("- 가리는 부품: 없음 (단단한 부품, 찍찍이 타이, 두 타이 사이에 묶인 선)")
    for cam, key, share, names in bad:
        print(f"    {cam}: {lp.LABELS[key]} {100 * share:.0f}% ← {', '.join(names)}")
    print(
        f"- 고리를 비켜 놓을 자리 (두 끝 사이의 둥근 호를 {LOOP_DIRS}방향으로 놓아, 어느 카메라에서도 글자를 안 가리는 "
        "방향의 수):"
    )
    for name, (slack, ways, hang) in loop_room(HOME_DEG, tilts).items():
        print(
            f"    {name}: 남는 길이 {slack:.0f} mm, {ways}/{LOOP_DIRS}방향, 아래로 늘어뜨리면 "
            f"{'안 가린다' if hang else '가린다'}"
        )


if __name__ == "__main__":
    report()
    if "gaps" in sys.argv[1:]:
        report_gaps()
