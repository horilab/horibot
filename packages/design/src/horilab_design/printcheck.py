"""출력 검사에 쓰는 계산: 출력 무게 어림, 바닥에 닿지 않고 아래를 보는 면 찾기, 층 가운데에 걸친 수평면 찾기,
공중에 뽑히는 벽 선 찾기 (docs/design.md의 "출력 전 확인").

형상은 부품 좌표 그대로 주고, 뽑을 때 위를 보는 방향(up)을 같이 준다.
"""

from __future__ import annotations

import functools
import math
import tomllib
from collections import defaultdict
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Literal

import numpy as np
from build123d import Compound, GeomType
from shapely import LineString, MultiLineString, MultiPolygon, Polygon
from shapely.geometry.base import BaseGeometry
from shapely.ops import linemerge

from horilab_design.values import REPO_ROOT

Vec = tuple[float, float, float]

# 출력 무게 어림식 (0.20mm Strength: 벽 6겹, 윗면 5층, 아랫면 3층, 채움 25%).
# 껍데기 부피 = Σ 넓이 × 두께 (세운 벽 2.52, 윗면 1.0, 아랫면 0.6, 비스듬한 면은 둘 중 큰 쪽), 부피를 넘으면 부피.
# 무게 = SOLID × (껍데기 + INFILL × 속). SOLID와 INFILL은 뽑아서 잰 Strength 부품 다섯(data/measured.toml의
# printed_mass_g: 서보 집, 원판 둘, 시험 링크, 받침 기둥)에 2% 안으로 맞춘 값이다 (tests/test_printcheck.py).
WALL_T, TOP_T, BOTTOM_T = 2.52, 1.0, 0.6
SOLID = 1.008e-3  # [g/mm³]
INFILL = 0.32
# 0.20mm Standard로 뽑는 얇은 덮개: 잰 것이 고리 하나뿐이라 그 값(무게 / 부피)을 쓴다
STANDARD_THIN = 1.125e-3  # [g/mm³]
LAYER = 0.2  # 층 높이 [mm] (params/print.toml의 두 프리셋 모두. 첫 층도 같다)


def _mesh(
    shape: Compound, tol: float = 0.05, ang: float = 0.3
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """삼각형마다 꼭짓점(3 × 3), 넓이, 바깥 단위 법선."""
    verts, tris = shape.tessellate(tol, ang)
    pts = np.array([(v.X, v.Y, v.Z) for v in verts])
    idx = np.array(tris)
    tri = pts[idx]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    length = np.linalg.norm(n, axis=1)
    return tri, length / 2, n / np.maximum(length, 1e-12)[:, None]


def printed_mass_g(shape: Compound, up: Vec, preset: str = "strength") -> float:
    """출력 무게 어림 [g]. preset은 "strength"(구조 부품)나 "standard"(두께 3 mm 이하 덮개)다."""
    volume = shape.volume
    if preset == "standard":
        return STANDARD_THIN * volume
    _, area, normal = _mesh(shape)
    nz = normal @ np.asarray(up, float)
    flat = np.where(nz > 0, TOP_T, BOTTOM_T) * np.abs(nz)
    wall = WALL_T * np.sqrt(np.clip(1 - nz**2, 0, 1))
    shell = min(float((area * np.maximum(flat, wall)).sum()), volume)
    return SOLID * (shell + INFILL * (volume - shell))


@dataclass(frozen=True)
class Overhangs:
    """바닥에 닿지 않으면서 수평에서 45°보다 누워 아래를 보는 면 [mm²]."""

    total: float
    flat: float  # 그중 거의 수평인 면 (다리로 뽑힌다)
    largest: float  # 한 자리(같은 높이, 6 mm 칸)에 모인 가장 넓은 면
    spots: tuple[
        tuple[float, float, float, float, bool], ...
    ]  # (넓이, 높이, 자리 u, 자리 v, 수평인지), 넓은 것부터


def overhangs(shape: Compound, up: Vec, grid: float = 6.0) -> Overhangs:
    """뽑는 방향으로 놓았을 때 아래를 보는 면. 바닥은 형상에서 가장 낮은 곳으로 본다."""
    tri, area, normal = _mesh(shape, ang=0.25)
    down = -np.asarray(up, float)
    cos_down = normal @ down  # 1이면 똑바로 아래를 본다
    center = tri.mean(axis=1)
    bed = float((tri.reshape(-1, 3) @ down).max())
    height = bed - center @ down
    over = (cos_down > 0.7072) & (height > 0.05)
    flat = over & (cos_down > 0.94)
    axes = [i for i in range(3) if abs(down[i]) < 0.5]
    groups: dict[tuple[int, int, int, bool], float] = defaultdict(float)
    for k in np.nonzero(over)[0]:
        u, v = center[k, axes[0]], center[k, axes[1]]
        groups[
            (round(float(height[k])), round(float(u) / grid), round(float(v) / grid), bool(flat[k]))
        ] += float(area[k])
    spots = sorted(
        ((a, float(h), u * grid, v * grid, fl) for (h, u, v, fl), a in groups.items()), reverse=True
    )
    return Overhangs(
        total=float(area[over].sum()),
        flat=float(area[flat].sum()),
        largest=spots[0][0] if spots else 0.0,
        spots=tuple(spots[:20]),
    )


def layer_snap(height: float, layer: float = LAYER) -> float:
    """바닥에서 잰 높이를 가장 가까운 층 경계로 맞춘다."""
    return round(height / layer) * layer


def mid_layer_levels(
    shape: Compound, up: Vec, layer: float = LAYER, tol: float = 0.02, min_area: float = 0.5
) -> list[float]:
    """수평인 평면 가운데 층의 한가운데에 걸친 것의 높이 [mm] (바닥에서, 낮은 것부터).

    슬라이서는 층마다 그 층의 가운데 높이에서 형상을 자른다. 수평면이 그 높이에 있으면 그 층이 뽑힐지가 계산
    오차로 갈려서, 뽑힌 높이가 그린 값과 한 층(0.2 mm) 다를 수 있다. 그런 면은 층 경계로 옮긴다. 층 가운데에서
    tol 안에 있는 평면을 찾고, 넓이가 min_area보다 작은 조각은 뺀다. 가로 구멍의 위아래 같은 곡면은 보지 않는다.
    """
    axis = np.asarray(up, float)
    axis = axis / np.linalg.norm(axis)
    tri, _, _ = _mesh(shape)
    bed = float((tri.reshape(-1, 3) @ axis).min())
    levels: dict[float, float] = defaultdict(float)
    for face in shape.faces():
        if face.geom_type != GeomType.PLANE:
            continue
        n = face.normal_at()
        if abs(n.X * axis[0] + n.Y * axis[1] + n.Z * axis[2]) < 0.9999:
            continue
        c = face.center()
        height = c.X * axis[0] + c.Y * axis[1] + c.Z * axis[2] - bed
        levels[round(float(height), 3)] += float(face.area)
    out: list[float] = []
    for z, area in sorted(levels.items()):
        off = abs((z / layer - 0.5) - round(z / layer - 0.5)) * layer  # 층 가운데에서 떨어진 거리
        if area >= min_area and off < tol - 1e-9:
            out.append(z)
    return out


# ---- 공중에 뽑히는 벽 선 ---------------------------------------------------------------------------------
#
# 슬라이서는 층마다 단면의 윤곽(바깥과 구멍)을 따라 벽 선을 정한 수만큼 안으로 겹쳐 두르고, 남은 안쪽을 채운다.
# 아래층이 없는 천장에서 안쪽은 다리 선(양 끝을 붙여 곧게 건너는 선)으로 채워지지만, 윤곽에서 벽 띠 안에 든
# 자리는 벽 선이 공중에 뽑힌다. 첫 관절 판(2026-11-15)에서 처진 자리가 모두 이것이었다: 구멍이 뚫린 천장(나사
# 머리 자리)은 구멍을 둘러싼 벽 고리가 어디에도 붙지 않고 통째로 공중이었고, 좁은 가로대 밑은 다리 선 없이
# 벽 선만으로 길게 건넜다. 잘 뽑힌 손목 몸통의 멈춤 홈 천장은 다리 선으로 채워지고, 공중 벽 선은 윤곽을 따라
# 양 끝이 붙은 짧은 구간뿐이었다 (docs/decisions/0035-slicer-automation.md의 공중 벽 선 검사).

AIR_GAP = (
    0.2  # 선 가운데가 아래층 단면에서 이만큼 넘게 떨어지면 선 폭(0.42 ~ 0.45) 전체가 공중이다 [mm]
)
RING_AIR = 0.5  # 구멍을 둘러싼 벽 고리가 둘레의 이만큼 넘게 공중이면 고리가 뜬다고 본다
LOOP_MIN = 2.0  # 이보다 짧은 고리는 점에 가까워 보지 않는다 [mm] (M2 구멍 둘레의 고리가 약 10 mm다)
# 다리 선이 들어갈 자리 없이 벽 선만으로 채워지는 천장에서, 양 끝이 붙은 공중 벽 선 구간의 한도 [mm].
# 가정: 손목 몸통 바깥 면의 0.6 mm 턱(벽 선 한 줄이 5.3 ~ 5.6 mm를 건넌다)은 처졌다는 말이 없었고, 서보 받침의
# 먼 가로대 밑(16.8 mm, 끝의 구멍 고리도 떴다)은 처졌다. 그 사이는 몰라서 덜 막는 쪽보다 다시 뽑지 않는 쪽을 골랐다
WALLS_ONLY_SPAN = 10.0
# 다리 선 옆을 지나는 공중 벽 선(천장 가장자리)은 막지 않고, 잘 뽑힌 손목 몸통 멈춤 홈의 가장자리(10.0 mm)보다
# 길면 참고로 알린다 [mm]
EDGE_REPORT = 10.0
AirKind = Literal["ring", "island", "span", "edge"]
AIR_KIND_KO: dict[str, str] = {
    "ring": "구멍을 둘러싼 벽 고리가 공중에 뜬다",
    "island": "벽 고리가 통째로 공중에 뜬 섬이다",
    "span": "다리 선 없이 벽 선만으로 공중을 길게 건넌다",
    "edge": "(참고) 천장 가장자리의 벽 선이 다리 선 옆으로 공중을 건넌다",
}
BLOCKING: frozenset[str] = frozenset({"ring", "island", "span"})


@dataclass(frozen=True)
class WallSettings:
    """벽 선의 자리를 정하는 출력 설정 (params/print.toml의 핵심 설정)."""

    loops: int  # 벽 수
    outer_w: float  # 바깥 벽 선 폭 [mm]
    inner_w: float  # 안쪽 벽 선 폭 [mm]
    layer: float  # 층 높이 [mm]

    def offsets(self) -> list[float]:
        """윤곽에서 벽 선 가운데까지 [mm], 바깥 벽부터. classic 벽: 선 사이 간격은 폭 - 층 × (1 - π/4)."""
        squash = self.layer * (1 - math.pi / 4)
        outer_sp, inner_sp = self.outer_w - squash, self.inner_w - squash
        out = [self.outer_w / 2]
        for i in range(1, self.loops):
            out.append(out[-1] + ((outer_sp + inner_sp) / 2 if i == 1 else inner_sp))
        return out


@functools.cache
def wall_settings(preset: str) -> WallSettings:
    """프리셋("strength", "standard")의 벽 설정을 값 파일에서 읽는다."""
    with (REPO_ROOT / "params" / "print.toml").open("rb") as f:
        keys = tomllib.load(f)["key_settings"]
    common, own = keys["common"], keys[preset]
    return WallSettings(
        loops=int(own["wall_loops"]),
        outer_w=float(common["outer_wall_line_width"]),
        inner_w=float(common["inner_wall_line_width"]),
        layer=float(common["layer_height"]),
    )


@dataclass(frozen=True)
class AirWall:
    """벽 선이 공중에 뽑히는 자리 하나."""

    kind: AirKind
    height: float  # 천장 높이 [mm] (바닥에서)
    where: Vec  # 자리 (부품 좌표) [mm]
    length: float  # 공중 구간의 길이 [mm] (고리와 섬은 둘레)
    loops: int  # 공중에 뜬 벽 선의 수

    @property
    def blocking(self) -> bool:
        """뽑기 전에 고쳐야 하는 자리인지 ("edge"는 참고)."""
        return self.kind in BLOCKING

    def describe(self) -> str:
        x, y, z = self.where
        return (
            f"{AIR_KIND_KO[self.kind]}: 바닥에서 {self.height:.2f} mm, 자리 ({x:.1f}, {y:.1f}, {z:.1f}), "
            f"공중 {self.length:.1f} mm, 벽 선 {self.loops}줄"
        )


def _welded_mesh(
    shape: Compound, tol: float = 0.05, ang: float = 0.3
) -> tuple[np.ndarray, np.ndarray]:
    """꼭짓점을 합친 삼각형 메시 (면 경계에서 모서리를 같이 쓰게 한다)."""
    verts, tris = shape.tessellate(tol, ang)
    pts = np.array([(v.X, v.Y, v.Z) for v in verts])
    _, first, inverse = np.unique(
        np.round(pts / 1e-5).astype(np.int64), axis=0, return_index=True, return_inverse=True
    )
    idx = inverse.reshape(-1)[np.array(tris, dtype=np.int64)]
    keep = (idx[:, 0] != idx[:, 1]) & (idx[:, 1] != idx[:, 2]) & (idx[:, 2] != idx[:, 0])
    return pts[first], idx[keep]


def _polygons(geom: BaseGeometry) -> list[Polygon]:
    if isinstance(geom, Polygon):
        return [] if geom.is_empty else [geom]
    if isinstance(geom, MultiPolygon):
        return list(geom.geoms)
    return [g for g in getattr(geom, "geoms", []) if isinstance(g, Polygon) and not g.is_empty]


def _lines(geom: BaseGeometry) -> list[LineString]:
    if isinstance(geom, LineString):
        return [] if geom.is_empty else [geom]
    if isinstance(geom, MultiLineString):
        merged = linemerge(geom)
        return [merged] if isinstance(merged, LineString) else list(merged.geoms)
    return [g for g in getattr(geom, "geoms", []) if isinstance(g, LineString) and not g.is_empty]


def section(h: np.ndarray, uv: np.ndarray, tris: np.ndarray, at: float) -> BaseGeometry:
    """높이 at에서 자른 단면 (수평 좌표 uv의 다각형). 고리를 홀짝 규칙으로 겹쳐 구멍을 낸다."""
    ht = h[tris]
    up = ht > at
    crossing = np.nonzero(up.any(axis=1) & ~up.all(axis=1))[0]
    links: dict[tuple[int, int], list[tuple[int, int]]] = defaultdict(list)
    for t in crossing:
        a, b, c = (int(v) for v in tris[t])
        edges = [
            (min(p, q), max(p, q))
            for p, q in ((a, b), (b, c), (c, a))
            if (h[p] > at) != (h[q] > at)
        ]
        if len(edges) == 2:
            links[edges[0]].append(edges[1])
            links[edges[1]].append(edges[0])

    def point(e: tuple[int, int]) -> tuple[float, float]:
        p, q = e
        f = (at - h[p]) / (h[q] - h[p])
        x = uv[p] + f * (uv[q] - uv[p])
        return float(x[0]), float(x[1])

    region: BaseGeometry = Polygon()
    seen: set[tuple[int, int]] = set()
    for start in links:
        if start in seen:
            continue
        loop, prev, cur = [start], None, start
        seen.add(start)
        while True:
            nxt = [e for e in links[cur] if e != prev and e not in seen]
            if not nxt:
                break
            prev, cur = cur, nxt[0]
            seen.add(cur)
            loop.append(cur)
        if len(loop) >= 3:
            ring = Polygon([point(e) for e in loop]).buffer(0)
            region = region.symmetric_difference(ring)
    return region


def _frame(up: Vec) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """뽑는 방향의 좌표축: 수평 두 축과 위."""
    w = np.asarray(up, float)
    w = w / np.linalg.norm(w)
    a = np.array([1.0, 0.0, 0.0]) if abs(w[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    e1 = a - (a @ w) * w
    e1 = e1 / np.linalg.norm(e1)
    return e1, np.cross(w, e1), w


def air_walls(shape: Compound, up: Vec, walls: WallSettings) -> tuple[AirWall, ...]:
    """천장(아래를 보는 거의 수평인 면)마다, 그 위 첫 층의 벽 선이 공중에 뽑히는 자리.

    벽 선은 단면 윤곽에서 walls.offsets()만큼 안으로 들어간 고리로 본다. 고리의 점이 아래층 단면에서 AIR_GAP보다
    멀면 공중이다. 구멍을 둘러싼 고리가 RING_AIR 넘게 공중이면 "ring", 바깥 고리가 통째로 공중이면 "island"다.
    양 끝이 붙은 공중 구간은, 그 천장에 다리 선이 들어갈 자리(벽 띠 안쪽)가 없으면 WALLS_ONLY_SPAN보다 길 때
    "span", 있으면 EDGE_REPORT보다 길 때 참고로 "edge"다. 같은 자리의 여러 벽 선은 하나로 묶는다.
    """
    pts, tris = _welded_mesh(shape)
    e1, e2, w = _frame(up)
    hz = pts @ w
    bed = float(hz.min())
    h = hz - bed
    uv = np.stack([pts @ e1, pts @ e2], axis=1)
    # 천장 높이: 아래를 보는 거의 수평인 삼각형 (overhangs의 flat과 같은 기준)
    tri = pts[tris]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    norm = np.linalg.norm(n, axis=1)
    ok = norm > 1e-12
    cos_down = -(n[ok] @ w) / norm[ok]
    centre_h = h[tris[ok]].mean(axis=1)
    ceilings = centre_h[(cos_down > 0.94) & (centre_h > 0.05)]
    layer = walls.layer
    # 천장 위 첫 층: 가운데 높이가 천장보다 높은 첫 층 (슬라이서는 층의 가운데 높이에서 자른다). 천장이 층의
    # 한가운데에 있으면 어느 층에 붙을지 정해지지 않아(mid_layer_levels가 따로 막는다) 두 층을 다 본다
    mids = sorted(
        {
            round((math.floor(c / layer - 0.5 + 1e-6) + 1.5) * layer, 6)
            for h0 in ceilings
            for c in (h0 - 0.03, h0 + 0.03)
        }
    )
    found: list[AirWall] = []
    for mid in mids:
        above = section(h, uv, tris, mid + 1e-4)
        below = section(h, uv, tris, mid - layer + 1e-4)
        support = below.buffer(AIR_GAP)
        over = above.difference(support)
        if over.area < 1e-3:
            continue
        height = mid - layer / 2
        # 벽 띠 안쪽(다리 선이 들어갈 자리)이 없는 천장 조각: 벽 선만으로 채워진다
        offsets = walls.offsets()
        core = above.buffer(-(offsets[-1] + walls.inner_w / 2), join_style="mitre", mitre_limit=3.0)
        walls_only = [p for p in _polygons(over) if p.intersection(core).area < 0.05]
        spots: list[tuple[AirKind, float, float, float]] = []  # (종류, u, v, 길이)
        for d in offsets:
            inner = above.buffer(-d, join_style="mitre", mitre_limit=3.0)
            for poly in _polygons(inner):
                rings = [(poly.exterior, False)] + [(r, True) for r in poly.interiors]
                for ring, is_hole in rings:
                    line = LineString(ring.coords)
                    if line.length < LOOP_MIN:
                        continue
                    air = line.difference(support)
                    if air.is_empty:
                        continue
                    frac = air.length / line.length
                    if is_hole and frac > RING_AIR:
                        c = ring.centroid
                        spots.append(("ring", c.x, c.y, line.length))
                    elif not is_hole and frac > 0.98:
                        c = ring.centroid
                        spots.append(("island", c.x, c.y, line.length))
                    else:
                        for piece in _lines(air):
                            if piece.length <= min(WALLS_ONLY_SPAN, EDGE_REPORT):
                                continue
                            m = piece.interpolate(0.5, normalized=True)
                            alone = any(p.distance(m) < walls.inner_w for p in walls_only)
                            if alone and piece.length > WALLS_ONLY_SPAN:
                                spots.append(("span", m.x, m.y, piece.length))
                            elif not alone and piece.length > EDGE_REPORT:
                                spots.append(("edge", m.x, m.y, piece.length))
        # 같은 종류가 3 mm 안에 모이면 한 자리로 묶는다 (벽 선마다 하나씩 나온다)
        groups: list[list[tuple[AirKind, float, float, float]]] = []
        for s in spots:
            for g in groups:
                if g[0][0] == s[0] and math.hypot(g[0][1] - s[1], g[0][2] - s[2]) < 3.0:
                    g.append(s)
                    break
            else:
                groups.append([s])
        for g in groups:
            kind = g[0][0]
            u = sum(s[1] for s in g) / len(g)
            v = sum(s[2] for s in g) / len(g)
            p = u * e1 + v * e2 + (height + bed) * w
            found.append(
                AirWall(
                    kind=kind,
                    height=round(height, 2),
                    where=(round(float(p[0]), 1), round(float(p[1]), 1), round(float(p[2]), 1)),
                    length=round(max(s[3] for s in g), 1),
                    loops=len(g),
                )
            )
    return tuple(found)


def air_wall_lines(printed: Mapping[str, tuple[Any, ...]]) -> list[str]:
    """부품 모듈의 printed() (형상, 위, 색, 프리셋, ...)에서 공중 벽 선을 찾아 보고할 줄로."""
    lines: list[str] = []
    for name, entry in printed.items():
        shape, up, _color, preset = entry[:4]
        lines += [f"  {name}: {a.describe()}" for a in air_walls(shape, up, wall_settings(preset))]
    head = "공중에 뽑히는 벽 선 (뽑기 전에 고친다. 참고는 막지 않는다)"
    return [f"{head}: 없음"] if not lines else [head, *lines]
