"""가짜 아래팔 끝: 300 mm 선 시험에만 쓰는 부품 (docs/status.md의 사용자 순서 1).

J4의 출력 원판과, 실제 팔에서 그 너머에 오는 아래팔 끝(J4 집, 목, 관의 첫머리)을 J4가 범위 가운데일 때의 자리로
합친 한 덩어리다. 손목 1의 발에 출력 원판 대신 단다. 20 cm 선 시험에서 남는 선의 고리가 발 쪽으로 처졌는데, 실제
팔에서는 그 자리를 J4 원판과 아래팔 끝이 막는다 (아래팔 끝의 앞면이 손목 1의 발 바닥에서 5.8 mm, J4가 범위
가운데면 아래팔 끝의 살과 멈춤 홈이 손목 1 앞벽의 가운데 홈 바로 앞에 온다).

선이 닿을 수 있는 겉은 지금 설계와 같은 모양과 자리다: 원판의 테와 귀, 멈춤 기둥, 혼 볼트 머리(parts/joint.py의
out_disc, inner_hardware), 아래팔 끝의 원통과 앞면, 멈춤 홈과 살, 영점 핀 구멍, 선 창, 관의 앞면과 둥근 윗부분
(parts/links.py의 link). 나머지는 뽑기 쉽게 바꿨다:
- 속은 채운다. 원판 테두리와 원통 앞 끝 사이 틈(0.8)과 그 안의 빈 곳도 메운다 (두께 1.5인 선이 들지 않는 틈이다).
- 관 밑(손목 1에서 먼 쪽)은 바닥까지 곧은 면으로 채운다.
- 멈춤 기둥과 귀 밑(손목 1의 뒷덮개 쪽, 선이 나오는 쪽의 맞은편)은 45° 받침으로 원통에 잇는다.
- 관은 X_CUT에서 자른다. 사용자는 시험 중에 이 관과 원통을 손잡이로 잡는다.
- 벽 나사의 머리는 없다 (나사 자리만 있다).
발 나사는 M3 × 12 넷이다. 너트는 옆 홈으로 밀어 넣고, 홈 천장 첫 층은 빗살 막이라 나사를 넣기 전에 뚫는다.

좌표는 J4 서보의 좌표다 (아래팔 끝 집이 품는 서보, parts/joint.py). 뽑을 때는 원판 윗면이 위(+y)이고 원통의 뒤
끝이 바닥이다. 손목 1의 서보 좌표로는 wrist.from_parent(flip=True)로 옮긴다.

    uv run python -m horilab_design.parts.fake_forearm
"""

from __future__ import annotations

import math

from build123d import Compound, Location, Plane, Polygon, Pos, RegularPolygon, extrude

from horilab_design.parts import joint as j
from horilab_design.parts import links, wrist
from horilab_design.parts.shapes import (
    COMB_PITCH,
    Point,
    box,
    comb_ceiling,
    cyl_x,
    cyl_y,
    nut_trap,
    solid,
)
from horilab_design.printcheck import (
    LAYER,
    air_wall_lines,
    mid_layer_levels,
    overhangs,
    printed_mass_g,
)
from horilab_design.values import ServoModel

UP: Point = (0.0, 1.0, 0.0)  # 뽑을 때 위: 원판 윗면 (바닥은 원통의 뒤 끝)
Y_BOT = j.BACK_END
TUBE_R = links.LINKS["forearm"][1]
YC = j.FRONT - (TUBE_R - links.EMBED)  # 관 가운데 (링크와 같다)
# 관을 자르는 x. 손목 1의 서보 좌표로는 y = AX_Y + 62(손목 2의 가운데쯤)까지 관의 앞면이 남는다
X_CUT = -62.0
SCREW = 12.0  # 발 나사 M3 × 12 (손목 1 발의 귀 안쪽에서 넣는다)
TIP = j.ATOP + j.EAR_T - SCREW  # 발 나사의 끝
NUT_W, NUT_H = j.K_NUT_LYING  # 너트가 누워 옆에서 밀려 들어가는 홈 (검정 시험편)
# 조인 너트가 붙는 홈 천장. 나사 끝이 너트를 0.3 넘게 지나가고, 바닥에서 잰 높이가 층 경계에 온다
ROOF = Y_BOT + math.ceil((TIP + 0.3 + j.NUT_T - Y_BOT) / LAYER - 1e-9) * LAYER
# 홈 입구 쪽 길이: 나사 자리에서 원판 테의 겉(반지름 R_DRUM)을 1.0 지나간다
SLOT_LEN = math.sqrt(j.R_DRUM**2 - j.RIM[0][0] ** 2) - abs(j.RIM[0][1]) + 1.0
# 나사 끝이 드는 홈 밑 구멍의 바닥: 나사 끝에서 1.5 넘게 밑이고 층 경계에 온다
HOLE_BOTTOM = Y_BOT + math.floor((TIP - 1.5 - Y_BOT) / LAYER + 1e-9) * LAYER
# 홈 천장 빗살 막의 틈: 홈을 가로질러(±x) 가까운 겉까지 터서, 틈 둘레의 벽 선이 공중에서 고리로 돌지 않는다
# (그리퍼 돋움의 너트 홈과 같다). 맨 안쪽 틈(나사에서 빗살 간격 둘)이 원판 테의 겉을 1.0 지나는 길이다
COMB_FRONT = (
    math.sqrt(j.R_DRUM**2 - (abs(j.RIM[0][1]) - 2 * COMB_PITCH) ** 2) - abs(j.RIM[0][0]) + 1.0
)
# 멈춤 기둥과 귀 밑의 45° 받침: (x, y) 다각형을 귀의 폭(±EAR_HW)만큼 뽑아 귀의 윤곽으로 자른다. 아래 모서리가
# 기둥의 바깥 밑을 지나고, 위는 귀 속으로 0.5 들어간다 (얇게 겹쳐 합치면 형상 계산이 깨진다)
WEDGE: list[tuple[float, float]] = [
    (23.0, 19.4),
    (32.0, 28.4),
    (32.0, j.SEAM + 0.5),
    (23.0, j.SEAM + 0.5),
]


def _prism_z(pts: list[tuple[float, float]], z0: float, z1: float) -> Compound:
    """(x, y) 다각형을 z0 ~ z1로 뽑은 기둥 (다각형은 반시계 방향)."""
    return solid(Pos(0, 0, z0) * extrude(Polygon(*pts, align=None), amount=z1 - z0))


def foot_nuts() -> list[tuple[str, Point, Point, Point]]:
    """발 나사의 너트 넷: (이름, 조인 너트의 가운데, 볼트 방향, 입구 쪽 방향). 입구는 가까운 옆(±z)을 본다."""
    out: list[tuple[str, Point, Point, Point]] = []
    for k, (x, z) in enumerate(j.RIM, start=1):
        s = 1.0 if z > 0 else -1.0
        out.append((f"발 나사 너트 {k}", (x, ROOF - j.NUT_T / 2, z), UP, (0.0, 0.0, s)))
    return out


def fake_forearm() -> Compound:
    """가짜 아래팔 끝 (검정). J4 서보 좌표, J4 각 0."""
    # 아래팔 끝: 링크(links.link)의 집, 목, 관과 같은 겉. 속은 채우고 관 밑은 바닥까지 채운다. 원통은 원판의
    # 테까지 한 원통이다 (j.drum_shell의 원통에 원판의 테와 메운 틈을 더한 것. 반지름이 같은 두 원통을 겹쳐
    # 합치면 형상 계산이 깨진다)
    tube = cyl_x(TUBE_R, X_CUT, links.NECK_END + 4, YC, 0.0)
    tube &= box(X_CUT, 0.0, Y_BOT, j.FRONT, -TUBE_R - 1, TUBE_R + 1)
    under = box(X_CUT, links.NECK_END + 4, Y_BOT, YC, -TUBE_R, TUBE_R)
    neck = box(links.NECK_END, -12.0, Y_BOT, j.FRONT, -j.NECK_HW, j.NECK_HW)
    h = solid(cyl_y(j.R_DRUM, Y_BOT, j.ATOP) + neck + tube + under)
    h = j.cut_joint_features(h, window=True)
    # 출력 원판의 귀와 멈춤 기둥 (out_disc와 같은 겉), 그 밑의 받침
    ear = box(20.0, j.R_POST, j.SEAM, j.ATOP, -j.EAR_HW, j.EAR_HW)
    ear += cyl_y(j.EAR_HW, j.SEAM, j.ATOP, j.R_POST, 0.0)
    ear += cyl_y(j.BOSS_R, j.POST_TIP, j.SEAM + 0.5, j.R_POST, 0.0)
    outline = box(20.0, j.R_POST, 19.0, j.SEAM + 0.5, -j.EAR_HW, j.EAR_HW)
    outline += cyl_y(j.EAR_HW, 19.0, j.SEAM + 0.5, j.R_POST, 0.0)
    wedge = _prism_z(WEDGE, -j.EAR_HW, j.EAR_HW) & outline
    h = solid(h + solid(ear) + solid(wedge))
    # 발 나사: 누운 너트의 옆 홈, 홈 천장의 빗살 막 (막 위로만 구멍), 나사 끝이 드는 홈 밑 구멍
    r = j.K_M3_FREE / 2
    for _name, (x, _y, z), axis, slide in foot_nuts():
        h = solid(h - nut_trap((x, ROOF - NUT_H / 2, z), axis, slide, SLOT_LEN, NUT_W, NUT_H))
        h = solid(h - cyl_y(r, ROOF + LAYER, j.ATOP + 1, x, z))
        h = solid(h - cyl_y(r, HOLE_BOTTOM, ROOF - NUT_H + 0.01, x, z))
        along = (1.0 if x > 0 else -1.0, 0.0, 0.0)  # 가까운 겉 쪽
        back = NUT_W / 2 + 0.5  # 맞은쪽 옆 벽 속으로 0.5
        h = solid(h - comb_ceiling((x, ROOF, z), UP, along, 2 * SLOT_LEN, back, COMB_FRONT))
    # 혼 볼트 머리 (M3 둥근머리, 실제 팔에서 원판 윗면에 앉는 자리와 크기): 손목 1 속에서 선이 그 옆을 지난다
    for x, z in j.bolt_xz():
        h = solid(h + cyl_y(2.85, j.ATOP - 0.5, j.ATOP + 1.65, x, z))
    if not h.is_valid:
        raise RuntimeError("가짜 아래팔 끝의 형상이 깨졌다")
    return h


def in_wrist1(p: Point) -> Point:
    """이 부품 좌표(J4 서보 좌표)의 점을 손목 1의 서보 좌표로 (발에 단 자리)."""
    v = (wrist.from_parent(flip=True) * Pos(*p)).position
    return (round(v.X, 2), round(v.Y, 2), round(v.Z, 2))


def hole_tops() -> list[Point]:
    """발 나사 구멍 넷의 입구 (원판 면, 손목 1의 서보 좌표). 작업 안내가 막을 뚫는 자리로 가리킨다."""
    return [in_wrist1((x, j.ATOP, z)) for x, z in j.RIM]


def _nut(center: Point, axis: Point, slide: Point) -> Compound:
    """M3 너트 (맞변 5.5, 두께 2.4)."""
    body = extrude(RegularPolygon(5.5 / 2, 6, major_radius=False), amount=j.NUT_T / 2, both=True)
    return solid(body.moved(Location(Plane(origin=center, x_dir=slide, z_dir=axis))))


def placed(model: ServoModel = "STS3215") -> list[j.Part]:
    """손목 1의 발에 단 가짜 아래팔 끝과 너트 넷, 발 나사(M3 × 12) 넷 (손목 1의 서보 좌표)."""
    del model  # 서보 모델과 관계없다 (부르는 쪽 모양을 맞춘다)
    loc = wrist.from_parent(flip=True)
    parts = [j.Part("가짜 아래팔 끝", solid(loc * fake_forearm()), color="cap")]
    for name, c, a, s in foot_nuts():
        parts.append(j.Part(name, solid(loc * _nut(c, a, s)), j.STEEL, False, "metal"))
    x = wrist.X_FOOT + j.EAR_T
    for k, (dy, z) in enumerate(j.RIM, start=1):
        s = cyl_x(1.5, x - SCREW, x, wrist.AX_Y + dy, z) + cyl_x(
            2.85, x, x + 1.65, wrist.AX_Y + dy, z
        )
        parts.append(j.Part(f"발 나사 M3x12 {k}", solid(s), j.STEEL, False, "metal"))
    return parts


LABELS = {"fake_forearm": "가짜 아래팔 끝"}

PRINT_SPEC = {"fake_forearm": (fake_forearm, UP, "black", "strength")}


def printed() -> dict[str, tuple[Compound, tuple[float, float, float], str, str]]:
    """뽑는 부품: 이름 → (형상, 뽑을 때 위를 보는 방향, 색, 프리셋)."""
    return {
        key: (make(), up, color, preset) for key, (make, up, color, preset) in PRINT_SPEC.items()
    }


def report(model: ServoModel = "STS3215") -> None:
    shape = fake_forearm()
    bb = shape.bounding_box()
    over = overhangs(shape, UP)
    print(
        f"가짜 아래팔 끝 (검정): {bb.size.X:.1f} × {bb.size.Y:.1f} × {bb.size.Z:.1f} mm, "
        f"{printed_mass_g(shape, UP):.1f} g (어림)"
    )
    print(
        f"  걸침 {over.total:.0f} mm² (거의 수평 {over.flat:.0f}, 가장 넓은 한 자리 {over.largest:.0f})"
    )
    for a, hgt, u, v, flat in over.spots:
        print(
            f"    높이 {hgt:5.1f}  자리 ({u:6.1f}, {v:6.1f})  넓이 {a:6.1f}  {'수평' if flat else '비스듬'}"
        )
    mid = mid_layer_levels(shape, UP)
    print(f"  층 가운데에 걸친 수평면: {mid or '없음'}")
    print("\n".join(air_wall_lines(printed())))
    print(
        f"  발 나사 M3 × {SCREW:.0f}: 조인 너트 {ROOF - j.NUT_T:.2f} ~ {ROOF:.2f}, 나사 끝 {TIP:.2f} "
        f"(너트를 {ROOF - j.NUT_T - TIP:.2f} 지나간다), 너트 위 살 {j.ATOP - ROOF:.2f}, "
        f"구멍 바닥과 나사 끝 사이 {TIP - HOLE_BOTTOM:.2f}"
    )
    entry = {n: j.nut_entry_overlap(shape, c, a, s) for n, c, a, s in foot_nuts()}
    print(
        "  옆 홈에 너트를 곧게 밀어 넣을 때 겹침 [mm³]: "
        + (", ".join(f"{n} {v:.2f}" for n, v in entry.items() if v > 0.01) or "없음")
    )
    # 지금 설계와 같은지: 아래팔 끝은 이 부품이 다 덮고, 원판에서 빠진 것은 너트 홈 자리뿐이어야 한다
    stub = solid(
        links.link(*links.LINKS["forearm"]) & box(X_CUT, 40.0, Y_BOT - 1, j.FRONT + 1, -40, 40)
    )
    tips = None  # 발 나사 끝이 드는 구멍 (원통 앞 끝의 턱에 걸친다. 원판에 덮여 겉에서 안 보인다)
    for x, z in j.RIM:
        one = cyl_y(j.K_M3_FREE / 2 + 0.01, HOLE_BOTTOM - 0.01, j.FRONT + 0.01, x, z)
        tips = one if tips is None else tips + one
    miss = solid(stub - shape - tips).volume
    print(f"  지금 설계의 아래팔 끝 중 이 부품에 없는 부피 (발 나사 끝 구멍 밖): {miss:.2f} mm³")
    print(
        f"  지금 설계의 출력 원판 중 이 부품에 없는 부피: {solid(j.out_disc() - shape).volume:.1f} mm³ "
        "(발 나사 너트 홈과 빗살)"
    )
    # 조립: 손목 1에 달았을 때 겹침과, J5를 돌릴 때 손목 2와 J5 원판까지의 틈
    mine = placed(model)
    hits = j.overlaps([p for p in wrist.first_joint(model) if "핀" not in p.name] + mine[:1])
    print(
        "  손목 1에 달았을 때 겹침 [mm³]: "
        + (", ".join(f"{a} ∩ {b} {v:.2f}" for (a, b), v in hits.items()) or "없음")
    )
    body = mine[0].shape
    screws = [p for p in mine if p.name.startswith("발 나사 M3")]
    print(
        "  발 나사와 이 부품이 겹치는 것은 홈 천장의 빗살 막뿐이다 (나사를 넣기 전에 뚫는다) [mm³]: "
        + ", ".join(f"{solid(body & p.shape).volume:.2f}" for p in screws)
    )
    child = wrist.wrist_body()
    disc = j.out_disc()
    gaps = []
    for q in (-j.STOP_DEG, -90.0, 0.0, 90.0, j.STOP_DEG):
        d_child = body.distance_to(wrist.as_child(child, q))
        d_disc = body.distance_to(j.turned(disc, q))
        gaps.append(f"{q:.0f}° 손목 2 {d_child:.1f}, J5 원판 {d_disc:.1f}")
    print("  J5를 돌릴 때 틈 [mm]: " + "; ".join(gaps))


if __name__ == "__main__":
    report()
