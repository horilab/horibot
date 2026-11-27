"""작업 안내 사이트 (0036): 형상 파일, 안내 정의의 검사와 셈, 만든 사이트, 브라우저 자체 점검."""

from __future__ import annotations

import hashlib
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from build123d import Box, Compound, Plane, Pos, Solid, Vector

from horibot_design.guide import cables, selftest
from horibot_design.guide.build import (
    GuideError,
    guide_json,
    plate_refs,
    print_json,
    resolve,
    write_site,
)
from horibot_design.guide.guides import (
    base_pi,
    d405_tie,
    first_joint,
    forearm,
    j1_base,
    j1_zero,
    j2_shoulder,
    prints,
    upper_arm,
    wrist_end,
    wrist_join,
)
from horibot_design.guide.meshes import pack, tessellate, unpack
from horibot_design.guide.model import Guide, Move, PrintGuide, ScenePart, Step, View
from horibot_design.parts import base as bp
from horibot_design.parts import camera_mount as cmt
from horibot_design.parts import joint as j
from horibot_design.parts import links, wrist
from horibot_design.parts.shapes import solid
from horibot_design.studies import wiring as wr

UP = View((0, 1, 0), (0, 0, 1))


def test_mesh_pack_round_trips_within_half_a_step() -> None:
    shape = solid(Pos(3.2, -40.0, 7.0) * Box(30.0, 12.0, 6.0))
    points, triangles = tessellate(shape)
    mesh = pack(points, triangles)
    back, indices = unpack(mesh.to_json())
    assert len(back) == len(points) and len(indices) == 3 * len(triangles)
    worst = max(
        abs(b[a] + mesh.origin[a] - p[a])
        for b, p in zip(back, points, strict=True)
        for a in range(3)
    )
    assert worst <= mesh.q / 2 + 1e-9
    assert mesh.q == pytest.approx(0.01)


def test_moved_copies_share_one_mesh_file() -> None:
    a = pack(*tessellate(solid(Box(5.0, 5.0, 2.4))))
    b = pack(*tessellate(solid(Pos(17.3, 12.0, -4.0) * Box(5.0, 5.0, 2.4))))
    turned = pack(*tessellate(solid(Box(2.4, 5.0, 5.0))))
    assert a.id == b.id and a.origin != b.origin
    assert turned.id != a.id


def _tiny_guide(steps: tuple[Step, ...]) -> Guide:
    def scene() -> list[ScenePart]:
        return [
            ScenePart("집", solid(Box(40.0, 40.0, 40.0)), "wrist_body", "body"),
            ScenePart("너트 1", solid(Pos(0, 30, 0) * Box(5.0, 2.4, 5.0)), "nut_m3", "metal"),
            ScenePart("너트 2", solid(Pos(0, -30, 0) * Box(5.0, 2.4, 5.0)), "nut_m3", "metal"),
        ]

    return Guide(
        "tiny", "assembly", "작은 안내", "시험", scene, steps, items=first_joint.GUIDE.items
    )


def _work(sid: str, adds: tuple[str, ...] = (), moves: tuple[Move, ...] = ()) -> Step:
    return Step(sid, sid, sid, ("한다",), check=("됐다",), view=UP, adds=adds, moves=moves)


def test_resolve_counts_once_and_rejects_bad_steps() -> None:
    good = _tiny_guide(
        (
            _work("house", adds=("집",)),
            _work("nuts", adds=("너트",), moves=(Move(("너트 1",), (0.0, 10.0, 0.0)),)),
        )
    )
    r = resolve(good)
    assert r.totals == {"wrist_body": 1, "nut_m3": 2}
    assert r.steps[1]["hot"] == [1, 2] and r.steps[1]["show"] == [0, 1, 2]
    with pytest.raises(GuideError, match="맞는 장면 부품이 없다"):
        resolve(_tiny_guide((_work("house", adds=("집", "볼트")),)))
    with pytest.raises(GuideError, match="이미 손에 든"):
        resolve(_tiny_guide((_work("a", adds=("집", "너트")), _work("b", adds=("너트 1",)))))
    with pytest.raises(GuideError, match="본체에 붙지 않는"):
        resolve(_tiny_guide((_work("house", adds=("집",)),)))
    with pytest.raises(GuideError, match="보이지 않는다"):
        resolve(_tiny_guide((_work("a", adds=("집",), moves=(Move(("너트 1",), (0, 1, 0)),)),)))
    with pytest.raises(GuideError, match="확인할 것"):
        resolve(_tiny_guide((Step("a", "a", "a", ("한다",), view=UP, adds=("집", "너트")),)))


def _sweep_guide(
    by: tuple[float, float, float], color: str = "cap", plate_y: float = 10.0
) -> Guide:
    """판(두께 2, 가운데 y = plate_y)이 있는 데로 말뚝(제자리 y -2 ~ 2)을 by만큼 떨어진 데서 넣는 안내."""

    def scene() -> list[ScenePart]:
        return [
            ScenePart("판", solid(Pos(0, plate_y, 0) * Box(40.0, 2.0, 40.0)), "wrist_body", "body"),
            ScenePart("말뚝", solid(Box(4.0, 4.0, 4.0)), "out_disc", color),
        ]

    steps = (
        _work("plate", adds=("판",)),
        _work("peg", adds=("말뚝",), moves=(Move(("말뚝",), by),)),
    )
    return Guide("sweep", "assembly", "길", "시험", scene, steps, items=first_joint.GUIDE.items)


def test_sweep_finds_a_part_that_hits_on_its_way_in() -> None:
    from horibot_design.guide import sweep

    hits = sweep.check(resolve(_sweep_guide((0.0, 30.0, 0.0))))  # 위에서 판을 뚫고 내려온다
    assert [(h.part, h.other) for h in hits] == [("말뚝", "판")]
    # 말뚝 밑면(제자리 y -2)이 판 윗면(y 11)을 지나기 시작하는 곳: 제자리에서 13 mm 앞, 재는 간격 안
    assert 13.0 - sweep.STEP_MM <= hits[0].before_mm < 13.0
    # 밑에서 오면 판에 닿지 않는다
    assert sweep.check(resolve(_sweep_guide((0.0, -30.0, 0.0)))) == []
    # 판(y 2 ~ 4)이 제자리 바로 위면 마지막 6 mm에서 겹친다: 플러그만 단자 속으로 들어가는 길이로 보고 뺀다
    assert len(sweep.check(resolve(_sweep_guide((0.0, 12.0, 0.0), plate_y=3.0)))) == 1
    assert sweep.check(resolve(_sweep_guide((0.0, 12.0, 0.0), "plug", plate_y=3.0))) == []


def _posed_guide(joint2: float, peg_turns: bool) -> Guide:
    """둘째 관절(원점을 지나는 z축)이 있는 안내. 말뚝은 by (0, 30, 0)로 위에서 들어온다.

    peg_turns가 거짓이면 판(가운데 y 10)이 둘째 관절과 같이 돌고 말뚝은 서 있다. 참이면 말뚝이 같이 돌고 판(가운데
    x 10)은 서 있다: 움직임 방향도 말뚝과 같이 돈다 (화면은 도는 관절 묶음의 좌표로 부품을 옮긴다)."""

    def scene() -> list[ScenePart]:
        if peg_turns:
            plate = ScenePart(
                "판", solid(Pos(10, 0, 0) * Box(2.0, 10.0, 10.0)), "wrist_body", "body"
            )
            peg = ScenePart(
                "말뚝", solid(Box(4.0, 4.0, 4.0)), "out_disc", "cap", True, moving2=True
            )
        else:
            plate = ScenePart(
                "판",
                solid(Pos(0, 10, 0) * Box(10.0, 2.0, 10.0)),
                "wrist_body",
                "body",
                True,
                moving2=True,
            )
            peg = ScenePart("말뚝", solid(Box(4.0, 4.0, 4.0)), "out_disc", "cap")
        return [plate, peg]

    peg_step = replace(
        _work("peg", adds=("말뚝",), moves=(Move(("말뚝",), (0.0, 30.0, 0.0)),)), joint2=joint2
    )
    steps = (_work("plate", adds=("판",)), peg_step)
    return Guide(
        "posed",
        "assembly",
        "길",
        "시험",
        scene,
        steps,
        items=first_joint.GUIDE.items,
        axis2=((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
    )


def test_sweep_poses_the_joints_of_the_step() -> None:
    from horibot_design.guide import sweep

    # 판이 관절과 같이 돈다: 관절 각 0이면 말뚝 길을 막고, 90°면 비켜 있다
    assert len(sweep.check(resolve(_posed_guide(0.0, peg_turns=False)))) == 1
    assert sweep.check(resolve(_posed_guide(90.0, peg_turns=False))) == []
    # 말뚝이 관절과 같이 돈다: 90°면 움직임 방향(위에서)이 +x 쪽에서로 돌아 서 있는 판(x 10)을 지난다
    assert sweep.check(resolve(_posed_guide(0.0, peg_turns=True))) == []
    assert len(sweep.check(resolve(_posed_guide(90.0, peg_turns=True)))) == 1


HEAD_TOP = 1.65  # 시험 나사의 머리 윗면 높이 (y)


def _reach_guide(roof: float | None = None, wall: bool = False, hint: float | None = None) -> Guide:
    """밑판(윗면 y 0)에 M3 × 10을 위에서(by (0, 14, 0)) 조이는 안내.

    roof를 주면 머리 위 그 높이에 지붕(축을 덮는다)이, wall이면 렌치 긴 팔 높이(머리 위 8 ~ 16 mm)의 +x 쪽(x 4 ~ 24)에
    벽이 있다. hint를 주면 그 방향(reach.l_key의 각, 0°가 +x)으로 L자 렌치를 그려 넣는다."""
    from horibot_design.guide import reach

    def scene() -> list[ScenePart]:
        up = Vector(0, 1, 0)
        shank = Solid.make_cylinder(1.5, 10.0, Plane(origin=(0, -10, 0), z_dir=(0, 1, 0)))
        head = Solid.make_cylinder(2.85, HEAD_TOP, Plane(origin=(0, 0, 0), z_dir=(0, 1, 0)))
        parts = [
            ScenePart("밑판", solid(Pos(0, -5, 0) * Box(40.0, 10.0, 40.0)), "wrist_body", "body"),
            ScenePart("나사", Compound([head.fuse(shank)]), "m3x10", "metal"),
        ]
        if roof is not None:
            box = Pos(0, HEAD_TOP + roof + 1.0, 0) * Box(20.0, 2.0, 20.0)
            parts.append(ScenePart("지붕", solid(box), "out_disc", "cap"))
        if wall:
            box = Pos(14, HEAD_TOP + 12.0, 0) * Box(20.0, 8.0, 20.0)
            parts.append(ScenePart("벽", solid(box), "carrier", "cap"))
        if hint is not None:
            key = reach.l_key(Vector(0, HEAD_TOP, 0), up, hint)
            parts.append(ScenePart("렌치", Compound([key]), "hexkey", "metal", count=False))
        return parts

    base = ("밑판", *(("지붕",) if roof is not None else ()), *(("벽",) if wall else ()))
    steps = (
        _work("base", adds=base),
        _work(
            "screw",
            adds=("나사", *(("렌치",) if hint is not None else ())),
            moves=(Move(("나사",), (0.0, 14.0, 0.0)),),
        ),
    )
    return Guide("reach", "assembly", "렌치", "시험", scene, steps, items=first_joint.GUIDE.items)


def test_reach_finds_screws_a_straight_key_cannot_reach() -> None:
    from horibot_design.guide import reach

    assert reach.check(resolve(_reach_guide())) == []  # 머리 위가 비었다
    # 머리 위 9 mm에 지붕: 곧은 렌치도, L자 렌치의 짧은 팔(12 mm)도 막힌다
    shut = resolve(_reach_guide(roof=9.0))
    [x] = reach.reaches(shut)
    assert x.blocked_mm == pytest.approx(9.0, abs=0.3) and x.turns == ()
    assert "들어가지 않는다" in reach.check(shut, [x])[0].text
    # 지붕이 높으면(16 mm) 짧은 팔이 들어가고, 벽이 없으면 긴 팔이 어느 쪽으로나 돈다: 그림이 없어도 된다
    assert reach.check(resolve(_reach_guide(roof=16.0))) == []
    # 한쪽에 벽이 있으면 일부 방향으로만 돈다: 렌치를 넣는 방향을 그려야 한다
    side = resolve(_reach_guide(roof=16.0, wall=True))
    [y] = reach.reaches(side)
    assert 0 < len(y.turns) < 360 // reach.TURN_STEP and 0 not in y.turns and 180 in y.turns
    assert "그려 보인다" in reach.check(side, [y])[0].text
    assert reach.check(resolve(_reach_guide(roof=16.0, wall=True, hint=180.0))) == []
    [p] = reach.check(resolve(_reach_guide(roof=16.0, wall=True, hint=0.0)))
    assert "겹친다" in p.text


def test_servo_without_id_is_rejected() -> None:
    def scene() -> list[ScenePart]:
        return [ScenePart("서보", solid(Box(20.0, 20.0, 20.0)), "servo_sts3215", "servo")]

    guide = Guide("noid", "assembly", "서보", "시험", scene, (_work("s", adds=("서보",)),))
    with pytest.raises(GuideError, match="ID가 없다"):
        resolve(guide)
    named = Guide(
        "named",
        "assembly",
        "서보",
        "시험",
        scene,
        (_work("s", adds=("서보",)),),
        items={"servo_sts3215": first_joint.SERVO_ITEM},
    )
    assert resolve(named).totals == {"servo_sts3215": 1}


@pytest.mark.slow
def test_first_joint_guide_counts_match_the_parts_list() -> None:
    r = resolve(first_joint.GUIDE)
    assert r.totals == {
        "wrist_body": 2,
        "bearing_6806": 1,
        "retainer": 1,
        "nut_m3": 10,
        "m3x8": 5,
        "out_disc": 1,
        "m3x10": 5,
        "carrier": 1,
        "servo_sts3215": 1,
        "selftap_so101": 4,
        "washer_m3": 1,
        "hub_plate": 1,
        "m3x20": 4,
        "servo_cable": 1,
        "wrist_cover": 1,
        "m3x12": 2,
        "pin_msh3_20": 1,
    }
    # 서보는 어느 서보인지(ID) 이름에 보인다
    assert "ID 5" in first_joint.GUIDE.items["servo_sts3215"].name
    kinds = [s["kind"] for s in r.steps]
    assert kinds[0] == "prep" and kinds[-1] == "end" and kinds.count("work") == 19
    # 옆 홈의 너트는 설계 코드의 입구 쪽에서 들어온다
    slots = {name: slide for name, _c, _a, slide in j.side_nut_slots()}
    nut_moves = [m for s in r.steps if s["id"] == "retainer-nuts" for m in s["moves"]]
    assert {tuple(m["by"]) for m in nut_moves} == {
        tuple(13.0 * v for v in slots["누름 틀 +z"]),
        tuple(13.0 * v for v in slots["누름 틀 +x"]),
    }


def test_second_joint_parts_turn_inside_the_first() -> None:
    def scene() -> list[ScenePart]:
        return [
            ScenePart("집", solid(Box(40.0, 40.0, 40.0)), "wrist_body", "body"),
            ScenePart(
                "자식", solid(Pos(0, 50, 0) * Box(30.0, 30.0, 30.0)), "wrist_body", "body", True
            ),
            ScenePart(
                "도구",
                solid(Pos(30, 50, 0) * Box(10.0, 5.0, 5.0)),
                "nut_m3",
                "metal",
                True,
                moving2=True,
            ),
        ]

    steps = (
        _work("all", adds=("집", "자식", "도구")),
        replace(_work("turn"), turn2=(-90.0, 90.0), uses=()),
    )
    axis = ((0.0, 50.0, 0.0), (1.0, 0.0, 0.0))
    items = first_joint.GUIDE.items
    guide = Guide("two", "assembly", "둘째 관절", "시험", scene, steps, items=items, axis2=axis)
    data = guide_json(resolve(guide), {})
    assert [row["m"] for row in data["scene"]] == [False, True, 2]
    assert data["axis2"] == {"o": [0.0, 50.0, 0.0], "d": [1.0, 0.0, 0.0]}
    assert data["steps"][1]["turn2"] == [-90.0, 90.0] and data["steps"][0]["turn2"] is None
    with pytest.raises(GuideError, match="axis2가 없"):
        resolve(replace(guide, axis2=None))


@pytest.mark.slow
def test_wrist_end_guide_counts_match_the_parts_list() -> None:
    prints_ = [prints.FIRST_JOINT, prints.WRIST_END]
    r = resolve(wrist_end.GUIDE, prints_)
    # 손목 끝 판의 열세 개와 뽑아 둔 서보 받침 하나 (조립해 둔 첫 관절의 부품은 세지 않는다)
    printed = {k: n for k, n in r.totals.items() if r.items[k].kind == "printed"}
    assert printed == {
        "carrier": 1,
        "retainer": 1,
        "flange_disc": 1,
        "hub_plate": 1,
        "tool_adapter": 1,
        "d405_mount": 1,
        "gripper_frame": 1,
        "gripper_floor": 1,
        "gripper_pinion": 1,
        "gripper_finger": 2,
        "gripper_tub": 1,
        "gripper_slider": 2,
    }
    hardware = {k: n for k, n in r.totals.items() if r.items[k].kind == "hardware"}
    assert hardware == {
        "nut_m3": 16,
        "washer_m3": 1,
        "m3x6": 6,
        "m3x8": 9,
        "m3x10": 3,
        "m3x12": 4,
        "m3x20": 4,
    }
    assert r.totals["pin_tool"] == 2 and r.totals["pin_msh3_20"] == 1
    # 서보는 ID가 이름에 있고, 그리퍼 서보는 ID 11이다
    assert "ID 11" in wrist_end.GUIDE.items[wrist_end.GRIP_ITEM.key].name
    kinds = [s["kind"] for s in r.steps]
    assert kinds[0] == "prep" and kinds[-1] == "end" and kinds.count("work") == 56
    # 이미 뽑아 둔 서보 받침 말고는 출력 부품이 모두 손목 끝 판에서 나온다
    assert all(r.items[k].file in r.plates for k in printed if k != "carrier")


def test_servo_cable_wraps_outside_the_rim_with_its_ends_held() -> None:
    rod = cables.servo_rod()
    # 손목 2로 들어가는 자리가 살에서 156° 쪽이라 -163°에서 가장 길게 감긴다. 선은 그 길이에 맞춘다 (0031)
    assert cables.tight_length(rod, cables.LONG_STOP) > cables.tight_length(rod, -cables.LONG_STOP)
    assert rod.length == pytest.approx(cables.longest(rod) + cables.SPARE)
    # 20 cm 선은 손목 1 속과 층을 지나고도 손목 2 안으로 들어간다 (끝을 테이프로 붙일 몫)
    assert 10.0 < cables.tail_mm(rod) < 30.0
    angles = cables.wrap_angles(40.0)
    frames = cables.solve(rod, angles)
    assert len(frames) == len(angles)
    for a, p in zip(angles, frames, strict=True):
        assert tuple(p[0]) == pytest.approx(rod.start)
        assert tuple(p[-1]) == pytest.approx(tuple(cables.rot(np.array(rod.end), a)))
        assert np.linalg.norm(p, axis=1).min() >= rod.radius - 1e-6  # 테 안으로 들어가지 않는다
        length = np.linalg.norm(np.diff(p, axis=0), axis=1).sum()
        assert rod.length - 0.5 < length < rod.length * 1.05  # 어림 풀이라 조금 늘어난다


def test_flex_needs_matching_frames() -> None:
    fx = cables.flex(angles=cables.wrap_angles(80.0))
    good = Guide(
        "flex",
        "measure",
        "선",
        "시험",
        lambda: [ScenePart("선", solid(Box(10.0, 10.0, 10.0)), "servo_cable", "wire", flex=fx)],
        (_work("lay", adds=("선",)),),
    )
    data = guide_json(resolve(good), {})
    assert data["meshes"] == [] and len(data["scene"][0]["flex"]["p"]) == len(fx.angles)
    bad = replace(fx, frames=fx.frames[:-1])
    broken = replace(
        good,
        scene=lambda: [
            ScenePart("선", solid(Box(10.0, 10.0, 10.0)), "servo_cable", "wire", flex=bad)
        ],
    )
    with pytest.raises(GuideError, match="꺾은선"):
        guide_json(resolve(broken), {})


def _plan(tmp: Path, bundle: str, plates: list[list[tuple[str, int]]]) -> None:
    folder = tmp / bundle
    folder.mkdir(parents=True)
    rows = []
    for k, parts in enumerate(plates, start=1):
        name = f"{k:02d}-black-abc12{k}"
        (folder / f"{name}.png").write_bytes(b"png")
        rows.append(
            {
                "file": f"{name}.3mf",
                "image": f"{name}.png",
                "color": "black",
                "preset": "0.20mm Strength @BBL X1C",
                "minutes": 70 * k,
                "grams": 10.0 * k,
                "overnight": False,
                "parts": [
                    {
                        "file": f,
                        "label": f"부품 {f}",
                        "count": c,
                        "height_mm": 9.0,
                        "slender": False,
                    }
                    for f, c in parts
                ],
            }
        )
    plan = {"name": bundle, "made": "2026-11-16T10:00:00", "bed_type": "Textured PEI Plate"}
    (folder / "plan.json").write_text(json.dumps({**plan, "plates": rows}), encoding="utf-8")


def test_print_guide_has_a_step_per_plate_and_numbers_reprints_after_them(tmp_path: Path) -> None:
    _plan(tmp_path, "demo", [[("a.stl", 2), ("b.stl", 1)], [("c.stl", 1)]])
    _plan(tmp_path, "demo-r1", [[("a.stl", 1)]])
    pg = PrintGuide("demo-print", "데모 출력", "시험", ("demo", "demo-r1"))
    data = print_json(pg, tmp_path)
    steps = data["steps"]
    assert [s["id"] for s in steps] == ["demo-1", "demo-2", "demo-r1-1"]
    assert [s["no"] for s in steps] == ["1", "2", "3"] and {s["kind"] for s in steps} == {"plate"}
    assert (
        steps[0]["check"] == []
        and steps[0]["plate"]["image"] == "data/prints/demo/01-black-abc121.png"
    )
    assert any("01-black-abc121.3mf" in t for t in steps[0]["do"])
    assert any("끝 6자리가 abc121" in t for t in steps[0]["do"])  # 프린터 화면과 견줄 파일 이름 끝
    assert data["summary"] == {"plates": 3, "minutes": 70 + 140 + 70, "grams": 40.0}
    # 조립 안내의 출력 부품은 처음 판과 다시 뽑기 판을 모두 안다 (뽑았는지는 화면이 db의 기록으로 셈한다)
    refs = plate_refs("demo", [pg], tmp_path)
    assert refs["a.stl"] == [["demo-print", "demo-1", 1, 2], ["demo-print", "demo-r1-1", 3, 1]]
    with pytest.raises(GuideError, match="판 계획이 없다"):
        print_json(PrintGuide("x-print", "x", "x", ("missing",)), tmp_path)


@pytest.mark.slow
def test_built_site_has_every_file_its_guides_need(tmp_path: Path) -> None:
    summary = write_site(tmp_path)
    page = (tmp_path / "index.html").read_text(encoding="utf-8")
    assert "/*@CSS@*/" not in page and "/*@JS@*/" not in page and "<title>작업 안내</title>" in page
    catalog = json.loads((tmp_path / "data" / "catalog.json").read_text(encoding="utf-8"))
    assert [g["id"] for g in catalog["guides"]] == [
        d405_tie.GUIDE.id,
        base_pi.GUIDE.id,
        j1_zero.GUIDE.id,
        j2_shoulder.GUIDE.id,
        upper_arm.GUIDE.id,
        forearm.GUIDE.id,
        wrist_join.GUIDE.id,
        prints.ARM.id,
        j1_base.GUIDE.id,
    ]
    for row in catalog["guides"]:
        guide = json.loads((tmp_path / row["file"]).read_text(encoding="utf-8"))
        # 처음 화면의 진행은 확인 칸(판은 Claude의 기록)으로 셈한다: 작업 단계와 판은 모두 track에 든다
        assert sum(1 for t in row["track"] if t[1] not in ("준비", "끝")) == row["work"]
        for step in guide["steps"]:
            if step["kind"] == "plate" and step["plate"]["image"]:
                assert (tmp_path / step["plate"]["image"]).exists()
        assert {s["mesh"] for s in guide["scene"] if "mesh" in s} == set(guide["meshes"])
        assert all(("mesh" in s) != ("flex" in s) for s in guide["scene"])
        for mesh in guide["meshes"]:
            assert (tmp_path / "data" / "meshes" / f"{mesh}.json").exists()
        for step in guide["steps"]:
            assert all(row[0] in guide["items"] for row in step["kit"]), step["id"]
    shoulder = json.loads((tmp_path / "data/guides/j2-shoulder.json").read_text(encoding="utf-8"))
    printed = [it for it in shoulder["items"].values() if it["kind"] == "printed"]
    assert printed and all(it["plates"][0][0] == prints.ARM.id for it in printed)
    disc = next(it for it in printed if it["file"] == "shoulder_out_disc.stl")
    assert disc["plates"] == [[prints.ARM.id, "arm-black-1-1", 3, 1]]  # 흰 판 둘 다음의 검정 판 1
    # 출력 안내: 판의 부품마다 3D로 볼 형상이 있다 (지금 설계 코드로 뽑는 방향에 놓아 만든다)
    pr = json.loads((tmp_path / "data/guides/arm-print.json").read_text(encoding="utf-8"))
    parts = [part for s in pr["steps"] for part in s["plate"]["parts"]]
    assert len(pr["scene"]) == len({p["file"] for p in parts}) > 0
    assert all(p["p"] is not None for p in parts)
    manifest = json.loads((tmp_path / "manifest.json").read_text(encoding="utf-8"))
    assert len(manifest) == summary.files
    for rel, digest in manifest.items():
        assert hashlib.sha256((tmp_path / rel).read_bytes()).hexdigest() == digest


@pytest.mark.slow
def test_site_passes_its_selftest_in_a_headless_browser(tmp_path: Path) -> None:
    if selftest.find_browser() is None:
        pytest.skip("크롬이나 엣지가 없다")
    write_site(tmp_path)
    report = selftest.run(tmp_path)
    if not report["three"]:
        pytest.skip("three.js를 받지 못했다 (인터넷)")
    assert selftest.problems(report) == []
    plates = len(print_json(prints.ARM)["steps"])
    guides = (d405_tie, base_pi, j1_zero, j2_shoulder, upper_arm, forearm, wrist_join, j1_base)
    steps = sum(len(g.STEPS) for g in guides) + plates
    assert sum(g["steps"] for g in report["guides"]) == steps


def test_d405_tie_lengths_and_spots() -> None:
    # 테이프의 바깥 끝은 설계에서 가정한 잡는 자리와 같은 반지름이고, 높이는 카메라 밑면이다
    assert d405_tie.hold_radius() == pytest.approx(cmt.HOLD_R, abs=2.0)
    assert d405_tie.HOLD_H == pytest.approx(-cmt.CAM_H / 2 - cmt.WIRE_D / 2 - wrist.X_FOOT)
    # 테이프는 밑면 안에 든다 (렌즈 면으로 넘어가지 않는다)
    assert -cmt.CAM_W / 2 <= d405_tie.TAPE_Y[0] < d405_tie.TAPE_Y[1] <= cmt.CAM_W / 2
    assert -cmt.CAM_D <= d405_tie.TAPE_Z[0] < d405_tie.TAPE_Z[1] <= 0.0
    # 고리는 설계 값(테이프 자리로 다시 센 J4·J5)보다 짧지 않고 5 mm 단위다
    loops = d405_tie.loop_mm()
    hold = wr.d405_wrist_loop(d405_tie.hold_radius(), d405_tie.HOLD_H).length
    assert loops["J4·J5"] >= hold and loops["J4·J5"] - hold < 5.0
    assert loops["J3"] >= wr.d405_elbow_loop().length
    assert loops["J1·J2"] >= wr.d405_base_loop().length
    assert all(v % 5 == 0 for v in loops.values())
    # 찍찍이 도막은 관을 한 바퀴 감고 3 cm 넘게 겹친다
    for name in ("forearm", "upper_arm"):
        assert d405_tie.TIE_CM[name] * 10.0 - links._tie_section(name).length >= 30.0
    # 파이 판 타이: 구멍(폭 3 mm)의 몸통 쪽 벽에서 판 가장자리까지 살 4 mm를 감는다
    t = d405_tie.base_tie()
    assert t["z1"] - t["z0"] == pytest.approx(4.0, abs=0.1)
    assert t["x1"] - t["x0"] == pytest.approx(links.TIE_W)
    # 남는 선 뭉치(책상과 나란한 면에 감는다)는 책상 뒤 가장자리에서 30 mm 넘게 뒤, 책상 윗면 밑에 매달린다
    cx, cy, _cz = d405_tie.COIL_AT
    assert cx < -bp.EDGE - 30.0
    assert cy + d405_tie.COIL_R < bp.Y_DESK
