"""작업 안내 사이트 (0036): 형상 파일, 안내 정의의 검사와 셈, 만든 사이트, 브라우저 자체 점검."""

from __future__ import annotations

import hashlib
import json
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from build123d import Box, Pos

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
    first_joint,
    j1_base,
    prints,
    st3120_horn,
    wrist_end,
)
from horibot_design.guide.meshes import pack, tessellate, unpack
from horibot_design.guide.model import Guide, Move, PrintGuide, ScenePart, Step, View
from horibot_design.parts import joint as j
from horibot_design.parts.shapes import solid

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
        prints.ARM.id,
        j1_base.GUIDE.id,
        prints.ST3120_COUPON.id,
        st3120_horn.GUIDE.id,
        "first-joint",
        prints.FIRST_JOINT.id,
        prints.WRIST_END.id,
        wrist_end.GUIDE.id,
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
    joint = json.loads((tmp_path / "data/guides/first-joint.json").read_text(encoding="utf-8"))
    printed = [it for it in joint["items"].values() if it["kind"] == "printed"]
    assert printed and all(it["plates"][0][0] == prints.FIRST_JOINT.id for it in printed)
    body = next(it for it in printed if it["file"] == "wrist_body.stl")
    assert body["plates"] == [[prints.FIRST_JOINT.id, "first-joint-3", 3, 2]]
    # 출력 안내: 판의 부품마다 3D로 볼 형상이 있다 (지금 설계 코드로 뽑는 방향에 놓아 만든다)
    pr = json.loads((tmp_path / "data/guides/first-joint-print.json").read_text(encoding="utf-8"))
    parts = [part for s in pr["steps"] for part in s["plate"]["parts"]]
    assert len(pr["scene"]) == len({p["file"] for p in parts}) == 10
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
    plates = sum(
        len(print_json(p)["steps"])
        for p in (prints.ARM, prints.ST3120_COUPON, prints.FIRST_JOINT, prints.WRIST_END)
    )
    guides = (j1_base, st3120_horn, first_joint, wrist_end)
    steps = sum(len(g.STEPS) for g in guides) + plates
    assert sum(g["steps"] for g in report["guides"]) == steps
