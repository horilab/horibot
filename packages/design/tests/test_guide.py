"""작업 안내 사이트 (0036): 형상 파일, 안내 정의의 검사와 셈, 만든 사이트, 브라우저 자체 점검."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
from build123d import Box, Pos

from horilab_design.guide import selftest
from horilab_design.guide.build import GuideError, resolve, write_site
from horilab_design.guide.guides import first_joint
from horilab_design.guide.meshes import pack, tessellate, unpack
from horilab_design.guide.model import Guide, Move, ScenePart, Step, View
from horilab_design.parts import joint as j
from horilab_design.parts.shapes import solid

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
        "d405_cable": 1,
        "pin_msh3_20": 1,
    }
    kinds = [s["kind"] for s in r.steps]
    assert kinds[0] == "prep" and kinds[-1] == "end" and kinds.count("work") == 19
    # 옆 홈의 너트는 설계 코드의 입구 쪽에서 들어온다
    slots = {name: slide for name, _c, _a, slide in j.side_nut_slots()}
    nut_moves = [m for s in r.steps if s["id"] == "retainer-nuts" for m in s["moves"]]
    assert {tuple(m["by"]) for m in nut_moves} == {
        tuple(13.0 * v for v in slots["누름 틀 +z"]),
        tuple(13.0 * v for v in slots["누름 틀 +x"]),
    }


@pytest.mark.slow
def test_built_site_has_every_file_its_guides_need(tmp_path: Path) -> None:
    summary = write_site(tmp_path)
    page = (tmp_path / "index.html").read_text(encoding="utf-8")
    assert "/*@CSS@*/" not in page and "/*@JS@*/" not in page and "<title>작업 안내</title>" in page
    catalog = json.loads((tmp_path / "data" / "catalog.json").read_text(encoding="utf-8"))
    assert [g["id"] for g in catalog["guides"]] == ["first-joint"]
    for row in catalog["guides"]:
        guide = json.loads((tmp_path / row["file"]).read_text(encoding="utf-8"))
        # 처음 화면의 진행은 확인 칸으로 셈한다: 작업 단계는 모두 확인 칸이 있어 track에 든다
        assert sum(1 for t in row["track"] if t[1] not in ("준비", "끝")) == row["work"]
        assert {s["mesh"] for s in guide["scene"]} == set(guide["meshes"])
        for mesh in guide["meshes"]:
            assert (tmp_path / "data" / "meshes" / f"{mesh}.json").exists()
        for step in guide["steps"]:
            assert all(row[0] in guide["items"] for row in step["kit"]), step["id"]
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
    assert sum(g["steps"] for g in report["guides"]) == len(first_joint.STEPS)
