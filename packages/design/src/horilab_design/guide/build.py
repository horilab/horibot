"""안내를 검사하고 사이트 파일을 쓴다 (0036의 데이터 모델).

    build/guide/index.html              화면 코드 전부 (web/의 파일을 이어 붙인다)
    build/guide/data/catalog.json       안내 목록
    build/guide/data/guides/<id>.json   안내 하나 (부품 목록, 장면, 단계, 준비물)
    build/guide/data/meshes/<id>.json   형상 하나 (meshes.py)
    build/guide/manifest.json           파일마다 sha256 (바뀐 파일만 다시 올린다)

단계가 가리키는 부품은 만들 때 장면의 번호로 바꾼다. 보이는 부품, 이번 부품, 쓰는 것과 준비물의 개수는 여기서
셈한다 (손으로 적지 않는다).
"""

from __future__ import annotations

import hashlib
import json
import re
import shutil
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

from horilab_design.guide import catalog
from horilab_design.guide.meshes import Mesh, pack, tessellate
from horilab_design.guide.model import Guide, Item, ScenePart, Step, Vec, View
from horilab_design.values import REPO_ROOT

WEB = Path(__file__).resolve().parent / "web"
OUT = REPO_ROOT / "build" / "guide"
PRINT_DIR = REPO_ROOT / "build" / "print"
ID = re.compile(r"^[a-z0-9]+(-[a-z0-9]+)*$")
KIND_ORDER = ("printed", "bought", "hardware", "tool")
STEP_KINDS = ("prep", "work", "end")
GUIDE_KINDS = ("assembly", "print", "measure", "wiring", "view")


class GuideError(ValueError):
    """안내 정의가 장면이나 부품 목록과 맞지 않는다."""


Box = tuple[Vec, Vec]


@dataclass
class Resolved:
    guide: Guide
    parts: list[ScenePart]
    boxes: list[Box]
    items: dict[str, Item]
    steps: list[dict[str, Any]]
    board: list[dict[str, Any]]
    totals: dict[str, int]
    plates: dict[str, tuple[int, str]]
    warnings: list[str] = field(default_factory=list)


def _r(v: Sequence[float], nd: int = 3) -> list[float]:
    return [round(float(x), nd) for x in v]


def _view(v: View | None) -> dict[str, Any] | None:
    if v is None:
        return None
    return {"up": _r(v.up), "face": _r(v.face), "el": v.el, "az": v.az}


class _Picker:
    """선택자(model.py)를 장면 번호로 바꾼다."""

    def __init__(self, guide: Guide, parts: list[ScenePart], boxes: list[Box]) -> None:
        self.guide, self.parts, self.boxes = guide, parts, boxes

    def one(self, sel: str, where: str) -> list[int]:
        at = sel.find("@")
        raw, cond = (sel, "") if at < 0 else (sel[:at], sel[at + 1 :])
        exact = raw.endswith("$")
        name = raw[:-1] if exact else raw
        out: list[int] = []
        for i, p in enumerate(self.parts):
            if (p.name != name) if exact else (not p.name.startswith(name)):
                continue
            if cond:
                if len(cond) != 2 or cond[0] not in "xyz" or cond[1] not in "+-":
                    raise GuideError(f"{self.guide.id} {where}: 선택자 '{sel}'의 조건을 모른다")
                axis = "xyz".index(cond[0])
                c = (self.boxes[i][0][axis] + self.boxes[i][1][axis]) / 2
                if (c <= 5) if cond[1] == "+" else (c >= -5):
                    continue
            out.append(i)
        if not out:
            raise GuideError(f"{self.guide.id} {where}: '{sel}'에 맞는 장면 부품이 없다")
        return out

    def many(self, sels: Sequence[str], where: str) -> list[int]:
        seen: list[int] = []
        for s in sels:
            for i in self.one(s, where):
                if i not in seen:
                    seen.append(i)
        return seen

    def single(self, sel: str, where: str) -> int:
        found = self.one(sel, where)
        if len(found) != 1:
            raise GuideError(
                f"{self.guide.id} {where}: '{sel}'가 부품 {len(found)}개를 고른다 (하나여야 한다)"
            )
        return found[0]


def _plates(bundle: str) -> dict[str, tuple[int, str]]:
    """출력 묶음의 plan.json에서 STL 파일 이름 → (판 번호, 3MF 이름). 묶음이 없으면 빈 것."""
    path = PRINT_DIR / bundle / "plan.json" if bundle else None
    if path is None or not path.exists():
        return {}
    plan = json.loads(path.read_text(encoding="utf-8"))
    out: dict[str, tuple[int, str]] = {}
    for k, plate in enumerate(plan["plates"], start=1):
        for part in plate["parts"]:
            out.setdefault(str(part["file"]), (k, str(plate["file"])))
    return out


def _check_step_text(gid: str, step: Step) -> None:
    where = f"{gid}/{step.id}"
    if not ID.match(step.id):
        raise GuideError(f"{where}: 단계 id는 소문자, 숫자, 하이픈만 쓴다")
    if step.kind not in STEP_KINDS:
        raise GuideError(f"{where}: 단계 종류 '{step.kind}'를 모른다")
    if not step.title or not step.short or not step.do:
        raise GuideError(f"{where}: 제목, 짧은 이름, 할 일이 있어야 한다")
    if step.kind == "work":
        if not step.check:
            raise GuideError(f"{where}: 작업 단계에는 다 하면 확인할 것이 있어야 한다")
        if step.view is None:
            raise GuideError(f"{where}: 작업 단계에는 손에 드는 방향(view)이 있어야 한다")
        if not (step.adds or step.moves or step.turn or step.uses or step.spots):
            raise GuideError(f"{where}: 이 단계에서 하는 일(나타나는 부품, 움직임, 쓰는 것)이 없다")
    if step.kind == "end" and step.view is None:
        raise GuideError(f"{where}: 끝 단계에도 손에 드는 방향(view)이 있어야 한다")


def resolve(guide: Guide) -> Resolved:
    """장면을 만들고(형상은 삼각형으로 바꾸지 않는다) 단계를 장면 번호로 바꿔 검사한다."""
    if not ID.match(guide.id):
        raise GuideError(f"안내 id '{guide.id}'는 소문자, 숫자, 하이픈만 쓴다")
    if guide.kind not in GUIDE_KINDS:
        raise GuideError(f"{guide.id}: 안내 종류 '{guide.kind}'를 모른다")
    parts = guide.scene()
    boxes: list[Box] = []
    for p in parts:
        bb = p.shape.bounding_box()
        boxes.append(((bb.min.X, bb.min.Y, bb.min.Z), (bb.max.X, bb.max.Y, bb.max.Z)))
    items = {**catalog.ITEMS, **guide.items}
    for p in parts:
        if p.key not in items:
            raise GuideError(f"{guide.id}: 장면 부품 '{p.name}'의 키 '{p.key}'가 부품 목록에 없다")
    plates = _plates(guide.bundle)
    warnings: list[str] = []
    used_printed = {p.key for p in parts if items[p.key].kind == "printed"}
    if guide.bundle and not plates:
        warnings.append(f"출력 묶음 '{guide.bundle}'의 plan.json이 없어 판 번호를 넣지 않았다")
    for key in sorted(used_printed):
        item = items[key]
        if plates:
            if item.file not in plates:
                raise GuideError(
                    f"{guide.id}: 출력 부품 '{item.file}'가 묶음 '{guide.bundle}'에 없다"
                )
            k, _f = plates[item.file]
            items[key] = Item(
                item.key, item.name, item.kind, f"{item.meta} · 판 {k}", item.file, item.color,
                item.draw, item.icon, item.thumb,
            )  # fmt: skip

    pick = _Picker(guide, parts, boxes)
    ids: set[str] = set()
    handled: set[int] = set()
    mounted: set[int] = set()
    steps: list[dict[str, Any]] = []
    totals: Counter[str] = Counter()
    where_used: dict[str, list[str]] = {}
    first_seen: dict[str, int] = {}
    tools: list[str] = []
    work_no = 0
    for si, step in enumerate(guide.steps):
        _check_step_text(guide.id, step)
        where = f"{guide.id}/{step.id}"
        if step.id in ids:
            raise GuideError(f"{where}: 단계 id가 겹친다")
        ids.add(step.id)
        if step.kind == "prep" and si != 0:
            raise GuideError(f"{where}: 준비물 단계는 맨 앞에만 둔다")
        if step.kind == "end" and si != len(guide.steps) - 1:
            raise GuideError(f"{where}: 끝 단계는 맨 뒤에만 둔다")
        if step.kind == "work":
            work_no += 1
        no = "준비" if step.kind == "prep" else "끝" if step.kind == "end" else str(work_no)

        adds = pick.many(step.adds, where)
        still = set(pick.many(step.still, where))
        if not still <= set(adds):
            raise GuideError(f"{where}: still은 adds 가운데서 고른다")
        again = [parts[i].name for i in adds if i in handled]
        if again:
            raise GuideError(f"{where}: 앞 단계에서 이미 손에 든 부품이다: {again}")
        handled.update(adds)
        focus = pick.many(step.focus, where)
        mount_sels = step.mount if step.mount is not None else (() if step.focus else step.adds)
        mount = pick.many(mount_sels, where)
        for i in mount:
            if i not in handled:
                raise GuideError(f"{where}: '{parts[i].name}'는 손에 들기 전에 본체에 붙는다")
            if i in mounted:
                raise GuideError(f"{where}: '{parts[i].name}'는 이미 본체에 붙어 있다")
        mounted.update(mount)
        hide = set(pick.many(step.hide, where))
        show = [i for i in (focus if focus else sorted(mounted)) if i not in hide]
        shown = set(show)

        moves = []
        moving: set[int] = set()
        for mv in step.moves:
            idx = pick.many(mv.parts, where)
            lost = [parts[i].name for i in idx if i not in shown]
            if lost:
                raise GuideError(f"{where}: 움직이는 부품이 보이지 않는다: {lost}")
            moving.update(idx)
            moves.append({"p": idx, "by": _r(mv.by), "delay": mv.delay})
        hot = sorted((set(adds) - still) | moving)
        if not set(hot) <= shown:
            raise GuideError(f"{where}: 이번 부품이 보이지 않는다")
        shell = pick.many(step.shell, where)
        if any(i not in shown for i in shell):
            raise GuideError(f"{where}: 자르거나 반투명하게 할 몸통이 보이지 않는다")
        places = []
        for pl in step.place:
            i = pick.single(pl.part, where)
            if i not in shown:
                raise GuideError(f"{where}: 따로 놓는 부품 '{pl.part}'가 보이지 않는다")
            places.append({"p": i, "pos": _r(pl.pos), "up": _r(pl.up), "face": _r(pl.face)})
        labels = []
        for lb in step.labels:
            i = pick.one(lb.part, where)[0]
            if i not in shown:
                raise GuideError(f"{where}: 이름표의 부품 '{lb.part}'가 보이지 않는다")
            labels.append({"p": i, "text": lb.text, "at": _r(lb.at) if lb.at else None})
        spots = []
        for sp in step.spots:
            i = pick.one(sp.part, where)[0]
            if i not in shown:
                raise GuideError(f"{where}: 손볼 자리의 부품 '{sp.part}'가 보이지 않는다")
            spots.append({"p": i, "label": sp.label, "at": [_r(a) for a in sp.at]})

        # 이번에 쓰는 것: 처음 손에 드는 부품을 키로 세고, 쓰는 것(공구, 묶음)을 더한다
        counted: Counter[str] = Counter(parts[i].key for i in adds if parts[i].count)
        kit: list[list[Any]] = [[key, n, ""] for key, n in counted.items()]
        for use in step.uses:
            if use.key not in items:
                raise GuideError(f"{where}: 쓰는 것의 키 '{use.key}'가 부품 목록에 없다")
            kit.append([use.key, use.qty, use.note])
            if use.count and use.qty:
                counted[use.key] += use.qty
            if items[use.key].kind == "tool" and use.key not in tools:
                tools.append(use.key)
        kit.sort(key=lambda row: KIND_ORDER.index(items[row[0]].kind))
        for key, n in counted.items():
            totals[key] += n
            first_seen.setdefault(key, si)
            where_used.setdefault(key, []).append(f"{no}단계 ×{n}")

        steps.append(
            {
                "id": step.id,
                "no": no,
                "kind": step.kind,
                "short": step.short,
                "title": step.title,
                "do": list(step.do),
                "warn": list(step.warn),
                "check": list(step.check),
                "stuck": step.stuck,
                "show": show,
                "hot": hot,
                "shell": shell,
                "cut": step.cut,
                "ghost": step.ghost,
                "moves": moves,
                "place": places,
                "view": _view(step.view),
                "labels": labels,
                "tags": [{"at": _r(t.at), "text": t.text} for t in step.tags],
                "spots": spots,
                "kit": kit,
                "joint": step.joint,
                "turn": list(step.turn) if step.turn else None,
            }
        )
    left = [parts[i].name for i in range(len(parts)) if i not in mounted]
    if left:
        raise GuideError(f"{guide.id}: 어느 단계에서도 본체에 붙지 않는 부품이 있다: {left}")

    board: list[dict[str, Any]] = []
    for kind in KIND_ORDER:
        keys = [k for k in items if items[k].kind == kind and (totals[k] or k in tools)]
        keys.sort(key=lambda k: (first_seen.get(k, 10**6), tools.index(k) if k in tools else 0))
        if not keys:
            continue
        group: dict[str, Any] = {"kind": kind, "title": catalog.KINDS[kind], "items": []}
        if kind == "printed" and guide.bundle:
            group["copy"] = f"build/print/{guide.bundle}/"
        for k in keys:
            note = " · ".join(where_used.get(k, [])) if kind != "tool" else ""
            group["items"].append([k, totals[k], note])
        board.append(group)
    return Resolved(guide, parts, boxes, items, steps, board, dict(totals), plates, warnings)


def _item_json(item: Item, parts: list[ScenePart]) -> dict[str, Any]:
    out: dict[str, Any] = {"name": item.name, "kind": item.kind, "meta": item.meta}
    if item.file:
        out["file"] = item.file
    if item.color:
        out["color"] = item.color
    if item.draw:
        out["draw"] = item.draw
    if item.icon:
        out["icon"] = item.icon
    if item.thumb is not None:
        main = next((i for i, p in enumerate(parts) if p.key == item.key and p.count), None)
        if main is not None:
            extra = [i for i, p in enumerate(parts) if p.key == item.key and not p.count]
            out["thumb"] = {"p": [main, *extra], **(_view(item.thumb) or {})}
    return out


def guide_json(r: Resolved, meshes: dict[str, Mesh]) -> dict[str, Any]:
    """안내 하나의 JSON. 형상을 삼각형으로 바꿔 meshes에 모은다."""
    scene = []
    for p in r.parts:
        m = pack(*tessellate(p.shape))
        meshes.setdefault(m.id, m)
        scene.append(
            {
                "n": p.name,
                "k": p.key,
                "mesh": m.id,
                "t": _r(m.origin, 4),
                "c": p.color,
                "m": p.moving,
            }
        )
    used = {row[0] for s in r.steps for row in s["kit"]} | {
        row[0] for g in r.board for row in g["items"]
    }
    g = r.guide
    return {
        "id": g.id,
        "kind": g.kind,
        "title": g.title,
        "sub": g.sub,
        "bundle": f"build/print/{g.bundle}/" if g.bundle else "",
        "items": {k: _item_json(r.items[k], r.parts) for k in sorted(used)},
        "scene": scene,
        "meshes": sorted({row["mesh"] for row in scene}),
        "steps": r.steps,
        "board": r.board,
        "work": sum(1 for s in r.steps if s["kind"] == "work"),
    }


def page() -> str:
    """web/의 틀에 CSS와 JS를 넣은 index.html."""
    html = (WEB / "index.html").read_text(encoding="utf-8")
    css = (WEB / "style.css").read_text(encoding="utf-8")
    js = (WEB / "app.js").read_text(encoding="utf-8")
    if "</script" in js.lower() or "</style" in css.lower():
        raise GuideError("화면 코드에 </script>나 </style>이 들어 있다")
    if html.count("/*@CSS@*/") != 1 or html.count("/*@JS@*/") != 1:
        raise GuideError("index.html 틀에 CSS와 JS 자리가 하나씩 있어야 한다")
    return html.replace("/*@CSS@*/", css).replace("/*@JS@*/", js)


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8", newline="\n")


def _dump(data: Any) -> str:
    return json.dumps(data, ensure_ascii=False, separators=(",", ":"))


@dataclass
class Summary:
    out: Path
    guides: list[str]
    meshes: int
    files: int
    bytes: int
    warnings: list[str]


def write_site(out: Path = OUT, guides: Sequence[Guide] | None = None) -> Summary:
    """모든 안내를 검사하고 out에 사이트를 쓴다. data/는 새로 쓴다 (지운 형상이 남지 않게)."""
    if guides is None:
        from horilab_design.guide.guides import ALL

        guides = ALL
    if (out / "data").exists():
        shutil.rmtree(out / "data")
    meshes: dict[str, Mesh] = {}
    rows = []
    warnings: list[str] = []
    for g in guides:
        r = resolve(g)
        warnings += [f"{g.id}: {w}" for w in r.warnings]
        data = guide_json(r, meshes)
        _write(out / "data" / "guides" / f"{g.id}.json", _dump(data))
        rows.append(
            {
                "id": g.id,
                "kind": g.kind,
                "title": g.title,
                "sub": g.sub,
                "work": data["work"],
                # 처음 화면이 안내를 열지 않고 진행을 셈한다: 확인 칸이 있는 단계마다 [id, 번호, 짧은 이름, 칸 수]
                "track": [
                    [s["id"], s["no"], s["short"], len(s["check"])]
                    for s in data["steps"]
                    if s["check"]
                ],
                "file": f"data/guides/{g.id}.json",
            }
        )
    for m in meshes.values():
        _write(out / "data" / "meshes" / f"{m.id}.json", _dump(m.to_json()))
    made = datetime.now().astimezone().isoformat(timespec="seconds")
    _write(out / "data" / "catalog.json", _dump({"made": made, "guides": rows}))
    _write(out / "index.html", page())
    files = sorted(p for p in out.rglob("*") if p.is_file() and p.name != "manifest.json")
    manifest = {
        p.relative_to(out).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in files
    }
    _write(out / "manifest.json", json.dumps(manifest, indent=1))
    size = sum(p.stat().st_size for p in files)
    return Summary(out, [g.id for g in guides], len(meshes), len(files), size, warnings)
