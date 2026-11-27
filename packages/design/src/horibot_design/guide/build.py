"""안내를 검사하고 사이트 파일을 쓴다 (0036의 데이터 모델).

    build/guide/index.html              화면 코드 전부 (web/의 파일을 이어 붙인다)
    build/guide/data/catalog.json       안내 목록
    build/guide/data/guides/<id>.json   안내 하나 (부품 목록, 장면, 단계, 준비물)
    build/guide/data/meshes/<id>.json   형상 하나 (meshes.py)
    build/guide/data/prints/<묶음>/     판 그림 (data/prints/<묶음>/에서 옮긴다)
    build/guide/manifest.json           파일마다 sha256 (바뀐 파일만 다시 올린다)

단계가 가리키는 부품은 만들 때 장면의 번호로 바꾼다. 보이는 부품, 이번 부품, 쓰는 것과 준비물의 개수는 여기서
셈한다 (손으로 적지 않는다). 출력 안내는 출력 준비 도구가 낸 판 계획(data/prints/<묶음>/plan.json)에서 판마다
단계를 만들고, 조립 안내의 출력 부품마다 어느 출력 안내의 어느 판에서 나오는지를 잇는다.
"""

from __future__ import annotations

import hashlib
import importlib
import itertools
import json
import re
import shutil
from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any

from horibot_design.guide import catalog
from horibot_design.guide.meshes import Mesh, pack, tessellate
from horibot_design.guide.model import Flex, Guide, Item, PrintGuide, ScenePart, Step, Vec, View
from horibot_design.slicing import PLANS_DIR, oriented
from horibot_design.values import REPO_ROOT

WEB = Path(__file__).resolve().parent / "web"
OUT = REPO_ROOT / "build" / "guide"
ID = re.compile(r"^[a-z0-9]+(-[a-z0-9]+)*$")
KIND_ORDER = ("printed", "bought", "hardware", "tool")
STEP_KINDS = ("prep", "work", "end")
BED_KO = {"Textured PEI Plate": "텍스처 PEI"}
PLATE_STUCK = "첫 층이 안 붙거나, 뽑는 중에 부품이 떨어지거나, 다 뽑은 부품이 이상하면 사진을 찍어 대화창에 보내 주세요."
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
    plates: dict[str, list[list[Any]]]  # STL 파일 이름 → [출력 안내 id, 판 단계 id, 판 번호, 개수]
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


def _hm(minutes: int) -> str:
    return f"{minutes // 60}시간 {minutes % 60}분" if minutes >= 60 else f"{minutes}분"


def read_plan(bundle: str, plans: Path = PLANS_DIR) -> dict[str, Any]:
    """출력 준비 도구가 낸 판 계획. 없으면 GuideError (출력 안내가 빠진 채로 사이트를 올리지 않게)."""
    path = plans / bundle / "plan.json"
    if not path.exists():
        raise GuideError(f"출력 묶음 '{bundle}'의 판 계획이 없다: {path}")
    plan: dict[str, Any] = json.loads(path.read_text(encoding="utf-8"))
    return plan


@dataclass(frozen=True)
class Plate:
    guide: str  # 출력 안내 id
    id: str  # 판 단계 id (<묶음>-<묶음 안의 차례>)
    no: int  # 출력 안내 안의 판 번호 (묶음을 이어 센다)
    bundle: str
    data: dict[str, Any]  # plan.json의 판 하나


def plates_of(pg: PrintGuide, plans: Path = PLANS_DIR) -> list[Plate]:
    out: list[Plate] = []
    for bundle in pg.bundles:
        for k, plate in enumerate(read_plan(bundle, plans)["plates"], start=1):
            sid = f"{bundle}-{k}"
            if not ID.match(sid):
                raise GuideError(f"{pg.id}: 묶음 이름 '{bundle}'은 소문자, 숫자, 하이픈만 쓴다")
            out.append(Plate(pg.id, sid, len(out) + 1, bundle, plate))
    return out


def plate_refs(
    bundle: str, prints: Sequence[PrintGuide], plans: Path = PLANS_DIR
) -> dict[str, list[list[Any]]]:
    """bundle을 담은 출력 안내들에서 STL 파일 이름 → [출력 안내 id, 판 단계 id, 판 번호, 개수]."""
    out: dict[str, list[list[Any]]] = {}
    for pg in prints:
        if bundle not in pg.bundles:
            continue
        for plate in plates_of(pg, plans):
            for part in plate.data["parts"]:
                out.setdefault(str(part["file"]), []).append(
                    [pg.id, plate.id, plate.no, int(part["count"])]
                )
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
        if not (step.adds or step.moves or step.turn or step.turn2 or step.uses or step.spots):
            raise GuideError(f"{where}: 이 단계에서 하는 일(나타나는 부품, 움직임, 쓰는 것)이 없다")
    if step.kind == "end" and step.view is None:
        raise GuideError(f"{where}: 끝 단계에도 손에 드는 방향(view)이 있어야 한다")


def resolve(guide: Guide, prints: Sequence[PrintGuide] = (), plans: Path = PLANS_DIR) -> Resolved:
    """장면을 만들고(형상은 삼각형으로 바꾸지 않는다) 단계를 장면 번호로 바꿔 검사한다. prints는 출력 부품의 판을
    찾을 출력 안내다."""
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
    # 서보는 어느 서보인지(ID)를 이름에 쓴다 (서보마다 ID 이름표가 붙어 있다). 서보 선은 서보가 아니다
    keys = {p.key for p in parts} | {u.key for s in guide.steps for u in s.uses}
    for key in sorted(keys):
        it = items.get(key)
        if (
            it is not None
            and key.startswith("servo_")
            and it.icon != "wire"
            and "ID" not in it.name
        ):
            raise GuideError(
                f"{guide.id}: 서보 '{it.name}'에 ID가 없다. 안내의 items에서 이름에 ID를 쓴다"
            )
    # 둘째 관절과 같이 도는 부품은 그 축이 있어야 하고, 첫째 관절과도 같이 돈다 (화면에서 그 안에 든다)
    for p in parts:
        if p.moving2 and (guide.axis2 is None or not p.moving):
            raise GuideError(
                f"{guide.id}: '{p.name}'는 둘째 관절과 같이 도는데 axis2가 없거나 moving이 거짓이다"
            )
    if guide.axis2 is None and any(s.joint2 or s.turn2 for s in guide.steps):
        raise GuideError(f"{guide.id}: 둘째 관절을 돌리는 단계가 있는데 axis2가 없다")
    plates = plate_refs(guide.bundle, prints, plans) if guide.bundle else {}
    warnings: list[str] = []
    # 이 안내에서 새로 쓰는 출력 부품(개수에 드는 것)만 묶음에서 찾는다. 이미 조립해 둔 부품은 다른 묶음에서 뽑았다
    used_printed = {p.key for p in parts if items[p.key].kind == "printed" and p.count}
    if guide.bundle and not plates:
        warnings.append(f"출력 묶음 '{guide.bundle}'을 담은 출력 안내가 없어 판을 잇지 않았다")
    for key in sorted(used_printed):
        if plates and items[key].file not in plates and key not in guide.printed_before:
            raise GuideError(
                f"{guide.id}: 출력 부품 '{items[key].file}'가 묶음 '{guide.bundle}'의 판에 없다"
            )

    pick = _Picker(guide, parts, boxes)
    ids: set[str] = set()
    handled: set[int] = set()
    mounted: set[int] = set()
    ever_mounted: set[int] = set()  # 떼어 낸 것도 한 번 붙었으면 센다
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
        ever_mounted.update(mount)
        hide = set(pick.many(step.hide, where))
        show = [i for i in (focus if focus else sorted(mounted)) if i not in hide]
        shown = set(show)
        removed = pick.many(step.removes, where)
        lost = [parts[i].name for i in removed if i not in mounted]
        if lost:
            raise GuideError(f"{where}: 붙지 않은 부품을 뗀다: {lost}")
        mounted.difference_update(removed)

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
                "joint2": step.joint2,
                "turn2": list(step.turn2) if step.turn2 else None,
                "zoom": step.zoom,
                "center": _r(step.center) if step.center else None,
            }
        )
    left = [parts[i].name for i in range(len(parts)) if i not in ever_mounted]
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


def _item_json(
    item: Item, parts: list[ScenePart], plates: dict[str, list[list[Any]]]
) -> dict[str, Any]:
    out: dict[str, Any] = {"name": item.name, "kind": item.kind, "meta": item.meta}
    if item.file:
        out["file"] = item.file
    if item.kind == "printed" and item.file in plates:
        out["plates"] = plates[item.file]
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


def _flex_json(name: str, f: Flex) -> dict[str, Any]:
    """관절 각에 따라 바뀌는 선 (model.Flex). 화면이 관절 각마다 이 꺾은선으로 납작한 선을 다시 그린다."""
    n = len(f.frames[0]) if f.frames else 0
    if (
        len(f.angles) < 2
        or len(f.frames) != len(f.angles)
        or n < 2
        or any(len(fr) != n for fr in f.frames)
        or any(b <= a for a, b in itertools.pairwise(f.angles))
        or not 0.0 <= f.flat_mm < f.edge_mm
    ):
        raise GuideError(f"'{name}': 관절 각과 꺾은선의 수, 차례, 눕고 세우는 길이가 맞지 않다")
    xs = [x for fr in f.frames for x, _ in fr]
    zs = [z for fr in f.frames for _, z in fr]
    ys = (f.y_flat, f.y_edge)
    pad = max(f.width, f.thick) / 2
    return {
        "a": [round(a, 2) for a in f.angles],
        "p": [[round(v, 2) for pt in fr for v in pt] for fr in f.frames],
        "w": f.width,
        "t": f.thick,
        "y": [round(y, 3) for y in ys],
        "s": [f.flat_mm, f.edge_mm],
        "box": [
            _r((min(xs) - pad, min(ys) - pad, min(zs) - pad)),
            _r((max(xs) + pad, max(ys) + pad, max(zs) + pad)),
        ],
    }


def guide_json(r: Resolved, meshes: dict[str, Mesh]) -> dict[str, Any]:
    """안내 하나의 JSON. 형상을 삼각형으로 바꿔 meshes에 모은다. 관절 각에 따라 바뀌는 선(flex)은 형상 대신
    꺾은선을 담는다."""
    scene: list[dict[str, Any]] = []
    for p in r.parts:
        if p.flex is not None:
            if p.moving:
                raise GuideError(
                    f"{r.guide.id}: '{p.name}'는 부모 좌표로 그리는 선이라 자식과 같이 돌지 않는다"
                )
            scene.append(
                {
                    "n": p.name,
                    "k": p.key,
                    "t": [0.0, 0.0, 0.0],
                    "c": p.color,
                    "m": False,
                    "flex": _flex_json(p.name, p.flex),
                }
            )
            continue
        m = pack(*tessellate(p.shape))
        meshes.setdefault(m.id, m)
        scene.append(
            {
                "n": p.name,
                "k": p.key,
                "mesh": m.id,
                "t": _r(m.origin, 4),
                "c": p.color,
                # 2면 둘째 관절과도 같이 돈다 (화면이 첫째 관절 안의 축 둘레로 다시 돌린다)
                "m": 2 if p.moving2 else p.moving,
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
        "axis2": {"o": _r(g.axis2[0]), "d": _r(g.axis2[1])} if g.axis2 else None,
        "items": {k: _item_json(r.items[k], r.parts, r.plates) for k in sorted(used)},
        "scene": scene,
        "meshes": sorted({row["mesh"] for row in scene if "mesh" in row}),
        "steps": r.steps,
        "board": r.board,
        "work": sum(1 for s in r.steps if s["kind"] == "work"),
    }


def _printed_shape(module: str, name: str, cache: dict[str, Any]) -> Any:
    """출력 준비 도구와 같이 뽑는 방향으로 놓은 형상. 지금 설계 코드로 만든다 (모르는 부품이면 None)."""
    if module == "coupons":
        from horibot_design import coupons

        make = coupons.COUPONS.get(name)
        return oriented(make(), (0.0, 0.0, 1.0)) if make else None
    if module not in cache:
        try:
            cache[module] = importlib.import_module(f"horibot_design.parts.{module}").printed()
        except (ImportError, AttributeError):
            cache[module] = {}
    entry = cache[module].get(name)
    if entry is None:
        return None
    up = (float(entry[1][0]), float(entry[1][1]), float(entry[1][2]))
    return oriented(entry[0], up)


def _plate_scene(
    plates: list[Plate], meshes: dict[str, Mesh], warnings: list[str]
) -> tuple[list[dict[str, Any]], dict[str, int]]:
    """판의 부품마다 장면 하나 (부품을 누르면 화면이 3D로 보인다). STL 파일 이름 → 장면 번호도 준다."""
    scene: list[dict[str, Any]] = []
    index: dict[str, int] = {}
    cache: dict[str, Any] = {}
    for plate in plates:
        for part in plate.data["parts"]:
            file = str(part["file"])
            if file in index:
                continue
            module = str(part.get("module") or "")
            shape = _printed_shape(module, Path(file).stem, cache) if module else None
            if shape is None:
                warnings.append(f"{plate.guide}: {file}의 형상을 만들지 못해 3D로 보이지 않는다")
                index[file] = -1
                continue
            m = pack(*tessellate(shape))
            meshes.setdefault(m.id, m)
            index[file] = len(scene)
            color = "body" if plate.data["color"] == "white" else "cap"
            scene.append(
                {
                    "n": part["label"],
                    "k": Path(file).stem,
                    "mesh": m.id,
                    "t": _r(m.origin, 4),
                    "c": color,
                    "m": False,
                }
            )
    return scene, index


def _plate_step(plate: Plate, bed: str, scene_of: dict[str, int]) -> dict[str, Any]:
    d = plate.data
    color = catalog.COLOR_KO.get(str(d["color"]), str(d["color"]))
    minutes = int(d["minutes"])
    folder = f"build/print/{plate.bundle}/"
    warn = ["슬라이스 뒤에 경고가 뜨면 보내지 않고 알립니다."]
    warn += [
        f"{p['label']}은 높이에 비해 바닥이 좁아 넘어질 수 있습니다 (참고값)."
        for p in d["parts"]
        if p.get("slender")
    ]
    return {
        "id": plate.id,
        "no": str(plate.no),
        "kind": "plate",
        "short": f"{color} 판",
        "title": f"{color} · {_hm(minutes)} · {d['grams']} g",
        # 자르는 설정은 도구가 정해 두어 늘 같다. 판마다 다른 것만 적는다
        "do": [
            f"{color} 필라멘트, {BED_KO.get(bed, bed)} 판으로 {d['file']}을 뽑습니다.",
            f"프린터 화면의 파일 이름 끝 6자리가 {Path(str(d['file'])).stem[-6:]}인지 봅니다.",
            "다 뽑으면 대화창에 알립니다. 못 쓰는 부품이 있으면 무엇이 왜 그런지 같이 적습니다.",
        ],
        "warn": warn,
        "check": [],
        "stuck": PLATE_STUCK,
        "kit": [],
        "show": [],
        "plate": {
            "bundle": plate.bundle,
            "file": d["file"],
            "folder": folder,
            "image": f"data/prints/{plate.bundle}/{d['image']}" if d.get("image") else None,
            "color": d["color"],
            "preset": d["preset"],
            "minutes": minutes,
            "grams": d["grams"],
            "parts": [
                {
                    "file": p["file"],
                    "label": p["label"],
                    "count": int(p["count"]),
                    "height": p.get("height_mm"),
                    "p": scene_of[p["file"]] if scene_of.get(p["file"], -1) >= 0 else None,
                }
                for p in d["parts"]
            ],
        },
    }


def print_json(
    pg: PrintGuide,
    plans: Path = PLANS_DIR,
    meshes: dict[str, Mesh] | None = None,
    warnings: list[str] | None = None,
) -> dict[str, Any]:
    """출력 안내 하나의 JSON. 단계가 판 하나다 (뽑았는지와 못 쓰는 부품은 db의 records에서 화면이 읽는다).
    meshes를 주면 판의 부품 형상을 지금 설계 코드로 만들어 담는다 (부품을 누르면 3D로 보인다)."""
    if not ID.match(pg.id):
        raise GuideError(f"안내 id '{pg.id}'는 소문자, 숫자, 하이픈만 쓴다")
    if pg.kind != "print" or not pg.bundles:
        raise GuideError(f"{pg.id}: 출력 안내는 종류가 print이고 묶음이 하나 이상 있어야 한다")
    plates = plates_of(pg, plans)
    beds = {b: str(read_plan(b, plans).get("bed_type", "")) for b in pg.bundles}
    scene: list[dict[str, Any]] = []
    scene_of: dict[str, int] = {}
    if meshes is not None:
        scene, scene_of = _plate_scene(plates, meshes, warnings if warnings is not None else [])
    steps = [_plate_step(p, beds[p.bundle], scene_of) for p in plates]
    return {
        "id": pg.id,
        "kind": pg.kind,
        "title": pg.title,
        "sub": pg.sub,
        "bundle": "",
        "items": {},
        "scene": scene,
        "meshes": sorted({row["mesh"] for row in scene}),
        "steps": steps,
        "board": [],
        "work": len(steps),
        "summary": {
            "plates": len(steps),
            "minutes": sum(int(p.data["minutes"]) for p in plates),
            "grams": round(sum(float(p.data["grams"]) for p in plates), 1),
        },
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


def write_site(
    out: Path = OUT,
    guides: Sequence[Guide | PrintGuide] | None = None,
    plans: Path = PLANS_DIR,
) -> Summary:
    """모든 안내를 검사하고 out에 사이트를 쓴다. data/는 새로 쓴다 (지운 형상이 남지 않게)."""
    if guides is None:
        from horibot_design.guide.guides import ALL

        guides = ALL
    ids = [g.id for g in guides]
    if len(set(ids)) != len(ids):
        raise GuideError(f"안내 id가 겹친다: {ids}")
    prints = [g for g in guides if isinstance(g, PrintGuide)]
    if (out / "data").exists():
        shutil.rmtree(out / "data")
    meshes: dict[str, Mesh] = {}
    rows = []
    warnings: list[str] = []
    for g in guides:
        if isinstance(g, PrintGuide):
            data = print_json(g, plans, meshes, warnings)
            for bundle in g.bundles:
                for plate in read_plan(bundle, plans)["plates"]:
                    if plate.get("image"):
                        dest = out / "data" / "prints" / bundle / str(plate["image"])
                        dest.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copyfile(plans / bundle / str(plate["image"]), dest)
        else:
            r = resolve(g, prints, plans)
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
                # 처음 화면이 안내를 열지 않고 진행을 셈한다. 끝이 정해지는 단계마다 [id, 번호, 짧은 이름, 칸 수]:
                # 확인 칸이 있는 단계는 칸을 다 누르면, 판 단계(칸 수 0)는 Claude가 뽑았다고 적으면 끝난다
                "track": [
                    [s["id"], s["no"], s["short"], len(s["check"])]
                    for s in data["steps"]
                    if s["check"] or s["kind"] == "plate"
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
