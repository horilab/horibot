"""안내 정의의 모양. 안내 모듈(guides/)이 이 데이터로 단계를 적고, 만들기(build)가 검사해 JSON으로 쓴다.

좌표는 그 안내 장면의 좌표(조립 안내는 집 관절의 서보 좌표, mm, y가 출력축)다.

부품 고르는 말(선택자): 장면 부품 이름의 앞부분. 끝에 "$"를 붙이면 이름이 꼭 같은 것만, "@z+"처럼 축과 부호를
붙이면 상자 가운데가 그쪽(5 mm 넘게)인 것만 고른다. 예: "벽 나사 1", "혼$", "너트 (누름 틀)@x+".
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from build123d import Compound

Vec = tuple[float, float, float]

# 장면 부품의 색 구실 (화면의 색과 같은 이름)
COLORS = ("body", "cap", "frame", "metal", "servo", "plug", "pin")


@dataclass(frozen=True)
class Item:
    """부품 목록의 한 줄. kind: printed(출력), bought(산 것), hardware(나사와 너트), tool(공구).

    draw는 화면이 그림끼리 같은 비율로 그리는 그림의 치수(나사, 너트, 핀, 와셔), icon은 공구와 선의 그림 이름,
    thumb는 작은 그림을 그릴 때의 방향(장면에 형상이 있는 것만)이다.
    """

    key: str
    name: str
    kind: str
    meta: str = ""
    file: str = ""
    color: str = ""
    draw: dict[str, Any] = field(default_factory=dict)
    icon: str = ""
    thumb: View | None = None


@dataclass(frozen=True)
class ScenePart:
    """장면에 놓인 부품 하나. key는 부품 목록(catalog)의 키, count가 거짓이면 쓰는 것의 개수에 넣지 않는다
    (서보에 딸린 혼처럼 다른 부품의 일부)."""

    name: str
    shape: Compound
    key: str
    color: str
    moving: bool = False
    count: bool = True


@dataclass(frozen=True)
class View:
    """손에 드는 방향: up은 화면 위로 오는 축, face는 보는 사람 쪽을 보는 축. el은 내려다보는 각, az는 오른쪽으로
    돌아가 보는 각 [°]."""

    up: Vec
    face: Vec
    el: float = 28.0
    az: float = 28.0


@dataclass(frozen=True)
class Move:
    """끼우는 움직임: 부품들이 by만큼 떨어진 자리에서 제자리로 들어온다. delay [s]는 같은 단계 안의 차례."""

    parts: tuple[str, ...]
    by: Vec
    delay: float = 0.0


@dataclass(frozen=True)
class Place:
    """따로 놓고 보이는 부품: 상자 가운데를 pos에 두고, up/face 방향으로 돌린다 (장면 좌표)."""

    part: str
    pos: Vec
    up: Vec
    face: Vec


@dataclass(frozen=True)
class Label:
    """부품 이름표. at은 부품의 설계 좌표(조립된 자리의 장면 좌표)이고, 없으면 상자 가운데다."""

    part: str
    text: str
    at: Vec | None = None


@dataclass(frozen=True)
class Tag:
    """방향 이름표 (뒤, 발 쪽 같은). 장면 좌표."""

    at: Vec
    text: str


@dataclass(frozen=True)
class Spot:
    """손볼 자리 번호 (①, ② 같은). at은 부품의 설계 좌표."""

    part: str
    label: str
    at: tuple[Vec, ...]


@dataclass(frozen=True)
class Use:
    """장면 부품 말고 이 단계에 쓰는 것: 공구, 앞 단계에서 만든 묶음, 그림이 없는 부품(선, 테이프).

    qty가 0이면 개수를 보이지 않는다. count가 참이고 qty가 있으면 준비물 개수에 더한다.
    """

    key: str
    qty: int = 0
    note: str = ""
    count: bool = True


@dataclass(frozen=True)
class Step:
    """단계 하나. kind: prep(준비물), work(손동작 하나), end(끝).

    adds는 이 단계에서 처음 손에 드는 부품이다. 쓰는 것의 개수는 여기서 한 번만 센다. 그 가운데 still은
    움직이지 않는 것(이번 손동작이 아닌 것)이다. mount는 이 단계에 본체에 붙는 부품이고, 주지 않으면 따로 놓고
    하는 단계(focus가 있는 단계)는 없음, 그 밖은 adds다. 보이는 부품은 앞 단계까지 본체에 붙은 것이고, focus를
    주면 그것만 보인다 (따로 놓고 하는 단계). 이번 부품(색을 입히는 것)은 adds에서 still을 뺀 것과 움직이는
    부품이다.
    """

    id: str
    short: str
    title: str
    do: tuple[str, ...]
    kind: str = "work"
    warn: tuple[str, ...] = ()
    check: tuple[str, ...] = ()
    stuck: str = ""
    adds: tuple[str, ...] = ()
    still: tuple[str, ...] = ()
    mount: tuple[str, ...] | None = None
    uses: tuple[Use, ...] = ()
    focus: tuple[str, ...] = ()
    hide: tuple[str, ...] = ()
    place: tuple[Place, ...] = ()
    moves: tuple[Move, ...] = ()
    view: View | None = None
    shell: tuple[str, ...] = ()
    cut: bool = False
    ghost: bool = False
    labels: tuple[Label, ...] = ()
    tags: tuple[Tag, ...] = ()
    spots: tuple[Spot, ...] = ()
    joint: float = 0.0
    turn: tuple[float, float] | None = None


@dataclass(frozen=True)
class Guide:
    """안내 하나. scene은 장면 부품을 만드는 함수(설계 코드를 부른다), items는 공통 목록에 더할 이 안내의 출력
    부품, bundle은 출력 묶음 이름(있으면 판 번호와 파일 이름을 그 plan.json과 맞춘다)."""

    id: str
    kind: str
    title: str
    sub: str
    scene: Callable[[], list[ScenePart]]
    steps: tuple[Step, ...]
    items: dict[str, Item] = field(default_factory=dict)
    bundle: str = ""
