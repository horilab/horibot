"""글자 핀 맞춰 보기: 핀 시험편(coupons.fit4_letter_pegs, 검정)을 흰 위팔과 아래팔의 글자 구멍에 꽂아 보고, 글자의
핀 지름(parts/links.py의 PEG_D)을 면마다 고른다 (docs/decisions/0019-ur-look.md).
글자는 검정 판 4에 넣어 뽑으므로 그 판을 내기 전에 한다 (status.md의 사용자 순서 2).

시험편은 작은 판 여섯이고, 판마다 한쪽 끝에 핀 하나가 서 있다 (지름 2.1 ~ 2.6, 판 윗면에 새김). 위팔의 납작한 면(BOMI)은
벽을 뚫은 구멍이고, 아래팔의 둥근 면(HORI)은 꼭대기에서 2.6 mm 막힌 구멍이라 따로 고른다. 고르는 법은 coupons.py의
LETTER_PEGS: 손으로 눌러 들어가고 뒤집어도 안 빠지는 가장 가는 칸이다.

장면은 링크 좌표다 (parts/links.py): x가 관을 따라가고, 납작한 면은 y = FRONT에서 +y를 보고, 둥근 면의 꼭대기는 -y를
본다. 아래팔은 위팔과 겹치지 않게 옆(+z)으로 옮겨 둔다. 시험편은 그림으로 한 칸만 꽂아 보인다.
"""

from __future__ import annotations

from build123d import Compound, Pos

from horibot_design import coupons
from horibot_design.guide.model import Guide, Item, Label, Move, ScenePart, Spot, Step, Use, View
from horibot_design.parts import links
from horibot_design.parts.shapes import Point, box, cyl_y, solid

FORE_SHIFT = 70.0  # 아래팔을 위팔 옆으로 옮겨 두는 거리 (z)
TAB_W, TAB_L = coupons.LETTER_TAB
TAB_T = links.LETTER_T["flat"]  # 시험편 판의 두께 (납작한 면 글자와 같다)
PEG_L = coupons.LETTER_PEG_LEN
PEG_D = 2.3  # 그림의 핀 지름 (가운데 값)

UPPER_ITEM = Item(
    "upper_arm",
    "위팔",
    "printed",
    "흰색",
    file="upper_arm.stl",
    color="white",
    thumb=View((0, 0, -1), (0, 1, 0), 20, 0),
)
FORE_ITEM = Item(
    "forearm",
    "아래팔",
    "printed",
    "흰색",
    file="forearm.stl",
    color="white",
    thumb=View((0, 0, 1), (0, -1, 0), 20, 0),
)
PEGS_ITEM = Item(
    "fit4_letter_pegs",
    "핀 시험편",
    "printed",
    "검정, 핀이 하나씩 선 작은 판 여섯",
    file="fit4_letter_pegs.stl",
    color="black",
    thumb=View((0, 0, 1), (0, -1, 0.5), 40, 0),
)

UPPER = "위팔"
FORE = "아래팔"
TAB_UPPER = "핀 시험편 (위팔 구멍)"
TAB_FORE = "핀 시험편 (아래팔 구멍)"


def _hole(name: str) -> Point:
    """시험하는 구멍: 글자 I의 첫 구멍 (링크 좌표의 구멍 가운데, 면의 높이)."""
    key = links.letter_keys(name)[-1]
    x, z = next((x, z) for k, x, z in links.peg_spots(name) if k == key)
    y = links.letter_point(name, 0.0, 0.0)[1]  # 납작한 면이면 FRONT, 둥근 면이면 관 꼭대기
    return (x, y, z)


UPPER_HOLE = _hole("upper_arm")
FORE_HOLE = _hole("forearm")
FORE_AT = (FORE_HOLE[0], FORE_HOLE[1], FORE_HOLE[2] + FORE_SHIFT)  # 옮겨 둔 아래팔에서의 자리


def _tab(hole: Point, outward: float) -> Compound:
    """구멍에 꽂은 시험편 한 칸: 판이 면 위에 눕고(관을 따라 길게), 핀이 구멍으로 들어간다. outward는 면의 바깥 방향(±y)."""
    x, y, z = hole
    plate_y = (y, y + TAB_T) if outward > 0 else (y - TAB_T, y)
    peg_y = (y - PEG_L, y) if outward > 0 else (y, y + PEG_L)
    plate = box(x - TAB_W / 2, x + TAB_L - TAB_W / 2, *plate_y, z - TAB_W / 2, z + TAB_W / 2)
    return solid(plate + cyl_y(PEG_D / 2, *peg_y, x, z))


def scene() -> list[ScenePart]:
    shift = Pos(0.0, 0.0, FORE_SHIFT)
    return [
        ScenePart(UPPER, links.body("upper_arm"), UPPER_ITEM.key, "body"),
        ScenePart(FORE, solid(links.body("forearm").moved(shift)), FORE_ITEM.key, "body"),
        ScenePart(TAB_UPPER, _tab(UPPER_HOLE, 1.0), PEGS_ITEM.key, "cap"),
        ScenePart(
            TAB_FORE, solid(_tab(FORE_HOLE, -1.0).moved(shift)), PEGS_ITEM.key, "cap", count=False
        ),
    ]


FLAT_VIEW = View((0, 0, -1), (0, 1, 0), 25, 0)  # 납작한 면을 마주 보고 (BOMI가 바로 읽힌다)
ROUND_VIEW = View((0, 0, 1), (0, -1, 0), 25, 0)  # 둥근 면을 마주 보고 (HORI가 바로 읽힌다)


def _try(face: str, word: str, example: str) -> tuple[str, ...]:
    return (
        f"{face}에 작은 구멍이 여덟 개 있습니다 ({word} 글자가 꽂힐 자리). 그림의 ① 구멍 하나로 합니다.",
        "2.1부터 합니다. 판에 새긴 숫자가 핀의 굵기(mm)입니다. 핀을 구멍에 대고 엄지로 판을 눌러 봅니다.",
        "칸마다 셋 가운데 하나로 봅니다: 힘 없이 들어간다, 엄지로 꾹 눌러야 들어간다, 꾹 눌러도 안 들어간다.",
        "들어갔으면 팔을 뒤집어 구멍이 바닥을 보게 하고, 시험편이 떨어지는지 봅니다. 그다음 판을 잡고 곧게 당겨 "
        "뺍니다.",
        "다음 굵기로 갑니다. 안 들어가는 칸이 나오면 거기서 멈춥니다.",
        f"칸마다 본 것을 대화창에 적어 주세요. 예: {example}",
    )


TRY_WARN = (
    "안 들어가는 핀은 세게 밀지 않습니다. 끼면 빼기 어렵고, 구멍이 넓어져 글자가 헐거워집니다.",
)

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "검정 판 1에서 나온 핀 시험편(핀이 하나씩 선 작은 검정 판 여섯), 흰 위팔, 흰 아래팔을 꺼냅니다.",
            "시험편 판에 새긴 2.1 ~ 2.6이 핀의 굵기(mm)입니다. 가는 것부터 차례로 꽂아 봅니다.",
        ),
        check=("핀 시험편 여섯", "위팔 (흰색)", "아래팔 (흰색)"),
        stuck="검정 판 1을 아직 꺼내지 않았으면 먼저 꺼냅니다. 모자라는 것이 있으면 대화창에 적어 주세요.",
    ),
    Step(
        "upper",
        "위팔",
        "위팔의 글자 구멍에 핀 꽂아 보기",
        do=_try(
            "위팔의 넓고 평평한 면",
            "BOMI",
            "위팔 2.1 힘 없이·떨어짐, 2.2 꾹·안 떨어짐, 2.3 안 들어감.",
        ),
        warn=TRY_WARN,
        check=("위팔에서 해 본 칸마다 결과를 대화창에 적었다.",),
        stuck="어느 면인지 헷갈리면 위팔을 찍어 보내 주세요.",
        adds=(UPPER, TAB_UPPER),
        still=(UPPER,),
        mount=(UPPER, TAB_UPPER),
        focus=(UPPER, TAB_UPPER),
        moves=(Move((TAB_UPPER,), (0.0, 14.0, 0.0)),),
        view=FLAT_VIEW,
        zoom=4.0,
        center=(UPPER_HOLE[0] + 4.0, UPPER_HOLE[1], UPPER_HOLE[2]),
        labels=(
            Label(UPPER, "위팔의 넓고 평평한 면", at=(UPPER_HOLE[0] - 30.0, UPPER_HOLE[1], 0.0)),
        ),
        spots=(Spot(UPPER, "①", (UPPER_HOLE,)),),
    ),
    Step(
        "fore",
        "아래팔",
        "아래팔의 글자 구멍에 핀 꽂아 보기",
        do=_try(
            "아래팔의 둥근 면(넓고 평평한 면의 반대쪽)",
            "HORI",
            "아래팔 2.1 힘 없이·떨어짐, 2.2 힘 없이·안 떨어짐, 2.3 꾹·안 떨어짐, 2.4 안 들어감.",
        ),
        warn=TRY_WARN,
        check=("아래팔에서 해 본 칸마다 결과를 대화창에 적었다.",),
        stuck="어느 면인지 헷갈리면 아래팔을 찍어 보내 주세요.",
        adds=(FORE, TAB_FORE),
        still=(FORE,),
        mount=(FORE, TAB_FORE),
        focus=(FORE, TAB_FORE),
        moves=(Move((TAB_FORE,), (0.0, -14.0, 0.0)),),
        uses=(Use(PEGS_ITEM.key, count=False),),  # 위팔 단계에서 센 시험편을 다시 쓴다
        view=ROUND_VIEW,
        zoom=4.0,
        center=(FORE_AT[0] + 4.0, FORE_AT[1], FORE_AT[2]),
        labels=(Label(FORE, "아래팔의 둥근 면", at=(FORE_AT[0] - 30.0, FORE_AT[1], FORE_SHIFT)),),
        spots=(Spot(FORE, "①", (FORE_AT,)),),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "핀 시험편은 글자를 다 꽂을 때까지 보관합니다.",
            "Claude가 면마다 핀 굵기를 고르고, 그 굵기로 글자를 넣은 검정 판 4를 다시 냅니다.",
        ),
        focus=(UPPER, FORE),
        view=View((0, 0, -1), (0, 1, 0), 35, 0),
    ),
)

GUIDE = Guide(
    id="letter-pegs",
    kind="measure",
    title="글자 핀 맞춰 보기",
    sub="검정 판 4 전에 · 위팔과 아래팔의 글자 구멍에 핀 시험편을 꽂아 보기",
    scene=scene,
    steps=STEPS,
    items={i.key: i for i in (UPPER_ITEM, FORE_ITEM, PEGS_ITEM)},
    bundle="arm-black-1",
)
