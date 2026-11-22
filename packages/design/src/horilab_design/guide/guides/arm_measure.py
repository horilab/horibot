"""검정 판 전 재기: 팔의 검정 판을 뽑기 전에 실물로 잴 것 (status.md의 사용자 순서 1).

- 6806 바깥 링의 안쪽 지름 (2026-11-22에 쟀다: 39.05). 베어링 미는 도구(parts/base.py의 PUSH_BORE)의 통이 바깥
  링만 밀고 옆 덮개(ZZ)에는 닿지 않게 한다.
- 서보 어댑터에서 가장 높은 부품. 어댑터 위로 파이 판이 지나가고 그 사이가 1.4 mm뿐이라(parts/base.py의
  ADAPTER_TALL, TRAY_UNDER), 책상에 무는 판을 뽑기 전에 잰다.

젠더(ML-CCSG3), 찍찍이 타이, 파이 판에만 걸리는 어림 값은 재지 않는다 (status.md의 사용자 순서 2).

장면은 y가 위다. 베어링과 어댑터는 어림 모양이다 (재는 자리를 가리키는 그림이다).
"""

from __future__ import annotations

from build123d import Compound

from horilab_design.guide.caliper import Caliper, caliper_shape
from horilab_design.guide.guides import base_measure as bm
from horilab_design.guide.guides import first_joint as fj
from horilab_design.guide.model import Guide, Item, Label, ScenePart, Spot, Step, View
from horilab_design.parts import base as bs
from horilab_design.parts import joint as j
from horilab_design.parts.shapes import Point, box

# ---- 6806 (어림 모양: 옆 덮개는 면에서 0.3 들어가 있다) ----
R_OUT = j.BRG_OD / 2  # 21.0
R_RING_IN = 39.05 / 2  # 바깥 링의 안쪽 (잰 값)
R_INNER_OUT = 16.5  # 안쪽 링의 바깥 (그림)
R_IN = j.BRG_D / 2  # 15.0
B = j.BRG_B  # 7.0
SHIELD_IN = 0.3

OUTER = "6806 바깥 링"
SHIELD = "6806 옆 덮개"
INNER = "6806 안쪽 링"
CAL = "캘리퍼스 ① (바깥 링 안쪽 지름)"

# ---- 서보 어댑터 (어림 모양: 기판 42 × 33 × 1.6, 가장 높은 부품을 한쪽 짧은 변 가까이에 그렸다) ----
O_AD = (90.0, 0.0, 0.0)
AD_HX, AD_HZ = bs.ADAPTER[0] / 2, bs.ADAPTER[1] / 2
AD_T = 1.6
TALL_TOP = AD_T + bs.ADAPTER_TALL  # 기판 밑면에서 가장 높은 부품 꼭대기까지 (가정 12.6)
TALL_X, TALL_Z = (AD_HX - 10.0, AD_HX), (-4.0, 6.0)
AD_BOARD = "서보 어댑터 기판"
AD_TALL = "가장 높은 부품"
AD_LOW = "낮은 부품"
CAL_T = "캘리퍼스 ① (어댑터 기판 두께)"
CAL_H = "캘리퍼스 ② (가장 높은 부품까지)"


def _at(x: float, y: float, z: float) -> Point:
    return (O_AD[0] + x, O_AD[1] + y, O_AD[2] + z)


def _box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Compound:
    return box(x0 + O_AD[0], x1 + O_AD[0], y0 + O_AD[1], y1 + O_AD[1], z0 + O_AD[2], z1 + O_AD[2])


BEARING = Item("bearing_6806", "6806 베어링 하나", "bought", "가진 것 아무거나")
ADAPTER = Item("bus_adapter", "서보 어댑터", "bought", "파이와 서보 선을 잇는 작은 기판")

_CAL = Caliper(
    a=(-R_RING_IN, B - 0.4, 0.0),
    b=(R_RING_IN, B - 0.4, 0.0),
    side=(0.0, 1.0, 0.0),
    kind="inner",
)
_Z_HEIGHT = (TALL_Z[0] + TALL_Z[1]) / 2
_CAL_T = Caliper(
    a=_at(-AD_HX + 2.0, 0.0, 0.0),
    b=_at(-AD_HX + 2.0, AD_T, 0.0),
    side=(-1.0, 0.0, 0.0),
    reach=10.0,
)
_CAL_H = Caliper(
    a=_at(AD_HX - 3.0, 0.0, _Z_HEIGHT),
    b=_at(AD_HX - 3.0, TALL_TOP, _Z_HEIGHT),
    side=(1.0, 0.0, 0.0),
    reach=12.0,
)


def scene() -> list[ScenePart]:
    return [
        ScenePart(OUTER, j.ring(R_OUT, R_RING_IN, 0.0, B), BEARING.key, "metal"),
        ScenePart(
            SHIELD,
            j.ring(R_RING_IN, R_INNER_OUT, SHIELD_IN, B - SHIELD_IN),
            BEARING.key,
            "frame",
            count=False,
        ),
        ScenePart(INNER, j.ring(R_INNER_OUT, R_IN, 0.0, B), BEARING.key, "metal", count=False),
        ScenePart(CAL, caliper_shape(_CAL), bm.CALIPER.key, "metal", count=False),
        ScenePart(AD_BOARD, _box(-AD_HX, AD_HX, 0.0, AD_T, -AD_HZ, AD_HZ), ADAPTER.key, "frame"),
        ScenePart(
            AD_TALL, _box(*TALL_X, AD_T, TALL_TOP, *TALL_Z), ADAPTER.key, "plug", count=False
        ),
        ScenePart(
            AD_LOW,
            _box(-12.0, 4.0, AD_T, AD_T + 2.5, -12.0, -4.0),
            ADAPTER.key,
            "plug",
            count=False,
        ),
        ScenePart(CAL_T, caliper_shape(_CAL_T), bm.CALIPER.key, "metal", count=False),
        ScenePart(CAL_H, caliper_shape(_CAL_H), bm.CALIPER.key, "metal", count=False),
    ]


_BEARING_PARTS = (OUTER, SHIELD, INNER)
_ADAPTER_PARTS = (AD_BOARD, AD_TALL, AD_LOW)

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "6806 베어링 하나, 서보 어댑터, 캘리퍼스를 꺼냅니다. 베어링은 가진 것 아무거나 됩니다.",
            '값은 대화창에 단계 번호와 같이 적어 주세요 (예: "2단계 ② 12.4").',
        ),
        check=("다 꺼냈다.",),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    Step(
        "bearing-ring",
        "베어링 바깥 링",
        "6806 바깥 링의 안쪽 지름 재기",
        do=(
            "베어링을 책상에 눕힙니다. 가장 바깥의 바깥 링과 가운데 구멍 둘레의 안쪽 링 사이에, 조금 들어가 있는 "
            "얇은 옆 덮개가 있습니다.",
            "① 캘리퍼스 위쪽의 작은 안쪽 턱을 옆 덮개 위에 넣고 벌려, 바깥 링의 안쪽 가장자리에 양쪽 다 댑니다. "
            "턱이 가운데를 지나게 하고, 가장 크게 나오는 값을 읽습니다.",
            "값을 대화창에 적어 주세요. 2026-11-22에 39.05 mm로 쟀습니다.",
        ),
        warn=("턱 끝으로 옆 덮개를 누르지 않습니다.",),
        check=("①을 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(*_BEARING_PARTS, CAL),
        still=(*_BEARING_PARTS, CAL),
        mount=(*_BEARING_PARTS, CAL),
        focus=(*_BEARING_PARTS, CAL),
        uses=(bm.CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 40, 20),
        zoom=1.4,
        center=(0.0, 8.0, 0.0),
        labels=(
            Label(OUTER, "바깥 링", (0.0, B, R_OUT - 1.0)),
            Label(SHIELD, "옆 덮개", (-12.5, B - SHIELD_IN, 12.5)),
            Label(INNER, "안쪽 링", (0.0, B, -R_IN - 0.7)),
        ),
        spots=(Spot(OUTER, "①", ((-R_RING_IN, B, 0.0), (R_RING_IN, B, 0.0))),),
    ),
    Step(
        "adapter",
        "서보 어댑터",
        "서보 어댑터에서 가장 높은 부품 재기",
        do=(
            "서보 어댑터를 옆에서 보고, 기판 위로 가장 높이 솟은 부품을 찾습니다 (그림의 덩어리는 자리를 가리키는 "
            "어림 모양입니다).",
            "① 기판 가장자리의 두께를 캘리퍼스 바깥 턱으로 잽니다.",
            "② 바깥 턱 하나를 기판 밑면에, 다른 하나를 가장 높은 부품의 꼭대기에 대고 잽니다. 밑면에 튀어나온 "
            "핀은 비켜서 댑니다.",
            "두 값을 대화창에 적어 주세요. 가장 높은 부품이 기판 위 11 mm라고 가정했습니다.",
        ),
        warn=("부품을 턱으로 세게 물지 않습니다.",),
        check=("①과 ②를 대화창에 적었다.",),
        stuck="어느 부품이 가장 높은지 헷갈리면 옆에서 찍은 사진을 보내 주세요.",
        adds=(*_ADAPTER_PARTS, CAL_T, CAL_H),
        still=(*_ADAPTER_PARTS, CAL_T, CAL_H),
        mount=(*_ADAPTER_PARTS, CAL_T, CAL_H),
        focus=(*_ADAPTER_PARTS, CAL_T, CAL_H),
        uses=(bm.CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 20, 0),
        zoom=1.2,
        center=_at(0.0, 6.0, 0.0),
        labels=(
            Label(AD_BOARD, "어댑터 기판", _at(-5.0, AD_T, AD_HZ - 3.0)),
            Label(AD_TALL, "가장 높은 부품", _at(AD_HX - 5.0, TALL_TOP, _Z_HEIGHT)),
        ),
        spots=(
            Spot(AD_BOARD, "①", (_at(-AD_HX, 0.0, 0.0), _at(-AD_HX, AD_T, 0.0))),
            Spot(AD_TALL, "②", (_at(AD_HX, 0.0, _Z_HEIGHT), _at(AD_HX, TALL_TOP, _Z_HEIGHT))),
        ),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "검정 판 전 재기가 끝났습니다.",
            "Claude가 어댑터 값으로 파이 판 밑의 여유를 보고, 책상에 무는 판과 파이 판의 뽑을 파일을 냅니다.",
        ),
        focus=(*_BEARING_PARTS, *_ADAPTER_PARTS),
        view=View((0, 1, 0), (0, 0, 1), 30, 20),
    ),
)

GUIDE = Guide(
    id="arm-measure",
    kind="measure",
    title="검정 판 전 재기",
    sub="책상에 무는 판을 뽑기 전에 · 캘리퍼스",
    scene=scene,
    steps=STEPS,
    items={i.key: i for i in (BEARING, ADAPTER, bm.CALIPER)},
)
