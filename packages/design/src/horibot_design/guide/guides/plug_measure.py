"""검정 판 3 전 재기: 어댑터 덮개를 뽑기 전에 꽂은 서보 플러그의 높이를 잰다 (status.md의 사용자 순서 1).

서보 어댑터의 서보 단자는 위를 봐서, 꽂은 플러그 위로 선이 곧게 올라와 반 바퀴 돈다 (parts/pi_mount.py의
AD_PLUG_BACK, AD_LOOP_R). 플러그 윗면 높이는 바짝 꺾은 선의 꼭대기를 잰 값(23.3)에서 꺾이는 몫을 어림해 뺀 값이고,
덮개 천장과 그 고리 사이가 2.55 mm뿐이라 덮개를 뽑기 전에 잰다. 덮개는 검정 판 3(`arm-black-2`의 판 1)에 있다.

장면은 y가 위다. 기판 위 부품의 자리는 parts/pi_mount.py의 AD_ITEMS(Waveshare 치수 그림에서 읽은 값)이고, 꽂은
플러그와 선은 어림 모양이다 (재는 자리를 가리키는 그림이다).
"""

from __future__ import annotations

from build123d import Compound

from horibot_design.guide.caliper import Caliper, caliper_shape
from horibot_design.guide.guides import arm_measure as am
from horibot_design.guide.guides import base_measure as bm
from horibot_design.guide.model import Guide, Item, Label, ScenePart, Spot, Step, View
from horibot_design.parts import base as bs
from horibot_design.parts import pi_mount as pm
from horibot_design.parts.shapes import Point, box, solid

HU, HV = bs.ADAPTER[0] / 2, bs.ADAPTER[1] / 2  # 기판 반 (긴 변 u를 x로, 짧은 변 v를 z로 그린다)
T = bs.ADAPTER_T
PLUG_TOP = pm.AD_PLUG_BACK  # 기판 밑면에서 꽂은 플러그 윗면까지 (어림 15.3)
PLUG_D = 5.4  # 꽂은 플러그의 앞뒤 두께 (그림)
WIRE_W, WIRE_T = 4.85, 1.5  # 납작한 서보 선 (폭, 두께)

BOARD = "서보 어댑터 기판"
PARTS = "어댑터 위 부품"
EMPTY = "빈 서보 단자"
PLUG = "꽂은 서보 플러그"
WIRE = "서보 선"
CAL = "캘리퍼스 ① (기판 밑면에서 플러그 윗면까지)"

SPARE = Item(
    "servo_cable_spare",
    "서보 선 하나",
    "bought",
    "어댑터에 꽂혀 있으면 그대로, 없으면 남는 20 cm 선",
    icon="wire",
)


def _x(u: float) -> float:
    return u - HU


def _z(v: float) -> float:
    return v - HV


def _item(key: str, top: float | None = None) -> Compound:
    u0, u1, v0, v1, h = pm.AD_ITEMS[key]
    return box(_x(u0), _x(u1), T, top if top is not None else T + h, _z(v0), _z(v1))


_U0, _U1, _V0, _V1, _H = pm.AD_ITEMS["서보 단자 2"]
_UC = (_U0 + _U1) / 2
PLUG_X = (_x(_UC) - pm.AD_PLUG_W / 2, _x(_UC) + pm.AD_PLUG_W / 2)
PLUG_Z = (_z(0.0), _z(0.0) + PLUG_D)
_ZC = (PLUG_Z[0] + PLUG_Z[1]) / 2
_X_MEASURE = PLUG_X[1] - 1.0  # 선 옆의 플러그 윗면 (선은 플러그 가운데에서 나온다)

_CAL = Caliper(
    a=(_X_MEASURE, 0.0, _z(0.0) + 0.5),
    b=(_X_MEASURE, PLUG_TOP, _z(0.0) + 0.5),
    side=(0.0, 0.0, -1.0),
    reach=6.0,
    tip=1.0,
)


def scene() -> list[ScenePart]:
    others = [k for k in pm.AD_ITEMS if not k.startswith("서보 단자")]
    low = _item(others[0])
    for k in others[1:]:
        low = solid(low + _item(k))
    wire_x = (_x(_UC) - WIRE_W / 2, _x(_UC) + WIRE_W / 2)
    return [
        ScenePart(BOARD, box(-HU, HU, 0.0, T, -HV, HV), am.ADAPTER.key, "frame"),
        ScenePart(PARTS, low, am.ADAPTER.key, "metal", count=False),
        ScenePart(EMPTY, _item("서보 단자 1"), am.ADAPTER.key, "plug", count=False),
        ScenePart(PLUG, box(*PLUG_X, T, PLUG_TOP, *PLUG_Z), SPARE.key, "plug"),
        ScenePart(
            WIRE,
            box(*wire_x, PLUG_TOP, PLUG_TOP + 18.0, _ZC - WIRE_T / 2, _ZC + WIRE_T / 2),
            SPARE.key,
            "wire",
            count=False,
        ),
        ScenePart(CAL, caliper_shape(_CAL), bm.CALIPER.key, "metal", count=False),
    ]


_ALL = (BOARD, PARTS, EMPTY, PLUG, WIRE, CAL)
_AD = (BOARD, PARTS, EMPTY, PLUG, WIRE)


def _at(x: float, y: float, z: float) -> Point:
    return (x, y, z)


STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "서보 어댑터, 서보 선 하나, 캘리퍼스를 꺼냅니다.",
            '값은 대화창에 단계 번호와 같이 적어 주세요 (예: "2단계 ① 15.4").',
        ),
        check=("다 꺼냈다.",),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    Step(
        "plug",
        "꽂은 플러그",
        "꽂은 서보 플러그의 높이 재기",
        do=(
            "어댑터의 서보 단자에 선이 꽂혀 있으면 그대로 씁니다. 없으면 남는 20 cm 선 하나를 단자 하나에 끝까지 "
            "꽂습니다 (20 cm 선은 팔에 쓰지 않습니다).",
            "선은 플러그 위로 곧게 세웁니다. 꺾지 않습니다.",
            "① 캘리퍼스 바깥 턱 하나를 기판 밑면에, 다른 하나를 플러그 윗면(선이 나오는 면)에 대고 잽니다. 턱은 "
            "기판 가장자리 쪽에서 넣어, 밑면에 튀어나온 핀과 플러그 위의 선을 비켜서 댑니다.",
            "값을 대화창에 적어 주세요. 기판 밑면에서 15.3 mm라고 어림했습니다.",
        ),
        warn=(
            "다 재고 꽂은 선은 빼지 않아도 됩니다 (커넥터가 꽉 껴서, 빼는 것은 조립할 때 한 번만 합니다).",
        ),
        check=("①을 대화창에 적었다.",),
        stuck="어디에 대는지 헷갈리면 옆에서 찍은 사진을 보내 주세요.",
        adds=_ALL,
        still=_ALL,
        mount=_ALL,
        focus=_ALL,
        uses=(bm.CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, -1), 20, 40),
        zoom=2.0,
        center=(_x(_UC), 9.0, _z(0.0) + 4.0),
        labels=(
            Label(BOARD, "어댑터 기판", _at(HU - 6.0, T, HV - 4.0)),
            Label(PLUG, "꽂은 서보 플러그", _at(PLUG_X[0], PLUG_TOP - 3.0, PLUG_Z[0])),
            Label(WIRE, "서보 선 (곧게 위로)", _at(_x(_UC), PLUG_TOP + 16.0, _ZC)),
        ),
        spots=(
            Spot(
                PLUG,
                "①",
                (_at(_X_MEASURE, 0.0, _z(0.0)), _at(_X_MEASURE, PLUG_TOP, _z(0.0))),
            ),
        ),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "검정 판 3 전 재기가 끝났습니다.",
            "Claude가 이 값으로 덮개 천장의 여유를 보고, 넉넉하면 그대로 두고, 모자라면 덮개를 고쳐 검정 판 3의 "
            "파일을 다시 냅니다.",
        ),
        focus=_AD,
        view=View((0, 1, 0), (0, 0, -1), 25, 40),
    ),
)

GUIDE = Guide(
    id="plug-measure",
    kind="measure",
    title="검정 판 3 전 재기",
    sub="어댑터 덮개를 뽑기 전에 · 캘리퍼스",
    scene=scene,
    steps=STEPS,
    items={i.key: i for i in (am.ADAPTER, SPARE, bm.CALIPER)},
)
