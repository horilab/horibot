"""파이 덮개 철망 전 재기: 철망에서 팬 나사 머리가 앉는 높이를 정하기 전에 봉지째인 팬의 나사와 너트, 찍찍이 타이의
두께를 잰다 (status.md의 사용자 순서 1, 0038의 파이 덮개). 타이 폭은 상품 규격 값(10 mm)이라 재지 않는다.

뽑는 철망(parts/pi_mount.py의 fan_grille_local)은 팬 나사 넷이 철망, 아크릴 판, 팬을 같이 묶는다. 두 덮개에는 케이스와
같이 온 봉지째인 새 팬과 그 봉지의 나사를 쓴다 (봉지째인 팬이 7개 남았다, hardware.md). 그래서 머리가 앉는
높이(GRILLE_HEAD_SEAT)는 봉지 나사의 길이와 너트로 정하고, 파이가 돌고 있는 쌓인 케이스는 풀지 않는다 (팬 나사를
풀면 팬 밑의 너트가 파이 위로 떨어질 수 있다). 쌓인 케이스의 금속 철망 두께는 재지 않는다: 새 덮개의 나사가 그
철망과 묶여 있던 나사가 아니다.

나사 길이는 바깥 턱으로 머리 밑을 물 수 없어서, 전체 길이와 머리 높이를 따로 재서 뺀다. 장면은 y가 위다. 팬은
상품 이름의 크기(30 × 30 × 7), 나사와 너트는 흔한 M3 둥근머리와 너트 크기의 예시 그림이다 (재기 전이다).
"""

from __future__ import annotations

from build123d import Compound

from horibot_design.guide.caliper import Caliper, caliper_shape
from horibot_design.guide.guides import base_measure as bm
from horibot_design.guide.guides import first_joint as fj
from horibot_design.guide.model import Guide, Item, Label, ScenePart, Spot, Step, Use, View
from horibot_design.parts import joint as j
from horibot_design.parts import pi_mount as pm
from horibot_design.parts.shapes import Point, box, cyl_x, cyl_y, solid

FAN_HALF = pm.FAN[0] / 2
FAN_T = pm.FAN[1]
FAN_HOLE = pm.FAN_HOLE_PITCH / 2
# 예시 나사와 너트 (그림): M3 둥근머리 × 10과 M3 너트
SCREW_L = 10.0  # 머리 밑에서 끝까지
HEAD_D, HEAD_H = 5.5, 2.0
THREAD_D = 3.0
NUT_AF, NUT_T = 5.5, 2.4
TIE_W, TIE_T, TIE_LEN = 10.0, 1.6, 120.0  # 폭은 상품 규격, 두께는 예시
YC = HEAD_D / 2  # 누운 나사의 축 높이

O_SCREW: Point = (100.0, 0.0, 0.0)  # 재는 나사의 머리 윗면 (머리가 -x)
O_NUT: Point = (150.0, 0.0, 0.0)
O_TIE = 190.0  # 찍찍이 타이의 한 끝 (x)
BAG_ROWS = (-12.0, -4.0, 4.0, 12.0)  # 봉지에서 나온 나사와 너트를 늘어놓은 줄 (z)

FAN = "봉지째인 팬"
BAG_SCREW = "봉지 나사"
BAG_NUT = "봉지 너트"
SCREW = "재는 나사"
NUT = "재는 너트"
TIE = "찍찍이 타이"
CAL_LEN = "캘리퍼스 ① (전체 길이)"
CAL_HEAD_H = "캘리퍼스 ② (머리 높이)"
CAL_HEAD_D = "캘리퍼스 ③ (머리 지름)"
CAL_THREAD = "캘리퍼스 ④ (나사 굵기)"
CAL_NUT = "캘리퍼스 ⑤ (너트 두께)"
CAL_TIE_T = "캘리퍼스 ⑥ (타이 두께)"

UP: Point = (0.0, 1.0, 0.0)
FRONT: Point = (0.0, 0.0, 1.0)
BACK: Point = (0.0, 0.0, -1.0)

FAN_ITEM = Item(
    "fan_bagged",
    "봉지째인 팬",
    "bought",
    "케이스와 같이 온 LD3007MS. 하나만 뜯는다",
    thumb=View((0, 1, 0), (0, 0, 1), 45, 10),
)
SCREW_ITEM = Item(
    "fan_screw",
    "팬 나사",
    "hardware",
    "봉지에서 나온 것 하나",
    thumb=View((0, 1, 0), (0, 0, 1), 30, 25),
)
NUT_ITEM = Item(
    "fan_nut",
    "너트",
    "hardware",
    "봉지에 있으면 하나",
    thumb=View((0, 1, 0), (0, 0, 1), 50, 10),
)
TIE_ITEM = Item(
    "velcro_tie",
    "찍찍이 케이블 타이",
    "bought",
    "2026-11-24에 산 롤",
    thumb=View((0, 1, 0), (0, 0, 1), 40, 20),
)
PHONE = Item("phone", "휴대폰", "tool", "사진", icon="phone")


def _fan() -> Compound:
    f = box(-FAN_HALF, FAN_HALF, 0.0, FAN_T, -FAN_HALF, FAN_HALF)
    f = solid(f - cyl_y(13.5, 1.0, FAN_T + 1.0))
    f = solid(f + cyl_y(6.0, 1.0, FAN_T - 0.5))
    for sx in (1, -1):
        for sz in (1, -1):
            f = solid(f - cyl_y(1.6, -1.0, FAN_T + 1.0, sx * FAN_HOLE, sz * FAN_HOLE))
    return f


def _screw(x0: float, z: float) -> Compound:
    """책상에 누운 나사: 머리 윗면이 x0, 끝이 +x."""
    s = cyl_x(HEAD_D / 2, x0, x0 + HEAD_H, YC, z)
    s = solid(s + cyl_x(THREAD_D / 2, x0 + HEAD_H, x0 + HEAD_H + SCREW_L, YC, z))
    s = solid(s - box(x0 - 0.1, x0 + 0.7, YC - 0.3, YC + 0.3, z - 2.0, z + 2.0))  # 십자 홈
    return solid(s - box(x0 - 0.1, x0 + 0.7, YC - 2.0, YC + 2.0, z - 0.3, z + 0.3))


def _nut(x: float, z: float) -> Compound:
    n = j.hex_y(x, z, 0.0, NUT_T, NUT_AF)
    return solid(n - cyl_y(THREAD_D / 2, -1.0, NUT_T + 1.0, x, z))


SX = O_SCREW[0]
TIP = SX + HEAD_H + SCREW_L
HEAD_MID = SX + HEAD_H / 2
THREAD_X = SX + HEAD_H + 7.0  # 나사 굵기를 무는 자리 (③의 캘리퍼스와 겹치지 않게 끝 쪽)
RIM_Y = YC + 2.2  # 머리 가장자리 (나사 줄 위)
NX, NZ = O_NUT[0], O_NUT[2]

# 작은 부품을 가리지 않게, 캘리퍼스의 자와 몸통은 보는 쪽(앞, +z)의 반대편인 위나 뒤에 두고, 가로로 무는 것은 몸통이
# 뒤로 가게 앞에서 뒤로 잰다
_CALS: tuple[tuple[str, Caliper], ...] = (
    (CAL_LEN, Caliper((SX, YC, 0.0), (TIP, YC, 0.0), UP, reach=10.0, tip=2.0)),
    # 머리만 위아래로 문다: 턱 끝이 머리 가장자리에 닿고 나사 줄은 비켜 간다
    (CAL_HEAD_H, Caliper((SX, RIM_Y, 0.0), (SX + HEAD_H, RIM_Y, 0.0), UP, reach=9.0, tip=0.5)),
    # 머리 지름과 나사 굵기는 앞뒤로 문다: 자는 머리 왼쪽(③)과 나사 끝 오른쪽(④)에 두고 위에서 내려다본다
    (
        CAL_HEAD_D,
        Caliper(
            (HEAD_MID, YC, HEAD_D / 2),
            (HEAD_MID, YC, -HEAD_D / 2),
            (-1.0, 0.0, 0.0),
            reach=9.0,
            tip=0.5,
        ),
    ),
    (
        CAL_THREAD,
        Caliper(
            (THREAD_X, YC, THREAD_D / 2),
            (THREAD_X, YC, -THREAD_D / 2),
            (1.0, 0.0, 0.0),
            reach=9.0,
            tip=0.5,
        ),
    ),
    (CAL_NUT, Caliper((NX, 0.0, NZ - 1.0), (NX, NUT_T, NZ - 1.0), BACK, reach=9.0, tip=1.0)),
    (CAL_TIE_T, Caliper((O_TIE + 60.0, 0.0, 3.0), (O_TIE + 60.0, TIE_T, 3.0), FRONT, tip=1.0)),
)


def scene() -> list[ScenePart]:
    parts = [ScenePart(FAN, _fan(), FAN_ITEM.key, "servo")]
    # 봉지에서 나온 것 (그림은 예시라 개수에 넣지 않는다. 봉지째인 팬의 작은 그림에 같이 들어간다)
    parts += [
        ScenePart(f"{BAG_SCREW} {k}", _screw(24.0, z), FAN_ITEM.key, "metal", count=False)
        for k, z in enumerate(BAG_ROWS, start=1)
    ]
    parts += [
        ScenePart(f"{BAG_NUT} {k}", _nut(46.0, z), FAN_ITEM.key, "metal", count=False)
        for k, z in enumerate(BAG_ROWS, start=1)
    ]
    parts += [
        ScenePart(SCREW, _screw(SX, 0.0), SCREW_ITEM.key, "metal"),
        ScenePart(NUT, _nut(NX, NZ), NUT_ITEM.key, "metal"),
        ScenePart(
            TIE,
            box(O_TIE, O_TIE + TIE_LEN, 0.0, TIE_T, -TIE_W / 2, TIE_W / 2),
            TIE_ITEM.key,
            "servo",
        ),
    ]
    parts += [
        ScenePart(name, caliper_shape(c), bm.CALIPER.key, "metal", count=False) for name, c in _CALS
    ]
    return parts


_BAG = (FAN, BAG_SCREW, BAG_NUT)
_WIDTH = (CAL_HEAD_D, CAL_THREAD)
LOW = View(
    (0, 1, 0), (0, 0, 1), 12, -10
)  # 앞에서 낮게: 위에 있는 캘리퍼스 자가 나사를 가리지 않는다

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "봉지째인 팬 하나(파이 케이스와 같이 온 것), 찍찍이 타이, 캘리퍼스, 휴대폰을 꺼냅니다.",
            "파이가 든 쌓인 케이스는 건드리지 않습니다. 새 덮개에는 봉지째인 새 팬과 그 나사를 씁니다.",
            '값은 대화창에 단계 번호와 같이 적어 주세요 (예: "2단계 ① 12.1").',
        ),
        check=("다 꺼냈다.",),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    Step(
        "bag",
        "팬 봉지",
        "팬 봉지 열어 보기",
        do=(
            "봉지째인 팬 하나를 뜯어, 든 것을 모두 책상에 늘어놓습니다.",
            "위에서 사진 한 장을 찍어 대화창에 보내고, 나사와 너트가 몇 개씩인지 적어 주세요. 나사 모양(머리, 끝)과 "
            "덮개 하나에 나사 넷이 있는지를 봅니다.",
            "봉지에 나사가 없으면 여기서 멈추고 그렇다고 적어 주세요. Claude가 다음을 정합니다.",
        ),
        warn=("그림의 나사와 너트는 예시입니다. 봉지에 든 것과 모양이나 개수가 달라도 됩니다.",),
        check=("사진을 보내고 나사와 너트 수를 적었다.",),
        stuck=fj.PHOTO,
        adds=_BAG,
        mount=_BAG,
        focus=_BAG,
        uses=(Use(PHONE.key),),
        view=View((0, 1, 0), (0, 0, 1), 55, 15),
        labels=(
            Label(FAN, "팬"),
            Label(f"{BAG_SCREW} 1", "나사 (그림은 예시)"),
            Label(f"{BAG_NUT} 1", "너트 (있으면)"),
        ),
    ),
    Step(
        "screw-len",
        "나사 길이",
        "팬 나사의 전체 길이 재기",
        do=(
            "봉지에서 나온 나사 하나로 잽니다.",
            "① 전체 길이: 캘리퍼스 바깥 턱으로 머리 윗면과 나사 끝을 물려 잽니다.",
            "①을 대화창에 적어 주세요. 다음 단계의 머리 높이를 빼면 머리 밑의 길이이고, 그것으로 철망에서 머리가 "
            "앉는 높이를 정합니다.",
        ),
        check=("①을 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(SCREW, CAL_LEN),
        still=(CAL_LEN,),
        mount=(SCREW, CAL_LEN),
        focus=(SCREW, CAL_LEN),
        uses=(bm.CALIPER_USE,),
        view=LOW,
        zoom=2.2,
        center=(SX + 6.0, YC, 0.0),
        spots=(Spot(SCREW, "①", ((SX, YC, HEAD_D / 2), (TIP, YC, THREAD_D / 2))),),
    ),
    Step(
        "head-h",
        "머리 높이",
        "팬 나사의 머리 높이 재기",
        do=(
            "같은 나사로 잽니다.",
            "② 머리 높이: 바깥 턱으로 머리만 윗면과 밑면을 물려 잽니다. 턱 끝을 머리 가장자리에 대서 나사 줄은 "
            "비켜 갑니다.",
            "②를 대화창에 적어 주세요.",
        ),
        check=("②를 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(CAL_HEAD_H,),
        still=(CAL_HEAD_H,),
        mount=(CAL_HEAD_H,),
        focus=(SCREW, CAL_HEAD_H),
        uses=(bm.CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 12, -30),
        zoom=3.5,
        center=(SX + 2.0, YC + 1.0, 0.0),
        spots=(Spot(SCREW, "②", ((HEAD_MID, YC + HEAD_D / 2, 0.0),)),),
    ),
    Step(
        "screw-width",
        "나사 굵기",
        "팬 나사의 머리 지름과 굵기 재기",
        do=(
            "같은 나사로 잽니다.",
            "③ 머리 지름: 바깥 턱으로 머리를 가장 넓은 곳에서 물려 잽니다.",
            "④ 나사 굵기: 바깥 턱으로 나사 줄을 물려 잽니다 (줄의 꼭대기끼리).",
            "③④를 대화창에 적어 주세요. 철망에서 머리가 들어가는 자리와 나사 구멍의 크기를 봅니다.",
        ),
        check=("③④를 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=_WIDTH,
        still=_WIDTH,
        mount=_WIDTH,
        focus=(SCREW, *_WIDTH),
        uses=(bm.CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 58, 10),
        zoom=4.0,
        center=(SX + 6.0, YC, 0.0),
        spots=(
            Spot(SCREW, "③", ((HEAD_MID, YC, HEAD_D / 2),)),
            Spot(SCREW, "④", ((THREAD_X, YC, THREAD_D / 2),)),
        ),
    ),
    Step(
        "nut",
        "너트",
        "너트 두께 재기",
        do=(
            "봉지에 너트가 있으면 하나를 바깥 턱으로 납작하게 물려 ⑤ 두께를 잽니다.",
            "너트가 없으면 없다고 적고 넘어갑니다.",
            "나사가 너트까지 닿는지 셈할 때 씁니다.",
        ),
        check=("⑤를 적었거나, 너트가 없다고 적었다.",),
        stuck=fj.PHOTO,
        adds=(NUT, CAL_NUT),
        still=(CAL_NUT,),
        mount=(NUT, CAL_NUT),
        focus=(NUT, CAL_NUT),
        uses=(bm.CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 20, 20),
        zoom=2.6,
        center=(NX, NUT_T / 2, NZ),
        spots=(Spot(NUT, "⑤", ((NX + 2.0, NUT_T / 2, NZ + 2.0),)),),
    ),
    Step(
        "tie",
        "찍찍이 타이",
        "찍찍이 타이 두께 재기",
        do=(
            "폭은 상품 규격으로 10 mm라 재지 않습니다.",
            "⑥ 한 겹 두께: 가장 두꺼운 곳을, 누르지 않고 턱이 닿을 만큼만 물려 잽니다.",
            "⑥을 대화창에 적어 주세요. 파이 판과 뒤 판의 타이 구멍(14 × 3 mm)을 지나는지 봅니다.",
        ),
        check=("⑥을 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(TIE, CAL_TIE_T),
        still=(CAL_TIE_T,),
        mount=(TIE, CAL_TIE_T),
        focus=(TIE, CAL_TIE_T),
        uses=(bm.CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 40, 20),
        zoom=1.6,
        center=(O_TIE + 67.0, 0.0, 0.0),
        spots=(Spot(TIE, "⑥", ((O_TIE + 60.0, 0.0, TIE_W / 2), (O_TIE + 60.0, TIE_T, TIE_W / 2))),),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "파이 덮개 철망 전 재기가 끝났습니다.",
            "뜯은 팬과 나사, 너트는 봉지에 다시 넣어 둡니다. 내일 저녁 파이 덮개에 씁니다.",
            "Claude가 이 값으로 철망에서 나사 머리가 앉는 높이를 정하고 철망 판을 냅니다. 내일 낮에 뽑습니다.",
        ),
        focus=_BAG,
        view=View((0, 1, 0), (0, 0, 1), 55, 15),
    ),
)

GUIDE = Guide(
    id="grille-measure",
    kind="measure",
    title="파이 덮개 철망 전 재기",
    sub="오늘 밤 · 봉지째인 팬 하나, 찍찍이 타이, 캘리퍼스",
    scene=scene,
    steps=STEPS,
    items={i.key: i for i in (FAN_ITEM, SCREW_ITEM, NUT_ITEM, TIE_ITEM, bm.CALIPER, PHONE)},
)
