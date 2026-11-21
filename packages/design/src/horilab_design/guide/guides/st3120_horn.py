"""ST-3120 혼 자리 맞춰 보기: 혼 자리 시험편(coupons.fit2_st3120_horn, 검정)을 어깨 서보(ID 2)의 혼에 엎어 덮어 본다
(status.md의 ST-3120). 어깨의 허브 판과 출력 원판을 뽑기 전에 한다.

시험편의 칸은 혼 지름에 여유(+0.1 ~ +0.4)를 더한 얕은 자리이고, 칸마다 가운데 구멍(어깨 허브 판의 가운데 구멍과 같은
지름)과 혼 볼트 구멍 여덟이 있다. 어깨의 허브 판은 혼 테에 앉고 볼트 여덟로 혼에 잡히므로, 이 안내에서 볼 것은
세 가지다: 가운데 돋움이 구멍에 들어 시험편이 테에 앉는지(옆 사진과 달그락거림), 볼트 구멍이 혼의 나사 구멍과
맞는지(마주 보는 구멍 둘에 나사를 손으로 돌려 넣기), 칸마다 덮이는 정도.

장면은 어깨 서보 좌표다 (parts/shoulder.py): y가 출력축(혼 쪽이 +)이고, 서보를 혼이 위를 보게 세운다. 시험편은
뒤집어(뽑을 때 윗면이 아래) +0.4 칸을 혼에 덮은 자리에 둔다.
"""

from __future__ import annotations

from build123d import Compound, Pos, Rot

from horilab_design import coupons
from horilab_design.guide.guides import first_joint as fj
from horilab_design.guide.model import (
    Guide,
    Item,
    Label,
    Move,
    ScenePart,
    Spot,
    Step,
    Tag,
    Use,
    View,
)
from horilab_design.parts import shoulder as sh
from horilab_design.parts.shapes import cyl_y, solid
from horilab_design.values import load_servos

SERVO_ID = 2
SPEC = load_servos()["ST-3120"]
CELLS = coupons.HORN_CLEARANCES  # 칸의 여유 (작은 것부터)
PITCH = SPEC.horn_diameter_mm + 6.0  # 칸 사이 (coupons.horn_recesses와 같다)
PLATE_H = coupons.HORN_RECESS_DEPTH + 2.0  # 시험편 두께
FIRST = len(CELLS) - 1  # 처음 덮는 칸: 가장 넉넉한 +0.4
TOP = sh.HORN_TOP  # 혼 테의 윗면 (서보 좌표 y)
UP = TOP + PLATE_H - coupons.HORN_RECESS_DEPTH  # 뒤집은 시험편의 윗면 (뽑을 때 바닥)
BOLT_R = SPEC.horn_bolt_circle_mm / 2
SCREW_L = 8.0
EDGE_X = coupons.MARGIN + PITCH / 2  # 처음 덮는 칸 쪽 시험편 끝 (혼 테 바깥으로 약 5 mm 나온다)

SERVO_ITEM = Item(
    "servo_st3120",
    f"서보 ST-3120 · ID {SERVO_ID}",
    "bought",
    f"어깨 서보. 출력 혼이 달린 것, ID {SERVO_ID} 이름표가 붙은 서보",
    thumb=View((0, 1, 0), (0.4, 0, 1), 22, 0),
)
COUPON_ITEM = Item(
    "fit2_st3120_horn",
    "혼 자리 시험편",
    "printed",
    "검정",
    file="fit2_st3120_horn.stl",
    color="black",
    thumb=View((0, 0, 1), (0, -1, 0.4), 30, 0),
)
PHONE = Item("phone", "휴대폰", "tool", "사진을 찍어 대화창에 보낸다", icon="phone")

SERVO = f"서보 ID {SERVO_ID}"
HORN = "출력 혼"
COUPON = "혼 자리 시험편"
SCREW_1 = "나사 M3x8 1"
SCREW_2 = "나사 M3x8 2"


def cell_x(i: int) -> float:
    """칸 i(여유 CELLS[i])의 가운데 x. 처음 덮는 칸(FIRST)이 혼 위(x = 0)다."""
    return PITCH * (i - FIRST)


def _coupon() -> ScenePart:
    """뒤집은 시험편: 뽑을 때의 (u, v, w) → (u - 칸 가운데, 테 윗면 + 2 - w, v - 칸 가운데).
    칸의 바닥(w = 2)이 혼 테의 윗면에 앉는다."""
    u_c = coupons.MARGIN + PITCH * (FIRST + 0.5)
    v_c = coupons.MARGIN + PITCH / 2
    plate = coupons.COUPONS["fit2_st3120_horn"]()
    shape = solid(plate.moved(Pos(-u_c, UP, -v_c) * Rot(90, 0, 0)))
    return ScenePart(COUPON, shape, COUPON_ITEM.key, "cap")


def _screw(x: float, z: float) -> Compound:
    """둥근머리 M3 × 8: 머리가 시험편 윗면에 앉고 나사가 혼의 나사 구멍으로 내려간다."""
    head = cyl_y(2.85, UP, UP + 1.65, x, z)
    return solid(head + cyl_y(1.5, UP - SCREW_L, UP, x, z))


def scene() -> list[ScenePart]:
    servo = sh.servo()
    return [
        ScenePart(SERVO, servo["body"], SERVO_ITEM.key, "servo"),
        ScenePart(HORN, servo["horn"], SERVO_ITEM.key, "metal", count=False),
        _coupon(),
        ScenePart(SCREW_1, _screw(BOLT_R, 0.0), "m3x8", "metal"),
        ScenePart(SCREW_2, _screw(-BOLT_R, 0.0), "m3x8", "metal"),
    ]


STAND = View((0, 1, 0), (0.6, 0, 1), 32, 0)  # 혼이 위, 비스듬히 위에서
FROM_TOP = View((1, 0, 0), (0, 1, 0), 0, 0)  # 바로 위에서 (화면 위가 서보의 가까운 끝)
FROM_SIDE = View((0, 1, 0), (1, 0, 0.15), 4, 0)  # 눈높이에서, 시험편 끝과 혼 테가 만나는 쪽
BACK_TAG = Tag((-17.0, -sh.HT - 6.0, 0.0), "뒷면 (책상 쪽)")
HORN_AT = (0.0, TOP + 6.0, 0.0)

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            f"어깨 서보 ST-3120(ID {SERVO_ID}), 혼 자리 시험편(검정), M3 × 8 나사 둘, 육각렌치, 휴대폰을 꺼냅니다.",
            "서보에는 출력 혼이 달려 있어야 합니다. 뒷면 혼은 빼서 보관하던 그대로 둡니다.",
        ),
        check=(f"서보 ST-3120 (ID {SERVO_ID})", "혼 자리 시험편", "M3 × 8 나사 둘과 육각렌치"),
        stuck=(
            "시험편을 아직 뽑지 않았으면 출력 안내 'ST-3120 혼 자리 시험편 출력'을 먼저 합니다. 모자라는 것이 "
            "있으면 대화창에 적어 주세요."
        ),
    ),
    Step(
        "stand",
        "세우기",
        "서보를 혼이 위로 오게 세우기",
        do=(
            f"ID {SERVO_ID} 서보를 혼이 위를 보게 책상에 세웁니다. 뒷면 가운데의 베어링이 볼록해서 흔들리면 한 손으로 "
            "잡습니다.",
            "혼 가운데의 볼록한 돋움과 그 위의 나사 머리를 봐 둡니다. 시험편 가운데 구멍에 이 둘이 들어갑니다.",
        ),
        check=("혼이 위를 보게 세웠다.",),
        adds=(SERVO, HORN),
        view=STAND,
        zoom=1.4,
        center=(0.0, TOP - 6.0, 0.0),
        labels=(Label(HORN, "출력 혼 (가운데 돋움)", at=HORN_AT),),
        tags=(BACK_TAG,),
    ),
    Step(
        "cover",
        "덮어 보기",
        "넉넉한 칸부터 혼에 덮어 보기",
        do=(
            "시험편 윗면에 새긴 숫자(+0.1 ~ +0.4)가 칸마다 혼 자리의 여유입니다. 뒤집으면 숫자가 밑으로 가니, "
            "뒤집기 전에 어느 칸인지 봐 둡니다.",
            "①(+0.4)부터 합니다. 시험편을 뒤집어 그 칸을 혼에 맞추고 손끝으로 살짝 누릅니다.",
            "칸마다 셋 가운데 하나로 봅니다: 힘 없이 내려가 바닥이 혼 테에 닿는다, 손끝으로 꾹 눌러야 들어간다, "
            "테에 걸려 안 내려간다.",
            "힘 없이 덮이면 다음 칸으로 갑니다: ②(+0.3), ③(+0.2), ④(+0.1) 차례입니다. 꾹 눌러야 하거나 안 "
            "내려가는 칸이 나오면 거기서 멈춥니다.",
            "칸마다 본 것을 대화창에 적어 주세요. 예: +0.4 덮임, +0.3 덮임, +0.2 꾹.",
        ),
        warn=(
            "꾹 눌러야 하는 칸은 끝까지 밀어 넣지 않습니다. 끼면 빼기 어렵고 혼이나 시험편이 상할 수 있습니다.",
        ),
        check=("해 본 칸마다 결과를 대화창에 적었다.",),
        stuck="칸이 헷갈리면 시험편 윗면(숫자가 보이는 쪽)을 찍어 보내 주세요.",
        adds=(COUPON,),
        moves=(Move((COUPON,), (0.0, 16.0, 0.0)),),
        view=STAND,
        zoom=1.0,
        center=(cell_x(FIRST - 1) - PITCH / 2, TOP, 0.0),
        labels=(Label(COUPON, "혼 자리 시험편 (뒤집어 덮는다)", at=(cell_x(1), UP, 0.0)),),
        spots=tuple(
            Spot(COUPON, label, ((cell_x(i), UP, 0.0),))
            for label, i in zip("①②③④", range(FIRST, -1, -1), strict=True)
        ),
    ),
    Step(
        "screws",
        "나사",
        "볼트 구멍이 맞는지 나사로 보기",
        do=(
            "힘 없이 덮인 칸 가운데 가장 작은 칸을 혼에 덮습니다. 다 꾹이었으면 +0.4 칸을 씁니다.",
            "구멍 여덟 가운데 마주 보는 둘(①, ②)에 M3 × 8 나사를 넣고, 손가락으로 돌려 혼의 나사 구멍에 넣어 "
            "봅니다. 구멍이 어긋나 보이면 시험편을 조금 돌려 맞는 자리를 찾습니다.",
            "두 나사가 걸리지 않고 서너 바퀴 들어가면 구멍이 맞는 것입니다. 손가락으로 안 돌아가면 육각렌치로 "
            "살살 돌려 보고, 그래도 걸리면 멈춥니다.",
            "두 나사가 들어갔는지, 걸렸다면 어느 쪽인지 대화창에 적어 주세요.",
        ),
        warn=("나사는 세게 조이지 않습니다. 구멍이 맞는지만 봅니다.",),
        check=("두 나사가 들어갔는지(또는 걸렸는지) 대화창에 적었다.",),
        adds=(SCREW_1, SCREW_2),
        moves=(Move((SCREW_1, SCREW_2), (0.0, 12.0, 0.0)),),
        uses=(Use("hexkey"),),
        view=STAND,
        zoom=1.6,
        center=(0.0, TOP, 0.0),
        spots=(
            Spot(SCREW_1, "①", ((BOLT_R, UP + 1.65, 0.0),)),
            Spot(SCREW_2, "②", ((-BOLT_R, UP + 1.65, 0.0),)),
        ),
    ),
    Step(
        "photo",
        "사진",
        "위와 옆에서 사진 찍기",
        do=(
            "나사를 꽂은 채로, 바로 위에서 한 장 찍습니다. 시험편의 구멍 여덟과 가운데 구멍 속이 보이게 합니다.",
            "눈높이에서, 시험편 끝의 아래 모서리와 그 밑의 혼 테(①)가 보이게 옆에서 한 장 찍습니다. 시험편이 테에 "
            "앉았는지, 가운데 돋움에 걸려 떠 있는지 Claude가 봅니다.",
            "시험편 양 끝을 번갈아 살짝 눌러 보고, 달그락거리는지도 적어 주세요.",
            "사진 두 장을 대화창에 보내 주세요.",
        ),
        check=("위와 옆 사진 두 장을 보냈다.", "달그락거리는지 적었다."),
        stuck=fj.PHOTO,
        uses=(Use(PHONE.key),),
        view=FROM_SIDE,
        zoom=2.2,
        center=(EDGE_X - 4.0, TOP - 1.0, 0.0),
        spots=(Spot(COUPON, "①", ((EDGE_X, TOP - coupons.HORN_RECESS_DEPTH, 0.0),)),),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "나사를 풀고 시험편을 혼에서 뗍니다.",
            "Claude가 결과를 적고, 어깨의 허브 판과 출력 원판의 볼트 구멍과 가운데 구멍을 확인합니다.",
        ),
        view=FROM_TOP,
    ),
)

GUIDE = Guide(
    id="st3120-horn",
    kind="measure",
    title="ST-3120 혼 자리 맞춰 보기",
    sub=f"어깨 허브 판을 뽑기 전에 · 서보 ID {SERVO_ID}의 혼에 시험편을 덮어 보고 사진 둘",
    scene=scene,
    steps=STEPS,
    items={SERVO_ITEM.key: SERVO_ITEM, COUPON_ITEM.key: COUPON_ITEM, PHONE.key: PHONE},
    bundle="st3120-coupon",
)
