"""아래팔 조립과 J3 영점: 아래팔 끝의 J4(STS3215, ID 4)와 선 둘, 아래팔 달기, J3(ID 3) 영점, 쉬는 자세 (0029, 0031, 0034).

조립 전에 영점 핀이 구멍 셋(위팔 끝 집의 살, 아래팔 발의 귀, 아래팔 끝 집의 살)에 드는지 본다. 위팔 발 귀는 시험편 값으로
그렸는데도 핀이 안 들어가 M3 × 8로 넓혔다 (2026-11-26). 막히면 같은 방법으로 넓힌다.

순서는 위팔(upper_arm)과 같은 까닭으로 정했다 (2026-11-25):
- 허브 판을 누름 틀보다 먼저 넣는다.
- 서보를 넣기 전에 선 둘을 꿴다: J3 → J4 선(어제 위팔의 선 창 밖으로 빼 둔 끝)을 아래팔 발의 큰 구멍으로 넣어 관 속으로,
  J4 → J5 선(새 300 mm)을 끝 집의 선 창으로. J3 → J4 선이 위팔에 매여 있어서 아래팔은 위팔 끝 옆에서 조립한다.
- J4 → J5 선은 손목에 꽂지 않고 창 밖으로 약 20 cm를 늘어뜨려 둔다. 내일 손목 1의 J5 -z 커넥터(비어 있다)에 꽂는다. 손목을
  오늘 선에 매달면 J3 영점을 잴 때 손목이 허공에 매달린다.
- J3 영점은 J2 영점 핀을 다시 꽂아 위팔을 곧게 세운 채(핀이 위팔을 잡는다), 아래팔을 곧게 세운 범위 가운데(0°)에서
  아래팔 발의 귀와 위팔 끝 집의 살에 핀을 꽂는다. + 쪽은 아래팔이 앞으로 숙는 쪽이다. 설계로 본 방향은 +1이다.
- 다 하면 쉬는 자세(J1 -55°, J2 52°, J3 154°)로 아래팔 끝을 선반에 얹어 둔다 (0037).

장면 좌표는 J1의 서보 좌표다 (arm_common). 둘째 관절은 J3다.
"""

from __future__ import annotations

from dataclasses import replace

from horibot_design.guide.catalog import printed_items
from horibot_design.guide.guides import arm_common as ac
from horibot_design.guide.guides import upper_arm as ua
from horibot_design.guide.model import (
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
from horibot_design.parts import joint as j
from horibot_design.parts import links, wrist
from horibot_design.parts.shapes import cyl_y, solid

SERVO_ID = 4
J3_ID = 3
MODEL = "STS3215"
AXIS_JOINT = 3
HAVE = ua.HAVE
REST_PREFIX = "쉬는 자세/"

DONE = Item(
    "assembled_upper",
    "조립해 둔 베이스, 어깨, 위팔",
    "bought",
    "J1 ~ J3(ID 1 ~ 3)과 누르개, 어댑터",
)
SERVO_ITEM = Item(
    "servo_sts3215_j4",
    f"서보 STS3215 · ID {SERVO_ID}",
    "bought",
    f"혼이 달린 것. ID {SERVO_ID} 이름표가 붙은 서보 (J4)",
)
PRINTED: dict[str, Item] = printed_items(
    {**wrist.LABELS, **links.LABELS},
    {
        "forearm": (None, None, "white"),
        **{
            k: (None, None, "black")
            for k in ("retainer", "hub_plate", "out_disc", "carrier", "link_cover")
        },
    },
)
OTHER: dict[str, Item] = {
    it.key: it
    for it in (
        DONE,
        SERVO_ITEM,
        Item(
            "servo_cable_900",
            "서보 선 900 mm",
            "bought",
            "어제 위팔의 선 창 밖으로 빼 둔 J3 → J4 선",
            icon="wire",
        ),
        Item(
            "servo_cable_300",
            "서보 선 300 mm",
            "bought",
            "Waveshare 5264 3핀 (J4 → J5)",
            icon="wire",
        ),
        Item("psu", "가변 전원", "tool", "전류 상한 3 A로 맞춰 둔 것"),
        Item("pc", "집 PC", "tool", "USB 5 m 선이 꽂힌 것. Claude가 명령을 돌린다"),
        Item("longnose", "롱노즈", "tool", "핀을 곧게 당겨 뺀다", icon="tweezers"),
        Item("phone", "휴대폰", "tool", "사진을 찍어 대화창에 보낸다", icon="phone"),
        Item("towel", "수건", "tool", "아래팔을 조립하는 동안 밑에 깐다"),
    )
}
ITEMS_HERE: dict[str, Item] = {**PRINTED, **OTHER}

J3_AXIS = ac.axis_of("j3")
_LEN = links.LINKS["forearm"][0]
TRY_UA, TRY_FOOT, TRY_END = (
    "시험 핀 (위팔 끝 집)",
    "시험 핀 (아래팔 발 귀)",
    "시험 핀 (아래팔 끝 집)",
)
# 위팔 안내의 것 가운데 이 안내에서 다시 그리는 것: 창 밖에 늘어뜨린 J3 → J4 선의 끝, J2 영점 핀 (다시 꽂는다)
_UA_REDRAW = ("J4 선 (창 밖)", "J4 선 끝 플러그", "영점 핀 (J2)")


def _forearm_screws() -> list[ac.Piece]:
    """아래팔 발을 J3 출력 원판에 다는 M3 × 10 넷 (아래팔의 틀: 머리가 귀 윗면에 앉고 원판 밑면의 너트에 물린다)."""
    top = j.FRONT - j.EAR_T
    out = []
    for k, (dx, z) in enumerate(links.STS_FOOT.rim):
        x = -_LEN + dx
        s = solid(cyl_y(1.5, top, top + j.BODY_SCREW, x, z) + cyl_y(2.85, top - 1.65, top, x, z))
        out.append(ac.Piece(f"아래팔 나사 {k + 1}", ac.put(s, "j4"), "m3x10", "metal", "j4"))
    return out


def _have_pieces() -> list[ac.Piece]:
    """위팔 안내에서 붙인 것 (J2 영점 핀과 창 밖에 늘어뜨린 선 끝은 빼고)."""
    return [
        replace(p, name=f"{HAVE}{p.name}", key=DONE.key, count=False)
        for p in ua.pieces()
        if p.name not in _UA_REDRAW
    ]


def pieces() -> list[ac.Piece]:
    out = ac.link_pieces("forearm", SERVO_ITEM.key, "selftap_so101")
    out += _forearm_screws()
    # J3 → J4 선: 위팔 선 창 → 아래팔 발의 높은 창 → 관 속 → J4의 -z 커넥터
    run_in = ac.link_in("forearm")
    across = [
        ac.pt("j3", ac.window_point(0.0)),
        ac.pt("j3", ac.window_point(10.0)),
        ac.pt("j4", run_in[0]),
    ]
    out.append(
        ac.Piece("J4 선 (관절 밖)", ac.wire_scene(across), "servo_cable_900", "wire", "j3", False)
    )
    out.append(
        ac.Piece("J4 선 (아래팔 속)", ac.wire(run_in, "j4"), "servo_cable_900", "wire", "j4", False)
    )
    out.append(
        ac.Piece(
            "J4 선 플러그 (J4)", ac.plug(-1, MODEL, "j4"), "servo_cable_900", "plug", "j4", False
        )
    )
    # J4 → J5 선: J4의 +z 커넥터 → 선 창 → 창 밖에 늘어뜨린 끝 (내일 손목 1에 꽂는다)
    out.append(
        ac.Piece(
            "J5 선 플러그 (J4)", ac.plug(1, MODEL, "j4"), "servo_cable_300", "plug", "j4", False
        )
    )
    out.append(
        ac.Piece(
            "J5 선 (아래팔 속)",
            ac.wire(ac.link_out_run(), "j4"),
            "servo_cable_300",
            "wire",
            "j4",
            False,
        )
    )
    tail = [ac.window_point(0.0), ac.window_point(20.0), (-55.0, j.FRONT + 38.0, 0.0)]
    out.append(
        ac.Piece("J5 선 (창 밖)", ac.wire(tail, "j4"), "servo_cable_300", "wire", "j4", False)
    )
    end = ac.pt("j4", tail[-1])
    out.append(
        ac.Piece(
            "J5 선 끝 플러그",
            ac.loose_plug(end, ac.vec("j4", (-1.0, 0.0, 0.0))),
            "servo_cable_300",
            "plug",
            "j4",
            False,
        )
    )
    # 영점 핀: J2(위팔을 곧게 잡는다)와 J3
    pin2 = next(p for p in ua.pieces() if p.name == "영점 핀 (J2)")
    out.append(replace(pin2, count=False))
    out.append(
        ac.Piece("영점 핀 (J3)", ac.put(ac.link_pin("forearm"), "j4"), "pin_msh3_20", "pin", "j3")
    )
    # 조립 전에 핀이 구멍에 드는지 보는 핀 (같은 영점 핀이라 개수에 넣지 않는다). 위팔 발 귀 구멍은 시험편 값(3.35)으로
    # 그렸는데도 핀이 안 들어가 M3 × 8로 넓혔다 (2026-11-26). 같은 식으로 뽑은 구멍 셋을 미리 본다:
    # J3 핀이 드는 위팔 끝 집의 살과 아래팔 발의 귀, J4 핀이 드는 아래팔 끝 집의 살 (내일 손목 잇기)
    j3_pin = ac.put(ac.link_pin("forearm"), "j4")
    pin4 = next(p for p in wrist.child_hardware() if p.name.startswith("영점 핀"))
    out += [
        ac.Piece(TRY_UA, j3_pin, "pin_msh3_20", "pin", "j3", False),
        ac.Piece(TRY_FOOT, j3_pin, "pin_msh3_20", "pin", "j3", False),
        ac.Piece(TRY_END, ac.put(pin4.shape, "j4"), "pin_msh3_20", "pin", "j4", False),
    ]
    return out


def rest_pieces() -> list[ac.Piece]:
    """쉬는 자세(J2 52°, J3 154°)의 위팔, 아래팔, 어깨 출력 원판 (J1은 첫째 관절로 돌린다). 개수에 넣지 않는다."""
    out = ac.link_pieces("upper_arm", DONE.key, DONE.key, ac.REST, REST_PREFIX)
    out += ac.link_pieces("forearm", DONE.key, DONE.key, ac.REST, REST_PREFIX)
    out += [
        p
        for p in ac.group_pieces("shoulder", DONE.key, ac.REST, REST_PREFIX)
        if p.frame == "j2_out"
    ]
    return [replace(p, key=DONE.key, count=False) for p in out]


def scene() -> list[ScenePart]:
    have = [replace(p, key=DONE.key, moving2=False) for p in ua._have()]
    return (
        have
        + ac.scene_of(_have_pieces(), AXIS_JOINT)
        + ac.scene_of(pieces(), AXIS_JOINT)
        + ac.scene_of(rest_pieces(), AXIS_JOINT)
    )


# 쉬는 자세 단계에서 숨길 것: 곧게 선 팔의 부품 (위팔과 그 너머, 영점 핀, J2와 같이 도는 어깨 부품). 이름 앞부분으로
# 고른다: 베이스와 J1 몸통에 붙은 선과 누르개는 이름이 "베이스 선/", "… 누르개"로 시작해 걸리지 않는다
SHOULDER_TURNING = tuple(
    f"{HAVE}{n}"
    for n in ("허브 판", "출력 원판", "혼", "멈춤 나사", "너트 (멈춤 기둥)", "너트 (출력 원판)")
)
UPRIGHT = (
    f"{HAVE}위팔",
    f"{HAVE}J3 ",
    f"{HAVE}J4 선",
    "아래팔",
    "J4 ",
    "J5 선",
    "영점 핀",
    *SHOULDER_TURNING,
)


def upright_names() -> tuple[str, ...]:
    return UPRIGHT


# ------------------------------------------------------------------------------------------------
# 이름

ARM = "아래팔$"
BRG = "J4 베어링"
HUB = "J4 허브 판"
DISC = "J4 출력 원판"
RET = "J4 누름 틀"
RET_NUTS = "J4 너트 (누름 틀)"
CAR = "J4 서보 받침"
CAR_NUTS = "J4 너트 (서보 받침)"
SERVO = "J4 서보 몸통"
HORN = "J4 혼$"
REAR = "J4 서보 뒷면 나사"
COVER, COVER_SCREWS = "J4 뒷덮개", "J4 덮개 나사"
DISC_NUTS = "J4 너트 (출력 원판)"
STOP = ("J4 멈춤 나사", "J4 너트 (멈춤 기둥)")
DISC_SET = (DISC, DISC_NUTS, *STOP)
SERVO_SET = (SERVO, HORN, CAR, CAR_NUTS, REAR)
BOLTS = "J4 혼 볼트"
FRONT_ROW = ("J4 벽 나사 1$", "J4 벽 나사 3$", "J4 벽 나사 5$")
BACK_ROW = ("J4 벽 나사 2$", "J4 벽 나사 4$")
IN_ACROSS, IN_ARM, IN_PLUG = "J4 선 (관절 밖)", "J4 선 (아래팔 속)", "J4 선 플러그 (J4)"
OUT_PLUG, OUT_ARM, OUT_TAIL, OUT_END = (
    "J5 선 플러그 (J4)",
    "J5 선 (아래팔 속)",
    "J5 선 (창 밖)",
    "J5 선 끝 플러그",
)
ARM_SCREWS = "아래팔 나사"
PIN2, PIN3 = "영점 핀 (J2)", "영점 핀 (J3)"
REST = REST_PREFIX
IN_HAND = (
    ARM,
    BRG,
    HUB,
    RET,
    RET_NUTS,
    *FRONT_ROW,
    *SERVO_SET,
    *DISC_SET,
    BOLTS,
    *BACK_ROW,
    IN_ARM,
    IN_PLUG,
    OUT_PLUG,
    OUT_ARM,
    OUT_TAIL,
    OUT_END,
)


def _slide(name: str, reach: float = 13.0) -> tuple[float, float, float]:
    _n, _c, _a, s = next(slot for slot in j.side_nut_slots() if slot[0] == name)
    return ac.vec("j4", (s[0] * reach, s[1] * reach, s[2] * reach))


BACK_UP = ac.link_view("j4", (0, -1, 0), (0.5, 0, 1), 30)
FRONT_UP = ac.link_view("j4", (0, 1, 0), (0.5, 0, 1), 30)
FOOT_VIEW = ac.link_view("j4", (0, -1, 0), (-1, 0, 0.6), 35)
FROM_RIGHT = View((0, 1, 0), (0.35, 0, 1), 15, 0)  # 로봇 오른쪽에서 (J3 출력 원판 쪽)
FROM_FRONT = View((0, 1, 0), (1, 0, 0.35), 15, 0)
_J4 = ac.pt("j4", (0.0, 0.0, 0.0))
_FOOT = ac.pt("j4", (-_LEN, j.FRONT, 0.0))
_FOOT_WIN = ac.pt("j4", ac.link_in("forearm")[1])
_WIN_OUT = ac.pt("j4", ac.window_point(2.0))
_PIN_AT = ac.pt("j4", (-_LEN - j.R_PIN, j.FRONT - j.EAR_T, 0.0))
_J3 = ac.pt("j3", (0.0, 0.0, 0.0))

PREP = Step(
    "prep",
    "준비물",
    "준비물 꺼내 놓기",
    kind="prep",
    do=(
        "서보 전원이 꺼져 있는지(빨강 플러그를 뺐는지) 봅니다.",
        "출력 부품: 아래팔(흰색, 글자를 꽂은 것), 검정 J4 속 부품 다섯(누름 틀, 허브 판, 출력 원판, 서보 받침, 링크 뒷덮개).",
        f"베어링 6806, ID {SERVO_ID} 이름표가 붙은 STS3215, 서보 선 300 mm 하나, 영점 핀 둘.",
        "나사: M3 × 8 다섯, M3 × 10 다섯(멈춤 기둥 하나, 아래팔 넷), M3 × 12 둘, M3 × 20 넷, 서보 뒷면 자가탭 넷, M3 너트 열.",
        "위팔 끝 옆(책상 앞쪽)에 수건을 깔아 아래팔을 조립할 자리를 만듭니다. 위팔은 어제처럼 수건 위에 누워 있습니다.",
    ),
    check=(
        "빨강 플러그가 빠져 있다.",
        "출력 부품 여섯과 베어링, 서보(ID 4), 300 mm 선, 영점 핀 둘",
        "M3 × 8 5, × 10 5, × 12 2, × 20 4, 자가탭 4, M3 너트 10",
    ),
    stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
)

_WIDEN = (
    "안 들어가거나 아주 빡빡하면, M3 × 8을 그 구멍에 육각렌치로 끝까지 조여 넣었다가 풀어 빼고 핀을 다시 꽂아 봅니다 "
    "(어제 위팔 발 귀를 넓힌 방법)."
)
_TRY_WARN = (
    "핀을 망치나 공구로 박지 않습니다.",
    "넓힌 뒤 핀이 구멍에서 헐렁하게 흔들리면 더 넓히지 말고 알려 주세요.",
)
_TRY_USES = (
    Use("longnose"),
    Use("hexkey", note="막히면 넓힐 때", count=False),
    Use("m3x8", note="막히면 구멍을 넓히는 데. 빼서 다시 쓴다", count=False),
)
PIN_TRY_STEPS: tuple[Step, ...] = (
    Step(
        "pin-try-upper",
        "핀 구멍 (위팔)",
        "J3 영점 핀이 위팔 끝 집 구멍에 드는지 보기",
        do=(
            "누워 있는 위팔 끝, J3 출력 원판 옆 끝 집의 살에 있는 작은 구멍(그림의 핀 자리)에 영점 핀을 손으로 꽂아 봅니다. "
            "5 mm쯤 들어가면 됩니다. 아래팔을 단 뒤 J3 영점을 잴 때 핀이 여기까지 들어갑니다.",
            _WIDEN,
            "핀은 롱노즈로 곧게 당겨 뺍니다.",
        ),
        warn=_TRY_WARN,
        check=("핀이 손으로 5 mm쯤 들어가고, 롱노즈로 빠진다.",),
        stuck=ac.PHOTO,
        adds=(TRY_UA,),
        mount=(TRY_UA,),
        removes=(TRY_UA,),
        focus=(f"{HAVE}위팔$", f"{HAVE}J3 ", TRY_UA),
        uses=_TRY_USES,
        moves=(Move((TRY_UA,), (0.0, 0.0, 24.0)),),
        view=FROM_RIGHT,
        zoom=1.8,
        center=_PIN_AT,
        labels=(Label(TRY_UA, "위팔 끝 집 살의 구멍 (J3 핀)"),),
    ),
    Step(
        "pin-try-forearm",
        "핀 구멍 (아래팔)",
        "영점 핀이 아래팔 구멍 둘에 드는지 보기 (손에 들고)",
        do=(
            "아래팔 발 끝의 핀 귀에 있는 작은 구멍에 영점 핀을 꽂아 봅니다. 핀이 귀를 지나 반대쪽으로 나오면 됩니다 (J3 핀).",
            "아래팔 끝 집(J4가 들어갈 쪽)의 살에 있는 작은 구멍에도 꽂아 봅니다. 5 mm쯤 들어가면 됩니다. 내일 손목을 이을 때 "
            "J4 핀이 여기에 듭니다.",
            _WIDEN,
        ),
        warn=_TRY_WARN,
        check=("두 구멍 모두 핀이 손으로 들어가고 롱노즈로 빠진다.",),
        stuck=ac.PHOTO,
        adds=(TRY_FOOT, TRY_END),
        mount=(TRY_FOOT, TRY_END),
        removes=(TRY_FOOT, TRY_END),
        focus=(ARM, TRY_FOOT, TRY_END),
        uses=_TRY_USES,
        moves=(
            Move((TRY_FOOT,), (0.0, 0.0, 24.0)),
            Move((TRY_END,), (0.0, 0.0, -24.0), 0.8),
        ),
        view=ac.link_view("j4", (0, 0, 1), (0, 1, 0.4), 30),
        labels=(
            Label(TRY_FOOT, "발 귀의 구멍 (J3 핀)"),
            Label(TRY_END, "끝 집 살의 구멍 (J4 핀)"),
        ),
    ),
)

J4_STEPS: tuple[Step, ...] = (
    Step(
        "disc-prep",
        "원판 준비",
        "J4 출력 원판에 너트와 멈춤 기둥 달기 (손에 들고)",
        do=(
            "출력 원판 테의 귀에서 나사 머리 자리 바닥의 얇은 막을 육각렌치 끝으로 눌러 뚫습니다.",
            "원판 밑면(가운데가 솟은 쪽)의 육각 홈 넷에 M3 너트를 엄지로 끼웁니다. 내일 손목 1의 발 나사가 여기에 물립니다.",
            "귀 위에서 M3 × 10을 꽂고, 밑으로 나온 끝에 너트를 끼워 육각렌치로 조입니다.",
        ),
        check=(
            "너트 넷이 홈에 들어가 면보다 튀어나오지 않는다.",
            "멈춤 기둥 끝의 너트가 손으로 안 돈다.",
        ),
        stuck=ac.PHOTO,
        adds=DISC_SET,
        focus=DISC_SET,
        uses=(Use("hexkey"),),
        moves=(
            Move((DISC_NUTS,), ac.vec("j4", (0.0, -10.0, 0.0))),
            Move(("J4 멈춤 나사",), ac.vec("j4", (0.0, 16.0, 0.0)), 0.8),
            Move(("J4 너트 (멈춤 기둥)",), ac.vec("j4", (0.0, -12.0, 0.0)), 1.4),
        ),
        view=ac.link_view("j4", (0, -1, 0), (1, 0, 0.3), 45),
        labels=(
            Label(f"{DISC_NUTS} 1", "M3 너트 × 4 (밑면)"),
            Label("J4 멈춤 나사", "M3 × 10 (귀)"),
        ),
    ),
    Step(
        "carrier",
        "받침 달기",
        f"서보(ID {SERVO_ID})에 받침 달기",
        do=(
            "서보 받침의 날개 둘 바깥 돋움의 옆 홈에 M3 너트를 하나씩 세워 밀어 넣습니다.",
            f"ID {SERVO_ID} 서보를 혼이 아래로 가게 엎어 놓고, 받침을 날개가 아래(혼 쪽)로 가게 들어 판을 서보 뒷면에 얹습니다.",
            "서보 뒷면 자가탭 넷을 작은 십자 드라이버로 조입니다. 세게 조이지 않습니다.",
        ),
        warn=("서보를 넣은 뒤에는 받침 너트에 손이 닿지 않습니다. 지금 끝까지 들어갔는지 봅니다.",),
        check=(
            "판이 서보 뒷면에 붙어 흔들리지 않는다.",
            "돋움 구멍으로 너트 구멍이 가운데에 보인다 (둘 다).",
        ),
        stuck=ac.PHOTO,
        adds=SERVO_SET,
        focus=SERVO_SET,
        uses=(Use("driver"),),
        moves=(
            Move((f"{CAR_NUTS} 1",), _slide("서보 받침 +z")),
            Move((f"{CAR_NUTS} 2",), _slide("서보 받침 -z"), 0.6),
            Move((CAR, CAR_NUTS), ac.vec("j4", (0.0, -25.0, 0.0)), 1.2),
            Move((REAR,), ac.vec("j4", (0.0, -12.0, 0.0)), 2.2),
        ),
        view=ac.link_view("j4", (0, -1, 0), (0, 0, 1), 35),
        labels=(Label(SERVO, f"STS3215 · ID {SERVO_ID}"), Label(f"{REAR} 1", "자가탭 × 4")),
    ),
    Step(
        "bearing",
        "베어링",
        "J4 집에 베어링 넣기",
        do=(
            "아래팔을 납작한 면이 아래로 가게 위팔 끝 옆의 수건 위에 눕힙니다. 끝 집의 열린 뒤가 위를 봅니다.",
            "베어링을 열린 뒤로 넣고, 바깥 링을 엄지로 눌러 앞 끝의 턱에 닿을 때까지 밉니다.",
        ),
        warn=("바깥 링만 누릅니다.",),
        check=("베어링이 턱에 닿아 기울지 않았다.", "안쪽 링이 부드럽게 돈다."),
        stuck=ac.PHOTO,
        adds=(ARM, BRG),
        still=(ARM,),
        focus=(ARM, BRG),
        mount=(ARM, BRG),
        uses=(Use("towel"),),
        moves=(Move((BRG,), ac.vec("j4", (0.0, -62.0, 0.0))),),
        view=BACK_UP,
        shell=(ARM,),
        cut=True,
        labels=(Label(BRG, "베어링 6806"),),
    ),
    Step(
        "hub-in",
        "허브 판",
        "허브 판 먼저 넣기",
        do=(
            "허브 판을 가운데가 볼록한 쪽이 위로 가게 열린 뒤로 떨어뜨려 넣어, 베어링 안쪽 링 위에 얹습니다.",
        ),
        warn=("누름 틀보다 먼저 넣습니다 (허브 판이 누름 틀 다리 사이보다 큽니다).",),
        check=("허브 판이 기울지 않고 안쪽 링 위에 얹혔다.",),
        stuck=ac.PHOTO,
        adds=(HUB,),
        focus=(ARM, BRG, HUB),
        mount=(HUB,),
        uses=(Use("phone", note="속이 어두우면 불빛", count=False),),
        moves=(Move((HUB,), ac.vec("j4", (0.0, -45.0, 0.0))),),
        view=BACK_UP,
        shell=(ARM,),
        cut=True,
        labels=(Label(HUB, "허브 판 (볼록한 쪽이 위)"),),
    ),
    Step(
        "ret-nuts",
        "틀 너트",
        "누름 틀에 너트 넣기 (손에 들고)",
        do=("누름 틀을 고리가 아래로 놓고, 다리 셋의 옆 홈에 M3 너트를 세워 홈 끝까지 밉니다.",),
        check=("다리 바깥 구멍으로 너트 구멍이 가운데에 보인다 (셋 다).",),
        stuck=ac.PHOTO,
        adds=(RET, RET_NUTS),
        still=(RET,),
        focus=(RET, RET_NUTS),
        uses=(Use("hexkey", note="뻑뻑하면 긴 쪽으로 민다", count=False),),
        moves=(
            Move((f"{RET_NUTS} 1",), _slide("누름 틀 +z")),
            Move((f"{RET_NUTS} 3",), _slide("누름 틀 +x"), 0.8),
            Move((f"{RET_NUTS} 2",), _slide("누름 틀 -z"), 1.6),
        ),
        view=ac.link_view("j4", (0, -1, 0), (1, 0, 0), 26),
        labels=(Label(f"{RET_NUTS} 3", "M3 너트 × 3"),),
    ),
    Step(
        "ret-in",
        "틀 넣기",
        "누름 틀 넣고 앞줄 벽 나사 셋 조이기",
        do=(
            "누름 틀을 고리가 아래로, 다리가 없는 쪽이 관 쪽으로 가게 열린 뒤로 넣어 고리가 베어링에 닿게 합니다.",
            "누른 채 앞줄(아래쪽 줄) 벽 구멍 셋에 M3 × 8을 넣고 조입니다.",
        ),
        warn=("뒷줄 구멍 둘은 서보를 잠글 때 씁니다.",),
        check=("앞줄 나사 셋의 머리가 벽에 닿았다.", "흔들어도 베어링이 달그락거리지 않는다."),
        stuck=ac.PHOTO,
        adds=FRONT_ROW,
        focus=(ARM, BRG, HUB, RET, RET_NUTS, *FRONT_ROW),
        mount=(RET, RET_NUTS, *FRONT_ROW),
        uses=(Use("hexkey"),),
        moves=(
            Move((RET, RET_NUTS), ac.vec("j4", (0.0, -58.0, 0.0))),
            Move(("J4 벽 나사 1$",), ac.vec("j4", (0.0, 0.0, 14.0)), 1.4),
            Move(("J4 벽 나사 5$",), ac.vec("j4", (14.0, 0.0, 0.0)), 1.8),
            Move(("J4 벽 나사 3$",), ac.vec("j4", (0.0, 0.0, -14.0)), 2.2),
        ),
        view=BACK_UP,
        shell=(ARM,),
        ghost=True,
        labels=(
            Label(RET, "누름 틀 (다리 없는 쪽이 관 쪽)"),
            Label("J4 벽 나사 5$", "M3 × 8 × 3 (앞줄)"),
        ),
    ),
    Step(
        "in-cable",
        "선 하나",
        "위팔에서 온 J3 → J4 선을 아래팔 관 속으로 꿰기",
        do=(
            "위팔 끝의 선 창 밖으로 빼 둔 선(약 35 cm)의 플러그를 아래팔 발 바닥의 큰 구멍에 넣어, 관 속을 따라 끝 집까지 밀어 "
            "넣습니다.",
            "끝 집의 열린 뒤로 플러그를 빼내 손가락 한 마디쯤 나오게 둡니다.",
        ),
        warn=("서보를 넣으면 플러그가 지나갈 틈이 없습니다. 꼭 서보보다 먼저 꿰입니다.",),
        check=(
            "플러그가 끝 집의 열린 뒤로 나와 있다.",
            "위팔 창과 아래팔 발 사이의 선이 팽팽하지 않다.",
        ),
        stuck="플러그가 관 속에서 걸리면 아래팔을 세워 흔들거나, 끝 집 쪽에서 핀셋으로 당깁니다.",
        adds=(IN_ARM,),
        focus=(ARM, BRG, HUB, RET, RET_NUTS, *FRONT_ROW, IN_ARM),
        mount=(IN_ARM,),
        uses=(Use("servo_cable_900", note="위팔 창 밖의 끝", count=False),),
        view=ac.link_view("j4", (0, -1, 0), (0.2, 0, 1), 20),
        shell=(ARM,),
        ghost=True,
        labels=(
            Label(
                IN_ARM,
                "J3 → J4 선 (발에서 관 속으로)",
                ac.pt("j4", (-100.0, ac.tube_center_y("forearm"), 0.0)),
            ),
        ),
        tags=(Tag(_FOOT, "발의 큰 구멍"),),
    ),
    Step(
        "out-cable",
        "선 둘",
        "J4 → J5 선(300 mm)을 선 창으로 빼기",
        do=(
            "300 mm 선의 한쪽 플러그를 끝 집의 열린 뒤에서 목 쪽으로 넣어, 목 앞면의 선 창으로 밖에 빼냅니다.",
            "다른 플러그가 열린 뒤에 손가락 한 마디쯤 남을 때까지 창 밖으로 당깁니다. 창 밖에 약 20 cm가 남습니다.",
        ),
        warn=("창 밖의 끝은 내일 손목에 꽂습니다. 오늘은 늘어뜨려 둡니다.",),
        check=("창 밖으로 약 20 cm, 열린 뒤로 플러그 하나가 나와 있다.",),
        stuck="창이 작아 플러그가 안 나오면 플러그를 세워 넣어 봅니다.",
        adds=(OUT_ARM, OUT_TAIL, OUT_END),
        focus=(ARM, BRG, HUB, RET, RET_NUTS, *FRONT_ROW, IN_ARM, OUT_ARM, OUT_TAIL, OUT_END),
        mount=(OUT_ARM, OUT_TAIL, OUT_END),
        uses=(Use("servo_cable_300", 1, "J4 → J5"),),
        view=ac.link_view("j4", (0, -1, 0), (0.2, 0, 1), 20),
        shell=(ARM,),
        ghost=True,
        labels=(Label(OUT_TAIL, "선 창 밖으로 약 20 cm"),),
        tags=(Tag(_WIN_OUT, "선 창"),),
    ),
    Step(
        "servo-in",
        "서보 넣기",
        f"서보(ID {SERVO_ID}) 넣기",
        do=(
            "서보를 혼이 아래로, 긴 쪽(혼에서 먼 끝)이 관 쪽으로 가게 들고 열린 뒤로 넣습니다.",
            "넣는 동안 선 둘을 목 끝 벽(관 쪽 벽)에 붙여 눕히고, 두 플러그는 열린 뒤 밖으로 잡아 둡니다.",
            "혼이 허브 판의 볼록한 곳에 닿을 때까지 내립니다.",
        ),
        warn=("선이 서보와 벽 사이에 끼면 멈추고 서보를 조금 빼서 다시 눕힙니다.",),
        check=(
            "서보가 기울지 않고 끝까지 들어갔다.",
            "선 둘이 끼지 않고 플러그 둘이 열린 뒤로 나와 있다.",
        ),
        stuck=ac.PHOTO,
        mount=SERVO_SET,
        focus=(
            ARM,
            BRG,
            HUB,
            RET,
            RET_NUTS,
            *FRONT_ROW,
            IN_ARM,
            OUT_ARM,
            OUT_TAIL,
            OUT_END,
            *SERVO_SET,
        ),
        uses=(Use(SERVO_ITEM.key, note="받침을 단 것", count=False),),
        moves=(Move(SERVO_SET, ac.vec("j4", (0.0, -70.0, 0.0))),),
        view=BACK_UP,
        shell=(ARM,),
        cut=True,
        labels=(Label(SERVO, f"ID {SERVO_ID}"),),
    ),
    Step(
        "disc-on",
        "원판 끼우기",
        "뒤집어 출력 원판 끼우기",
        do=(
            "서보 받침을 엄지로 눌러 잡은 채 아래팔을 뒤집어 끝 집의 앞쪽(납작한 면)이 위로 오게 합니다.",
            "허브 판의 구멍 넷이 혼의 나사 구멍과 맞게 육각렌치 끝으로 돌려 맞춥니다.",
            "출력 원판을 귀가 관 반대쪽(바깥 끝)에 오게 들고, 가운데 볼록한 곳을 베어링 안쪽 링에 끼워 끝까지 밉니다.",
        ),
        warn=("귀를 관 쪽에 두면 멈춤 기둥이 살에 걸려 안 들어갑니다.",),
        check=("원판이 기울지 않고 끝까지 들어갔다.", "원판 구멍 넷으로 혼의 나사 구멍이 보인다."),
        stuck=ac.PHOTO,
        mount=DISC_SET,
        focus=(ARM, BRG, HUB, RET, RET_NUTS, *FRONT_ROW, *SERVO_SET, *DISC_SET),
        uses=(Use("out_disc", note="너트와 멈춤 기둥을 단 것", count=False),),
        moves=(Move(DISC_SET, ac.vec("j4", (0.0, 40.0, 0.0))),),
        view=FRONT_UP,
        labels=(Label(DISC, "출력 원판 (귀가 바깥 끝)"),),
    ),
    Step(
        "horn-bolts",
        "혼 볼트",
        "혼 볼트 넷 조이기",
        do=("M3 × 20 넷을 걸어 둔 뒤 대각선 차례로 조금씩, 머리가 원판에 닿을 때까지 조입니다.",),
        warn=("넷이 다 걸릴 때까지 받침을 누른 손을 떼지 않습니다.",),
        check=("볼트 넷의 머리가 원판에 닿았다.", "원판이 흔들리지 않는다."),
        stuck=ac.PHOTO,
        adds=(BOLTS,),
        focus=(ARM, BRG, HUB, RET, RET_NUTS, *FRONT_ROW, *SERVO_SET, *DISC_SET, BOLTS),
        mount=(BOLTS,),
        uses=(Use("hexkey"),),
        moves=tuple(
            Move((f"{BOLTS} {k}$",), ac.vec("j4", (0.0, 22.0, 0.0)), 0.45 * (k - 1))
            for k in range(1, 5)
        ),
        view=FRONT_UP,
        labels=(Label(f"{BOLTS} 1$", "M3 × 20 × 4"),),
    ),
    Step(
        "lock",
        "서보 잠금",
        "다시 뒤집어 뒷줄 벽 나사 둘로 서보 잠그기",
        do=("아래팔을 다시 납작한 면이 아래로 눕히고, 뒷줄 구멍 둘에 M3 × 8을 넣어 조입니다.",),
        check=("나사 둘의 머리가 벽에 닿았다.", "서보를 뒤에서 밀어도 움직이지 않는다."),
        stuck=ac.PHOTO,
        adds=BACK_ROW,
        focus=(
            ARM,
            BRG,
            HUB,
            RET,
            RET_NUTS,
            *FRONT_ROW,
            *SERVO_SET,
            *DISC_SET,
            BOLTS,
            *BACK_ROW,
            IN_ARM,
            OUT_ARM,
        ),
        mount=BACK_ROW,
        uses=(Use("hexkey"),),
        moves=(
            Move(("J4 벽 나사 2$",), ac.vec("j4", (0.0, 0.0, 14.0))),
            Move(("J4 벽 나사 4$",), ac.vec("j4", (0.0, 0.0, -14.0)), 0.7),
        ),
        view=BACK_UP,
        labels=(Label("J4 벽 나사 2$", "M3 × 8 × 2 (뒷줄)"),),
    ),
    Step(
        "plug",
        "선 꽂기",
        f"선 둘을 서보(ID {SERVO_ID})에 꽂기",
        do=(
            "관에서 온 선(J3 → J4)의 플러그를 그림의 ① 커넥터에, 선 창으로 나가는 선(J4 → J5)의 플러그를 ② 커넥터에 꽂습니다.",
            "두 선을 받침 판 위에 납작하게 눕혀 서보 먼 끝(관 쪽)으로 보냅니다.",
        ),
        warn=("선이 판 위로 들뜨면 뒷덮개가 뜹니다.",),
        check=("두 플러그가 끝까지 꽂혀 살짝 당겨도 안 빠진다.", "두 선이 판 위에 눕혀져 있다."),
        stuck="어느 커넥터인지 헷갈리면 사진을 보내 주세요.",
        adds=(IN_PLUG, OUT_PLUG),
        focus=(ARM, *SERVO_SET, *BACK_ROW, IN_ARM, OUT_ARM, IN_PLUG, OUT_PLUG),
        mount=(IN_PLUG, OUT_PLUG),
        uses=(Use("servo_cable_900", note="열린 뒤로 나온 플러그", count=False),),
        moves=(
            Move((IN_PLUG,), ac.vec("j4", (0.0, -12.0, 0.0))),
            Move((OUT_PLUG,), ac.vec("j4", (0.0, -12.0, 0.0)), 0.8),
        ),
        view=ac.link_view("j4", (0, -1, 0), (-0.3, 0, 1), 40),
        zoom=1.6,
        center=_J4,
        labels=(Label(IN_PLUG, "① 관에서 온 선"), Label(OUT_PLUG, "② 선 창으로 나가는 선")),
    ),
    Step(
        "cover",
        "뒷덮개",
        "링크 뒷덮개 달고 J4 돌려 보기",
        do=(
            "링크 뒷덮개를 얹고 M3 × 12 둘을 조입니다.",
            "출력 원판을 손으로 천천히 양쪽 끝까지 돌려 본 뒤 가운데로 되돌립니다.",
        ),
        warn=("선이 덮개 테 밑에 끼이지 않게 봅니다.",),
        check=("덮개가 들뜨지 않았다.", "원판이 양 끝까지 걸림 없이 돈다."),
        stuck=ac.PHOTO,
        adds=(COVER, COVER_SCREWS),
        focus=(*IN_HAND, COVER, COVER_SCREWS),
        mount=(COVER, COVER_SCREWS),
        uses=(Use("hexkey"),),
        moves=(
            Move((COVER,), ac.vec("j4", (0.0, -30.0, 0.0))),
            Move((COVER_SCREWS,), ac.vec("j4", (0.0, -16.0, 0.0)), 1.0),
        ),
        view=BACK_UP,
        labels=(Label(COVER, "링크 뒷덮개"), Label(f"{COVER_SCREWS} 1", "M3 × 12 × 2")),
    ),
    Step(
        "foot-route",
        "발의 선 자리",
        "J3 → J4 선을 아래팔 발의 높은 창에 눕히기",
        do=(
            "아래팔 발 바닥의 큰 구멍에서 끝벽 쪽으로 난 긴 틈에, 발 쪽으로 나온 선을 옆에서 밀어 넣어 끝벽 밑의 낮은 구멍으로 "
            "뺍니다.",
            "끝벽의 세로 틈을 따라 선을 세워 밀어 올린 뒤 높은 창(①)에 눕힙니다.",
        ),
        warn=("플러그를 좁은 구멍에 꿰지 않습니다. 선만 옆에서 밀어 넣습니다.",),
        check=("선이 발 바닥의 틈을 지나 끝벽의 높은 창으로 나와 있다.",),
        stuck=ac.PHOTO,
        focus=(*IN_HAND, COVER, COVER_SCREWS),
        uses=(Use("servo_cable_900", note="발 쪽의 J3 → J4 선", count=False),),
        view=FOOT_VIEW,
        zoom=1.8,
        center=_FOOT,
        spots=(Spot(ARM, "①", (_FOOT_WIN,)),),
        tags=(Tag(_FOOT, "발 바닥의 큰 구멍"),),
    ),
)

ATTACH_STEPS: tuple[Step, ...] = (
    Step(
        "attach",
        "아래팔 달기",
        "아래팔을 J3 출력 원판에 달기 (M3 × 10 넷)",
        do=(
            "누워 있는 위팔 끝의 J3 출력 원판(로봇 오른쪽 면)에 아래팔 발을 얹습니다. 발 끝의 핀 귀가 원판 귀(멈춤 기둥)의 "
            "반대쪽에 오게 합니다. 아래팔 관은 원판 귀 쪽으로 뻗습니다.",
            "발의 구멍 넷을 원판의 너트 넷에 맞추고, M3 × 10 넷을 대각선 차례로 조입니다.",
        ),
        warn=("발 바닥의 틈에 넣은 선이 원판과 발 사이에 끼지 않게 봅니다.",),
        check=("발이 원판에 들뜨지 않고 앉았다.", "나사 넷의 머리가 다 닿았다."),
        stuck=ac.PHOTO,
        adds=(HAVE, ARM_SCREWS, IN_ACROSS),
        still=(HAVE, IN_ACROSS),
        uses=(Use("hexkey"), Use("forearm", note="J4를 조립한 것", count=False)),
        moves=(
            Move((*IN_HAND, COVER, COVER_SCREWS), (0.0, 0.0, 45.0)),
            Move((ARM_SCREWS,), (0.0, 0.0, 14.0), 1.4),
        ),
        view=FROM_RIGHT,
        zoom=1.3,
        center=_J3,
        labels=(Label(f"{ARM_SCREWS} 1", "M3 × 10 × 4"), Label(ARM, "아래팔")),
    ),
    Step(
        "turn-j3",
        "돌려 보기",
        "아래팔을 손으로 돌려 선 보기",
        do=(
            "위팔을 곧게 세워 한 손으로 받치고, 다른 손으로 아래팔을 천천히 앞으로 숙였다가 뒤로 젖혀 봅니다 (위팔에 닿기 전까지).",
            "위팔 창에서 아래팔 발로 가는 선이 끼거나 팽팽해지지 않는지 봅니다. 아래팔을 곧게 세워 둡니다.",
        ),
        warn=("선이 걸리면 억지로 돌리지 말고 사진을 보내 주세요.",),
        check=("돌리는 동안 선이 걸리거나 당겨지지 않았다.",),
        uses=(Use("phone", note="걸리면 사진", count=False),),
        turn2=(-120.0, 120.0),
        view=FROM_RIGHT,
    ),
)

ZERO_STEPS: tuple[Step, ...] = (
    Step(
        "power-on",
        "전원 넣기",
        "서보 전원 넣고 위팔을 J2 핀으로 세우기",
        do=(
            "검정 플러그를 − 단자에 꽂고 가변 전원을 켜 12.0 V를 본 뒤, 빨강 플러그를 + 단자에 꽂습니다. 전류를 알려 주세요.",
            f"Claude가 서보 넷(ID 1 ~ {SERVO_ID})을 읽습니다.",
            "위팔을 곧게 세우고, 어제처럼 위팔 발의 귀에 영점 핀을 꽂아 둡니다. 핀이 위팔을 곧게 잡아 줍니다.",
        ),
        warn=(
            "아무것도 움직이지 않아야 합니다. 움직이거나 C.C 표시등이 켜지면 바로 빨강 플러그를 뺍니다.",
            "핀을 꽂아도 팔을 놓지 않습니다. 아래팔은 계속 한 손으로 받칩니다.",
        ),
        check=(
            "12.0 V이고 C.C 표시등이 꺼져 있다.",
            "Claude가 서보 넷을 읽었다고 했다.",
            "J2 핀이 꽂혀 위팔이 서 있다.",
        ),
        stuck="화면을 사진으로 찍어 보내 주세요.",
        adds=(PIN2,),
        uses=(Use("psu"), Use("pc")),
        moves=(Move((PIN2,), (0.0, 0.0, -24.0)),),
        view=FROM_RIGHT,
        labels=(Label(PIN2, "J2 영점 핀 (위팔을 잡는다)"),),
    ),
    Step(
        "direction",
        "방향 재기",
        "J3를 + 쪽으로 돌려 방향 재기",
        do=(
            "Claude가 PC에 영점 창을 띄웁니다. 창에 나오는 말대로 합니다.",
            '"영점 핀을 빼고 …"가 나오면 그대로 Enter를 누릅니다.',
            '"+ 쪽으로 20°쯤 돌린 채 Enter"가 나오면, 아래팔을 앞(책상 쪽)으로 20°쯤 숙인 채 Enter를 누릅니다.',
        ),
        warn=("아래팔을 놓지 않습니다.",),
        check=('창에 "방향"이 나왔다.',),
        stuck="창에 나온 글을 사진으로 보내 주세요.",
        uses=(Use("pc", note="영점 창"),),
        turn2=(0.0, 20.0),  # J3의 설계 부호가 +라 운동학 각과 같다
        view=FROM_RIGHT,
        labels=(Label(ARM, "+ 쪽: 아래팔이 앞으로 숙는 쪽"),),
    ),
    Step(
        "pin-in",
        "핀 꽂기",
        "J3 영점 핀 꽂고 Enter",
        do=(
            "아래팔을 위팔과 일직선으로 곧게 세웁니다.",
            "아래팔 발 끝의 핀 귀(위팔 쪽 아래, 원판 밖으로 나온 귀)의 작은 구멍에 영점 핀을 꽂습니다. 위팔 끝 집의 살까지 쑥 "
            "들어가면 맞습니다. 안 들어가면 아래팔을 조금씩 앞뒤로 돌려 맞춥니다.",
            '"영점 핀을 꽂고 …"가 나오면, 아래팔을 앞으로 살짝 민 채 Enter를 누릅니다. 창에 세 줄이 나오면 됩니다.',
        ),
        warn=("핀을 공구로 박지 않습니다.",),
        check=("핀이 귀 위로 3 mm쯤 나와 있다.", "창에 세 줄이 나왔다."),
        stuck="핀을 꽂은 모습과 창을 사진으로 보내 주세요.",
        adds=(PIN3,),
        uses=(Use("pc", note="영점 창"),),
        moves=(Move((PIN3,), (0.0, 0.0, 24.0)),),
        view=FROM_RIGHT,
        zoom=1.8,
        center=_PIN_AT,
        labels=(Label(PIN3, "J3 영점 핀 (아래팔 발의 귀)"),),
    ),
    Step(
        "apply",
        "한계 굽기",
        "각도 한계 굽기 (Claude)",
        do=(
            "Claude가 세 줄을 로봇 설정에 옮기고, 잰 방향을 설계로 본 방향(+1)과 견줍니다.",
            "값 적용을 돌려도 되는지 물으면 답합니다. 서보는 움직이지 않습니다.",
        ),
        check=("Claude가 값 적용이 끝났다고 했다.",),
        uses=(Use("pc"),),
        view=FROM_RIGHT,
        zoom=1.8,
        center=_PIN_AT,
    ),
    Step(
        "pin-out",
        "핀 빼기",
        "J3 영점 핀 빼기",
        do=("아래팔을 받친 채 롱노즈로 J3 핀을 곧게 당겨 뺍니다. J2 핀은 그대로 둡니다.",),
        check=("J3 핀을 뺐다.",),
        uses=(Use("longnose"),),
        view=FROM_RIGHT,
        zoom=1.8,
        center=_PIN_AT,
        labels=(Label(PIN3, "곧게 당겨 뺀다"),),
    ),
    Step(
        "check",
        "확인",
        "아래팔을 앞으로 90° 숙여 읽어 보기",
        do=(
            "아래팔을 앞으로 숙여 수평(90°쯤)으로 받쳐 듭니다.",
            "Claude가 J3 각을 읽습니다. +90°쯤이면 맞습니다.",
        ),
        check=("Claude가 +90°쯤으로 읽었다고 했다.",),
        uses=(Use("pc"),),
        hide=(PIN3,),
        turn2=(0.0, 90.0),
        view=FROM_RIGHT,
        labels=(Label(ARM, "앞으로 수평 (+90°)"),),
    ),
    Step(
        "power-off",
        "전원 끄기",
        "전원 끄고 J2 핀 빼기",
        do=(
            "빨강 플러그를 뺀 뒤 가변 전원을 끕니다. Claude가 기록을 올립니다.",
            "팔을 받친 채 J2 핀을 롱노즈로 곧게 당겨 뺍니다.",
        ),
        warn=("핀을 빼면 팔이 넘어갑니다. 두 손으로 받칩니다.",),
        check=("가변 전원이 꺼졌다.", "J2 핀을 뺐고 팔을 받치고 있다."),
        uses=(Use("psu"), Use("longnose")),
        hide=(PIN3, PIN2),
        view=FROM_RIGHT,
    ),
)


def _rest_step(sid: str, kind: str = "work") -> Step:
    rest = Step(
        sid,
        "쉬는 자세",
        "쉬는 자세로 선반에 얹기",
        kind=kind,
        do=(
            "J1 몸통을 위에서 볼 때 시계 방향으로 55°쯤 돌립니다.",
            "위팔을 앞으로 50°쯤 숙이며 아래팔을 위팔 쪽으로 접어, 아래팔 끝 집이 J1 몸통의 선반에 얹히게 합니다.",
            "창 밖의 J4 → J5 선은 늘어뜨려 둡니다. 내일 손목에 꽂습니다.",
            "다 된 모습을 사진으로 보내 주세요.",
        ),
        check=("아래팔 끝이 선반에 얹혀 팔이 스스로 서 있다.",) if kind == "work" else (),
        adds=(REST,) if kind == "work" else (),
        uses=(Use("phone", note="다 된 모습", count=False),) if kind == "work" else (),
        hide=(*upright_names(), PIN2, PIN3),
        joint=ac.REST_J1,
        view=View((0, 1, 0), (1, 0, 0.6), 20, 0),
    )
    return rest


STEPS: tuple[Step, ...] = (
    PREP,
    *PIN_TRY_STEPS,
    *J4_STEPS,
    *ATTACH_STEPS,
    *ZERO_STEPS,
    _rest_step("rest"),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            f"아래팔(J4, ID {SERVO_ID})을 달고 J3(ID {J3_ID})의 영점을 구웠습니다.",
            "다음 안내는 손목 잇기입니다. 창 밖의 J4 → J5 선을 손목 1에 꽂습니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요.",
        ),
        hide=(*upright_names(), PIN2, PIN3),
        joint=ac.REST_J1,
        view=View((0, 1, 0), (1, 0, 0.6), 20, 0),
    ),
)

GUIDE = Guide(
    id="forearm",
    kind="assembly",
    title="아래팔과 J3 영점",
    sub=f"J4 STS3215 ID {SERVO_ID} · 선 둘 · 아래팔 달기 · J3(ID {J3_ID}) 영점 · 쉬는 자세",
    scene=scene,
    steps=STEPS,
    items=ITEMS_HERE,
    bundle="arm-black-1",
    printed_before=("forearm",),
    axis2=J3_AXIS,
)
