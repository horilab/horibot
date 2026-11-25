"""위팔 조립과 J2 영점: 위팔 끝의 J3(STS3250, ID 3)과 선 둘, 위팔 달기, 선 누르개 셋, J2(ID 2) 영점 (0029, 0031, 0034).

순서는 형상으로 따졌다 (2026-11-25):
- J3는 손목 관절과 같은 STS 관절이다. 허브 판(지름 33 mm)이 누름 틀 다리 사이(30 mm)를 곧게 지나가지 못해서, 어깨처럼
  허브 판을 누름 틀보다 먼저 넣는다 (첫 관절에서는 떨어뜨려 넣었다).
- 서보를 넣으면 서보 먼 끝과 목 끝 벽 사이가 2.3 mm라 플러그가 지나가지 못한다 (arm_common). 그래서 어댑터 → J3 선
  (관 속으로 온다)과 J3 → J4 선(선 창으로 나간다)을 서보보다 먼저 꿴다. J3 → J4 선의 남는 길이(약 45 cm)는 접어서 위팔 관
  가운데에 둔다 (0031: 끝 집 쪽에 두면 어깨가 정격의 0.8배를 넘는 칸이 생긴다). 창 밖으로 약 35 cm가 남아 내일 아래팔로
  간다.
- 위팔 발의 선 자리는 발 겉으로 도는 길(끝벽의 높은 창)을 쓴다 (0031의 J2·J3 주된 길). 링크를 원판에 얹기 전에 선을 발
  바닥의 긴 틈으로 옆에서 밀어 넣는다.
- 누르개 셋은 J1 → J2 선(어제 길에 놓아 둔 것)과 어댑터 → J3 선을 같이 잡는다.
- J2 영점은 위팔을 곧게 세운 범위 가운데(0°)에서 위팔 발의 귀와 어깨의 살에 핀을 꽂는다. + 쪽은 위팔이 앞으로 숙는 쪽이다
  (축이 로봇 왼쪽을 본다). 설계로 본 방향은 -1이다 (ST-3120의 서보 값이 느는 쪽은 이날 잰 방향으로 확인한다).
- 다 하면 위팔을 앞으로 눕혀(약 125°) 책상의 수건 위에 둔다. 전원이 없으면 서지 않는다.

장면 좌표는 J1의 서보 좌표다 (arm_common). 둘째 관절은 J2다.
"""

from __future__ import annotations

from dataclasses import replace

from horibot_design.guide.catalog import printed_items
from horibot_design.guide.guides import arm_common as ac
from horibot_design.guide.guides import j2_shoulder as j2s
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
from horibot_design.parts import base as b
from horibot_design.parts import joint as j
from horibot_design.parts import links, wire_clamps, wrist
from horibot_design.parts import pi_mount as pm
from horibot_design.parts import shoulder as sh
from horibot_design.studies import wiring as w

SERVO_ID = 3
J2_ID = 2
MODEL = "STS3250"
AXIS_JOINT = 2
HAVE = "있는 것/"

DONE = Item(
    "assembled_shoulder", "조립해 둔 베이스와 어깨", "bought", "J1(ID 1), 어깨(ID 2), 파이와 어댑터"
)
SERVO_ITEM = Item(
    "servo_sts3250_j3",
    f"서보 STS3250 · ID {SERVO_ID}",
    "bought",
    f"혼이 달린 것. ID {SERVO_ID} 이름표가 붙은 서보 (팔꿈치, J3)",
)
PRINTED: dict[str, Item] = printed_items(
    {
        **wrist.LABELS,
        **links.LABELS,
        **wire_clamps.LABELS,
        "base_bearing_pusher": b.LABELS["base_bearing_pusher"],
    },
    {
        "upper_arm": (None, None, "white"),
        **{
            k: (None, None, "black")
            for k in (
                "retainer",
                "hub_plate",
                "out_disc",
                "carrier",
                "link_cover",
                "wire_clamp_band",
                "wire_clamp_ear",
                "wire_clamp_strip",
                "base_bearing_pusher",
            )
        },
    },
)
OTHER: dict[str, Item] = {
    it.key: it
    for it in (
        DONE,
        SERVO_ITEM,
        Item(
            "m2x5",
            "M2 × 5 십자",
            "hardware",
            "둥근머리 십자볼트 (가진 것). STS3250 뒷면",
            draw={"t": "tap", "L": 5},
        ),
        Item(
            "servo_cable_900",
            "서보 선 900 mm",
            "bought",
            "Waveshare 5264 3핀 (둘을 쓴다)",
            icon="wire",
        ),
        Item("tape", "절연 테이프", "tool", "접은 선을 묶는다", icon="tape"),
        Item("stick", "젓가락이나 연필", "tool", "접은 선을 관 속으로 민다", icon="pen"),
        Item("towel", "수건", "tool", "다 한 뒤 위팔을 눕혀 둔다"),
        Item("psu", "가변 전원", "tool", "전류 상한 3 A로 맞춰 둔 것"),
        Item("pc", "집 PC", "tool", "USB 5 m 선이 꽂힌 것. Claude가 명령을 돌린다"),
        Item("longnose", "롱노즈", "tool", "핀을 곧게 당겨 뺀다", icon="tweezers"),
        Item("phone", "휴대폰", "tool", "사진을 찍어 대화창에 보낸다", icon="phone"),
    )
}
ITEMS_HERE: dict[str, Item] = {**PRINTED, **OTHER}

J3_AXIS = ac.axis_of("j3")
J2_AXIS = ac.axis_of("j2")
_YC = ac.tube_center_y("upper_arm")
_SPARE_X = -115.0  # 접은 J4 선의 끝 (관 가운데쯤, 끝 집 목에서 7 cm)


def _have() -> list[ScenePart]:
    out = []
    for p in j2s.scene():
        name = p.name if p.name.startswith(HAVE) else f"{HAVE}{p.name}"
        out.append(replace(p, name=name, key=DONE.key, count=False))
    return out


def pieces() -> list[ac.Piece]:
    """위팔 묶음과 선, 누르개, 영점 핀 (범위 가운데 자세)."""
    out = ac.link_pieces("upper_arm", SERVO_ITEM.key, "m2x5")
    # 어댑터 → J3 선: J3의 -z 커넥터, 위팔 속, 발의 높은 창 → J1 몸통의 길 → 띠 밑 누르개
    out.append(
        ac.Piece(
            "J3 선 플러그 (J3)", ac.plug(-1, MODEL, "j3"), "servo_cable_900", "plug", "j3", False
        )
    )
    out.append(
        ac.Piece(
            "J3 선 (위팔 속)",
            ac.wire(ac.link_in("upper_arm"), "j3"),
            "servo_cable_900",
            "wire",
            "j3",
            False,
        )
    )
    strip = w.j1_strip()
    clamp = w.core_run()[-1]
    run = [
        tuple(float(c) for c in clamp),
        *(ac.pt("j2", p) for p in strip[:4]),
        ac.pt("j3", ac.link_in("upper_arm")[0]),
    ]
    out.append(
        ac.Piece("베이스 선/J1 몸통", ac.wire_scene(run), "servo_cable_900", "wire", "j2", False)
    )  # type: ignore[arg-type]
    for p in pm.adapter_parts():
        if p.name.endswith("/서보 플러그 1"):
            out.append(
                ac.Piece("베이스 선/어댑터 플러그", p.shape, "servo_cable_900", "plug", "j1", False)
            )
        elif p.name.endswith("/서보 선 (어댑터 → 띠 밑 누르개)"):
            out.append(
                ac.Piece("베이스 선/어댑터 쪽", p.shape, "servo_cable_900", "wire", "j1", False)
            )
    # J3 → J4 선: J3의 +z 커넥터, 남는 길이를 관 가운데에 접어 넣고 선 창으로, 창 밖에 늘어진 끝
    out.append(
        ac.Piece(
            "J4 선 플러그 (J3)", ac.plug(1, MODEL, "j3"), "servo_cable_900", "plug", "j3", False
        )
    )
    inner = ac.link_out_run(_SPARE_X, _YC)
    out.append(
        ac.Piece("J4 선 (위팔 속)", ac.wire(inner, "j3"), "servo_cable_900", "wire", "j3", False)
    )
    tail = [ac.window_point(0.0), ac.window_point(25.0), (-70.0, j.FRONT + 45.0, 0.0)]
    out.append(
        ac.Piece("J4 선 (창 밖)", ac.wire(tail, "j3"), "servo_cable_900", "wire", "j3", False)
    )
    end = ac.pt("j3", tail[-1])
    out.append(
        ac.Piece(
            "J4 선 끝 플러그",
            ac.loose_plug(end, ac.vec("j3", (-1.0, 0.0, 0.0))),
            "servo_cable_900",
            "plug",
            "j3",
            False,
        )
    )
    # 누르개 셋 (띠 밑은 베이스, 귀와 왼쪽 면은 J1과 같이 돈다)
    keys = {"띠 밑": "wire_clamp_band", "귀": "wire_clamp_ear", "왼쪽 면": "wire_clamp_strip"}
    for p in ac.group_pieces("wire_clamps", "m3x8"):
        rest = p.name.split("/", 1)[1]
        if rest.startswith("누르개 나사"):
            where = rest[rest.index("(") + 1 : rest.index(")")]
            out.append(replace(p, name=f"누르개 나사 ({where})", key="m3x8", count=True))
        else:
            where = rest.removesuffix(" 누르개")
            out.append(replace(p, name=rest, key=keys[where], count=True))
    pin = next(
        p for p in sh.child_hardware(links.LINKS["upper_arm"][0]) if p.name.startswith("영점 핀")
    )
    out.append(ac.Piece("영점 핀 (J2)", ac.put(pin.shape, "j2"), "pin_msh3_20", "pin", "j2"))
    return out


def scene() -> list[ScenePart]:
    return _have() + ac.scene_of(pieces(), AXIS_JOINT)


# ------------------------------------------------------------------------------------------------
# 이름

ARM = "위팔$"
BRG = "J3 베어링"
HUB = "J3 허브 판"
DISC = "J3 출력 원판"
RET = "J3 누름 틀"
RET_NUTS = "J3 너트 (누름 틀)"
CAR = "J3 서보 받침"
CAR_NUTS = "J3 너트 (서보 받침)"
SERVO = "J3 서보 몸통"
HORN = "J3 혼$"
REAR = "J3 서보 뒷면 나사"
COVER, COVER_SCREWS = "J3 뒷덮개", "J3 덮개 나사"
DISC_NUTS = "J3 너트 (출력 원판)"
STOP = ("J3 멈춤 나사", "J3 너트 (멈춤 기둥)")
DISC_SET = (DISC, DISC_NUTS, *STOP)
SERVO_SET = (SERVO, HORN, CAR, CAR_NUTS, REAR)
BOLTS = "J3 혼 볼트"
FRONT_ROW = ("J3 벽 나사 1$", "J3 벽 나사 3$", "J3 벽 나사 5$")
BACK_ROW = ("J3 벽 나사 2$", "J3 벽 나사 4$")
IN_PLUG, IN_ARM, IN_BODY = "J3 선 플러그 (J3)", "J3 선 (위팔 속)", "베이스 선/J1 몸통"
IN_AD_PLUG, IN_AD = "베이스 선/어댑터 플러그", "베이스 선/어댑터 쪽"
OUT_PLUG, OUT_ARM, OUT_TAIL, OUT_END = (
    "J4 선 플러그 (J3)",
    "J4 선 (위팔 속)",
    "J4 선 (창 밖)",
    "J4 선 끝 플러그",
)
ARM_SCREWS = "위팔 나사"
CLAMPS = ("띠 밑 누르개", "귀 누르개", "왼쪽 면 누르개")
CLAMP_SCREWS = "누르개 나사"
PIN = "영점 핀 (J2)"
# 위팔과 함께 들고 있는 것 (따로 들고 하는 단계에서 보이는 것). 뒷덮개는 따로 더한다
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
    IN_PLUG,
    IN_ARM,
    OUT_PLUG,
    OUT_ARM,
    OUT_TAIL,
    OUT_END,
)


def _slide(name: str, reach: float = 13.0) -> tuple[float, float, float]:
    """옆 홈에 너트를 밀어 넣는 움직임 (J3 서보 좌표의 홈 입구 쪽에서)."""
    _n, _c, _a, s = next(slot for slot in j.side_nut_slots() if slot[0] == name)
    return ac.vec("j3", (s[0] * reach, s[1] * reach, s[2] * reach))


# 손에 드는 방향 (J3 서보 좌표: 뒤가 -y, 목과 관이 -x)
BACK_UP = ac.link_view(
    "j3", (0, -1, 0), (0.5, 0, 1), 30
)  # 위팔을 납작한 면이 아래로 눕혀, 끝 집의 열린 뒤가 위
FRONT_UP = ac.link_view("j3", (0, 1, 0), (0.5, 0, 1), 30)  # 뒤집어 출력 쪽이 위
FOOT_VIEW = ac.link_view("j3", (0, -1, 0), (-1, 0, 0.6), 35)  # 발 쪽에서
FROM_LEFT = View((0, 1, 0), (0.35, 0, -1), 15, 0)  # 로봇 왼쪽에서 (어깨 출력 원판 쪽)
FROM_BACK_LEFT = View((0, 1, 0), (-0.7, 0, -1), 18, 0)
_J3 = ac.pt("j3", (0.0, 0.0, 0.0))
_FOOT = ac.pt("j3", (-links.LINKS["upper_arm"][0], j.FRONT, 0.0))
_WIN_OUT = ac.pt("j3", ac.window_point(2.0))
_FOOT_WIN = ac.pt("j3", ac.link_in("upper_arm")[1])
_PIN_AT = ac.pt("j2", (-sh.R_PIN, sh.ATOP + j.EAR_T, 0.0))
_AD = (b.ADAPTER_CX + 15.0, b.Y_DESK, 0.0)
_TUBE_MID = ac.pt("j3", (_SPARE_X + 30.0, _YC, 0.0))

PREP = Step(
    "prep",
    "준비물",
    "준비물 꺼내 놓기",
    kind="prep",
    do=(
        "서보 전원이 꺼져 있는지(빨강 플러그를 뺐는지) 봅니다. 서보 선은 전원을 끈 채로 꽂습니다.",
        "출력 부품: 위팔(흰색, 글자를 꽂은 것), 검정 J3 속 부품 다섯(누름 틀, 허브 판, 출력 원판, 서보 받침, 링크 뒷덮개), "
        "누르개 셋(띠 밑, 귀, 왼쪽 면). 베이스용 베어링을 미는 도구도 꺼내 둡니다.",
        f"베어링 6806, ID {SERVO_ID} 이름표가 붙은 STS3250, 서보 선 900 mm 둘, 영점 핀.",
        "나사: M3 × 8 여덟(벽 나사 다섯, 누르개 셋), M3 × 10 일곱(멈춤 기둥 하나, 위팔 여섯), M3 × 12 둘, M3 × 20 넷, "
        "M2 × 5 십자 넷, M3 너트 열.",
    ),
    check=(
        "빨강 플러그가 빠져 있다.",
        "출력 부품 아홉과 베어링, 서보(ID 3), 900 mm 선 둘, 영점 핀",
        "M3 × 8 8, × 10 7, × 12 2, × 20 4, M2 × 5 4, M3 너트 10",
    ),
    stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
)

J3_STEPS: tuple[Step, ...] = (
    Step(
        "disc-prep",
        "원판 준비",
        "J3 출력 원판에 너트와 멈춤 기둥 달기 (손에 들고)",
        do=(
            "출력 원판 테의 귀(튀어나온 곳)에서 나사 머리 자리 바닥의 얇은 막을 육각렌치 끝으로 눌러 뚫습니다.",
            "원판 밑면(가운데가 솟은 쪽)의 육각 홈 넷에 M3 너트를 엄지로 끼웁니다. 내일 아래팔 나사가 여기에 물립니다.",
            "귀 위에서 M3 × 10을 꽂고, 밑으로 나온 끝에 너트를 손으로 돌려 끼운 뒤 육각렌치로 조입니다.",
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
            Move((DISC_NUTS,), ac.vec("j3", (0.0, -10.0, 0.0))),
            Move(("J3 멈춤 나사",), ac.vec("j3", (0.0, 16.0, 0.0)), 0.8),
            Move(("J3 너트 (멈춤 기둥)",), ac.vec("j3", (0.0, -12.0, 0.0)), 1.4),
        ),
        view=ac.link_view("j3", (0, -1, 0), (1, 0, 0.3), 45),
        labels=(
            Label(f"{DISC_NUTS} 1", "M3 너트 × 4 (밑면)"),
            Label("J3 멈춤 나사", "M3 × 10 (귀)"),
        ),
    ),
    Step(
        "carrier",
        "받침 달기",
        f"서보(ID {SERVO_ID})에 받침 달기",
        do=(
            "서보 받침의 날개 둘 바깥 돋움의 옆 홈에 M3 너트를 하나씩 세워 밀어 넣습니다.",
            f"ID {SERVO_ID} 서보를 혼이 아래로 가게 엎어 놓고, 받침을 날개가 아래(혼 쪽)로 가게 들어 판을 서보 뒷면에 얹습니다.",
            "M2 × 5 넷을 작은 십자 드라이버로 조입니다.",
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
            Move((CAR, CAR_NUTS), ac.vec("j3", (0.0, -25.0, 0.0)), 1.2),
            Move((REAR,), ac.vec("j3", (0.0, -12.0, 0.0)), 2.2),
        ),
        view=ac.link_view("j3", (0, -1, 0), (0, 0, 1), 35),
        labels=(Label(SERVO, f"STS3250 · ID {SERVO_ID}"), Label(f"{REAR} 1", "M2 × 5 × 4")),
    ),
    Step(
        "bearing",
        "베어링",
        "J3 집에 베어링 넣기",
        do=(
            "위팔을 납작한 면(글자가 있는 면)이 아래로 가게 책상에 눕힙니다. 끝 집의 넓게 열린 뒤가 위를 봅니다.",
            "베어링을 열린 뒤로 넣고, 바깥 링을 엄지나 베이스용 미는 도구로 눌러 앞 끝의 턱에 닿을 때까지 밉니다. 마지막 "
            "6.6 mm는 꾹 눌러야 들어갑니다.",
        ),
        warn=("바깥 링만 누릅니다. 두드리거나 바이스로 누르지 않습니다.",),
        check=("베어링이 턱에 닿아 기울지 않았다.", "안쪽 링이 부드럽게 돈다."),
        stuck=ac.PHOTO,
        adds=(ARM, BRG),
        still=(ARM,),
        focus=(ARM, BRG),
        mount=(ARM, BRG),
        uses=(Use("base_bearing_pusher", note="엄지로 안 되면", count=False),),
        moves=(Move((BRG,), ac.vec("j3", (0.0, -62.0, 0.0))),),
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
            "허브 판을 가운데가 볼록한 쪽이 위(열린 뒤 쪽)로 가게 들고, 열린 뒤로 떨어뜨려 넣어 베어링 안쪽 링 위에 얹습니다.",
        ),
        warn=(
            "누름 틀보다 먼저 넣습니다. 허브 판(지름 33 mm)이 누름 틀 다리 사이(30 mm)보다 커서 나중에는 곧게 안 들어갑니다.",
        ),
        check=("허브 판이 기울지 않고 안쪽 링 위에 얹혔다.",),
        stuck=ac.PHOTO,
        adds=(HUB,),
        focus=(ARM, BRG, HUB),
        mount=(HUB,),
        uses=(Use("phone", note="속이 어두우면 불빛", count=False),),
        moves=(Move((HUB,), ac.vec("j3", (0.0, -45.0, 0.0))),),
        view=BACK_UP,
        shell=(ARM,),
        cut=True,
        labels=(Label(HUB, "허브 판 (볼록한 쪽이 위)"),),
    ),
    Step(
        "ret-nuts",
        "틀 너트",
        "누름 틀에 너트 넣기 (손에 들고)",
        do=(
            "누름 틀을 고리가 아래, 다리 셋이 위로 오게 놓습니다.",
            "다리마다 옆면의 홈 입구에 M3 너트를 세워 넣고 홈 끝까지 밉니다.",
        ),
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
        view=ac.link_view("j3", (0, -1, 0), (1, 0, 0), 26),
        labels=(Label(f"{RET_NUTS} 3", "M3 너트 × 3"),),
    ),
    Step(
        "ret-in",
        "틀 넣기",
        "누름 틀 넣고 앞줄 벽 나사 셋 조이기",
        do=(
            "누름 틀을 고리가 아래로, 다리가 없는 쪽이 관(목) 쪽으로 가게 들고 열린 뒤로 넣어 고리가 베어링에 닿게 합니다. "
            "허브 판은 다리 앞에 갇힙니다.",
            "누름 틀을 누른 채 아래쪽 줄(앞줄)의 벽 구멍 셋에 M3 × 8을 넣고 육각렌치로 조입니다.",
        ),
        warn=("위쪽 줄(뒷줄) 구멍 둘은 서보를 잠글 때 씁니다. 비워 둡니다.",),
        check=("앞줄 나사 셋의 머리가 벽에 닿았다.", "흔들어도 베어링이 달그락거리지 않는다."),
        stuck=ac.PHOTO,
        adds=FRONT_ROW,
        focus=(ARM, BRG, HUB, RET, RET_NUTS, *FRONT_ROW),
        mount=(RET, RET_NUTS, *FRONT_ROW),
        uses=(Use("hexkey"),),
        moves=(
            Move((RET, RET_NUTS), ac.vec("j3", (0.0, -58.0, 0.0))),
            Move(("J3 벽 나사 1$",), ac.vec("j3", (0.0, 0.0, 14.0)), 1.4),
            Move(("J3 벽 나사 5$",), ac.vec("j3", (14.0, 0.0, 0.0)), 1.8),
            Move(("J3 벽 나사 3$",), ac.vec("j3", (0.0, 0.0, -14.0)), 2.2),
        ),
        view=BACK_UP,
        shell=(ARM,),
        ghost=True,
        labels=(
            Label(RET, "누름 틀 (다리 없는 쪽이 관 쪽)"),
            Label("J3 벽 나사 5$", "M3 × 8 × 3 (앞줄)"),
        ),
    ),
    Step(
        "in-cable",
        "선 하나",
        "어댑터 → J3 선을 관 속으로 꿰기",
        do=(
            "900 mm 선 하나를 집습니다. 한쪽 플러그를 위팔 발 바닥의 큰 구멍(가운데 둥근 구멍)에 넣어, 관 속을 따라 끝 집까지 "
            "밀어 넣습니다. 위팔을 기울이면 잘 내려갑니다.",
            "끝 집의 열린 뒤로 그 플러그를 빼내 손가락 한 마디쯤 나오게 둡니다. 다른 끝(어댑터로 갈 쪽)은 발 쪽에 늘어뜨립니다.",
        ),
        warn=("서보를 넣으면 플러그가 지나갈 틈이 없습니다. 꼭 서보보다 먼저 꿰입니다.",),
        check=("한 플러그는 열린 뒤로, 다른 끝은 발로 나와 있다.",),
        stuck="플러그가 관 속에서 걸리면 위팔을 세워 흔들거나, 끝 집 쪽에서 핀셋으로 당깁니다.",
        adds=(IN_ARM,),
        focus=(ARM, BRG, HUB, RET, RET_NUTS, *FRONT_ROW, IN_ARM),
        mount=(IN_ARM,),
        uses=(Use("servo_cable_900", 1, "어댑터 → J3"),),
        view=ac.link_view("j3", (0, -1, 0), (0.2, 0, 1), 20),
        shell=(ARM,),
        ghost=True,
        labels=(
            Label(IN_ARM, "어댑터 → J3 선 (발에서 관 속으로)", ac.pt("j3", (-120.0, _YC, 0.0))),
        ),
        tags=(Tag(_FOOT, "발의 큰 구멍"),),
    ),
    Step(
        "out-cable",
        "선 둘",
        "J3 → J4 선을 선 창으로 빼고 남는 선을 관 속에 접어 넣기",
        do=(
            "남은 900 mm 선의 한쪽 플러그를 끝 집의 열린 뒤에서 목 쪽으로 넣어, 목 앞면(납작한 면 쪽)의 작은 네모 창(선 창)으로 "
            "밖에 빼냅니다. 창 밖으로 약 35 cm를 당겨 둡니다. 내일 아래팔로 갑니다.",
            "다른 플러그는 열린 뒤에 손가락 한 마디쯤 남깁니다. 그 사이에 남는 선(약 45 cm)을 10 cm쯤으로 지그재그로 접고, "
            "테이프로 한 바퀴 감아 묶습니다.",
            "묶은 선을 열린 뒤에서 목을 지나 관 속으로 넣고, 젓가락이나 연필로 관 가운데(목에서 7 cm쯤)까지 밀어 둡니다.",
        ),
        warn=(
            "남는 선은 꼭 관 가운데에 둡니다. 끝 집 쪽에 몰리면 어깨가 버티는 힘의 여유가 모자랍니다.",
            "앞에서 꿴 어댑터 → J3 선이 같이 끌려 나오지 않게 봅니다.",
        ),
        check=(
            "창 밖으로 약 35 cm, 열린 뒤로 플러그 하나가 나와 있다.",
            "접어 묶은 선이 관 가운데에 들어가 있다.",
        ),
        stuck="창이 작아 플러그가 안 나오면 플러그를 세워(넓은 면을 창의 긴 쪽으로) 넣어 봅니다.",
        adds=(OUT_ARM, OUT_TAIL, OUT_END),
        focus=(ARM, BRG, HUB, RET, RET_NUTS, *FRONT_ROW, IN_ARM, OUT_ARM, OUT_TAIL, OUT_END),
        mount=(OUT_ARM, OUT_TAIL, OUT_END),
        uses=(Use("servo_cable_900", 1, "J3 → J4"), Use("tape"), Use("stick")),
        view=ac.link_view("j3", (0, -1, 0), (0.2, 0, 1), 20),
        shell=(ARM,),
        ghost=True,
        labels=(
            Label(OUT_TAIL, "선 창 밖으로 약 35 cm"),
            Label(OUT_ARM, "남는 선을 접어 관 가운데에", _TUBE_MID),
        ),
        tags=(Tag(_WIN_OUT, "선 창"),),
    ),
    Step(
        "servo-in",
        "서보 넣기",
        f"서보(ID {SERVO_ID}) 넣기",
        do=(
            "서보를 혼이 아래로, 긴 쪽(혼에서 먼 끝)이 관 쪽으로 가게 들고 열린 뒤로 넣습니다. 받침 날개가 벽을 따라 미끄러집니다.",
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
        moves=(Move(SERVO_SET, ac.vec("j3", (0.0, -70.0, 0.0))),),
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
            "서보 받침을 엄지로 눌러 서보가 빠지지 않게 잡은 채, 위팔을 뒤집어 끝 집의 앞쪽(납작한 면)이 위로 오게 합니다.",
            "베어링 가운데 구멍으로 보이는 허브 판의 구멍 넷이 혼의 나사 구멍과 맞게, 허브 판을 육각렌치 끝으로 돌려 맞춥니다.",
            "출력 원판을 귀가 관 반대쪽(바깥 끝)에 오게 들고, 가운데 볼록한 곳을 베어링 안쪽 링에 끼워 끝까지 밉니다.",
        ),
        warn=("귀를 관 쪽에 두면 멈춤 기둥이 목의 살에 걸려 원판이 안 들어갑니다.",),
        check=("원판이 기울지 않고 끝까지 들어갔다.", "원판 구멍 넷으로 혼의 나사 구멍이 보인다."),
        stuck=ac.PHOTO,
        mount=DISC_SET,
        focus=(ARM, BRG, HUB, RET, RET_NUTS, *FRONT_ROW, *SERVO_SET, *DISC_SET),
        uses=(Use("out_disc", note="너트와 멈춤 기둥을 단 것", count=False),),
        moves=(Move(DISC_SET, ac.vec("j3", (0.0, 40.0, 0.0))),),
        view=FRONT_UP,
        labels=(Label(DISC, "출력 원판 (귀가 바깥 끝)"),),
    ),
    Step(
        "horn-bolts",
        "혼 볼트",
        "혼 볼트 넷 조이기",
        do=(
            "M3 × 20 넷을 원판 구멍에 넣어 손으로 몇 바퀴씩 걸어 둡니다.",
            "육각렌치로 대각선 차례로 조금씩, 머리가 원판에 닿을 때까지 조입니다.",
        ),
        warn=("넷이 다 걸릴 때까지 받침을 누른 손을 떼지 않습니다.",),
        check=("볼트 넷의 머리가 원판에 닿았다.", "원판이 흔들리지 않는다."),
        stuck=ac.PHOTO,
        adds=(BOLTS,),
        focus=(ARM, BRG, HUB, RET, RET_NUTS, *FRONT_ROW, *SERVO_SET, *DISC_SET, BOLTS),
        mount=(BOLTS,),
        uses=(Use("hexkey"),),
        moves=tuple(
            Move((f"{BOLTS} {k}$",), ac.vec("j3", (0.0, 22.0, 0.0)), 0.45 * (k - 1))
            for k in range(1, 5)
        ),
        view=FRONT_UP,
        labels=(Label(f"{BOLTS} 1$", "M3 × 20 × 4"),),
    ),
    Step(
        "lock",
        "서보 잠금",
        "다시 뒤집어 뒷줄 벽 나사 둘로 서보 잠그기",
        do=(
            "위팔을 다시 납작한 면이 아래로 가게 눕힙니다.",
            "벽의 뒷줄(열린 뒤에 가까운 줄) 구멍 둘에 M3 × 8을 넣고 조입니다. 서보를 밀거나 당기지 않고 지금 자리 그대로 잠급니다.",
        ),
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
            Move(("J3 벽 나사 2$",), ac.vec("j3", (0.0, 0.0, 14.0))),
            Move(("J3 벽 나사 4$",), ac.vec("j3", (0.0, 0.0, -14.0)), 0.7),
        ),
        view=BACK_UP,
        labels=(Label("J3 벽 나사 2$", "M3 × 8 × 2 (뒷줄)"),),
    ),
    Step(
        "plug",
        "선 꽂기",
        f"선 둘을 서보(ID {SERVO_ID})에 꽂기",
        do=(
            "관에서 온 선(어댑터 → J3)의 플러그를 그림의 ① 커넥터에, 선 창으로 나가는 선(J3 → J4)의 플러그를 ② 커넥터에 꽂습니다. "
            "받침 판의 터진 쪽으로 플러그 옆면을 잡고 끝까지 밉니다.",
            "두 선을 받침 판 위에 납작하게 눕혀 서보 먼 끝(관 쪽)으로 보냅니다.",
        ),
        warn=("선이 판 위로 들뜨면 뒷덮개가 뜹니다. 판 위에 붙여 눕힙니다.",),
        check=("두 플러그가 끝까지 꽂혀 살짝 당겨도 안 빠진다.", "두 선이 판 위에 눕혀져 있다."),
        stuck="어느 커넥터인지 헷갈리면 사진을 보내 주세요.",
        adds=(IN_PLUG, OUT_PLUG),
        focus=(ARM, *SERVO_SET, *BACK_ROW, IN_ARM, OUT_ARM, IN_PLUG, OUT_PLUG),
        mount=(IN_PLUG, OUT_PLUG),
        uses=(Use("servo_cable_900", note="열린 뒤로 나온 두 플러그", count=False),),
        moves=(
            Move((IN_PLUG,), ac.vec("j3", (0.0, -12.0, 0.0))),
            Move((OUT_PLUG,), ac.vec("j3", (0.0, -12.0, 0.0)), 0.8),
        ),
        view=ac.link_view("j3", (0, -1, 0), (-0.3, 0, 1), 40),
        zoom=1.6,
        center=_J3,
        labels=(Label(IN_PLUG, "① 관에서 온 선"), Label(OUT_PLUG, "② 선 창으로 나가는 선")),
    ),
    Step(
        "cover",
        "뒷덮개",
        "링크 뒷덮개 달고 J3 돌려 보기",
        do=(
            "링크 뒷덮개를 테가 몸통 쪽으로 가게 끝 집에 얹고, M3 × 12 둘을 조입니다.",
            "출력 원판을 손으로 천천히 양쪽 끝까지 돌려 봅니다. 귀 밑 기둥이 살에 닿아 멈추면 더 밀지 않습니다. 가운데로 "
            "되돌립니다.",
        ),
        warn=("선이 덮개 테 밑에 끼이지 않게 봅니다.",),
        check=("덮개가 들뜨지 않았다.", "원판이 양 끝까지 걸림 없이 돈다."),
        stuck=ac.PHOTO,
        adds=(COVER, COVER_SCREWS),
        focus=(*IN_HAND, COVER, COVER_SCREWS),
        mount=(COVER, COVER_SCREWS),
        uses=(Use("hexkey"),),
        moves=(
            Move((COVER,), ac.vec("j3", (0.0, -30.0, 0.0))),
            Move((COVER_SCREWS,), ac.vec("j3", (0.0, -16.0, 0.0)), 1.0),
        ),
        view=BACK_UP,
        labels=(Label(COVER, "링크 뒷덮개"), Label(f"{COVER_SCREWS} 1", "M3 × 12 × 2")),
    ),
    Step(
        "foot-route",
        "발의 선 자리",
        "어댑터 → J3 선을 발의 높은 창에 눕히기",
        do=(
            "위팔 발 바닥(어깨에 얹힐 면)의 큰 구멍에서 끝벽 쪽으로 긴 틈이 나 있습니다. 발 쪽으로 나온 선을 큰 구멍에서 그 틈으로 "
            "옆에서 밀어 넣어, 끝벽 밑의 낮은 구멍으로 뺍니다.",
            "끝벽의 세로 틈을 따라 선을 세워 밀어 올린 뒤, 높은 창(①)에 눕힙니다. 눕힌 선은 틈보다 넓어 되내려가지 않습니다.",
        ),
        warn=("플러그를 좁은 구멍에 꿰지 않습니다. 선만 틈으로 옆에서 밀어 넣습니다.",),
        check=("선이 발 바닥의 틈을 지나 끝벽의 높은 창으로 나와 있다.",),
        stuck=ac.PHOTO,
        focus=(*IN_HAND, COVER, COVER_SCREWS),
        uses=(Use("servo_cable_900", note="발 쪽으로 나온 어댑터 → J3 선", count=False),),
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
        "위팔 달기",
        "위팔을 어깨 출력 원판에 달기 (M3 × 10 여섯)",
        do=(
            "어깨 출력 원판의 귀가 위에 오게 돌려 둡니다.",
            "위팔을 끝 집이 위로 서게 들고, 발을 원판(로봇 왼쪽 면)에 얹습니다. 발 끝의 핀 귀가 아래(어깨의 살 쪽)에 옵니다.",
            "발의 구멍 여섯을 원판의 너트 여섯에 맞추고, M3 × 10 여섯을 넣어 대각선 차례로 조입니다.",
        ),
        warn=(
            "발 바닥의 틈에 넣은 선이 원판과 발 사이에 끼지 않게 봅니다.",
            "조이는 동안 위팔이 쓰러지지 않게 한 손으로 받칩니다.",
        ),
        check=("발이 원판에 들뜨지 않고 앉았다.", "나사 여섯의 머리가 다 닿았다."),
        stuck=ac.PHOTO,
        adds=(HAVE, ARM_SCREWS),
        still=(HAVE,),
        uses=(Use("hexkey"), Use("upper_arm", note="J3를 조립한 것", count=False)),
        moves=(
            Move((*IN_HAND, COVER, COVER_SCREWS), (0.0, 0.0, -45.0)),
            Move((ARM_SCREWS,), (0.0, 0.0, -14.0), 1.4),
        ),
        view=FROM_LEFT,
        zoom=1.3,
        center=ac.pt("j2", (40.0, 0.0, 0.0)),
        labels=(Label(f"{ARM_SCREWS} 1", "M3 × 10 × 6"), Label(ARM, "위팔 (끝 집이 위)")),
    ),
    Step(
        "route",
        "선 놓기",
        "어댑터 → J3 선을 J1 몸통을 따라 내리기",
        do=(
            "발의 높은 창에서 나온 선을 J1 몸통 왼쪽 면(어깨 원판 밑)으로 내립니다. 어제 놓은 J1 → J2 선과 나란히 갑니다.",
            "왼쪽 면을 따라 뒤로 가서 뒤 왼쪽 모서리를 돌고, J1 몸통 뒤쪽 귀 위로 넘어가 베이스 몸통의 띠를 따라 뒤 왼쪽으로 "
            "내려 몸통 뒤로 뺍니다. 남는 선은 책상 뒤로 늘어뜨립니다.",
        ),
        check=("두 선이 나란히 같은 길에 놓였다.", "선이 바짝 꺾이거나 당겨진 곳이 없다."),
        stuck=ac.PHOTO,
        adds=(IN_BODY,),
        uses=(Use("servo_cable_900", note="어댑터 → J3", count=False),),
        view=FROM_BACK_LEFT,
        zoom=1.3,
        center=ac.pt("j2", (-30.0, 0.0, 0.0)),
        labels=(Label(IN_BODY, "어댑터 → J3 선"),),
    ),
    Step(
        "clamps",
        "누르개",
        "누르개 셋으로 선 둘 잡기",
        do=(
            "왼쪽 면 누르개: J1 몸통 왼쪽 면 아래 띠의 나사 구멍에 누르개를 대고, 선 둘을 그 밑에 넣은 채 M3 × 8로 조입니다.",
            "귀 누르개: J1 몸통 뒤쪽 귀 위의 나사 구멍에서 같은 식으로 선 둘을 잡습니다.",
            "띠 밑 누르개: 베이스 몸통 띠의 아래 턱을 튼 자리 밑 나사 구멍에서 선 둘을 잡습니다.",
        ),
        warn=(
            "선이 누르개 밑에서 겹치지 않게 나란히 눕힙니다. 너무 세게 조여 선을 짓누르지 않습니다.",
        ),
        check=(
            "누르개 셋이 선 둘을 잡았다.",
            "J1 몸통을 손으로 조금 돌려도 선이 누르개에서 빠지지 않는다.",
        ),
        stuck=ac.PHOTO,
        adds=(*CLAMPS, CLAMP_SCREWS),
        uses=(Use("hexkey"),),
        moves=(
            Move(("왼쪽 면 누르개", "누르개 나사 (왼쪽 면)"), (0.0, 0.0, -12.0)),
            Move(("귀 누르개", "누르개 나사 (귀)"), (0.0, 12.0, 0.0), 0.8),
            Move(("띠 밑 누르개", "누르개 나사 (띠 밑)"), (-8.0, 0.0, -8.0), 1.6),
        ),
        view=FROM_BACK_LEFT,
        zoom=1.5,
        center=ac.pt("j2", (-60.0, 0.0, 0.0)),
        labels=tuple(Label(c, c) for c in CLAMPS),
    ),
    Step(
        "adapter",
        "어댑터",
        "어댑터 → J3 선을 어댑터 왼쪽 단자에 꽂기",
        do=(
            "어댑터 덮개를 들어 뗍니다.",
            "책상 뒤로 늘어진 선의 끝 플러그를 어댑터의 왼쪽 단자에 꽂습니다. 가운데 단자에는 J1 선이 꽂혀 있습니다.",
            "J1 선처럼 플러그 위로 고리를 지어 덮개의 선 창으로 내리고, 덮개를 다시 얹습니다.",
        ),
        warn=("전원이 꺼져 있는지 다시 봅니다.",),
        check=("왼쪽 단자에 플러그가 끝까지 꽂혔다.", "덮개가 선을 누르지 않고 얹혔다."),
        stuck=ac.PHOTO,
        adds=(IN_AD, IN_AD_PLUG),
        uses=(Use("servo_cable_900", note="어댑터 → J3의 끝", count=False),),
        moves=(Move((IN_AD_PLUG,), (0.0, 12.0, 0.0)),),
        hide=(f"{HAVE}어댑터 덮개",),
        view=View((0, 1, 0), (-0.4, 0, 1), 20, 0),
        zoom=2.0,
        center=_AD,
        labels=(Label(IN_AD_PLUG, "왼쪽 단자 (J3 줄)"),),
    ),
    Step(
        "turn-j2",
        "돌려 보기",
        "위팔을 손으로 돌려 선 보기",
        do=(
            "위팔을 잡고 천천히 앞으로 숙여 책상에 닿기 전까지, 다시 뒤로 45°쯤까지 돌려 봅니다.",
            "발의 높은 창에서 나온 선이 끼거나 팽팽해지지 않는지, 누르개에서 빠지지 않는지 봅니다.",
            "위팔을 곧게 세워 둡니다.",
        ),
        warn=(
            "뒤로 많이 돌리면 끝 집이 벽에 닿습니다. 45°쯤에서 멈춥니다.",
            "선이 걸리면 억지로 돌리지 말고 사진을 보내 주세요. 발의 낮은 구멍으로 바꾸는 길(테에 감는 길)도 있습니다.",
        ),
        check=("돌리는 동안 선이 걸리거나 당겨지지 않았다.", "위팔이 곧게 서 있다."),
        uses=(Use("phone", note="걸리면 사진", count=False),),
        turn2=(40.0, -110.0),
        view=FROM_LEFT,
    ),
)

ZERO_STEPS: tuple[Step, ...] = (
    Step(
        "power-on",
        "전원 넣기",
        "서보 전원 넣기",
        do=(
            "검정 플러그를 가변 전원의 − 단자에 꽂고, 가변 전원을 켜 12.0 V인지 봅니다.",
            "빨강 플러그를 + 단자에 꽂습니다. 화면의 전류를 대화창에 알려 주세요.",
            f"Claude가 서보 셋(ID 1, {J2_ID}, {SERVO_ID})을 읽습니다. 다 읽었다는 말을 기다립니다.",
        ),
        warn=(
            "서보는 힘이 꺼진 채 켜져 아무것도 움직이지 않아야 합니다. 움직이거나 C.C 표시등이 켜지거나 냄새가 나면 바로 빨강 "
            "플러그를 뺍니다.",
            "위팔은 힘이 없어 넘어갑니다. 계속 한 손으로 받칩니다.",
        ),
        check=("12.0 V이고 C.C 표시등이 꺼져 있다.", "Claude가 서보 셋을 읽었다고 했다."),
        stuck="화면을 사진으로 찍어 보내 주세요.",
        uses=(Use("psu"), Use("pc")),
        view=View((0, 1, 0), (-0.4, 0, 1), 20, 0),
        zoom=2.0,
        center=_AD,
    ),
    Step(
        "direction",
        "방향 재기",
        "J2를 + 쪽으로 돌려 방향 재기",
        do=(
            "Claude가 PC에 영점 창을 띄웁니다. 창에 나오는 말대로 합니다.",
            '"영점 핀을 빼고 …"가 나오면, 핀이 꽂혀 있지 않으니 그대로 Enter를 누릅니다.',
            '"+ 쪽으로 20°쯤 돌린 채 Enter"가 나오면, 위팔을 앞(책상 쪽)으로 20°쯤 숙인 채 Enter를 누릅니다.',
        ),
        warn=("천천히 돌립니다. 위팔을 놓지 않습니다.",),
        check=('창에 "방향"이 나왔다.',),
        stuck="창에 나온 글을 사진으로 보내 주세요.",
        uses=(Use("pc", note="영점 창"),),
        turn2=(0.0, -20.0),  # 운동학 +20°는 설계 각 -20° (J2의 설계 부호가 -)
        view=FROM_LEFT,
        labels=(Label(ARM, "+ 쪽: 위팔이 앞으로 숙는 쪽"),),
    ),
    Step(
        "pin-in",
        "핀 꽂기",
        "J2 영점 핀 꽂고 Enter",
        do=(
            "위팔을 곧게 세웁니다.",
            "위팔 발 끝의 핀 귀(아래쪽, 어깨 원판 밖으로 나온 귀)의 작은 구멍에 영점 핀을 꽂습니다. 핀이 어깨의 살까지 쑥 들어가면 "
            "맞는 자리입니다. 안 들어가면 위팔을 조금씩 앞뒤로 돌려 맞춥니다.",
            '"영점 핀을 꽂고 …"가 나오면, 위팔을 앞으로 살짝 민 채 Enter를 누릅니다.',
            "창에 세 줄이 나오면 됩니다. 핀은 그대로 둡니다.",
        ),
        warn=("핀을 공구로 박지 않습니다. 손으로 쑥 들어가는 자리를 찾습니다.",),
        check=("핀이 귀 위로 3 mm쯤 나와 있다.", "창에 세 줄(위치 오프셋, 영점, 방향)이 나왔다."),
        stuck="핀을 꽂은 모습과 창을 사진으로 보내 주세요.",
        adds=(PIN,),
        uses=(Use("pc", note="영점 창"),),
        moves=(Move((PIN,), (0.0, 0.0, -24.0)),),
        view=FROM_LEFT,
        zoom=1.8,
        center=_PIN_AT,
        labels=(Label(PIN, "영점 핀 (위팔 발의 귀)"),),
    ),
    Step(
        "apply",
        "한계 굽기",
        "각도 한계 굽기 (Claude)",
        do=(
            "Claude가 세 줄을 로봇 설정에 옮기고, 잰 방향을 설계로 본 방향(-1)과 견줍니다.",
            "값 적용(각도 한계와 보호 설정 굽기)을 돌려도 되는지 물으면 답합니다. 서보는 움직이지 않습니다.",
        ),
        check=("Claude가 값 적용이 끝났다고 했다.",),
        uses=(Use("pc"),),
        view=FROM_LEFT,
        zoom=1.8,
        center=_PIN_AT,
    ),
    Step(
        "pin-out",
        "핀 빼기",
        "영점 핀 빼기",
        do=("위팔을 받친 채 롱노즈로 핀 머리를 잡고 곧게 당겨 뺍니다.",),
        warn=("귀를 받침 삼아 젖히지 않습니다.",),
        check=("핀을 뺐다.",),
        uses=(Use("longnose"),),
        view=FROM_LEFT,
        zoom=1.8,
        center=_PIN_AT,
        labels=(Label(PIN, "곧게 당겨 뺀다"),),
    ),
    Step(
        "check",
        "확인",
        "앞으로 90° 숙여 읽어 보기",
        do=(
            "위팔을 앞으로 숙여 수평(90°쯤)으로 받쳐 듭니다.",
            "Claude가 J2 각을 읽습니다. +90°쯤이면 맞습니다.",
        ),
        check=("Claude가 +90°쯤으로 읽었다고 했다.",),
        uses=(Use("pc"),),
        hide=(PIN,),
        turn2=(0.0, -90.0),
        view=FROM_LEFT,
        labels=(Label(ARM, "앞으로 수평 (+90°)"),),
    ),
    Step(
        "power-off",
        "전원 끄기",
        "전원 끄고 위팔 눕혀 두기",
        do=(
            "빨강 플러그를 뺀 뒤 가변 전원을 끕니다. Claude가 기록을 올립니다.",
            "책상 앞쪽에 수건을 깔고, 위팔을 천천히 앞으로 숙여 끝 집이 수건 위에 닿게 눕혀 둡니다 (125°쯤).",
            "창 밖으로 나온 J3 → J4 선은 끝 집 옆에 둡니다.",
        ),
        warn=("전원이 없으면 위팔이 서지 않습니다. 세워 두지 않습니다.",),
        check=("빨강 플러그를 뺐고 가변 전원이 꺼졌다.", "위팔이 수건 위에 누워 있다."),
        uses=(Use("psu"), Use("towel")),
        hide=(PIN,),
        joint2=-125.0,
        view=FROM_LEFT,
    ),
)

STEPS: tuple[Step, ...] = (
    PREP,
    *J3_STEPS,
    *ATTACH_STEPS,
    *ZERO_STEPS,
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            f"위팔(J3, ID {SERVO_ID})을 달고 J2(ID {J2_ID})의 영점을 구웠습니다.",
            "다음 안내는 아래팔입니다. 창 밖의 J3 → J4 선이 아래팔로 갑니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요.",
        ),
        hide=(PIN,),
        joint2=-125.0,
        view=FROM_LEFT,
    ),
)

GUIDE = Guide(
    id="upper-arm",
    kind="assembly",
    title="위팔과 J2 영점",
    sub=f"J3 STS3250 ID {SERVO_ID} · 선 둘 · 위팔 달기 · 누르개 셋 · J2(ID {J2_ID}) 영점",
    scene=scene,
    steps=STEPS,
    items=ITEMS_HERE,
    bundle="arm-black-1",
    printed_before=("upper_arm", "base_bearing_pusher"),
    axis2=J2_AXIS,
)
