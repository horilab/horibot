"""손목 잇기와 J4 영점: 손목 끝을 J4 출력 원판에 달고, J4 → J5 선을 J5에 꽂고, J4(ID 4) 영점, 쉬는 자세 (0029, 0031, 0037).

순서는 형상으로 따졌다 (2026-11-25):
- J4 → J5 선은 어제 아래팔 끝의 선 창 밖으로 늘어뜨려 둔 끝이다 (J4 서보를 넣은 뒤에는 그쪽 플러그가 지나갈 틈이 없어 어제
  꿰었다). 손목 1의 J5 서보는 -z 커넥터가 비어 있다 (+z에는 J5 → J6 선). 가짜 아래팔 끝을 뗀 손목 1은 발 쪽이 열려 있어서,
  뒷덮개를 떼고 선 끝을 발 쪽에서 서보 먼 끝을 돌아 뒤로 올려 꽂는다. 손목 1 앞벽의 -z 옆 홈으로 선이 나가게 놓고 발을
  원판에 단다 (0031: 손목 몸통 속에서는 두 선이 서보 먼 끝 밑을 혼 볼트 머리 양옆으로 지난다).
- 손목 1은 J4 출력 원판에 다른 관절과 반 바퀴 다르게 단다: J4의 멈춤 귀가 손목 1의 뒤쪽(J5 뒷덮개 쪽)을 본다 (0029).
- J4 영점은 J2와 J3 영점 핀으로 팔을 곧게 세운 채, J4가 90°(범위 가운데 180°에서 90° 돌린 자리: 손목 몸통은 범위
  가운데에서 90° 돌린 자리에서 핀이 맞는다, 0029)일 때 손목 1의 핀 귀와 아래팔 끝 집의 살에 핀을 꽂는다. + 쪽은 손목이
  앞으로 숙는 쪽이다. 설계로 본 방향은 -1이다. 핀 셋이 든다.
- 발 나사 넷은 머리 위 13 ~ 17 mm에서 손목 1 몸통(누름 틀, 서보 받침, 벽 나사)에 막혀 곧은 렌치가 들어가지 않는다
  (guide/reach.py). L자 렌치의 짧은 쪽을 머리에 넣고 긴 쪽을 손목 몸통 바깥으로 눕히면 나사마다 반 바퀴쯤 돌릴 수 있어서,
  그 방향을 그려 보인다 (같은 구멍의 가짜 아래팔 나사를 손목 끝 조립 때 육각렌치로 풀었다).
- 다 하면 쉬는 자세(J1 -55°, J2 52°, J3 154°, J4 325°)로 둔다 (0037). D405 선은 아직 묶지 않는다.

장면 좌표는 J1의 서보 좌표다 (arm_common). 둘째 관절은 J4다.
"""

from __future__ import annotations

from dataclasses import replace

from build123d import Compound, Vector

from horibot_design.guide import reach
from horibot_design.guide.guides import arm_common as ac
from horibot_design.guide.guides import forearm as fa
from horibot_design.guide.guides import upper_arm as ua
from horibot_design.guide.model import Guide, Item, Label, Move, ScenePart, Step, Use, View
from horibot_design.parts import fake_forearm as ff
from horibot_design.parts import joint as j
from horibot_design.parts import wrist
from horibot_design.studies import wiring as w

SERVO_ID = 4
AXIS_JOINT = 4
HAVE = ua.HAVE
W = "손목/"  # 조립해 둔 손목 끝 (손목 1, 손목 2, D405, 그리퍼)
REST_PREFIX = "쉬는 자세/"
PIN_POSE = (
    90.0  # J4 핀 자세의 둘째 관절 각(설계 각): 운동학 90°는 범위 가운데(180°)에서 설계 각 +90°
)

DONE = Item("assembled_arm", "조립해 둔 팔", "bought", "베이스, 어깨, 위팔(J3), 아래팔(J4, ID 4)")
WRIST = Item(
    "assembled_wrist",
    "조립해 둔 손목 끝",
    "bought",
    "손목 1(J5, ID 5), 손목 2(J6, ID 6), D405, 그리퍼(ID 11)",
)
OTHER: dict[str, Item] = {
    it.key: it
    for it in (
        DONE,
        WRIST,
        Item(
            "fake_forearm",
            "가짜 아래팔 끝",
            "printed",
            "선 시험에 쓴 것 (떼어 보관한다)",
            color="black",
        ),
        Item(
            "servo_cable_300",
            "서보 선 300 mm",
            "bought",
            "어제 아래팔 선 창 밖으로 늘어뜨린 J4 → J5 선",
            icon="wire",
        ),
        Item("psu", "가변 전원", "tool", "전류 상한 3 A로 맞춰 둔 것"),
        Item("pc", "집 PC", "tool", "USB 5 m 선이 꽂힌 것. Claude가 명령을 돌린다"),
        Item("longnose", "롱노즈", "tool", "핀을 곧게 당겨 뺀다", icon="tweezers"),
        Item("phone", "휴대폰", "tool", "사진을 찍어 대화창에 보낸다", icon="phone"),
    )
}

J4_AXIS = ac.axis_of("j4")
_FA_REDRAW = ("J5 선 (창 밖)", "J5 선 끝 플러그", "영점 핀 (J2)", "영점 핀 (J3)")

# 발 나사를 조이는 L자 렌치 (그림): 발 나사 1(뒤쪽 아래)에 짧은 쪽을 넣고 긴 쪽을 뒤(손목 몸통 바깥)로 눕힌 자리.
# 방향은 reach.l_key의 각이다 (0° 위, 90° 앞, 180° 아래, 270° 뒤). 발 나사 1, 2는 180 ~ 345°, 3, 4는 15 ~ 180°로 돈다.
KEY = "육각렌치 (발 나사)"
KEY_ON = "1"
KEY_OUT = Vector(0, 0, -1)
KEY_TURN = 270.0


def _have_pieces() -> list[ac.Piece]:
    """위팔과 아래팔 안내에서 붙인 것 (다시 그리는 것, 영점 핀, 꽂아 보기만 한 시험 핀은 빼고)."""
    ups = [replace(p, name=f"{HAVE}{p.name}") for p in ua.pieces() if p.name not in fa._UA_REDRAW]
    tried = (fa.TRY_UA, fa.TRY_FOOT, fa.TRY_END)
    fores = [
        replace(p, name=f"{HAVE}{p.name}")
        for p in fa.pieces()
        if p.name not in _FA_REDRAW and p.name not in tried
    ]
    return [replace(p, key=DONE.key, count=False) for p in (*ups, *fores)]


def _wrist_pieces() -> list[ac.Piece]:
    """조립해 둔 손목 끝 (범위 가운데). 손목 1의 뒷덮개와 덮개 나사는 이 안내에서 떼었다 단다."""
    out: list[ac.Piece] = []
    for group in ("wrist1", "wrist2", "wrist_end"):
        out += ac.group_pieces(group, WRIST.key, prefix=W)
    # 손목 1의 덮개 나사는 떼었다 다시 조인다: 나사 목록 키로 둔다 (끼우는 길 검사가 나사를 빼고 본다)
    return [
        replace(p, key="m3x12") if p.name.startswith(f"{W}손목 1/덮개 나사") else p for p in out
    ]


def pieces() -> list[ac.Piece]:
    out: list[ac.Piece] = []
    # 가짜 아래팔 끝 (손목 1의 서보 좌표)
    for p in ff.placed():
        key = (
            "fake_forearm"
            if p.name == "가짜 아래팔 끝"
            else "nut_m3"
            if p.name.startswith("발 나사 너트")
            else "m3x12"
        )
        name = p.name.replace("발 나사", "가짜 발 나사")
        out.append(ac.Piece(name, ac.put(p.shape, "j5"), key, p.color, "j5", False))
    # J4 → J5 선: 아래팔 선 창 → 손목 1 앞벽의 -z 옆 홈 → 서보 먼 끝 밑 → J5의 -z 커넥터
    inside = [tuple(float(c) for c in p) for p in w.wrist_in(front=True)]
    across = [
        ac.pt("j4", ac.window_point(0.0)),
        ac.pt("j4", ac.window_point(8.0)),
        ac.pt("j5", inside[0]),
    ]
    out.append(
        ac.Piece("J5 선 (관절 밖)", ac.wire_scene(across), "servo_cable_300", "wire", "j4", False)
    )
    out.append(
        ac.Piece("J5 선 (손목 1 속)", ac.wire(inside, "j5"), "servo_cable_300", "wire", "j5", False)
    )
    out.append(
        ac.Piece(
            "J5 선 플러그 (J5)",
            ac.plug(-1, "STS3215", "j5"),
            "servo_cable_300",
            "plug",
            "j5",
            False,
        )
    )
    # 손목 1의 발 나사 M3 × 10 넷 (J4 출력 원판 밑면의 너트에 물린다). 머리는 장면의 -z 쪽이다
    for p in wrist.foot_mount("STS3215", flip=True):
        if p.name.startswith("발 나사"):
            k = p.name.rsplit(" ", 1)[1]
            screw = ac.put(p.shape, "j5")
            out.append(ac.Piece(f"손목 발 나사 {k}", screw, "m3x10", "metal", "j5"))
            if k == KEY_ON:
                top = reach.head_top(screw, KEY_OUT)
                shape = Compound([reach.l_key(top, KEY_OUT, KEY_TURN)])
                out.append(ac.Piece(KEY, shape, "hexkey", "metal", "j5", False))
    # 영점 핀 셋: J2, J3(팔을 곧게 잡는다)과 J4 (손목 1의 귀, J4 90°에 꽂은 자리)
    for p in fa.pieces():
        if p.name in ("영점 핀 (J2)", "영점 핀 (J3)"):
            out.append(replace(p, count=True))
    pin4 = next(p for p in wrist.child_hardware() if p.name.startswith("영점 핀"))
    out.append(ac.Piece("영점 핀 (J4)", ac.put(pin4.shape, "j4"), "pin_msh3_20", "pin", "j4"))
    return out


def rest_pieces() -> list[ac.Piece]:
    """쉬는 자세의 팔 전체 (J1은 첫째 관절로 돌린다). 개수에 넣지 않는다."""
    out = ac.link_pieces("upper_arm", DONE.key, DONE.key, ac.REST, REST_PREFIX)
    out += ac.link_pieces("forearm", DONE.key, DONE.key, ac.REST, REST_PREFIX)
    out += [
        p
        for p in ac.group_pieces("shoulder", DONE.key, ac.REST, REST_PREFIX)
        if p.frame == "j2_out"
    ]
    for group in ("wrist1", "wrist2", "wrist_end"):
        out += ac.group_pieces(group, WRIST.key, ac.REST, REST_PREFIX)
    return [replace(p, key=DONE.key, count=False) for p in out]


def scene() -> list[ScenePart]:
    have = [replace(p, key=DONE.key, moving2=False) for p in ua._have()]
    return (
        have
        + ac.scene_of(_have_pieces(), AXIS_JOINT)
        + ac.scene_of(_wrist_pieces(), AXIS_JOINT)
        + ac.scene_of(pieces(), AXIS_JOINT)
        + ac.scene_of(rest_pieces(), AXIS_JOINT)
    )


# ------------------------------------------------------------------------------------------------
# 이름

FAKE = ("가짜 아래팔 끝", "가짜 발 나사")
W1 = f"{W}손목 1/"
W1_COVER, W1_COVER_SCREWS = f"{W}손목 1/뒷덮개", f"{W}손목 1/덮개 나사"
W_BODY = (f"{W}손목 1/손목 몸통", f"{W}손목 2/", f"{W}D405/", f"{W}그리퍼/")
W1_INNER = tuple(
    f"{W}손목 1/{n}"
    for n in (
        "서보 몸통",
        "서보 받침",
        "혼$",
        "혼 볼트",
        "출력 원판",
        "허브 판",
        "누름 틀",
        "베어링",
        "벽 나사",
        "너트",
        "멈춤",
        "서보 뒷면",
    )
)
WRIST_SET = (*W_BODY, *W1_INNER, W1_COVER, W1_COVER_SCREWS)
# 아래팔 끝 (J4 집과 그 속, J4 → J5 선). 위팔 속의 J4 선(J3 → J4)은 이름이 "J4 선"이라 고르지 않는다
FOREARM_END = (
    f"{HAVE}아래팔",
    *(
        f"{HAVE}J4 {n}"
        for n in (
            "베어링",
            "허브",
            "출력",
            "누름",
            "서보",
            "혼",
            "벽",
            "너트",
            "멈춤",
            "뒷덮개",
            "덮개",
        )
    ),
    f"{HAVE}J5 선",
)
ACROSS, INSIDE, PLUG = "J5 선 (관절 밖)", "J5 선 (손목 1 속)", "J5 선 플러그 (J5)"
FEET = "손목 발 나사"
PIN2, PIN3, PIN4 = "영점 핀 (J2)", "영점 핀 (J3)", "영점 핀 (J4)"
SHOULDER_TURNING = fa.SHOULDER_TURNING
UPRIGHT = (
    f"{HAVE}위팔",
    f"{HAVE}J3 ",
    f"{HAVE}J4 ",
    f"{HAVE}아래팔",
    f"{HAVE}J5 선",
    W,
    "J5 선",
    FEET,
    "영점 핀",
    *SHOULDER_TURNING,
)

_W1 = ac.pt("j5", (0.0, 0.0, 0.0))
_J4 = ac.pt("j4", (0.0, 0.0, 0.0))
_PIN_AT = ac.pt("j4", (-j.R_PIN, j.ATOP + j.EAR_T, 0.0))
W1_BACK_UP = ac.link_view("j5", (0, -1, 0), (1, 0, 0.4), 30)  # 손목 1의 열린 뒤가 위
W1_FOOT = ac.link_view("j5", (1, 0, 0), (0, -0.5, 1), 25)  # 손목 1의 발 쪽에서
FROM_LEFT = View((0, 1, 0), (0.35, 0, -1), 15, 0)
REST_VIEW = View((0, 1, 0), (1, 0, 0.6), 20, 0)

PREP = Step(
    "prep",
    "준비물",
    "준비물 꺼내 놓기",
    kind="prep",
    do=(
        "서보 전원이 꺼져 있는지 봅니다. 팔은 어제처럼 쉬는 자세로 선반에 얹혀 있습니다.",
        "조립해 둔 손목 끝(손목 1, 손목 2, D405, 그리퍼)을 아래팔 끝 옆 책상에 둡니다. D405 선은 늘어뜨려 둡니다.",
        "나사: M3 × 10 넷 (손목 1의 발). 영점 핀 셋 (J2, J3, J4).",
    ),
    check=("빨강 플러그가 빠져 있다.", "손목 끝, M3 × 10 4, 영점 핀 3"),
    stuck="핀이 셋이 안 되면 대화창에 알려 주세요. 둘이면 위팔은 손으로 받치고 합니다.",
)

WRIST_STEPS: tuple[Step, ...] = (
    Step(
        "ff-off",
        "가짜 아래팔 떼기",
        "손목 1에서 가짜 아래팔 끝 떼기 (달려 있으면)",
        do=(
            "손목 1 발에 가짜 아래팔 끝이 달려 있으면, 발 양옆 귀의 M3 × 12 넷을 풀어 뗍니다. 이미 떼어 뒀으면 이 단계는 "
            "넘어갑니다.",
            "손목 1 발 밑에 눕혀 둔 J5 → J6 선(+z 옆 홈으로 나가는 선)이 빠지지 않게 봅니다.",
        ),
        check=("가짜 아래팔 끝을 뗐다 (또는 이미 떼어 뒀다).", "J5 → J6 선이 제자리에 있다."),
        stuck=ac.PHOTO,
        adds=(*WRIST_SET, *FAKE),
        still=WRIST_SET,
        focus=(*WRIST_SET, *FAKE),
        mount=(*WRIST_SET, *FAKE),
        uses=(Use("hexkey"),),
        view=W1_FOOT,
        labels=(Label("가짜 아래팔 끝$", "가짜 아래팔 끝 (뗀다)"),),
    ),
    Step(
        "w1-open",
        "덮개 떼기",
        "손목 1 뒷덮개 떼기",
        do=("손목 1 뒷덮개의 M3 × 12 둘을 풀어 덮개를 뗍니다. 나사와 덮개는 다시 씁니다.",),
        check=("덮개를 뗐고, 서보 뒷면의 커넥터 둘이 보인다 (하나는 비어 있다).",),
        stuck=ac.PHOTO,
        focus=WRIST_SET,
        uses=(Use("hexkey"),),
        view=W1_BACK_UP,
        labels=(Label(W1_COVER, "손목 1 뒷덮개 (뗀다)"),),
    ),
    Step(
        "cable-in",
        "선 넣기",
        "J4 → J5 선을 손목 1 속으로 넣기",
        do=(
            "아래팔 끝 선 창 밖으로 나온 J4 → J5 선의 플러그를, 손목 1 발 쪽(가짜 아래팔 끝을 뗀 자리)에서 서보 먼 끝 밑으로 "
            "넣어 먼 끝을 돌아 뒤(덮개를 뗀 쪽)로 빼냅니다.",
            "발 쪽에 남은 선은 손목 1 앞벽 발 쪽 끝의 옆 홈 셋 가운데 J5 → J6 선이 나가는 홈의 반대쪽 홈(①)에 눕힙니다.",
        ),
        warn=("J5 → J6 선과 겹치지 않게, 두 선이 서보 먼 끝 밑에서 양옆으로 갈라지게 둡니다.",),
        check=("플러그가 손목 1의 뒤로 나왔다.", "선이 반대쪽 옆 홈에 눕혀져 있다."),
        stuck=ac.PHOTO,
        adds=(ACROSS, INSIDE),
        focus=(*FOREARM_END, *W_BODY, *W1_INNER, ACROSS, INSIDE),
        mount=(ACROSS, INSIDE),
        uses=(Use("servo_cable_300", note="아래팔 창 밖의 끝", count=False),),
        view=W1_FOOT,
        labels=(Label(INSIDE, "① 반대쪽 옆 홈으로"),),
    ),
    Step(
        "plug",
        "선 꽂기",
        "J5 서보(ID 5)의 빈 커넥터에 꽂기",
        do=(
            "뒤로 빼낸 플러그를 J5 서보 뒷면의 빈 커넥터(그림)에 끝까지 꽂습니다. 받침 판의 터진 쪽으로 플러그 옆면을 잡고 밉니다.",
            "선을 받침 판 위에 납작하게 눕혀 서보 먼 끝 쪽으로 보냅니다.",
        ),
        check=("플러그가 끝까지 꽂혀 살짝 당겨도 안 빠진다.", "커넥터 둘에 선이 하나씩 꽂혀 있다."),
        stuck=ac.PHOTO,
        adds=(PLUG,),
        focus=(*W_BODY, *W1_INNER, INSIDE, PLUG),
        mount=(PLUG,),
        uses=(Use("servo_cable_300", note="뒤로 빼낸 플러그", count=False),),
        moves=(Move((PLUG,), ac.vec("j5", (0.0, -12.0, 0.0))),),
        view=W1_BACK_UP,
        zoom=1.6,
        center=_W1,
        labels=(Label(PLUG, "빈 커넥터에"),),
    ),
    Step(
        "w1-close",
        "덮개 달기",
        "손목 1 뒷덮개 다시 달기",
        do=("뒷덮개를 얹고 M3 × 12 둘을 조입니다. 선이 덮개 테 밑에 끼지 않게 봅니다.",),
        check=("덮개가 들뜨지 않았다.",),
        stuck=ac.PHOTO,
        focus=(*WRIST_SET, INSIDE, PLUG),
        uses=(Use("hexkey"),),
        moves=(
            Move((W1_COVER,), ac.vec("j5", (0.0, -30.0, 0.0))),
            Move((W1_COVER_SCREWS,), ac.vec("j5", (0.0, -16.0, 0.0)), 1.0),
        ),
        view=W1_BACK_UP,
        labels=(Label(W1_COVER, "손목 1 뒷덮개"),),
    ),
    Step(
        "attach",
        "손목 달기",
        "손목 1을 J4 출력 원판에 달기 (반 바퀴 돌려서, M3 × 10 넷)",
        do=(
            "아래팔 끝의 J4 출력 원판(로봇 왼쪽 면)에 손목 1의 발을 얹습니다. J4 원판의 귀(멈춤 기둥)가 손목 1의 뒤쪽(뒷덮개 "
            "쪽)을 보게 돌려 얹습니다. 다른 관절과 반 바퀴 다르게 다는 것입니다.",
            "얹을 때 손목 1 발 밑의 두 선이 원판의 혼 볼트 머리 양옆으로 가는지 봅니다.",
            "발 양옆 귀의 구멍 넷에 M3 × 10을 넣어 원판의 너트에 대각선 차례로 조입니다.",
            "렌치는 L자로 씁니다: 짧은 쪽을 나사 머리에 끝까지 넣고, 긴 쪽은 손목 몸통 바깥으로 눕힙니다 (그림). 돌릴 수 "
            "있는 만큼 돌린 뒤 렌치를 빼서 다시 끼워 돌립니다. 반대쪽 귀의 두 나사는 긴 쪽을 그쪽 바깥으로 눕힙니다.",
        ),
        warn=(
            "손목 끝이 무겁습니다. 다 조일 때까지 한 손으로 받칩니다.",
            "곧은 드라이버나 렌치의 긴 쪽은 나사 머리 위의 손목 1 몸통에 막혀 들어가지 않습니다.",
        ),
        check=(
            "발이 원판에 들뜨지 않고 앉았다.",
            "나사 넷의 머리가 다 닿았다.",
            "선 둘이 발과 원판 사이에 끼지 않았다.",
        ),
        stuck=ac.PHOTO,
        adds=(FEET, KEY),
        focus=(*FOREARM_END, *WRIST_SET, ACROSS, INSIDE, PLUG, FEET, KEY),
        mount=(FEET, KEY),
        uses=(Use("hexkey", note="L자로 (그림)"),),
        moves=(
            Move((*WRIST_SET, INSIDE, PLUG), (0.0, 0.0, -45.0)),
            Move((FEET,), (0.0, 0.0, -14.0), 1.4),
            Move((KEY,), (0.0, 0.0, -4.0), 2.8),
        ),
        view=FROM_LEFT,
        zoom=1.4,
        center=_J4,
        labels=(
            Label(f"{FEET} 3", "M3 × 10 × 4"),
            Label(KEY, "육각렌치: 짧은 쪽을 머리에, 긴 쪽은 바깥으로"),
        ),
    ),
)

ZERO_STEPS: tuple[Step, ...] = (
    Step(
        "power-on",
        "전원 넣기",
        "전원 넣고 팔을 J2·J3 핀으로 곧게 세우기",
        do=(
            "가변 전원을 켜 12.0 V를 본 뒤 빨강 플러그를 + 단자에 꽂습니다. 전류를 알려 주세요. Claude가 서보 일곱을 읽습니다.",
            "팔을 선반에서 들어 곧게 세우고, J2 핀(위팔 발의 귀)과 J3 핀(아래팔 발의 귀)을 꽂아 팔을 곧게 잡아 둡니다.",
        ),
        warn=(
            "아무것도 움직이지 않아야 합니다. 움직이거나 C.C 표시등이 켜지면 바로 빨강 플러그를 뺍니다.",
            "핀을 꽂아도 손목은 계속 한 손으로 받칩니다.",
        ),
        check=(
            "12.0 V이고 C.C 표시등이 꺼져 있다.",
            "Claude가 서보 일곱을 읽었다고 했다.",
            "J2·J3 핀이 꽂혀 팔이 서 있다.",
        ),
        stuck="화면을 사진으로 찍어 보내 주세요.",
        adds=(HAVE, PIN2, PIN3),
        still=(HAVE,),
        uses=(Use("psu"), Use("pc")),
        moves=(Move((PIN2,), (0.0, 0.0, -24.0)), Move((PIN3,), (0.0, 0.0, 24.0), 0.8)),
        view=FROM_LEFT,
        labels=(Label(PIN2, "J2 핀"), Label(PIN3, "J3 핀")),
    ),
    Step(
        "direction",
        "방향 재기",
        "J4를 + 쪽으로 돌려 방향 재기",
        do=(
            "Claude가 PC에 영점 창을 띄웁니다.",
            '"영점 핀을 빼고 …"가 나오면, 손목을 뒤로 90°쯤 젖혀(그림) 받쳐 든 채 Enter를 누릅니다.',
            '"+ 쪽으로 20°쯤 돌린 채 Enter"가 나오면, 손목을 앞쪽으로 20°쯤 되돌린 채 Enter를 누릅니다.',
        ),
        warn=("손목을 놓지 않습니다.",),
        check=('창에 "방향"이 나왔다.',),
        stuck="창에 나온 글을 사진으로 보내 주세요.",
        uses=(Use("pc", note="영점 창"),),
        turn2=(PIN_POSE, PIN_POSE - 20.0),  # 운동학 +20°는 설계 각 -20° (J4의 설계 부호가 -)
        view=FROM_LEFT,
        labels=(Label(f"{W}손목 1/손목 몸통", "+ 쪽: 손목이 앞으로 숙는 쪽"),),
    ),
    Step(
        "pin-in",
        "핀 꽂기",
        "J4 영점 핀 꽂고 Enter",
        do=(
            "손목을 다시 뒤로 젖혀 손목 1 발의 핀 귀가 아래팔 끝 집의 살 위에 오게 합니다 (J4 90°).",
            "손목 1 발 귀의 작은 구멍에 영점 핀을 꽂습니다. 살까지 쑥 들어가면 맞습니다. 안 들어가면 손목을 조금씩 돌려 맞춥니다.",
            '"영점 핀을 꽂고 …"가 나오면, 손목을 앞쪽으로 살짝 민 채 Enter를 누릅니다. 창에 세 줄이 나오면 됩니다.',
        ),
        warn=("핀을 공구로 박지 않습니다.",),
        check=("핀이 귀 위로 3 mm쯤 나와 있다.", "창에 세 줄이 나왔다."),
        stuck="핀을 꽂은 모습과 창을 사진으로 보내 주세요.",
        adds=(PIN4,),
        uses=(Use("pc", note="영점 창"),),
        moves=(Move((PIN4,), (0.0, 0.0, -24.0)),),
        joint2=PIN_POSE,
        view=FROM_LEFT,
        zoom=1.6,
        center=_PIN_AT,
        labels=(Label(PIN4, "J4 영점 핀 (손목 1 발의 귀)"),),
    ),
    Step(
        "apply",
        "한계 굽기",
        "각도 한계 굽기 (Claude)",
        do=(
            "Claude가 세 줄을 로봇 설정에 옮기고, 잰 방향을 설계로 본 방향(-1)과 견줍니다.",
            "값 적용을 돌려도 되는지 물으면 답합니다. 서보는 움직이지 않습니다.",
        ),
        check=("Claude가 값 적용이 끝났다고 했다.",),
        uses=(Use("pc"),),
        joint2=PIN_POSE,
        view=FROM_LEFT,
        zoom=1.6,
        center=_PIN_AT,
    ),
    Step(
        "pin-out",
        "핀 빼기",
        "J4 영점 핀 빼기",
        do=("손목을 받친 채 롱노즈로 J4 핀을 곧게 당겨 뺍니다. J2·J3 핀은 그대로 둡니다.",),
        check=("J4 핀을 뺐다.",),
        uses=(Use("longnose"),),
        joint2=PIN_POSE,
        view=FROM_LEFT,
        zoom=1.6,
        center=_PIN_AT,
        labels=(Label(PIN4, "곧게 당겨 뺀다"),),
    ),
    Step(
        "check",
        "확인",
        "손목을 + 쪽으로 90° 돌려 읽어 보기",
        do=(
            "손목을 앞쪽으로 90°쯤 돌려 그림 자리(범위 가운데)에 둡니다.",
            "Claude가 J4 각을 읽습니다. 180°쯤이면 맞습니다.",
        ),
        check=("Claude가 180°쯤으로 읽었다고 했다.",),
        uses=(Use("pc"),),
        hide=(PIN4,),
        turn2=(PIN_POSE, 0.0),
        view=FROM_LEFT,
    ),
    Step(
        "turn-j4",
        "돌려 보기",
        "손목을 손으로 끝까지 돌려 선 보기",
        do=(
            "손목을 천천히 한쪽 끝까지, 다시 반대쪽 끝까지 돌려 봅니다. 끝에서 멈춤 기둥이 살에 닿으면 더 밀지 않습니다.",
            "아래팔 창에서 손목 1로 가는 선이 손목 1 발의 귀에 걸리거나 팽팽해지지 않는지 봅니다. D405 선이 끼지 않는지도 "
            "봅니다.",
            "손목을 그림 자리(범위 가운데)로 되돌립니다.",
        ),
        warn=("선이 걸리면 억지로 돌리지 말고 사진을 보내 주세요.",),
        check=("양 끝까지 선이 걸리지 않고 돈다.",),
        uses=(Use("phone", note="걸리면 사진", count=False),),
        hide=(PIN4,),
        turn2=(160.0, -160.0),
        view=FROM_LEFT,
    ),
    Step(
        "power-off",
        "전원 끄기",
        "전원 끄고 J2·J3 핀 빼기",
        do=(
            "빨강 플러그를 뺀 뒤 가변 전원을 끕니다. Claude가 기록을 올립니다.",
            "팔을 두 손으로 받친 채 J3 핀, J2 핀을 차례로 곧게 당겨 뺍니다.",
        ),
        warn=("핀을 빼면 팔이 넘어갑니다. 두 손으로 받칩니다.",),
        check=("가변 전원이 꺼졌다.", "핀 둘을 뺐고 팔을 받치고 있다."),
        uses=(Use("psu"), Use("longnose")),
        hide=(PIN2, PIN3, PIN4),
        view=FROM_LEFT,
    ),
)


def _rest(sid: str, kind: str) -> Step:
    work = kind == "work"
    return Step(
        sid,
        "쉬는 자세",
        "쉬는 자세로 두기",
        kind=kind,
        do=(
            "J1 몸통을 위에서 볼 때 시계 방향으로 55°쯤 돌립니다.",
            "위팔을 앞으로 50°쯤 숙이며 아래팔을 접어 아래팔 끝 집을 선반에 얹고, 손목은 아래로 늘어지게 둡니다.",
            "D405 선은 아무 데도 끼지 않게 늘어뜨려 둡니다 (찍찍이로 묶는 일은 따로 합니다).",
            "다 된 모습을 사진으로 보내 주세요.",
        )
        if work
        else (
            f"손목을 J4(ID {SERVO_ID})에 달고 J4의 영점을 구웠습니다. 팔 조립이 끝났습니다.",
            "다음은 처음 전원(서보 일곱의 통신 시간, 어깨 버티기)과 D405 선 묶기입니다. 대화창에서 이어서 정합니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요.",
        ),
        check=(
            "아래팔 끝이 선반에 얹혀 팔이 스스로 서 있다.",
            "손목이 아래로 늘어져 책상에 닿지 않는다.",
        )
        if work
        else (),
        adds=(REST_PREFIX,) if work else (),
        uses=(Use("phone", note="다 된 모습", count=False),) if work else (),
        hide=UPRIGHT,
        joint=ac.REST_J1,
        view=REST_VIEW,
    )


def _hide_after(steps: tuple[Step, ...], start: str, names: tuple[str, ...]) -> tuple[Step, ...]:
    """start 단계 뒤의 단계에서 names를 숨긴다 (뗀 가짜 아래팔 끝, 다 쓴 렌치: 본체에 붙은 채로 남아 있어서)."""
    out, after = [], False
    for s in steps:
        if after:
            s = replace(s, hide=(*s.hide, *names))
        if s.id == start:
            after = True
        out.append(s)
    return tuple(out)


STEPS: tuple[Step, ...] = _hide_after(
    _hide_after(
        (PREP, *WRIST_STEPS, *ZERO_STEPS, _rest("rest", "work"), _rest("done", "end")),
        "ff-off",
        FAKE,
    ),
    "attach",
    (KEY,),
)

GUIDE = Guide(
    id="wrist-join",
    kind="assembly",
    title="손목 잇기와 J4 영점",
    sub=f"손목 끝을 J4 원판에 · J4 → J5 선 · J4(ID {SERVO_ID}) 영점 · 쉬는 자세",
    scene=scene,
    steps=STEPS,
    items=OTHER,
    axis2=J4_AXIS,
)
