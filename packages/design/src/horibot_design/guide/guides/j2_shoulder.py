"""어깨(J2) 조립: 출력 원판, 허브 판, ST-3120(ID 2)과 받침, 혼 볼트 여덟, J1 뒷덮개 (0029의 어깨 관절).

순서는 0029의 어깨 조립 순서를 형상으로 다시 따진 것이다 (2026-11-25):
- 허브 판(지름 43 mm)은 누름 틀 다리 사이(40 mm)를 지나가지 못하고, 다리 앞 빈 곳(누름 틀 고리 길이 4.0 mm)도 허브
  판 두께(2.8 mm)에 기울일 자리가 없다 (다리를 비키려면 5.5° 넘게 기울여야 하는데 그러면 5.5 mm가 든다). 그래서 J1
  베이스 조립에서 넣어 둔 누름 틀을 한 번 뺀다. 허브 판은 혼에 따로 달지 않고(혼 볼트가 묶는다) 원통 속에 먼저 넣고,
  누름 틀을 다시 넣으면 허브 판이 다리 앞에 갇힌다 (서보와 한 덩어리로 넣은 J1과 다르다).
- 허브 판을 가운데에 잡으려고 출력 원판을 단 뒤 혼 볼트 둘(1, 5)을 앞에서 꽂는다. 볼트 끝이 허브 판 뒤로 3.8 mm
  나와 판이 처지지 않고, 서보를 넣으면 혼 구멍이 그 끝에 맞는다.
- 서보 선(J1 → J2, 900 mm)은 집 속에서 꽂을 수 없다 (목 속 옆 틈 9.5 mm, 플러그 9.6 mm). 플러그를 목 앞면의 선 창
  (9.6 × 6 mm)으로 넣어 원통의 열린 뒤로 뺀 뒤 서보에 꽂고, 서보를 넣으며 남는 선을 창 밖으로 당긴다 (0031).
- 혼 볼트 여덟 가운데 위팔 발의 선 틈(0031, links.FOOT_SLOT_Z) 쪽 넷(5 ~ 8, 귀를 위로 두면 앞쪽)은 둥근머리, 반대쪽
  넷(1 ~ 4)은 가진 유두다 (status의 나사: 선이 머리 가장자리에 걸친다).
- 누르개(띠 밑, J1 몸통 귀)는 내일 J3 선과 같이 잡는다. 오늘은 선이 그 길에 놓이게만 한다.

장면 좌표는 J1의 서보 좌표다 (parts/base.py: x 로봇 앞, y 위, z 로봇 오른쪽). 어깨는 b.as_child로 옮긴다 (어깨의
x → 위, y(출력 쪽) → 로봇 왼쪽, z → 뒤). J2 축은 둘째 관절(axis2)이다.
"""

from __future__ import annotations

from dataclasses import replace

from horibot_design.guide.catalog import printed_items
from horibot_design.guide.guides import base_pi
from horibot_design.guide.model import Guide, Item, Label, Move, ScenePart, Step, Tag, Use, View
from horibot_design.parts import base as b
from horibot_design.parts import shoulder as sh

SERVO_ID = 2
HAVE = "있는 것/"
C = base_pi.DONE_PREFIX  # J1 베이스 부품의 이름 앞부분
# J1 베이스 조립에서 넣어 둔 어깨 누름 틀과 그 너트, 앞줄 벽 나사: 이 안내에서 뺐다 다시 넣는다
_ACTIVE = (f"{C}어깨 누름 틀", f"{C}어깨 너트", f"{C}어깨 벽 나사")

DONE = Item(
    "assembled_base2",
    "조립해 둔 베이스",
    "bought",
    "J1 베이스(ID 1, 영점을 잰 것), 파이, 서보 어댑터",
)
SERVO_ITEM = Item(
    "servo_st3120_j2",
    f"서보 ST-3120 · ID {SERVO_ID}",
    "bought",
    f"혼이 달린 것. ID {SERVO_ID} 이름표가 붙은 서보 (어깨, 뒷면 혼은 뺀 것)",
)
PRINTED: dict[str, Item] = printed_items(
    {**sh.LABELS, "j1_cover": b.LABELS["j1_cover"]},
    {
        k: (None, None, "black")
        for k in (
            "shoulder_out_disc",
            "shoulder_hub_plate",
            "shoulder_retainer",
            "shoulder_carrier",
            "shoulder_bearing_pusher",
            "j1_cover",
        )
    },
)
OTHER: dict[str, Item] = {
    it.key: it
    for it in (
        DONE,
        SERVO_ITEM,
        Item(
            "m3x20_cap",
            "M3 × 20 유두",
            "hardware",
            "유두 렌치볼트 (머리가 높은 원통). 2.5 mm 렌치",
            draw={"t": "cap", "L": 20},
        ),
        Item(
            "m25x6",
            "M2.5 × 6",
            "hardware",
            "둥근머리 렌치볼트 (가진 것). 서보 뒷면",
            draw={"t": "screw", "L": 6},
        ),
        Item(
            "servo_cable_900",
            "서보 선 900 mm",
            "bought",
            "J1 서보에 꽂아 둔 것 (어깨로)",
            icon="wire",
        ),
        Item("hexkey_25", "육각렌치 2.5 mm", "tool", "유두 볼트용", icon="hex"),
        Item("hexkey_15", "육각렌치 1.5 mm", "tool", "M2.5 볼트용", icon="hex"),
        Item("pencil", "샤프", "tool", "뻑뻑한 너트를 끝으로 살살 밀 때", icon="pen"),
        Item("phone", "휴대폰", "tool", "사진을 찍어 대화창에 보낸다", icon="phone"),
    )
}
ITEMS_HERE: dict[str, Item] = {**PRINTED, **OTHER}


def S(sx: float, sy: float, sz: float) -> tuple[float, float, float]:
    """어깨의 서보 좌표 점을 장면 좌표로 (b.child_at, J1 각 0)."""
    return (-sz, b.Y_DESK + b.SHOULDER_H + sx, sh.J1_AXIS_Y - sy)


J2_AXIS = (S(0.0, 0.0, 0.0), (0.0, 0.0, -1.0))  # 어깨의 +y(출력 쪽) = 로봇 왼쪽

# 어깨 부품 이름 → (이 안내의 이름, 목록 키, J2와 같이 도는지)
_INNER = {
    "허브 판": ("허브 판", "shoulder_hub_plate", True),
    "출력 원판": ("출력 원판", "shoulder_out_disc", True),
    "서보 받침": ("서보 받침", "shoulder_carrier", False),
    "서보 몸통": ("서보 몸통", SERVO_ITEM.key, False),
    "혼": ("혼", SERVO_ITEM.key, True),
    "플러그": ("J2 선 플러그", "servo_cable_900", False),
    "선 자리": ("J2 선", "servo_cable_900", False),
}


def _hw(name: str) -> tuple[str, str, bool] | None:
    """어깨 나사 이름 → (이 안내의 이름, 목록 키, J2와 같이 도는지). 이 안내에서 쓰지 않으면 None."""
    k = name.rsplit(" ", 1)[1]
    if name.startswith("벽 나사 (서보 받침)"):
        return (f"뒷줄 벽 나사 {k}", "m3x8", False)
    if name.startswith("혼 볼트"):
        cap = int(k) <= 4  # 1 ~ 4는 위팔 발의 선 틈 반대쪽 (유두)
        return (f"혼 볼트 {k}", "m3x20_cap" if cap else "m3x20", True)
    if name.startswith("멈춤 나사"):
        return ("멈춤 나사", "m3x10", True)
    if name == "너트 (멈춤 기둥)":
        return ("너트 (멈춤 기둥)", "nut_m3", True)
    if name.startswith("너트 (출력 원판)"):
        return (f"너트 (출력 원판) {k}", "nut_m3", True)
    if name.startswith("M2.5x6"):
        return (f"서보 뒷면 나사 {k}", "m25x6", False)
    if name.startswith("덮개 나사"):
        return (f"덮개 나사 {k}", "m3x12", False)
    if name == "너트 (서보 받침)":
        return ("너트 (서보 받침)", "nut_m3", False)
    return None  # 누름 틀의 벽 나사와 너트는 J1 베이스에 이미 있다


def _active_key(name: str) -> str:
    if name.startswith(f"{C}어깨 누름 틀"):
        return "shoulder_retainer"
    return "nut_m3" if name.startswith(f"{C}어깨 너트") else "m3x8"


def scene() -> list[ScenePart]:
    out: list[ScenePart] = []
    for p in base_pi.scene():
        if p.name.startswith(_ACTIVE):
            out.append(replace(p, key=_active_key(p.name), count=False))
        else:
            out.append(replace(p, name=f"{HAVE}{p.name}", key=DONE.key, count=False))
    for p in sh.inner_parts(b.NECK_X):
        if p.name not in _INNER:
            continue
        name, key, turns = _INNER[p.name]
        count = key != SERVO_ITEM.key or name == "서보 몸통"
        if key == "servo_cable_900":
            count = False
        out.append(ScenePart(name, b.as_child(p.shape), key, p.color, True, count, moving2=turns))
    out.append(ScenePart("J1 뒷덮개", b.as_child(b.j1_cover()), "j1_cover", "cap", True))
    for p in sh.hardware():
        got = _hw(p.name)
        if got is None:
            continue
        name, key, turns = got
        out.append(ScenePart(name, b.as_child(p.shape), key, "metal", True, True, moving2=turns))
    return out


# ------------------------------------------------------------------------------------------------
# 이름

RET = f"{C}어깨 누름 틀"
RET_NUTS = f"{C}어깨 너트"
FRONT_SCREWS = f"{C}어깨 벽 나사"
HUB = "허브 판"
DISC = "출력 원판$"
RIM_NUTS = "너트 (출력 원판)"
STOP = ("멈춤 나사", "너트 (멈춤 기둥)")
DISC_SET = (DISC, RIM_NUTS, *STOP)
CARRIER = "서보 받침"
CAR_NUTS = "너트 (서보 받침)"
BACK_SCREWS = "서보 뒷면 나사"
SERVO = "서보 몸통"
HORN = "혼$"
PLUG, WIRE = "J2 선 플러그", "J2 선$"
SERVO_SET = (SERVO, HORN, CARRIER, CAR_NUTS, BACK_SCREWS, PLUG, WIRE)
HOLD_BOLTS = ("혼 볼트 1$", "혼 볼트 5$")
OTHER_BOLTS = tuple(f"혼 볼트 {k}$" for k in (2, 3, 4, 6, 7, 8))
REAR_SCREWS = "뒷줄 벽 나사"
COVER, COVER_SCREWS = "J1 뒷덮개", "덮개 나사"

FROM_RIGHT = View((0, 1, 0), (0.3, 0, 1), 22, 0)  # 원통의 열린 뒤(로봇 오른쪽)가 보는 사람 쪽
FROM_LEFT = View((0, 1, 0), (0.3, 0, -1), 22, 0)  # 출력 원판 쪽(로봇 왼쪽)
_CENTER = S(0.0, 0.0, 0.0)
_WINDOW = S(sum(sh.WIN_X) / 2, sh.FRONT, 0.0)
PHOTO = "막히면 그 자리를 사진으로 보내 주세요."

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "서보 전원이 꺼져 있는지(빨강 플러그를 뺐는지) 봅니다. 서보 선은 전원을 끈 채로 꽂습니다.",
            "출력 부품: 어깨 출력 원판, 어깨 허브 판(다시 뽑은 것), 어깨 서보 받침, J1 뒷덮개, 어깨 베어링을 미는 도구.",
            "나사는 M3 × 20을 머리별로 나눕니다: 둥근머리 넷, 유두 넷. 그 밖에 M3 × 10 하나, M3 × 8 셋, M3 × 12 둘, "
            "M2.5 × 6 여섯, M3 너트 열.",
            f"ID {SERVO_ID} 이름표가 붙은 ST-3120을 꺼냅니다.",
        ),
        check=(
            "빨강 플러그가 빠져 있다.",
            "출력 부품 다섯과 서보(ID 2)",
            "M3 × 20 둥근머리 4와 유두 4, M3 × 10 1, M3 × 8 3, M3 × 12 2, M2.5 × 6 6, M3 너트 10",
        ),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    Step(
        "disc-prep",
        "원판 준비",
        "출력 원판에 너트와 멈춤 나사 달기 (손에 들고)",
        do=(
            "출력 원판 테의 튀어나온 귀(멈춤 기둥 위)에서, 나사 머리 자리 바닥의 얇은 막을 육각렌치 끝으로 꾹 눌러 뚫습니다.",
            "원판 밑면(저널이 나온 쪽) 테 둘레의 육각 홈 여섯에 M3 너트를 하나씩 눌러 끼웁니다. 위팔 나사가 여기에 물립니다.",
            "귀 위에서 M3 × 10 둥근머리를 꽂아, 밑면 멈춤 기둥 끝에서 M3 너트를 걸고 2 mm 렌치로 조입니다.",
        ),
        warn=("막만 뚫습니다. 너트가 헐거우면 빠지지 않게 원판을 밑면이 위로 가게 들고 다룹니다.",),
        check=(
            "너트 여섯이 홈에 들어가 뒤집어도 안 빠진다.",
            "멈춤 나사가 기둥 끝 너트에 조여 있다.",
        ),
        stuck=PHOTO,
        adds=DISC_SET,
        focus=DISC_SET,
        uses=(Use("hexkey"),),
        moves=(
            Move((RIM_NUTS,), (0.0, 0.0, 10.0)),
            Move(("멈춤 나사",), (0.0, 0.0, -14.0), 0.8),
            Move(("너트 (멈춤 기둥)",), (0.0, 0.0, 10.0), 1.4),
        ),
        view=View((0, 1, 0), (0.2, 0, 1), 30, 0),
        labels=(
            Label(f"{RIM_NUTS} 1", "M3 너트 × 6 (밑면)"),
            Label("멈춤 나사", "M3 × 10 (귀 위)"),
        ),
    ),
    Step(
        "carrier",
        "받침 달기",
        f"서보(ID {SERVO_ID})에 받침 달기",
        do=(
            "서보 받침의 날개 셋 옆 홈에 M3 너트를 하나씩 세워 밀어 넣습니다. 뻑뻑하면 샤프 끝으로 밉니다.",
            "받침의 판을 서보 뒷면(혼 반대쪽)에 대고, M2.5 × 6 여섯을 1.5 mm 렌치로 조입니다.",
        ),
        warn=("옆면 커넥터 둘이 날개 사이로 드러나게 댑니다. 받침이 한 방향으로만 맞습니다.",),
        check=("받침이 서보 뒷면에 붙었다.", "날개의 너트 구멍 셋이 가운데에 보인다."),
        stuck=PHOTO,
        adds=(SERVO, HORN, CARRIER, CAR_NUTS, BACK_SCREWS),
        focus=(SERVO, HORN, CARRIER, CAR_NUTS, BACK_SCREWS),
        uses=(Use("hexkey_15"), Use("pencil", note="뻑뻑할 때만", count=False)),
        moves=(
            Move((CAR_NUTS,), (0.0, 10.0, 0.0)),
            Move((CARRIER, CAR_NUTS), (0.0, 0.0, 22.0), 0.8),
            Move((BACK_SCREWS,), (0.0, 0.0, 14.0), 1.6),
        ),
        view=View((0, 1, 0), (0.4, 0, 1), 25, 0),
        labels=(
            Label(SERVO, f"ST-3120 · ID {SERVO_ID}"),
            Label(f"{BACK_SCREWS} 1", "M2.5 × 6 × 6"),
        ),
    ),
    Step(
        "ret-out",
        "누름 틀 빼기",
        "어깨 누름 틀 빼기 (앞줄 벽 나사 셋)",
        do=(
            "어깨 원통의 위, 앞, 뒤에 하나씩 있는 앞줄 벽 나사(원판 쪽 줄) 셋을 2 mm 렌치로 풉니다.",
            "원통의 열린 뒤(로봇 오른쪽)로 누름 틀을 곧게 빼냅니다. 다리 옆 홈의 너트 셋이 빠지지 않게 누름 틀을 바로 든 채 "
            "옆에 둡니다.",
            "벽 나사 셋은 다시 쓰니 모아 둡니다.",
        ),
        warn=(
            "허브 판이 누름 틀 다리 사이(40 mm)보다 커서 지나가지 못합니다. 그래서 누름 틀을 한 번 뺍니다.",
        ),
        check=("누름 틀을 뺐고 너트 셋이 홈에 남아 있다.", "원통 속 베어링이 그대로 자리에 있다."),
        stuck=PHOTO,
        adds=(HAVE, RET, RET_NUTS, FRONT_SCREWS),
        still=(HAVE, RET, RET_NUTS, FRONT_SCREWS),
        uses=(Use("hexkey"),),
        view=FROM_RIGHT,
        zoom=1.8,
        center=_CENTER,
        shell=(f"{HAVE}{C}J1 몸통$",),
        ghost=True,
        labels=(
            Label(RET, "누름 틀 (뺀다)"),
            Label(f"{FRONT_SCREWS} 3", "앞줄 벽 나사 × 3 (푼다)"),
        ),
    ),
    Step(
        "hub-in",
        "허브 판",
        "허브 판을 원통 속에 넣기",
        do=(
            "허브 판을 넓고 평평한 테가 원판 쪽(베어링 쪽), 가운데 돋움이 열린 뒤 쪽을 보게 듭니다.",
            "열린 뒤로 넣어 베어링에 닿을 때까지 밀어 넣습니다. 손가락이 안 닿으면 미는 도구로 살살 밉니다.",
        ),
        check=("허브 판이 베어링 바로 뒤에 들어갔다.",),
        stuck=PHOTO,
        adds=(HUB,),
        uses=(Use("shoulder_bearing_pusher", note="손이 안 닿으면", count=False),),
        hide=(RET, RET_NUTS, FRONT_SCREWS),
        moves=(Move((HUB,), (0.0, 0.0, 70.0)),),
        view=FROM_RIGHT,
        zoom=1.8,
        center=_CENTER,
        shell=(f"{HAVE}{C}J1 몸통$",),
        cut=True,
        labels=(Label(HUB, "허브 판 (돋움이 뒤로)"),),
    ),
    Step(
        "ret-in",
        "누름 틀 넣기",
        "누름 틀 다시 넣고 잠그기 (앞줄 벽 나사 셋)",
        do=(
            "누름 틀을 다리 셋이 위, 앞, 뒤로 가게 열린 뒤로 넣습니다. 허브 판은 다리 앞에 갇혀 빠지지 않습니다.",
            "미는 도구로 누름 틀을 끝까지 밀어 고리가 베어링에 닿게 한 채, 앞줄 벽 나사 셋을 다시 넣어 조입니다.",
        ),
        warn=("누름 틀을 민 채 조입니다. 나사가 안 맞으면 누름 틀을 조금 돌려 구멍을 맞춥니다.",),
        check=(
            "벽 나사 셋이 다 조여졌다.",
            "원통을 흔들어도 허브 판이 속에서 달그락거리기만 하고 빠지지 않는다.",
        ),
        stuck=PHOTO,
        uses=(Use("shoulder_bearing_pusher", count=False), Use("hexkey")),
        moves=(
            Move((RET, RET_NUTS), (0.0, 0.0, 70.0)),
            Move((f"{FRONT_SCREWS} 1",), (-12.0, 0.0, 0.0), 1.2),
            Move((f"{FRONT_SCREWS} 2",), (12.0, 0.0, 0.0), 1.5),
            Move((f"{FRONT_SCREWS} 3",), (0.0, 12.0, 0.0), 1.8),
        ),
        view=FROM_RIGHT,
        zoom=1.8,
        center=_CENTER,
        shell=(f"{HAVE}{C}J1 몸통$",),
        cut=True,
        labels=(Label(f"{FRONT_SCREWS} 3", "M3 × 8 × 3"),),
    ),
    Step(
        "disc-on",
        "원판 끼우기",
        "출력 원판 끼우기 (로봇 왼쪽에서)",
        do=(
            "출력 원판을 귀가 위로 오게 들고, 저널을 원통 앞 끝(로봇 왼쪽)의 베어링 안쪽 링에 맞춥니다.",
            "귀 밑의 멈춤 기둥이 원통 앞면의 둥근 홈에 들어가게 하며, 저널을 안쪽 링 속으로 끝까지 꾹 밀어 넣습니다.",
        ),
        warn=("저널은 안쪽 링에 꽉 끼게 만들었습니다. 원판의 넓은 테를 고르게 밀어 넣습니다.",),
        check=("원판 테와 원통 앞 끝 사이 틈이 고르다.", "멈춤 기둥이 홈 속에 있다."),
        stuck=PHOTO,
        mount=DISC_SET,
        uses=(Use("shoulder_out_disc", note="너트와 멈춤 나사를 단 것", count=False),),
        moves=(Move(DISC_SET, (0.0, 0.0, -40.0)),),
        view=FROM_LEFT,
        zoom=1.6,
        center=_CENTER,
        labels=(Label(DISC, "출력 원판 (귀가 위)"),),
    ),
    Step(
        "hold",
        "볼트 둘",
        "혼 볼트 둘로 허브 판 잡기",
        do=(
            "원판의 볼트 구멍 여덟 가운데 대각선의 둘(그림의 1과 5)에 M3 × 20을 앞에서 꽂아 끝까지 밀어 넣습니다. 1은 유두, "
            "5는 둥근머리입니다.",
            "볼트 끝이 원판, 베어링, 허브 판을 지나 허브 판 뒤로 조금 나옵니다. 허브 판이 볼트에 걸려 가운데에 섭니다.",
            "아직 조이지 않습니다 (혼이 없습니다).",
        ),
        check=("볼트 둘이 허브 판 구멍까지 들어갔다.",),
        stuck="볼트가 허브 판 구멍에 안 들어가면 원판 쪽에서 볼트를 꽂은 채 허브 판을 렌치 끝으로 살살 돌려 맞춥니다.",
        adds=HOLD_BOLTS,
        moves=(Move(HOLD_BOLTS, (0.0, 0.0, -22.0)),),
        view=FROM_LEFT,
        zoom=1.8,
        center=_CENTER,
        labels=(Label("혼 볼트 1$", "1 (유두)"), Label("혼 볼트 5$", "5 (둥근머리)")),
    ),
    Step(
        "window",
        "선 꿰기",
        "어깨로 가는 선을 선 창에 꿰기",
        do=(
            "J1 서보에서 나와 베이스 뒤로 늘어진 900 mm 선의 플러그 끝을 찾습니다.",
            "J1 몸통 왼쪽 면(원판 쪽), 어깨 원통 밑의 목에 난 작은 네모 창(선 창)에 플러그를 밖에서 넣습니다.",
            "속을 지나 원통의 열린 뒤(로봇 오른쪽)로 플러그를 빼냅니다.",
        ),
        check=("플러그가 창을 지나 열린 뒤로 나왔다.",),
        stuck="플러그가 창에 안 들어가면 플러그를 세워(넓은 면을 창의 긴 쪽으로) 넣어 봅니다.",
        uses=(Use("servo_cable_900", note="J1 서보에 꽂아 둔 것", count=False),),
        view=View((0, 1, 0), (0.2, 0, -1), 10, 0),
        zoom=1.6,
        center=_WINDOW,
        tags=(Tag(_WINDOW, "선 창 (밖에서 넣는다)"),),
    ),
    Step(
        "plug",
        "선 꽂기",
        f"선을 서보(ID {SERVO_ID})에 꽂기",
        do=(
            "열린 뒤로 빼낸 플러그를 서보 옆면의 커넥터(그림 자리)에 끝까지 꽂습니다.",
            "선이 받침 날개 사이로 나가게 둡니다.",
        ),
        warn=("집 속에서는 커넥터에 손이 닿지 않아서, 꼭 서보를 넣기 전에 꽂습니다.",),
        check=("살짝 당겨도 플러그가 빠지지 않는다.",),
        stuck=PHOTO,
        adds=(PLUG, WIRE),
        focus=SERVO_SET,
        uses=(Use("servo_cable_900", note="선 창에 꿴 것", count=False),),
        moves=(Move((PLUG,), (-12.0, 0.0, 0.0)),),
        view=View((0, 1, 0), (-0.6, 0, 1), 25, 0),
        labels=(Label(PLUG, "옆면 커넥터"),),
    ),
    Step(
        "servo-in",
        "서보 넣기",
        f"서보(ID {SERVO_ID}) 넣기",
        do=(
            "서보를 혼이 원판 쪽, 받침이 열린 뒤 쪽, 먼 끝이 아래(목 쪽)로 가게 들고 열린 뒤로 밀어 넣습니다. 받침 날개가 "
            "벽을 따라 미끄러집니다.",
            "넣는 동안 남는 선을 선 창 밖으로 살살 당겨, 속에 고리가 생기지 않게 합니다.",
            "혼이 볼트 끝에 닿으면 멈추고, 혼(원판)을 손으로 조금씩 돌려 볼트 끝이 혼의 구멍에 맞게 합니다.",
        ),
        warn=("선이 서보와 벽 사이에 끼지 않게 봅니다.",),
        check=(
            "서보가 혼이 볼트 끝에 닿을 때까지 들어갔다.",
            "선이 창 밖으로 당겨져 속에 끼인 곳이 없다.",
        ),
        stuck=PHOTO,
        mount=SERVO_SET,
        uses=(Use(SERVO_ITEM.key, note="받침과 선을 단 것", count=False),),
        moves=(Move(SERVO_SET, (0.0, 0.0, 90.0)),),
        view=FROM_RIGHT,
        zoom=1.5,
        center=_CENTER,
        shell=(f"{HAVE}{C}J1 몸통$",),
        cut=True,
        labels=(Label(SERVO, f"ID {SERVO_ID}"),),
    ),
    Step(
        "horn-bolts",
        "혼 볼트",
        "혼 볼트 여덟 조이기 (둥근머리 넷, 유두 넷)",
        do=(
            "볼트 1과 5를 번갈아 몇 바퀴씩 조입니다. 서보가 원판 쪽으로 조금씩 당겨 들어옵니다.",
            "나머지 여섯을 넣습니다: 귀를 위로 두고 볼 때 앞쪽 반의 6, 7, 8은 둥근머리, 뒤쪽 반의 2, 3, 4는 유두입니다.",
            "여덟을 대각선 차례로 조금씩, 끝까지 조입니다 (둥근머리는 2 mm, 유두는 2.5 mm 렌치).",
        ),
        warn=(
            "앞쪽 넷은 꼭 둥근머리입니다. 위팔을 달면 그 위로 선이 지나갑니다.",
            "한 볼트만 먼저 끝까지 조이지 않습니다.",
        ),
        check=("볼트 여덟의 머리가 원판에 앉았다.", "앞쪽 반이 둥근머리, 뒤쪽 반이 유두다."),
        stuck=PHOTO,
        adds=OTHER_BOLTS,
        uses=(Use("hexkey"), Use("hexkey_25")),
        moves=(Move(OTHER_BOLTS, (0.0, 0.0, -20.0)),),
        view=FROM_LEFT,
        zoom=1.8,
        center=_CENTER,
        labels=(
            Label("혼 볼트 6$", "둥근머리 (앞쪽 넷)"),
            Label("혼 볼트 3$", "유두 (뒤쪽 넷)"),
        ),
    ),
    Step(
        "lock",
        "서보 잠금",
        "서보 잠그기 (뒷줄 벽 나사 셋)",
        do=(
            "원통의 위, 앞, 뒤에서 열린 뒤 쪽 줄(뒷줄)의 벽 나사 구멍 셋에 M3 × 8 둥근머리를 넣어 2 mm 렌치로 조입니다.",
            "구멍이 앞뒤로 조금 길어서 서보를 밀거나 당기지 않고 지금 자리 그대로 잠급니다.",
        ),
        check=("뒷줄 벽 나사 셋이 조여졌다.",),
        stuck=PHOTO,
        adds=(REAR_SCREWS,),
        uses=(Use("hexkey"),),
        moves=(
            Move((f"{REAR_SCREWS} 1",), (-12.0, 0.0, 0.0)),
            Move((f"{REAR_SCREWS} 2",), (12.0, 0.0, 0.0), 0.4),
            Move((f"{REAR_SCREWS} 3",), (0.0, 12.0, 0.0), 0.8),
        ),
        view=FROM_RIGHT,
        zoom=1.6,
        center=_CENTER,
        labels=(Label(f"{REAR_SCREWS} 3", "M3 × 8 × 3 (뒷줄)"),),
    ),
    Step(
        "cover",
        "뒷덮개",
        "J1 뒷덮개 달기",
        do=(
            "J1 뒷덮개로 원통의 열린 뒤를 덮습니다. 아래쪽이 목을 따라 J1 몸통 바닥 가까이까지 내려옵니다.",
            "M3 × 12 둘을 넣어 2 mm 렌치로 조입니다.",
        ),
        check=("덮개가 뜨지 않고 붙었다.",),
        stuck=PHOTO,
        adds=(COVER, COVER_SCREWS),
        uses=(Use("hexkey"),),
        moves=(
            Move((COVER,), (0.0, 0.0, 25.0)),
            Move((COVER_SCREWS,), (0.0, 0.0, 14.0), 0.8),
        ),
        view=FROM_RIGHT,
        zoom=1.4,
        center=_CENTER,
        labels=(Label(COVER, "J1 뒷덮개"), Label(f"{COVER_SCREWS} 1", "M3 × 12 × 2")),
    ),
    Step(
        "turn",
        "돌려 보기",
        "어깨를 손으로 끝까지 돌려 보기",
        do=(
            "출력 원판을 잡고 한쪽 끝까지 천천히 돌린 뒤, 반대쪽 끝까지 돌립니다.",
            "귀가 위로 오게 되돌려 둡니다.",
        ),
        warn=(
            "서보 기어 때문에 뻑뻑한 것이 정상입니다. 끝에서 기둥이 살에 닿으면 더 밀지 않습니다.",
        ),
        check=("양쪽 끝까지 걸리거나 긁히는 곳 없이 돈다.", "귀가 위로 와 있다."),
        uses=(Use("phone", note="이상하면 사진", count=False),),
        turn2=(-163.0, 163.0),
        view=FROM_LEFT,
    ),
    Step(
        "route",
        "선 정리",
        "어깨로 가는 선 정리",
        do=(
            "선 창에서 나온 선을 J1 몸통 왼쪽 면을 따라 뒤로, 뒤 왼쪽 모서리를 돌아 J1 몸통 뒤쪽 귀 위로 보냅니다.",
            "거기서 베이스 몸통 윗부분의 띠를 따라 내려가 몸통 뒤 왼쪽으로 내려보냅니다. 남는 선(약 50 cm)은 어댑터 뒤 판의 "
            "창으로 내려 책상 뒤에 고리로 둡니다.",
            "누르개로 잡는 것은 내일 J3 선과 같이 합니다. 오늘은 선이 이 길에 놓이게만 합니다.",
            "다 된 모습을 사진으로 보내 주세요.",
        ),
        check=("선이 바짝 꺾이거나 당겨진 곳이 없다.", "남는 선이 책상 뒤로 늘어져 있다."),
        uses=(Use("phone"),),
        view=View((0, 1, 0), (-0.6, 0, -1), 25, 0),
        tags=(
            Tag(_WINDOW, "선 창"),
            Tag((-b.R_CORE * 0.9, b.FRONT - 10.0, -b.R_CORE * 0.45), "띠를 따라 뒤 왼쪽으로"),
        ),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            f"어깨(J2, ID {SERVO_ID}) 조립이 끝났습니다. J2 영점은 내일 위팔을 단 뒤에 잽니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요.",
        ),
        view=FROM_LEFT,
    ),
)

GUIDE = Guide(
    id="j2-shoulder",
    kind="assembly",
    title="어깨(J2) 조립",
    sub=f"출력 원판과 허브 판 · ST-3120 ID {SERVO_ID} · 혼 볼트 여덟 · J1 뒷덮개",
    scene=scene,
    steps=STEPS,
    items=ITEMS_HERE,
    bundle="arm-black-1",
    axis2=J2_AXIS,
)
