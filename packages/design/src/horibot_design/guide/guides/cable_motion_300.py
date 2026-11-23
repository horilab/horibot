"""300 mm 선으로 돌려 보기: 서보로 선 돌려 보기(guides/cable_motion.py)를 실제 손목 구간에 쓸 300 mm 선으로 다시 한다
(status.md의 사용자 순서 1과 남은 질문 "서보 선이 관절을 넘는 방법").

20 cm 선 시험에서 고칠 것을 넣었다: J6 서보 받침은 새것으로 바꾸고(옛 받침은 너트를 넣다 부러졌다), 손목 1의 발에는
가짜 아래팔 끝(parts/fake_forearm.py: J4 원판과 아래팔 끝을 J4가 범위 가운데일 때의 자리로 합친 시험 부품)을 단다.
판지 대신 발을 막아 J5 → J6 선이 서보 먼 끝과 그 원판 사이 2.6 mm 틈을 지나고, 20 cm 선 시험에서 발 쪽으로 처진
남는 선의 고리가 실제 팔처럼 아래팔 끝에 닿는다. 세워서는 손목 1의 뒷덮개 밑에 책을 괴고(이 부품의 멈춤 귀가 뒷덮개
덮개 나사 머리보다 5.4 mm 낮다), 눕혀서 ①은 이 부품의 바깥 끝을 책상에 대고 세운다. 거꾸로와 눕혀서 ②는 사용자가 이
부품의 바깥 끝을 잡는다. 손목 1의 뒷덮개는 첫 관절에 단 것을 떼었다 다시 단다. 먼저 아무것도 달지 않고 자세 넷을 돌리고, 이어서 사용자가 생각한 방식으로
고무줄을 걸어 남는 선을 당겨 두고 다시 돌린다. 둘 다 녹화하고 Claude가 영상과 서보 기록을 분석한다.

선은 손목 1 속(꽂는 커넥터, 서보 받침 판 위, 서보 먼 끝 앞, 앞벽 가운데 홈)과 J6 쪽 플러그만 그린다. 손목 2 겉의 길은
사용자가 손으로 찾은 길이라 설계 코드에 없어서, 나가는 곳과 들어가는 곳을 이름표로 가리킨다.
"""

from __future__ import annotations

from dataclasses import replace

from horibot_design.guide.catalog import printed_items
from horibot_design.guide.guides import cable_motion as cm
from horibot_design.guide.guides import first_joint as fj
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
from horibot_design.parts import fake_forearm as ff
from horibot_design.parts import joint as j
from horibot_design.parts import wrist

LIMIT = 160.0  # 시험에서 돌리는 끝 [°]: 소프트 한계와 같다 (멈춤 ±163°)

CABLE_ITEM = Item(
    "servo_cable_300", "서보 선 300 mm", "bought", "Waveshare 6개 묶음에서 하나", icon="wire"
)
LONG_ITEM = Item(
    "servo_cable_900", "서보 선 900 mm", "bought", "재기만 하고 넣어 둔다", icon="wire"
)
TOOLS = (
    Item("caliper", "캘리퍼스", "tool", "선의 폭과 두께, 플러그", icon="caliper"),
    Item("ruler", "자나 줄자", "tool", "선의 끝에서 끝까지 길이", icon="ruler"),
    Item("scale", "저울", "tool", "선 하나의 무게", icon="scale"),
    Item("band", "고무줄", "tool", "직접 고른 것 (얇은 고무줄, 머리끈 같은 것)", icon="band"),
)

# 장면 부품 이름
FAKE = "가짜 아래팔 끝$"
FAKE_NUTS = "발 나사 너트"
FOOT_SCREWS = "발 나사 M3x12"
PLUG = "서보 선 플러그$"
INSIDE = "서보 선 (손목 1 속)"
J6_PLUG = "선 플러그 (J6)"
_NEW_J6 = ("J6 서보 받침", "J6 너트 (서보 받침)")  # 이번에 새로 드는 J6 쪽 부품


def scene() -> list[ScenePart]:
    # 20 cm 선 시험 때 조립한 그대로다. 새로 드는 것은 J6 서보 받침과 그 너트, 가짜 아래팔 끝과 너트, 발 나사,
    # 300 mm 선이다
    out: list[ScenePart] = []
    for p in cm.scene():
        if p.name.startswith("영점 핀"):
            continue
        out.append(replace(p, count=p.name.startswith(_NEW_J6)))
    for p in ff.placed(fj.MODEL):
        if p.name == "가짜 아래팔 끝":
            out.append(ScenePart(p.name, p.shape, "fake_forearm", p.color, False, True))
        elif p.name.startswith("발 나사 너트"):
            out.append(ScenePart(p.name, p.shape, "nut_m3", p.color, False, True))
        else:
            out.append(ScenePart(p.name, p.shape, "m3x12", p.color, False, True))
    plug, inside, _hang = fj.cable_parts()
    out.append(ScenePart("서보 선 플러그", plug, CABLE_ITEM.key, "plug", False, False))
    out.append(ScenePart(INSIDE, inside, CABLE_ITEM.key, "wire", False, False))
    out.append(ScenePart(J6_PLUG, wrist.as_child(plug), CABLE_ITEM.key, "plug", True, False))
    return out


CABLE = Use(CABLE_ITEM.key, note="손목 2 겉으로 감은 것", count=False)
J6_SET = (fj.CHILD, *cm.J6_SERVO, "J6 벽 나사", "J6 뒷덮개", "J6 덮개 나사")
FAKE_TOP = View((0, 1, 0), (1, 0, 0.35), 20, 0)  # 가짜 아래팔 끝의 원판 면 (발에 닿는 면)
FAKE_SIDE = View((0, 1, 0), (0.3, 0, 1), 15, 0)  # 옆면의 너트 홈
MOUNT_VIEW = View((0, 1, 0), (-0.35, 0, 1), 15, 0)  # 옆에서: 손목 1의 발과 가짜 아래팔 끝
# 가짜 아래팔 끝의 멈춤 귀는 손목 1의 뒷덮개 쪽(-y)을, 관은 앞벽 가운데 홈 쪽(+y)을 본다 (J4가 범위 가운데)
EAR_TAG = Tag((wrist.X_FOOT - 2.0, wrist.AX_Y - j.R_POST - j.EAR_HW - 2.0, 0.0), "귀 (뒷덮개 쪽)")
TUBE_TAG = Tag((-58.0, 70.0, 0.0), "관 (앞벽 가운데 홈 쪽)")
GRIP_TAG = Tag((-98.0, 20.0, 0.0), "잡는 곳 (손목 1에서 먼 끝)")
FACE_TAG = Tag((-98.0, 20.0, 0.0), "책상에 대는 면 (바깥 끝)")
BOOK_TAG = Tag((0.0, -30.0, 0.0), "뒷덮개 밑에 책")
EAR_FREE_TAG = Tag(
    (wrist.X_FOOT - 4.0, wrist.AX_Y - j.R_POST - j.EAR_HW - 3.0, 0.0), "귀 (책 밖, 책상 위로 뜬다)"
)
STAND_WARN = "돌리는 동안 관절이 기울거나 넘어지려 하면 바로 창에서 Enter를 누릅니다."
BAND_WARN = "고무줄이 풀리거나 미끄러지면 창에서 Enter를 눌러 멈추고 대화창에 알려 주세요."
HOLD_WARN = (
    "가짜 아래팔 끝의 바깥 끝만 잡습니다. 손가락이 관의 앞면(손목 2 쪽)이나 원판 둘레로 넘어오면 선이 손에 "
    "얹혀 실제 팔과 다른 시험이 됩니다."
)


def _pose(
    sid: str,
    short: str,
    title: str,
    setup: str,
    view: View,
    labels: tuple[Label, ...],
    tags: tuple[Tag, ...],
    held: bool,
    band: bool = False,
) -> Step:
    warn = (cm.STOP_NOW, cm.FINGERS, HOLD_WARN if held else STAND_WARN)
    if band:
        warn += (BAND_WARN,)
    return Step(
        sid,
        short,
        title,
        do=(
            setup,
            "휴대폰을 셀카봉이나 거치대로 그림을 보는 쪽에 고정하고, 관절과 선이 다 화면에 들어오게 녹화를 "
            "시작합니다.",
            f"Claude가 띄운 창에서 Enter를 누르면 J5 서보(ID {fj.SERVO_ID})가 끝에서 끝까지 천천히 세 번 "
            "오갑니다. 2분 남짓 걸립니다.",
            "창에 끝났다고 나오면 녹화를 멈춥니다. 서보는 가운데에서 버티고 있습니다.",
        ),
        warn=warn,
        check=(
            "세 번 오가는 것을 처음부터 끝까지 녹화했다.",
            "걸리거나 당겨진 곳이 있었으면 대화창에 적었다.",
        ),
        stuck="녹화를 놓쳤으면 대화창에 알려 주세요. 같은 자세로 다시 돌립니다.",
        uses=(CABLE, Use("band", note="건 그대로", count=False)) if band else (CABLE,),
        turn=(-LIMIT, LIMIT),
        view=view,
        zoom=1.2,
        labels=labels,
        tags=(fj.EXIT_TAG, *tags),
    )


def _poses(band: bool) -> tuple[Step, ...]:
    pre, head = ("band-", "고무줄: ") if band else ("", "")
    grip = Label(FAKE, "가짜 아래팔 끝 (바깥 끝을 잡기)")
    return (
        _pose(
            f"{pre}pose-up",
            "세워서",
            f"{head}세워서 돌리기",
            "책상에 두께 6 mm가 넘는 책을 놓고, 손목 2가 위로 오게 손목 1의 뒷덮개를 책 위에 세웁니다. 책 모서리는 "
            "손목 1의 발 쪽 끝(가짜 아래팔 끝이 붙은 면)에 맞추거나 1 cm 안쪽에 둡니다. 가짜 아래팔 끝의 귀는 책 "
            "밖에서 책상 위로 뜹니다.",
            cm.UP,
            (Label(fj.CHILD, "손목 2 (위)"),),
            (BOOK_TAG, EAR_FREE_TAG),
            held=False,
            band=band,
        ),
        _pose(
            f"{pre}pose-down",
            "거꾸로",
            f"{head}거꾸로 들고 돌리기",
            "가짜 아래팔 끝의 바깥 끝(손목 1에서 먼 쪽)을 한 손으로 잡고 뒤집어 듭니다. 손목 2가 아래로 오고, "
            "책상에 닿지 않게 띄웁니다.",
            cm.DOWN,
            (Label(fj.CHILD, "손목 2 (아래)"), grip),
            (GRIP_TAG,),
            held=True,
            band=band,
        ),
        _pose(
            f"{pre}pose-slot-low",
            "눕혀서 ①",
            f"{head}눕혀서 돌리기: 앞벽 가운데 홈이 아래",
            "가짜 아래팔 끝의 바깥 끝(손목 1에서 먼 쪽의 납작한 면)을 책상에 대고 세웁니다. 관절 축이 옆으로 "
            "눕고, 선이 나오는 앞벽 가운데 홈이 아래를 봅니다.",
            cm.SLOT_LOW,
            (Label(FAKE, "가짜 아래팔 끝 (바깥 끝을 책상에 댄다)"),),
            (FACE_TAG,),
            held=False,
            band=band,
        ),
        _pose(
            f"{pre}pose-slot-high",
            "눕혀서 ②",
            f"{head}눕혀서 돌리기: 앞벽 가운데 홈이 위",
            "가짜 아래팔 끝의 바깥 끝을 한 손으로 잡고, 앞의 자세에서 반 바퀴 돌려 앞벽 가운데 홈이 위를 보게 "
            "듭니다. 책상에 닿지 않게 띄웁니다.",
            cm.SLOT_HIGH,
            (grip,),
            (GRIP_TAG,),
            held=True,
            band=band,
        ),
    )


_OLD = {s.id: s for s in cm.STEPS}

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "20 cm 선 시험 때 조립한 첫 관절을 책상에 놓습니다. J6 서보(ID 6)가 손목 2에 든 그대로입니다.",
            "새로 뽑은 가짜 아래팔 끝 하나와 서보 받침 하나를 꺼냅니다. 같이 뽑은 받침 하나는 여분입니다.",
            "M3 너트 6개(가짜 아래팔 끝 넷, 받침 둘)와 M3 × 12 나사 4개를 꺼냅니다.",
            "300 mm 서보 선과 900 mm 서보 선을 하나씩 꺼냅니다. 900 mm 선은 재기만 합니다.",
            "캘리퍼스, 자나 줄자, 저울, 육각렌치, 작은 십자 드라이버, 절연 테이프를 꺼냅니다.",
            "어댑터와 서보 전원, 휴대폰을 고정할 셀카봉이나 거치대, 세워서 돌릴 때 괼 책 한 권(두께 6 mm 넘게), "
            "뒤에서 쓸 고무줄을 준비합니다.",
        ),
        check=(
            "20 cm 선 시험 때 조립한 첫 관절",
            "새로 뽑은 가짜 아래팔 끝 하나와 서보 받침 하나",
            "M3 너트 6개와 M3 × 12 나사 4개",
            "300 mm 선과 900 mm 선 하나씩",
            "캘리퍼스, 자나 줄자, 저울, 육각렌치, 작은 십자 드라이버, 절연 테이프",
            "어댑터, 서보 전원, 셀카봉이나 휴대폰 거치대, 책 한 권, 고무줄",
        ),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
        uses=(
            Use("tape", 2, "약 5 cm 두 조각 (남는 선 붙이기)"),
            Use("band", note="자세 넷을 돌린 뒤에 건다", count=False),
        ),
    ),
    Step(
        "measure",
        "선 재기",
        "300 mm 선과 900 mm 선 재기",
        do=(
            "캘리퍼스로 선 가운데쯤에서 세 가닥이 붙은 납작한 선의 폭과 두께를 잽니다.",
            "플러그 하나의 세 방향을 잽니다: 꽂히는 방향의 길이, 넓은 쪽 폭, 좁은 쪽 두께.",
            "자나 줄자로 한쪽 플러그 끝에서 반대쪽 플러그 끝까지 잽니다.",
            "저울로 선 하나의 무게를 잽니다.",
            "300 mm 선과 900 mm 선을 하나씩 재서 값을 대화창에 적어 주세요. Claude가 홈과 틈에 드는지 봅니다.",
        ),
        warn=(
            "폭이 6.0 mm를 넘거나 두께가 2.5 mm를 넘으면 조립하기 전에 알려 주세요. 앞벽 가운데 홈(폭 6.0 mm)과 서보 "
            "먼 끝 앞의 틈(2.6 mm)을 지나야 합니다.",
        ),
        check=("두 선의 폭, 두께, 플러그 세 방향, 끝에서 끝까지 길이, 무게를 대화창에 적었다.",),
        stuck="재는 자리가 헷갈리면 선과 캘리퍼스를 같이 찍어 보내 주세요.",
        uses=(
            Use(CABLE_ITEM.key, note="잰 뒤에 이 선을 쓴다", count=False),
            Use(LONG_ITEM.key, 1, "재기만 한다", count=False),
            Use("caliper"),
            Use("ruler"),
            Use("scale"),
        ),
        focus=(PLUG, INSIDE),
        view=View((0, -1, 0), (1, 0, 0.3), 35, 20),
        labels=(
            Label("서보 선 플러그", "플러그: 길이 · 폭 · 두께"),
            Label(INSIDE, "선: 폭 · 두께"),
        ),
    ),
    Step(
        "j6-out",
        "J6 빼기",
        "손목 2 덮개 떼고 J6 서보 빼기",
        do=(
            "손목 2의 뒷덮개 나사(M3 × 12) 2개를 육각렌치로 풀어 덮개를 뗍니다.",
            f"J6 서보(ID {cm.J6_ID})에 꽂힌 20 cm 시험 선을 뽑습니다.",
            "손목 2 벽의 뒷줄 나사(M3 × 8) 2개를 풉니다.",
            "J6 서보를 뒤쪽(열린 쪽)으로 당겨 뺍니다. 부러진 서보 받침이 붙은 채로 나옵니다.",
        ),
        warn=(
            "뗀 덮개와 나사 넷(M3 × 12 둘, M3 × 8 둘)은 다시 쓰니 모아 둡니다.",
            "손목 1 쪽의 선과 어댑터 선은 아직 그대로 둡니다.",
        ),
        check=("J6 서보를 손목 2에서 뺐다.", "덮개와 나사 넷을 모아 두었다."),
        stuck=fj.PHOTO,
        uses=(Use("hexkey"),),
        focus=J6_SET,
        view=cm.J6_VIEW,
        shell=(fj.CHILD,),
        ghost=True,
        labels=(
            Label("J6 뒷덮개", "뒷덮개 (나사 둘)"),
            Label("J6 벽 나사 2", "뒷줄 나사"),
            Label("J6 서보 몸통", f"J6 서보 (ID {cm.J6_ID})"),
        ),
    ),
    Step(
        "j6-old",
        "옛 받침",
        "부러진 서보 받침 떼기",
        do=(
            "J6 서보를 혼이 아래로 가게 엎어 놓습니다.",
            "서보 뒷면 나사 4개를 작은 십자 드라이버로 풀고, 부러진 받침을 뗍니다.",
        ),
        warn=("나사 넷은 새 받침에 다시 쓰니 모아 둡니다.",),
        check=("부러진 받침을 뗐다.", "서보 뒷면 나사 넷을 모아 두었다."),
        stuck=fj.PHOTO,
        uses=(Use("driver"),),
        focus=cm.J6_SERVO,
        view=_OLD["j6-carrier"].view,
        labels=(
            Label("J6 서보 받침", "부러진 받침"),
            Label("J6 서보 뒷면 나사 1", "서보 뒷면 나사 × 4"),
        ),
    ),
    _OLD["j6-nuts"],
    replace(
        _OLD["j6-carrier"],
        title="J6 서보에 새 받침 달기",
        do=(
            "J6 서보를 혼이 아래로 가게 엎어 둔 그대로 둡니다.",
            "새 받침을 날개가 아래(혼 쪽)로 가게 들고, 판을 서보 뒷면에 얹습니다. 판의 작은 구멍 넷이 서보 뒷면의 "
            "나사 구멍 넷과 맞아야 합니다. 거꾸로 놓으면 구멍이 맞지 않습니다.",
            "앞 단계에서 모아 둔 서보 뒷면 나사 4개를 작은 십자 드라이버로 조입니다.",
        ),
    ),
    replace(
        _OLD["j6-servo"],
        title="손목 2에 J6 서보 다시 넣기",
        do=(
            "위 몸통(손목 2)의 뒤쪽(열린 쪽)으로, 서보의 긴 쪽(혼에서 먼 끝)이 손목 2의 발 쪽을 보게 넣습니다.",
            "서보 받침 돋움의 구멍이 벽의 뒷줄 구멍으로 보이는 자리에서 손으로 잡아 둡니다.",
        ),
    ),
    replace(
        _OLD["j6-lock"],
        do=(
            "모아 둔 M3 × 8 나사 둘을 손목 2 벽의 뒷줄 구멍에 하나씩 넣고 육각렌치로 조입니다.",
            "머리가 벽에 닿아 더 돌지 않으면 멈춥니다.",
        ),
    ),
    Step(
        "j5-cable",
        "300 mm 선",
        "손목 1 덮개 떼고 300 mm 선 꽂기",
        do=(
            "손목 1을 뒤쪽(뒷덮개)이 위로 오게 듭니다. 지난 시험에서 발에 붙인 테이프나 판지는 뗍니다.",
            "뒷덮개 나사(M3 × 12) 2개를 풀어 덮개를 뗍니다.",
            f"J5 서보(ID {fj.SERVO_ID})에서 20 cm 시험 선을 뽑습니다. 옆 홈으로 나간 어댑터 선은 그대로 둡니다.",
            "시험 선을 뽑은 커넥터에 300 mm 선의 한쪽 플러그를 꽂습니다.",
            "선을 서보 받침 판 위에 눕혀 발 쪽으로 보내고, 서보 먼 끝 앞으로 내려 앞벽 가운데 홈으로 뺍니다. "
            "그림의 길입니다.",
            "뗀 뒷덮개를 테가 몸통 쪽으로 가게 얹고 나사 2개로 다시 조입니다.",
        ),
        warn=(
            "선이 덮개 테 밑에 끼이지 않게 봅니다.",
            "서보 먼 끝 앞에서 선을 비틀지 않고 납작하게 눕힙니다. 뒤에서 발을 가짜 아래팔 끝으로 막으면 이 틈이 "
            "2.6 mm입니다.",
        ),
        check=(
            "300 mm 선이 앞벽 가운데 홈으로, 어댑터 선이 옆 홈으로 나와 있다.",
            "뒷덮개가 들뜨지 않고 원통 뒤 끝을 덮었다.",
        ),
        stuck=fj.PHOTO,
        adds=(*cm.J5_COVER, PLUG, INSIDE),
        still=(PLUG, INSIDE),
        uses=(Use(CABLE_ITEM.key, 1, "앞 단계에서 잰 것"), Use("hexkey")),
        moves=(
            Move(("뒷덮개",), (0.0, -30.0, 0.0)),
            Move(("덮개 나사",), (0.0, -16.0, 0.0), 1.0),
        ),
        view=fj.BACK_UP,
        labels=(
            Label("뒷덮개", "뒷덮개 (떼었다 다시 단다)"),
            Label("서보 선 플러그", "300 mm 선을 꽂는 곳"),
        ),
        tags=(fj.BACK_TAG, fj.EXIT_TAG),
    ),
    Step(
        "fake-pierce",
        "막 뚫기",
        "가짜 아래팔 끝의 나사 구멍 막 뚫기",
        do=(
            "새로 뽑은 가짜 아래팔 끝을 둥근 돌기 넷이 있는 면(발에 대는 면)이 위로 오게 놓습니다.",
            "돌기 바깥쪽에 나사 구멍이 넷 있습니다. 구멍마다 육각렌치의 짧은 끝을 넣고 꾹 눌러, 구멍 속의 얇은 막을 "
            "뚫습니다.",
            "렌치 끝이 막 밑으로 쑥 들어가면 됩니다.",
        ),
        warn=(
            "막은 출력할 때 너트 홈의 천장을 받치려고 둔 한 층입니다. 뚫어도 부품에는 지장이 없습니다.",
        ),
        check=("나사 구멍 넷의 막을 모두 뚫었다.",),
        stuck=fj.PHOTO,
        adds=(FAKE,),
        still=(FAKE,),
        focus=(FAKE,),
        uses=(Use("hexkey"),),
        view=FAKE_TOP,
        labels=(Label(FAKE, "가짜 아래팔 끝 (발에 대는 면)"),),
        spots=(Spot(FAKE, "막을 뚫을 구멍", tuple(ff.hole_tops())),),
    ),
    Step(
        "fake-nuts",
        "너트",
        "가짜 아래팔 끝에 너트 넣기",
        do=(
            "발에 대는 면 바로 밑의 옆면에 납작한 홈이 양옆에 둘씩 있습니다.",
            "홈마다 M3 너트를 눕혀서 끝까지 밀어 넣습니다. 홈 끝의 육각 자리에 걸려 멈춥니다.",
            "위의 나사 구멍으로 들여다봐 너트 구멍이 보이면, 홈 입구에 테이프를 작게 한 조각씩 붙여 너트가 빠지지 "
            "않게 합니다.",
        ),
        warn=("너트가 안 들어가면 밀어붙이지 말고 사진을 보내 주세요.",),
        check=("너트 넷이 홈 끝까지 들어갔고, 홈 입구를 테이프로 막았다.",),
        stuck=fj.PHOTO,
        adds=(FAKE_NUTS,),
        focus=(FAKE, FAKE_NUTS),
        uses=(Use("tape", 4, "작은 조각 (홈 입구)"),),
        moves=tuple(
            Move((f"{FAKE_NUTS} {k}",), (0.0, 0.0, 12.0 if k <= 2 else -12.0), 0.45 * (k - 1))
            for k in range(1, 5)
        ),
        view=FAKE_SIDE,
        labels=(Label(f"{FAKE_NUTS} 1", "M3 너트 × 4 (눕혀서)"),),
    ),
    Step(
        "fake-mount",
        "발에 달기",
        "손목 1의 발에 가짜 아래팔 끝 달기",
        do=(
            "손목 1을 발 쪽(납작한 끝)이 앞으로 오게 듭니다. 선은 앞벽 가운데 홈으로 나와 있습니다.",
            "가짜 아래팔 끝의 돌기 넷이 있는 면을 발에 댑니다. 돌기는 발 속으로 들어갑니다. 관(긴 쪽)은 앞벽 가운데 "
            "홈 쪽을, 멈춤 귀는 뒷덮개 쪽을 봅니다. 실제 팔의 J4가 범위 가운데일 때의 방향입니다.",
            "발 양옆 귀의 구멍 넷이 나사 구멍과 맞으면, M3 × 12 나사를 귀 안쪽에서 하나씩 넣어 육각렌치로 조입니다.",
            "다 조이면 홈 입구의 테이프를 뗍니다.",
        ),
        warn=(
            "선이 가짜 아래팔 끝과 발 사이에 끼이지 않게 봅니다. 선은 앞벽 가운데 홈으로 나와 있어야 합니다.",
            "반 바퀴 돌려 달아도 나사 구멍이 맞습니다. 그러면 관이 뒷덮개 쪽으로 가니, 관이 선이 나오는 홈 쪽인지 "
            "꼭 봅니다.",
        ),
        check=(
            "가짜 아래팔 끝이 발에 붙어 흔들리지 않는다.",
            "관이 앞벽 가운데 홈 쪽, 멈춤 귀가 뒷덮개 쪽을 본다.",
            "선이 끼이지 않고 홈으로 나와 있다.",
        ),
        stuck=fj.PHOTO,
        adds=(FOOT_SCREWS,),
        mount=(FAKE, FAKE_NUTS, FOOT_SCREWS),
        uses=(Use("hexkey"), Use("tape", note="떼어 낸다", count=False)),
        moves=(
            Move((FAKE, FAKE_NUTS), (-25.0, 0.0, 0.0)),
            Move((FOOT_SCREWS,), (14.0, 0.0, 0.0), 1.0),
        ),
        view=MOUNT_VIEW,
        zoom=1.1,
        labels=(
            Label(FAKE, "가짜 아래팔 끝"),
            Label(f"{FOOT_SCREWS} 1", "M3 × 12 × 4"),
        ),
        tags=(EAR_TAG, TUBE_TAG, fj.EXIT_TAG),
    ),
    Step(
        "lay",
        "선 놓기",
        "손목 2 겉으로 선 놓고 길이 맞추기",
        do=(
            "20 cm 선 때와 같은 길로 놓습니다: 앞벽 가운데 홈에서 나와 손목 2의 겉을 돌아, 손목 2의 뒤쪽 모서리로 "
            f"넣어 J6 서보(ID {cm.J6_ID})의 커넥터 하나에 꽂습니다.",
            "서보는 아직 전원에 잇지 않습니다. 위 몸통을 손으로 천천히 한쪽 끝까지, 가운데를 지나 반대쪽 끝까지 "
            "돌려 봅니다.",
            "선이 가장 길게 필요한 끝에서 살짝 처질 만큼만 두고, 남는 선은 접어서 손목 2 겉에 테이프로 붙입니다.",
            "어느 자리에서도 선이 팽팽해지지 않고 플러그가 당겨지지 않는지 봅니다.",
            "다 놓으면 양 끝과 가운데에서 사진을 한 장씩 보내 주세요.",
        ),
        warn=(
            "테이프로 붙인 남는 선을 관절 둘레에 두지 않습니다. 고리가 커져서 실제 팔과 다른 시험이 됩니다.",
            "손으로 돌릴 때는 천천히 돌립니다. 끝에서 멈춤 기둥이 살에 닿으면 더 밀지 않습니다.",
        ),
        check=(
            "양 끝과 가운데 어디에서도 선이 팽팽하지 않다.",
            "선을 놓은 사진 세 장을 보냈다.",
        ),
        stuck=fj.PHOTO,
        adds=(J6_PLUG,),
        uses=(CABLE, Use("tape", note="남는 선 붙이기", count=False)),
        turn=(-j.STOP_DEG, j.STOP_DEG),
        view=cm.UP,
        zoom=1.2,
        labels=(Label(fj.CHILD, "손목 2 (겉으로 감는다)"),),
        tags=(fj.EXIT_TAG, cm.ENTRY_TAG),
    ),
    replace(
        _OLD["j6-cover"],
        do=(
            "떼어 둔 뒷덮개를 테가 몸통 쪽으로 가게 들고 손목 2의 뒤쪽에 얹습니다.",
            "모아 둔 M3 × 12 나사 2개를 넣고 육각렌치로 조입니다.",
        ),
        labels=(
            Label("J6 뒷덮개", "뒷덮개 (떼어 둔 것)"),
            Label("J6 덮개 나사 M3x12 1", "M3 × 12 × 2"),
        ),
    ),
    _OLD["connect"],
    *_poses(band=False),
    replace(
        _OLD["fast"],
        uses=(CABLE,),
        hide=(),
        turn=(-LIMIT, LIMIT),
        do=(
            "앞의 네 자세 가운데 선이 가장 크게 움직였거나 가장 걸릴 뻔했던 자세를 대화창에 알려 주세요. Claude가 "
            "실제로 쓸 빠르기의 창을 띄웁니다.",
            "그 자세로 세우거나 들고, 녹화를 시작한 뒤 창에서 Enter를 누릅니다. 끝에서 끝까지 세 번 빠르게 "
            "오갑니다.",
            "끝나면 녹화를 멈춥니다. 마지막에 창이 시키면 위 몸통을 받치고 Enter를 누릅니다. 서보 힘이 꺼집니다. "
            "다음 단계에서 고무줄을 걸며 손으로 돌려 봅니다.",
        ),
    ),
    Step(
        "band",
        "고무줄",
        "고무줄 걸기",
        do=(
            "생각하신 방식으로 고무줄을 걸어 남는 선을 당겨 둡니다.",
            "돌리기 전에 사진을 찍어 보내 주세요: 고무줄 양 끝을 건 자리 하나씩, 고무줄 자체 하나(캘리퍼스나 자 "
            "옆에 두고).",
            "서보 힘이 꺼진 채로 위 몸통을 손으로 천천히 끝에서 끝까지 돌려, 고무줄이나 선이 걸리는 데가 없는지 "
            "봅니다.",
        ),
        warn=(
            "고무줄은 세게 당기지 않습니다. 세게 당기면 커넥터가 늘 당겨집니다.",
            "고무줄이 멈춤 귀나 두 몸통 사이 틈, 손목 2의 튀어나온 곳에 닿지 않게 겁니다.",
            "가짜 아래팔 끝에는 걸지 않습니다. 실제 팔에서는 J4가 돌며 움직이는 부분입니다.",
            "고무줄 자리를 옮기면 사진을 다시 찍고 대화창에 알려 주세요.",
        ),
        check=(
            "고무줄 사진 셋을 보냈다.",
            "손으로 끝에서 끝까지 돌려 걸리는 데가 없었다.",
        ),
        stuck="어떻게 걸지 헷갈리면 걸어 본 사진을 보내 주세요. 같이 봅니다.",
        uses=(Use("band", 1, "직접 고른 것"), CABLE),
        turn=(-j.STOP_DEG, j.STOP_DEG),
        view=cm.UP,
        zoom=1.2,
        labels=(Label(fj.CHILD, "손목 2"), Label(fj.HOUSE, "손목 1")),
        tags=(fj.EXIT_TAG, cm.ENTRY_TAG),
    ),
    *_poses(band=True),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "300 mm 선으로 돌려 보기가 끝났습니다. 서보 전원을 끕니다.",
            "영상과 고무줄 사진을 PC의 폴더 하나에 옮기고, 그 폴더 경로를 대화창에 적어 주세요. Claude가 영상과 "
            "서보 기록을 맞춰 분석합니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요.",
        ),
        view=cm.UP,
        zoom=1.2,
    ),
)

GUIDE = Guide(
    id="cable-motion-300",
    kind="measure",
    title="300 mm 선으로 돌려 보기",
    sub=f"J5 · ID {fj.SERVO_ID} 서보 · 300 mm 선 · 자세 넷, 이어서 고무줄",
    scene=scene,
    steps=STEPS,
    items={
        **cm.GUIDE.items,
        **printed_items(ff.LABELS, ff.PRINT_SPEC),
        CABLE_ITEM.key: CABLE_ITEM,
        LONG_ITEM.key: LONG_ITEM,
        **{it.key: it for it in TOOLS},
    },
)
