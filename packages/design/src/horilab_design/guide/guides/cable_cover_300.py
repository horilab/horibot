"""새 뒷덮개로 다시 돌려 보기: 300 mm 선 시험(guides/cable_motion_300.py)에서 손목 2 뒷덮개 밑으로 나온 선이 영점 핀
귀 둘레를 긁어서 고친 길로 다시 돌린다 (0031).

손목 2는 새 뒷덮개(wrist2_cover: 선이 영점 핀 귀보다 위의 창으로 나온다)로 바꾸고, 300 mm 선은 J6 서보의 영점 핀 귀
반대쪽(-z) 커넥터에 꽂는다. 손목 1에서는 J5 → J6 선을 앞벽 가운데 홈 대신 영점 핀 귀 쪽(+z) 옆 홈으로 곧게 낸다
(커넥터는 그대로 +z 쪽, 어댑터 선은 -z 쪽 옆 홈). 자세는 지난번에 돌린 셋(세워서, 거꾸로, 눕혀서 ① 빠르게)이다.
"""

from __future__ import annotations

from dataclasses import replace

from horilab_design.guide.catalog import printed_items
from horilab_design.guide.guides import cable_motion as cm
from horilab_design.guide.guides import cable_motion_300 as c3
from horilab_design.guide.guides import first_joint as fj
from horilab_design.guide.model import Guide, Label, Move, ScenePart, Step, Tag, Use
from horilab_design.parts import joint as j
from horilab_design.parts import wrist
from horilab_design.parts.shapes import box, solid

OLD_COVER = ("J6 뒷덮개", "J6 덮개 나사")
NEW_COVER = "손목 2 새 뒷덮개"
W1_WIRE = "서보 선 (손목 1 속, 옆 홈으로)"
W2_WIRE = "서보 선 (손목 2 속)"
J6_PLUG = "선 플러그 (J6, 귀 반대쪽)"
# 손목 1에서 떼었다 다시 다는 것
W1_OPEN = ("뒷덮개", "덮개 나사", c3.FAKE, c3.FAKE_NUTS, c3.FOOT_SCREWS)
# 300 mm 선 시험 때 이미 조립해 둔 것 (첫 단계에서 본체에 붙은 것으로 둔다)
BASE = (
    *(prefix for prefix, _key, _count in fj.PART_KEYS if prefix != "영점 핀"),
    "J6 ",
    c3.FAKE,
    c3.FAKE_NUTS,
    c3.FOOT_SCREWS,
    c3.PLUG,
)

SIDE_TAG = Tag(
    (wrist.X_FOOT + 1.2, j.FRONT + 3.0, wrist.SIDE_LANE),
    "옆 홈 (영점 핀 귀 쪽): 300 mm 선이 나간다",
)
ADAPTER_TAG = Tag(
    (wrist.X_FOOT + 1.2, j.FRONT + 3.0, -wrist.SIDE_LANE), "옆 홈 (반대쪽): 어댑터 선"
)
# 새 덮개의 창 (관절 각 0의 장면 좌표. 자식 (x, y, z) → (y - AX_Y, x + ATOP - X_FOOT, -z))
WINDOW = (
    j.BACK_OUT - 3.0 - wrist.AX_Y,
    j.ATOP + wrist.EXIT_H + wrist.EXIT_GAP / 2,
    -wrist.EXIT_PLUG_Z,
)
WINDOW_TAG = Tag(WINDOW, "새 덮개의 창: 선이 나오는 곳")


def _w2(name: str) -> ScenePart:
    """손목 2 속 부품(wrist.wrist2_inner)을 장면 좌표로 옮긴 것."""
    p = next(p for p in wrist.wrist2_inner(fj.MODEL) if p.name == name)
    if name == "뒷덮개":
        return ScenePart(NEW_COVER, wrist.as_child(p.shape), "wrist2_cover", p.color, True, True)
    return ScenePart(W2_WIRE, wrist.as_child(p.shape), c3.CABLE_ITEM.key, "wire", True, False)


def scene() -> list[ScenePart]:
    # 300 mm 선 시험 때 조립한 그대로다. 새로 드는 것은 손목 2의 새 뒷덮개뿐이다. 선은 고친 길로 그린다
    out = [replace(p, count=False) for p in c3.scene() if p.name not in (c3.INSIDE, c3.J6_PLUG)]
    _plug, inside, _hang = fj.cable_parts(wrist.SIDE_LANE)
    out.append(ScenePart(W1_WIRE, inside, c3.CABLE_ITEM.key, "wire", False, False))
    minus = box(-16.6, -11.85, j.PLUG_TOP + fj.DY, j.L1 + 0.5 + fj.DY, -9.46, -0.45)
    out.append(
        ScenePart(J6_PLUG, wrist.as_child(solid(minus)), c3.CABLE_ITEM.key, "plug", True, False)
    )
    out.append(_w2("선 (J5에서)"))
    out.append(_w2("뒷덮개"))
    return out


_CM = {s.id: s for s in cm.STEPS}
_C3 = {s.id: s for s in c3.STEPS}
GONE = ("J6 뒷덮개",)  # 뗀 옛 덮개는 다시 보이지 않는다


def _after(step: Step) -> Step:
    """옛 300 mm 선 시험의 단계를 이 안내에 쓴다: 옛 덮개를 감추고, 가운데 홈 이름표를 옆 홈으로 바꾼다."""
    tags = tuple(SIDE_TAG if t is fj.EXIT_TAG else t for t in step.tags)
    return replace(step, hide=GONE, tags=tags)


STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "300 mm 선 시험 때 조립한 그대로의 첫 관절을 책상에 놓습니다 (가짜 아래팔 끝과 300 mm 선이 달린 것).",
            "새로 뽑은 손목 2 뒷덮개(wrist2_cover.stl, 검정)를 꺼냅니다.",
            "육각렌치, 절연 테이프, 어댑터와 서보 전원, 휴대폰 거치대, 세워서 돌릴 때 괼 책 한 권을 준비합니다.",
        ),
        check=(
            "300 mm 선 시험 때의 첫 관절",
            "새로 뽑은 손목 2 뒷덮개",
            "육각렌치, 절연 테이프",
            "어댑터, 서보 전원, 휴대폰 거치대, 책 한 권",
        ),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
        uses=(Use("hexkey"), Use("tape", note="너트 홈 막기, 남는 선 붙이기", count=False)),
    ),
    Step(
        "w2-open",
        "덮개 떼기",
        "손목 2의 옛 뒷덮개 떼기",
        do=(
            "손목 2의 뒷덮개 나사(M3 × 12) 2개를 육각렌치로 풀어 덮개를 뗍니다. 뗀 덮개는 더 쓰지 않고, 나사 둘은 "
            "새 덮개에 다시 씁니다.",
            f"J6 서보(ID {cm.J6_ID})를 잡은 서보 받침이 새것인지 봅니다. 부러진 옛 받침이면 대화창에 알려 주세요.",
        ),
        check=("옛 덮개를 뗐고 나사 둘을 모아 두었다.", f"J6 서보(ID {cm.J6_ID}) 받침이 새것이다."),
        stuck=fj.PHOTO,
        adds=BASE,
        still=BASE,
        mount=BASE,
        uses=(Use("hexkey"),),
        focus=c3.J6_SET,
        shell=(fj.CHILD,),
        ghost=True,
        view=cm.J6_VIEW,
        labels=(
            Label("J6 뒷덮개", "옛 뒷덮개 (뗀다)"),
            Label("J6 덮개 나사 M3x12 1", "M3 × 12 × 2 (다시 쓴다)"),
            Label("J6 서보 받침", "서보 받침 (새것인지 본다)"),
        ),
    ),
    Step(
        "j6-plug",
        "J6 커넥터",
        "300 mm 선을 J6 서보의 귀 반대쪽 커넥터에 꽂기",
        do=(
            f"J6 서보(ID {cm.J6_ID}) 뒷면에 커넥터가 둘 있습니다. 300 mm 선이 그림의 커넥터에 꽂혀 있는지 봅니다. "
            "손목 2 발의 귀 둘 가운데 영점 핀 구멍이 있는 귀의 반대쪽입니다.",
            "다른 커넥터에 꽂혀 있으면 뽑아서 그림의 커넥터로 옮겨 꽂습니다. 다른 커넥터는 비워 둡니다 (그리퍼 선 "
            "자리).",
        ),
        check=("300 mm 선이 영점 핀 귀 반대쪽 커넥터에 꽂혀 있다.",),
        stuck=fj.PHOTO,
        adds=(J6_PLUG,),
        mount=(J6_PLUG,),
        uses=(c3.CABLE,),
        focus=(*c3.J6_SET, J6_PLUG),
        shell=(fj.CHILD,),
        ghost=True,
        hide=OLD_COVER,
        view=cm.J6_VIEW,
        labels=(Label(J6_PLUG, "300 mm 선을 꽂는 커넥터 (귀 반대쪽)"),),
    ),
    Step(
        "w2-cover",
        "새 덮개",
        "선을 새 덮개의 창으로 빼고 덮개 달기",
        do=(
            "300 mm 선을 J6 서보 뒷면(서보 받침 판 위)에 납작하게 눕혀 발 쪽으로 보냅니다.",
            "새 덮개를 테가 몸통 쪽으로 가게 들고, 덮개 안쪽의 턱에서 선을 바깥으로 꺾어 창으로 뺍니다. 창은 덮개 옆 "
            "가장자리까지 트여 있어서, 선을 눕힌 채 옆에서 밀어 넣으면 됩니다. 플러그는 뽑지 않습니다.",
            "덮개를 손목 2의 뒤쪽에 얹고, 모아 둔 M3 × 12 나사 2개로 조입니다.",
            "선이 창에서 바깥쪽으로 곧게 나오는지 봅니다.",
        ),
        warn=(
            "선이 덮개 테 밑에 끼이지 않게 봅니다. 덮개 밑으로 발까지 내려가는 길은 턱이 막습니다.",
            "선을 비틀지 않고 납작하게 눕힙니다.",
        ),
        check=("선이 새 덮개의 창으로 나와 있다.", "덮개가 들뜨지 않고 원통 뒤 끝을 덮었다."),
        stuck=fj.PHOTO,
        adds=(W2_WIRE, NEW_COVER),
        still=(W2_WIRE,),
        uses=(Use("hexkey"),),
        moves=(
            Move((NEW_COVER,), (-30.0, 0.0, 0.0)),
            Move(("J6 덮개 나사",), (-16.0, 0.0, 0.0), 1.0),
        ),
        hide=GONE,
        view=cm.J6_VIEW,
        zoom=1.3,
        center=WINDOW,
        labels=(Label(NEW_COVER, "새 뒷덮개 (손목 2)"), Label(W2_WIRE, "300 mm 선")),
        tags=(WINDOW_TAG,),
    ),
    Step(
        "w1-open",
        "손목 1 열기",
        "손목 1의 뒷덮개와 가짜 아래팔 끝 떼기",
        do=(
            "가짜 아래팔 끝의 옆면 홈 넷에 든 너트가 빠지지 않게, 홈 입구에 테이프를 작게 한 조각씩 붙입니다.",
            "손목 1 발 양옆 귀의 M3 × 12 나사 4개를 풀어 가짜 아래팔 끝을 뗍니다.",
            "손목 1의 뒷덮개 나사(M3 × 12) 2개를 풀어 덮개를 뗍니다.",
        ),
        warn=("뗀 덮개, 가짜 아래팔 끝, 나사 여섯은 다시 쓰니 모아 둡니다.",),
        check=("가짜 아래팔 끝과 손목 1의 뒷덮개를 뗐다.", "나사 여섯을 모아 두었다."),
        stuck=fj.PHOTO,
        uses=(Use("hexkey"), Use("tape", 4, "작은 조각 (너트 홈 입구)")),
        hide=GONE,
        view=c3.MOUNT_VIEW,
        labels=(
            Label(c3.FAKE, "가짜 아래팔 끝 (나사 넷)"),
            Label("뒷덮개", "손목 1 뒷덮개 (나사 둘)"),
        ),
    ),
    Step(
        "w1-cable",
        "옆 홈으로",
        "손목 1에서 300 mm 선을 옆 홈으로 옮기기",
        do=(
            f"J5 서보(ID {fj.SERVO_ID})에 꽂힌 300 mm 선은 그대로 두고, 선을 앞벽 가운데 홈에서 빼냅니다.",
            "선을 서보 받침 판 위에서 곧게 펴 발 쪽으로 보내고, 서보 먼 끝 앞으로 내려 그림의 옆 홈으로 뺍니다. 손목 "
            "1 발의 귀 둘 가운데 영점 핀 구멍이 있는 귀 쪽입니다.",
            "어댑터 선은 반대쪽 옆 홈으로 나와 있어야 합니다. 두 선이 서보 먼 끝 앞에서 겹치지 않게 봅니다.",
        ),
        warn=("서보 먼 끝 앞의 틈은 2.6 mm라, 선 둘이 겹치면 가짜 아래팔 끝을 달 때 눌립니다.",),
        check=(
            "300 mm 선이 영점 핀 귀 쪽 옆 홈으로, 어댑터 선이 반대쪽 옆 홈으로 나와 있다.",
            "두 선이 겹치지 않았다.",
        ),
        stuck=fj.PHOTO,
        adds=(W1_WIRE,),
        uses=(c3.CABLE,),
        hide=(*GONE, *W1_OPEN),
        view=fj.BACK_UP,
        labels=(Label(W1_WIRE, "300 mm 선 (곧게 옆 홈으로)"),),
        tags=(SIDE_TAG, ADAPTER_TAG),
    ),
    Step(
        "w1-close",
        "다시 달기",
        "손목 1의 뒷덮개와 가짜 아래팔 끝 다시 달기",
        do=(
            "손목 1의 뒷덮개를 테가 몸통 쪽으로 가게 얹고 나사 2개로 조입니다.",
            "가짜 아래팔 끝을 전과 같은 방향으로 발에 댑니다: 관(긴 쪽)은 앞벽 쪽, 멈춤 귀는 뒷덮개 쪽입니다.",
            "나사 4개를 귀 안쪽에서 넣어 조이고, 홈 입구의 테이프를 뗍니다.",
        ),
        warn=("선이 덮개 테 밑이나 가짜 아래팔 끝과 발 사이에 끼이지 않게 봅니다.",),
        check=(
            "덮개와 가짜 아래팔 끝이 흔들리지 않게 붙었다.",
            "300 mm 선과 어댑터 선이 끼이지 않고 옆 홈으로 나와 있다.",
        ),
        stuck=fj.PHOTO,
        uses=(Use("hexkey"), Use("tape", note="떼어 낸다", count=False)),
        moves=(
            Move(("뒷덮개",), (0.0, -30.0, 0.0)),
            Move(("덮개 나사",), (0.0, -16.0, 0.0), 1.0),
            Move((c3.FAKE, c3.FAKE_NUTS), (-25.0, 0.0, 0.0), 1.6),
            Move((c3.FOOT_SCREWS,), (14.0, 0.0, 0.0), 2.6),
        ),
        hide=GONE,
        view=c3.MOUNT_VIEW,
        zoom=1.1,
        labels=(Label(c3.FAKE, "가짜 아래팔 끝 (관은 앞벽 쪽)"),),
        tags=(SIDE_TAG, c3.TUBE_TAG),
    ),
    Step(
        "lay",
        "선 놓기",
        "손목 2 겉으로 선 놓고 길이 맞추기",
        do=(
            "옆 홈에서 나온 선을 손목 2의 겉으로 돌려 새 덮개의 창까지 잇습니다. 남는 길이는 고리로 둡니다.",
            "서보는 아직 전원에 잇지 않습니다. 위 몸통을 손으로 천천히 한쪽 끝까지, 가운데를 지나 반대쪽 끝까지 "
            "돌려 봅니다.",
            "선이 가장 길게 필요한 끝에서 살짝 처질 만큼만 두고, 남는 선은 접어서 손목 2 겉에 테이프로 붙입니다.",
            "어느 자리에서도 선이 팽팽해지지 않고, 영점 핀 귀(핀 구멍이 있는 귀)에 닿지 않는지 봅니다.",
            "다 놓으면 양 끝과 가운데에서 사진을 한 장씩 보내 주세요.",
        ),
        warn=(
            "테이프로 붙인 남는 선을 관절 둘레에 두지 않습니다. 고리가 커져서 실제 팔과 다른 시험이 됩니다.",
            "손으로 돌릴 때는 천천히 돌립니다. 끝에서 멈춤 기둥이 살에 닿으면 더 밀지 않습니다.",
        ),
        check=("양 끝과 가운데 어디에서도 선이 팽팽하지 않다.", "선을 놓은 사진 세 장을 보냈다."),
        stuck=fj.PHOTO,
        uses=(c3.CABLE, Use("tape", note="남는 선 붙이기", count=False)),
        hide=GONE,
        turn=(-j.STOP_DEG, j.STOP_DEG),
        view=cm.UP,
        zoom=1.2,
        labels=(Label(fj.CHILD, "손목 2 (겉으로 감는다)"),),
        tags=(SIDE_TAG, WINDOW_TAG),
    ),
    replace(_CM["connect"], hide=GONE),
    _after(_C3["pose-up"]),
    _after(_C3["pose-down"]),
    Step(
        "fast",
        "눕혀서 빠르게",
        "눕혀서 빠르게 돌리기: 앞벽 가운데 홈이 아래",
        do=(
            "가짜 아래팔 끝의 바깥 끝(납작한 면)을 책상에 대고 세웁니다. 관절 축이 옆으로 눕고, 앞벽 가운데 홈이 "
            "아래를 봅니다. 지난번에 빠르게 돌린 자세와 같습니다.",
            "휴대폰을 그림을 보는 쪽에 고정하고, 관절과 선이 다 화면에 들어오게 녹화를 시작합니다.",
            f"Claude가 띄운 창에서 Enter를 누르면 J5 서보(ID {fj.SERVO_ID})가 끝에서 끝까지 빠르게 세 번 "
            "오갑니다.",
            "창에 끝났다고 나오면 녹화를 멈춥니다. 서보는 가운데에서 버티고 있습니다.",
        ),
        warn=(cm.STOP_NOW, cm.FINGERS, c3.STAND_WARN),
        check=(
            "세 번 오가는 것을 처음부터 끝까지 녹화했다.",
            "걸리거나 당겨진 곳이 있었으면 대화창에 적었다.",
        ),
        stuck="녹화를 놓쳤으면 대화창에 알려 주세요. 같은 자세로 다시 돌립니다.",
        uses=(c3.CABLE,),
        hide=GONE,
        turn=(-c3.LIMIT, c3.LIMIT),
        view=cm.SLOT_LOW,
        zoom=1.2,
        labels=(Label(c3.FAKE, "가짜 아래팔 끝 (바깥 끝을 책상에 댄다)"),),
        tags=(c3.FACE_TAG, SIDE_TAG),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "새 뒷덮개로 다시 돌려 보기가 끝났습니다. 서보 전원을 끕니다.",
            "영상을 PC의 temp 폴더에 옮기고 대화창에 알려 주세요. Claude가 영상과 서보 기록을 맞춰 분석합니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요.",
        ),
        hide=GONE,
        view=cm.UP,
        zoom=1.2,
    ),
)

GUIDE = Guide(
    id="cable-cover-300",
    kind="measure",
    title="새 뒷덮개로 다시 돌려 보기",
    sub=f"J5 · ID {fj.SERVO_ID} 서보 · 손목 2 새 뒷덮개 · 선은 옆 홈으로 · 자세 셋",
    scene=scene,
    steps=STEPS,
    items={**c3.GUIDE.items, **printed_items(wrist.LABELS, wrist.PRINT_SPEC)},
)
