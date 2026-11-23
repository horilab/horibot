"""서보 선 감아 돌려 보기: 첫 관절(J5, ID 5 서보)의 20 cm 서보 선을 출력 원판 테에 감아 양 끝 멈춤까지 돌려 본다
(0031의 "첫 관절을 조립할 때 볼 것", 0028의 시험과 측정).

서보 선만 감는다 (D405 선은 관절 밖으로 넘는다). 손목 2(자식)에는 아직 꽂을 서보가 없어서, 선 끝을 뒷덮개 쪽
모서리로 넣고 그 바로 위에서 테이프로 붙여 플러그에 꽂힌 것처럼 둔다. 손목 1 쪽(홈)은 붙이지 않는다: 선을 따로
잡지 않아도 홈에서 밀려 들어가거나 커넥터를 당기지 않는지가 이 시험에서 볼 것이다 (0031). 선 길이는 가장 길게
감기는 멈춤(-163°)에서 선을 테에 붙여 맞춘다 (0031: 선은 가장 길 때에 맞춘다). 층 안의 모양은 어림이다
(guide/cables.py).
"""

from __future__ import annotations

from dataclasses import replace

from build123d import Compound

from horibot_design.guide import cables as cb
from horibot_design.guide.guides import first_joint as fj
from horibot_design.guide.model import Guide, Label, ScenePart, Spot, Step, Use, View
from horibot_design.parts import joint as j
from horibot_design.parts import wrist
from horibot_design.parts.shapes import box, solid
from horibot_design.studies import wiring as w

ROD = cb.servo_rod()
TAIL = cb.tail_mm(ROD)  # 선 끝(플러그 뒤)이 손목 2의 모서리를 지나 들어가는 길이
EX, EZ = ROD.end  # 손목 2로 들어가는 자리 (관절 각 0)
LONG = cb.LONG_STOP
_W, _T = j.WIRE_W, j.WIRE_T

HANG = "서보 선 (늘어뜨린 끝)"
LAYER = "서보 선 (층)"
TAIL_PART = "서보 선 (손목 2 속)"
END_PLUG = "서보 선 끝 플러그"
TAPE = "테이프"
MARK = "홈 앞 네임펜 줄"


def _tail() -> tuple[Compound, Compound]:
    """손목 2 속의 선과 끝 플러그 (관절 각 0의 자리, 자식과 같이 돈다): 모서리에서 층을 떠나 손목 2의 열린 뒤쪽 안을
    곧게 올라간다. 선은 뒷덮개 자리 바로 안쪽에 눕는다 (studies/wiring.py의 wrist_in)."""
    y0 = cb.LAYER_Y - _W / 2
    top = cb.LAYER_Y + TAIL
    wire = box(EX - _T / 2, EX + _T / 2, y0, top, EZ - _W / 2, EZ + _W / 2)
    plug_len, plug_w, plug_t = w.PLUG
    zc = j.NECK_IN - 0.3 - plug_w / 2  # 넓은 플러그는 목 안쪽 벽 안으로 비켜 둔다
    plug = box(
        EX - plug_t / 2, EX + plug_t / 2, top, top + plug_len, zc - plug_w / 2, zc + plug_w / 2
    )
    return solid(wire), solid(plug)


def _tape() -> Compound:
    """모서리 바로 위에서 선을 가로질러 손목 2의 뒤쪽 가장자리(뒷덮개 자리)에 붙인 테이프 (관절 각 0의 자리)."""
    back = j.BACK_IN - wrist.AX_Y  # 손목 2의 뒷덮개 자리 (관절 각 0에서 부모의 x)
    y = cb.LAYER_Y + 4.0
    return solid(box(back - 0.3, back, y, y + 12.0, EZ - _W / 2 - 1.0, j.NECK_HW))


def _mark() -> Compound:
    """홈 가장자리에 맞춰 살 위의 선을 가로질러 그은 네임펜 줄."""
    x = wrist.X_FOOT + 2.4  # 가운데 홈의 안쪽 가장자리
    top = cb.FLAT_Y + _T / 2
    return solid(box(x, x + 0.8, top, top + 0.3, -_W / 2 - 0.1, _W / 2 + 0.1))


def scene() -> list[ScenePart]:
    # 조립한 첫 관절을 그대로 쓴다 (새로 드는 부품이 아니다). 영점 핀은 맞춰 본 뒤 뺐다
    out = [
        replace(p, count=False) for p in fj.scene() if not p.name.startswith(("서보 선", "영점 핀"))
    ]
    plug, inside, hang = fj.cable_parts()
    fx = cb.flex(ROD)
    xs = [x for fr in fx.frames for x, _ in fr]
    zs = [z for fr in fx.frames for _, z in fr]
    span = box(min(xs), max(xs), cb.FLAT_Y - _T, cb.LAYER_Y + _W / 2, min(zs), max(zs))
    tail, end_plug = _tail()
    return [
        *out,
        ScenePart("서보 선 플러그", plug, "servo_cable", "plug", False, False),
        ScenePart("서보 선 (손목 1 속)", inside, "servo_cable", "wire", False, False),
        ScenePart(HANG, hang, "servo_cable", "wire", False, False),
        ScenePart(LAYER, solid(span), "servo_cable", "wire", False, False, flex=fx),
        ScenePart(TAIL_PART, tail, "servo_cable", "wire", True, False),
        ScenePart(END_PLUG, end_plug, "servo_cable", "plug", True, False),
        ScenePart(TAPE, _tape(), "tape", "cap", True, False),
        ScenePart(MARK, _mark(), "pen", "cap", False, False),
    ]


# 손에 드는 방향 (출력축 y가 위: 손목 1을 책상에 두고 손목 2가 위에 있다)
ABOVE = View((0, 1, 0), (0, 0, 1), 58, 0)  # 앞(+z)에서 내려다봄: 살(홈)이 왼쪽
ENTRY = View((0, 1, 0), (1, 0, 0.35), 28, 0)  # -163°에서 손목 2의 열린 뒤쪽
GROOVE = View((0, 1, 0), (-1, 0, 0.45), 42, 0)  # 살 쪽
TOP = View((0, 1, 0), (0, 0, 1), 72, 0)

# -163°에서 테에 감긴 선의 가운데 (방위각 90°)
_RIM_AT = (0.0, cb.LAYER_Y + _W / 2 + 0.6, ROD.radius)
_ENTRY_AT = (EX, cb.LAYER_Y + 3.0, EZ)
_MID = (0.0, cb.LAYER_Y, 0.0)  # 출력 원판 가운데 (층 높이)

# 조립한 첫 관절의 부품 (이름 앞부분)
ASSEMBLY = tuple(prefix for prefix, _key, _count in fj.PART_KEYS if prefix != "영점 핀")
STILL = (*ASSEMBLY, "서보 선 플러그", "서보 선 (손목 1 속)", HANG)

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            f'"첫 관절 조립" 안내로 조립한 첫 관절(J5, ID {fj.SERVO_ID} 서보)을 책상에 놓습니다. 서보 선(20 cm)은 '
            "서보에 꽂힌 채 앞벽 가운데 홈으로 나와 늘어져 있습니다.",
            "절연 테이프를 손가락 한 마디(약 2 cm) 길이로 한 조각 잘라 둡니다.",
            "네임펜을 꺼냅니다.",
            "서보는 전원에 잇지 않습니다. 손으로 돌려 봅니다.",
        ),
        check=(
            "조립한 첫 관절 (서보 선이 홈으로 나와 있다)",
            "절연 테이프 한 조각",
            "네임펜",
        ),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
        uses=(Use("tape", 1, "약 2 cm 한 조각"), Use("pen")),
    ),
    Step(
        "turn-long",
        "끝까지",
        "선을 붙일 끝까지 돌리기",
        do=(
            "한 손으로 아래 몸통(손목 1)을 책상에 눌러 잡습니다.",
            "다른 손으로 위 몸통(손목 2)을 그림처럼 천천히 돌립니다. 손목 2의 열린 뒤쪽이 홈에서 멀어지는 쪽이고, "
            "위에서 보면 시계 반대 방향입니다.",
            "멈춤 기둥이 살에 닿아 더 돌지 않을 때까지 돌립니다. 반 바퀴 가까이 돕니다.",
        ),
        warn=(
            "그림과 같은 쪽으로 돌립니다. 이쪽 끝에서 선이 가장 길게 감겨서, 여기서 선 길이를 맞춥니다.",
            "천천히 돌립니다. 손으로 돌리면 뻑뻑한 것이 정상입니다.",
            "늘어진 서보 선은 아직 그대로 둡니다.",
        ),
        check=("멈춤 기둥이 살에 닿아 더 돌지 않는다.",),
        stuck="어느 쪽으로 돌려야 할지 헷갈리면 그대로 사진을 보내 주세요.",
        adds=STILL,
        still=STILL,
        uses=(Use("servo_cable", note="홈에서 늘어뜨린 채 둔다", count=False),),
        turn=(0.0, LONG),
        view=ABOVE,
        shell=(fj.CHILD,),
        ghost=True,
        zoom=1.3,
        center=_MID,
        labels=(Label(fj.CHILD, "손목 2 (위)"), Label(fj.HOUSE, "손목 1 (아래)")),
        tags=(fj.EXIT_TAG,),
    ),
    Step(
        "lay",
        "선 감기",
        "선을 테에 감아 손목 2로 넣기",
        do=(
            "홈에서 나온 선을 살 위에 눕혀 출력 원판 쪽으로 보냅니다 (①).",
            "원판 테에 닿기 전에 선을 세워(넓은 면이 테를 보게), 손목 2가 돌아간 쪽으로 테를 따라 감습니다 (②).",
            "선 끝(플러그)을 손목 2의 열린 뒤쪽 모서리(③)로 넣어 안으로 밀어 넣습니다. 2 cm쯤 들어갑니다.",
            "손목 2 안에서 선을 살살 당겨, 감긴 선이 테에 가볍게 붙게 합니다.",
        ),
        warn=(
            "세게 당기지 않습니다. 테에 붙기만 하면 됩니다. 팽팽하게 당기면 반대쪽에서 남는 선이 줄어듭니다.",
            "손목 1 쪽(홈)은 테이프로 붙이지 않습니다. 붙이지 않아도 선이 제자리에 있는지가 이 시험에서 볼 것입니다.",
            "선이 원판과 몸통 사이의 틈에 끼지 않게 봅니다.",
        ),
        check=(
            "선이 살 위에 누웠다가 세워져 테에 감겼다.",
            "선 끝이 손목 2의 모서리로 들어갔다.",
        ),
        stuck=fj.PHOTO,
        adds=(LAYER, TAIL_PART, END_PLUG),
        uses=(Use("servo_cable", note="20 cm 선의 늘어진 끝", count=False),),
        hide=(HANG,),
        joint=LONG,
        view=ABOVE,
        shell=(fj.CHILD,),
        ghost=True,
        zoom=1.9,
        center=(-4.0, cb.LAYER_Y, 8.0),
        spots=(
            Spot(LAYER, "①", ((-33.0, cb.FLAT_Y + _T, 0.0),)),
            Spot(LAYER, "②", (_RIM_AT,)),
            Spot(TAIL_PART, "③", (_ENTRY_AT,)),
        ),
        tags=(fj.EXIT_TAG,),
    ),
    Step(
        "tape",
        "테이프",
        "선을 손목 2에 테이프로 붙이기",
        do=(
            "선이 손목 2로 들어간 모서리 바로 위(①)에서, 테이프로 선을 손목 2의 뒤쪽 가장자리에 붙입니다.",
            "테이프는 선을 가로질러 양옆의 몸통 가장자리에 걸치게 붙입니다.",
            "남은 선 끝과 플러그는 손목 2 안에 둡니다.",
        ),
        warn=(
            "붙인 뒤에는 선을 당기거나 밀지 않습니다. 이 길이가 다 만든 팔에서 테에 감길 선의 길이입니다.",
        ),
        check=(
            "선을 손으로 살짝 당겨도 테이프 자리에서 미끄러지지 않는다.",
            "감긴 선이 테에 가볍게 붙어 있다 (팽팽하지 않다).",
        ),
        stuck=fj.PHOTO,
        adds=(TAPE,),
        uses=(Use("tape", note="준비해 둔 한 조각", count=False),),
        hide=(HANG,),
        joint=LONG,
        view=ENTRY,
        shell=(fj.CHILD,),
        ghost=True,
        zoom=2.2,
        center=(24.0, cb.LAYER_Y + 8.0, -3.0),
        spots=(Spot(TAPE, "①", ((j.BACK_IN - wrist.AX_Y - 0.3, cb.LAYER_Y + 10.0, EZ),)),),
        labels=(Label(END_PLUG, "선 끝 플러그 (손목 2 안에 둔다)"),),
    ),
    Step(
        "mark",
        "표시",
        "홈 앞의 선에 줄 긋기",
        do=(
            "앞벽 가운데 홈에서 선이 나오는 곳(①)을 찾습니다.",
            "홈 가장자리에 딱 맞춰, 선 위에 네임펜으로 선을 가로지르는 짧은 줄을 긋습니다.",
        ),
        warn=(
            "돌려 본 뒤 이 줄이 홈 안으로 들어갔거나 밖으로 나왔으면, 선이 밀리거나 당겨진 것입니다.",
        ),
        check=("줄이 홈 가장자리에 맞춰 그어졌다.",),
        stuck=fj.PHOTO,
        adds=(MARK,),
        uses=(Use("pen"),),
        hide=(HANG,),
        joint=LONG,
        view=GROOVE,
        zoom=2.6,
        center=(wrist.X_FOOT + 2.0, cb.FLAT_Y, 0.0),
        spots=(Spot(MARK, "①", ((wrist.X_FOOT + 2.8, cb.FLAT_Y + _T, 0.0),)),),
        tags=(fj.EXIT_TAG,),
    ),
    Step(
        "turn-over",
        "반대쪽 끝",
        "반대쪽 끝까지 천천히 돌리기",
        do=(
            "아래 몸통을 눌러 잡고, 위 몸통을 반대쪽(위에서 보면 시계 방향)으로 천천히 돌립니다.",
            "가운데쯤에서 남는 선이 고리로 나왔다가, 반대쪽으로 다시 감기는 것을 보며 돌립니다.",
            "반대쪽 끝(멈춤 기둥이 살에 닿는 곳)까지 갑니다.",
        ),
        warn=(
            "어디든 걸리면 더 돌리지 말고 그 자리를 사진으로 보내 주세요.",
            "고리 모양은 그림과 달라도 됩니다. 그림은 어림입니다. 끼거나 당기지만 않으면 됩니다.",
        ),
        check=(
            "선이 원판과 몸통 사이 틈, 멈춤 귀, 멈춤 기둥에 끼지 않고 끝까지 돌았다.",
            "테이프 자리와 홈에서 선이 당겨지지 않았다.",
        ),
        stuck=fj.PHOTO,
        uses=(Use("servo_cable", note="테이프로 붙인 것", count=False),),
        hide=(HANG,),
        turn=(LONG, -LONG),
        view=TOP,
        shell=(fj.CHILD,),
        ghost=True,
        zoom=1.4,
        center=_MID,
        tags=(fj.EXIT_TAG,),
    ),
    Step(
        "repeat",
        "다섯 번",
        "다섯 번 오가며 보고 사진 찍기",
        do=(
            "끝에서 끝까지 천천히 다섯 번 오갑니다.",
            "홈 앞의 네임펜 줄이 홈 가장자리에 그대로 있는지 봅니다.",
            "세 자리에서 위에서 내려다본 사진을 한 장씩 찍어 대화창에 보냅니다: 선을 붙인 끝, 가운데, 반대쪽 끝.",
            "가운데 사진에는 남는 선의 고리가 몸통 밖으로 얼마나 나오는지 보이게 찍습니다.",
        ),
        warn=("오가다 걸리는 곳이 생기면 멈추고 그 자리를 사진으로 보내 주세요.",),
        check=(
            "다섯 번 오가도 걸리는 곳이 없다.",
            "네임펜 줄이 홈 가장자리에서 움직이지 않았다.",
            "사진 세 장을 보냈다.",
        ),
        stuck=fj.PHOTO,
        uses=(Use("servo_cable", note="테이프로 붙인 것", count=False),),
        hide=(HANG,),
        turn=(-LONG, LONG),
        view=TOP,
        shell=(fj.CHILD,),
        ghost=True,
        zoom=1.4,
        center=_MID,
        tags=(fj.EXIT_TAG,),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "선 감아 돌려 보기가 끝났습니다. 그림은 가운데로 돌린 모습입니다.",
            "테이프와 선은 그대로 둡니다. 300 mm 선으로 바꿀 때 뗍니다.",
            "걸리거나 밀린 곳이 없었으면 대화창에 알려 주세요. 나머지 손목 몸통을 뽑습니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요.",
        ),
        hide=(HANG,),
        view=View((0, 1, 0), (-0.6, 0, 1), 50, 0),
        shell=(fj.CHILD,),
        ghost=True,
        zoom=1.5,
        center=(-18.0, cb.LAYER_Y, 0.0),
    ),
)

GUIDE = Guide(
    id="cable-wrap",
    kind="measure",
    title="서보 선 감아 돌려 보기",
    sub=f"J5 · ID {fj.SERVO_ID} 서보 · 20 cm 선",
    scene=scene,
    steps=STEPS,
    items=fj.GUIDE.items,
)
