"""서보로 선 돌려 보기: 첫 관절(J5, ID 5 서보)을 서보로 천천히 끝에서 끝까지 돌리며, 손목 2 겉으로 감은 서보 선이
걸리거나 당겨지지 않는지 자세 넷에서 녹화한다 (status.md의 남은 질문 "서보 선이 관절을 넘는 방법").

선은 사용자가 손으로 찾은 길로 놓는다: 앞벽 가운데 홈 → 손목 2의 겉 → 손목 2의 뒤쪽 모서리 → J6 서보(ID 6). 이
길은 설계 코드에 없어서 그리지 않고, 나가는 곳과 들어가는 곳만 이름표로 가리킨다. 지금 꽂힌 20 cm 선으로 먼저 하고
300 mm 선이 오면 같은 순서로 다시 한다. 손목 2에는 J6 서보를 서보 받침과 뒷줄 벽 나사, 뒷덮개로만 단다 (J6의
베어링, 출력 원판은 달지 않는다). 손목 1의 뒷덮개도 이때 새로 뽑아 단다. 서보는 영점을 잰 뒤 실행 명령(`run`)이
힘 한도를 낮춰 멈춤에서 8° 안쪽(±155°)까지 돌린다. 서보 기록과 영상은 Claude가 분석한다.
"""

from __future__ import annotations

from dataclasses import replace

from horilab_design.guide import cables as cb
from horilab_design.guide.catalog import ITEMS
from horilab_design.guide.guides import first_joint as fj
from horilab_design.guide.model import Guide, Label, Move, ScenePart, Step, Tag, Use, View
from horilab_design.parts import joint as j
from horilab_design.parts import wrist

LIMIT = 155.0  # 시험에서 돌리는 끝 [°] (멈춤 ±163°에서 8° 안쪽)
PIN = wrist.PIN_POSE_DEG

J6_ID = 6
J6_ITEM = replace(
    ITEMS["servo_sts3215"],
    key="servo_sts3215_j6",
    name=f"서보 STS3215 · ID {J6_ID}",
    meta=f"혼이 달린 것. ID {J6_ID} 이름표가 붙은 서보 (J6)",
)

# 손목 2(자식) 안에 다는 J6 서보와 그것을 잡는 부품: 첫 관절의 손목 1 속 부품을 자식 자리로 옮긴 것
_J6_NAMES = ("서보 받침", "뒷덮개", "서보 몸통", "혼")
_J6_PREFIX = ("서보 뒷면 나사", "덮개 나사", "너트 (서보 받침)", "벽 나사 2", "벽 나사 4")


def _j6_parts() -> list[ScenePart]:
    out: list[ScenePart] = []
    for p in wrist.first_joint(fj.MODEL):
        if p.name not in _J6_NAMES and not p.name.startswith(_J6_PREFIX):
            continue
        key, count = next((k, c) for prefix, k, c in fj.PART_KEYS if p.name.startswith(prefix))
        if key == "servo_sts3215":
            key = J6_ITEM.key
        out.append(ScenePart(f"J6 {p.name}", wrist.as_child(p.shape), key, p.color, True, count))
    return out


J5_COVER = ("뒷덮개", "덮개 나사")  # 손목 1의 뒷덮개는 새로 뽑아 단다


def scene() -> list[ScenePart]:
    # 조립한 첫 관절을 그대로 쓴다 (뒷덮개 말고는 새로 드는 부품이 아니다). 선은 그리지 않는다
    out = [
        replace(p, count=p.name.startswith(J5_COVER))
        for p in fj.scene()
        if not p.name.startswith("서보 선")
    ]
    return out + _j6_parts()


ASSEMBLY = tuple(
    prefix for prefix, _key, _count in fj.PART_KEYS if prefix not in ("영점 핀", *J5_COVER)
)
J6_SERVO = ("J6 서보 받침", "J6 너트", "J6 서보 몸통", "J6 혼", "J6 서보 뒷면 나사")
J6_NUTS = "J6 너트 (서보 받침)"
J6_CARRIER = ("J6 서보 받침", J6_NUTS)


def _child_dir(v: tuple[float, float, float]) -> tuple[float, float, float]:
    """손목 2 서보 좌표의 방향을 장면 좌표로 (관절 각 0, wrist.as_child: x → +y, y → +x, z → -z)."""
    return (v[1], v[0], -v[2])


def _child_view(v: View) -> View:
    """첫 관절 안내에서 손목 1 속 부품을 보던 방향을 손목 2 속 같은 부품에 맞춘다."""
    return View(_child_dir(v.up), _child_dir(v.face), v.el, v.az)


_EX, _EZ = cb.servo_rod().end  # 손목 2로 들어가는 자리 (관절 각 0)
ENTRY_TAG = Tag((_EX, cb.LAYER_Y + 8.0, _EZ), "손목 2로 들어가는 곳 (뒤쪽 모서리)")

# 손목 2의 뒤쪽(열린 쪽)은 관절 각 0에서 손목 1의 발 쪽(-x)을 본다
J6_VIEW = View((0, 1, 0), (-1, 0, 0.5), 30, 0)
SIDE_VIEW = View((0, 1, 0), (0, 0, 1), 25, 30)
ZERO_VIEW = View((0, 1, 0), (-1, 0, 0), 35, 0)
# 자세 넷: 화면의 위가 실제의 위다. 휴대폰은 그림을 보는 쪽에서 찍는다
UP = View((0, 1, 0), (-1, 0, 0.5), 15, 0)  # 세워서: 손목 2가 위
DOWN = View((0, -1, 0), (-1, 0, 0.5), 15, 0)  # 거꾸로: 손목 2가 아래
SLOT_LOW = View((1, 0, 0), (0, 0, 1), 15, 0)  # 눕혀서: 앞벽 가운데 홈(발 쪽)이 아래
SLOT_HIGH = View((-1, 0, 0), (0, 0, 1), 15, 0)  # 눕혀서: 홈이 위

CABLE = Use("servo_cable", note="손목 2 겉으로 감은 것", count=False)
STOP_NOW = (
    "선이 걸리거나 당겨지면 바로 창에서 Enter를 누릅니다. 서보가 천천히 서서 그 자리에서 버팁니다. 더 급하면 "
    "서보 전원을 뽑습니다."
)
FINGERS = "손가락은 두 몸통 사이와 멈춤 귀 근처에 두지 않습니다."


def _pose(
    sid: str, short: str, title: str, hold: str, view: View, labels: tuple[Label, ...]
) -> Step:
    return Step(
        sid,
        short,
        title,
        do=(
            hold,
            "휴대폰을 그림을 보는 쪽에 세워 두고, 관절과 선이 다 화면에 들어오게 녹화를 시작합니다.",
            f"Claude가 띄운 창에서 Enter를 누르면 J5 서보(ID {fj.SERVO_ID})가 끝에서 끝까지 천천히 세 번 "
            "오갑니다. 2분 남짓 걸립니다.",
            "창에 끝났다고 나오면 녹화를 멈춥니다. 서보는 가운데에서 버티고 있습니다.",
        ),
        warn=(STOP_NOW, FINGERS),
        check=(
            "세 번 오가는 것을 처음부터 끝까지 녹화했다.",
            "걸리거나 당겨진 곳이 있었으면 대화창에 적었다.",
        ),
        stuck="녹화를 놓쳤으면 대화창에 알려 주세요. 같은 자세로 다시 돌립니다.",
        uses=(CABLE,),
        hide=("영점 핀",),
        turn=(-LIMIT, LIMIT),
        view=view,
        zoom=1.2,
        labels=labels,
        tags=(fj.EXIT_TAG,),
    )


STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            f'"첫 관절 조립" 안내로 조립한 첫 관절(J5, ID {fj.SERVO_ID} 서보)을 책상에 놓습니다. 20 cm 서보 선은 '
            "꽂힌 그대로 둡니다.",
            f'ID {J6_ID} 이름표가 붙은 서보 하나와, 출력 안내 "선 돌려 보기 출력"에서 뽑은 서보 받침 하나와 '
            "뒷덮개 둘을 꺼냅니다.",
            "가진 20 cm 서보 선을 하나 더 꺼냅니다. 어댑터로 가는 선입니다.",
            "서보 첫 연결 때 쓴 어댑터와 서보 전원, 휴대폰을 세울 거치대(없으면 기댈 물건)를 준비합니다.",
        ),
        warn=(
            '300 mm 선이 오면 같은 순서로 한 번 더 합니다. 그때는 "어댑터 선 꽂고 손목 1에 뒷덮개 달기" 단계에서 '
            "선을 바꿉니다.",
        ),
        check=(
            "조립한 첫 관절",
            f"ID {J6_ID} 서보, 새로 뽑은 서보 받침 하나와 뒷덮개 둘",
            "어댑터로 갈 20 cm 서보 선",
            "어댑터, 서보 전원, 휴대폰 거치대",
            "영점 핀",
        ),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
        uses=(Use("tape", 2, "약 5 cm 두 조각 (남는 선 붙이기)"),),
    ),
    Step(
        "j6-nuts",
        "받침 너트",
        "J6 서보 받침에 너트 넣기",
        do=(
            "새로 뽑은 서보 받침을 넓은 판이 아래, 날개 둘이 위로 오게 놓습니다.",
            "날개마다 바깥에 돋움이 있고, 그 옆면(판이 터진 쪽)에 너트 홈 입구가 있습니다. 그림에서 화살표가 "
            "들어가는 곳입니다.",
            "너트를 동전 세우듯 세워서 입구에 넣고, 홈 끝에 닿을 때까지 밉니다. 돋움 둘에 하나씩 넣습니다.",
        ),
        warn=(
            "두 입구는 같은 쪽(판이 터진 쪽)에 있습니다. 옆면에 네모난 구멍이 뚫린 쪽으로만 넣고, 막힌 면으로 "
            "밀지 않습니다.",
            "너트가 안 들어가면 공구로 세게 밀지 말고 사진을 보내 주세요. 홈 벽이 얇아 부러집니다.",
            "서보를 손목 2에 넣은 뒤에는 이 너트에 손이 닿지 않습니다. 지금 끝까지 들어갔는지 꼭 봅니다.",
        ),
        check=("돋움 바깥 면의 구멍으로 들여다보면 너트 구멍이 가운데에 보인다 (둘 다).",),
        stuck=fj.PHOTO,
        adds=J6_CARRIER,
        still=("J6 서보 받침",),
        focus=J6_CARRIER,
        moves=(
            Move((J6_NUTS + "@z+",), _child_dir((13.0, 0.0, 0.0))),
            Move((J6_NUTS + "@z-",), _child_dir((13.0, 0.0, 0.0)), 0.8),
        ),
        view=_child_view(View((0, 1, 0), (1, 0, 0), 30, 35)),
        labels=(Label(J6_NUTS + "@z+", "M3 너트 × 2"),),
    ),
    Step(
        "j6-carrier",
        "받침 달기",
        "J6 서보에 서보 받침 달기",
        do=(
            f"ID {J6_ID} 이름표가 붙은 서보를 혼이 아래로 가게 엎어 놓습니다.",
            "서보 받침을 날개가 아래(혼 쪽)로 가게 들고, 판을 서보 뒷면에 얹습니다. 판의 작은 구멍 넷이 서보 "
            "뒷면의 나사 구멍 넷과 맞아야 합니다. 거꾸로 놓으면 구멍이 맞지 않습니다.",
            "서보 뒷면 나사 4개를 작은 십자 드라이버로 조입니다.",
        ),
        warn=(
            "나사를 다 조였는데 판이 흔들리면 나사가 서보 구멍 바닥에 먼저 닿은 것입니다. 머리 밑에 와셔를 하나 "
            "넣고, 넣었다고 알려 주세요.",
            "자가탭 나사는 세게 조이지 않습니다. 서보 구멍의 나사산이 뭉개질 수 있습니다.",
        ),
        check=(
            "판이 서보 뒷면에 붙어 흔들리지 않는다.",
            "나사 머리 넷이 판의 머리 자리에 고르게 앉았다.",
        ),
        stuck=fj.PHOTO,
        adds=("J6 서보 몸통", "J6 혼$", "J6 서보 뒷면 나사"),
        still=("J6 서보 몸통", "J6 혼$"),
        uses=(
            Use("carrier", note="앞 단계에서 너트 둘을 넣은 것", count=False),
            Use("driver"),
            Use("washer_m3", 1, "예비: 판이 끝까지 안 조일 때"),
        ),
        focus=J6_SERVO,
        moves=(
            Move(J6_CARRIER, _child_dir((0.0, -25.0, 0.0))),
            Move(("J6 서보 뒷면 나사",), _child_dir((0.0, -12.0, 0.0)), 1.4),
        ),
        view=_child_view(View((0, -1, 0), (0, 0, 1), 35, 20)),
        labels=(
            Label("J6 서보 받침", "서보 받침 (너트 둘)"),
            Label("J6 서보 뒷면 나사 1", "서보 뒷면 나사 × 4"),
        ),
    ),
    Step(
        "j6-servo",
        "J6 서보",
        "손목 2에 J6 서보 넣기",
        do=(
            "위 몸통(손목 2)의 뒤쪽(열린 쪽)으로, 서보의 긴 쪽(혼에서 먼 끝)이 손목 2의 발 쪽을 보게 넣습니다.",
            "허브 판과 출력 원판이 없어서 서보가 끝까지 미끄러지지 않습니다. 서보 받침 돋움의 구멍이 벽의 뒷줄 "
            "구멍으로 보이는 자리에서 손으로 잡아 둡니다.",
        ),
        warn=(
            "J6의 베어링, 허브 판, 출력 원판은 달지 않습니다. 서보를 손목 2에 잡아 두기만 합니다.",
        ),
        check=(
            "서보 받침의 너트 둘이 끝까지 들어갔다.",
            "서보 받침 돋움의 구멍 둘이 벽의 뒷줄 구멍으로 보인다.",
        ),
        stuck=fj.PHOTO,
        adds=ASSEMBLY,
        still=ASSEMBLY,
        mount=(*ASSEMBLY, *J6_SERVO),
        uses=(Use(J6_ITEM.key, note="앞 단계에서 서보 받침을 단 것", count=False),),
        moves=(Move(J6_SERVO, (-50.0, 0.0, 0.0)),),
        view=J6_VIEW,
        shell=(fj.CHILD,),
        ghost=True,
        labels=(
            Label("J6 서보 몸통", f"J6 서보 (ID {J6_ID})"),
            Label(fj.CHILD, "손목 2 (위 몸통)"),
        ),
    ),
    Step(
        "j6-lock",
        "서보 잠금",
        "뒷줄 벽 나사로 J6 서보 잠그기",
        do=(
            "손목 2 벽의 뒷줄(뒤쪽 열린 끝에 가까운 줄) 구멍 둘에 M3 × 8 나사를 하나씩 넣고 육각렌치로 조입니다.",
            "머리가 벽에 닿아 더 돌지 않으면 멈춥니다.",
        ),
        warn=("머리가 닿은 뒤로 세게 더 조이지 않습니다.",),
        check=("나사 둘의 머리가 벽에 닿았다.", "서보를 뒤에서 밀어도 움직이지 않는다."),
        adds=("J6 벽 나사",),
        uses=(Use("hexkey"),),
        moves=(
            Move(("J6 벽 나사 2",), (0.0, 0.0, -14.0)),
            Move(("J6 벽 나사 4",), (0.0, 0.0, 14.0), 0.7),
        ),
        view=SIDE_VIEW,
        labels=(Label("J6 벽 나사 2", "M3 × 8"), Label("J6 벽 나사 4", "M3 × 8")),
    ),
    Step(
        "j5-cover",
        "J5 덮개",
        "어댑터 선 꽂고 손목 1에 뒷덮개 달기",
        do=(
            "손목 1을 뒤쪽(열린 쪽)이 위로 오게 듭니다. 지난 시험의 테이프는 뗍니다.",
            f"J5 서보(ID {fj.SERVO_ID})의 남은 커넥터에 어댑터로 갈 20 cm 선을 꽂아, 앞벽 양옆의 홈 하나로 "
            "뺍니다.",
            "지금 꽂힌 시험 선은 그대로 서보 받침 판 위에 눕혀 앞벽 가운데 홈으로 나와 있게 둡니다.",
            "새로 뽑은 뒷덮개를 테가 몸통 쪽으로 가게 얹고, M3 × 12 나사 2개로 조입니다.",
        ),
        warn=(
            "선이 덮개 테 밑에 끼이지 않게 봅니다.",
            "300 mm 선으로 다시 할 때는 이 단계에서 덮개를 떼고 시험 선을 300 mm 선으로 바꿉니다.",
        ),
        check=(
            "시험 선이 앞벽 가운데 홈으로, 어댑터 선이 옆 홈으로 나와 있다.",
            "뒷덮개가 들뜨지 않고 원통 뒤 끝을 덮었다.",
        ),
        stuck=fj.PHOTO,
        adds=J5_COVER,
        uses=(Use("servo_cable", 1, "20 cm, 어댑터로 가는 것"), Use("hexkey")),
        moves=(
            Move(("뒷덮개",), (0.0, -30.0, 0.0)),
            Move(("덮개 나사",), (0.0, -16.0, 0.0), 1.0),
        ),
        view=fj.BACK_UP,
        labels=(
            Label("뒷덮개", "뒷덮개 (새로 뽑은 것)"),
            Label("덮개 나사 M3x12 1", "M3 × 12 × 2"),
        ),
        tags=(fj.BACK_TAG,),
    ),
    Step(
        "lay",
        "선 놓기",
        "손목 2 겉으로 선 놓고 길이 맞추기",
        do=(
            "보내 주신 사진의 길로 선을 놓습니다: 앞벽 가운데 홈에서 나와 손목 2의 겉을 돌아, 손목 2의 뒤쪽 "
            f"모서리로 넣어 J6 서보(ID {J6_ID})의 커넥터 하나에 꽂습니다.",
            "서보는 아직 전원에 잇지 않습니다. 위 몸통을 손으로 천천히 한쪽 끝까지, 가운데를 지나 반대쪽 "
            "끝까지 돌려 봅니다.",
            "어느 자리에서도 선이 팽팽해지지 않고 플러그가 당겨지지 않는지 봅니다.",
            "300 mm 선으로 할 때는 선이 가장 길게 필요한 끝에서 살짝 처질 만큼만 두고, 남는 선은 접어서 손목 2 "
            "겉에 테이프로 붙입니다.",
            "다 놓으면 양 끝과 가운데에서 사진을 한 장씩 보내 주세요.",
        ),
        warn=(
            "20 cm 선이 어느 끝에서 팽팽해지면 거기서 멈추고 사진을 보내 주세요. 300 mm 선이 온 뒤에 합니다.",
            "남는 선을 관절 둘레에 두지 않습니다. 고리가 커져서 실제 팔과 다른 시험이 됩니다.",
            "손으로 돌릴 때는 천천히 돌립니다. 끝에서 멈춤 기둥이 살에 닿으면 더 밀지 않습니다.",
        ),
        check=(
            "양 끝과 가운데 어디에서도 선이 팽팽하지 않다.",
            "선을 놓은 사진 세 장을 보냈다.",
        ),
        stuck=fj.PHOTO,
        uses=(CABLE, Use("tape", note="300 mm 선일 때 남는 선 붙이기", count=False)),
        turn=(-j.STOP_DEG, j.STOP_DEG),
        view=UP,
        zoom=1.2,
        labels=(Label(fj.CHILD, "손목 2 (겉으로 감는다)"),),
        tags=(fj.EXIT_TAG, ENTRY_TAG),
    ),
    Step(
        "j6-cover",
        "J6 덮개",
        "손목 2에 뒷덮개 달기",
        do=(
            "새로 뽑은 뒷덮개를 테가 몸통 쪽으로 가게 들고 손목 2의 뒤쪽에 얹습니다.",
            "M3 × 12 나사 2개를 넣고 육각렌치로 조입니다.",
        ),
        warn=("선이 덮개 테 밑에 끼이지 않게 봅니다. 선은 뒤쪽 모서리로 들어가 있어야 합니다.",),
        check=("덮개가 들뜨지 않고 원통 뒤 끝을 덮었다.", "선이 덮개 밑에 끼지 않았다."),
        stuck=fj.PHOTO,
        adds=("J6 뒷덮개", "J6 덮개 나사"),
        uses=(Use("hexkey"),),
        moves=(
            Move(("J6 뒷덮개",), (-30.0, 0.0, 0.0)),
            Move(("J6 덮개 나사",), (-16.0, 0.0, 0.0), 1.0),
        ),
        view=J6_VIEW,
        labels=(
            Label("J6 뒷덮개", "뒷덮개 (새로 뽑은 것)"),
            Label("J6 덮개 나사 M3x12 1", "M3 × 12 × 2"),
        ),
    ),
    Step(
        "connect",
        "전원",
        "어댑터와 서보 전원 잇기",
        do=(
            "서보 첫 연결 때처럼, 옆 홈으로 나온 어댑터 선을 어댑터에 꽂고 어댑터를 PC에 잇습니다.",
            "서보 전원을 어댑터에 잇고 켭니다.",
            f"대화창에 알려 주시면 Claude가 ID {fj.SERVO_ID}와 ID {J6_ID} 서보가 보이는지 읽어 봅니다. 서보는 "
            "움직이지 않습니다.",
        ),
        check=(f"Claude가 ID {fj.SERVO_ID}와 ID {J6_ID} 서보를 찾았다고 알렸다.",),
        stuck="서보가 안 보이면 어댑터와 선을 꽂은 사진을 보내 주세요.",
        uses=(Use("servo_cable", note="20 cm, 어댑터로 가는 것", count=False),),
        view=UP,
        zoom=1.2,
    ),
    Step(
        "zero-dir",
        "+ 쪽",
        "영점 재기 ①: + 쪽으로 돌리기",
        do=(
            "손목 1을 책상에 세우고 손목 2가 위에 오게 둡니다. 영점 핀은 아직 꽂지 않습니다.",
            "Claude가 영점 재기 창을 띄웁니다. 창이 시키는 대로 합니다.",
            "창이 + 쪽으로 돌리라고 하면, 위 몸통을 그림처럼 20°쯤 돌립니다. + 쪽은 위에서 내려다볼 때 시계 "
            "방향입니다.",
        ),
        warn=(
            "반대쪽으로 돌리면 서보가 양 끝을 거꾸로 알게 됩니다. 헷갈리면 돌리기 전에 대화창에 물어 주세요.",
        ),
        check=("창이 + 쪽으로 돈 것을 받았다고 했다.",),
        stuck="창의 말이 헷갈리면 창을 그대로 두고 대화창에 알려 주세요.",
        uses=(Use("pin_msh3_20", note="첫 관절에 쓴 것, 다음 단계에서 꽂는다", count=False),),
        turn=(PIN - 30.0, PIN),
        view=View((0, 1, 0), (0, 0, 1), 55, 0),
        zoom=1.3,
        labels=(Label(fj.CHILD, "손목 2: 위에서 볼 때 시계 방향이 + 쪽"),),
    ),
    Step(
        "zero-pin",
        "영점 핀",
        "영점 재기 ②: 핀 꽂고 재기",
        do=(
            "창이 핀을 꽂으라고 하면, 위 몸통을 그림의 자리(가운데에서 90°)로 돌려 영점 핀을 손목 2의 핀 귀 "
            "구멍으로 넣어 살의 구멍까지 밉니다.",
            "위 몸통을 + 쪽(위에서 볼 때 시계 방향)으로 살짝 밀어 핀의 틈을 붙인 채, 창에서 Enter를 누릅니다.",
            "다 됐다고 나오면 핀을 곧게 당겨 뺍니다.",
        ),
        warn=("핀을 꼭 뺍니다. 꽂힌 채 서보가 돌면 핀과 귀가 부러집니다.",),
        check=("창에 영점을 다 쟀다고 나왔다.", "영점 핀을 뺐다."),
        stuck="창의 말이 헷갈리면 창을 그대로 두고 대화창에 알려 주세요.",
        adds=("영점 핀",),
        uses=(Use("pin_msh3_20", note="첫 관절에 쓴 것", count=False),),
        joint=PIN,
        view=ZERO_VIEW,
        zoom=1.4,
        labels=(Label("영점 핀", "영점 핀 (다 재면 뺀다)"),),
    ),
    _pose(
        "pose-up",
        "세워서",
        "세워서 돌리기",
        "그림처럼 손목 1을 책상에 세우고 한 손으로 눌러 잡습니다. 손목 2가 위입니다.",
        UP,
        (Label(fj.CHILD, "손목 2 (위)"), Label(fj.HOUSE, "손목 1 (책상에 눌러 잡기)")),
    ),
    _pose(
        "pose-down",
        "거꾸로",
        "거꾸로 들고 돌리기",
        "손목 1을 한 손으로 들어 뒤집습니다. 손목 2가 아래로 오고, 책상에 닿지 않게 띄웁니다.",
        DOWN,
        (Label(fj.CHILD, "손목 2 (아래, 띄우기)"), Label(fj.HOUSE, "손목 1 (손으로 들기)")),
    ),
    _pose(
        "pose-slot-low",
        "눕혀서 ①",
        "눕혀서 돌리기: 앞벽 가운데 홈이 아래",
        "손목 1을 한 손으로 들어 관절 축이 옆으로 눕게 돌립니다. 선이 나오는 앞벽 가운데 홈이 아래를 봅니다.",
        SLOT_LOW,
        (Label(fj.HOUSE, "손목 1 (손으로 들기)"),),
    ),
    _pose(
        "pose-slot-high",
        "눕혀서 ②",
        "눕혀서 돌리기: 앞벽 가운데 홈이 위",
        "앞의 자세에서 반 바퀴 돌려 앞벽 가운데 홈이 위를 보게 듭니다.",
        SLOT_HIGH,
        (Label(fj.HOUSE, "손목 1 (손으로 들기)"),),
    ),
    Step(
        "fast",
        "빠르게",
        "가장 나빴던 자세에서 빠르게 돌리기",
        do=(
            "앞의 네 자세 가운데 선이 가장 크게 움직였거나 가장 걸릴 뻔했던 자세를 대화창에 알려 주세요. Claude가 "
            "실제로 쓸 빠르기의 창을 띄웁니다.",
            "그 자세로 들고 녹화를 시작한 뒤 창에서 Enter를 누릅니다. 끝에서 끝까지 세 번 빠르게 오갑니다.",
            "끝나면 녹화를 멈춥니다. 마지막에 창이 시키면 위 몸통을 받치고 Enter를 누릅니다. 서보 힘이 꺼집니다.",
        ),
        warn=(
            STOP_NOW,
            FINGERS,
            "빠르게 돌아서 위 몸통이 손에 부딪힐 수 있습니다. 손은 손목 1만 잡습니다.",
        ),
        check=("세 번 오가는 것을 녹화했다.", "서보 힘이 꺼졌다 (위 몸통이 손으로 돌아간다)."),
        stuck="녹화를 놓쳤으면 대화창에 알려 주세요. 다시 돌립니다.",
        uses=(CABLE,),
        hide=("영점 핀",),
        turn=(-LIMIT, LIMIT),
        view=UP,
        zoom=1.2,
        tags=(fj.EXIT_TAG,),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "서보로 선 돌려 보기가 끝났습니다. 서보 전원을 끕니다.",
            "영상 파일을 PC의 폴더 하나에 옮기고, 그 폴더 경로를 대화창에 적어 주세요. Claude가 영상과 서보 "
            "기록을 맞춰 분석합니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요.",
        ),
        hide=("영점 핀",),
        view=UP,
        zoom=1.2,
    ),
)

GUIDE = Guide(
    id="cable-motion",
    kind="measure",
    title="서보로 선 돌려 보기",
    sub=f"J5 · ID {fj.SERVO_ID} 서보 · 20 cm 선 (300 mm 선이 오면 다시) · 자세 넷",
    scene=scene,
    steps=STEPS,
    items={**fj.GUIDE.items, J6_ITEM.key: J6_ITEM},
)
