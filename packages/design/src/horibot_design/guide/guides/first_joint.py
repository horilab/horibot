"""첫 관절 조립 안내: 손목 몸통 둘(집과 자식)과 속 부품, STS3215 (0028, 0029).

단계 글은 3D 페이지 "STS 관절 구조"의 여덟 단계를 손동작 하나씩으로 나눈 것이다. 자리와 방향은 설계 코드의 값을
쓴다 (너트 홈 입구, 벽 나사 줄, 손볼 자리). 선의 길은 0031을 따른다.
"""

from __future__ import annotations

from build123d import Box, Compound, Plane, Vector

from horibot_design.guide.catalog import ITEMS, printed_items
from horibot_design.guide.model import (
    Guide,
    Item,
    Label,
    Move,
    Place,
    ScenePart,
    Spot,
    Step,
    Tag,
    Use,
    View,
)
from horibot_design.parts import joint as j
from horibot_design.parts import wrist
from horibot_design.parts.shapes import box, solid
from horibot_design.values import ServoModel

MODEL: ServoModel = "STS3215"
DY = j.servo_dy(MODEL)  # 서보와 거기 달린 서보 받침, 뒷덮개가 축 방향으로 옮겨진 값
# 이 관절은 팔의 J5다: 서보가 든 손목 몸통이 J5의 집(손목 1)이고, 자식은 J6의 집이 될 손목 2다 (0029). 서보 ID는
# 관절 번호다 (servo-setup.md)
SERVO_ID = 5
SERVO_ITEM = Item(
    "servo_sts3215",
    f"서보 STS3215 · ID {SERVO_ID}",
    "bought",
    f"혼이 달린 것. ID {SERVO_ID} 이름표가 붙은 서보 (이 관절은 J{SERVO_ID})",
    thumb=ITEMS["servo_sts3215"].thumb,
)

# 서보 선이 지나는 길 (0031): +z 쪽 커넥터에 꽂아 서보 받침 판 위에 눕히고, 서보 먼 끝을 따라 앞쪽으로 내려 앞벽
# 가운데 홈으로 뺀다. 홈을 나온 뒤는 발 쪽으로 늘어뜨린 모습이다 (출력 원판 둘레에 감는 길은 안내 "서보 선 감아
# 돌려 보기"가 그린다, guides/cable_wrap.py)
_W, _T = j.WIRE_W, j.WIRE_T
_PLUG_Z = (0.45, 9.46)  # +z 쪽 커넥터에 꽂은 플러그 (parts/joint.py의 servo())
_ZC = sum(_PLUG_Z) / 2
_WY = (j.WIRE_TOP + DY, j.PLUG_TOP + DY)  # 판 위에 눕힌 선의 두께 범위
_XD = -(j.FAR + 0.3 + _T / 2)  # 서보 먼 끝 앞으로 내리는 선의 가운데 (먼 끝 면에서 0.3 띄운다)


def _slant(a: tuple[float, float], b: tuple[float, float]) -> Compound:
    """판 위에서 비스듬히 가는 선 한 토막: (x, z) a에서 b까지, 두께는 y 방향. 이음매를 덮게 양 끝을 늘린다."""
    va, vb = Vector(a[0], 0.0, a[1]), Vector(b[0], 0.0, b[1])
    d = vb - va
    x = d.normalized()
    mid = (va + vb) * 0.5 + Vector(0.0, sum(_WY) / 2, 0.0)
    plane = Plane(origin=mid, x_dir=x, z_dir=x.cross(Vector(0.0, 1.0, 0.0)))
    return solid(plane * Box(d.length + _W, _T, _W))


def cable_parts(exit_z: float = 0.0) -> tuple[Compound, Compound, Compound]:
    """서보 선: (꽂은 플러그, 손목 몸통 속의 선, 앞벽 홈을 나와 발 쪽으로 늘어뜨린 끝).

    exit_z는 선이 나가는 앞벽 홈의 z다: 0이면 가운데 홈, wrist.SIDE_LANE이면 +z 옆 홈 (0031의 고친 길).
    """
    z0, z1 = exit_z - _W / 2, exit_z + _W / 2
    plug = box(-16.6, -11.85, j.PLUG_TOP + DY, j.L1 + 0.5 + DY, *_PLUG_Z)
    inside = box(-26.0, -12.6, *_WY, _ZC - _W / 2, _ZC + _W / 2)  # 판 위, 발 쪽으로
    inside += _slant((-26.0, _ZC), (-33.0, exit_z))  # 나가는 홈 쪽으로 옮기며
    inside += box(_XD - _T / 2, -33.0, *_WY, z0, z1)
    inside += box(_XD - _T / 2, _XD + _T / 2, _WY[0], j.FRONT + _T, z0, z1)  # 먼 끝 앞으로
    hang = box(-45.0, _XD + _T / 2, j.FRONT, j.FRONT + _T, z0, z1)  # 홈을 나와 발 쪽으로
    return solid(plug), solid(inside), solid(hang)


# 장면 부품 이름(설계 코드) → 부품 목록 키. 앞에서부터 이름의 앞부분이 맞는 것을 쓴다. 세 번째는 개수에 넣는지
PART_KEYS: tuple[tuple[str, str, bool], ...] = (
    ("손목 몸통", "wrist_body", True),
    ("베어링", "bearing_6806", True),
    ("허브 판", "hub_plate", True),
    ("출력 원판", "out_disc", True),
    ("누름 틀", "retainer", True),
    ("서보 받침", "carrier", True),
    ("뒷덮개", "wrist_cover", True),
    ("서보 몸통", "servo_sts3215", True),
    ("혼 볼트", "m3x20", True),
    ("혼", "servo_sts3215", False),  # 서보에 딸린 혼
    ("벽 나사", "m3x8", True),
    ("멈춤 나사", "m3x10", True),
    ("몸통 나사", "m3x10", True),
    ("덮개 나사", "m3x12", True),
    ("서보 뒷면 나사", "selftap_so101", True),
    ("너트", "nut_m3", True),
    ("영점 핀", "pin_msh3_20", True),
)
# 설계 코드의 검사용 부품 가운데 장면에 두지 않는 것: 눕힌 선이 차지하는 자리, 커넥터 둘에 꽂은 플러그(선은 하나만
# 꽂는다. 커넥터 자리는 손볼 자리로 가리킨다)
_SKIP = ("선 자리", "플러그")


def scene() -> list[ScenePart]:
    out: list[ScenePart] = []
    for p in wrist.first_joint(MODEL):
        if p.name in _SKIP:
            continue
        key, count = next((k, c) for prefix, k, c in PART_KEYS if p.name.startswith(prefix))
        out.append(ScenePart(p.name, p.shape, key, p.color, p.moving, count))
    plug, inside, hang = cable_parts()
    out.append(ScenePart("서보 선 플러그", plug, "servo_cable", "plug", False, False))
    out.append(ScenePart("서보 선", solid(inside + hang), "servo_cable", "wire", False, False))
    return out


HOUSE = "손목 몸통 (부모)"
CHILD = "손목 몸통 (자식)"
DISC = "출력 원판$"
RET_NUTS = "너트 (누름 틀)"
CAR_NUTS = "너트 (서보 받침)"
DISC_NUTS = "너트 (출력 원판)"
CARRIER = ("서보 받침", CAR_NUTS)
SERVO_SET = ("서보 몸통", "혼$", "서보 받침", CAR_NUTS, "서보 뒷면 나사")
DISC_SET = (DISC, DISC_NUTS, "멈춤 나사", "너트 (멈춤 기둥)")

# 손에 드는 방향
BACK_UP = View((0, -1, 0), (1, 0, 0), 30, 28)  # 뒤쪽(열린 쪽)이 위
FRONT_UP = View((0, 1, 0), (1, 0, 0), 30, 28)  # 앞쪽(출력 원판)이 위

BACK_TAG = Tag((0.0, j.BACK_END - 9.0, 0.0), "뒤 (열린 쪽)")
FOOT_TAG = Tag((wrist.X_FOOT - 6.0, 6.0, 0.0), "발 쪽 (납작한 끝)")
EAR_TAG = Tag((j.R_POST + 8.0, j.ATOP, 0.0), "귀")
BACK_ROW = j.CAR_SCREW_Y  # 서보를 잠그는 벽 나사 줄 (몸통 좌표)
# 앞벽 가운데 선 홈 (발 쪽 끝): 서보 선이 살 위로 나가는 곳 (0031)
EXIT_TAG = Tag((wrist.X_FOOT + 1.2, j.FRONT + 3.0, 0.0), "앞벽 가운데 홈")


def _slide(name: str, reach: float = 13.0) -> tuple[float, float, float]:
    """옆 홈에 너트를 밀어 넣는 움직임: 입구 쪽으로 reach만큼 떨어진 자리에서 들어온다."""
    _n, _c, _a, s = next(slot for slot in j.side_nut_slots() if slot[0] == name)
    return (s[0] * reach, s[1] * reach, s[2] * reach)


_CAR_Y = j.L1 - j.PLATE_T + DY  # 서보 받침 판의 뒷면 (조립한 자리)
_COVER_Y = j.BACK_OUT + DY  # 뒷덮개 바깥 면

PHOTO = "막히면 그 자리를 사진으로 보내 주세요."

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "목록의 부품, 나사, 공구를 모두 꺼내 책상에 늘어놓습니다.",
            "나사는 길이별로 따로 모읍니다. M3 × 8, × 10, × 12는 2 mm씩 차이라 헷갈리기 쉽습니다.",
            "길이를 모르는 나사가 섞여 있으면 자나 캘리퍼스 옆에 놓고 찍어 대화창에 보내 주세요. "
            "Claude가 알려 드립니다.",
        ),
        check=(
            "출력 부품 7개: 흰색 손목 몸통 둘, 검정 다섯",
            f"베어링, 서보(ID {SERVO_ID}), 영점 핀, 서보 선",
            "나사와 너트를 개수대로",
            "공구",
        ),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    Step(
        "clean",
        "실 걷기",
        "처진 실 걷어 내기",
        do=(
            "출력 원판의 윗면(매끈한 면)에서, 귀의 나사 머리 자리 ①에 처진 실을 걷어 냅니다.",
            "서보 받침의 뒷면(넓은 판 쪽)에서, 먼 가로대 밑의 선 홈 ②와 작은 나사 머리 자리 넷 ③의 실을 걷어 냅니다.",
            "뒷덮개의 바깥 면에서, 나사 머리 자리 둘 ④의 실을 걷어 냅니다.",
        ),
        warn=("실만 걷어 냅니다. 구멍 가장자리를 깎아 넓히지 않습니다.",),
        check=("네 군데 모두 실이 남지 않았다. 손톱으로 긁어 걸리는 것이 없다.",),
        stuck="서보 받침의 덮개 나사 기둥 바로 위(돋움 안쪽 모서리)가 많이 지저분하면 사진을 보내 주세요. "
        "고친 모양으로 다시 뽑을지 봅니다.",
        uses=(
            Use("out_disc", 1, count=False),
            Use("carrier", 1, count=False),
            Use("wrist_cover", 1, count=False),
            Use("tweezers"),
            Use("knife"),
        ),
        focus=(DISC, "서보 받침", "뒷덮개"),
        place=(
            Place(DISC, (-84.0, 0.0, 0.0), (0, 1, 0), (1, 0, 0)),
            Place("서보 받침", (0.0, 0.0, 0.0), (0, -1, 0), (-1, 0, 0)),
            Place("뒷덮개", (82.0, 0.0, 0.0), (0, -1, 0), (1, 0, 0)),
        ),
        spots=(
            Spot(DISC, "①", ((j.R_POST, j.ATOP, 0.0),)),
            Spot("서보 받침", "②", ((sum(j.WIRE_GROOVE_X) / 2, _CAR_Y, 0.0),)),
            Spot("서보 받침", "③", tuple((x, _CAR_Y, z) for x, z in j.BACK_HOLES)),
            Spot("뒷덮개", "④", tuple((x, _COVER_Y, z) for x, z in j.COVER_BOSS)),
        ),
        labels=(
            Label(DISC, "출력 원판 · 윗면", (-j.R_DRUM, j.ATOP, 0.0)),
            Label("서보 받침", "서보 받침 · 뒷면", (10.0, _CAR_Y, 0.0)),
            Label("뒷덮개", "뒷덮개 · 바깥 면", (wrist.X_FOOT, _COVER_Y, 0.0)),
        ),
        view=View((0, 1, 0), (0, 0, 1), 58, 0),
    ),
    Step(
        "bearing",
        "베어링",
        "베어링 넣기",
        do=(
            "손목 몸통 하나를 뒤쪽(넓게 열린 쪽)이 위로 오게 잡습니다. 둘은 같은 부품이라 아무것이나 씁니다.",
            "베어링을 열린 쪽으로 넣습니다. 앞 끝 가까이까지는 저절로 내려갑니다.",
            "바깥 링을 엄지로 눌러, 앞 끝의 턱에 닿을 때까지 밉니다. 마지막 6.6 mm는 꾹 눌러야 들어갑니다.",
        ),
        warn=(
            "바깥 링만 누릅니다. 안쪽 링을 누르면 베어링 속의 알이 상할 수 있습니다.",
            "엄지로 끝까지 안 들어가거나 넣은 뒤 무겁게 돌면 두드리거나 바이스로 누르지 않습니다. 사진을 보내 주세요.",
        ),
        check=(
            "베어링이 턱에 닿았다. 빙 둘러 깊이가 같고 기울지 않았다.",
            "안쪽 링을 손가락으로 돌리면 걸림 없이 부드럽게 돈다.",
        ),
        adds=(HOUSE, "베어링"),
        still=(HOUSE,),
        moves=(Move(("베어링",), (0.0, -62.0, 0.0)),),
        view=BACK_UP,
        shell=(HOUSE,),
        cut=True,
        labels=(Label("베어링", "베어링 6806"),),
        tags=(BACK_TAG, FOOT_TAG, Tag((-(j.R_LIP + 1.5), j.FRONT, 0.0), "앞 끝 턱")),
    ),
    Step(
        "retainer-nuts",
        "틀 너트",
        "누름 틀에 너트 넣기",
        do=(
            "누름 틀을 고리가 아래, 다리 셋이 위로 오게 놓습니다.",
            "다리마다 옆면에 너트 홈의 입구가 있습니다. 그림에서 화살표가 들어가는 곳입니다.",
            "너트를 동전 세우듯 세워서 입구에 넣고, 홈 끝에 닿을 때까지 밉니다. 다리 셋에 하나씩 넣습니다.",
        ),
        warn=(
            "너트가 홈 끝까지 들어가야 다음 단계의 나사가 너트에 닿습니다. 손가락으로 안 되면 육각렌치의 긴 쪽으로 "
            "밀어 넣습니다.",
        ),
        check=("다리 바깥 면의 구멍으로 들여다보면 너트 구멍이 가운데에 보인다 (다리 셋 모두).",),
        adds=("누름 틀", RET_NUTS),
        still=("누름 틀",),
        focus=("누름 틀", RET_NUTS),
        moves=(
            Move((RET_NUTS + "@z+",), _slide("누름 틀 +z")),
            Move((RET_NUTS + "@x+",), _slide("누름 틀 +x"), 0.8),
            Move((RET_NUTS + "@z-",), _slide("누름 틀 -z"), 1.6),
        ),
        view=View((0, -1, 0), (1, 0, 0), 26, 36),
        labels=(Label(RET_NUTS + "@x+", "M3 너트 × 3"),),
        tags=(
            Tag((-(j.R_SLIDE + 2.0), j.RET_Y[1] + 2.0, 0.0), "다리 없는 쪽"),
            Tag((0.0, j.B0 + 1.5, j.R_SLIDE + 1.0), "고리"),
        ),
    ),
    Step(
        "retainer-in",
        "틀 넣기",
        "누름 틀 넣기",
        do=(
            "손목 몸통을 뒤쪽(열린 쪽)이 위로 오게 잡습니다. 베어링이 아래에 있습니다.",
            "누름 틀을 고리가 아래로 가게 들고, 다리가 없는 쪽을 발(납작한 끝) 쪽으로 돌립니다.",
            "그대로 열린 쪽에 넣어, 고리가 베어링에 닿을 때까지 내립니다.",
        ),
        warn=(
            "다리가 벽의 나사 구멍 셋과 맞아야 다음 단계의 나사가 너트에 닿습니다. 넣기 전에 방향을 그림과 맞춥니다.",
        ),
        check=("아래쪽 줄(앞줄)의 벽 구멍 셋으로 들여다보면 너트 구멍이 가운데에 보인다.",),
        uses=(
            Use("retainer", note="3단계에서 너트 셋을 넣은 것", count=False),
            Use("wrist_body", note="베어링을 넣은 것", count=False),
        ),
        mount=("누름 틀", RET_NUTS),
        moves=(Move(("누름 틀", RET_NUTS), (0.0, -58.0, 0.0)),),
        view=BACK_UP,
        shell=(HOUSE,),
        cut=True,
        labels=(Label("누름 틀", "누름 틀 (너트 셋)"),),
        tags=(BACK_TAG, FOOT_TAG, Tag((-(j.R_SLIDE + 3.0), j.RET_SCREW_Y, 0.0), "다리 없는 쪽")),
    ),
    Step(
        "front-screws",
        "앞줄 나사",
        "앞줄 벽 나사 조이기",
        do=(
            "누름 틀을 손가락으로 아래(베어링 쪽)로 누른 채 잡습니다.",
            "아래쪽 줄(앞줄)의 구멍 셋에 M3 × 8 나사를 하나씩 넣습니다.",
            "육각렌치로 조입니다. 머리가 벽에 닿아 더 돌지 않으면 멈춥니다.",
        ),
        warn=(
            "위쪽 줄(뒷줄)의 구멍 둘은 나중에 서보를 잠글 때 씁니다. 지금은 비워 둡니다.",
            "나사가 헛돌거나 안 들어가면 멈추고, 구멍으로 들여다봐 너트 구멍이 가운데에 있는지 봅니다.",
            "머리가 닿은 뒤로 세게 더 조이지 않습니다. 출력한 홈이 상할 수 있습니다.",
        ),
        check=(
            "나사 셋의 머리가 모두 벽에 닿았다.",
            "손목 몸통을 흔들어도 베어링이 달그락거리지 않는다.",
        ),
        adds=("벽 나사 1", "벽 나사 3", "벽 나사 5"),
        uses=(Use("hexkey"),),
        moves=(
            Move(("벽 나사 1",), (0.0, 0.0, 14.0)),
            Move(("벽 나사 5",), (14.0, 0.0, 0.0), 0.7),
            Move(("벽 나사 3",), (0.0, 0.0, -14.0), 1.4),
        ),
        view=View((0, -1, 0), (1, 0, 0), 16, 30),
        shell=(HOUSE,),
        ghost=True,
        labels=(
            Label("벽 나사 1", "M3 × 8"),
            Label("벽 나사 5", "M3 × 8"),
            Label("벽 나사 3", "M3 × 8"),
        ),
        tags=(BACK_TAG, Tag((0.0, BACK_ROW, j.R_DRUM + 7.0), "뒷줄: 지금은 비워 둠")),
    ),
    Step(
        "disc-nuts",
        "원판 너트",
        "출력 원판에 너트 넣기",
        do=(
            "출력 원판을 밑면(가운데가 둥글게 솟은 쪽)이 위로 오게 놓습니다.",
            "테두리 가까이에 있는 육각 홈 넷에 너트를 하나씩 올립니다.",
            "엄지로 눌러 끝까지 끼웁니다.",
        ),
        warn=("엄지로 안 들어가면 두드리지 않습니다. 사진을 보내 주세요.",),
        check=("너트 넷이 모두 홈 속에 들어가, 면보다 튀어나오지 않는다.",),
        adds=(DISC, DISC_NUTS),
        still=(DISC,),
        focus=(DISC, DISC_NUTS),
        moves=tuple(
            Move((f"{DISC_NUTS} {k}",), (0.0, -10.0, 0.0), 0.45 * (k - 1)) for k in range(1, 5)
        ),
        view=View((0, -1, 0), (1, 0, 0), 50, 20),
        labels=(Label(f"{DISC_NUTS} 1", "M3 너트 × 4"),),
        tags=(
            Tag((j.R_POST + 7.0, j.SEAM - 3.0, 0.0), "귀"),
            Tag((0.0, j.B0 - 6.0, 0.0), "밑면 (솟은 쪽)"),
        ),
    ),
    Step(
        "post",
        "멈춤 기둥",
        "멈춤 기둥 나사 조이기",
        do=(
            "출력 원판을 뒤집어 윗면(매끈한 면)이 위로 오게 놓습니다.",
            "귀의 구멍에 위에서 M3 × 10 나사를 꽂습니다.",
            "밑으로 나온 나사 끝에 너트를 손으로 돌려 끼웁니다.",
            "너트를 손가락으로 잡고 육각렌치로 나사를 조입니다.",
        ),
        warn=(
            "머리 자리에 실이 남아 있으면 머리가 끝까지 들어가지 않습니다. 1단계에서 걷어 내지 않았으면 먼저 걷어 "
            "냅니다.",
        ),
        check=(
            "나사 머리가 윗면의 홈 속에 들어가 튀어나오지 않는다.",
            "너트가 기둥 끝에 붙어, 손으로 돌려도 돌지 않는다.",
        ),
        adds=("멈춤 나사", "너트 (멈춤 기둥)"),
        uses=(Use("out_disc", note="6단계에서 너트를 넣은 것", count=False), Use("hexkey")),
        focus=DISC_SET,
        moves=(
            Move(("멈춤 나사",), (0.0, 16.0, 0.0)),
            Move(("너트 (멈춤 기둥)",), (0.0, -12.0, 0.0), 0.9),
        ),
        view=View((0, 1, 0), (1, 0, 0), 14, 38),
        labels=(Label("멈춤 나사", "M3 × 10"), Label("너트 (멈춤 기둥)", "M3 너트")),
        tags=(Tag((-8.0, j.ATOP + 3.0, 0.0), "윗면 (매끈한 면)"),),
    ),
    Step(
        "carrier-nuts",
        "받침 너트",
        "서보 받침에 너트 넣기",
        do=(
            "서보 받침을 넓은 판이 아래, 날개 둘이 위로 오게 놓습니다.",
            "날개마다 바깥에 돋움이 있고, 그 옆면(판이 터진 쪽)에 너트 홈 입구가 있습니다. 그림에서 화살표가 "
            "들어가는 곳입니다.",
            "너트를 동전 세우듯 세워서 입구에 넣고, 홈 끝에 닿을 때까지 밉니다. 돋움 둘에 하나씩 넣습니다.",
        ),
        warn=(
            "서보를 몸통에 넣은 뒤에는 이 너트에 손이 닿지 않습니다. 지금 끝까지 들어갔는지 꼭 봅니다.",
        ),
        check=("돋움 바깥 면의 구멍으로 들여다보면 너트 구멍이 가운데에 보인다 (둘 다).",),
        adds=CARRIER,
        still=("서보 받침",),
        focus=CARRIER,
        moves=(
            Move((CAR_NUTS + "@z+",), _slide("서보 받침 +z")),
            Move((CAR_NUTS + "@z-",), _slide("서보 받침 -z"), 0.8),
        ),
        view=View((0, 1, 0), (1, 0, 0), 30, 35),
        labels=(Label(CAR_NUTS + "@z+", "M3 너트 × 2"),),
        tags=(Tag((j.WIRE_GROOVE_X[0] - 3.0, _CAR_Y, 0.0), "판 (아래)"),),
    ),
    Step(
        "carrier-on-servo",
        "받침 달기",
        "서보에 서보 받침 달기",
        do=(
            f"ID {SERVO_ID} 이름표가 붙은 서보를 혼이 아래로 가게 엎어 놓습니다.",
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
        adds=("서보 몸통", "혼$", "서보 뒷면 나사"),
        still=("서보 몸통", "혼$"),
        uses=(
            Use("carrier", note="8단계에서 너트 둘을 넣은 것", count=False),
            Use("driver"),
            Use("washer_m3", 1, "예비: 판이 끝까지 안 조일 때"),
        ),
        focus=SERVO_SET,
        moves=(
            Move(CARRIER, (0.0, -25.0, 0.0)),
            Move(("서보 뒷면 나사",), (0.0, -12.0, 0.0), 1.4),
        ),
        view=View((0, -1, 0), (0, 0, 1), 35, 20),
        labels=(
            Label("서보 받침", "서보 받침 (너트 둘)"),
            Label("서보 뒷면 나사 1", "서보 뒷면 나사 × 4"),
        ),
    ),
    Step(
        "hub-plate",
        "허브 판",
        "허브 판 넣기",
        do=(
            "손목 몸통을 뒤쪽(열린 쪽)이 위로 오게 놓습니다.",
            "허브 판을 가운데가 볼록한 쪽이 위로 오게 들고, 열린 쪽으로 떨어뜨려 넣습니다.",
            "허브 판이 베어링 안쪽 링 위에 평평하게 얹혔는지 봅니다.",
        ),
        warn=("볼록한 쪽이 아래로 가면 서보의 혼이 제자리에 앉지 않습니다.",),
        check=("허브 판이 기울지 않고 베어링 안쪽 링 위에 얹혀 있다.",),
        adds=("허브 판",),
        moves=(Move(("허브 판",), (0.0, -45.0, 0.0)),),
        view=BACK_UP,
        shell=(HOUSE,),
        cut=True,
        labels=(Label("허브 판", "허브 판 (볼록한 쪽이 위)"),),
        tags=(BACK_TAG, FOOT_TAG),
    ),
    Step(
        "servo-in",
        "서보 넣기",
        "서보 넣기",
        do=(
            "9단계의 서보를 혼이 아래로 가게 듭니다.",
            "서보의 긴 쪽(혼에서 먼 끝)을 발 쪽으로 돌려, 열린 쪽으로 넣습니다. 서보 받침의 날개가 벽을 따라 "
            "미끄러져 들어갑니다.",
            "혼이 허브 판의 볼록한 곳에 닿을 때까지 내립니다.",
        ),
        warn=(
            "벽 나사는 아직 조이지 않습니다. 혼 볼트를 먼저 조여 서보가 제자리를 찾은 뒤에 잠가야 서보 축에 억지 "
            "힘이 걸리지 않습니다 (14단계).",
        ),
        check=(
            "벽의 뒷줄 구멍 둘(뒤쪽 열린 끝에 가까운 줄)로 서보 받침 돋움의 구멍이 보인다.",
            "서보가 기울지 않고 끝까지 들어갔다.",
        ),
        uses=(Use("servo_sts3215", note="9단계에서 서보 받침을 단 것", count=False),),
        mount=SERVO_SET,
        moves=(Move(SERVO_SET, (0.0, -70.0, 0.0)),),
        view=View((0, -1, 0), (1, 0, 0), 35, 28),
        shell=(HOUSE,),
        cut=True,
        labels=(Label("서보 몸통", "서보 (서보 받침을 단 것)"),),
        tags=(BACK_TAG, FOOT_TAG),
    ),
    Step(
        "disc-on",
        "원판 끼우기",
        "뒤집어 출력 원판 끼우기",
        do=(
            "서보 받침을 엄지로 눌러 서보가 빠지지 않게 잡은 채, 손목 몸통을 뒤집어 앞쪽이 위로 오게 합니다.",
            "베어링 가운데 구멍으로 허브 판의 구멍 넷이 보입니다. 그 밑으로 혼의 나사 구멍이 보이게, 허브 판을 "
            "손끝이나 육각렌치 끝으로 돌려 맞춥니다.",
            "출력 원판을 윗면(매끈한 면)이 위로, 귀가 발 반대쪽에 오게 듭니다. 가운데 볼록한 곳을 베어링 안쪽 "
            "링에 끼웁니다.",
            "원판을 조금씩 돌려, 원판의 구멍 넷으로 혼의 나사 구멍이 보이게 맞춥니다. 속이 어두우면 휴대폰 "
            "불빛을 비춥니다.",
        ),
        warn=(
            "뒤집는 동안 서보 받침에서 손을 떼면 서보와 허브 판이 빠집니다.",
            "귀를 발 쪽에 두면 멈춤 기둥이 목의 살에 걸려 원판이 들어가지 않습니다.",
        ),
        check=(
            "원판이 안쪽 링에 끝까지 들어가 기울지 않았다.",
            "원판의 구멍 넷으로 혼의 나사 구멍이 보인다.",
        ),
        uses=(Use("out_disc", note="너트와 멈춤 나사를 단 것", count=False),),
        mount=DISC_SET,
        moves=(Move(DISC_SET, (0.0, 40.0, 0.0)),),
        view=View((0, 1, 0), (1, 0, 0), 40, 28),
        labels=(Label(DISC, "출력 원판"),),
        tags=(FOOT_TAG, Tag((j.R_POST + 8.0, j.ATOP, 0.0), "귀는 발 반대쪽")),
    ),
    Step(
        "horn-bolts",
        "혼 볼트",
        "혼 볼트 조이기",
        do=(
            "M3 × 20 나사 4개를 원판의 구멍에 하나씩 넣고, 손으로 몇 바퀴씩 돌려 넷 다 걸어 둡니다.",
            "육각렌치로 대각선 차례로 조금씩 조입니다.",
            "머리가 원판에 닿으면 넷을 한 번 더 단단히 조입니다.",
        ),
        warn=(
            "서보 받침을 잡은 손은 넷이 다 걸릴 때까지 떼지 않습니다.",
            "한 개만 끝까지 먼저 조이면 원판이 기울어 붙습니다. 넷을 고르게 조입니다.",
        ),
        check=(
            "나사 머리 넷이 원판 윗면에 닿았다.",
            "출력 원판이 위아래로도 옆으로도 흔들리지 않는다.",
        ),
        adds=("혼 볼트",),
        uses=(Use("hexkey"),),
        moves=tuple(
            Move((f"혼 볼트 M3x20 {k}",), (0.0, 22.0, 0.0), 0.45 * (k - 1)) for k in range(1, 5)
        ),
        view=View((0, 1, 0), (1, 0, 0), 58, 20),
        labels=(Label("혼 볼트 M3x20 1", "M3 × 20 × 4"),),
    ),
    Step(
        "lock-servo",
        "서보 잠금",
        "뒷줄 벽 나사로 서보 잠그기",
        do=(
            "벽의 뒷줄(뒤쪽 열린 끝에 가까운 줄) 구멍 둘을 찾습니다. 구멍은 위아래로 조금 깁니다.",
            "M3 × 8 나사를 하나씩 넣고 육각렌치로 조입니다. 머리가 벽에 닿아 더 돌지 않으면 멈춥니다.",
        ),
        warn=(
            "서보를 밀거나 당기지 않고 지금 자리 그대로 잠급니다.",
            "머리가 닿은 뒤로 세게 더 조이지 않습니다.",
        ),
        check=("나사 둘의 머리가 벽에 닿았다.", "서보를 뒤에서 밀어도 움직이지 않는다."),
        adds=("벽 나사 2", "벽 나사 4"),
        uses=(Use("hexkey"),),
        moves=(
            Move(("벽 나사 2",), (0.0, 0.0, 14.0)),
            Move(("벽 나사 4",), (0.0, 0.0, -14.0), 0.7),
        ),
        view=View((0, 1, 0), (1, 0, 0), 18, 40),
        labels=(Label("벽 나사 2", "M3 × 8"), Label("벽 나사 4", "M3 × 8")),
        tags=(Tag((0.0, BACK_ROW, j.R_DRUM + 8.0), "뒷줄"),),
    ),
    Step(
        "cable",
        "서보 선",
        "서보 선 꽂기",
        do=(
            "손목 몸통을 뒤쪽(열린 쪽)이 위로 오게 놓습니다.",
            "서보 선(20 cm)의 한쪽 플러그를 서보 뒷면의 커넥터 둘 가운데 아무 쪽에나 꽂습니다 (①). 서보 받침 "
            "판의 터진 쪽으로 플러그 옆면을 잡고 끝까지 밉니다.",
            "선을 서보 받침 판 위에 눕혀 발 쪽(서보 먼 끝)으로 보냅니다 (②).",
            "판 끝에서 선을 앞쪽(출력 원판 쪽)으로 꺾어 서보 먼 끝의 검은 면을 따라 내리고 (③), 앞벽 가운데 "
            "홈으로 뺍니다. 남는 선은 발 쪽으로 늘어뜨립니다.",
        ),
        warn=(
            "선이 비틀리거나 꺾이지 않게 눕힙니다. 뒷덮개를 달면 선은 판 위 좁은 틈(약 2 mm)을 지나갑니다.",
            "선을 열린 쪽 가장자리나 옆벽 위로 빼지 않습니다. 그러면 뒷덮개가 선 위에 떠서 들뜹니다.",
        ),
        check=(
            "플러그가 끝까지 꽂혀 살짝 당겨도 빠지지 않는다.",
            "선이 서보 먼 끝을 따라 내려와 앞벽 가운데 홈으로 나와 있다.",
        ),
        stuck="그림은 한쪽 커넥터에 꽂은 모습입니다. 다른 쪽에 꽂아도 길은 같습니다. 헷갈리면 사진을 보내 주세요.",
        adds=("서보 선",),
        uses=(Use("servo_cable", 1),),
        view=View((0, -1, 0), (-1, 0, 1.1), 22, 0),
        shell=(HOUSE,),
        ghost=True,
        labels=(
            Label("서보 선 플러그", "① 플러그", (-14.2, _WY[0] - 8.0, _ZC)),
            Label("서보 선$", "② 판 위에 눕혀", (-31.0, _WY[0], 1.5)),
            Label("서보 선$", "③ 먼 끝 면 따라 앞쪽으로", (_XD, 10.0, 0.0)),
        ),
        tags=(EXIT_TAG,),
    ),
    Step(
        "cover",
        "뒷덮개",
        "뒷덮개 달기",
        do=(
            "뒷덮개를 테가 몸통 쪽으로 가게 들고, 원통과 목 모양에 맞춰 열린 쪽에 얹습니다.",
            "M3 × 12 나사 2개를 덮개의 구멍에 넣고 육각렌치로 조입니다.",
        ),
        warn=("선이 덮개 테 밑에 끼이지 않게 봅니다.",),
        check=(
            "덮개가 들뜨지 않고 원통 뒤 끝을 덮었다.",
            "나사 둘의 머리가 덮개의 머리 자리에 들어갔다.",
        ),
        adds=("뒷덮개", "덮개 나사"),
        still=(),
        uses=(Use("hexkey"),),
        moves=(
            Move(("뒷덮개",), (0.0, -30.0, 0.0)),
            Move(("덮개 나사",), (0.0, -16.0, 0.0), 1.0),
        ),
        view=View((0, -1, 0), (1, 0, 0), 40, 28),
        labels=(Label("뒷덮개", "뒷덮개"), Label("덮개 나사 M3x12 1", "M3 × 12 × 2")),
    ),
    Step(
        "child",
        "자식 몸통",
        "자식 몸통 달기",
        do=(
            "손목 몸통을 앞쪽(출력 원판)이 위로 오게 놓습니다.",
            "남은 손목 몸통을 발이 아래로 가게 들고, 원통 앞 끝이 출력 원판의 귀 쪽을 보게 돌립니다.",
            "발을 출력 원판 위에 얹고, 발 양옆 귀의 구멍 넷을 원판의 구멍 넷에 맞춥니다.",
            "M3 × 10 나사 4개를 위에서 넣고 육각렌치로 조입니다.",
        ),
        warn=(
            "원통 앞 끝을 귀 반대쪽으로 돌려 달면 영점 핀이 들어가지 않습니다 (19단계). 귀 쪽인지 다시 봅니다.",
        ),
        check=("발이 원판 위에 들뜨지 않고 앉았다.", "나사 넷의 머리가 귀에 닿았다."),
        adds=(CHILD, "몸통 나사"),
        uses=(Use("hexkey"),),
        moves=(
            Move((CHILD,), (0.0, 45.0, 0.0)),
            Move(("몸통 나사",), (0.0, 14.0, 0.0), 1.2),
        ),
        view=View((0, 1, 0), (1, 0, 0), 22, 30),
        labels=(
            Label(CHILD, "손목 몸통 (원통 앞 끝이 귀 쪽)"),
            Label("몸통 나사 M3x10 1", "M3 × 10 × 4"),
        ),
        tags=(EAR_TAG,),
    ),
    Step(
        "turn",
        "돌려 보기",
        "끝까지 돌려 보기",
        do=(
            "서보 선은 앞벽 가운데 홈에서 나온 채 발 쪽으로 늘어뜨립니다. 선을 출력 원판 둘레에 감아 돌려 보는 "
            '일은 다음 안내 "서보 선 감아 돌려 보기"에서 합니다.',
            "한 손으로 아래 몸통(집)을 책상에 눌러 잡고, 다른 손으로 위 몸통(자식)을 천천히 한쪽 끝까지 돌립니다.",
            "반대쪽 끝까지 천천히 돌립니다.",
        ),
        warn=(
            "천천히 돌립니다. 서보는 기어비가 커서 손으로 돌리면 뻑뻑한 것이 정상이고, 빨리 돌리면 기어에 무리가 "
            "갈 수 있습니다.",
            "끝에서 멈춤 기둥이 살에 닿으면 더 밀지 않습니다. 걸리면 억지로 돌리지 말고 사진을 보내 주세요.",
        ),
        check=(
            "양쪽 끝까지 걸리는 곳 없이 돈다.",
            "양쪽 끝에서 멈춤 기둥이 살에 닿아 멈춘다.",
        ),
        uses=(Use("servo_cable", note="홈에서 나온 채 발 쪽으로 늘어뜨린다", count=False),),
        turn=(-j.STOP_DEG, j.STOP_DEG),
        view=View((0, 1, 0), (1, 0, 0), 30, 35),
    ),
    Step(
        "pin",
        "영점 핀",
        "영점 핀 맞춰 보기",
        do=(
            "자식 몸통을 돌려, 자식의 핀 귀(구멍 난 귀)가 집 목의 살 위에 오게 합니다. 가운데에서 90° 돌린 "
            "자리입니다.",
            "영점 핀을 자식의 귀 구멍으로 넣어 살의 구멍까지 밉니다.",
            "끝까지 들어가는지 본 뒤 핀을 뺍니다. 영점을 잴 때 다시 씁니다. 손으로 안 빠지면 펜치로 나온 끝을 "
            "잡고, 핀을 제자리에서 살짝 비틀며 곧게 당깁니다.",
        ),
        warn=(
            "핀이 안 들어가면 억지로 밀지 않습니다. 사진을 보내 주세요.",
            "핀이 꽂힌 채로 관절을 돌리지 않습니다.",
        ),
        check=("핀이 두 구멍을 지나 걸림 없이 들어간다.",),
        adds=("영점 핀",),
        moves=(Move(("영점 핀",), (0.0, 24.0, 0.0)),),
        joint=wrist.PIN_POSE_DEG,
        view=View((0, 1, 0), (-1, 0, 0), 25, 25),
        labels=(Label("영점 핀", "영점 핀"),),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "첫 관절 조립이 끝났습니다. 그림은 다 조립한 첫 관절입니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요. 안내를 고칩니다.",
            "다음 할 일은 대화창에서 이어서 정합니다.",
        ),
        hide=("영점 핀",),
        view=View((0, 1, 0), (0.5, 0, 1), 18, 18),
    ),
)

GUIDE = Guide(
    id="first-joint",
    kind="assembly",
    title="첫 관절 조립",
    sub=f"J5 · 손목 몸통 둘 · STS3215 ID {SERVO_ID}",
    scene=scene,
    steps=STEPS,
    items={**printed_items(wrist.LABELS, wrist.PRINT_SPEC), SERVO_ITEM.key: SERVO_ITEM},
    bundle="first-joint",
)
