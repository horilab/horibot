"""J1 베이스 조립: 베이스 몸통과 베어링 둘, 돌림판, J1 서보(ID 1), J1 몸통, 책상에 무는 판, 책상에 물리기 (0029의 J1 관절).

순서는 0029의 J1 조립 순서를 형상으로 다시 따진 것이다 (2026-11-24):
- 돌림판의 멈춤 나사 머리는 J1 몸통의 앞 귀가 덮는다. 그래서 멈춤 나사는 돌림판을 J1 몸통에 달기 전에 넣는다.
- 조임 볼트 위로는 J1 몸통 바닥판에서 78 mm가 비지만, 어깨 서보가 들어가면 6 mm만 남는다. 조임 볼트는 어깨 서보
  전에 조인다 (어깨 서보는 다음 안내).
- 선반 너트 둘은 J1 몸통을 어깨 원통 앞 끝으로 세웠을 때(뽑힌 자세)만 떨어뜨려 넣을 수 있다. 베이스에 세우면 홈이
  눕는다. 그래서 J1 몸통 너트와 같이 넣고, 선반을 바로 달아 너트를 잡는다.
- 어깨 베어링(6808)은 원통 뒤 끝에서 64 mm 안이라 미는 도구로 넣는다 (shoulder.bearing_pusher). J1 몸통이 베이스에
  오르면 축이 눕으므로, 뽑힌 자세로 세운 지금 위에서 누른다. 누름 틀과 앞줄 벽 나사 셋까지 여기서 한다.
- J1 서보의 선 둘(어댑터로 가는 300 mm는 오른쪽 커넥터, 어깨로 가는 900 mm는 왼쪽 커넥터, 0031)은 서보를 넣기 전에
  꽂는다. 판을 달면 커넥터에 손이 닿지 않는다.

볼트 머리 (0029, 가진 것은 bom.md와 hardware.md): 유두는 베이스 벽 나사 다섯(M3 × 8, 몸통 옆 굴 속에 앉아 머리가
들어간다)과 조임 볼트 넷(M3 × 45, 이 길이는 유두만 있다)이다. 나머지는 둥근머리다: 허브 혼 나사(유두는 머리가 돌림판
저널 끝에 닿는다), J1 몸통 나사(돌림판 밑면의 1.8 mm 자리), 멈춤 나사(J1 몸통 앞 귀 밑), 판 나사(와셔와 함께 판
밑면의 2.2 mm 자리에 든다), 어깨 벽 나사, 선반 나사.

장면 좌표는 J1의 서보 좌표다 (parts/base.py: x 로봇 앞, y 위, z 로봇 오른쪽). J1 몸통 쪽 부품은 J1과 같이 돈다.
"""

from __future__ import annotations

from dataclasses import replace

from build123d import Box, Compound, Plane, Pos, Vector

from horibot_design.guide.catalog import ITEMS, printed_items
from horibot_design.guide.guides import first_joint as fj
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
from horibot_design.parts import base as b
from horibot_design.parts import joint as j
from horibot_design.parts import rest_shelf as rs
from horibot_design.parts import shoulder as sh
from horibot_design.parts.shapes import box, cyl_x, cyl_z, solid
from horibot_design.values import ServoModel

MODEL: ServoModel = "STS3215"
DY = j.servo_dy(MODEL)
SERVO_ID = 1  # 서보 ID는 관절 번호다 (servo-setup.md)

# ------------------------------------------------------------------------------------------------
# 부품 목록


def _cap(length: int) -> Item:
    return Item(
        f"m3x{length}_cap",
        f"M3 × {length} 유두",
        "hardware",
        "유두 렌치볼트 (머리가 높은 원통). 2.5 mm 렌치",
        draw={"t": "cap", "L": length},
    )


def _colored(labels: dict[str, str], color: str) -> dict[str, Item]:
    """출력 부품 목록 (뽑는 부품 표는 형상을 만드는 함수라서, 색만 적은 표로 만든다)."""
    return printed_items(labels, {k: (None, None, color) for k in labels})


_THUMBS = {
    "base_core": View((0, 1, 0), (1, 0, 0.6), 30, 0),
    "base_turntable": View((0, 1, 0), (1, 0, 0.6), 40, 0),
    "base_hub": View((0, 1, 0), (1, 0, 0.6), 45, 0),
    "base_spacer": View((0, 1, 0), (1, 0, 0.6), 30, 0),
    "base_retainer": View((0, -1, 0), (1, 0, 0), 28, 34),
    "base_plate": View((0, 1, 0), (1, 0, 0.4), 45, 0),
    "base_bearing_pusher": View((0, 1, 0), (1, 0, 0.6), 20, 0),
    "j1_body": View((0, 1, 0), (1, 0, 1), 15, 0),
    "rest_shelf": View((0, 1, 0), (1, 0, 0.8), 20, 0),
    "shoulder_retainer": View((0, 0, 1), (1, 0, 0), 28, 34),
    "shoulder_bearing_pusher": View((0, 0, 1), (1, 0, 0.6), 20, 0),
}
_BASE_BLACK = {k: v for k, v in b.LABELS.items() if k not in ("j1_body", "j1_cover")}
PRINTED: dict[str, Item] = {
    k: replace(it, thumb=_THUMBS.get(k, it.thumb))
    for k, it in {
        **_colored(_BASE_BLACK, "black"),
        **_colored({"j1_body": b.LABELS["j1_body"]}, "white"),
        **_colored({"rest_shelf": rs.LABELS["rest_shelf"]}, "white"),
        **_colored(
            {k: sh.LABELS[k] for k in ("shoulder_retainer", "shoulder_bearing_pusher")}, "black"
        ),
        **_colored({"carrier": "서보 받침"}, "black"),
    }.items()
}
SERVO_ITEM = Item(
    "servo_sts3215_j1",
    f"서보 STS3215 · ID {SERVO_ID}",
    "bought",
    f"혼이 달린 것. ID {SERVO_ID} 이름표가 붙은 서보 (J1)",
    thumb=ITEMS["servo_sts3215"].thumb,
)
OTHER: dict[str, Item] = {
    it.key: it
    for it in (
        SERVO_ITEM,
        Item(
            "bearing_6808",
            "베어링 6808",
            "bought",
            "바깥 지름 52 · 안지름 40 · 두께 7 mm (어깨, 큰 것)",
            thumb=View((0, 0, 1), (1, 0, 0.3), 40, 0),
        ),
        Item("servo_cable_300", "서보 선 300 mm", "bought", "J1 서보 → 서보 어댑터", icon="wire"),
        Item("servo_cable_900", "서보 선 900 mm", "bought", "J1 서보 → 어깨 서보(J2)", icon="wire"),
        _cap(8),
        _cap(45),
        Item(
            "hexkey_25",
            "육각렌치 2.5 mm",
            "tool",
            "유두 볼트용 ㄱ자. 짧은 쪽이 16 mm 넘는 것",
            icon="hex",
        ),
        Item("pencil", "샤프", "tool", "뻑뻑한 너트를 끝으로 살살 밀 때", icon="pen"),
        Item("ruler", "자", "tool", "아래 베어링을 넣은 깊이를 볼 때", icon="ruler"),
        Item("metal_clamp", "금속 클램프", "tool", "가진 것 둘. 책상 뒤에서 판의 날개를 문다"),
        Item("desk", "작업 책상", "tool", "긴 변 55 cm, 상판 두께 4 cm"),
    )
}
ITEMS_HERE: dict[str, Item] = {**PRINTED, **OTHER}

# ------------------------------------------------------------------------------------------------
# 장면

_W, _T = j.WIRE_W, j.WIRE_T
_Y_WIRE = (j.WIRE_TOP + DY, j.WIRE_TOP + DY + _T)  # 받침 판 밑에 눕힌 선의 두께 범위
_Z_PLUG = 4.955  # 커넥터에 꽂은 플러그의 가운데 |z| (parts/joint.py의 servo())
_Z_NOTCH = 2.6  # 몸통 뒤 아래의 선 구멍(반폭 6)을 지날 때 두 선의 가운데 |z|
_X_TAIL = -(b.R_CORE + 28.0)  # 몸통 밖으로 나온 선 끝


def _flat(a: tuple[float, float], c: tuple[float, float]) -> Compound:
    """받침 판 밑에 눕힌 선 한 토막: (x, z) a에서 c까지, 두께는 y 방향. 이음매를 덮게 양 끝을 늘린다."""
    va, vc = Vector(a[0], 0.0, a[1]), Vector(c[0], 0.0, c[1])
    d = vc - va
    x = d.normalized()
    mid = (va + vc) * 0.5 + Vector(0.0, sum(_Y_WIRE) / 2, 0.0)
    plane = Plane(origin=mid, x_dir=x, z_dir=x.cross(Vector(0.0, 1.0, 0.0)))
    return solid(plane * Box(d.length + _W, _T, _W))


def cable(side: int) -> tuple[Compound, Compound]:
    """J1 서보의 선 하나: (꽂은 플러그, 선). side +1은 오른쪽 커넥터(어댑터로), -1은 왼쪽(어깨로).

    선은 받침 판 밑으로 서보 먼 끝(뒤) 쪽으로 눕고, 몸통 뒤 아래의 선 구멍을 지나며 가운데로 모여 뒤로 나온다.
    """
    z0, z1 = sorted((side * 0.45, side * 9.46))
    plug = box(-16.6, -11.85, j.PLUG_TOP + DY, j.L1 + 0.5 + DY, z0, z1)
    zp, zn = side * _Z_PLUG, side * _Z_NOTCH
    wire = _flat((-16.6, zp), (-29.0, zp))
    wire = solid(wire + _flat((-29.0, zp), (-35.0, zn)))
    wire = solid(wire + _flat((-35.0, zn), (_X_TAIL, zn)))
    return solid(plug), wire


def _cap_wall_screws() -> list[tuple[str, Compound]]:
    """베이스 벽 나사 다섯을 유두 머리로 (parts/joint.py의 wall_screws와 같은 자리와 차례)."""
    a, c = j.SPOT_R - j.SCREW_LEN, j.SPOT_R
    hr, hh = b.CLAMP_HEAD_R, b.CLAMP_HEAD_H
    out: list[tuple[str, Compound]] = []
    for s, y in ((1, j.RET_SCREW_Y), (1, j.CAR_SCREW_Y), (-1, j.RET_SCREW_Y), (-1, j.CAR_SCREW_Y)):
        if s > 0:
            shape = cyl_z(1.5, a, c, 0.0, y) + cyl_z(hr, c, c + hh, 0.0, y)
        else:
            shape = cyl_z(1.5, -c, -a, 0.0, y) + cyl_z(hr, -c - hh, -c, 0.0, y)
        out.append((f"벽 나사 {len(out) + 1}", solid(shape)))
    shape = cyl_x(1.5, a, c, j.RET_SCREW_Y, 0.0) + cyl_x(hr, c, c + hh, j.RET_SCREW_Y, 0.0)
    out.append(("벽 나사 5", solid(shape)))
    return out


# 장면 부품 이름(설계 코드) → 부품 목록 키. 앞에서부터 이름의 앞부분이 맞는 것을 쓴다. 세 번째는 개수에 넣는지
PART_KEYS: tuple[tuple[str, str, bool], ...] = (
    ("베이스 몸통", "base_core", True),
    ("책상에 무는 판", "base_plate", True),
    ("베어링 6806", "bearing_6806", True),
    ("베어링 미는 도구 (위)", "base_bearing_pusher", True),
    ("베어링 미는 도구 (아래)", "base_bearing_pusher", False),
    ("사이 고리", "base_spacer", True),
    ("허브", "base_hub", True),
    ("돌림판", "base_turntable", True),
    ("누름 틀", "base_retainer", True),
    ("서보 받침", "carrier", True),
    ("서보 몸통", SERVO_ITEM.key, True),
    ("서보 뒷면 나사", "selftap_so101", True),
    ("혼 나사", "m3x8", True),
    ("혼", SERVO_ITEM.key, False),  # 서보에 딸린 혼
    ("J1 몸통 나사", "m3x12", True),
    ("J1 몸통", "j1_body", True),
    ("조임 볼트", "m3x45_cap", True),
    ("와셔", "washer_m3", True),
    ("너트", "nut_m3", True),
    ("멈춤 나사", "m3x10", True),
    ("판 나사", "m3x10", True),
    ("벽 나사", "m3x8_cap", True),
    ("어깨 베어링 미는 도구", "shoulder_bearing_pusher", True),
    ("어깨 베어링", "bearing_6808", True),
    ("어깨 누름 틀", "shoulder_retainer", True),
    ("어깨 너트", "nut_m3", True),
    ("어깨 벽 나사", "m3x8", True),
    ("쉬는 자세 선반", "rest_shelf", True),
    ("선반 나사", "m3x12", True),
    ("책상", "desk", False),
    ("클램프", "metal_clamp", True),
    ("선 플러그 (어댑터)", "servo_cable_300", False),
    ("선 플러그 (어깨)", "servo_cable_900", False),
    ("어댑터로 가는 선", "servo_cable_300", True),
    ("어깨로 가는 선", "servo_cable_900", True),
)
# 설계 코드의 부품 가운데 장면에 두지 않는 것: 꽂은 플러그 둘과 눕힌 선의 자리(선은 cable()로 따로 그린다), 영점 핀
# (영점은 다음 안내), 둥근머리로 그린 벽 나사(유두로 다시 그린다)
_SKIP = ("플러그", "선 자리", "영점 핀", "벽 나사")


def _key(name: str) -> tuple[str, bool]:
    return next((k, c) for prefix, k, c in PART_KEYS if name.startswith(prefix))


def _part(name: str, shape: Compound, color: str, moving: bool) -> ScenePart:
    key, count = _key(name)
    return ScenePart(name, shape, key, color, moving, count)


def scene() -> list[ScenePart]:
    out: list[ScenePart] = []
    for p in b.own_parts(MODEL) + b.hardware(MODEL):
        if p.name.startswith(_SKIP):
            continue
        out.append(_part(p.name, p.shape, p.color, p.moving))
    out += [_part(n, s, "metal", False) for n, s in _cap_wall_screws()]
    out.append(_part("J1 몸통", b.as_child(b.j1_body()), "body", True))
    # 어깨 베어링과 누름 틀, 그 너트와 앞줄 벽 나사 (어깨 좌표로 그린 것을 옮긴다)
    out.append(_part("어깨 베어링 6808", b.as_child(sh.bearing()), "metal", True))
    out.append(_part("어깨 누름 틀", b.as_child(sh.retainer()), "frame", True))
    hw = sh.hardware()
    nuts = [p for p in hw if p.name == "너트 (누름 틀)"]
    # 어깨의 +z → 로봇 뒤, -z → 앞, +x → 위
    for p, where in zip(nuts, ("뒤", "앞", "위"), strict=True):
        out.append(_part(f"어깨 너트 (누름 틀 {where})", b.as_child(p.shape), "metal", True))
    for p in hw:
        if p.name.startswith("벽 나사 (누름 틀)"):
            k = p.name.rsplit(" ", 1)[1]
            out.append(_part(f"어깨 벽 나사 {k}", b.as_child(p.shape), "metal", True))
    for p in rs.parts():
        out.append(_part(p.name, p.shape, p.color, True))
    # 베어링 미는 도구: 위 베어링과 아래 베어링을 밀 때의 자리 (밀린 끝이 베어링 밑면에 닿은 자리)
    pusher = b.bearing_pusher()
    out.append(_part("베어링 미는 도구 (위)", solid(Pos(0, b.B1, 0) * pusher), "frame", False))
    out.append(_part("베어링 미는 도구 (아래)", solid(Pos(0, b.B0, 0) * pusher), "frame", False))
    sh_push = solid(Pos(0, sh.B0, 0) * sh.bearing_pusher())
    out.append(_part("어깨 베어링 미는 도구", b.as_child(sh_push), "frame", True))
    for side, plug_name, wire_name in (
        (1, "선 플러그 (어댑터)", "어댑터로 가는 선"),
        (-1, "선 플러그 (어깨)", "어깨로 가는 선"),
    ):
        plug, wire = cable(side)
        out.append(_part(plug_name, plug, "plug", False))
        out.append(_part(wire_name, wire, "wire", False))
    for p in b.desk_parts():
        out.append(_part(p.name, p.shape, "frame" if p.name == "책상" else "metal", False))
    return out


# ------------------------------------------------------------------------------------------------
# 이름 (선택자는 이름의 앞부분이다)

BODY = "베이스 몸통"
PLATE = "책상에 무는 판"
BRG_UP, BRG_LO = "베어링 6806 (위)", "베어링 6806 (아래)"
PUSH_UP, PUSH_LO = "베어링 미는 도구 (위)", "베어링 미는 도구 (아래)"
SPACER = "사이 고리"
HUB = "허브"
TABLE = "돌림판"
RET = "누름 틀"
RET_NUTS = "너트 (누름 틀)"
CARRIER = "서보 받침"
CAR_NUTS = "너트 (서보 받침)"
SERVO = "서보 몸통"
HORN = "혼$"
BACK_SCREWS = "서보 뒷면 나사"
SERVO_SET = (SERVO, HORN, CARRIER, CAR_NUTS, BACK_SCREWS)
HUB_NUTS = "너트 (허브)"
HORN_SCREWS = "혼 나사"
HUB_SET = (HUB, HUB_NUTS, HORN_SCREWS)
CABLES = ("선 플러그", "어댑터로 가는 선", "어깨로 가는 선")
CLAMP_BOLTS = "조임 볼트"
CLAMP_WASHERS = "와셔 (조임 볼트)"
J1 = "J1 몸통$"
FOOT_NUTS = "너트 (J1 몸통)"
FOOT_SCREWS = "J1 몸통 나사"
STOP = ("멈춤 나사", "너트 (멈춤 기둥)")
SH_BRG = "어깨 베어링 6808"
SH_PUSH = "어깨 베어링 미는 도구"
SH_RET = "어깨 누름 틀"
SH_NUTS = "어깨 너트"
SH_SCREWS = "어깨 벽 나사"
SHELF = "쉬는 자세 선반"
SHELF_SCREWS = "선반 나사"
SHELF_NUTS = "너트 (선반)"
PLATE_SCREWS = "판 나사"
PLATE_WASHERS = "와셔 (판 나사)"
PLATE_NUTS = "너트 (판)"
# J1과 같이 도는 묶음 (J1 몸통 쪽): 베이스에 끼울 때 같이 내려온다
J1_SIDE = (
    J1,
    FOOT_NUTS,
    SHELF_NUTS,
    SH_BRG,
    SH_RET,
    SH_NUTS,
    SH_SCREWS,
    SHELF,
    SHELF_SCREWS,
    *STOP,
    TABLE,
    FOOT_SCREWS,
)
J1_LYING = (J1, FOOT_NUTS, SHELF_NUTS)  # 뽑힌 자세로 세운 J1 몸통과 그 속 너트

# 손에 드는 방향 (장면: x 로봇 앞, y 위, z 로봇 오른쪽)
UPRIGHT = View((0, 1, 0), (1, 0, 0), 22, 30)
UPSIDE = View((0, -1, 0), (1, 0, 0), 25, 30)  # 베이스 몸통을 뒤집어 윗면이 책상에 닿은 자세
FROM_RIGHT = View((0, 1, 0), (0, 0, 1), 25, 25)  # J1 몸통의 열린 뒤(로봇 오른쪽)가 보는 사람 쪽
# J1 몸통을 어깨 원통의 앞 끝(로봇 왼쪽)이 책상에 닿게 세운 자세 (뽑힌 자세): 장면의 +z가 위다
J1_UP = View((0, 0, 1), (1, 0, 0), 30, 30)
# 같은 자세에서 발 바닥(돌림판이 붙는 면)이 보는 사람 쪽
J1_UP_FOOT = View((0, 0, 1), (0, -1, 0), 35, 25)

_Y_BOTTOM = b.Y_FLOOR
BOTTOM_TAG = Tag((-(b.R_CORE + 8.0), _Y_BOTTOM, 0.0), "뒤 (네모난 선 구멍)")
J1_BACK = Tag((0.0, b.ATOP + 70.0, 46.0), "열린 뒤 (로봇 오른쪽)")
SHOULDER_FRONT = Tag((0.0, b.ATOP + 65.0, -42.0), "어깨 원통 앞 끝 (책상 쪽)")
PHOTO = "막히면 그 자리를 사진으로 보내 주세요."


def _slide_sh(name: str, reach: float = 13.0) -> tuple[float, float, float]:
    """어깨 누름 틀의 옆 홈에 너트를 미는 움직임 (어깨 좌표의 입구 방향을 장면 좌표로: x → y, y → -z, z → -x)."""
    _n, _c, _a, s = next(slot for slot in sh.side_nut_slots() if slot[0] == name)
    return (-s[2] * reach, s[0] * reach, -s[1] * reach)


def _radial(angle_deg: float, reach: float = 12.0) -> tuple[float, float, float]:
    x, z = j.polar(1.0, angle_deg)
    return (x * reach, 0.0, z * reach)


_CLAMP_SEAT = b.ATOP - b.SINK  # 조임 볼트 머리 자리의 바닥 (와셔가 앉는 면)

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "목록의 부품, 나사, 공구를 모두 꺼내 책상에 늘어놓습니다.",
            "나사는 머리 모양과 길이별로 따로 모읍니다. 이 안내에는 머리가 높은 원통인 유두(M3 × 8, × 45)와 머리가 "
            "낮고 둥근 둥근머리(M3 × 8, × 10, × 12)가 같이 나옵니다. 길이가 같아도 머리가 다르면 다른 자리에 씁니다.",
            "J1 몸통(흰색)과 쉬는 자세 선반(흰색), 어깨 베어링 6808(큰 것)과 6806 둘(작은 것), 미는 도구 둘(작은 통과 큰 "
            "통)을 같이 둡니다.",
            "길이나 머리를 모르는 나사가 섞여 있으면 자 옆에 놓고 찍어 대화창에 보내 주세요.",
        ),
        check=(
            "출력 부품: 검정 베이스 몸통, 돌림판, 허브, 사이 고리, 누름 틀, 어깨 누름 틀, 서보 받침, 책상에 무는 판, "
            "미는 도구 둘. 흰 J1 몸통과 선반",
            f"베어링 6806 둘, 6808 하나, 서보(ID {SERVO_ID}), 서보 선 300 mm와 900 mm",
            "나사, 너트, 와셔를 개수대로 (와셔 8개)",
            "공구 (육각렌치 2 mm와 2.5 mm, 금속 클램프 둘)",
        ),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    Step(
        "pierce",
        "막 뚫기",
        "나사 구멍의 막 뚫기",
        do=(
            "베이스 몸통의 밑면(열린 쪽)에서, 바깥 가까이 대각선 자리의 작은 구멍 넷 ①을 찾습니다.",
            "돌림판의 윗면(넓은 테)에서, 가운데 둘레의 깊은 구멍 넷 ②와 앞쪽 테의 작은 머리 자리 ③을 찾습니다.",
            "허브의 윗면(작은 머리 자리 넷이 파인 면)에서, 그 사이의 구멍 넷 ④를 찾습니다.",
            "구멍마다 바닥에 얇은 막이 있습니다. 육각렌치의 짧은 끝을 대고 꾹 눌러 뚫습니다. 렌치 끝이 구멍으로 쑥 "
            "들어가면 됩니다.",
        ),
        warn=("막만 뚫습니다. 구멍 가장자리를 깎아 넓히지 않습니다.",),
        check=("①, ②, ③, ④의 막 열세 곳을 모두 뚫어, 구멍이 반대편까지 보인다.",),
        stuck="막이 안 보이거나 이미 뚫려 있으면 그대로 두고 넘어갑니다. 헷갈리면 사진을 보내 주세요.",
        adds=(BODY, TABLE, HUB),
        still=(BODY, TABLE, HUB),
        uses=(Use("hexkey"),),
        focus=(BODY, TABLE, HUB),
        place=(
            Place(BODY, (-120.0, 0.0, 0.0), (0, -1, 0), (1, 0, 0)),
            Place(TABLE, (0.0, 0.0, 0.0), (0, 1, 0), (1, 0, 0)),
            Place(HUB, (85.0, 0.0, 0.0), (0, 1, 0), (1, 0, 0)),
        ),
        spots=(
            Spot(BODY, "①", tuple((x, b.Y_FLOOR, z) for x, z in b.plate_xz())),
            Spot(TABLE, "②", tuple((x, _CLAMP_SEAT, z) for x, z in b.clamp_xz())),
            Spot(TABLE, "③", ((b.R_POST, b.ATOP - j.POST_HEAD_SINK, 0.0),)),
            Spot(HUB, "④", tuple((x, b.B0, z) for x, z in b.clamp_xz())),
        ),
        labels=(
            Label(BODY, "베이스 몸통 · 밑면", (0.0, b.Y_FLOOR, -b.R_CORE)),
            Label(TABLE, "돌림판 · 윗면", (0.0, b.ATOP, -b.R_TABLE)),
            Label(HUB, "허브 · 윗면", (0.0, b.B0, -j.R_STEP)),
        ),
        view=View((0, 1, 0), (0, 0, 1), 55, 0),
    ),
    Step(
        "bearing-up",
        "위 베어링",
        "위 베어링 넣기",
        do=(
            "베이스 몸통을 뒤집어 윗면(턱이 있는 둥근 면)을 책상에 댑니다. 열린 밑면이 위를 봅니다.",
            "6806 하나를 열린 쪽으로 넣습니다. 조금 들어가다 걸립니다 (아래 베어링 자리, 6.6 mm).",
            "작은 미는 도구(베어링 미는 도구)를 손잡이 테가 위로 가게 들고, 통 끝을 베어링에 댑니다. 통 벽이 베어링의 "
            "가장 바깥 쇠 테에만 닿는지 봅니다.",
            "손잡이 테를 꾹 눌러 밉니다. 걸린 곳을 지나면 쑥 내려가다가, 맨 끝에서 다시 걸립니다 (위 베어링 자리). "
            "다시 꾹 눌러 손잡이 테가 몸통 밑면에 거의 닿을 때까지 밉니다.",
        ),
        warn=(
            "바깥 쇠 테만 밉니다. 가운데 쪽(안쪽 링, 얇은 덮개)을 누르면 베어링 속의 알이 상할 수 있습니다.",
            "두드리거나 바이스로 누르지 않습니다. 손으로 안 들어가면 사진을 보내 주세요.",
        ),
        check=(
            "손잡이 테가 몸통 밑면에 거의 닿았다 (종이 한두 장 틈).",
            "도구를 빼고, 베어링 안쪽 링을 손가락이나 연필로 돌리면 걸림 없이 돈다.",
        ),
        adds=(BRG_UP, PUSH_UP),
        mount=(BODY, BRG_UP),
        focus=(BODY, BRG_UP, PUSH_UP),
        moves=(Move((BRG_UP, PUSH_UP), (0.0, -95.0, 0.0)),),
        view=UPSIDE,
        shell=(BODY,),
        cut=True,
        labels=(Label(BRG_UP, "베어링 6806"), Label(PUSH_UP, "베어링 미는 도구")),
        tags=(BOTTOM_TAG,),
    ),
    Step(
        "spacer",
        "사이 고리",
        "사이 고리 넣기",
        do=(
            "몸통을 뒤집은 채로 둡니다.",
            "사이 고리를 열린 쪽으로 떨어뜨려 넣습니다. 위 베어링의 안쪽 링 위에 얹힙니다. 위아래가 같아서 아무 쪽이나 "
            "됩니다.",
        ),
        check=("사이 고리가 기울지 않고 안쪽 링 위에 얹혔다.",),
        adds=(SPACER,),
        moves=(Move((SPACER,), (0.0, -80.0, 0.0)),),
        view=UPSIDE,
        shell=(BODY,),
        cut=True,
        labels=(Label(SPACER, "사이 고리"),),
    ),
    Step(
        "bearing-low",
        "아래 베어링",
        "아래 베어링 넣기",
        do=(
            "남은 6806을 열린 쪽으로 넣습니다. 바로 걸립니다 (아래 베어링 자리).",
            "미는 도구로 바깥 쇠 테를 밀어, 더 들어가지 않을 때까지 넣습니다. 베어링이 사이 고리에 닿으면 멈춥니다.",
            "자로 손잡이 테와 몸통 밑면 사이를 잽니다. 30 mm쯤이면 제자리입니다.",
        ),
        warn=(
            "멈춘 뒤로 세게 더 누르지 않습니다. 그때부터는 힘이 베어링 속의 알로 갑니다.",
            "30 mm보다 많이 크면(35 mm 넘게) 사이 고리가 기울었을 수 있습니다. 사진을 보내 주세요.",
        ),
        check=(
            "더 밀어도 들어가지 않는다.",
            "손잡이 테와 몸통 밑면 사이가 30 mm쯤이다.",
        ),
        adds=(BRG_LO, PUSH_LO),
        mount=(BRG_LO,),
        focus=(BODY, BRG_UP, SPACER, BRG_LO, PUSH_LO),
        moves=(Move((BRG_LO, PUSH_LO), (0.0, -75.0, 0.0)),),
        uses=(Use("base_bearing_pusher", note="2단계에서 쓴 것", count=False), Use("ruler")),
        view=UPSIDE,
        shell=(BODY,),
        cut=True,
        labels=(Label(BRG_LO, "베어링 6806"), Label(PUSH_LO, "테와 밑면 사이 30 mm쯤")),
    ),
    Step(
        "ret-nuts",
        "틀 너트",
        "누름 틀에 너트 넣기",
        do=(
            "누름 틀(검정, 긴 고리에 다리 셋)을 고리가 아래, 다리 셋이 위로 오게 놓습니다.",
            "다리마다 옆면에 너트 홈의 입구가 있습니다. 그림에서 화살표가 들어가는 곳입니다.",
            "너트를 동전 세우듯 세워서 입구에 넣고, 홈 끝에 닿을 때까지 밉니다. 다리 셋에 하나씩 넣습니다.",
        ),
        warn=("손가락으로 안 되면 샤프 끝이나 육각렌치의 긴 쪽으로 밀어 넣습니다.",),
        check=("다리 바깥 면의 구멍으로 들여다보면 너트 구멍이 가운데에 보인다 (다리 셋 모두).",),
        adds=(RET, RET_NUTS),
        still=(RET,),
        focus=(RET, RET_NUTS),
        uses=(Use("pencil"),),
        moves=(
            Move((RET_NUTS + "@z+",), fj._slide("누름 틀 +z")),
            Move((RET_NUTS + "@x+",), fj._slide("누름 틀 +x"), 0.8),
            Move((RET_NUTS + "@z-",), fj._slide("누름 틀 -z"), 1.6),
        ),
        view=View((0, -1, 0), (1, 0, 0), 26, 36),
        labels=(Label(RET_NUTS + "@x+", "M3 너트 × 3"),),
        tags=(Tag((-(j.R_SLIDE + 3.0), j.RET_SCREW_Y, 0.0), "다리 없는 쪽"),),
    ),
    Step(
        "ret-in",
        "틀 넣기",
        "누름 틀 넣기",
        do=(
            "베이스 몸통을 뒤집은 채로 둡니다.",
            "누름 틀을 고리가 아래로 가게 들고, 다리가 없는 쪽을 몸통 밑면의 네모난 선 구멍 쪽(뒤)으로 돌립니다.",
            "열린 쪽에 넣어, 고리가 아래 베어링에 닿을 때까지 내립니다.",
        ),
        warn=(
            "다리 셋이 몸통 옆과 앞의 네모난 굴 셋과 맞아야 다음 단계의 나사가 너트에 닿습니다.",
        ),
        check=(
            "네모난 굴 셋(양옆과 앞, 밑면에서 먼 줄)으로 들여다보면 너트 구멍이 가운데에 보인다.",
        ),
        uses=(Use("base_retainer", note="너트 셋을 넣은 것", count=False),),
        mount=(RET, RET_NUTS),
        moves=(Move((RET, RET_NUTS), (0.0, -60.0, 0.0)),),
        view=UPSIDE,
        shell=(BODY,),
        cut=True,
        labels=(Label(RET, "누름 틀 (너트 셋)"),),
        tags=(BOTTOM_TAG,),
    ),
    Step(
        "ret-screws",
        "틀 나사",
        "누름 틀 잠그기 (유두 셋)",
        do=(
            "누름 틀의 고리를 손가락으로 베어링 쪽으로 누른 채 잡습니다.",
            "몸통 옆과 앞의 네모난 굴 가운데, 밑면(지금 위)에서 먼 줄의 셋에 M3 × 8 유두를 하나씩 넣습니다.",
            "2.5 mm 렌치로 조입니다. 머리가 굴 바닥에 닿아 더 돌지 않으면 멈춥니다.",
        ),
        warn=(
            "밑면 가까운 줄(양옆의 위아래로 조금 긴 구멍 둘)은 나중에 서보를 잠글 때 씁니다. 지금은 비워 둡니다.",
            "나사가 헛돌면 멈추고, 굴로 들여다봐 너트 구멍이 가운데에 있는지 봅니다.",
        ),
        check=(
            "유두 셋의 머리가 굴 바닥에 닿았다.",
            "몸통을 흔들어도 베어링이 달그락거리지 않는다.",
        ),
        adds=("벽 나사 1", "벽 나사 3", "벽 나사 5"),
        uses=(Use("hexkey_25"),),
        moves=(
            Move(("벽 나사 1",), (0.0, 0.0, 16.0)),
            Move(("벽 나사 5",), (16.0, 0.0, 0.0), 0.7),
            Move(("벽 나사 3",), (0.0, 0.0, -16.0), 1.4),
        ),
        view=View((0, -1, 0), (1, 0, 0.4), 18, 30),
        shell=(BODY,),
        ghost=True,
        labels=(
            Label("벽 나사 1", "M3 × 8 유두"),
            Label("벽 나사 5", "M3 × 8 유두"),
            Label("벽 나사 3", "M3 × 8 유두"),
        ),
    ),
    Step(
        "foot-nuts",
        "몸통 너트",
        "J1 몸통 바닥판에 너트 넣기",
        do=(
            "J1 몸통(흰색)을 어깨 원통의 앞 끝(안쪽에 턱이 있어 구멍이 조금 좁은 쪽, 위팔이 달릴 쪽)이 책상에 닿게 "
            "세웁니다. 뽑혀 나온 자세입니다. 목 아래 끝의 두꺼운 바닥판이 옆을 봅니다.",
            "바닥판의 위쪽 가장자리에 너트 구멍 둘, 바닥판 가운데 큰 구멍 속 벽에 너트 구멍 둘이 위로 열려 있습니다.",
            "너트를 하나씩 떨어뜨려 넣습니다. 핀셋이나 손가락으로 구멍 위에 맞춰 놓으면 쏙 들어갑니다.",
        ),
        warn=(
            "이 너트는 다음에 돌림판을 달 때까지 위로 열려 있습니다. J1 몸통을 뒤집지 않습니다.",
        ),
        check=("바닥판 바닥의 나사 구멍 넷으로 들여다보면 너트 구멍이 보인다.",),
        stuck=PHOTO,
        adds=(J1, FOOT_NUTS),
        still=(J1,),
        mount=(J1, FOOT_NUTS),
        focus=(J1, FOOT_NUTS),
        uses=(Use("tweezers"),),
        moves=tuple(
            Move((f"{FOOT_NUTS} {k}",), (0.0, 0.0, 30.0), 0.5 * (k - 1)) for k in range(1, 5)
        ),
        view=J1_UP_FOOT,
        labels=(Label(f"{FOOT_NUTS} 1", "M3 너트 × 4"),),
        tags=(SHOULDER_FRONT,),
    ),
    Step(
        "shelf-nuts",
        "선반 너트",
        "선반 너트 넣기",
        do=(
            "J1 몸통을 그대로 세워 둡니다. 위로 열린 쪽(로봇 오른쪽이 될 뒤)으로 속을 들여다봅니다.",
            "목의 앞벽 안쪽(바닥판에서 가까운 쪽)에 좁은 세로 홈 둘이 위로 열려 있습니다.",
            "너트를 하나씩 떨어뜨려 넣습니다. 홈을 따라 3.5 cm쯤 내려가 바닥의 칸에 들어갑니다.",
        ),
        warn=("너트가 홈 중간에 걸리면 샤프 끝으로 살살 밀어 내립니다.",),
        check=("두 홈 모두 너트가 바닥까지 내려갔다 (흔들어도 달그락 소리가 홈 끝에서 난다).",),
        stuck=PHOTO,
        adds=(SHELF_NUTS,),
        mount=(SHELF_NUTS,),
        focus=J1_LYING,
        uses=(Use("pencil", note="걸리면 밀어 내린다", count=False),),
        moves=(
            Move((f"{SHELF_NUTS} 1",), (0.0, 0.0, 40.0)),
            Move((f"{SHELF_NUTS} 2",), (0.0, 0.0, 40.0), 0.8),
        ),
        view=View((0, 0, 1), (1, 0, 0.5), 45, 20),
        ghost=True,
        shell=(J1,),
        labels=(Label(f"{SHELF_NUTS} 1", "M3 너트 × 2"),),
    ),
    Step(
        "sh-bearing",
        "어깨 베어링",
        "어깨 베어링 넣기",
        do=(
            "J1 몸통을 그대로 세워 둡니다. 어깨 원통이 위로 열려 있습니다.",
            "6808(큰 베어링)을 원통에 넣습니다. 아래 끝 가까이까지 저절로 내려가다 걸립니다.",
            "큰 미는 도구(어깨 베어링을 미는 도구)를 손잡이 테가 위로 가게 들고 통 끝을 베어링에 댑니다. 통 벽이 "
            "베어링의 가장 바깥 쇠 테에만 닿는지 봅니다.",
            "손잡이 테를 꾹 눌러, 테가 원통 끝에 거의 닿을 때까지 밉니다. 마지막 6.6 mm는 눌러야 들어갑니다.",
        ),
        warn=(
            "통 벽이 가운데 쪽 얇은 덮개에 걸치면 밀지 않습니다. 사진을 보내 주세요 (도구는 바깥 링 두께를 어림해 "
            "그렸습니다).",
            "두드리거나 바이스로 누르지 않습니다.",
        ),
        check=(
            "손잡이 테가 원통 끝에 거의 닿았다 (종이 한두 장 틈).",
            "도구를 빼고, 안쪽 링을 손가락으로 돌리면 걸림 없이 돈다.",
        ),
        stuck=PHOTO,
        adds=(SH_BRG, SH_PUSH),
        mount=(SH_BRG,),
        focus=(*J1_LYING, SH_BRG, SH_PUSH),
        moves=(Move((SH_BRG, SH_PUSH), (0.0, 0.0, 95.0)),),
        view=J1_UP,
        shell=(J1,),
        cut=True,
        labels=(Label(SH_BRG, "베어링 6808"), Label(SH_PUSH, "어깨 베어링을 미는 도구")),
    ),
    Step(
        "sh-ret-nuts",
        "어깨 틀 너트",
        "어깨 누름 틀에 너트 넣기",
        do=(
            "어깨 누름 틀(검정, 넓은 고리에 다리 셋)을 고리가 아래, 다리 셋이 위로 오게 놓습니다.",
            "다리마다 옆면의 너트 홈 입구에 너트를 세워 넣고, 홈 끝까지 밉니다. 다리 셋에 하나씩 넣습니다.",
        ),
        warn=("손가락으로 안 되면 샤프 끝으로 밀어 넣습니다.",),
        check=("다리 바깥 면의 구멍으로 들여다보면 너트 구멍이 가운데에 보인다 (다리 셋 모두).",),
        adds=(SH_RET, SH_NUTS),
        still=(SH_RET,),
        focus=(SH_RET, SH_NUTS),
        moves=(
            Move(("어깨 너트 (누름 틀 뒤)",), _slide_sh("누름 틀 +z")),
            Move(("어깨 너트 (누름 틀 위)",), _slide_sh("누름 틀 +x"), 0.8),
            Move(("어깨 너트 (누름 틀 앞)",), _slide_sh("누름 틀 -z"), 1.6),
        ),
        view=View((0, 0, 1), (1, 0, 0), 26, 36),
        labels=(Label("어깨 너트 (누름 틀 위)", "M3 너트 × 3"),),
    ),
    Step(
        "sh-ret-in",
        "어깨 틀 넣기",
        "어깨 누름 틀 넣기",
        do=(
            "J1 몸통을 그대로 세워 둡니다.",
            "어깨 누름 틀을 고리가 아래로 가게 들고, 다리가 없는 쪽을 목(바닥판) 쪽으로 돌립니다.",
            "원통에 넣어, 고리가 베어링에 닿을 때까지 내립니다.",
        ),
        warn=("다리 셋이 원통 벽의 나사 구멍 셋(위와 양옆, 베어링에 가까운 줄)과 맞아야 합니다.",),
        check=("베어링에 가까운 줄의 벽 구멍 셋으로 들여다보면 너트 구멍이 가운데에 보인다.",),
        uses=(Use("shoulder_retainer", note="너트 셋을 넣은 것", count=False),),
        mount=(SH_RET, SH_NUTS),
        focus=(*J1_LYING, SH_BRG, SH_RET, SH_NUTS),
        moves=(Move((SH_RET, SH_NUTS), (0.0, 0.0, 60.0)),),
        view=J1_UP,
        shell=(J1,),
        cut=True,
        labels=(Label(SH_RET, "어깨 누름 틀 (너트 셋)"),),
    ),
    Step(
        "sh-screws",
        "어깨 틀 나사",
        "어깨 누름 틀 잠그기 (둥근머리 셋)",
        do=(
            "어깨 누름 틀의 고리를 손가락으로 베어링 쪽으로 누른 채 잡습니다.",
            "베어링에 가까운 줄의 벽 구멍 셋에 M3 × 8 둥근머리를 하나씩 넣습니다.",
            "2 mm 렌치로 조입니다. 머리가 벽에 닿아 더 돌지 않으면 멈춥니다.",
        ),
        warn=(
            "원통 벽의 다른 줄(목에서 먼 쪽 셋)은 다음 안내에서 어깨 서보를 잠글 때 씁니다. 지금은 비워 둡니다.",
            "머리가 닿은 뒤로 세게 더 조이지 않습니다.",
        ),
        check=(
            "둥근머리 셋의 머리가 벽에 닿았다.",
            "J1 몸통을 흔들어도 베어링이 달그락거리지 않는다.",
        ),
        adds=(SH_SCREWS,),
        uses=(Use("hexkey"),),
        focus=(*J1_LYING, SH_BRG, SH_RET, SH_NUTS, SH_SCREWS),
        mount=(SH_SCREWS,),
        moves=(
            Move(("어깨 벽 나사 1",), (-16.0, 0.0, 0.0)),
            Move(("어깨 벽 나사 3",), (0.0, 16.0, 0.0), 0.7),
            Move(("어깨 벽 나사 2",), (16.0, 0.0, 0.0), 1.4),
        ),
        view=J1_UP,
        shell=(J1,),
        ghost=True,
        labels=(
            Label("어깨 벽 나사 1", "M3 × 8 둥근머리"),
            Label("어깨 벽 나사 3", "M3 × 8 둥근머리"),
            Label("어깨 벽 나사 2", "M3 × 8 둥근머리"),
        ),
    ),
    Step(
        "shelf",
        "선반",
        "쉬는 자세 선반 달기",
        do=(
            "J1 몸통을 바로 세웁니다: 바닥판(목 아래 끝)이 책상에 닿고 어깨 원통이 위입니다.",
            "선반(흰색)의 뿌리 판을 앞 귀(바닥판 앞으로 나온 둥근 판) 윗면에 얹고 목의 앞벽에 붙입니다. 받침이 "
            "앞을 봅니다.",
            "M3 × 12 둥근머리 둘을 선반 뿌리 판의 구멍으로 넣어, 9단계에서 넣은 너트에 2 mm 렌치로 조입니다.",
        ),
        warn=(
            "나사가 너트에 안 닿으면 너트가 홈 바닥까지 안 내려간 것입니다. 선반을 떼고 사진을 보내 주세요.",
        ),
        check=("선반이 앞 귀 위에 들뜨지 않고 앉았다.", "선반을 흔들어도 움직이지 않는다."),
        adds=(SHELF, SHELF_SCREWS),
        uses=(Use("hexkey"),),
        focus=(*J1_LYING, SH_BRG, SH_RET, SH_NUTS, SH_SCREWS, SHELF, SHELF_SCREWS),
        mount=(SHELF, SHELF_SCREWS),
        moves=(Move((SHELF,), (40.0, 0.0, 0.0)), Move((SHELF_SCREWS,), (25.0, 0.0, 0.0), 1.0)),
        view=View((0, 1, 0), (1, 0, 0.6), 20, 0),
        labels=(
            Label(SHELF, "쉬는 자세 선반"),
            Label(f"{SHELF_SCREWS} M3x12 1", "M3 × 12 둥근머리 × 2"),
        ),
    ),
    Step(
        "stop-screw",
        "멈춤 나사",
        "돌림판의 멈춤 나사 조이기",
        do=(
            "돌림판을 윗면(넓은 테)이 위로 오게 놓습니다.",
            "테 가장자리 한 곳의 작은 머리 자리(1단계 ③)에 M3 × 10 둥근머리를 위에서 꽂습니다. 밑면의 짧은 기둥으로 "
            "나옵니다.",
            "기둥 끝으로 나온 나사 끝에 너트를 손으로 돌려 끼우고, 너트를 손가락으로 잡은 채 2 mm 렌치로 조입니다.",
        ),
        warn=("돌림판을 J1 몸통에 단 뒤에는 이 머리를 J1 몸통이 덮습니다. 꼭 지금 조입니다.",),
        check=(
            "나사 머리가 테 윗면보다 들어가 있다.",
            "너트가 기둥 끝에 붙어, 손으로 돌려도 돌지 않는다.",
        ),
        adds=STOP,
        uses=(Use("hexkey"),),
        focus=(TABLE, *STOP),
        mount=STOP,
        moves=(
            Move(("멈춤 나사",), (0.0, 18.0, 0.0)),
            Move(("너트 (멈춤 기둥)",), (0.0, -14.0, 0.0), 0.9),
        ),
        view=View((0, 1, 0), (1, 0, 0.5), 20, 0),
        labels=(Label("멈춤 나사", "M3 × 10 둥근머리"), Label("너트 (멈춤 기둥)", "M3 너트")),
    ),
    Step(
        "table-on",
        "돌림판 달기",
        "J1 몸통에 돌림판 달기",
        do=(
            "J1 몸통을 다시 어깨 원통의 앞 끝이 책상에 닿게 세웁니다 (8단계의 자세). 바닥판 바닥이 옆을 봅니다.",
            "돌림판의 넓은 테 윗면을 바닥판 바닥에 대고, 돌림판의 구멍 넷을 바닥판의 나사 구멍 넷에 맞춥니다. 한 방향으로만 "
            "다 맞습니다. 그때 멈춤 기둥이 선반 쪽(앞 귀 밑)에 옵니다.",
            "돌림판 밑면 쪽에서 M3 × 12 둥근머리 넷을 넣어, 바닥판 속 너트에 2 mm 렌치로 조입니다.",
        ),
        warn=(
            "둥근머리를 씁니다. 머리가 돌림판 밑면의 얕은 자리(1.8 mm) 속에 들어가야, 돌아가는 돌림판과 베이스 몸통 "
            "윗면 사이의 틈(2 mm)이 그대로 남습니다.",
        ),
        check=(
            "돌림판이 바닥판에 들뜨지 않고 붙었다.",
            "나사 머리 넷이 돌림판 밑면보다 들어가 있다.",
            "멈춤 기둥이 선반 쪽에 있다.",
        ),
        adds=(FOOT_SCREWS,),
        uses=(Use("base_turntable", note="멈춤 나사를 단 것", count=False), Use("hexkey")),
        focus=(
            *J1_LYING,
            SH_BRG,
            SH_RET,
            SH_NUTS,
            SH_SCREWS,
            SHELF,
            SHELF_SCREWS,
            TABLE,
            *STOP,
            FOOT_SCREWS,
        ),
        mount=(TABLE, FOOT_SCREWS),
        moves=(
            Move((TABLE, *STOP), (0.0, -30.0, 0.0)),
            Move((FOOT_SCREWS,), (0.0, -22.0, 0.0), 1.0),
        ),
        view=J1_UP_FOOT,
        labels=(Label(TABLE, "돌림판"), Label(f"{FOOT_SCREWS} M3x12 1", "M3 × 12 둥근머리 × 4")),
    ),
    Step(
        "join",
        "베이스에 끼우기",
        "J1 몸통을 베이스 몸통에 끼우기",
        do=(
            "베이스 몸통을 바로 세워 열린 밑면을 책상에 댑니다.",
            "J1 몸통을 바로 세워 들고, 선반이 베이스 몸통의 앞(밑면의 네모난 선 구멍 반대쪽)을 보게 돌립니다.",
            "돌림판 가운데의 긴 원통을 위 베어링 구멍에 넣고 내립니다. 사이 고리와 아래 베어링까지 지나며 조금 뻑뻑합니다.",
            "멈춤 기둥이 몸통 윗면의 둥근 홈에 들어가, 돌림판 테가 몸통 윗면 바로 위(2 mm)에 올 때까지 꾹 누릅니다.",
        ),
        warn=(
            "기둥이 홈 뒤쪽의 막힌 살 위에 오면 끝까지 안 들어갑니다. 억지로 누르지 말고 J1 몸통을 앞으로 돌려 맞춥니다.",
            "누를 때 J1 몸통의 원통이 아니라 바닥판 가까이를 누릅니다.",
        ),
        check=(
            "돌림판 테와 몸통 윗면 사이가 빙 둘러 고르다.",
            "J1 몸통을 손으로 천천히 돌리면 양쪽 끝에서 기둥이 살에 닿아 멈춘다.",
        ),
        uses=(Use("j1_body", note="돌림판과 선반을 단 것", count=False),),
        moves=(Move(J1_SIDE, (0.0, 90.0, 0.0)),),
        view=UPRIGHT,
        shell=(BODY,),
        cut=True,
        tags=(BOTTOM_TAG,),
    ),
    Step(
        "car-nuts",
        "받침 너트",
        "서보 받침에 너트 넣기",
        do=(
            "서보 받침(검정)을 넓은 판이 아래, 날개 둘이 위로 오게 놓습니다.",
            "날개마다 바깥 돋움의 옆면(판이 터진 쪽)에 너트 홈 입구가 있습니다.",
            "너트를 세워서 입구에 넣고 홈 끝에 닿을 때까지 밉니다. 돋움 둘에 하나씩 넣습니다.",
        ),
        warn=("서보를 넣은 뒤에는 이 너트에 손이 닿지 않습니다. 끝까지 들어갔는지 꼭 봅니다.",),
        check=("돋움 바깥 면의 구멍으로 들여다보면 너트 구멍이 가운데에 보인다 (둘 다).",),
        adds=(CARRIER, CAR_NUTS),
        still=(CARRIER,),
        focus=(CARRIER, CAR_NUTS),
        moves=(
            Move((CAR_NUTS + "@z+",), fj._slide("서보 받침 +z")),
            Move((CAR_NUTS + "@z-",), fj._slide("서보 받침 -z"), 0.8),
        ),
        view=View((0, 1, 0), (1, 0, 0), 30, 35),
        labels=(Label(CAR_NUTS + "@z+", "M3 너트 × 2"),),
    ),
    Step(
        "car-on",
        "받침 달기",
        "서보에 서보 받침 달기",
        do=(
            f"ID {SERVO_ID} 이름표가 붙은 서보를 혼이 아래로 가게 엎어 놓습니다.",
            "서보 받침을 날개가 아래(혼 쪽)로 가게 들고 판을 서보 뒷면에 얹습니다. 판의 작은 구멍 넷이 서보 뒷면의 나사 "
            "구멍 넷과 맞아야 합니다.",
            "서보 뒷면 나사 4개를 작은 십자 드라이버로 조입니다.",
        ),
        warn=("자가탭 나사는 세게 조이지 않습니다. 서보 구멍의 나사산이 뭉개질 수 있습니다.",),
        check=("판이 서보 뒷면에 붙어 흔들리지 않는다.",),
        adds=(SERVO, HORN, BACK_SCREWS),
        still=(SERVO, HORN),
        uses=(Use("carrier", note="너트 둘을 넣은 것", count=False), Use("driver")),
        focus=SERVO_SET,
        moves=(
            Move((CARRIER, CAR_NUTS), (0.0, -25.0, 0.0)),
            Move((BACK_SCREWS,), (0.0, -12.0, 0.0), 1.4),
        ),
        view=View((0, -1, 0), (0, 0, 1), 35, 20),
        labels=(
            Label(SERVO, f"서보 ID {SERVO_ID}"),
            Label(f"{BACK_SCREWS} 1", "서보 뒷면 나사 × 4"),
        ),
    ),
    Step(
        "hub-nuts",
        "허브 너트",
        "허브에 너트 넣기",
        do=(
            "허브(검정, 두꺼운 원판)를 윗면(1단계에서 막을 뚫은 면)이 위로 오게 놓습니다.",
            "옆면을 빙 둘러 너트 홈 입구 넷이 있습니다. 너트를 눕혀 하나씩 밀어 넣어 끝까지 보냅니다.",
        ),
        warn=(
            "이 너트는 허브를 혼에 단 뒤에도 옆에서 보이지만, 서보를 넣은 뒤에는 손이 닿지 않습니다.",
        ),
        check=("윗면의 큰 구멍 넷으로 들여다보면 너트 구멍이 가운데에 보인다.",),
        adds=(HUB_NUTS,),
        focus=(HUB, HUB_NUTS),
        moves=tuple(
            Move((f"{HUB_NUTS} {k + 1}",), _radial(a), 0.5 * k)
            for k, a in enumerate((0, 90, 180, 270))
        ),
        uses=(Use("pencil", note="뻑뻑하면", count=False),),
        view=View((0, 1, 0), (1, 0, 0.5), 40, 0),
        labels=(Label(f"{HUB_NUTS} 1", "M3 너트 × 4"),),
    ),
    Step(
        "hub-on",
        "허브 달기",
        "혼에 허브 달기",
        do=(
            "서보를 혼이 위로 오게 세웁니다.",
            "허브를 평평한 면(가운데에 둥근 홈이 있는 면)이 혼 쪽으로 가게 혼 위에 얹습니다. 허브의 작은 구멍 넷이 혼의 "
            "나사 구멍 넷과 맞게 돌립니다.",
            "M3 × 8 둥근머리 넷을 넣어 2 mm 렌치로 대각선 차례로 조입니다.",
        ),
        warn=(
            "둥근머리를 씁니다. 유두는 머리가 높아 나중에 돌림판의 원통 끝에 닿습니다.",
            "혼의 나사산이 얇습니다(2.5 mm). 머리가 닿은 뒤로 세게 더 조이지 않습니다.",
        ),
        check=("허브가 혼 위에 들뜨지 않고 앉았다.", "나사 머리 넷이 허브 윗면보다 들어가 있다."),
        adds=(HORN_SCREWS,),
        uses=(Use("hexkey"),),
        focus=(*SERVO_SET, *HUB_SET),
        moves=(
            Move((HUB, HUB_NUTS), (0.0, 25.0, 0.0)),
            Move((HORN_SCREWS,), (0.0, 16.0, 0.0), 1.2),
        ),
        view=View((0, 1, 0), (1, 0, 0.5), 35, 0),
        labels=(
            Label(HUB, "허브 (평평한 면이 혼 쪽)"),
            Label(f"{HORN_SCREWS} M3x8 1", "M3 × 8 둥근머리 × 4"),
        ),
    ),
    Step(
        "cables",
        "선 꽂기",
        "서보 선 둘 꽂기",
        do=(
            "서보를 혼이 아래로 가게 엎어 놓고, 서보의 긴 쪽(혼에서 먼 끝)이 내 쪽을 보게 돌립니다. 뒷면의 커넥터 둘이 "
            "보입니다.",
            "그림의 ①(엎어 놓은 지금 내 왼쪽, 서보를 넣으면 로봇 오른쪽)에 300 mm 선을, ②에 900 mm 선을 끝까지 "
            "꽂습니다.",
            "두 선을 서보 받침 판 위로 눕혀 먼 끝 쪽으로 보냅니다.",
        ),
        warn=(
            "베이스 판을 달면 이 커넥터에 손이 닿지 않습니다. 끝까지 꽂혔는지 살짝 당겨 봅니다.",
            "두 선의 반대쪽 끝은 그대로 둡니다. 300 mm는 서보 어댑터에, 900 mm는 어깨 서보(J2)에 다음 안내에서 꽂습니다.",
        ),
        check=(
            "두 플러그가 끝까지 꽂혀 살짝 당겨도 빠지지 않는다.",
            "두 선이 먼 끝 쪽으로 눕혀 있다.",
        ),
        adds=CABLES,
        focus=(*SERVO_SET, *HUB_SET, *CABLES),
        view=View((0, -1, 0), (-1, 0, 0.3), 40, 0),
        zoom=1.6,
        center=(-25.0, -18.0, 0.0),
        labels=(
            Label("선 플러그 (어댑터)", "① 300 mm (어댑터로)"),
            Label("선 플러그 (어깨)", "② 900 mm (어깨로)"),
        ),
    ),
    Step(
        "servo-in",
        "서보 넣기",
        "서보 넣기",
        do=(
            "베이스 몸통을 두 손으로 잡아 J1 몸통째 들어 올립니다. J1 몸통이 아니라 베이스 몸통을 잡습니다.",
            "서보를 허브가 위로, 먼 끝(긴 쪽)이 몸통 밑면의 네모난 선 구멍 쪽(뒤)으로 가게 들고 열린 밑면으로 밀어 "
            "넣습니다. 서보 받침의 날개가 벽을 따라 미끄러집니다.",
            "두 선은 네모난 선 구멍으로 빼냅니다.",
            "끝까지 넣고 베이스 몸통을 책상에 내려놓습니다. 서보는 책상 위에 얹혀 있습니다.",
        ),
        warn=(
            "밑면 가까운 줄의 벽 나사(서보 잠금)는 아직 조이지 않습니다. 조임 볼트가 서보 자리를 잡은 뒤에 잠급니다.",
        ),
        check=(
            "두 선이 네모난 선 구멍으로 나와 있고, 서보 밑에 깔리지 않았다.",
            "몸통 양옆의 낮은 줄 굴로 서보 받침 돋움의 구멍이 보인다.",
        ),
        stuck=PHOTO,
        uses=(Use(SERVO_ITEM.key, note="받침, 허브, 선을 단 것", count=False),),
        mount=(*SERVO_SET, *HUB_SET, *CABLES),
        moves=(Move((*SERVO_SET, *HUB_SET, *CABLES), (0.0, -75.0, 0.0)),),
        view=View((0, 1, 0), (1, 0, 0.6), 15, 0),
        shell=(BODY,),
        cut=True,
        labels=(Label(SERVO, f"서보 ID {SERVO_ID}"),),
        tags=(BOTTOM_TAG,),
    ),
    Step(
        "clamp-bolts",
        "조임 볼트",
        "조임 볼트 넷 조이기 (유두)",
        do=(
            "M3 × 45 유두 넷에 와셔를 하나씩 끼웁니다.",
            "J1 몸통의 열린 뒤(로봇 오른쪽)로 들여다보면, 바닥판 가운데 큰 구멍 밑에 돌림판의 깊은 구멍 넷이 보입니다. "
            "볼트를 하나씩 세워 떨어뜨립니다.",
            "볼트가 끝까지 안 내려가면 허브의 구멍과 어긋난 것입니다. J1 몸통을 천천히 돌리면 넷이 툭 내려가는 자리가 "
            "있습니다.",
            "2.5 mm ㄱ자 렌치의 짧은 쪽을 볼트 머리에 꽂고 긴 쪽을 열린 뒤로 내밀어 돌립니다. 넷 다 몇 바퀴씩 걸어 둔 "
            "뒤, 대각선 차례로 조금씩 조입니다.",
        ),
        warn=(
            "유두 볼트입니다 (이 길이는 유두만 있습니다). 와셔를 빼먹지 않습니다.",
            "한 개만 끝까지 먼저 조이지 않습니다. 조이면 서보가 책상에서 조금 들립니다.",
        ),
        check=(
            "머리 넷이 와셔를 눌러 앉았다.",
            "J1 몸통을 돌리면 서보와 같이 뻑뻑하게 돈다 (기어 때문에 정상).",
        ),
        stuck=PHOTO,
        adds=(CLAMP_BOLTS, CLAMP_WASHERS),
        uses=(Use("hexkey_25"),),
        moves=(Move((CLAMP_BOLTS, CLAMP_WASHERS), (0.0, 50.0, 0.0)),),
        view=FROM_RIGHT,
        shell=(J1, BODY),
        ghost=True,
        labels=(Label(f"{CLAMP_BOLTS} M3x45 1", "M3 × 45 유두 × 4 (와셔)"),),
        tags=(J1_BACK,),
    ),
    Step(
        "lock",
        "서보 잠금",
        "서보 잠그기 (유두 둘)",
        do=(
            "몸통 양옆, 밑면 가까운 줄의 네모난 굴 둘(구멍이 위아래로 조금 깁니다)에 M3 × 8 유두를 하나씩 넣습니다.",
            "2.5 mm 렌치로 조입니다. 머리가 굴 바닥에 닿아 더 돌지 않으면 멈춥니다.",
        ),
        warn=("서보를 밀거나 당기지 않고 지금 자리 그대로 잠급니다.",),
        check=("유두 둘의 머리가 굴 바닥에 닿았다.",),
        adds=("벽 나사 2", "벽 나사 4"),
        uses=(Use("hexkey_25"),),
        moves=(
            Move(("벽 나사 2",), (0.0, 0.0, 16.0)),
            Move(("벽 나사 4",), (0.0, 0.0, -16.0), 0.7),
        ),
        view=View((0, 1, 0), (0.3, 0, 1), 10, 0),
        shell=(BODY,),
        ghost=True,
        labels=(Label("벽 나사 2", "M3 × 8 유두"), Label("벽 나사 4", "M3 × 8 유두")),
    ),
    Step(
        "turn",
        "돌려 보기",
        "J1을 끝까지 돌려 보기",
        do=(
            "한 손으로 베이스 몸통을 책상에 눌러 잡고, 다른 손으로 J1 몸통의 바닥판 가까이를 잡아 천천히 한쪽 끝까지 "
            "돌립니다.",
            "반대쪽 끝까지 천천히 돌린 뒤, 선반이 앞을 보게 돌려 둡니다.",
        ),
        warn=(
            "천천히 돌립니다. 서보는 기어비가 커서 손으로 돌리면 뻑뻑한 것이 정상입니다.",
            "끝에서 기둥이 살에 닿으면 더 밀지 않습니다.",
        ),
        check=(
            "양쪽 끝까지 걸리거나 긁히는 곳 없이 돈다.",
            "밑면으로 나온 선 둘은 같이 돌지 않는다.",
        ),
        uses=(Use("j1_body", note="J1 몸통을 잡고 돌린다", count=False),),
        turn=(-j.STOP_DEG, j.STOP_DEG),
        view=UPRIGHT,
    ),
    Step(
        "plate-nuts",
        "판 너트",
        "베이스 몸통에 판 너트 넣기",
        do=(
            "베이스 몸통 아래 끝 둘레의 대각선 자리에 옆으로 열린 너트 홈 넷이 있습니다 (1단계 ①의 구멍과 같은 자리).",
            "너트를 눕혀 하나씩 밀어 넣어 끝까지 보냅니다.",
        ),
        warn=(
            "몸통을 기울이면 너트가 빠질 수 있습니다. 다음 단계에서 판 나사를 걸 때까지 바로 세워 둡니다.",
        ),
        check=("밑면의 작은 구멍 넷으로 들여다보면 너트 구멍이 보인다.",),
        adds=(PLATE_NUTS,),
        uses=(Use("pencil", note="뻑뻑하면", count=False),),
        moves=tuple(
            Move((f"{PLATE_NUTS} {k + 1}",), _radial(a), 0.5 * k)
            for k, a in enumerate((45, 135, 225, 315))
        ),
        view=View((0, 1, 0), (1, 0, 0.8), 8, 0),
        labels=(Label(f"{PLATE_NUTS} 1", "M3 너트 × 4"),),
    ),
    Step(
        "plate-on",
        "판 달기",
        "책상에 무는 판 달기",
        do=(
            "베이스를 J1 몸통째 조심히 눕힙니다. 로봇 왼쪽(어깨 원통의 앞 끝)이 아래로 가게 하고, 너트가 빠지지 않게 "
            "천천히 눕힙니다.",
            "책상에 무는 판의 둥근 턱 안에 몸통 밑면을 넣습니다. 턱이 터진 곳을 몸통의 네모난 선 구멍에 맞춰 선 둘이 그리로 "
            "나가게 합니다.",
            "판 밑면의 구멍 넷에 와셔를 끼운 M3 × 10 둥근머리를 넣어, 2 mm 렌치로 조입니다.",
        ),
        warn=(
            "둥근머리를 씁니다. 머리와 와셔가 판 밑면의 얕은 자리(2.2 mm) 속에 들어가야 판이 책상에 뜨지 않습니다.",
            "선 둘이 몸통과 판 사이에 끼지 않게 봅니다.",
        ),
        check=(
            "판이 몸통 밑면에 들뜨지 않고 붙었다.",
            "나사 머리 넷이 판 밑면보다 들어가 있다.",
            "선 둘이 턱의 터진 곳으로 나와 있다.",
        ),
        adds=(PLATE, PLATE_SCREWS, PLATE_WASHERS),
        uses=(Use("hexkey"),),
        moves=(
            Move((PLATE,), (0.0, -30.0, 0.0)),
            Move((PLATE_SCREWS, PLATE_WASHERS), (0.0, -22.0, 0.0), 1.0),
        ),
        view=View((0, -1, 0), (1, 0, 0.6), 30, 0),
        labels=(
            Label(PLATE, "책상에 무는 판"),
            Label(f"{PLATE_SCREWS} M3x10 1", "M3 × 10 둥근머리 × 4 (와셔)"),
        ),
    ),
    Step(
        "desk",
        "책상에 물리기",
        "책상에 물리기",
        do=(
            "베이스를 바로 세워 책상 긴 변(55 cm)의 가운데, 뒤 가장자리에 놓습니다. 선반이 책상 안쪽을 봅니다.",
            "판의 넓은 날개 둘의 뒤 끝을 책상 뒤 가장자리에 맞춥니다. 가운데의 뒤 판(서보 어댑터 자리)은 책상 밖으로 "
            "나옵니다.",
            "금속 클램프 둘을 책상 뒤에서 끼워, 위 턱이 날개의 두꺼운 띠(가장자리에서 2.5 cm 안)를 누르게 하고 손으로 "
            "조입니다.",
            "선 둘은 책상 뒤로 늘어뜨려 둡니다.",
        ),
        warn=("클램프는 손으로만 조입니다. 공구로 세게 조이면 날개가 휩니다.",),
        check=(
            "베이스를 손으로 흔들어도 책상에서 움직이지 않는다.",
            "날개 뒤 끝이 책상 가장자리와 맞다.",
        ),
        adds=("책상$", "클램프"),
        still=("책상$",),
        moves=(Move(("클램프",), (-45.0, 0.0, 0.0)),),
        view=View((0, 1, 0), (-1, 0, 0.7), 28, 0),
        zoom=2.2,
        center=(-40.0, 30.0, 0.0),
        labels=(Label("클램프 1", "금속 클램프 (뒤에서)"),),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "J1 베이스 조립이 끝났습니다. 그림은 책상에 물린 베이스입니다.",
            "미는 도구 둘은 보관합니다 (작은 것은 다시 쓸 일이 없지만, 큰 것은 어깨를 다시 조립할 때 씁니다).",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요. 다음 안내는 파이와 서보 어댑터, J1 영점, 어깨입니다.",
        ),
        mount=(PUSH_UP, PUSH_LO, SH_PUSH),
        hide=(PUSH_UP, PUSH_LO, SH_PUSH),
        view=View((0, 1, 0), (1, 0, 0.6), 20, 0),
    ),
)

GUIDE = Guide(
    id="j1-base",
    kind="assembly",
    title="J1 베이스 조립",
    sub=f"베이스 몸통, 돌림판, J1 몸통과 어깨 베어링, 선반 · STS3215 ID {SERVO_ID} · 책상에 물리기",
    scene=scene,
    steps=STEPS,
    items=ITEMS_HERE,
    bundle="arm-black-1",
)
