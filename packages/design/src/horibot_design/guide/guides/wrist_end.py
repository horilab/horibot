"""손목 끝 조립: J6 서보(ID 6)의 부러진 받침을 새것으로 바꾸고, J6의 앞쪽을 첫 관절의 J5처럼 조립한 뒤 선, D405,
평행 그리퍼를 달고 J5와 J6를 손으로 돌려 본다 (status.md의 사용자 순서 2, 0029, 0030, 0031).

장면은 첫 관절처럼 J5의 서보 좌표다: 손목 1이 서 있고, 손목 2와 거기 달린 것은 J5와 같이 돈다. J6의 원판과 도구는
둘째 관절(J6, Guide.axis2)과도 같이 돈다. 손목 2는 J5 원판에서 떼어 첫 관절의 J5처럼 책상 위에서 조립하고 다시
단다: 단 채로는 J6 서보를 뒤로 빼면 가짜 아래팔 끝의 관에 걸리고, 매달린 손목 1과 가짜 아래팔 끝을 든 채 베어링을
누르거나 뒤집게 된다. 떼어 둔 동안에는 손목 1 쪽 부품을 숨기고, 손목 2는 J5 각 0의 자리 그대로 두되 손에 드는
방향만 손목 2의 것으로 바꿔 보인다 (_child_view). 몸통 나사 구멍의 틈만큼 J5의 영점이 조금 달라질 수 있다.

선은 0031의 길이다. J5에서 온 선은 J6 서보의 -z 커넥터에 꽂고 손목 2 뒷덮개의 창으로, 그리퍼 선은 +z 커넥터에 꽂고
덮개 밑 +z 절반으로 발까지 내려가 손목 2 앞벽의 +z 옆 홈으로 나간다. 그리퍼 선은 서보 먼 끝과 J5 원판 사이 틈(2.6 mm)을
지나므로 손목 2를 J5 원판에 다시 달기 전에 놓는다 (손목 1의 선을 놓을 때 가짜 아래팔 끝을 뗀 것과 같다).

D405 받침의 돋움 너트는 뒤에서 떨어뜨려 넣는 홈이라, 뒷덮개를 달기 전에 넣는다. 덮개를 달면 테가 홈 입구를 막아
너트가 빠지지 않지만 2.5 mm쯤 뒤로 밀릴 수 있어서, 받침은 손목 2의 뒤쪽이 위를 보는 자세(가짜 아래팔 끝을 책상에
세우고 J5를 - 끝까지)에서 단다. 그 자세에서 플랜지는 책상에서 6 cm쯤 떠 있다.

가짜 아래팔 끝(J4가 범위 가운데인 자리)을 단 채로는 D405가 J5 +62 ~ +138°에서, 그리퍼가 J5 ±125° 밖에서 그 관에
닿는다. 그래서 돌려 보기 전에 가짜 아래팔 끝을 뗀다 (실제 팔에서 그 각은 J4가 가운데가 아닐 때 간다).

J6의 영점 핀은 그리퍼를 달면 그리퍼 서보가 핀 위를 가려서(0030), 그리퍼를 달기 전에 맞춰 본다.
"""

from __future__ import annotations

import itertools
from dataclasses import replace

from build123d import Box, Compound, Plane, Vector

from horibot_design.guide.catalog import ITEMS, _screw, printed_items
from horibot_design.guide.guides import cable_motion as cm
from horibot_design.guide.guides import cable_motion_300 as c3
from horibot_design.guide.guides import d405_plug as dp
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
    Vec,
    View,
)
from horibot_design.parts import camera_mount as cam
from horibot_design.parts import fake_forearm as ff
from horibot_design.parts import flange, wrist
from horibot_design.parts import gripper as gp
from horibot_design.parts import joint as j
from horibot_design.parts.shapes import box, solid

MODEL = fj.MODEL
J5_ID = fj.SERVO_ID
J6_ID = cm.J6_ID
GRIP_ID = 11  # 그리퍼 서보 (0030, servo-setup.md: ID는 관절 번호, 그리퍼는 11)
DY = fj.DY

# ------------------------------------------------------------------------------------------------
# 손목 2(자식)의 서보 좌표 → 장면(J5의 서보 좌표, J5 각 0). wrist.as_child와 같은 옮김이다


def _p(v: Vec) -> Vec:
    """손목 2 서보 좌표의 점을 장면 좌표로."""
    x, y, z = v
    return (y - wrist.AX_Y, x + j.ATOP - wrist.X_FOOT, -z)


_d = cm._child_dir  # 방향
_v = cm._child_view  # 손에 드는 방향

# 둘째 관절(J6)의 축: 손목 2의 출력축(자식 +y)이 장면의 +x로 간다. 이 둘레로 -q 돈 것이 J6 각 q다
AXIS2: tuple[Vec, Vec] = ((0.0, j.ATOP - wrist.X_FOOT, 0.0), (1.0, 0.0, 0.0))

# ------------------------------------------------------------------------------------------------
# 부품 목록

J6_ITEM = cm.J6_ITEM
GRIP_ITEM = replace(
    ITEMS["servo_sts3215"],
    key="servo_sts3215_grip",
    name=f"서보 STS3215 · ID {GRIP_ID}",
    meta=f"혼이 달린 것. ID {GRIP_ID} 이름표가 붙은 서보 (그리퍼)",
    thumb=View((1, 0, 0), (0, 0.4, 1), 22, 0),
)
CABLE_ITEM = c3.CABLE_ITEM
USB_ITEM = dp.CABLE_ITEM
CAMERA_ITEM = replace(dp.CAMERA_ITEM, thumb=View((0, 1, 0), (1, 0, -0.3), 15, 0))
TOOL_PIN = Item(
    "pin_tool",
    "도구 핀 MSH3-20",
    "bought",
    "강철 핀, 지름 3 · 길이 20 mm. 영점 핀과 같은 핀이다",
    draw={"t": "pin", "L": 20, "d": 3},
)
TOOLS = (
    Item("pliers", "펜치나 롱노즈", "tool", "영점 핀을 곧게 당겨 뺄 때", icon="pliers"),
    Item(
        "phone", "휴대폰", "tool", "사진을 찍어 대화창에 보낸다. 속이 어두우면 불빛", icon="phone"
    ),
    Item("pencil", "샤프", "tool", "뻑뻑한 너트를 끝으로 살살 밀 때", icon="pen"),
    Item("book", "책 한 권", "tool", "두께 6 mm 넘게. 첫 관절을 세울 때 괸다", icon="book"),
)
# 이미 조립해 둔 첫 관절의 부품 (개수에 넣지 않는다). 키를 하나로 모아, 같은 부품을 새로 쓰는 J6의 작은 그림에
# 손목 1 속의 것이 같이 그려지지 않게 한다
ASSEMBLED = Item("assembled_j5", "조립해 둔 첫 관절", "bought", f"손목 1과 J5 서보(ID {J5_ID})")
M3X6 = _screw(6)


def _black(labels: dict[str, str]) -> dict[str, Item]:
    """검정으로 뽑는 부품 (뽑는 부품 표가 형상을 만드는 함수라서, 색만 적은 표로 목록을 만든다)."""
    return printed_items(labels, {k: (None, None, "black") for k in labels})


THUMBS = {
    "flange_disc": View((1, 0, 0), (0, 0.4, 1), 40, 0),
    "tool_adapter": View((1, 0, 0), (0, 0, 1), 45, 0),
    "gripper_frame": View((-1, 0, 0), (0, 0.5, -1), 25, 0),
    "gripper_floor": View((1, 0, 0), (0, 0, 1), 55, 0),
    "gripper_tub": View((1, 0, 0), (0, 0, 1), 35, 0),
    "gripper_pinion": View((-1, 0, 0), (0, 0, 1), 50, 0),
    "gripper_slider": View((1, 0, 0), (0, 0.6, 1), 30, 0),
    "gripper_finger": View((-1, 0, 0), (0, 1, 0.5), 15, 0),
    "d405_mount": View((0, 1, 0), (0.4, 0, 1), 20, 0),
}
_WRIST = printed_items(wrist.LABELS, wrist.PRINT_SPEC)


def _in_w2(item: Item) -> Item:
    """첫 관절 안내의 작은 그림 방향(손목 1 속)을 손목 2 속의 같은 부품에 맞춘다."""
    return replace(item, thumb=_v(item.thumb)) if item.thumb else item


CARRIER_ITEM = replace(
    _in_w2(_WRIST["carrier"]),
    meta='검정 · 출력 안내 "선 돌려 보기 출력"에서 둘 뽑아 둔 것 가운데 하나 (하나는 여분)',
)
PRINTED: dict[str, Item] = {
    **_WRIST,
    **{k: _in_w2(_WRIST[k]) for k in ("hub_plate", "retainer")},
    **printed_items(ff.LABELS, ff.PRINT_SPEC),
    **{
        k: replace(it, thumb=THUMBS.get(k))
        for k, it in {
            **_black(flange.LABELS),
            **_black(gp.LABELS),
            **_black(cam.LABELS),
        }.items()
    },
    "carrier": CARRIER_ITEM,
}
BEARING_ITEM = _in_w2(ITEMS["bearing_6806"])

# ------------------------------------------------------------------------------------------------
# 장면

_W, _T = j.WIRE_W, j.WIRE_T
# 서보 먼 끝 앞으로 내리는 선의 가운데 (first_joint.cable_parts와 같다)
_XD = -(j.FAR + 0.3 + _T / 2)


def _flat(a: tuple[float, float], b: tuple[float, float], z: float) -> Compound:
    """서보 좌표의 (x, y) 평면 위 a에서 b까지 눕힌 선 한 토막: 폭은 z 방향, 두께는 그 평면 안에서 길과 직각."""
    va, vb = Vector(a[0], a[1], z), Vector(b[0], b[1], z)
    d = vb - va
    x = d.normalized()
    plane = Plane(origin=(va + vb) * 0.5, x_dir=x, z_dir=Vector(0.0, 0.0, 1.0))
    return solid(plane * Box(d.length + _T, _T, _W))


def _grip_tail() -> Compound:
    """그리퍼 속의 선 (J6 서보 좌표): 주머니 밖 6 mm → 주머니와 턱 → 서보 뒷면 밑 → 플러그. 선 가운데의 꺾은선
    (gripper.wire_tail)을 이은 것이다."""
    (_x0, z), (xo, _z), h = gp.wire_exit()
    pts = [(xo, j.ATOP + h), *[(x, j.ATOP + hh) for x, hh, _ in gp.wire_tail()]]
    segs = [_flat(a, b, z) for a, b in itertools.pairwise(pts)]
    shape = segs[0]
    for seg in segs[1:]:
        shape = solid(shape + seg)
    return shape


# 이름 (선택자는 이름의 앞부분이다. 겹치지 않게 짓는다). 손목 1 쪽(첫 관절) 부품은 "J5 "로 시작한다. 손목 2에 든
# J5 → J6 선은 "J5에서"로 시작해 손목 1 쪽과 가른다
W1 = "손목 1 몸통"
W2 = "손목 2 몸통"
BODY_SCREWS = "몸통 나사"
FAKE = "가짜 아래팔 끝$"
FAKE_SET = ("가짜 아래팔 끝", "발 나사")
J5_CABLE_W1 = ("J5 선 (손목 1 속)", "J5 선 플러그 (J5)")
J5_IN_W2 = ("J5에서 온 선 (손목 2 속)", "J5에서 온 선 플러그 (J6)")
GRIP_IN_W2 = ("그리퍼 선 (손목 2 속)", "그리퍼 선 플러그 (J6)")
OLD_CARRIER = "J6 옛 받침"
CARRIER = "J6 서보 받침"
CAR_NUTS = "J6 너트 (서보 받침"
SERVO6 = "J6 서보 몸통"
HORN6 = "J6 혼$"
BACK6 = "J6 서보 뒷면 나사"
SERVO6_SET = (SERVO6, HORN6, CARRIER, CAR_NUTS, BACK6)
BEARING6 = "J6 베어링"
RET6 = "J6 누름 틀"
RET_NUTS = "J6 너트 (누름 틀"
WALL = "J6 벽 나사"
LOCK = ("J6 벽 나사 2", "J6 벽 나사 4")
HUB6 = "J6 허브 판"
DISC = "플랜지 원판"
DISC_NUTS = "J6 너트 (원판)"
POST = ("J6 멈춤 나사", "J6 너트 (멈춤 기둥)")
PINS = "도구 핀"
DISC_SET = (DISC, DISC_NUTS, *POST, PINS)
BOLTS6 = "J6 혼 볼트"
COVER = "손목 2 뒷덮개"
COVER_SCREWS = "J6 덮개 나사"
ZERO_PIN = "J6 영점 핀"
ADAPTER = "도구 어댑터 판"
THUMB = "손나사"
D_NUTS = "D405 너트 (돋움)"
D_MOUNT = "D405 받침$"
D_CAM = "D405 카메라$"
D_PLUG = "D405 플러그"
D_WIRE = "D405 선"
D_SCREWS = "D405 받침 나사"
CAM_SCREWS = "D405 카메라 나사"
D_SET = (D_MOUNT, D_CAM, D_PLUG, D_WIRE, CAM_SCREWS)
G_FRAME = "그리퍼 몸통"
G_FLOOR = "그리퍼 바닥판"
G_TUB = "랙 집"
G_PINION = "피니언$"
G_SLIDERS = "손가락 받침"
G_FINGERS = ("손가락 1$", "손가락 2$")
G_SERVO = ("그리퍼 서보", "그리퍼 혼")
G_PLUG = "그리퍼 선 플러그 (그리퍼)"
G_TAIL = "그리퍼 선 (그리퍼 속)"
PINION_SCREWS = "피니언 나사"
FLOOR_SCREWS = "바닥판 나사"
BOSS_NUTS = "너트 (몸통 돋움)"
END_SCREWS = "끝벽 나사"
LUG_SCREWS = "귀 나사"
FINGER_SCREWS = "손가락 나사"
FINGER_NUTS = "너트 (손가락)"
GRIPPER = (
    G_FRAME,
    G_FLOOR,
    G_TUB,
    G_PINION,
    G_SLIDERS,
    *G_FINGERS,
    *G_SERVO,
    PINION_SCREWS,
    FLOOR_SCREWS,
    BOSS_NUTS,
    END_SCREWS,
    LUG_SCREWS,
    FINGER_SCREWS,
    FINGER_NUTS,
)

# 첫 관절의 두 몸통 이름. 나머지 첫 관절 부품은 이름 앞에 "J5 "를 붙인다 (손목 2 속 J6 부품과 가르려고)
_J5_RENAME = {"손목 몸통 (부모)": W1, "손목 몸통 (자식)": W2}


def _j5_side() -> list[ScenePart]:
    """조립해 둔 첫 관절: 손목 1과 속 부품, J5 원판, 손목 2 몸통, 몸통 나사. 모두 이미 있는 것이다."""
    out: list[ScenePart] = []
    for p in fj.scene():
        if p.name.startswith(("서보 선", "영점 핀")):
            continue
        key = ASSEMBLED.key
        if p.name in _J5_RENAME:
            name = _J5_RENAME[p.name]
            key = p.key if name == W2 else key
        elif p.name.startswith("몸통 나사"):
            name, key = p.name, p.key
        else:
            name = f"J5 {p.name}"
        out.append(replace(p, name=name, key=key, count=False))
    return out


def _fake() -> list[ScenePart]:
    out: list[ScenePart] = []
    for p in ff.placed(MODEL):
        key = (
            "fake_forearm"
            if p.name == "가짜 아래팔 끝"
            else "nut_m3"
            if p.name.startswith("발 나사 너트")
            else "m3x12"
        )
        out.append(ScenePart(p.name, p.shape, key, p.color, False, False))
    return out


def _cables() -> list[ScenePart]:
    """서보 선: J5 → J6 (있는 선), J6 → 그리퍼 (새 300 mm 선)."""
    plus, inside, _hang = fj.cable_parts(wrist.SIDE_LANE)  # +z 커넥터 → +z 옆 홈
    minus = box(-16.6, -11.85, j.PLUG_TOP + DY, j.L1 + 0.5 + DY, -9.46, -0.45)
    w2_in = next(p for p in wrist.wrist2_inner(MODEL) if p.name == "선 (J5에서)").shape
    # 손목 2의 +z 옆 홈을 나와 J6 축과 나란히 도구 쪽으로 6 mm (그 뒤는 그리퍼 몸통 겉을 도는 고리다)
    z0, z1 = wrist.SIDE_LANE - _W / 2, wrist.SIDE_LANE + _W / 2
    rise = box(_XD - _T / 2, _XD + _T / 2, j.FRONT - 0.5, j.FRONT + 6.0, z0, z1)
    key = CABLE_ITEM.key
    return [
        ScenePart(J5_CABLE_W1[0], inside, key, "wire", False, False),
        ScenePart(J5_CABLE_W1[1], plus, key, "plug", False, False),
        ScenePart(J5_IN_W2[0], wrist.as_child(w2_in), key, "wire", True, False),
        ScenePart(J5_IN_W2[1], wrist.as_child(solid(minus)), key, "plug", True, False),
        ScenePart(GRIP_IN_W2[0], wrist.as_child(solid(inside + rise)), key, "wire", True, False),
        ScenePart(GRIP_IN_W2[1], wrist.as_child(plus), key, "plug", True, True),
    ]


def _j6() -> list[ScenePart]:
    """손목 2 속의 J6 부품 (손목 2 서보 좌표의 것을 장면으로 옮긴다). 출력 원판 대신 플랜지 원판이다."""
    out: list[ScenePart] = []
    keys = {
        "베어링 6806": ("bearing_6806", BEARING6),
        "허브 판": ("hub_plate", HUB6),
        "누름 틀": ("retainer", RET6),
        "서보 받침": ("carrier", CARRIER),
        "뒷덮개": ("wrist2_cover", COVER),
        "서보 몸통": (J6_ITEM.key, SERVO6),
        "혼": (J6_ITEM.key, "J6 혼"),
    }
    old = {"뒷덮개", "서보 몸통", "혼"}  # 이미 손목 2에 든 것
    rotor = {"허브 판", "혼"}
    for p in j.inner_parts(MODEL, wrist.X_FOOT, False, wrist.wrist2_cover()):
        if p.name in ("플러그", "선 자리", "출력 원판"):
            continue
        shape = wrist.as_child(p.shape)
        if p.name.startswith("벽 나사"):
            k = int(p.name.split()[-1])
            out.append(ScenePart(f"J6 {p.name}", shape, "m3x8", "metal", True, k not in (2, 4)))
            continue
        key, name = keys[p.name]
        r = p.name in rotor
        out.append(ScenePart(name, shape, key, p.color, True, p.name not in old, moving2=r))
        if p.name == "서보 받침":  # 부러진 옛 받침 (같은 모양, 처음에 달려 있다)
            out.append(ScenePart(OLD_CARRIER, shape, "carrier", "frame", True, False))
    disc = wrist.as_child(flange.flange_disc())
    out.append(ScenePart(DISC, disc, "flange_disc", "cap", True, True, moving2=True))
    ret_n, car_n = 0, 0
    for p in j.inner_hardware(MODEL):
        shape = wrist.as_child(p.shape)
        name, key, new, rot = p.name, "", True, p.moving
        if p.name.startswith("혼 볼트"):
            name, key = f"J6 {p.name}", "m3x20"
        elif p.name.startswith("멈춤 나사"):
            name, key = "J6 멈춤 나사 M3x10", "m3x10"
        elif p.name.startswith("너트 (멈춤"):
            name, key = "J6 너트 (멈춤 기둥)", "nut_m3"
        elif p.name.startswith("너트 (출력 원판)"):
            name, key = f"{DISC_NUTS} {p.name.split()[-1]}", "nut_m3"
        elif p.name.startswith("서보 뒷면 나사"):
            name, key, new = f"J6 {p.name}", "selftap_so101", False
        elif p.name.startswith("덮개 나사"):
            name, key, new = f"J6 {p.name}", "m3x12", False
        elif p.name == "너트 (누름 틀)":
            side = ("+z", "-z", "+x")[ret_n]
            ret_n += 1
            name, key = f"{RET_NUTS} {side})", "nut_m3"
        elif p.name == "너트 (서보 받침)":
            side = ("+z", "-z")[car_n]
            car_n += 1
            name, key = f"{CAR_NUTS} {side})", "nut_m3"
        out.append(ScenePart(name, shape, key, p.color, True, new, moving2=rot))
    for p in flange.hardware():
        shape = wrist.as_child(p.shape)
        if p.name.startswith("도구 핀"):
            out.append(
                ScenePart(
                    f"{PINS} {p.name.split()[-1]}",
                    shape,
                    TOOL_PIN.key,
                    "pin",
                    True,
                    True,
                    moving2=True,
                )
            )
        elif p.name.startswith("손나사"):
            out.append(
                ScenePart(
                    f"{THUMB} {p.name[4:]}", shape, "m3x10", "metal", True, True, moving2=True
                )
            )
        elif p.name.startswith("영점 핀"):
            out.append(ScenePart(ZERO_PIN, shape, "pin_msh3_20", "pin", True, True))
    adapter = wrist.as_child(flange.tool_adapter())
    out.append(ScenePart(ADAPTER, adapter, "tool_adapter", "frame", True, True, moving2=True))
    return out


def _d405() -> list[ScenePart]:
    out = [
        ScenePart(D_MOUNT[:-1], wrist.as_child(cam.mount()), "d405_mount", "cap", True),
        ScenePart(D_CAM[:-1], wrist.as_child(cam.camera()), CAMERA_ITEM.key, "servo", True),
        ScenePart(D_PLUG, wrist.as_child(cam.plug()), USB_ITEM.key, "plug", True),
        ScenePart(D_WIRE, wrist.as_child(cam.wire_stub()), USB_ITEM.key, "wire", True, False),
    ]
    for p in cam.hardware():
        shape = wrist.as_child(p.shape)
        k = p.name.split()[-1]
        if p.name.startswith("카메라 나사"):
            out.append(ScenePart(f"{CAM_SCREWS} {k}", shape, "m3x6", "metal", True))
        elif p.name.startswith("받침 나사"):
            out.append(ScenePart(f"{D_SCREWS} {k}", shape, "m3x8", "metal", True))
        else:
            out.append(ScenePart(f"{D_NUTS} {k}", shape, "nut_m3", "metal", True))
    return out


_GRIP_KEYS = {
    "그리퍼 몸통": ("gripper_frame", G_FRAME),
    "바닥판": ("gripper_floor", G_FLOOR),
    "랙 집": ("gripper_tub", G_TUB),
    "피니언": ("gripper_pinion", "피니언"),
    "그리퍼 서보": (GRIP_ITEM.key, "그리퍼 서보"),
    "그리퍼 혼": (GRIP_ITEM.key, "그리퍼 혼"),
}


def _gripper() -> list[ScenePart]:
    """평행 그리퍼 (손가락을 닫은 자리). J6와 같이 돈다."""
    out: list[ScenePart] = []
    for p in gp.gripper_parts(0.0):
        shape = wrist.as_child(p.shape)
        if p.name == "그리퍼 플러그":
            out.append(ScenePart(G_PLUG, shape, CABLE_ITEM.key, "plug", True, False, moving2=True))
            continue
        if p.name in _GRIP_KEYS:
            key, name = _GRIP_KEYS[p.name]
            count = p.name != "그리퍼 혼"
        elif p.name.startswith("손가락 받침"):
            key, name, count = "gripper_slider", p.name, True
        elif p.name.startswith("손가락 나사"):
            key, name, count = "m3x12", p.name, True
        elif p.name.startswith("손가락"):
            key, name, count = "gripper_finger", p.name, True
        elif p.name.startswith("피니언 나사"):
            key, name, count = "m3x6", p.name, True
        elif p.name.startswith(("바닥판 나사", "끝벽 나사")):
            key, name, count = "m3x8", p.name, True
        elif p.name.startswith("귀 나사"):
            key, name, count = "m3x12", p.name, True
        else:  # 너트
            key, name, count = "nut_m3", p.name, True
        if p.name == "바닥판":
            name = G_FLOOR
        out.append(ScenePart(name, shape, key, p.color, True, count, moving2=True))
    tail = wrist.as_child(_grip_tail())
    out.append(ScenePart(G_TAIL, tail, CABLE_ITEM.key, "wire", True, False, moving2=True))
    return out


def scene() -> list[ScenePart]:
    return [*_j5_side(), *_fake(), *_cables(), *_j6(), *_d405(), *_gripper()]


# ------------------------------------------------------------------------------------------------
# 손에 드는 방향, 이름표 자리

# 첫 관절을 책 위에 세운 자세 (손목 1의 뒷덮개가 아래, 손목 2가 위): J5 축이 위다
STAND_SIDE = View((0, 1, 0), (0.25, 0, 1), 22, 0)  # 옆에서: 손목 2 발의 귀와 나사
STAND_FRONT = View((0, 1, 0), (1, 0, 0.6), 18, 0)  # 손목 2의 앞쪽(플랜지 쪽)에서
STAND_D405 = View((0, 1, 0), (-0.4, 0, 1), 20, 0)  # D405 쪽(장면의 +z)에서
# 가짜 아래팔 끝을 책상에 세운 자세 (관절 축이 눕는다): 장면의 +x가 위다
LOW = View((1, 0, 0), (0, 0.35, 1), 20, 0)
LOW_D405 = View((1, 0, 0), (0, 0.2, -1), 25, 0)  # 같은 자세를 D405 쪽에서
# 떼어 둔 손목 2를 손에 드는 방향. 자르는 면이 J6 축을 지나게 장면의 ±z를 본다 (자르는 면은 장면 원점을 지나
# 보는 방향과 직각이다)
W2_BACK_UP = View((-1, 0, 0), (0, 0, 1), 30, 28)  # 뒤쪽(열린 쪽)이 위
W2_FRONT_UP = View((1, 0, 0), (0, 0, 1), 32, 28)  # 앞쪽(플랜지)이 위
# 그리퍼를 손에 드는 방향 (도구 쪽이 장면의 +x)
G_DECK_DOWN = View((1, 0, 0), (0, 0.6, -1), 30, 0)  # 판이 아래, 서보 자리가 위
G_DECK_UP = View((-1, 0, 0), (0, 0.6, -1), 25, 0)  # 판이 위, 손가락이 아래
G_TUB_OPEN_UP = View((-1, 0, 0), (0, 0.4, -1), 40, 0)  # 랙 집의 열린 쪽이 위

W2_BACK_TAG = Tag(_p((0.0, j.BACK_END - 9.0, 0.0)), "뒤 (열린 쪽)")
W2_FOOT_TAG = Tag(_p((wrist.X_FOOT - 6.0, 6.0, 0.0)), "발 쪽 (납작한 끝)")
W2_LIP_TAG = Tag(_p((-(j.R_LIP + 1.5), j.FRONT, 0.0)), "앞 끝 턱")
W2_SIDE_SLOT = Tag(
    _p((wrist.X_FOOT + 1.2, j.FRONT + 3.0, wrist.SIDE_LANE)), "옆 홈 (영점 핀 귀 쪽)"
)
POST_EAR_TAG = Tag(_p((j.R_POST + 8.0, j.ATOP, 0.0)), "멈춤 귀 (발 반대쪽)")
PIN_EAR_TAG = Tag(_p((-(j.R_PIN + 8.0), j.ATOP, 0.0)), "영점 핀 귀 (발 쪽)")
POST_EAR_ONLY = Tag(POST_EAR_TAG.at, "멈춤 귀 (밑에 기둥)")
PIN_EAR_ONLY = Tag(PIN_EAR_TAG.at, "영점 핀 귀 (구멍만)")
J5_EAR_AT = (j.R_POST + j.EAR_HW, j.ATOP - 1.0, 0.0)  # J5 원판의 멈춤 귀 끝 (장면 좌표, J5 각 0)

PHOTO = fj.PHOTO
SLOW = (
    "천천히 돌립니다. 서보는 기어비가 커서 손으로 돌리면 뻑뻑한 것이 정상이고, 빨리 돌리면 기어에 무리가 갈 수 "
    "있습니다."
)
END_STOP = (
    "끝에서 멈춤 기둥이 살에 닿으면 더 밀지 않습니다. 걸리면 억지로 돌리지 말고 사진을 보내 주세요."
)
HOLD = "돌린 뒤 손을 떼도 관절은 그 자리에 섭니다 (서보 기어가 잡아 줍니다)."
SHOOT = "그림을 보는 쪽에서, 그림처럼 손목과 선이 다 들어오게 찍어 대화창에 보내 주세요."


def _slide(name: str, reach: float = 13.0) -> Vec:
    """옆 홈에 너트를 밀어 넣는 움직임 (손목 2 속 부품): 입구 쪽으로 reach만큼 떨어진 자리에서 들어온다."""
    return _d(fj._slide(name, reach))


def _cam_dir(v: Vec, length: float) -> Vec:
    """카메라 좌표의 방향을 장면 방향으로 (camera_mount.FRAME의 축, 손목 2 서보 좌표를 거친다)."""
    x, y, z = v
    ay, az_ = cam.AXIS[1], cam.AXIS[2]
    oy, oz = -cam.INWARD[1], -cam.INWARD[2]
    servo = (x * length, (y * oy + z * ay) * length, (y * oz + z * az_) * length)
    return _d(servo)


def _cam_pt(v: Vec) -> Vec:
    """카메라 좌표의 점을 장면 좌표로."""
    return _p(cam.to_servo(v))


CAM_LENS_TAG = Tag(_cam_pt((0.0, 0.0, 4.0)), "렌즈 면")
CAM_BOTTOM_TAG = Tag(_cam_pt((-cam.CAM_H / 2 - 5.0, 0.0, -cam.CAM_D / 2)), "바닥면 (1/4 나사 구멍)")
CAM_TOP_TAG = Tag(_cam_pt((cam.CAM_H / 2 + 5.0, 0.0, -cam.CAM_D / 2)), "윗면")


# 손목 2에 붙어 J5와 같이 움직이는 것 (손목 2를 J5 원판에 다는 움직임)
W2_GROUP = (
    W2,
    BEARING6,
    RET6,
    "J6 너트",
    WALL,
    "J6 서보",
    "J6 혼",
    HUB6,
    "J6 멈춤 나사",
    COVER_SCREWS,
    DISC,
    PINS,
    COVER,
    J5_IN_W2[0],
    J5_IN_W2[1],
    *GRIP_IN_W2,
    D_NUTS,
)
G_TUB_GROUP = (
    G_TUB,
    G_SLIDERS,
    *G_FINGERS,
    FINGER_SCREWS,
    FINGER_NUTS,
)
FINGER_1 = ("손가락 1$", "손가락 받침 1", "손가락 나사 M3x12 1", "너트 (손가락) 1")
FINGER_2 = ("손가락 2$", "손가락 받침 2", "손가락 나사 M3x12 2", "너트 (손가락) 2")
# 다 단 그리퍼 (선까지)
G_ALL = (*GRIPPER, G_PLUG, G_TAIL)

# 처음에 이미 조립돼 있는 것 (첫 단계에서 본체에 붙은 것으로 둔다)
BASE = (
    W1,
    "J5 ",
    *FAKE_SET,
    BODY_SCREWS,
    W2,
    *J5_IN_W2,
    SERVO6,
    HORN6,
    OLD_CARRIER,
    BACK6,
    *LOCK,
    COVER,
    COVER_SCREWS,
)

STEPS_RAW: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "목록의 부품, 나사, 공구를 모두 꺼내 책상에 늘어놓습니다.",
            "출력 부품은 손목 끝 판에서 뽑은 열세 개와, 선 돌려 보기 때 뽑아 둔 서보 받침 하나입니다.",
            "나사는 길이별로 따로 모읍니다. M3 × 6, × 8, × 10, × 12, × 20 다섯 가지이고, × 8, × 10, × 12는 "
            "2 mm씩 차이라 헷갈리기 쉽습니다.",
            f"서보는 그리퍼에 넣을 ID {GRIP_ID} 서보 하나를 꺼냅니다. J6 서보(ID {J6_ID})는 손목 2에 든 그대로 "
            "씁니다.",
            "길이를 모르는 나사가 섞여 있으면 자나 캘리퍼스 옆에 놓고 찍어 대화창에 보내 주세요. Claude가 알려 "
            "드립니다.",
        ),
        check=(
            "출력 부품 14개: 손목 끝 판의 13개(검정)와 서보 받침 하나",
            f"베어링, ID {GRIP_ID} 서보, D405, 얇은 USB 2 선, 300 mm 서보 선, 도구 핀 둘, 첫 관절의 영점 핀",
            "나사와 너트를 개수대로",
            "공구, 책 한 권(두께 6 mm 넘게)",
        ),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    # ---- 손목 2를 J5 원판에서 떼고 책상 위에서 연다 ----
    Step(
        "w2-off",
        "손목 2 떼기",
        "손목 2를 J5 원판에서 떼기",
        do=(
            "책상에 책을 놓고, 손목 2가 위로 오게 손목 1의 뒷덮개를 책 위에 세웁니다. 가짜 아래팔 끝의 귀는 책 "
            "밖으로 나오게 합니다 (선 시험 때와 같은 자세).",
            "손목 2 발의 양옆 귀에 있는 나사(M3 × 10) 4개를 육각렌치로 풉니다. 귀 하나에 둘씩입니다.",
            "손목 2를 곧게 들어 J5 원판에서 떼고, 손목 1 바로 옆에 뒷덮개가 위로 오게 내려놓습니다. 300 mm 선은 "
            "두 몸통에 꽂힌 그대로입니다.",
        ),
        warn=(
            "나사 넷은 손목 2를 다시 달 때 쓰니 모아 둡니다. 원판 밑면의 너트 넷은 원판에 남습니다.",
            "300 mm 선이 당겨지지 않게 손목 2를 손목 1 가까이에 둡니다.",
            "손목 2를 단 채로는 J6 서보를 뺄 수 없습니다. 뒤쪽으로 당기면 가짜 아래팔 끝의 관에 걸립니다.",
        ),
        check=("손목 2를 뗐다.", "나사 넷을 모아 두었다."),
        stuck=PHOTO,
        adds=BASE,
        still=BASE,
        mount=BASE,
        uses=(Use("hexkey"), Use("book")),
        view=STAND_SIDE,
        zoom=1.3,
        center=(0.0, 50.0, 0.0),
        labels=(
            Label("몸통 나사 M3x10 1", "M3 × 10 × 4 (푼다)"),
            Label(W2, "손목 2 (뗀다)"),
        ),
    ),
    Step(
        "w2-open",
        "덮개 떼기",
        "손목 2의 뒷덮개 떼기",
        do=(
            "손목 2의 뒷덮개 나사(M3 × 12) 2개를 육각렌치로 풉니다.",
            "덮개를 들어 뗍니다. 덮개의 창에 든 300 mm 선은 창이 트인 옆으로 빠져나옵니다.",
        ),
        warn=("뗀 덮개와 나사 둘은 다시 쓰니 모아 둡니다.", "선을 잡아당기지 않습니다."),
        check=("덮개를 뗐고, 나사 둘을 모아 두었다.",),
        stuck=PHOTO,
        uses=(Use("hexkey"),),
        view=W2_BACK_UP,
        labels=(
            Label(COVER, "뒷덮개 (뗀다)", _p((-26.0, j.BACK_OUT, -6.0))),
            Label("J6 덮개 나사 M3x12 1", "M3 × 12 × 2 (다시 쓴다)"),
        ),
        tags=(W2_FOOT_TAG,),
    ),
    Step(
        "j6-unplug",
        "선 뽑기",
        "J6 서보에서 300 mm 선 뽑기",
        do=(
            f"J6 서보(ID {J6_ID}) 뒷면의 커넥터 둘 가운데, 300 mm 선이 꽂힌 커넥터(그림)에서 플러그를 뽑습니다. "
            "선이 아니라 플러그 몸통을 잡고 곧게 당깁니다.",
            "뽑은 선은 손목 1에 꽂힌 채로 늘어뜨려 둡니다.",
        ),
        check=("300 mm 선의 플러그를 J6 서보에서 뽑았다.",),
        stuck=PHOTO,
        uses=(Use(CABLE_ITEM.key, note="손목 1에서 온 것 (손목 1에 꽂힌 채)", count=False),),
        view=_v(View((0, -1, 0), (-1, 0, 1.1), 22, 0)),
        zoom=1.4,
        labels=(Label(J5_IN_W2[1], "뽑을 플러그"),),
    ),
    Step(
        "j6-out",
        "J6 빼기",
        "J6 서보 빼기",
        do=(
            "손목 2 벽의 뒷줄(뒤쪽 열린 끝에 가까운 줄) 나사(M3 × 8) 2개를 육각렌치로 풉니다.",
            f"J6 서보(ID {J6_ID})를 뒤쪽(열린 쪽)으로 당겨 뺍니다. 부러진 서보 받침이 붙은 채로 나옵니다.",
        ),
        warn=("나사 둘은 다시 쓰니 모아 둡니다.",),
        check=("J6 서보를 손목 2에서 뺐다.", "나사 둘을 모아 두었다."),
        stuck=PHOTO,
        uses=(Use("hexkey"),),
        view=W2_BACK_UP,
        shell=(W2,),
        ghost=True,
        labels=(
            Label("J6 벽 나사 2", "뒷줄 나사"),
            Label("J6 벽 나사 4", "뒷줄 나사"),
            Label(SERVO6, f"J6 서보 (ID {J6_ID})"),
        ),
        tags=(W2_BACK_TAG,),
    ),
    # ---- J6 서보의 받침을 새것으로 ----
    Step(
        "j6-old",
        "옛 받침",
        "부러진 서보 받침 떼기",
        do=(
            f"J6 서보(ID {J6_ID})를 혼이 아래로 가게 엎어 놓습니다.",
            "서보 뒷면 나사 4개를 작은 십자 드라이버로 풀고, 부러진 받침을 뗍니다.",
        ),
        warn=(
            "나사 넷은 새 받침에 다시 쓰니 모아 둡니다.",
            "부러진 받침에 너트가 남아 있으면 빼서 따로 둡니다 (여분).",
        ),
        check=("부러진 받침을 뗐다.", "서보 뒷면 나사 넷을 모아 두었다."),
        stuck=PHOTO,
        uses=(Use("driver"),),
        focus=(SERVO6, HORN6, OLD_CARRIER, BACK6),
        view=_v(View((0, -1, 0), (0, 0, 1), 35, 20)),
        labels=(
            Label(OLD_CARRIER, "부러진 받침"),
            Label("J6 서보 뒷면 나사 1", "서보 뒷면 나사 × 4"),
        ),
    ),
    Step(
        "j6-nuts",
        "받침 너트",
        "새 서보 받침에 너트 넣기",
        do=(
            "새 서보 받침을 넓은 판이 아래, 날개 둘이 위로 오게 놓습니다.",
            "날개마다 바깥에 돋움이 있고, 그 옆면(판이 터진 쪽)에 너트 홈 입구가 있습니다. 그림에서 화살표가 "
            "들어가는 곳입니다.",
            "너트를 동전 세우듯 세워서 입구에 넣고, 홈 끝에 닿을 때까지 밉니다. 돋움 둘에 하나씩 넣습니다.",
        ),
        warn=(
            "두 입구는 같은 쪽(판이 터진 쪽)에 있습니다. 옆면에 네모난 구멍이 뚫린 쪽으로만 넣고, 막힌 면으로 "
            "밀지 않습니다.",
            "뻑뻑하면 샤프 끝으로 너트를 살살 밉니다. 공구로 세게 밀거나 두드리지 않습니다. 홈 벽이 얇아 옛 "
            "받침이 이렇게 부러졌습니다.",
            "서보를 손목 2에 넣은 뒤에는 이 너트에 손이 닿지 않습니다. 지금 끝까지 들어갔는지 꼭 봅니다.",
        ),
        check=("돋움 바깥 면의 구멍으로 들여다보면 너트 구멍이 가운데에 보인다 (둘 다).",),
        stuck="살살 밀어도 안 들어가면 그 자리를 사진으로 보내 주세요. 받침이 하나 더 있습니다 (여분).",
        adds=(CARRIER, CAR_NUTS),
        still=(CARRIER,),
        focus=(CARRIER, CAR_NUTS),
        uses=(Use("pencil", note="뻑뻑할 때만"),),
        moves=(
            Move((f"{CAR_NUTS} +z)",), _slide("서보 받침 +z")),
            Move((f"{CAR_NUTS} -z)",), _slide("서보 받침 -z"), 0.8),
        ),
        view=_v(View((0, 1, 0), (1, 0, 0), 30, 35)),
        labels=(Label(f"{CAR_NUTS} +z)", "M3 너트 × 2"),),
    ),
    Step(
        "j6-carrier",
        "받침 달기",
        "J6 서보에 새 받침 달기",
        do=(
            f"J6 서보(ID {J6_ID})를 혼이 아래로 가게 엎어 둔 그대로 둡니다.",
            "새 받침을 날개가 아래(혼 쪽)로 가게 들고, 판을 서보 뒷면에 얹습니다. 판의 작은 구멍 넷이 서보 뒷면의 "
            "나사 구멍 넷과 맞아야 합니다. 거꾸로 놓으면 구멍이 맞지 않습니다.",
            "앞 단계에서 모아 둔 서보 뒷면 나사 4개를 작은 십자 드라이버로 조입니다.",
        ),
        warn=(
            "나사를 다 조였는데 판이 흔들리면 나사가 서보 구멍 바닥에 먼저 닿은 것입니다. 머리 밑에 와셔를 하나 "
            "넣고, 넣었다고 알려 주세요.",
            "자가탭 나사는 세게 조이지 않습니다. 서보 구멍의 나사산이 뭉개질 수 있습니다.",
        ),
        check=("판이 서보 뒷면에 붙어 흔들리지 않는다.", "나사 머리 넷이 판에 고르게 앉았다."),
        stuck=PHOTO,
        mount=(CARRIER, CAR_NUTS),
        uses=(
            Use("carrier", note="앞 단계에서 너트 둘을 넣은 것", count=False),
            Use("selftap_so101", 4, "옛 받침에서 푼 것", count=False),
            Use("driver"),
            Use("washer_m3", 1, "예비: 판이 끝까지 안 조일 때"),
        ),
        focus=SERVO6_SET,
        moves=(
            Move((CARRIER, CAR_NUTS), _d((0.0, -25.0, 0.0))),
            Move((BACK6,), _d((0.0, -12.0, 0.0)), 1.4),
        ),
        view=_v(View((0, -1, 0), (0, 0, 1), 35, 20)),
        labels=(
            Label(CARRIER, "새 받침 (너트 둘)"),
            Label("J6 서보 뒷면 나사 1", "서보 뒷면 나사 × 4"),
        ),
    ),
    # ---- J6의 앞쪽 (손목 2를 책상 위에서 조립한다) ----
    Step(
        "bearing",
        "베어링",
        "베어링 넣기",
        do=(
            "손목 2를 뒤쪽(넓게 열린 쪽)이 위로 오게 잡습니다.",
            "베어링을 열린 쪽으로 넣습니다. 앞 끝 가까이까지는 저절로 내려갑니다.",
            "바깥 링을 엄지로 눌러, 앞 끝의 턱에 닿을 때까지 밉니다. 마지막 6.6 mm는 꾹 눌러야 들어갑니다 (첫 관절 "
            "때와 같습니다).",
        ),
        warn=(
            "바깥 링만 누릅니다. 안쪽 링을 누르면 베어링 속의 알이 상할 수 있습니다.",
            "엄지로 끝까지 안 들어가거나 넣은 뒤 무겁게 돌면 두드리거나 바이스로 누르지 않습니다. 사진을 보내 주세요.",
        ),
        check=(
            "베어링이 턱에 닿았다. 빙 둘러 깊이가 같고 기울지 않았다.",
            "안쪽 링을 손가락으로 돌리면 걸림 없이 부드럽게 돈다.",
        ),
        adds=(BEARING6,),
        moves=(Move((BEARING6,), _d((0.0, -62.0, 0.0))),),
        view=W2_BACK_UP,
        shell=(W2,),
        cut=True,
        labels=(Label(BEARING6, "베어링 6806"),),
        tags=(W2_BACK_TAG, W2_FOOT_TAG, W2_LIP_TAG),
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
            "뻑뻑하면 샤프 끝으로 살살 밀어 넣습니다 (첫 관절 때 이렇게 넣었습니다). 두드리지 않습니다.",
            "너트가 홈 끝까지 들어가야 다음 단계의 나사가 너트에 닿습니다.",
        ),
        check=("다리 바깥 면의 구멍으로 들여다보면 너트 구멍이 가운데에 보인다 (다리 셋 모두).",),
        stuck=PHOTO,
        adds=(RET6, RET_NUTS),
        still=(RET6,),
        focus=(RET6, RET_NUTS),
        uses=(Use("pencil", note="뻑뻑할 때만", count=False),),
        moves=(
            Move((f"{RET_NUTS} +z)",), _slide("누름 틀 +z")),
            Move((f"{RET_NUTS} +x)",), _slide("누름 틀 +x"), 0.8),
            Move((f"{RET_NUTS} -z)",), _slide("누름 틀 -z"), 1.6),
        ),
        view=_v(View((0, -1, 0), (1, 0, 0), 26, 36)),
        labels=(Label(f"{RET_NUTS} +x)", "M3 너트 × 3"),),
        tags=(
            Tag(_p((-(j.R_SLIDE + 2.0), j.RET_Y[1] + 2.0, 0.0)), "다리 없는 쪽"),
            Tag(_p((0.0, j.B0 + 1.5, j.R_SLIDE + 1.0)), "고리"),
        ),
    ),
    Step(
        "retainer-in",
        "틀 넣기",
        "누름 틀 넣기",
        do=(
            "손목 2를 뒤쪽(열린 쪽)이 위로 오게 잡습니다. 베어링이 아래에 있습니다.",
            "누름 틀을 고리가 아래로 가게 들고, 다리가 없는 쪽을 발(납작한 끝) 쪽으로 돌립니다.",
            "그대로 열린 쪽에 넣어, 고리가 베어링에 닿을 때까지 내립니다.",
        ),
        warn=(
            "다리가 벽의 나사 구멍 셋과 맞아야 다음 단계의 나사가 너트에 닿습니다. 넣기 전에 방향을 그림과 맞춥니다.",
        ),
        check=("아래쪽 줄(앞줄)의 벽 구멍 셋으로 들여다보면 너트 구멍이 가운데에 보인다.",),
        stuck=PHOTO,
        uses=(Use("retainer", note="앞 단계에서 너트 셋을 넣은 것", count=False),),
        mount=(RET6, RET_NUTS),
        moves=(Move((RET6, RET_NUTS), _d((0.0, -58.0, 0.0))),),
        view=W2_BACK_UP,
        shell=(W2,),
        cut=True,
        labels=(Label(RET6, "누름 틀 (너트 셋)"),),
        tags=(
            W2_BACK_TAG,
            W2_FOOT_TAG,
            Tag(_p((-(j.R_SLIDE + 3.0), j.RET_SCREW_Y, 0.0)), "다리 없는 쪽"),
        ),
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
            "손목 2를 흔들어도 베어링이 달그락거리지 않는다.",
        ),
        stuck=PHOTO,
        adds=("J6 벽 나사 1", "J6 벽 나사 3", "J6 벽 나사 5"),
        uses=(Use("hexkey"),),
        moves=(
            Move(("J6 벽 나사 1",), _d((0.0, 0.0, 14.0))),
            Move(("J6 벽 나사 5",), _d((14.0, 0.0, 0.0)), 0.7),
            Move(("J6 벽 나사 3",), _d((0.0, 0.0, -14.0)), 1.4),
        ),
        view=_v(View((0, -1, 0), (1, 0, 0), 16, 30)),
        shell=(W2,),
        ghost=True,
        labels=(
            Label("J6 벽 나사 1", "M3 × 8"),
            Label("J6 벽 나사 5", "M3 × 8"),
            Label("J6 벽 나사 3", "M3 × 8"),
        ),
        tags=(
            W2_BACK_TAG,
            Tag(_p((0.0, j.CAR_SCREW_Y, j.R_DRUM + 7.0)), "뒷줄: 지금은 비워 둠"),
        ),
    ),
    Step(
        "disc-nuts",
        "원판 너트",
        "플랜지 원판에 너트 넣기",
        do=(
            "플랜지 원판을 밑면(가운데가 둥글게 솟은 쪽)이 위로 오게 놓습니다. 귀가 둘 있는 원판입니다.",
            "테두리 가까이에 있는 육각 홈 넷에 너트를 하나씩 올립니다.",
            "엄지로 눌러 끝까지 끼웁니다.",
        ),
        warn=("엄지로 안 들어가면 두드리지 않습니다. 사진을 보내 주세요.",),
        check=("너트 넷이 모두 홈 속에 들어가, 면보다 튀어나오지 않는다.",),
        stuck=PHOTO,
        adds=(DISC, DISC_NUTS),
        still=(DISC,),
        focus=(DISC, DISC_NUTS),
        moves=tuple(
            Move((f"{DISC_NUTS} {k}",), _d((0.0, -10.0, 0.0)), 0.45 * (k - 1)) for k in range(1, 5)
        ),
        view=_v(View((0, -1, 0), (1, 0, 0), 50, 20)),
        labels=(Label(f"{DISC_NUTS} 1", "M3 너트 × 4"),),
        tags=(
            Tag(_p((j.R_POST + 7.0, j.SEAM - 3.0, 0.0)), "멈춤 귀 (기둥이 솟은 귀)"),
            Tag(_p((-(j.R_PIN + 7.0), j.SEAM - 3.0, 0.0)), "영점 핀 귀"),
            Tag(_p((0.0, j.B0 - 6.0, 0.0)), "밑면 (솟은 쪽)"),
        ),
    ),
    Step(
        "disc-pierce",
        "막 뚫기",
        "멈춤 귀의 막 뚫기",
        do=(
            "플랜지 원판을 뒤집어 윗면(매끈한 면)이 위로 오게 놓습니다.",
            "멈춤 귀(밑으로 기둥이 솟은 귀)의 둥근 머리 자리 ①에 얇은 막이 덮여 있습니다.",
            "육각렌치의 짧은 끝을 ①의 가운데에 대고 꾹 눌러 막을 뚫습니다. 렌치 끝이 구멍으로 쑥 들어가면 됩니다.",
        ),
        warn=(
            "막은 출력할 때 머리 자리의 천장을 받치려고 둔 한 층입니다. 뚫어도 부품에는 지장이 없습니다.",
            "영점 핀 귀의 구멍은 처음부터 뚫려 있습니다. 뚫을 것은 ① 하나입니다.",
        ),
        check=("①의 막을 뚫어 구멍이 아래까지 보인다.",),
        stuck=PHOTO,
        uses=(Use("flange_disc", note="너트 넷을 넣은 것", count=False), Use("hexkey")),
        focus=(DISC, DISC_NUTS),
        view=_v(View((0, 1, 0), (1, 0, 0), 50, 30)),
        zoom=1.4,
        center=_p((j.R_POST - 6.0, j.ATOP, 0.0)),
        spots=(Spot(DISC, "①", (_p((j.R_POST, j.ATOP, 0.0)),)),),
        tags=(POST_EAR_ONLY,),
    ),
    Step(
        "post",
        "멈춤 기둥",
        "멈춤 기둥 나사 조이기",
        do=(
            "윗면이 위인 채로, 멈춤 귀의 구멍(①)에 위에서 M3 × 10 나사를 꽂습니다.",
            "밑으로 나온 나사 끝에 너트를 손으로 돌려 끼웁니다.",
            "너트를 손가락으로 잡고 육각렌치로 나사를 조입니다.",
        ),
        warn=("나사가 막에 걸려 안 들어가면 ①의 막을 다시 뚫습니다.",),
        check=(
            "나사 머리가 윗면의 홈 속에 들어가 튀어나오지 않는다.",
            "너트가 기둥 끝에 붙어, 손으로 돌려도 돌지 않는다.",
        ),
        stuck=PHOTO,
        adds=POST,
        uses=(Use("hexkey"),),
        focus=(DISC, DISC_NUTS, *POST),
        moves=(
            Move(("J6 멈춤 나사",), _d((0.0, 16.0, 0.0))),
            Move(("J6 너트 (멈춤 기둥)",), _d((0.0, -12.0, 0.0)), 0.9),
        ),
        view=_v(View((0, 1, 0), (1, 0, 0), 14, 38)),
        labels=(
            Label("J6 멈춤 나사", "M3 × 10"),
            Label("J6 너트 (멈춤 기둥)", "M3 너트"),
        ),
        tags=(Tag(_p((-8.0, j.ATOP + 3.0, 0.0)), "윗면 (매끈한 면)"),),
    ),
    Step(
        "tool-pins",
        "도구 핀",
        "도구 핀 둘 눌러 박기",
        do=(
            "윗면에서 작은 구멍 둘을 찾습니다. 두 귀를 잇는 선과 직각인 자리이고, 바닥이 막힌 구멍입니다 (그림).",
            "도구 핀을 구멍에 세우고 손끝으로 끝까지 밀어 넣습니다. 바닥에 닿으면 6 mm쯤 남습니다.",
            "남은 구멍에도 핀을 똑같이 넣습니다.",
        ),
        warn=(
            "영점 핀 귀의 구멍(뚫린 구멍)에는 넣지 않습니다.",
            "손끝으로 끝까지 안 들어가면 두드리거나 펜치로 누르지 말고 사진을 보내 주세요.",
        ),
        check=(
            "핀 둘이 끝까지 들어가 윗면 위로 같은 높이(6 mm쯤)만큼 나왔다.",
            "손가락으로 당겨도 빠지지 않는다.",
        ),
        stuck=PHOTO,
        adds=(PINS,),
        focus=DISC_SET,
        moves=(
            Move((f"{PINS} 1",), _d((0.0, 20.0, 0.0))),
            Move((f"{PINS} 2",), _d((0.0, 20.0, 0.0)), 0.8),
        ),
        view=_v(View((0, 1, 0), (0, 0, 1), 35, 25)),
        labels=(Label(f"{PINS} 1", "도구 핀 × 2"),),
        tags=(POST_EAR_ONLY, PIN_EAR_ONLY),
    ),
    Step(
        "hub-plate",
        "허브 판",
        "허브 판 넣기",
        do=(
            "손목 2를 뒤쪽(열린 쪽)이 위로 오게 놓습니다.",
            "허브 판을 가운데가 볼록한 쪽이 위로 오게 들고, 열린 쪽으로 떨어뜨려 넣습니다.",
            "허브 판이 베어링 안쪽 링 위에 평평하게 얹혔는지 봅니다.",
        ),
        warn=("볼록한 쪽이 아래로 가면 서보의 혼이 제자리에 앉지 않습니다.",),
        check=("허브 판이 기울지 않고 베어링 안쪽 링 위에 얹혀 있다.",),
        stuck=PHOTO,
        adds=(HUB6,),
        moves=(Move((HUB6,), _d((0.0, -45.0, 0.0))),),
        view=W2_BACK_UP,
        shell=(W2,),
        cut=True,
        labels=(Label(HUB6, "허브 판 (볼록한 쪽이 위)"),),
        tags=(W2_BACK_TAG, W2_FOOT_TAG),
    ),
    Step(
        "servo-in",
        "서보 넣기",
        "J6 서보 넣기",
        do=(
            f"J6 서보(ID {J6_ID})를 혼이 아래로 가게 듭니다.",
            "서보의 긴 쪽(혼에서 먼 끝)을 손목 2의 발 쪽으로 돌려, 열린 쪽으로 넣습니다. 서보 받침의 날개가 벽을 "
            "따라 미끄러져 들어갑니다.",
            "혼이 허브 판의 볼록한 곳에 닿을 때까지 내립니다.",
        ),
        warn=(
            "벽 나사는 아직 조이지 않습니다. 혼 볼트를 먼저 조여 서보가 제자리를 찾은 뒤에 잠가야 서보 축에 억지 "
            "힘이 걸리지 않습니다.",
        ),
        check=(
            "벽의 뒷줄 구멍 둘로 서보 받침 돋움의 구멍이 보인다.",
            "서보가 기울지 않고 끝까지 들어갔다.",
        ),
        stuck=PHOTO,
        uses=(Use(J6_ITEM.key, note="새 받침을 단 것", count=False),),
        moves=(Move(SERVO6_SET, _d((0.0, -70.0, 0.0))),),
        view=W2_BACK_UP,
        shell=(W2,),
        cut=True,
        labels=(Label(SERVO6, f"J6 서보 (ID {J6_ID}, 새 받침)"),),
        tags=(W2_BACK_TAG, W2_FOOT_TAG),
    ),
    Step(
        "disc-on",
        "원판 끼우기",
        "뒤집어 플랜지 원판 끼우기",
        do=(
            "서보 받침을 엄지로 눌러 서보가 빠지지 않게 잡은 채, 손목 2를 뒤집어 앞쪽이 위로 오게 합니다.",
            "베어링 가운데 구멍으로 허브 판의 구멍 넷이 보입니다. 그 밑으로 혼의 나사 구멍이 보이게, 허브 판을 "
            "손끝이나 육각렌치 끝으로 돌려 맞춥니다.",
            "플랜지 원판을 핀 둘이 나온 면이 위로 가게 듭니다. 멈춤 귀(밑으로 기둥이 솟은 귀)는 발 반대쪽, 영점 핀 "
            "귀(구멍만 있는 귀)는 발 쪽에 오게 돌립니다.",
            "가운데 볼록한 곳을 베어링 안쪽 링에 끼우고, 원판을 조금씩 돌려 원판의 구멍 넷으로 혼의 나사 구멍이 "
            "보이게 맞춥니다.",
        ),
        warn=(
            "뒤집는 동안 서보 받침에서 손을 떼면 서보와 허브 판이 빠집니다.",
            "멈춤 귀를 발 쪽에 두면 멈춤 기둥이 목의 살에 걸려 원판이 들어가지 않습니다.",
            "원판이 안쪽 링에 손끝으로 들어가지 않으면 누르지 말고 사진을 보내 주세요. 도구 핀을 박은 뒤라 저널이 "
            "조금 부풀었을 수 있습니다.",
        ),
        check=(
            "원판이 안쪽 링에 끝까지 들어가 기울지 않았다.",
            "원판의 구멍 넷으로 혼의 나사 구멍이 보인다.",
        ),
        stuck=PHOTO,
        uses=(
            Use("flange_disc", note="너트, 멈춤 기둥, 도구 핀을 단 것", count=False),
            Use("phone", note="속이 어두우면 불빛"),
        ),
        mount=DISC_SET,
        moves=(Move(DISC_SET, _d((0.0, 40.0, 0.0))),),
        view=W2_FRONT_UP,
        zoom=1.3,
        labels=(Label(DISC, "플랜지 원판"),),
        tags=(W2_FOOT_TAG, POST_EAR_TAG, PIN_EAR_TAG),
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
            "속이 어두워 구멍이 안 보이면 휴대폰 불빛을 비춥니다. 구멍 속에 실이 보이면 핀셋으로 걷어 냅니다. 감으로 "
            "조이지 않습니다.",
            "나사가 손으로 걸리지 않고 헛돌거나 비스듬히 들어가면 빼서 구멍을 다시 맞춥니다.",
            "한 개만 끝까지 먼저 조이면 원판이 기울어 붙습니다. 넷을 고르게 조입니다.",
        ),
        check=(
            "나사 머리 넷이 원판 윗면에 닿았다.",
            "플랜지 원판이 위아래로도 옆으로도 흔들리지 않는다.",
        ),
        stuck="넷 가운데 하나라도 손으로 걸리지 않으면 그 구멍을 불빛을 비춰 찍어 보내 주세요.",
        adds=(BOLTS6,),
        uses=(Use("hexkey"), Use("tweezers", note="구멍 속의 실을 걷을 때")),
        moves=tuple(
            Move((f"{BOLTS6} M3x20 {k}",), _d((0.0, 22.0, 0.0)), 0.45 * (k - 1))
            for k in range(1, 5)
        ),
        view=_v(View((0, 1, 0), (1, 0, 0), 58, 20)),
        labels=(Label(f"{BOLTS6} M3x20 1", "M3 × 20 × 4"),),
    ),
    Step(
        "lock-servo",
        "서보 잠금",
        "뒷줄 벽 나사로 서보 잠그기",
        do=(
            "벽의 뒷줄(뒤쪽 열린 끝에 가까운 줄) 구멍 둘을 찾습니다. 구멍은 앞뒤로 조금 깁니다.",
            "모아 둔 M3 × 8 나사 둘을 하나씩 넣고 육각렌치로 조입니다. 머리가 벽에 닿아 더 돌지 않으면 멈춥니다.",
        ),
        warn=(
            "서보를 밀거나 당기지 않고 지금 자리 그대로 잠급니다.",
            "머리가 닿은 뒤로 세게 더 조이지 않습니다.",
        ),
        check=("나사 둘의 머리가 벽에 닿았다.", "서보를 뒤에서 밀어도 움직이지 않는다."),
        stuck=PHOTO,
        uses=(Use("m3x8", 2, "뒷줄에서 푼 것", count=False), Use("hexkey")),
        moves=(
            Move(("J6 벽 나사 2",), _d((0.0, 0.0, 14.0))),
            Move(("J6 벽 나사 4",), _d((0.0, 0.0, -14.0)), 0.7),
        ),
        view=_v(View((0, 1, 0), (1, 0, 0), 18, 40)),
        labels=(Label("J6 벽 나사 2", "M3 × 8"), Label("J6 벽 나사 4", "M3 × 8")),
        tags=(Tag(_p((0.0, j.CAR_SCREW_Y, j.R_DRUM + 8.0)), "뒷줄"),),
    ),
    Step(
        "turn6",
        "J6 돌려 보기",
        "J6 끝까지 돌려 보기",
        do=(
            "손목 2를 앞쪽(플랜지)이 위로 오게 놓고, 한 손으로 책상에 눌러 잡습니다.",
            f"다른 손으로 플랜지 원판을 그림처럼 천천히 한쪽 끝까지 돌립니다. J6 서보(ID {J6_ID})가 손으로 돌려집니다 "
            "(전원 없이).",
            "반대쪽 끝까지 천천히 돌린 뒤, 가운데로 돌려 둡니다.",
        ),
        warn=(SLOW, END_STOP),
        check=(
            "양쪽 끝까지 걸리는 곳 없이 돈다.",
            "양쪽 끝에서 멈춤 기둥이 살에 닿아 멈춘다.",
        ),
        stuck=PHOTO,
        uses=(Use(J6_ITEM.key, note="전원 없이 손으로 돌린다", count=False),),
        turn2=(-j.STOP_DEG, j.STOP_DEG),
        view=W2_FRONT_UP,
        tags=(W2_FOOT_TAG,),
    ),
    Step(
        "pin6",
        "J6 영점 핀",
        "J6 영점 핀 맞춰 보기",
        do=(
            "플랜지 원판을 가운데 자리로 돌려, 영점 핀 귀(구멍만 있는 귀)가 목의 살 위에 오게 합니다. 귀의 구멍으로 "
            "살의 구멍이 보이는 자리입니다.",
            "영점 핀을 귀의 구멍으로 넣어 살의 구멍까지 밉니다.",
            "끝까지 들어가는지 본 뒤 핀을 뺍니다. 손으로 안 빠지면 펜치나 롱노즈로 나온 끝을 잡고 곧게 당깁니다.",
        ),
        warn=(
            "핀이 안 들어가면 억지로 밀지 않습니다. 사진을 보내 주세요.",
            "핀이 꽂힌 채로 원판을 돌리지 않습니다.",
            "귀를 받침 삼아 핀을 젖히지 않습니다. 출력한 귀가 눌립니다.",
        ),
        check=("핀이 두 구멍을 지나 걸림 없이 들어갔다.", "핀을 뺐다."),
        stuck=PHOTO,
        adds=(ZERO_PIN,),
        uses=(Use("pliers"),),
        moves=(Move((ZERO_PIN,), _d((0.0, 24.0, 0.0))),),
        view=View((1, 0, 0), (0, -1, 0.5), 30, 0),
        zoom=1.3,
        labels=(Label(ZERO_PIN, "영점 핀 (첫 관절에 쓴 것)"),),
        tags=(PIN_EAR_TAG,),
    ),
    Step(
        "adapter",
        "어댑터 판",
        "도구 어댑터 판 맞춰 보기",
        do=(
            "도구 어댑터 판을 홈 넷이 파인 면이 아래(플랜지 쪽)로 가게 들고, 둥근 구멍과 긴 구멍을 도구 핀 둘에 맞춰 "
            "내려놓습니다.",
            "M3 × 10 나사 2개를 판의 나머지 구멍 둘에 넣어 육각렌치로 가볍게 조입니다. 원판 밑면의 너트에 물립니다.",
            "판이 플랜지 면에 붙어 흔들리지 않는지 본 뒤, 나사 둘을 풀어 판을 뗍니다. 판은 다음 도구(포인터)에 "
            "쓰니 보관하고, 나사 둘은 그리퍼를 달 때 다시 씁니다.",
        ),
        warn=("판이 핀에 손으로 안 들어가면 억지로 누르지 말고 사진을 보내 주세요.",),
        check=(
            "판이 핀 둘에 끼워지고, 나사 둘이 너트에 물렸다.",
            "판이 들뜨지 않고 플랜지 면에 붙었다.",
            "판을 떼어 보관했고, 나사 둘을 모아 두었다.",
        ),
        stuck=PHOTO,
        adds=(ADAPTER, THUMB),
        uses=(Use("hexkey"),),
        moves=(
            Move((ADAPTER,), _d((0.0, 20.0, 0.0))),
            Move((THUMB,), _d((0.0, 14.0, 0.0)), 1.2),
        ),
        view=W2_FRONT_UP,
        labels=(Label(ADAPTER, "도구 어댑터 판"), Label(f"{THUMB} M3x10 1", "M3 × 10 × 2")),
    ),
    # ---- 선과 뒷덮개 (손목 2를 J5 원판에 다시 달기 전에) ----
    Step(
        "grip-cable",
        "그리퍼 선",
        "그리퍼로 갈 300 mm 선 꽂고 놓기",
        do=(
            "손목 2를 뒤쪽(열린 쪽)이 위로 오게 놓습니다.",
            f"새 300 mm 서보 선의 한쪽 플러그를 J6 서보(ID {J6_ID}) 뒷면의 커넥터 둘 가운데 그림의 커넥터(①)에 "
            "꽂습니다. 손목 2 발의 귀 둘 가운데 영점 핀 구멍이 있는 귀 쪽입니다.",
            "선을 서보 받침 판 위에 눕혀 발 쪽으로 보내고(②), 서보 먼 끝의 검은 면을 따라 앞쪽으로 내려(③) 앞벽의 "
            "옆 홈(영점 핀 귀 쪽)으로 뺍니다.",
            "남는 선은 홈 밖으로 늘어뜨려 둡니다. 나중에 그리퍼에 꽂습니다.",
        ),
        warn=(
            "다른 커넥터는 비워 둡니다. 손목 1에서 온 선을 꽂을 자리입니다.",
            "서보 먼 끝 앞에서는 선을 목의 옆벽에 붙여 납작하게 눕힙니다. 손목 2를 J5 원판에 달면 그 자리가 2.6 mm "
            "틈이 되고, 선이 가운데로 오면 원판의 혼 볼트 머리에 눌립니다.",
        ),
        check=(
            "플러그가 끝까지 꽂혀 살짝 당겨도 빠지지 않는다.",
            "선이 앞벽의 옆 홈(영점 핀 귀 쪽)으로 나와 있다.",
        ),
        stuck="그림의 길이 헷갈리면 꽂은 자리와 선을 사진으로 보내 주세요.",
        adds=GRIP_IN_W2,
        view=_v(View((0, -1, 0), (-0.3, 0, 1), 32, 0)),
        zoom=1.7,
        shell=(W2,),
        ghost=True,
        labels=(
            Label(GRIP_IN_W2[1], "① 플러그"),
            Label(GRIP_IN_W2[0], "② 판 위에 눕혀", _p((-24.0, fj._WY[0], 4.95))),
            Label(GRIP_IN_W2[0], "③ 먼 끝 면 따라 앞쪽으로", _p((_XD, 10.0, wrist.SIDE_LANE))),
        ),
        tags=(W2_SIDE_SLOT,),
    ),
    Step(
        "j5-plug",
        "J5 선 꽂기",
        "손목 1에서 온 선 꽂기",
        do=(
            "손목 2를 뒤쪽이 위인 채로 손목 1 옆에 가져갑니다. 손목 1에 꽂힌 300 mm 선이 닿는 자리입니다.",
            f"그 선의 플러그를 J6 서보(ID {J6_ID})의 남은 커넥터(그림)에 꽂습니다.",
            "선을 서보 받침 판 위에 납작하게 눕혀 발 쪽으로 2 cm쯤 보냅니다. 덮개를 달면 덮개의 창으로 나갈 "
            "자리입니다.",
        ),
        warn=("선을 당기지 않게 손목 2를 손목 1 가까이에 둡니다.",),
        check=(
            "손목 1에서 온 선이 남은 커넥터에 끝까지 꽂혀 있다.",
            "커넥터 둘에 선이 하나씩 꽂혀 있다.",
        ),
        stuck=PHOTO,
        uses=(Use(CABLE_ITEM.key, note="손목 1에서 온 것", count=False),),
        moves=(Move(J5_IN_W2, _d((0.0, -12.0, 0.0))),),
        view=_v(View((0, -1, 0), (-1, 0, 1.1), 22, 0)),
        shell=(W2,),
        ghost=True,
        labels=(
            Label(J5_IN_W2[1], "손목 1에서 온 선"),
            Label(GRIP_IN_W2[1], "그리퍼 선 (꽂아 둔 것)"),
        ),
    ),
    Step(
        "d405-nuts",
        "받침 너트",
        "D405 받침 자리에 너트 넣기",
        do=(
            "손목 2를 뒤쪽이 위인 채로 둡니다. 원통 옆, 영점 핀 귀의 반대쪽에 네모난 돋움이 둘 있습니다.",
            "돋움마다 뒤 끝 면에 납작한 홈이 있습니다. M3 너트를 세워 하나씩 떨어뜨려 넣습니다.",
            "너트가 끝까지 내려가면 돋움 옆면의 나사 구멍으로 너트 구멍이 보입니다.",
        ),
        warn=(
            "덮개를 달기 전까지 손목 2를 뒤집지 않습니다. 너트가 빠집니다.",
            "너트가 안 내려가면 샤프 끝으로 살살 밉니다.",
        ),
        check=("돋움 둘의 옆면 구멍으로 너트 구멍이 가운데에 보인다.",),
        stuck=PHOTO,
        adds=(D_NUTS,),
        moves=(
            Move((f"{D_NUTS} 1",), _d((0.0, -14.0, 0.0))),
            Move((f"{D_NUTS} 2",), _d((0.0, -14.0, 0.0)), 0.8),
        ),
        view=View((-1, 0, 0), (0, 0.3, 1), 35, 0),
        zoom=1.6,
        center=_p((0.0, -12.0, -wrist.MOUNT_Z)),
        labels=(Label(f"{D_NUTS} 1", "M3 너트 × 2"),),
        tags=(Tag(_p((0.0, -12.0, -(wrist.MOUNT_Z + 6.0))), "돋움 (D405 받침 자리)"),),
    ),
    Step(
        "w2-cover",
        "덮개 달기",
        "손목 2에 뒷덮개 달기",
        do=(
            "떼어 둔 덮개를 테가 몸통 쪽으로 가게 듭니다. 덮개 한쪽 가장자리에 트인 창이 있습니다.",
            "손목 1에서 온 선을 눕힌 채 옆에서 창으로 밀어 넣으며 덮개를 얹습니다. 그리퍼로 가는 선은 덮개 밑으로 "
            "그대로 지나갑니다.",
            "모아 둔 M3 × 12 나사 2개를 넣고 육각렌치로 조입니다.",
        ),
        warn=(
            "선이 덮개 테 밑에 끼이지 않게 봅니다.",
            "덮개를 달면 돋움의 너트가 빠지지 않습니다. 그 뒤로는 손목 2를 뒤집어도 됩니다.",
        ),
        check=(
            "손목 1에서 온 선이 덮개의 창으로 나와 있다.",
            "덮개가 들뜨지 않고 원통 뒤 끝을 덮었다.",
        ),
        stuck=PHOTO,
        uses=(Use("m3x12", 2, "떼어 둔 것", count=False), Use("hexkey")),
        moves=(
            Move((COVER,), _d((0.0, -30.0, 0.0))),
            Move((COVER_SCREWS,), _d((0.0, -16.0, 0.0)), 1.0),
        ),
        view=_v(View((0, -1, 0), (-1, 0, 1.1), 22, 0)),
        zoom=1.5,
        labels=(
            Label(COVER, "뒷덮개 (떼어 둔 것)", _p((10.0, j.BACK_OUT, -12.0))),
            Label(J5_IN_W2[0], "창으로 나가는 선"),
        ),
    ),
    Step(
        "w2-on",
        "손목 2 달기",
        "손목 2를 J5 원판에 다시 달기",
        do=(
            "손목 1을 처음처럼 책 위에 세웁니다 (뒷덮개가 책에, J5 원판이 위).",
            "손목 2를 발이 아래로 가게 들고, 원통 앞 끝(플랜지 쪽)이 J5 원판의 멈춤 귀 쪽을 보게 돌립니다.",
            "발을 J5 원판 위에 얹고, 발 양옆 귀의 구멍 넷을 원판의 구멍 넷에 맞춥니다.",
            "모아 둔 M3 × 10 나사 4개를 위에서 넣고 육각렌치로 조입니다.",
        ),
        warn=(
            "원통 앞 끝을 귀 반대쪽으로 돌려 달면 J5의 영점과 D405 자리가 맞지 않습니다. 귀 쪽인지 다시 봅니다.",
            "두 선이 손목 2의 발과 원판 사이에 끼이지 않게 바깥으로 빼 둡니다. 그리퍼 선은 앞벽의 옆 홈으로 나와 "
            "있어야 합니다.",
        ),
        check=(
            "발이 원판 위에 들뜨지 않고 앉았다.",
            "나사 넷의 머리가 귀에 닿았다.",
            "두 선이 끼이지 않았다.",
        ),
        stuck=PHOTO,
        uses=(Use("m3x10", 4, "떼어 둔 것", count=False), Use("hexkey"), Use("book")),
        moves=(
            Move(W2_GROUP, (0.0, 45.0, 0.0)),
            Move((BODY_SCREWS,), (0.0, 14.0, 0.0), 1.2),
        ),
        view=View((0, 1, 0), (0.35, 0, 1), 22, 0),
        zoom=1.3,
        center=(0.0, 50.0, 0.0),
        labels=(
            Label(W2, "손목 2 (원통 앞 끝이 귀 쪽)", _p((0.0, j.FRONT, 0.0))),
            Label("몸통 나사 M3x10 1", "M3 × 10 × 4"),
        ),
        tags=(Tag((j.R_POST + 8.0, j.ATOP, 0.0), "J5 원판의 멈춤 귀"),),
    ),
    # ---- D405 ----
    Step(
        "d405-plug",
        "카메라 선",
        "D405에 선 꽂기",
        do=(
            "얇은 USB 2 선(1.55 m)의 ㄱ자 플러그를 D405 옆면의 커넥터에 끝까지 꽂습니다.",
            "선은 카메라의 바닥면(1/4 나사 구멍이 있는 면) 쪽으로 나가게 합니다.",
        ),
        warn=(
            "카메라를 손목에 단 뒤에는 플러그를 꽂을 틈이 없습니다. 지금 꽂습니다.",
            "꽂은 뒤 선을 꺾어 당기지 않습니다.",
        ),
        check=(
            "플러그가 끝까지 꽂혀 살짝 당겨도 빠지지 않는다.",
            "선이 바닥면(나사 구멍) 쪽으로 나간다.",
        ),
        stuck=PHOTO,
        adds=(D_CAM, D_PLUG, D_WIRE),
        still=(D_CAM,),
        focus=(D_CAM, D_PLUG, D_WIRE),
        moves=(Move((D_PLUG, D_WIRE), _cam_dir((0.0, -1.0, 0.0), 12.0)),),
        view=View((0, 1, 0), (0, 0.3, -1), 15, 25),
        labels=(Label(D_PLUG, "ㄱ자 플러그"),),
        tags=(CAM_LENS_TAG, CAM_BOTTOM_TAG, CAM_TOP_TAG),
    ),
    Step(
        "d405-cam",
        "카메라 달기",
        "D405를 받침에 달기",
        do=(
            "D405 받침의 큰 판에 카메라 뒷면(렌즈 반대쪽)을 댑니다.",
            "카메라의 플러그가 받침의 발(꺾인 짧은 쪽) 쪽에 오게 돌립니다. 반대로 돌려도 구멍은 맞지만 그러면 "
            "플러그가 바깥으로 나갑니다.",
            "판 뒤에서 M3 × 6 나사 2개를 넣어 카메라 뒷면의 구멍에 조입니다.",
        ),
        warn=("카메라 구멍은 깊이가 4 mm입니다. M3 × 6보다 긴 나사를 쓰지 않습니다.",),
        check=("카메라가 판에 붙어 흔들리지 않는다.", "플러그가 받침의 발 쪽에 있다."),
        stuck=PHOTO,
        adds=(D_MOUNT, CAM_SCREWS),
        still=(D_MOUNT,),
        focus=(D_MOUNT, D_CAM, D_PLUG, D_WIRE, CAM_SCREWS),
        uses=(Use("hexkey"),),
        moves=(
            Move((D_CAM, D_PLUG, D_WIRE), _cam_dir((0.0, 0.0, 1.0), 25.0)),
            Move((CAM_SCREWS,), _cam_dir((0.0, 0.0, -1.0), 14.0), 1.2),
        ),
        view=View((0, 1, 0), (-0.2, -0.5, 1), 20, 0),
        labels=(
            Label(D_MOUNT, "D405 받침 (발)"),
            Label(f"{CAM_SCREWS} 1", "M3 × 6 × 2"),
            Label(D_PLUG, "플러그는 발 쪽"),
        ),
        tags=(CAM_LENS_TAG, CAM_BOTTOM_TAG),
    ),
    Step(
        "pose-low",
        "눕혀 세우기",
        "가짜 아래팔 끝을 책상에 세우고 J5를 끝까지",
        do=(
            "가짜 아래팔 끝의 바깥 끝(납작한 면)을 책상에 대고 세웁니다. 관절 축이 옆으로 눕습니다.",
            f"손목 2를 그림처럼 천천히 끝까지 돌려(J5, ID {J5_ID} 서보), 손목 2의 뒷덮개가 위를 보게 합니다. 돋움의 "
            "너트가 홈 끝으로 "
            "내려갑니다.",
            "넘어지려 하면 한 손으로 가짜 아래팔 끝을 잡습니다.",
        ),
        warn=(SLOW, END_STOP),
        check=("손목 2의 뒷덮개가 위를 보고, 첫 관절이 넘어지지 않고 서 있다.",),
        stuck=PHOTO,
        uses=(Use("fake_forearm", note="바깥 끝을 책상에 댄다", count=False),),
        turn=(0.0, -j.STOP_DEG),
        view=LOW_D405,
        labels=(
            Label(FAKE, "가짜 아래팔 끝 (바깥 끝을 책상에)"),
            Label(COVER, "손목 2 뒷덮개 (위)"),
        ),
    ),
    Step(
        "d405-mount",
        "받침 달기",
        "D405 받침을 손목 2에 달기",
        do=(
            "카메라를 단 받침의 발을 손목 2의 돋움 둘에 댑니다. 카메라는 원통 옆, 플러그는 카메라와 원통 사이에 "
            "들어갑니다.",
            "M3 × 8 나사 2개를 발의 구멍에 넣어 돋움 속 너트에 조입니다.",
        ),
        warn=(
            "나사가 너트에 안 물리고 헛돌면, 손목 2의 뒷덮개가 위를 보는지 보고(너트가 홈 끝으로 내려간다) 다시 "
            "넣습니다.",
            "플러그가 원통이나 벽 나사 머리에 눌리면 멈추고 사진을 보내 주세요.",
        ),
        check=(
            "받침이 돋움에 붙어 흔들리지 않는다.",
            "플러그가 카메라와 원통 사이에 들어 눌리지 않았다.",
        ),
        stuck=PHOTO,
        adds=(D_SCREWS,),
        mount=(*D_SET, D_SCREWS),
        uses=(Use("hexkey"),),
        moves=(
            Move(D_SET, _d((0.0, 0.0, -25.0))),
            Move((D_SCREWS,), _d((0.0, 0.0, -14.0)), 1.2),
        ),
        joint=-j.STOP_DEG,
        view=LOW_D405,
        zoom=1.3,
        labels=(Label(D_CAM, "D405"), Label(f"{D_SCREWS} 1", "M3 × 8 × 2")),
    ),
    # ---- 평행 그리퍼 (책상 위에서) ----
    Step(
        "g-pierce",
        "그리퍼 막",
        "그리퍼 몸통과 피니언의 막 뚫기",
        do=(
            "그리퍼 몸통의 귀 둘(눕힌 끝벽 쪽) 나사 구멍 ①은 밑(머리 자리) 쪽에서, 돋움 둘의 나사 구멍 ②는 위(바닥판이 "
            "닿는 면) 쪽에서 육각렌치의 짧은 끝을 대고 꾹 눌러 막을 뚫습니다.",
            "②를 뚫은 부스러기는 돋움 옆면의 납작한 입구로 털어 냅니다.",
            "피니언의 나사 구멍 넷 ③은 넓게 파인 머리 자리 쪽에서 뚫습니다.",
        ),
        warn=("막은 출력할 때 천장을 받치려고 둔 한 층입니다. 뚫어도 부품에는 지장이 없습니다.",),
        check=("①, ②, ③의 막 여덟 곳을 모두 뚫었다.",),
        stuck=PHOTO,
        uses=(
            Use("gripper_frame", 1, count=False),
            Use("gripper_pinion", 1, count=False),
            Use("hexkey"),
        ),
        focus=(G_FRAME, G_PINION),
        place=(
            Place(G_FRAME, (0.0, 0.0, 0.0), (1, 0, 0), (0, 0.4, -1)),
            Place(G_PINION, (85.0, 0.0, 0.0), (1, 0, 0), (0, 0, -1)),
        ),
        view=View((0, 1, 0), (0, 0, 1), 30, 0),
        spots=(
            Spot(
                G_FRAME,
                "①",
                tuple(_p((-gp.TS[0], j.ATOP + gp.LUG_H0, s * gp.TS[1])) for s in (1, -1)),
            ),
            Spot(
                G_FRAME,
                "②",
                tuple(_p((gp.FS[0], j.ATOP + gp.H_FRONT, s * gp.FS[1])) for s in (1, -1)),
            ),
            Spot(
                G_PINION,
                "③",
                tuple(_p((x, j.ATOP + gp.PIN_FACE[1] - gp.HEAD_REC, z)) for x, z in j.bolt_xz()),
            ),
        ),
        labels=(Label(G_FRAME, "그리퍼 몸통"), Label(G_PINION, "피니언")),
    ),
    Step(
        "g-boss-nuts",
        "돋움 너트",
        "그리퍼 몸통의 돋움에 너트 넣기",
        do=(
            "그리퍼 몸통의 돋움 둘(눕힌 끝벽 반대쪽)에서 옆면의 납작한 입구를 찾습니다.",
            "M3 너트를 눕혀서 입구에 넣고 끝까지 밉니다. 위에서 뚫은 구멍으로 너트 구멍이 보이면 됩니다.",
        ),
        warn=("뻑뻑하면 샤프 끝으로 살살 밉니다. 세게 밀지 않습니다.",),
        check=("돋움 둘의 구멍으로 너트 구멍이 가운데에 보인다.",),
        stuck=PHOTO,
        adds=(G_FRAME, BOSS_NUTS),
        still=(G_FRAME,),
        focus=(G_FRAME, BOSS_NUTS),
        moves=(
            Move((f"{BOSS_NUTS} 1",), _d((10.0, 0.0, 0.0))),
            Move((f"{BOSS_NUTS} 2",), _d((10.0, 0.0, 0.0)), 0.8),
        ),
        view=G_DECK_DOWN,
        labels=(Label(f"{BOSS_NUTS} 1", "M3 너트 × 2 (눕혀서)"),),
    ),
    Step(
        "g-servo",
        "그리퍼 서보",
        "그리퍼 서보 넣기",
        do=(
            "그리퍼 몸통을 둥근 판(플랜지에 앉는 면)이 아래, 서보 자리가 위로 오게 놓습니다.",
            f"ID {GRIP_ID} 서보를 혼이 위로 가게 들고, 긴 쪽(혼에서 먼 끝)을 눕힌 끝벽(귀가 있는 쪽)으로 돌려 서보 "
            "자리에 넣습니다.",
            "서보 뒷면이 판 위의 받침 자리에 닿을 때까지 내립니다.",
        ),
        warn=(
            "혼이 아래로 가면 바닥판을 달 수 없습니다.",
            "반대로 돌리면 들어가지 않습니다. 억지로 밀지 않습니다.",
        ),
        check=("서보가 끝까지 들어가, 앞면(혼 쪽)이 몸통의 위 끝과 같은 높이다.",),
        stuck=PHOTO,
        adds=G_SERVO,
        focus=(G_FRAME, BOSS_NUTS, *G_SERVO),
        moves=(Move(G_SERVO, _d((0.0, 50.0, 0.0))),),
        view=G_DECK_DOWN,
        labels=(Label("그리퍼 서보", f"그리퍼 서보 (ID {GRIP_ID})"),),
    ),
    Step(
        "g-floor",
        "바닥판",
        "바닥판 달기",
        do=(
            "바닥판을 서보 앞면에 얹습니다. 가운데 둥근 구멍으로 혼이 나오고, 긴 네모 구멍에 서보 앞면의 튀어나온 "
            "라벨 판이 들어가게 방향을 맞춥니다.",
            "돋움 위의 구멍 둘에 M3 × 8 나사를 하나씩 넣어 조입니다. 돋움 속 너트에 물립니다.",
        ),
        warn=("바닥판을 반 바퀴 돌려 얹으면 라벨 판 구멍과 나사 구멍이 맞지 않습니다.",),
        check=(
            "바닥판이 들뜨지 않고 서보 앞면과 몸통 위 끝에 앉았다.",
            "나사 둘의 머리가 바닥판에 닿았다.",
        ),
        stuck=PHOTO,
        adds=(G_FLOOR, FLOOR_SCREWS),
        uses=(Use("hexkey"),),
        focus=(G_FRAME, BOSS_NUTS, *G_SERVO, G_FLOOR, FLOOR_SCREWS),
        moves=(
            Move((G_FLOOR,), _d((0.0, 20.0, 0.0))),
            Move((FLOOR_SCREWS,), _d((0.0, 14.0, 0.0)), 1.2),
        ),
        view=G_DECK_DOWN,
        labels=(Label(G_FLOOR, "바닥판"), Label(f"{FLOOR_SCREWS} M3x8 1", "M3 × 8 × 2")),
    ),
    Step(
        "g-pinion",
        "피니언",
        "피니언 달기",
        do=(
            "피니언을 넓게 파인 머리 자리가 위로 가게 들고, 가운데 목을 바닥판 구멍으로 넣어 혼 위에 얹습니다.",
            "피니언을 조금씩 돌려, 머리 자리 넷의 구멍으로 혼의 나사 구멍이 보이게 맞춥니다.",
            "M3 × 6 나사 4개를 넣고 대각선 차례로 조입니다.",
        ),
        warn=("M3 × 6보다 긴 나사를 쓰면 끝이 서보 속에 닿습니다.",),
        check=(
            "나사 머리 넷이 머리 자리 속에 들었다.",
            "피니언이 혼에 붙어 흔들리지 않는다.",
        ),
        stuck=PHOTO,
        adds=(G_PINION, PINION_SCREWS),
        uses=(Use("hexkey"),),
        focus=(G_FRAME, BOSS_NUTS, *G_SERVO, G_FLOOR, FLOOR_SCREWS, G_PINION, PINION_SCREWS),
        moves=(
            Move((G_PINION,), _d((0.0, 15.0, 0.0))),
            *(
                Move((f"{PINION_SCREWS} M3x6 {k}",), _d((0.0, 12.0, 0.0)), 1.2 + 0.4 * (k - 1))
                for k in range(1, 5)
            ),
        ),
        view=G_DECK_DOWN,
        labels=(Label(G_PINION, "피니언"), Label(f"{PINION_SCREWS} M3x6 1", "M3 × 6 × 4")),
    ),
    Step(
        "g-finger-nuts",
        "손가락 너트",
        "손가락에 너트 넣기",
        do=(
            "손가락 하나를 네모난 홈이 위로 오게 눕힙니다.",
            "M3 너트를 세워서 홈에 떨어뜨려 넣습니다. 홈 바닥까지 내려가면 됩니다.",
            "홈 입구에 테이프를 작게 한 조각 붙여 너트가 빠지지 않게 합니다. 남은 손가락도 똑같이 합니다.",
        ),
        check=("손가락 둘의 홈에 너트가 하나씩 들었고, 입구를 테이프로 막았다.",),
        stuck=PHOTO,
        adds=(*G_FINGERS, FINGER_NUTS),
        still=G_FINGERS,
        focus=(*G_FINGERS, FINGER_NUTS),
        uses=(Use("tape", 2, "작은 조각 (너트 홈 입구)"),),
        moves=(
            Move(("너트 (손가락) 1",), _d((8.0, 0.0, 0.0))),
            Move(("너트 (손가락) 2",), _d((-8.0, 0.0, 0.0)), 0.8),
        ),
        view=View((0, 1, 0), (0, 0, 1), 40, 0),
        labels=(Label("너트 (손가락) 1", "M3 너트 × 2"),),
    ),
    Step(
        "g-sliders",
        "받침 넣기",
        "랙 집에 손가락 받침 넣기",
        do=(
            "랙 집을 열린 쪽이 위, 긴 틈이 난 덮개가 아래로 오게 놓습니다.",
            "손가락 받침 둘을 이(톱니)가 집 가운데를 보게, 서로 반 바퀴 돌린 방향으로 넣습니다 (그림).",
            "받침마다 구멍 난 넓은 판은 집 가운데에, 긴 팔은 반대쪽 끝으로 갑니다. 두 판의 구멍이 덮개의 긴 틈 위에 "
            "옵니다.",
        ),
        check=("두 받침이 평평하게 들어가, 판의 구멍 둘이 덮개의 긴 틈 위에 있다.",),
        stuck=PHOTO,
        adds=(G_TUB, G_SLIDERS),
        still=(G_TUB,),
        focus=(G_TUB, G_SLIDERS),
        moves=(
            Move(("손가락 받침 1",), _d((0.0, -20.0, 0.0))),
            Move(("손가락 받침 2",), _d((0.0, -20.0, 0.0)), 0.8),
        ),
        view=G_TUB_OPEN_UP,
        zoom=1.4,
        labels=(
            Label(
                G_TUB, "랙 집 (열린 쪽이 위)", _p((gp.HOUSE_HL, j.ATOP + gp.H_COVER, gp.HOUSE_HW))
            ),
            Label("손가락 받침 1", "손가락 받침 × 2"),
        ),
    ),
    Step(
        "g-fingers",
        "손가락 달기",
        "손가락 달기",
        do=(
            "한 손으로 랙 집을 열린 쪽이 위인 채로 들고, 손가락 하나를 덮개 밑에서 긴 틈으로 밀어 올려 받침 판의 "
            "밑에 댑니다. 너트를 넣은 홈은 집의 끝 쪽을 봅니다.",
            "위(집 안)에서 M3 × 12 나사를 받침 판의 구멍에 넣어 손가락 속 너트에 조입니다.",
            "남은 손가락도 다른 받침에 똑같이 답니다. 다 조이면 홈 입구의 테이프를 뗍니다.",
        ),
        warn=("나사가 헛돌면 너트가 홈에서 빠진 것입니다. 손가락을 빼서 너트를 다시 넣습니다.",),
        check=(
            "손가락 둘이 받침에 붙어, 덮개를 사이에 두고 흔들리지 않는다.",
            "손가락을 밀면 받침과 같이 틈을 따라 미끄러진다.",
        ),
        stuck=PHOTO,
        adds=(FINGER_SCREWS,),
        uses=(Use("hexkey"), Use("tape", note="떼어 낸다", count=False)),
        focus=G_TUB_GROUP,
        moves=(
            Move(("손가락 1$", "너트 (손가락) 1"), _d((0.0, 25.0, 0.0))),
            Move(("손가락 나사 M3x12 1",), _d((0.0, -14.0, 0.0)), 1.0),
            Move(("손가락 2$", "너트 (손가락) 2"), _d((0.0, 25.0, 0.0)), 1.6),
            Move(("손가락 나사 M3x12 2",), _d((0.0, -14.0, 0.0)), 2.6),
        ),
        view=G_TUB_OPEN_UP,
        labels=(Label("손가락 1$", "손가락"), Label("손가락 나사 M3x12 1", "M3 × 12 × 2")),
    ),
    Step(
        "g-tub",
        "랙 집 달기",
        "랙 집을 바닥판 밑에 달기",
        do=(
            "두 손가락을 가운데로 밀어 끝이 맞닿게 합니다 (닫은 자리).",
            "그리퍼 몸통을 둥근 판이 위로 오게 들고, 랙 집을 바닥판 밑에 맞춥니다. 피니언이 두 받침의 이 사이로 "
            "들어가 맞물려야 집이 바닥판에 붙습니다.",
            "귀 쪽 구멍 둘에 M3 × 12, 반대쪽 끝의 구멍 둘에 M3 × 8 나사를 위에서 넣습니다. 넷 다 손으로 몇 바퀴 "
            "걸어 둔 뒤 육각렌치로 조입니다.",
        ),
        warn=(
            "M3 × 12는 몸통의 귀 쪽(눕힌 끝벽 쪽), M3 × 8은 반대쪽입니다. 바꿔 넣으면 길이가 맞지 않습니다.",
            "집이 바닥판에 붙지 않고 뜨면 이가 맞물리지 않은 것입니다. 손가락을 살짝 벌렸다 닫으며 다시 맞춥니다.",
            "나사는 랙 집의 플라스틱에 바로 조입니다. 머리가 닿은 뒤로 세게 조이지 않습니다.",
        ),
        check=("랙 집이 바닥판에 붙어 들뜨지 않는다.", "나사 넷의 머리가 닿았다."),
        stuck=PHOTO,
        adds=(END_SCREWS, LUG_SCREWS),
        uses=(Use("hexkey"),),
        focus=GRIPPER,
        moves=(
            Move(G_TUB_GROUP, _d((0.0, 25.0, 0.0))),
            Move((LUG_SCREWS,), _d((0.0, -14.0, 0.0)), 1.4),
            Move((END_SCREWS,), _d((0.0, -14.0, 0.0)), 2.0),
        ),
        view=G_DECK_UP,
        zoom=1.3,
        labels=(
            Label(f"{LUG_SCREWS} M3x12 1", "M3 × 12 × 2 (귀 쪽)"),
            Label(f"{END_SCREWS} M3x8 1", "M3 × 8 × 2"),
        ),
    ),
    Step(
        "g-slide",
        "손가락 보기",
        "손가락 움직여 보기",
        do=(
            f"그리퍼 서보(ID {GRIP_ID})에는 전원을 잇지 않습니다.",
            "두 손가락을 손끝으로 잡고 천천히 벌립니다. 둘이 같이 벌어지고, 서보가 손으로 돌려지느라 뻑뻑한 것이 "
            "정상입니다.",
            "끝까지 벌린 뒤 천천히 닫아, 손가락 끝이 맞닿게 둡니다.",
        ),
        warn=(
            "빨리 밀거나 세게 밀지 않습니다. 서보 기어와 피니언의 이가 상할 수 있습니다.",
            "끝에서 받침이 집의 끝벽에 닿으면 더 밀지 않습니다.",
        ),
        check=(
            "손가락 둘이 걸리는 곳 없이 같이 벌어지고 닫힌다.",
            "닫으면 손가락 끝이 맞닿는다.",
        ),
        stuck="걸리거나 한쪽만 움직이면 멈추고 사진을 보내 주세요.",
        uses=(Use("gripper_finger", note="손끝으로 잡고 민다", count=False),),
        focus=GRIPPER,
        moves=(
            Move(FINGER_1, _d((gp.STROKE, 0.0, 0.0))),
            Move(FINGER_2, _d((-gp.STROKE, 0.0, 0.0))),
        ),
        view=G_DECK_UP,
        labels=(Label("손가락 1$", "손가락"),),
    ),
    # ---- 그리퍼를 플랜지에 ----
    Step(
        "pose-mid",
        "가운데로",
        "손목 2를 가운데로 돌리기",
        do=(
            f"가짜 아래팔 끝을 책상에 세운 그대로, 손목 2를 천천히 가운데까지 돌립니다 (J5, ID {J5_ID} 서보).",
            "플랜지(핀 둘이 나온 면)가 위를 보면 멈춥니다.",
        ),
        warn=(SLOW,),
        check=("플랜지가 위를 본다.",),
        stuck=PHOTO,
        uses=(Use("fake_forearm", note="책상에 세운 그대로", count=False),),
        turn=(-j.STOP_DEG, 0.0),
        view=LOW,
        labels=(Label(DISC, "플랜지 (위)"),),
    ),
    Step(
        "g-plug",
        "그리퍼 선 꽂기",
        "그리퍼 서보에 선 꽂기",
        do=(
            "그리퍼를 둥근 판이 위로 오게 들고 손목 2 가까이 가져갑니다.",
            f"손목 2의 옆 홈에서 나온 그리퍼 선의 끝 플러그를, 판의 트인 자리로 넣어 그리퍼 서보(ID {GRIP_ID}) 뒷면의 "
            "커넥터 둘 가운데 그림의 커넥터에 꽂습니다. 눕힌 끝벽의 주머니와 같은 쪽입니다.",
        ),
        warn=(
            "다른 커넥터는 비워 둡니다.",
            "선을 당기지 않게 그리퍼를 손목 2 가까이에 둡니다.",
        ),
        check=("플러그가 끝까지 꽂혀 살짝 당겨도 빠지지 않는다.",),
        stuck=PHOTO,
        uses=(Use(CABLE_ITEM.key, note="손목 2의 옆 홈에서 나온 그리퍼 선", count=False),),
        adds=(G_PLUG,),
        focus=(*GRIPPER, G_PLUG),
        moves=(Move((G_PLUG,), _d((0.0, -12.0, 0.0))),),
        view=View((-1, 0, 0), (0, -0.6, -1), 35, 0),
        zoom=1.4,
        labels=(Label(G_PLUG, "꽂을 커넥터 (주머니 쪽)"),),
    ),
    Step(
        "g-pocket",
        "주머니",
        "그리퍼 선을 주머니에 넣기",
        do=(
            "판이 위인 채로, 꽂은 선을 서보 뒷면을 따라 눕힌 끝벽 쪽으로 보냅니다.",
            "눕힌 끝벽의 판 쪽 가장자리에 좁은 틈(①)이 있습니다. 선을 세워(옆으로 90° 비틀어) 이 틈으로 밀어 넣습니다.",
            "선이 틈 안쪽의 주머니(②)까지 들어가면 선을 다시 눕힙니다. 눕힌 선은 틈보다 넓어 틈으로 되나오지 "
            "않습니다.",
            "주머니 밖으로 나온 선은 그리퍼 몸통 옆으로 느슨하게 둡니다.",
        ),
        warn=(
            "선을 꺾지 않고 비틀기만 합니다.",
            "선이 판의 밑면(플랜지에 앉는 면) 밖으로 튀어나오면 그리퍼를 달 때 낍니다.",
        ),
        check=("선이 주머니에 눕혀져 있다.", "판의 밑면 밖으로 튀어나온 선이 없다."),
        stuck="틈으로 안 들어가면 선과 틈을 같이 찍어 보내 주세요.",
        uses=(Use(CABLE_ITEM.key, note="꽂은 그리퍼 선", count=False),),
        adds=(G_TAIL,),
        focus=(*GRIPPER, G_PLUG, G_TAIL),
        view=View((-1, 0, 0), (0, -1, -0.5), 40, 0),
        zoom=2.6,
        center=_p((-36.0, j.ATOP + 3.0, gp.EXIT_ZC)),
        spots=(
            Spot(G_FRAME, "①", (_p((-33.0, j.ATOP, gp.EXIT_ZC)),)),
            Spot(G_FRAME, "②", (_p((-38.0, j.ATOP + 5.5, gp.EXIT_ZC)),)),
        ),
        labels=(Label(G_TAIL, "그리퍼 선"),),
    ),
    Step(
        "g-mount",
        "그리퍼 달기",
        "그리퍼를 플랜지에 달기",
        do=(
            "그리퍼를 둥근 판이 아래로 가게 뒤집어 플랜지 위로 가져갑니다. 눕힌 끝벽(선이 든 주머니)을 플랜지의 "
            "영점 핀 귀(구멍만 있는 귀) 쪽으로 돌립니다.",
            "판의 둥근 구멍과 긴 구멍을 도구 핀 둘에 맞춰 내려놓습니다.",
            "모아 둔 M3 × 10 나사 2개를 판의 나머지 구멍 둘에 넣어 육각렌치로 조입니다.",
        ),
        warn=(
            "반 바퀴 돌려 달아도 핀과 구멍이 맞습니다. 그러면 J6를 끝까지 돌릴 때 그리퍼가 J5 원판의 귀에 닿습니다. "
            "주머니가 영점 핀 귀 쪽인지 꼭 봅니다.",
            "선이 판과 플랜지 사이에 끼이지 않게 봅니다.",
        ),
        check=(
            "판이 플랜지 면에 붙어 흔들리지 않는다.",
            "주머니가 영점 핀 귀 쪽에 있다.",
            "선이 끼이지 않았다.",
        ),
        stuck=PHOTO,
        mount=G_ALL,
        uses=(Use("m3x10", 2, "어댑터 판에서 뺀 것", count=False), Use("hexkey")),
        moves=(
            Move(G_ALL, _d((0.0, 45.0, 0.0))),
            Move((THUMB,), _d((0.0, 14.0, 0.0)), 1.2),
        ),
        view=LOW,
        labels=(
            Label(G_FRAME, "그리퍼 (주머니가 영점 핀 귀 쪽)"),
            Label(f"{THUMB} M3x10 1", "M3 × 10 × 2"),
        ),
    ),
    # ---- 손으로 돌려 보기 (서보 전원 없이) ----
    Step(
        "ff-off",
        "아래팔 떼기",
        "가짜 아래팔 끝 떼기",
        do=(
            "가짜 아래팔 끝의 옆면 홈 넷에 든 너트가 빠지지 않게, 홈 입구에 테이프를 작게 한 조각씩 붙입니다.",
            "손목 1 발 양옆 귀의 M3 × 12 나사 4개를 풀어 가짜 아래팔 끝을 뗍니다.",
            "가짜 아래팔 끝과 나사 넷은 모아 보관합니다.",
        ),
        warn=(
            "가짜 아래팔 끝(J4가 가운데인 자리)을 단 채로 돌리면, D405와 그리퍼가 그 관에 닿아 볼 자리까지 못 "
            "갑니다. 그래서 뗍니다.",
        ),
        check=("가짜 아래팔 끝을 뗐다.", "나사 넷과 가짜 아래팔 끝을 모아 두었다."),
        stuck=PHOTO,
        uses=(Use("hexkey"), Use("tape", 4, "작은 조각 (너트 홈 입구)")),
        view=View((0, 1, 0), (-0.35, 0, 1), 15, 0),
        labels=(Label(FAKE, "가짜 아래팔 끝 (뗀다)"), Label("발 나사 M3x12 1", "M3 × 12 × 4")),
    ),
    Step(
        "d405-route",
        "D405 선",
        "D405 선 놓기",
        do=(
            "손목 1을 뒷덮개가 책상에 닿게 세웁니다 (손목 2가 위).",
            "D405 플러그에서 나온 선을 손목 2 옆을 따라 손목 1 쪽으로 내립니다. 두 몸통 사이(관절의 틈)로 넣지 "
            "말고 바깥으로 넘깁니다.",
            "나머지 선은 손목 1의 발(가짜 아래팔 끝을 뗀 자리) 쪽으로 늘어뜨려 책상에 둡니다.",
        ),
        warn=(
            "선을 테이프로 붙이지 않습니다.",
            "팽팽하게 당겨 두지 않습니다. 관절을 돌릴 때 고리가 생길 만큼 느슨하게 둡니다.",
        ),
        check=("D405 선이 관절 바깥으로 넘어 손목 1의 발 쪽으로 늘어져 있다.",),
        stuck="선을 놓은 모습이 그림과 다르면 사진을 보내 주세요.",
        uses=(Use(USB_ITEM.key, note="카메라에 꽂은 것", count=False),),
        view=STAND_D405,
        labels=(Label(D_WIRE, "① D405 선: 여기서 손목 1 쪽으로"),),
        tags=(
            Tag((0.0, 31.0, 32.0), "② 관절 바깥으로 넘긴다"),
            Tag((wrist.X_FOOT - 6.0, 0.0, 26.0), "③ 손목 1의 발 쪽으로 늘어뜨린다"),
        ),
    ),
    Step(
        "t-start",
        "가운데 사진",
        "가운데에서 찍기",
        do=(
            f"J5(ID {J5_ID} 서보)와 J6(ID {J6_ID} 서보)를 가운데에 둡니다 (그림). 서보 전원은 잇지 않습니다.",
            "한 손으로 손목 1을 책상에 눌러 잡고, 다른 손으로 휴대폰을 듭니다.",
            SHOOT,
        ),
        check=("가운데 사진 한 장을 보냈다.",),
        stuck=PHOTO,
        uses=(Use("phone"),),
        view=STAND_D405,
        labels=(
            Label(D_WIRE, "D405 선"),
            Label(
                J5_CABLE_W1[0], "J5 선 (옆 홈에서)", (wrist.X_FOOT + 1.2, j.FRONT, wrist.SIDE_LANE)
            ),
        ),
    ),
    Step(
        "t-j6",
        "J6 멈춤 귀",
        "J6: 그리퍼 선이 J5 멈춤 귀 위를 지나는 자리",
        do=(
            f"한 손으로 손목 2를 잡고, 다른 손으로 그리퍼를 그림처럼 천천히 돌립니다 (J6, ID {J6_ID} 서보).",
            "그리퍼 선이 J5 원판의 멈춤 귀(손목 2 발 밑의 귀) 바로 위를 지나는 자리에서 멈춥니다.",
            SHOOT,
        ),
        warn=(SLOW, HOLD),
        check=("멈춘 자리 사진 한 장을 보냈다.",),
        stuck=PHOTO,
        uses=(Use("phone"),),
        turn2=(0.0, 45.0),
        view=View((0, 1, 0), (0.6, 0.3, -1), 15, 0),
        zoom=1.8,
        center=J5_EAR_AT,
        labels=(
            Label("J5 출력 원판", "J5 멈춤 귀", J5_EAR_AT),
            Label(GRIP_IN_W2[0], "그리퍼 선"),
        ),
    ),
    Step(
        "t-j6-ends",
        "J6 끝까지",
        "J6: 끝까지 돌리며 찍기",
        do=(
            "그리퍼를 그림처럼 한쪽 끝까지 천천히 돌리고 한 장 찍습니다.",
            "반대쪽 끝까지 천천히 돌리고 한 장 더 찍습니다.",
            "주머니에 눕힌 선이 빠지지 않았는지 보고, 그리퍼를 가운데로 돌려 둡니다.",
        ),
        warn=(SLOW, END_STOP),
        check=("양 끝 사진 두 장을 보냈다.", "주머니의 선이 그대로 있다."),
        stuck=PHOTO,
        uses=(Use("phone"),),
        turn2=(-j.STOP_DEG, j.STOP_DEG),
        view=View((0, 1, 0), (0.6, 0.3, -1), 15, 0),
        zoom=1.3,
        center=J5_EAR_AT,
        labels=(Label(G_TAIL, "주머니의 선"),),
    ),
    Step(
        "t-j5",
        "J5: D405 선",
        "J5: D405 선과 J5 선",
        do=(
            "그리퍼(J6)는 가운데에 둡니다.",
            "한 손으로 손목 1을 책상에 눌러 잡고, 다른 손으로 손목 2를 위에서 볼 때 시계 방향으로 천천히 돌립니다 "
            f"(J5, ID {J5_ID} 서보, 그림).",
            "그림이 멈춘 자리(D405가 손목 1의 발 쪽 위에 온 자리)에서 멈추고 찍습니다.",
            SHOOT,
        ),
        warn=(SLOW, HOLD),
        check=("멈춘 자리 사진 한 장을 보냈다.",),
        stuck=PHOTO,
        uses=(Use("phone"),),
        turn=(0.0, 100.0),
        view=View((0, 1, 0), (-1, 0, 0.45), 18, 0),
        zoom=1.6,
        center=(-30.0, 40.0, 5.0),
        labels=(
            Label(D_WIRE, "D405 선"),
            Label(
                J5_CABLE_W1[0], "J5 선 (옆 홈에서)", (wrist.X_FOOT + 1.2, j.FRONT, wrist.SIDE_LANE)
            ),
        ),
    ),
    Step(
        "t-plus-a",
        "J5 + 끝 ①",
        "J5를 끝까지, J6를 한쪽 끝까지",
        do=(
            "손목 2를 같은 쪽(위에서 볼 때 시계 방향)으로 끝까지 천천히 돌립니다.",
            "그리퍼를 그림처럼 한쪽 끝까지 돌립니다.",
            "도구 끝에 남는 두 선의 고리가 보이게 " + SHOOT,
        ),
        warn=(SLOW, END_STOP, "고리가 두 몸통 사이에 끼면 더 돌리지 말고 그 자리를 찍습니다."),
        check=("사진 한 장을 보냈다.",),
        stuck=PHOTO,
        uses=(Use("phone"),),
        turn=(100.0, j.STOP_DEG),
        turn2=(0.0, -j.STOP_DEG),
        view=View((0, 1, 0), (-0.3, 0, 1), 25, 0),
        zoom=1.8,
        center=(-20.0, 40.0, 15.0),
        labels=(Label(GRIP_IN_W2[0], "그리퍼 선"),),
    ),
    Step(
        "t-plus-b",
        "J5 + 끝 ②",
        "J5는 끝에 둔 채, J6를 반대쪽 끝까지",
        do=(
            "손목 2는 그대로 둡니다.",
            "그리퍼를 반대쪽 끝까지 천천히 돌립니다.",
            "도구 끝에 남는 두 선의 고리가 보이게 " + SHOOT,
        ),
        warn=(SLOW, END_STOP, "고리가 두 몸통 사이에 끼면 더 돌리지 말고 그 자리를 찍습니다."),
        check=("사진 한 장을 보냈다.",),
        stuck=PHOTO,
        uses=(Use("phone"),),
        joint=j.STOP_DEG,
        turn2=(-j.STOP_DEG, j.STOP_DEG),
        view=View((0, 1, 0), (-0.3, 0, 1), 25, 0),
        zoom=1.8,
        center=(-20.0, 40.0, 15.0),
        labels=(Label(GRIP_IN_W2[0], "그리퍼 선"),),
    ),
    Step(
        "t-minus-a",
        "J5 - 끝 ①",
        "J5를 반대쪽 끝까지, J6를 한쪽 끝까지",
        do=(
            "손목 2를 반대쪽(위에서 볼 때 반시계 방향)으로, 가운데를 지나 끝까지 천천히 돌립니다.",
            "그리퍼를 그림처럼 한쪽 끝까지 돌립니다.",
            "도구 끝에 남는 두 선의 고리가 보이게 " + SHOOT,
        ),
        warn=(SLOW, END_STOP, "고리가 두 몸통 사이에 끼면 더 돌리지 말고 그 자리를 찍습니다."),
        check=("사진 한 장을 보냈다.",),
        stuck=PHOTO,
        uses=(Use("phone"),),
        turn=(j.STOP_DEG, -j.STOP_DEG),
        turn2=(j.STOP_DEG, -j.STOP_DEG),
        view=View((0, 1, 0), (-1, 0, 0.3), 25, 0),
        zoom=1.8,
        center=(-25.0, 40.0, 0.0),
        labels=(Label(GRIP_IN_W2[0], "그리퍼 선"),),
    ),
    Step(
        "t-minus-b",
        "J5 - 끝 ②",
        "J5는 끝에 둔 채, J6를 반대쪽 끝까지",
        do=(
            "손목 2는 그대로 둡니다.",
            "그리퍼를 반대쪽 끝까지 천천히 돌립니다.",
            "도구 끝에 남는 두 선의 고리가 보이게 " + SHOOT,
        ),
        warn=(SLOW, END_STOP, "고리가 두 몸통 사이에 끼면 더 돌리지 말고 그 자리를 찍습니다."),
        check=("사진 한 장을 보냈다.",),
        stuck=PHOTO,
        uses=(Use("phone"),),
        joint=-j.STOP_DEG,
        turn2=(-j.STOP_DEG, j.STOP_DEG),
        view=View((0, 1, 0), (-1, 0, 0.3), 25, 0),
        zoom=1.8,
        center=(-25.0, 40.0, 0.0),
        labels=(Label(GRIP_IN_W2[0], "그리퍼 선"),),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "손목 끝 조립과 돌려 보기가 끝났습니다. J5와 J6를 가운데로 돌려 둡니다.",
            "보내 주신 사진으로 Claude가 선이 만나거나 끼이는지 보고 알려 드립니다.",
            "떼어 둔 가짜 아래팔 끝은 홈의 테이프를 붙인 채 나사와 같이 보관합니다.",
            "헷갈렸던 단계가 있으면 그 단계의 메모에 적어 주세요. 안내를 고칩니다.",
        ),
        view=STAND_FRONT,
    ),
)

# 떼어 둔 동안 숨기는 것: (선택자, 떼는 단계, 다시 다는 단계). 떼는 단계 다음부터 다시 다는 단계 앞까지 숨긴다
# (다시 다는 단계에서 움직임과 함께 보인다). 다시 달지 않으면 끝까지 숨긴다
W1_SIDE = (W1, "J5 ", *FAKE_SET, BODY_SCREWS)
_OFF: tuple[tuple[tuple[str, ...], str, str | None], ...] = (
    ((COVER, COVER_SCREWS), "w2-open", "w2-cover"),
    (J5_IN_W2, "j6-unplug", "j5-plug"),
    (LOCK, "j6-out", "lock-servo"),
    (SERVO6_SET, "j6-out", "j6-old"),  # 손목 2에서 뺀 서보: 받침을 바꾸는 동안만 따로 보인다
    (SERVO6_SET, "j6-carrier", "servo-in"),
    (W1_SIDE, "w2-off", "w2-on"),
    ((OLD_CARRIER,), "j6-old", None),
    ((ZERO_PIN,), "pin6", None),
    ((ADAPTER,), "adapter", None),
    ((THUMB,), "adapter", "g-mount"),
    (FAKE_SET, "ff-off", None),
)


def _with_hidden(steps: tuple[Step, ...]) -> tuple[Step, ...]:
    order = {s.id: k for k, s in enumerate(steps)}
    out: list[Step] = []
    for k, s in enumerate(steps):
        hide: list[str] = []
        for sels, off, on in _OFF:
            end = order[on] if on is not None else len(steps)
            if order[off] < k < end:
                hide += [x for x in sels if x not in hide]
        out.append(replace(s, hide=tuple(hide)) if hide else s)
    return tuple(out)


STEPS = _with_hidden(STEPS_RAW)

GUIDE = Guide(
    id="wrist-end",
    kind="assembly",
    title="손목 끝 조립",
    sub=(
        f"J6 · ID {J6_ID} 서보 받침 바꾸기와 앞쪽 · 그리퍼 ID {GRIP_ID} · D405 · J5와 J6 손으로 돌려 보기"
    ),
    scene=scene,
    steps=STEPS,
    items={
        **PRINTED,
        ASSEMBLED.key: ASSEMBLED,
        M3X6.key: M3X6,
        BEARING_ITEM.key: BEARING_ITEM,
        J6_ITEM.key: J6_ITEM,
        GRIP_ITEM.key: GRIP_ITEM,
        CABLE_ITEM.key: CABLE_ITEM,
        USB_ITEM.key: USB_ITEM,
        CAMERA_ITEM.key: CAMERA_ITEM,
        TOOL_PIN.key: TOOL_PIN,
        **{it.key: it for it in TOOLS},
    },
    bundle="wrist-end",
    printed_before=("carrier",),
    axis2=AXIS2,
)
