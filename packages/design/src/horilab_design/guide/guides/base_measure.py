"""베이스·어깨 전 재기: 베이스와 어깨를 뽑기 전에 실물로 잴 것을 한 안내로 모은다 (status.md의 사용자 순서 2).

어깨는 2026-11-15에 잰 ST-3120으로 그렸고 남은 값(혼 가운데 돋움과 나사 머리의 지름, 옆면 플러그와 꺾인 선,
뒷면 구멍 간격과 몸체 단면)은 가정이다 (parts/shoulder.py의 ASSUMED). 베이스의 파이 덮개 기둥, 클램프 위 턱,
책상 뒤 틈, 꽂을 플러그, 타이 구멍도 가정으로 그렸다. 값은 사용자가 대화창에 적고 Claude가 data/measured.toml에
넣는다.

장면은 물건마다 따로 놓고(단계마다 그 물건만 보인다) x를 따라 늘어놓는다. ST-3120은 서보 좌표 그대로(y가 출력축,
옆면 플러그가 +z)이고, 나머지는 y가 위다. 서보와 클램프 말고는 모양이 코드에 없어서 어림 모양으로 그렸다 (재는
자리를 가리키는 그림이다).
"""

from __future__ import annotations

from build123d import Compound

from horilab_design.guide.guides import first_joint as fj
from horilab_design.guide.model import (
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
from horilab_design.parts import base as bs
from horilab_design.parts import shoulder as sh
from horilab_design.parts.shapes import Point, box, cyl_y, rod, solid

Off = tuple[float, float, float]


def _at(o: Off, x: float, y: float, z: float) -> Point:
    return (o[0] + x, o[1] + y, o[2] + z)


def _box(o: Off, x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Compound:
    return box(x0 + o[0], x1 + o[0], y0 + o[1], y1 + o[1], z0 + o[2], z1 + o[2])


def _cy(o: Off, r: float, y0: float, y1: float, x: float, z: float) -> Compound:
    return cyl_y(r, y0 + o[1], y1 + o[1], x + o[0], z + o[2])


def _union(shapes: list[Compound]) -> Compound:
    out = shapes[0]
    for s in shapes[1:]:
        out = solid(out + s)
    return out


# ---- 물건 자리 (장면 좌표의 원점) ----
O_CASE: Off = (170.0, -28.0, 0.0)
O_CLAMP: Off = (330.0, 40.0, 0.0)
O_DESK: Off = (520.0, 0.0, 0.0)
O_PLUG: Off = (700.0, 0.0, 0.0)
O_USB: Off = (780.0, 0.0, 120.0)
O_TIE: Off = (1060.0, 0.0, 0.0)
O_ROLL: Off = (1330.0, -28.0, 0.0)
O_SCREW: Off = (1480.0, 0.0, 0.0)

# ---- 아크릴 케이스 (어림: 판 100 × 70 × 3, 귀퉁이 구멍 90 × 60, 기둥 20, 팬 30 × 30 × 7) ----
PL_HX, PL_HZ, PL_T = 50.0, 35.0, 3.0
HOLE_X, HOLE_Z, HOLE_R = 45.0, 30.0, 1.6
POST_R, POST_L = 2.5, 20.0
FAN, FAN_T, FAN_HOLE = 15.0, 7.0, 12.0  # 반변, 두께, 구멍의 반간격
LEVELS = (0.0, PL_T + POST_L, 2 * (PL_T + POST_L))  # 쌓인 판 셋의 밑면
FAN_Y0 = LEVELS[2] + PL_T  # 맨 위 판 위에 얹힌 팬의 밑면
SPARE_Z = 110.0  # 남는 판과 팬은 쌓인 케이스 앞줄에 둔다
SPARE_FAN_X, SPARE_FAN_Z = 95.0, 0.0  # 남는 팬은 쌓인 케이스 오른쪽 바닥에 둔다

# ---- 책상과 벽 (어림 틈 45) ----
GAP = 45.0

# ---- 플러그 셋: (이름, 품목 키, 단자에 들어가는 쇠 (길이, 반두께, 반폭), 몸통 (길이, 두께, 폭), 선 반지름, 곧은 꼬리)
PLUGS = (
    (
        "pi-power",
        "파이 전원 USB-C",
        "pi_power",
        (6.5, 1.25, 4.2),
        (22.0, 7.0, 12.0),
        1.8,
        6.0,
        -55.0,
    ),
    ("lan", "랜", "lan_cable", (14.0, 4.0, 5.85), (16.0, 10.0, 14.0), 3.0, 8.0, 0.0),
    ("d405-usba", "D405 USB-A", "usb2_cable", (12.0, 2.25, 6.0), (18.0, 8.0, 16.0), 1.5, 6.0, 55.0),
)

# ---- 이름 ----
ST_BODY = "ST-3120 몸체"
ST_HORN = "ST-3120 혼"
ST_PLUG = "ST-3120 플러그"
ST_WIRE = "ST-3120 선"
STACK = "쌓인 아크릴 판"
POSTS = "쌓인 케이스 기둥"
CASE_SCREWS = "쌓인 케이스 나사"
STACK_FAN = "쌓인 케이스 팬"
SPARE_PLATE = "남는 아크릴 판"
SPARE_FAN = "남는 팬"
CLAMP = "금속 클램프"
DESK = "책상 상판"
WALL = "벽"
USB_AC = "짧은 USB A–C 선"
TIE = "찍찍이 타이"
SCALE_PART = "저울"
ROLL = "검정 롤"
SCREWS = "SO-101 나사"


def _plug_names(label: str) -> tuple[str, str, str]:
    return (f"{label} 플러그", f"{label} 선", f"{label} 꽂을 기기")


ST_ITEM = Item("st3120", "ID 2 서보 (ST-3120)", "bought", "어깨 서보")
LEAD = Item("servo_lead", "서보 선 하나", "bought", "3핀이면 아무거나", icon="wire")
CASE_ITEM = Item("acrylic_case", "적층 아크릴 케이스", "bought", "지금 쌓여 있는 것")
SPARE_ITEM = Item("acrylic_spare", "남는 아크릴 판과 팬", "bought", "하나씩")
CLAMP_ITEM = Item("clamp", "금속 클램프", "bought", "베이스를 책상에 무는 것")
DESK_ITEM = Item("desk", "책상과 벽", "bought", "로봇을 놓을 자리")
PORT_ITEM = Item("port_device", "꽂을 기기", "bought", "파이, 공유기, PC")
PI_POWER = Item("pi_power", "파이 USB-C 전원선", "bought", "어댑터에 달린 선", icon="wire")
LAN_ITEM = Item("lan_cable", "랜선", "bought", "아무거나 하나", icon="wire")
D405_CABLE = Item("usb2_cable", "얇은 USB 2 선", "bought", "D405 선 (1.55 m)", icon="wire")
USB_AC_ITEM = Item("usb_ac_short", "짧은 USB A–C 선", "bought", "있으면", icon="wire")
TIE_ITEM = Item("velcro_tie", "찍찍이 케이블 타이", "bought", "샀으면")
ROLL_ITEM = Item("black_roll", "검정 필라멘트 첫 롤", "bought", "스풀째")
SCREW_ITEM = Item("so101_screw", "옛 SO-101 서보 나사", "hardware", "작은 십자 자가탭")
CALIPER = Item("caliper", "캘리퍼스", "tool", "바깥 턱, 안쪽 턱", icon="caliper")
RULER = Item("ruler", "자나 줄자", "tool", "긴 길이", icon="ruler")
SCALE = Item("scale", "저울", "tool", "필라멘트 롤", icon="scale")
PHONE = Item("phone", "휴대폰", "tool", "사진")


def _servo() -> list[ScenePart]:
    sv = sh.servo()
    holes = [cyl_y(1.25, -sh.HT - 0.5, -sh.HT + 3.0, x, z) for x, z in sh.BACK_HOLES]
    body = sv["body"]
    for h in holes:
        body = solid(body - h)
    return [
        ScenePart(ST_BODY, body, ST_ITEM.key, "servo"),
        ScenePart(ST_HORN, sv["horn"], ST_ITEM.key, "metal", count=False),
        ScenePart(ST_PLUG, sv["plug"], LEAD.key, "plug"),
        ScenePart(ST_WIRE, sv["wire"], LEAD.key, "wire", count=False),
    ]


def _plate(o: Off, y0: float, z: float, fan_hole: bool) -> Compound:
    p = _box(o, -PL_HX, PL_HX, y0, y0 + PL_T, z - PL_HZ, z + PL_HZ)
    for sx in (-1, 1):
        for sz in (-1, 1):
            p = solid(p - _cy(o, HOLE_R, y0 - 1, y0 + PL_T + 1, sx * HOLE_X, z + sz * HOLE_Z))
    if fan_hole:
        p = solid(p - _cy(o, 13.0, y0 - 1, y0 + PL_T + 1, 0.0, z))
    return p


def _fan(o: Off, x: float, y0: float, z: float) -> Compound:
    f = _box(o, x - FAN, x + FAN, y0, y0 + FAN_T, z - FAN, z + FAN)
    f = solid(f - _cy(o, 13.5, y0 + 1.0, y0 + FAN_T + 1, x, z))
    f = solid(f + _cy(o, 6.0, y0 + 1.0, y0 + FAN_T - 0.5, x, z))
    for sx in (-1, 1):
        for sz in (-1, 1):
            f = solid(f - _cy(o, 1.6, y0 - 1, y0 + FAN_T + 1, x + sx * FAN_HOLE, z + sz * FAN_HOLE))
    return f


def _case() -> list[ScenePart]:
    o = O_CASE
    plates = _union([_plate(o, y, 0.0, i == 2) for i, y in enumerate(LEVELS)])
    posts = _union(
        [
            _cy(o, POST_R, LEVELS[k] + PL_T, LEVELS[k + 1], sx * HOLE_X, sz * HOLE_Z)
            for k in (0, 1)
            for sx in (-1, 1)
            for sz in (-1, 1)
        ]
    )
    top = LEVELS[2] + PL_T
    screws = _union(
        [_cy(o, 2.6, top, top + 1.6, sx * HOLE_X, sz * HOLE_Z) for sx in (-1, 1) for sz in (-1, 1)]
    )
    return [
        ScenePart(STACK, plates, CASE_ITEM.key, "cap"),
        ScenePart(POSTS, posts, CASE_ITEM.key, "metal", count=False),
        ScenePart(CASE_SCREWS, screws, CASE_ITEM.key, "metal", count=False),
        ScenePart(STACK_FAN, _fan(o, 0.0, FAN_Y0, 0.0), CASE_ITEM.key, "servo", count=False),
        ScenePart(SPARE_PLATE, _plate(o, 0.0, SPARE_Z, True), SPARE_ITEM.key, "cap"),
        ScenePart(
            SPARE_FAN, _fan(o, SPARE_FAN_X, 0.0, SPARE_FAN_Z), SPARE_ITEM.key, "servo", count=False
        ),
    ]


def _clamp() -> Compound:
    o = O_CLAMP
    hw, jt, st, reach = bs.JAW_HW, bs.JAW_T, bs.SPINE_T, bs.JAW_REACH
    low_top = -(bs.CLAMP_OPEN + bs.PAD_T)  # 아래 턱의 윗면 (위 턱 밑면이 y = 0)
    shape = _box(o, -st, reach, 0.0, jt, -hw, hw)  # 위 턱
    shape = solid(shape + _box(o, -st, 0.0, low_top - st, jt, -hw, hw))  # 등
    shape = solid(shape + _box(o, -st, reach, low_top - st, low_top, -hw, hw))  # 아래 턱
    shape = solid(shape + _cy(o, 4.0, low_top - st - 40.0, -bs.CLAMP_OPEN - 4.0, bs.JAW_AXIS, 0.0))
    shape = solid(shape + _cy(o, 8.0, -bs.CLAMP_OPEN - 4.0, -bs.CLAMP_OPEN, bs.JAW_AXIS, 0.0))
    return solid(shape + _cy(o, 9.0, low_top - st - 52.0, low_top - st - 40.0, bs.JAW_AXIS, 0.0))


def _plugs() -> list[ScenePart]:
    o = O_PLUG
    out: list[ScenePart] = []
    for _id, label, key, metal, body, r, tail, z in PLUGS:
        plug_n, wire_n, dev_n = _plug_names(label)
        ml, mt, mw = metal
        bl, bt, bw = body
        plug = _box(o, 0.0, ml, -mt, mt, z - mw, z + mw)
        plug = solid(plug + _box(o, -bl, 0.0, -bt / 2, bt / 2, z - bw / 2, z + bw / 2))
        bend = _at(o, -bl - tail, 0.0, z)
        wire = solid(rod(_at(o, -bl, 0.0, z), bend, r) + rod(bend, _at(o, -bl - tail, 30.0, z), r))
        dev = _box(o, 0.0, 4.0, -14.0, 14.0, z - 18.0, z + 18.0)
        dev = solid(dev - _box(o, -1.0, 5.0, -mt - 0.3, mt + 0.3, z - mw - 0.3, z + mw + 0.3))
        out += [
            ScenePart(plug_n, plug, key, "plug"),
            ScenePart(wire_n, wire, key, "wire", count=False),
            ScenePart(dev_n, dev, PORT_ITEM.key, "frame", count=False),
        ]
    return out


def _usb_ac() -> Compound:
    o = O_USB
    s = _box(o, 0.0, 12.0, -2.25, 2.25, -6.0, 6.0)
    s = solid(s + _box(o, 12.0, 32.0, -4.0, 4.0, -8.0, 8.0))
    s = solid(s + rod(_at(o, 32.0, 0.0, 0.0), _at(o, 212.0, 0.0, 0.0), 1.8))
    s = solid(s + _box(o, 212.0, 232.0, -3.5, 3.5, -6.0, 6.0))
    return solid(s + _box(o, 232.0, 238.5, -1.25, 1.25, -4.2, 4.2))


def _tie() -> Compound:
    o = O_TIE
    s = _box(o, 0.0, 150.0, 0.0, 1.6, -6.0, 6.0)
    s = solid(s + _box(o, -14.0, 0.0, 0.0, 2.5, -8.0, 8.0))
    return solid(s - _box(o, -11.0, -4.0, -1.0, 4.0, -6.5, 6.5))


def _roll() -> tuple[Compound, Compound]:
    o = O_ROLL
    scale = _box(o, -110.0, 110.0, 0.0, 22.0, -110.0, 110.0)
    scale = solid(scale + _box(o, -25.0, 25.0, 8.0, 18.0, 110.0, 111.0))  # 앞면의 화면
    y0 = 22.0
    roll = _cy(o, 100.0, y0, y0 + 4.0, 0.0, 0.0)
    roll = solid(roll + _cy(o, 70.0, y0 + 4.0, y0 + 62.0, 0.0, 0.0))
    roll = solid(roll + _cy(o, 100.0, y0 + 62.0, y0 + 66.0, 0.0, 0.0))
    return scale, solid(roll - _cy(o, 27.0, y0 - 1.0, y0 + 67.0, 0.0, 0.0))


def _screws() -> Compound:
    o = O_SCREW
    parts: list[Compound] = []
    for i in range(5):
        x = 8.0 * i
        s = _cy(o, 1.0, 0.0, 6.0, x, 0.0)
        s = solid(s + _cy(o, 2.0, 6.0, 7.3, x, 0.0))
        s = solid(s - _box(o, x - 1.4, x + 1.4, 6.7, 7.4, -0.3, 0.3))
        parts.append(solid(s - _box(o, x - 0.3, x + 0.3, 6.7, 7.4, -1.4, 1.4)))
    return _union(parts)


def scene() -> list[ScenePart]:
    scale, roll = _roll()
    return [
        *_servo(),
        *_case(),
        ScenePart(CLAMP, _clamp(), CLAMP_ITEM.key, "metal"),
        ScenePart(
            DESK,
            _box(O_DESK, -160.0, 0.0, -40.0, 0.0, -80.0, 80.0),
            DESK_ITEM.key,
            "frame",
            count=False,
        ),
        ScenePart(
            WALL,
            _box(O_DESK, GAP, GAP + 12.0, -160.0, 120.0, -80.0, 80.0),
            DESK_ITEM.key,
            "body",
            count=False,
        ),
        *_plugs(),
        ScenePart(USB_AC, _usb_ac(), USB_AC_ITEM.key, "plug"),
        ScenePart(TIE, _tie(), TIE_ITEM.key, "servo"),
        ScenePart(SCALE_PART, scale, SCALE.key, "frame", count=False),
        ScenePart(ROLL, roll, ROLL_ITEM.key, "servo"),
        ScenePart(SCREWS, _screws(), SCREW_ITEM.key, "metal"),
    ]


SIDE = View((0, 1, 0), (0, 0, 1), 30, 25)  # 위가 y, 앞이 +z
HORN_Y = sh.HORN_TOP + sh.HORN_BOSS_H
HEAD_Y = sh.HORN_TOP + sh.HORN_HEAD_H
CX = (sh.CONN_X[0] + sh.CONN_X[1]) / 2
CY = (sh.CONN_Y[0] + sh.CONN_Y[1]) / 2
BACK = -sh.HT
HA, HB, HC, HE = (12.9, 12.0), (12.9, -12.0), (-22.1, 12.0), (-46.1, 12.0)  # 뒷면 구멍 (x, z)
CALIPER_USE = Use(CALIPER.key)

TWICE = (
    "구멍 간격은 두 번 잽니다: 캘리퍼스 위쪽의 작은 안쪽 턱을 두 구멍에 넣고 벌려 먼 가장자리끼리, 아래 바깥 턱의 "
    "끝을 두 구멍에 넣고 오므려 가까운 가장자리끼리. 두 값을 다 적으면 Claude가 가운데 사이를 냅니다."
)


def _plug_step(spec: tuple[str, str, str, Off, Off, float, float, float], device: str) -> Step:
    sid, label, _key, _metal, body, r, tail, z = spec
    plug_n, wire_n, dev_n = _plug_names(label)
    bl, bt, bw = body
    o = O_PLUG
    return Step(
        sid,
        label,
        f"{label} 플러그 재기",
        do=(
            "① 플러그 몸통의 폭과 ② 두께를 캘리퍼스 바깥 턱으로 잽니다.",
            f"플러그를 {device}에 끝까지 꽂고, ③ 단자 면에서 플러그의 딱딱한 끝(고무 꼬리까지)까지 잽니다.",
            "④ 꽂은 채로 플러그 바로 뒤에서 선을 손으로 90° 꺾어 쥐고, 단자 면에서 꺾인 선의 가장 바깥까지 잽니다.",
            "네 값을 대화창에 적어 주세요.",
        ),
        warn=("④에서 선을 세게 접지 않습니다. 꽂아 두었을 때 휘는 만큼만 꺾습니다.",),
        check=("①②③④를 대화창에 적었다.",),
        stuck="재는 자리가 헷갈리면 캘리퍼스를 댄 채로 찍어 보내 주세요.",
        adds=(plug_n, wire_n, dev_n),
        still=(dev_n,),
        mount=(plug_n, wire_n, dev_n),
        focus=(plug_n, wire_n, dev_n),
        moves=(Move((plug_n, wire_n), (-15.0, 0.0, 0.0)),),
        uses=(CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 30, 35),
        spots=(
            Spot(
                plug_n,
                "①",
                (_at(o, -bl / 2, bt / 2, z - bw / 2), _at(o, -bl / 2, bt / 2, z + bw / 2)),
            ),
            Spot(
                plug_n,
                "②",
                (_at(o, -bl / 2, -bt / 2, z + bw / 2), _at(o, -bl / 2, bt / 2, z + bw / 2)),
            ),
            Spot(plug_n, "③", (_at(o, 0.0, bt / 2, z), _at(o, -bl, bt / 2, z))),
            Spot(wire_n, "④", (_at(o, 0.0, -bt / 2, z), _at(o, -bl - tail - r, 10.0, z))),
        ),
        tags=(Tag(_at(o, 6.0, 20.0, z), "단자 면"),),
    )


_PLUG_SPECS = {p[0]: p for p in PLUGS}

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "ID 2 서보(ST-3120), 서보 선 하나, 적층 아크릴 케이스(쌓인 것)와 남는 판·팬 하나씩, 금속 클램프, 파이 "
            "USB-C 전원선, 랜선, D405 선, 짧은 USB A–C 선(있으면), 찍찍이 타이(샀으면), 검정 필라멘트 첫 롤, 옛 "
            "SO-101 서보 나사, 캘리퍼스, 자나 줄자, 저울, 휴대폰을 꺼냅니다.",
            '값은 단계마다 대화창에 단계 번호와 같이 적어 주세요 (예: "1단계 ① 14.1").',
        ),
        check=("다 꺼냈다 (없는 것은 대화창에 적었다).",),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    Step(
        "st-horn",
        "혼 가운데",
        "ID 2 서보 혼 가운데 재기",
        do=(
            "ID 2 서보(ST-3120)를 혼이 위로 오게 놓습니다.",
            "① 혼 가운데 솟은 부분의 지름을 캘리퍼스 바깥 턱으로 잽니다. 지금은 14 mm로 가정했습니다.",
            "② 그 위 가운데 나사 머리의 지름을 잽니다. 6 mm 이하로 가정했습니다.",
            "두 값을 대화창에 적어 주세요.",
        ),
        warn=("턱이 가운데를 지나게 물리고, 가장 크게 나오는 값을 읽습니다.",),
        check=("①과 ②를 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(ST_BODY, ST_HORN),
        still=(ST_BODY,),
        mount=(ST_BODY, ST_HORN),
        focus=(ST_BODY, ST_HORN),
        uses=(CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 50, 20),
        zoom=1.3,
        center=(0.0, sh.HORN_TOP, 0.0),
        labels=(Label(ST_BODY, "ID 2 서보 (ST-3120)"),),
        spots=(
            Spot(ST_HORN, "①", ((-sh.HORN_BOSS_R, HORN_Y, 0.0), (sh.HORN_BOSS_R, HORN_Y, 0.0))),
            Spot(ST_HORN, "②", ((-sh.HORN_HEAD_R, HEAD_Y, 0.0), (sh.HORN_HEAD_R, HEAD_Y, 0.0))),
        ),
    ),
    Step(
        "st-plug",
        "옆면 플러그",
        "ID 2 서보 옆면 플러그 재기",
        do=(
            "서보 선의 플러그를 서보 옆면 커넥터에 끝까지 꽂고, 선을 서보 먼 끝 쪽으로 무리 없이 꺾어 몸체를 따라 "
            "눕힙니다.",
            "캘리퍼스 바깥 턱 하나를 반대쪽 옆면에 대고, ① 플러그 바깥 면까지 잽니다.",
            "같은 식으로 ② 꺾인 선의 가장 바깥까지 잽니다. 서보 폭이 들어간 값 그대로 적으면 Claude가 뺍니다.",
        ),
        warn=("선을 눌러 꺾지 않습니다. 손을 떼었을 때 선이 자리 잡는 모양 그대로 잽니다.",),
        check=("①과 ②를 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(ST_PLUG, ST_WIRE),
        mount=(ST_PLUG, ST_WIRE),
        focus=(ST_BODY, ST_HORN, ST_PLUG, ST_WIRE),
        moves=(Move((ST_PLUG, ST_WIRE), (0.0, 0.0, 14.0)),),
        uses=(CALIPER_USE,),
        view=View((0, 1, 0), (-1, 0, 0), 30, 35),
        center=(CX - 6.0, CY - 22.0, sh.HW),
        spots=(
            Spot(ST_PLUG, "①", ((CX, CY, sh.HW + sh.PLUG_OUT),)),
            Spot(ST_WIRE, "②", ((sh.CONN_X[0] - 7.0, CY, sh.HW + sh.WIRE_OUT),)),
        ),
        tags=(Tag((CX, CY, -sh.HW - 6.0), "반대쪽 옆면"),),
    ),
    Step(
        "st-holes",
        "뒷면 구멍",
        "ID 2 서보 뒷면 구멍 간격 재기",
        do=(
            "플러그를 빼고, 서보를 뒷면(혼 반대쪽)이 위로 오게 놓습니다. 작은 구멍 여섯과 가운데 둥근 보스가 "
            "보입니다.",
            "① 짧은 쪽으로 마주 본 두 구멍(도면 24 mm), ② 보스를 사이에 둔 두 구멍(도면 35 mm), ③ 먼 끝 쪽 두 "
            "구멍(도면 24 mm)을 잽니다.",
            "번호마다 두 값(먼 가장자리끼리, 가까운 가장자리끼리)을 대화창에 적어 주세요.",
        ),
        warn=(TWICE, "턱을 서보 면에 수직으로 세웁니다. 비스듬하면 값이 틀립니다."),
        check=("①②③의 여섯 값을 대화창에 적었다.",),
        stuck=fj.PHOTO,
        focus=(ST_BODY,),
        uses=(CALIPER_USE,),
        view=View((0, 0, 1), (0, -1, 0), 15, 0),
        zoom=0.85,
        center=(-16.6, BACK, -8.0),
        spots=(
            Spot(ST_BODY, "①", ((HA[0], BACK, HA[1]), (HB[0], BACK, HB[1]))),
            Spot(ST_BODY, "②", ((HA[0], BACK, HA[1] + 1.5), (HC[0], BACK, HC[1] + 1.5))),
            Spot(ST_BODY, "③", ((HC[0], BACK, HC[1] - 1.5), (HE[0], BACK, HE[1] - 1.5))),
        ),
    ),
    Step(
        "st-body",
        "몸체",
        "ID 2 서보 몸체 길이와 폭 재기",
        do=(
            "캘리퍼스 바깥 턱으로 몸체의 ① 긴 쪽 길이(도면 65 mm)와 ② 짧은 쪽 폭(도면 30 mm)을 잽니다. 혼과 "
            "뒷면 보스는 빼고 몸체만 뭅니다.",
            "두 값을 대화창에 적어 주세요.",
        ),
        check=("①과 ②를 대화창에 적었다.",),
        stuck=fj.PHOTO,
        focus=(ST_BODY, ST_HORN),
        uses=(CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 35, 35),
        spots=(
            Spot(ST_BODY, "①", ((-sh.FAR, 0.0, sh.HW), (sh.NEAR, 0.0, sh.HW))),
            Spot(ST_BODY, "②", ((-30.0, sh.HT, -sh.HW), (-30.0, sh.HT, sh.HW))),
        ),
    ),
    Step(
        "plate",
        "판 크기",
        "남는 아크릴 판의 크기와 두께 재기",
        do=(
            "케이스는 풀지 않고, 남는 아크릴 판 하나로 잽니다.",
            "① 가로, ② 세로, ③ 두께를 캘리퍼스 바깥 턱으로 잽니다.",
            "세 값을 대화창에 적어 주세요.",
        ),
        check=("①②③을 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(SPARE_PLATE,),
        mount=(SPARE_PLATE,),
        focus=(SPARE_PLATE,),
        uses=(CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 45, 20),
        spots=(
            Spot(
                SPARE_PLATE,
                "①",
                (
                    _at(O_CASE, -PL_HX, PL_T, SPARE_Z + PL_HZ),
                    _at(O_CASE, PL_HX, PL_T, SPARE_Z + PL_HZ),
                ),
            ),
            Spot(
                SPARE_PLATE,
                "②",
                (
                    _at(O_CASE, PL_HX, PL_T, SPARE_Z - PL_HZ),
                    _at(O_CASE, PL_HX, PL_T, SPARE_Z + PL_HZ),
                ),
            ),
            Spot(SPARE_PLATE, "③", (_at(O_CASE, -PL_HX, PL_T / 2, SPARE_Z + PL_HZ),)),
        ),
    ),
    Step(
        "plate-holes",
        "판 구멍",
        "아크릴 판 귀퉁이 구멍 재기",
        do=(
            "같은 판의 귀퉁이 구멍 넷을 잽니다.",
            "① 가로로 마주 본 두 구멍, ② 세로로 마주 본 두 구멍을 이렇게 잽니다.",
            "③ 구멍 하나의 지름을 작은 안쪽 턱으로 잽니다.",
        ),
        warn=(TWICE,),
        check=("①②의 네 값과 ③을 대화창에 적었다.",),
        stuck=fj.PHOTO,
        focus=(SPARE_PLATE,),
        uses=(CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 55, 20),
        spots=(
            Spot(
                SPARE_PLATE,
                "①",
                (
                    _at(O_CASE, -HOLE_X, PL_T, SPARE_Z + HOLE_Z),
                    _at(O_CASE, HOLE_X, PL_T, SPARE_Z + HOLE_Z),
                ),
            ),
            Spot(
                SPARE_PLATE,
                "②",
                (
                    _at(O_CASE, HOLE_X, PL_T, SPARE_Z - HOLE_Z),
                    _at(O_CASE, HOLE_X, PL_T, SPARE_Z + HOLE_Z),
                ),
            ),
            Spot(SPARE_PLATE, "③", (_at(O_CASE, -HOLE_X, PL_T, SPARE_Z - HOLE_Z),)),
        ),
    ),
    Step(
        "posts",
        "기둥",
        "케이스 기둥 길이와 나사 굵기 재기",
        do=(
            "쌓인 케이스에서 ① 판과 판 사이 틈(기둥 길이)을 잽니다. 남는 기둥이 있으면 그 길이를 재도 됩니다.",
            "맨 위 판의 나사 하나를 풀어 ② 나사산의 바깥 지름을 바깥 턱으로 잽니다. 잰 뒤 다시 조입니다.",
            "③ 기둥 양 끝이 모두 암나사(구멍)인지, 한쪽 끝에 나사가 튀어나와 있는지 보고 적어 주세요.",
        ),
        check=("①②③을 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(STACK, POSTS, CASE_SCREWS, STACK_FAN),
        still=(STACK, STACK_FAN),
        mount=(STACK, POSTS, CASE_SCREWS, STACK_FAN),
        focus=(STACK, POSTS, CASE_SCREWS, STACK_FAN),
        uses=(CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 25, 30),
        spots=(
            Spot(POSTS, "①", (_at(O_CASE, HOLE_X, LEVELS[1] + PL_T + POST_L / 2, HOLE_Z),)),
            Spot(CASE_SCREWS, "②", (_at(O_CASE, HOLE_X, FAN_Y0 + 1.6, HOLE_Z),)),
            Spot(POSTS, "③", (_at(O_CASE, -HOLE_X, LEVELS[1] + PL_T, HOLE_Z),)),
        ),
    ),
    Step(
        "fan",
        "팬",
        "남는 팬 크기와 전압 보기",
        do=(
            "남는 팬으로 ① 한 변, ② 두께를 잽니다. ③ 고정 구멍 간격은 마주 본 두 구멍을 판 구멍처럼 두 번 "
            "잽니다.",
            "④ 팬 스티커에 적힌 전압(5V, 12V 같은 글씨)을 읽고, 스티커를 찍어 보내 주세요.",
        ),
        check=("①②③④를 대화창에 적고 스티커 사진을 보냈다.",),
        stuck=fj.PHOTO,
        adds=(SPARE_FAN,),
        mount=(SPARE_FAN,),
        focus=(SPARE_FAN,),
        uses=(CALIPER_USE, Use(PHONE.key)),
        view=View((0, 1, 0), (0, 0, 1), 40, 20),
        spots=(
            Spot(
                SPARE_FAN,
                "①",
                (
                    _at(O_CASE, SPARE_FAN_X - FAN, FAN_T, SPARE_FAN_Z + FAN),
                    _at(O_CASE, SPARE_FAN_X + FAN, FAN_T, SPARE_FAN_Z + FAN),
                ),
            ),
            Spot(SPARE_FAN, "②", (_at(O_CASE, SPARE_FAN_X + FAN, FAN_T / 2, SPARE_FAN_Z + FAN),)),
            Spot(
                SPARE_FAN,
                "③",
                (
                    _at(O_CASE, SPARE_FAN_X - FAN_HOLE, FAN_T, SPARE_FAN_Z + FAN_HOLE),
                    _at(O_CASE, SPARE_FAN_X + FAN_HOLE, FAN_T, SPARE_FAN_Z + FAN_HOLE),
                ),
            ),
            Spot(SPARE_FAN, "④", (_at(O_CASE, SPARE_FAN_X, FAN_T, SPARE_FAN_Z),)),
        ),
    ),
    Step(
        "fan-place",
        "팬 자리",
        "쌓인 케이스의 팬 자리 보기",
        do=(
            "쌓인 케이스에서 팬(①)이 맨 위 판 위에 얹혀 있는지, 판 밑에 매달려 있는지 보고 적어 주세요.",
            "팬 선이 어디에 꽂혀 있는지(파이의 핀 줄 몇 번째인지)도 보이면 적어 주세요.",
        ),
        check=("팬 자리를 대화창에 적었다.",),
        stuck=fj.PHOTO,
        focus=(STACK, POSTS, CASE_SCREWS, STACK_FAN),
        uses=(Use(CASE_ITEM.key, note="쌓인 것", count=False),),
        view=View((0, 1, 0), (0, 0, 1), 30, 20),
        spots=(Spot(STACK_FAN, "①", (_at(O_CASE, 0.0, FAN_Y0 + FAN_T, FAN),)),),
    ),
    Step(
        "clamp",
        "클램프",
        "금속 클램프 위 턱 재기",
        do=(
            "클램프의 위 턱(나사가 없는 고정 턱)을 잽니다: ① 폭, ② 두께, ③ 등 안쪽 면에서 턱 끝까지 나온 길이.",
            "턱 끝에 받침이나 고무가 붙어 있으면 그것까지 넣어 재고, 위 턱을 옆에서 한 장 찍어 보내 주세요.",
        ),
        check=("①②③을 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(CLAMP,),
        mount=(CLAMP,),
        focus=(CLAMP,),
        uses=(CALIPER_USE, Use(PHONE.key)),
        view=View((0, 1, 0), (0, 0, 1), 30, -15),
        zoom=2.4,
        center=_at(O_CLAMP, bs.JAW_REACH / 2, -6.0, 0.0),
        spots=(
            Spot(
                CLAMP,
                "①",
                (
                    _at(O_CLAMP, bs.JAW_REACH - 3.0, bs.JAW_T, -bs.JAW_HW),
                    _at(O_CLAMP, bs.JAW_REACH - 3.0, bs.JAW_T, bs.JAW_HW),
                ),
            ),
            Spot(
                CLAMP,
                "②",
                (
                    _at(O_CLAMP, bs.JAW_REACH - 10.0, 0.0, bs.JAW_HW),
                    _at(O_CLAMP, bs.JAW_REACH - 10.0, bs.JAW_T, bs.JAW_HW),
                ),
            ),
            Spot(
                CLAMP,
                "③",
                (_at(O_CLAMP, 0.0, 0.0, bs.JAW_HW), _at(O_CLAMP, bs.JAW_REACH, 0.0, bs.JAW_HW)),
            ),
        ),
    ),
    Step(
        "desk",
        "책상 뒤",
        "책상 뒤 가장자리와 벽 사이 재기",
        do=(
            "로봇을 놓을 자리에서, 책상 상판 뒤 가장자리(①)와 벽(②) 사이를 책상 윗면 높이에서 잽니다. 캘리퍼스로 "
            "모자라면 자나 줄자로 잽니다.",
            "값을 대화창에 적어 주세요.",
        ),
        check=("①에서 ②까지 잰 값을 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(DESK, WALL),
        mount=(DESK, WALL),
        focus=(DESK, WALL),
        uses=(Use(RULER.key),),
        view=View((0, 1, 0), (0, 0, 1), 25, 0),
        spots=(
            Spot(DESK, "①", (_at(O_DESK, 0.0, 0.0, 0.0),)),
            Spot(WALL, "②", (_at(O_DESK, GAP, 0.0, 0.0),)),
        ),
    ),
    Step(
        "desk-photo",
        "책상 사진",
        "책상 뒤 둘레 찍기",
        do=(
            "베이스를 놓을 책상 뒤 가장자리 둘레를 위에서 비스듬히 한두 장 찍어 보내 주세요. 벽, 콘센트, 지나가는 "
            "선이 보이게 찍습니다.",
        ),
        check=("사진을 보냈다.",),
        stuck=fj.PHOTO,
        focus=(DESK, WALL),
        uses=(Use(PHONE.key),),
        view=View((0, 1, 0), (0, 0, 1), 55, -35),
    ),
    _plug_step(_PLUG_SPECS["pi-power"], "파이"),
    _plug_step(_PLUG_SPECS["lan"], "공유기나 파이"),
    _plug_step(_PLUG_SPECS["d405-usba"], "PC"),
    Step(
        "usb-ac",
        "짧은 USB 선",
        "짧은 USB A–C 선 찾기",
        do=(
            "파이와 서보 어댑터를 이을 짧은 USB A–C 선(30 cm 아래쯤)이 있는지 찾아봅니다.",
            "있으면 ① 한쪽 끝에서 ② 다른 끝까지 길이를 자로 잽니다. 충전만 되는 선은 쓸 수 없어서, 데이터가 "
            "되는 선인지 아는 대로 같이 적어 주세요.",
            "없으면 없다고 적어 주세요.",
        ),
        check=("길이를 적었거나, 없다고 적었다.",),
        stuck=fj.PHOTO,
        adds=(USB_AC,),
        mount=(USB_AC,),
        focus=(USB_AC,),
        uses=(Use(RULER.key),),
        view=View((0, 1, 0), (0, 0, 1), 35, 15),
        spots=(
            Spot(USB_AC, "①", (_at(O_USB, 0.0, 0.0, 0.0),)),
            Spot(USB_AC, "②", (_at(O_USB, 238.5, 0.0, 0.0),)),
        ),
    ),
    Step(
        "tie",
        "찍찍이 타이",
        "찍찍이 타이 폭과 두께 재기",
        do=(
            "찍찍이 케이블 타이를 샀으면 ① 폭과 ② 한 겹 두께를 잽니다. 두께는 가장 두꺼운 곳에서 잽니다.",
            "베이스에 낼 타이 구멍의 크기에 씁니다. 아직 안 샀으면 안 샀다고 적어 주세요.",
        ),
        check=("①②를 적었거나, 아직 안 샀다고 적었다.",),
        stuck=fj.PHOTO,
        adds=(TIE,),
        mount=(TIE,),
        focus=(TIE,),
        uses=(CALIPER_USE,),
        view=View((0, 1, 0), (0, 0, 1), 40, 20),
        zoom=1.8,
        center=_at(O_TIE, 75.0, 0.0, 0.0),
        spots=(
            Spot(TIE, "①", (_at(O_TIE, 75.0, 1.6, -6.0), _at(O_TIE, 75.0, 1.6, 6.0))),
            Spot(TIE, "②", (_at(O_TIE, 75.0, 0.0, 6.0), _at(O_TIE, 75.0, 1.6, 6.0))),
        ),
    ),
    Step(
        "roll",
        "필라멘트",
        "검정 첫 롤 무게 달기",
        do=(
            "검정 필라멘트 첫 롤을 스풀째 저울에 올려 ① 무게를 적습니다.",
            "저울(최대 500 g)을 넘으면 집에 있는 다른 저울로 답니다.",
            "빈 스풀이 있으면 그것도 달아 주세요. 필라멘트만의 무게를 낼 때 씁니다.",
        ),
        check=("무게를 대화창에 적었다.",),
        stuck="저울이 없거나 한도를 넘으면 그렇다고 적어 주세요.",
        adds=(SCALE_PART, ROLL),
        still=(SCALE_PART,),
        mount=(SCALE_PART, ROLL),
        focus=(SCALE_PART, ROLL),
        uses=(Use(SCALE.key),),
        view=View((0, 1, 0), (0, 0, 1), 30, 15),
        spots=(Spot(SCALE_PART, "①", (_at(O_ROLL, 0.0, 13.0, 111.0),)),),
    ),
    Step(
        "screws",
        "나사",
        "옛 SO-101 서보 나사 세기",
        do=(
            "옛 SO-101에서 뺀 서보 고정 나사(작은 십자 자가탭)를 꺼내, 머리의 십자(①)가 뭉개지지 않은 것만 셉니다.",
            "센 수를 대화창에 적어 주세요. 15개(서보 다섯에 3개씩)가 넘으면 나사는 안 삽니다.",
        ),
        check=("멀쩡한 나사 수를 대화창에 적었다.",),
        stuck=fj.PHOTO,
        adds=(SCREWS,),
        mount=(SCREWS,),
        focus=(SCREWS,),
        view=View((0, 1, 0), (0, 0, 1), 50, 20),
        spots=(Spot(SCREWS, "①", (_at(O_SCREW, 0.0, 7.3, 0.0),)),),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "베이스·어깨 전 재기가 끝났습니다.",
            "Claude가 값을 data/measured.toml에 넣고, 어깨와 베이스의 가정 값을 바꿔 검사를 다시 돌립니다.",
            "ST-3120 값이 들어가면 틀·혼 자리 시험편(fit2_st3120_)을 뽑아 서보를 끼워 봅니다.",
        ),
        focus=(ST_BODY, ST_HORN),
        view=SIDE,
    ),
)

GUIDE = Guide(
    id="base-measure",
    kind="measure",
    title="베이스·어깨 전 재기",
    sub="베이스와 어깨를 뽑기 전에 · 캘리퍼스, 자, 저울, 사진",
    scene=scene,
    steps=STEPS,
    items={
        i.key: i
        for i in (
            ST_ITEM,
            LEAD,
            CASE_ITEM,
            SPARE_ITEM,
            CLAMP_ITEM,
            DESK_ITEM,
            PORT_ITEM,
            PI_POWER,
            LAN_ITEM,
            D405_CABLE,
            USB_AC_ITEM,
            TIE_ITEM,
            ROLL_ITEM,
            SCREW_ITEM,
            CALIPER,
            RULER,
            SCALE,
            PHONE,
        )
    },
)
