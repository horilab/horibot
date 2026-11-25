"""베이스 둘레 조립: 책상 가운데 맞추기, 파이 둘, 서보 어댑터, 전원선, J1 선 꽂기 (0038의 베이스 둘레, 0016의 전원선).

순서는 parts/pi_mount.py의 조립 순서를 따른다:
- 책상 가운데는 파이를 달기 전에 맞춘다. 클램프는 파이 판 밑을 물어서, 파이를 단 뒤에는 베이스를 옮기기 번거롭다.
  양쪽(책상 옆 끝 ~ 책상에 무는 판의 옆면)이 같으면 가운데다: 책상 550 mm(0014), 판 244.7 mm라 152.7 mm씩이다.
- 파이는 손에 든 파이 판에 기둥 넷(밑에서 M3 너트)과 기판(M2.5 접시머리, 흰 나일론 링, 밑에서 M2.5 너트)을 단 뒤
  날개의 기둥에 얹는다. 판 나사(M3 × 8 유두 넷)는 기판의 긴 변 밖이라 기판을 단 채로 위에서 렌치가 닿는다. 기판의
  구멍 넷은 한 방향으로만 맞지만, 판은 기둥에 반대로도 얹히므로 USB·랜 단자가 책상 뒤를 보게 얹는다.
- 팬 선은 케이스에서 꽂혀 있던 핀 그대로 꽂는다. 빼기 전에 사진을 찍는다.
- 서보 어댑터는 파이와 따로 언제든 손이 닿는다. 전원선은 나사 단자에 위에서 조여 두고 떼지 않는다 (0016). 길이는
  사용자가 넉넉하게 정한다 (2026-11-25). 바나나 플러그(BU-P1825)는 선을 옆으로 넣고 작은 8-32 멈춤 나사로 문다
  (제조사 자료, 22 ~ 18 AWG).
- 영점은 오늘 집 PC에서 잰다 (다음 안내). 그래서 어댑터의 USB-C에는 PC로 가는 5 m 선을 꽂는다. 컨트롤러 파이로 가는
  선(UM2)은 처음 전원 날에 바꿔 꽂는다. J3 줄(왼쪽 단자)과 D405 선(카메라 파이 USB 3.0)은 팔을 이은 뒤다.

볼트 머리: 파이 판 나사는 유두다 (status의 나사: 머리 위가 비어 있다. 젠더 밑과 1.5 mm). 어댑터 나사는 M2 × 5
둥근머리 십자다.

장면 좌표는 J1의 서보 좌표다 (parts/base.py: x 로봇 앞, y 위, z 로봇 오른쪽).
"""

from __future__ import annotations

from dataclasses import replace

from build123d import Compound

from horibot_design.guide.catalog import printed_items
from horibot_design.guide.guides import j1_base as j1b
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
from horibot_design.parts import base as b
from horibot_design.parts import pi_mount as pm

# ------------------------------------------------------------------------------------------------
# 부품 목록

DONE = Item(
    "assembled_j1",
    "조립해 둔 J1 베이스",
    "bought",
    "어제 조립해 책상에 물린 것 (ID 1 서보가 들어 있다)",
)
PRINTED: dict[str, Item] = printed_items(
    pm.LABELS, {k: (None, None, "black") for k in ("pi_tray", "fan_grille", "adapter_cover")}
)
OTHER: dict[str, Item] = {
    it.key: it
    for it in (
        DONE,
        Item(
            "pi_hori1",
            "라즈베리파이 4 · hori1",
            "bought",
            "컨트롤러 파이. 케이스에서 뺀 기판, 로봇 오른쪽에 단다",
        ),
        Item(
            "pi_hori2",
            "라즈베리파이 4 · hori2",
            "bought",
            "카메라 파이. 케이스에서 뺀 기판, 로봇 왼쪽에 단다",
        ),
        Item("gender_ccsg3", "ㄱ자 USB-C 젠더", "bought", "마하링크 ML-CCSG3"),
        Item("pi_power_cable", "파이 전원 선", "bought", "지금 쓰는 USB-C 전원 선", icon="wire"),
        Item("lan_cable", "랜 선", "bought", "지금 쓰는 것", icon="wire"),
        Item("gamepad_dongle", "게임패드 무선 동글", "bought", "컨트롤러 파이에 꽂는다"),
        Item(
            "nylon_ring", "흰 나일론 링", "hardware", "케이스에서 기판 밑에 있던 것 (높이 3.17 mm)"
        ),
        Item(
            "brass_post",
            "황동 기둥",
            "hardware",
            "케이스의 육각 기둥 (몸통 22.93 mm, 한 끝 M3 수나사)",
        ),
        Item("acrylic_plate", "아크릴 판", "bought", "케이스의 남는 판 (팬 구멍이 난 것)"),
        Item("fan_30", "팬 30 × 30", "bought", "봉지째인 새 팬 (5 V). 봉지에 나사와 너트가 넷씩"),
        Item(
            "m3x8_cap",
            "M3 × 8 유두",
            "hardware",
            "유두 렌치볼트 (머리가 높은 원통). 2.5 mm 렌치",
            draw={"t": "cap", "L": 8},
        ),
        Item("bus_adapter", "서보 어댑터", "bought", "Waveshare Bus Servo Adapter (A)"),
        Item(
            "m2x5",
            "M2 × 5 십자",
            "hardware",
            "둥근머리 십자볼트 (가진 것)",
            draw={"t": "tap", "L": 5},
        ),
        Item(
            "usb_ac_5m",
            "USB A–C 선 5 m",
            "bought",
            "Coms IF091. 오늘은 PC와 어댑터를 잇는다",
            icon="wire",
        ),
        Item("servo_cable_300", "서보 선 300 mm", "bought", "J1 서보에 꽂아 둔 것", icon="wire"),
        Item("power_cable", "전원선", "bought", "이 안내에서 만드는 것 (빨강·검정)", icon="wire"),
        Item(
            "wire_18awg",
            "18 AWG 실리콘 전선",
            "bought",
            "빨강, 검정 (가진 것). 길이는 넉넉하게",
            icon="wire",
        ),
        Item("banana_plug", "바나나 플러그", "bought", "Mueller BU-P1825 빨강과 검정"),
        Item("m25x12_flat", "M2.5 × 12 접시머리", "hardware", "케이스에서 기판을 달았던 나사"),
        Item("m25_nut", "M2.5 너트", "hardware", "케이스의 것"),
        Item("fan_screw", "팬 나사와 너트", "hardware", "팬 봉지에 든 M2.5 둥근머리 나사와 너트"),
        Item("cover_screw", "덮개 판 나사", "hardware", "케이스 맨 위 판을 달았던 M3 나사"),
        Item("hexkey_25", "육각렌치 2.5 mm", "tool", "유두 볼트용", icon="hex"),
        Item(
            "flat_driver",
            "작은 일자 드라이버",
            "tool",
            "나사 단자, 플러그의 멈춤 나사",
            icon="driver",
        ),
        Item("stripper", "와이어 스트리퍼", "tool", "가진 것 (자르기도 한다)", icon="knife"),
        Item("caliper", "캘리퍼스", "tool", "닿지 않으면 줄자", icon="caliper"),
        Item("phone", "휴대폰", "tool", "사진을 찍어 대화창에 보낸다", icon="phone"),
        Item("desk", "작업 책상", "tool", "긴 변 55 cm"),
        Item("metal_clamp", "금속 클램프", "tool", "물려 있는 둘"),
    )
}
ITEMS_HERE: dict[str, Item] = {**PRINTED, **OTHER}

# ------------------------------------------------------------------------------------------------
# 장면

DONE_PREFIX = "J1 베이스/"
CTRL, CAM = "컨트롤러 파이/", "카메라 파이/"
# 조립해 둔 J1 베이스에서 뺄 것: 미는 도구, J1 서보의 선 둘(선 끝은 아래 어댑터 쪽에서 다시 그린다), 책상과 클램프
_DONE_SKIP = (
    "베어링 미는 도구",
    "어깨 베어링 미는 도구",
    "선 플러그",
    "어댑터로 가는 선",
    "어깨로 가는 선",
    "책상",
    "클램프",
)
_PORTS = ("랜", "USB 2.0", "USB 3.0", "USB-C 전원", "GPIO")
_PI_SKIP = ("D405 USB 플러그", "어댑터 USB 플러그")  # 팔을 이은 뒤, 처음 전원 날
# pi_mount의 이름 → 이 안내의 이름 (앞부분을 바꾼다)
_PI_RENAME = (
    ("덮개 기둥 수나사와 너트 ", "너트 (기둥) "),
    ("덮개 기둥 ", "기둥 "),
    ("나일론 링 ", "링 "),
    ("파이 판 나사 M3x8 ", "판 나사 "),
    ("게임패드 동글", "동글"),
)
_PI_KEYS = (
    ("젠더", "gender_ccsg3", True),
    ("전원 플러그", "pi_power_cable", True),
    ("랜 플러그", "lan_cable", True),
    ("동글", "gamepad_dongle", True),
    ("링 ", "nylon_ring", True),
    ("기둥 ", "brass_post", True),
    ("너트 (기둥)", "nut_m3", True),
    ("덮개 판", "acrylic_plate", True),
    ("팬", "fan_30", True),
    ("철망", "fan_grille", True),
    ("판 나사", "m3x8_cap", True),
)
_AD_SKIP = ("/서보 플러그 1", "/서보 선 (어댑터 → 띠 밑 누르개)")  # J3 줄은 위팔을 단 뒤다


def _pi(p_name: str, shape: Compound, color: str) -> ScenePart:
    who, rest = p_name.split("/", 1)
    for old, new in _PI_RENAME:
        if rest.startswith(old):
            rest = new + rest[len(old) :]
    pi_key = "pi_hori1" if who == "컨트롤러 파이" else "pi_hori2"
    if rest == "기판":
        key, count = pi_key, True
    elif rest in _PORTS:
        rest, key, count = f"단자 {rest}", pi_key, False
    else:
        key, count = next((k, c) for prefix, k, c in _PI_KEYS if rest.startswith(prefix))
    return ScenePart(f"{who}/{rest}", shape, key, color, False, count)


def _adapter(p_name: str, shape: Compound, color: str) -> ScenePart | None:
    if p_name.endswith(_AD_SKIP):
        return None
    if p_name == "어댑터 덮개":
        return ScenePart("어댑터 덮개", shape, "adapter_cover", "cap")
    if p_name == "서보 어댑터":
        return ScenePart("어댑터 기판", shape, "bus_adapter", color)
    rest = p_name.split("/", 1)[1]
    if rest in pm.AD_ITEMS:
        return ScenePart(f"어댑터 기판/{rest}", shape, "bus_adapter", color, False, False)
    if rest == "서보 플러그 2":
        return ScenePart("J1 선 플러그", shape, "servo_cable_300", color, False, False)
    if rest.startswith("서보 선 (어댑터 → J1)"):
        return ScenePart("J1 선", shape, "servo_cable_300", color, False, False)
    if rest == "USB 플러그":
        return ScenePart("USB 플러그 (PC로)", shape, "usb_ac_5m", color)
    if rest == "전원선 자리":
        return ScenePart("전원선 끝", shape, "power_cable", "wire")
    if rest.startswith("M2 나사 "):
        return ScenePart(f"어댑터 나사 {rest.rsplit(' ', 1)[1]}", shape, "m2x5", color)
    raise ValueError(f"모르는 어댑터 부품: {p_name}")


def scene() -> list[ScenePart]:
    out: list[ScenePart] = []
    for p in j1b.scene():
        if p.name.startswith(_DONE_SKIP):
            continue
        out.append(replace(p, name=f"{DONE_PREFIX}{p.name}", key=DONE.key, count=False))
    for p in b.desk_parts():
        if p.name == "책상":
            out.append(ScenePart("책상", p.shape, "desk", "frame", False, False))
        else:
            out.append(ScenePart(p.name, p.shape, "metal_clamp", "metal", False, False))
    out.append(ScenePart("파이 판 (오른쪽)", pm.tray(pm.CONTROLLER), "pi_tray", "cap"))
    out.append(ScenePart("파이 판 (왼쪽)", pm.tray(pm.CAMERA), "pi_tray", "cap"))
    for side in (pm.CONTROLLER, pm.CAMERA):
        for p in pm.pi_parts(side):
            if p.name.split("/", 1)[1] in _PI_SKIP:
                continue
            out.append(_pi(p.name, p.shape, p.color))
    for p in pm.adapter_parts():
        sp = _adapter(p.name, p.shape, p.color)
        if sp is not None:
            out.append(sp)
    return out


# ------------------------------------------------------------------------------------------------
# 이름 (선택자는 이름의 앞부분이다)


def both(rest: str) -> tuple[str, str]:
    """두 파이의 같은 부품."""
    return (f"{CTRL}{rest}", f"{CAM}{rest}")


TRAYS = ("파이 판",)
POSTS = both("기둥 ")
POST_NUTS = both("너트 (기둥)")
BOARDS = both("기판")
PORTS = both("단자 ")
RINGS = both("링 ")
PI_SET = (*TRAYS, *POSTS, *POST_NUTS, *BOARDS, *PORTS, *RINGS)
TRAY_SCREWS = both("판 나사")
GENDERS = both("젠더")
POWER_PLUGS = both("전원 플러그")
LAN_PLUGS = both("랜 플러그")
DONGLE = f"{CTRL}동글"
COVERS = both("덮개 판")
FANS = both("팬")
GRILLES = both("철망")
COVER_SET = (*COVERS, *FANS, *GRILLES)
AD_BOARD = "어댑터 기판"  # 기판과 그 위 부품
AD_SCREWS = "어댑터 나사"
POWER_END = "전원선 끝"
J1_PLUG, J1_WIRE = "J1 선 플러그", "J1 선$"
USB = "USB 플러그 (PC로)"
AD_COVER = "어댑터 덮개"

# 손에 드는 방향 (장면: x 로봇 앞, y 위, z 로봇 오른쪽)
FROM_FRONT_TOP = View((0, 1, 0), (1, 0, 0), 62, 0)
FRONT = View((0, 1, 0), (1, 0, 0.35), 38, 0)
FROM_BACK = View((0, 1, 0), (-1, 0, 0.35), 35, 0)
_AD_CENTER = (b.ADAPTER_CX, b.Y_FLOOR + 10.0, 0.0)
_GAP_Z = (b.DESK_W / 2 + b.WING_Z) / 2  # 책상 옆 끝과 판 옆면 사이의 가운데
_CENTER_MM = (b.DESK_W - 2 * b.WING_Z) / 2  # 가운데일 때 한쪽 거리 (152.7)
_TOP = pm.Y_BOARD + pm.BOARD_T  # 기판 윗면


def _gpio_pin(side: int, k: int) -> tuple[float, float, float]:
    """핀 줄 바깥 줄에서 핀 1 쪽 끝부터 k번째 핀의 자리 (보통 2번이 5 V, 4번이 5 V, 6번이 GND)."""
    x0, _x1, _y0, y1, h = pm.PORTS["GPIO"]
    hx, hv = b.PI_BOARD[0] / 2, b.PI_BOARD[1] / 2
    x = hx - x0 - 1.27 - 2.54 * (k - 1)
    v = hv - y1 + 1.27  # 바깥 줄 (기판 가장자리 쪽)
    return (b.PI_CX + x, _TOP + h, side * b.PI_CZ - v)


PHOTO = "막히면 그 자리를 사진으로 보내 주세요."

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "목록의 부품, 나사, 공구를 책상 옆에 늘어놓습니다. 파이와 케이스 부품은 다음 단계에서 꺼냅니다.",
            "출력 부품은 검정 파이 판 둘, 철망 둘, 어댑터 덮개 하나입니다.",
            "나사는 M3 × 8 유두(파이 판), M2 × 5 십자(어댑터)를 따로 모읍니다. 팬 나사와 너트는 팬 봉지에 든 채로 둡니다.",
            "전원선 재료(빨강·검정 18 AWG 전선, 바나나 플러그 빨강·검정)와 와이어 스트리퍼를 같이 둡니다.",
        ),
        check=(
            "출력 부품: 파이 판 둘, 철망 둘, 어댑터 덮개",
            "봉지째인 팬 둘, 서보 어댑터, ㄱ자 젠더 둘, USB A–C 5 m 선",
            "M3 × 8 유두 8개, M2 × 5 십자 4개",
            "전선 빨강·검정, 바나나 플러그 둘, 공구",
        ),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    Step(
        "desk",
        "책상 가운데",
        "베이스를 책상 가운데에 맞추기",
        do=(
            "금속 클램프 둘을 손으로 반 바퀴쯤 풀어, 베이스가 옆으로 밀릴 만큼만 헐겁게 합니다.",
            "날개 뒤 끝을 책상 뒤 가장자리에 댄 채 베이스를 옆으로 밀어, 책상 옆 끝에서 판 옆면까지를 양쪽 다 "
            f"{_CENTER_MM:.1f} mm로 맞춥니다. 두 값이 같으면 가운데입니다.",
            "클램프 둘을 손으로 다시 조입니다.",
            "양쪽을 잰 모습을 사진으로 찍어 대화창에 보내 주세요. 캘리퍼스가 닿지 않으면 줄자로 잽니다.",
        ),
        warn=("클램프는 손으로만 조입니다.",),
        check=(
            "양쪽 거리를 잰 사진 둘을 보냈다.",
            "날개 뒤 끝이 책상 뒤 가장자리와 맞다.",
            "베이스를 흔들어도 움직이지 않는다.",
        ),
        stuck=PHOTO,
        adds=(DONE_PREFIX, "책상$", "클램프"),
        still=(DONE_PREFIX, "책상$", "클램프"),
        uses=(Use("caliper"), Use("phone")),
        moves=(Move((DONE_PREFIX,), (0.0, 0.0, 30.0)),),
        view=FROM_FRONT_TOP,
        tags=(
            Tag((-30.0, b.Y_DESK + 2.0, _GAP_Z), f"{_CENTER_MM:.1f} mm"),
            Tag((-30.0, b.Y_DESK + 2.0, -_GAP_Z), f"{_CENTER_MM:.1f} mm"),
        ),
    ),
    Step(
        "pi-out",
        "파이 빼기",
        "케이스에서 파이 둘 빼기",
        do=(
            "케이스에 든 hori1(컨트롤러)과 hori2(카메라)를 뺍니다. hori3은 케이스에 둡니다.",
            "빼기 전에 두 파이의 팬 선이 핀 줄 어디에 꽂혀 있는지 가까이서 사진을 찍어 대화창에 보내 주세요. "
            "새 팬도 같은 자리에 꽂습니다.",
            "파이를 끄고(가능하면 sudo poweroff) 선을 모두 뽑은 뒤, 기판을 판에 단 M2.5 접시머리 나사 넷을 풀어 "
            "기판을 뺍니다. 나사, 흰 나일론 링, 너트는 파이마다 모아 둡니다.",
            "덮개에 쓸 것도 케이스에서 챙깁니다: 황동 기둥 8개, 팬 구멍이 난 아크릴 판 2장, 맨 위 판을 달았던 나사 8개.",
        ),
        warn=("기판의 핀과 부품을 손으로 누르지 않고 가장자리를 잡습니다.",),
        check=(
            "팬 선 사진을 보냈다.",
            "기판 둘과 파이마다 나사 넷, 링 넷, 너트 넷을 모았다.",
            "황동 기둥 8개, 아크릴 판 2장, 덮개 판 나사 8개를 챙겼다.",
        ),
        stuck="기둥이나 나사가 모자라면 몇 개인지 대화창에 적어 주세요.",
        uses=(
            Use("phone"),
            Use("driver", note="M2.5 나사"),
            Use("m25x12_flat", 8, "케이스에서 푼 것"),
            Use("m25_nut", 8, "케이스에서 푼 것"),
            Use("cover_screw", 8, "케이스에서 푼 것"),
        ),
        view=FRONT,
    ),
    Step(
        "posts",
        "기둥 넷",
        "파이 판에 황동 기둥 넷 달기 (두 판)",
        do=(
            "파이 판을 손에 들고, 귀퉁이 가까이의 긴 구멍 넷에 황동 기둥의 수나사를 위에서 꽂습니다.",
            "판 밑으로 나온 수나사에 M3 너트를 손으로 돌려 겁니다. 기둥을 돌려 너트가 판에 닿을 때까지 조입니다.",
            "다른 판도 똑같이 합니다.",
        ),
        warn=(
            "긴 구멍이라 기둥이 조금 밀립니다. 손으로 꼭 조이기만 하고 공구로 세게 조이지 않습니다.",
        ),
        check=("두 판에 기둥이 넷씩 서 있고 흔들리지 않는다.",),
        adds=(*TRAYS, *POSTS, *POST_NUTS),
        focus=(*TRAYS, *POSTS, *POST_NUTS),
        moves=(
            Move(POSTS, (0.0, 26.0, 0.0)),
            Move(POST_NUTS, (0.0, -12.0, 0.0), 0.8),
        ),
        view=FRONT,
        labels=(
            Label(f"{CTRL}기둥 1", "황동 기둥 × 4"),
            Label(f"{CTRL}너트 (기둥) 1", "M3 너트 × 4 (밑)"),
        ),
    ),
    Step(
        "boards",
        "기판 달기",
        "파이 판에 기판 달기",
        do=(
            "판의 둥근 돋움 넷 위에 흰 나일론 링을 하나씩 놓습니다.",
            "기판을 그 위에 얹어 구멍 넷을 링과 돋움에 맞춥니다. 맞는 방향은 하나뿐입니다.",
            "M2.5 × 12 접시머리를 기판 위에서 넣고, 판 밑에서 M2.5 너트를 겁니다. 작은 십자 드라이버로 조입니다.",
            "오른쪽에 달 판에는 hori1(컨트롤러)을, 왼쪽에 달 판에는 hori2(카메라)를 답니다.",
            "두 기판의 USB-C 전원 단자에 ㄱ자 젠더를 꽂습니다. 젠더의 구멍(소켓)이 USB·랜 단자 쪽을 보게 꽂습니다.",
        ),
        warn=(
            "너트를 손가락으로 잡고 나사를 돌립니다. 세게 조이면 링이 눌립니다.",
            "젠더는 지금 꽂습니다. hori1은 판을 얹으면 젠더와 몸통 사이가 3 mm뿐이라, 얹은 뒤에는 꽂을 수 없습니다.",
        ),
        check=(
            "기판이 링 넷 위에 고르게 앉아 흔들리지 않는다.",
            "hori1과 hori2를 어느 판에 달았는지 안다.",
            "두 젠더의 구멍이 USB·랜 단자 쪽을 본다.",
        ),
        stuck=PHOTO,
        adds=(*BOARDS, *PORTS, *RINGS, *GENDERS),
        focus=(*PI_SET, *GENDERS),
        uses=(Use("m25x12_flat", 8, count=False), Use("m25_nut", 8, count=False), Use("driver")),
        moves=(
            Move(RINGS, (0.0, 10.0, 0.0)),
            Move((*BOARDS, *PORTS), (0.0, 28.0, 0.0), 0.7),
            Move(GENDERS, (0.0, 0.0, -16.0), 1.6),
        ),
        view=FRONT,
        labels=(
            Label(f"{CTRL}기판", "hori1 (컨트롤러)"),
            Label(f"{CAM}기판", "hori2 (카메라)"),
            Label(f"{CTRL}젠더", "젠더 (구멍이 USB 쪽)"),
        ),
    ),
    Step(
        "trays-on",
        "판 얹기",
        "파이 판을 날개 기둥에 얹기 (유두 넷씩)",
        do=(
            "hori1 판을 로봇 오른쪽 날개(책상 앞에서 보면 왼쪽)의 기둥 넷 위에, hori2 판을 반대쪽 날개에 얹습니다.",
            "파이의 USB·랜 단자가 책상 뒤 가장자리 쪽을 보게 얹습니다. 판은 반대로도 얹히니 단자 방향으로 맞춥니다.",
            "기판의 긴 변 바로 바깥, 판의 구멍 넷에 M3 × 8 유두를 넣어 2.5 mm 렌치로 조입니다.",
        ),
        warn=(
            "유두 머리가 기판 가장자리 가까이에 옵니다. 렌치로 기판을 긁지 않게 곧게 세워 돌립니다.",
        ),
        check=(
            "두 판이 날개 기둥에 붙어 흔들리지 않는다.",
            "두 파이의 USB·랜 단자가 책상 뒤를 본다.",
        ),
        stuck=PHOTO,
        adds=TRAY_SCREWS,
        mount=(*PI_SET, *GENDERS, *TRAY_SCREWS),
        uses=(Use("hexkey_25"),),
        moves=(
            Move((*PI_SET, *GENDERS), (0.0, 45.0, 0.0)),
            Move(TRAY_SCREWS, (0.0, 15.0, 0.0), 1.0),
        ),
        view=FRONT,
        labels=(
            Label(f"{CTRL}기판", "hori1 (로봇 오른쪽)"),
            Label(f"{CAM}기판", "hori2 (로봇 왼쪽)"),
            Label(f"{CTRL}판 나사 1", "M3 × 8 유두 × 4"),
        ),
    ),
    Step(
        "plugs",
        "선 꽂기",
        "파이에 선 꽂기",
        do=(
            "파이 전원 선을 젠더의 구멍에 뒤에서 꽂고, 랜 선을 랜 단자에 꽂습니다.",
            "hori1(컨트롤러)의 검은 USB 2.0 단자에 게임패드 동글을 꽂습니다. hori2의 파란 USB 3.0은 D405 자리라 비워 둡니다.",
            "hori1의 젠더와 베이스 몸통 사이를 사진으로 찍어 보내 주세요 (설계로는 3 mm 뜬다).",
        ),
        warn=("전원 선을 꽂으면 파이가 바로 켜집니다. 켜져도 괜찮습니다.",),
        check=(
            "젠더의 구멍이 뒤를 보고, 전원 선이 파이 가장자리를 따라 뒤로 간다.",
            "hori1의 젠더가 몸통에 닿지 않는다 (사진을 보냈다).",
        ),
        stuck=PHOTO,
        adds=(*POWER_PLUGS, *LAN_PLUGS, DONGLE),
        uses=(Use("phone"),),
        moves=(Move((*POWER_PLUGS, *LAN_PLUGS, DONGLE), (-22.0, 0.0, 0.0)),),
        view=FROM_BACK,
        labels=(Label(f"{CTRL}전원 플러그", "전원 선 (젠더 뒤)"), Label(DONGLE, "게임패드 동글")),
    ),
    Step(
        "covers",
        "덮개 판",
        "아크릴 판에 팬과 철망 달기 (두 덮개)",
        do=(
            "아크릴 판의 둥근 팬 구멍 위에 철망을 얹고, 판 밑에 팬을 댑니다. 팬은 케이스 맨 위 판에 달려 있던 것과 같은 "
            "방향으로 댑니다.",
            "팬 봉지의 M2.5 나사 넷을 철망의 귀 위에서 넣어 판과 팬을 지나게 하고, 팬 밑에서 너트를 겁니다.",
            "나사 머리가 철망 귀의 자리에 앉을 때까지 대각선 차례로 조입니다. 다른 덮개도 똑같이 합니다.",
        ),
        warn=("세게 조이면 철망 귀가 눌립니다. 철망이 흔들리지 않을 만큼만 조입니다.",),
        check=("두 덮개에 철망과 팬이 붙어 흔들리지 않는다.", "팬 선이 판 밖으로 나와 있다."),
        adds=COVER_SET,
        focus=COVER_SET,
        uses=(Use("fan_screw", 8, "팬 봉지의 것 (너트도 8)"), Use("driver")),
        moves=(
            Move(GRILLES, (0.0, 14.0, 0.0)),
            Move(FANS, (0.0, -14.0, 0.0), 0.7),
        ),
        view=FRONT,
        labels=(Label(f"{CTRL}철망", "철망 (위)"), Label(f"{CTRL}팬", "팬 (판 밑)")),
    ),
    Step(
        "covers-on",
        "덮개 얹기",
        "팬 선 꽂고 덮개 얹기",
        do=(
            "팬 선을 파이의 핀 줄에 꽂습니다. 2단계에서 찍은 사진과 같은 자리입니다. 보통은 바깥 줄의 ①(빨강, 5 V)과 "
            "②(검정, GND)이지만, 사진과 다르면 사진대로 꽂습니다.",
            "덮개를 철망이 위로 오게 기둥 넷 위에 얹어 귀퉁이 구멍을 맞춥니다.",
            "덮개 판 나사 넷을 기둥 위 암나사에 넣어 조입니다. 다른 파이도 똑같이 합니다.",
        ),
        warn=(
            "팬 선을 다른 핀에 꽂으면 파이가 상할 수 있습니다. 헷갈리면 꽂기 전에 사진을 보내 주세요.",
        ),
        check=(
            "팬 선이 사진과 같은 핀에 꽂혀 있다.",
            "두 덮개가 기둥 넷에 붙어 흔들리지 않는다.",
            "켜져 있는 파이는 팬이 돈다.",
        ),
        stuck="팬 선을 꽂은 모습을 사진으로 보내 주세요.",
        mount=COVER_SET,
        uses=(Use("cover_screw", 8, count=False), Use("driver")),
        moves=(Move(COVER_SET, (0.0, 34.0, 0.0)),),
        view=FRONT,
        spots=(
            Spot(f"{CTRL}단자 GPIO", "①", (_gpio_pin(pm.CONTROLLER, 2),)),
            Spot(f"{CTRL}단자 GPIO", "②", (_gpio_pin(pm.CONTROLLER, 3),)),
            Spot(f"{CAM}단자 GPIO", "①", (_gpio_pin(pm.CAMERA, 2),)),
            Spot(f"{CAM}단자 GPIO", "②", (_gpio_pin(pm.CAMERA, 3),)),
        ),
    ),
    Step(
        "adapter",
        "어댑터",
        "서보 어댑터 달기 (M2 × 5 넷)",
        do=(
            "몸통 바로 뒤, 책상 밖으로 나온 뒤 판의 돋움 넷 위에 서보 어댑터를 얹습니다.",
            "흰 서보 단자 둘이 있는 긴 변이 몸통을 보고, 나사 단자와 USB-C가 있는 긴 변이 벽을 보게 놓습니다.",
            "M2 × 5 십자 넷을 기판 구멍으로 돋움에 조여 넣습니다. 작은 십자 드라이버로 기판이 흔들리지 않을 만큼만 조입니다.",
        ),
        warn=("돋움은 플라스틱에 바로 조여 넣는 구멍입니다. 세게 조이면 구멍이 헛돕니다.",),
        check=(
            "어댑터가 돋움 넷에 붙어 흔들리지 않는다.",
            "서보 단자가 몸통 쪽, 나사 단자가 벽 쪽이다.",
        ),
        stuck=PHOTO,
        adds=(AD_BOARD, AD_SCREWS),
        uses=(Use("driver"),),
        moves=(
            Move((AD_BOARD,), (0.0, 18.0, 0.0)),
            Move((AD_SCREWS,), (0.0, 10.0, 0.0), 0.8),
        ),
        view=FROM_BACK,
        zoom=2.2,
        center=_AD_CENTER,
        labels=(
            Label("어댑터 기판/서보 단자 2", "서보 단자 (몸통 쪽)"),
            Label("어댑터 기판/나사 단자", "나사 단자 (벽 쪽)"),
        ),
    ),
    Step(
        "cable-make",
        "전원선 만들기",
        "전원선 만들기 (바나나 플러그)",
        do=(
            "빨강과 검정 전선을 같은 길이로 자릅니다. 가변 전원 자리에서 어댑터까지 넉넉하게, 길이는 직접 정합니다.",
            "한쪽 끝(어댑터 쪽)은 피복을 6 mm쯤, 다른 끝(가변 전원 쪽)은 1 cm쯤 벗기고 가닥을 꼬아 둡니다.",
            "바나나 플러그는 옆구리 구멍으로 선을 넣고, 뒤쪽 구멍 속의 작은 멈춤 나사로 뭅니다. 빨강 선은 빨강 플러그, "
            "검정 선은 검정 플러그에 넣고, 멈춤 나사를 작은 일자 드라이버(맞지 않으면 작은 육각렌치)로 조입니다.",
            "선을 세게 당겨 봐서 빠지지 않으면 됩니다.",
        ),
        warn=(
            "가변 전원은 끈 채로 두고, 플러그는 아직 꽂지 않습니다 (다음 안내에서 순서대로 꽂습니다).",
            "벗긴 가닥이 흩어지지 않게 꼬고, 한 가닥이라도 삐져나오면 잘라 냅니다.",
        ),
        check=(
            "빨강 선에 빨강 플러그, 검정 선에 검정 플러그가 달렸다.",
            "당겨도 선이 빠지지 않는다.",
        ),
        stuck="플러그의 멈춤 나사가 어디 있는지 모르겠으면 플러그 사진을 보내 주세요.",
        uses=(
            Use("wire_18awg", note="빨강, 검정 같은 길이"),
            Use("banana_plug", 2),
            Use("stripper"),
            Use("flat_driver"),
        ),
        view=FROM_BACK,
        zoom=2.2,
        center=_AD_CENTER,
    ),
    Step(
        "power-in",
        "나사 단자",
        "전원선을 어댑터 나사 단자에 조이기",
        do=(
            "나사 단자의 두 나사를 작은 일자 드라이버로 몇 바퀴 풉니다. 기판에 DC+와 DC−가 찍혀 있습니다.",
            "빨강 선을 DC+에, 검정 선을 DC−에 벽 쪽에서 넣고 나사를 조입니다.",
            "선을 하나씩 당겨 봐서 빠지지 않는지 봅니다. 단자 밖으로 벗긴 구리가 보이지 않아야 합니다.",
            "전원선은 벽 쪽으로 곧게 빼 둡니다. 이 끝은 앞으로 떼지 않습니다.",
        ),
        warn=("빨강은 DC+, 검정은 DC−입니다. 바꿔 꽂지 않습니다.",),
        check=(
            "빨강이 DC+, 검정이 DC−에 조여 있고 당겨도 빠지지 않는다.",
            "단자 밖으로 구리가 보이지 않는다.",
        ),
        stuck="단자에 꽂은 모습을 사진으로 보내 주세요.",
        adds=(POWER_END,),
        uses=(Use("flat_driver"),),
        moves=(Move((POWER_END,), (-16.0, 0.0, 0.0)),),
        view=FROM_BACK,
        zoom=2.4,
        center=_AD_CENTER,
        labels=(Label(POWER_END, "빨강 → DC+, 검정 → DC−"),),
    ),
    Step(
        "j1-cable",
        "J1 선",
        "J1 서보 선을 어댑터에 꽂기 (ID 1)",
        do=(
            "몸통 뒤 아래의 네모난 선 구멍에서 나온 300 mm 선(ID 1 서보에서 온 것)을 찾습니다. 900 mm 선은 어깨로 가니 "
            "그대로 둡니다.",
            "그림처럼 선을 뒤 판의 창으로 내려 책상 뒤에 느슨한 고리로 늘어뜨린 뒤, 창으로 다시 올립니다.",
            "플러그 위로 선이 곧게 조금 서고 몸통 쪽으로 둥글게 반 바퀴 돌아 창으로 내려가게 하고, 플러그를 가운데 서보 "
            "단자에 위에서 꽂습니다.",
        ),
        warn=("선을 바짝 꺾지 않습니다. 남는 길이는 창 밑의 고리로 늘어지게 둡니다.",),
        check=(
            "플러그가 가운데 단자에 끝까지 꽂혔다.",
            "선이 바짝 꺾인 곳 없이 창으로 내려갔다가 올라온다.",
        ),
        stuck=PHOTO,
        adds=(J1_PLUG, J1_WIRE),
        uses=(Use("servo_cable_300", note="ID 1 서보에 꽂아 둔 것", count=False),),
        moves=(Move((J1_PLUG,), (0.0, 12.0, 0.0)),),
        view=View((0, 1, 0), (-0.4, 0, 1), 20, 0),
        zoom=2.0,
        center=(b.ADAPTER_CX + 15.0, b.Y_DESK, 0.0),
        labels=(Label(J1_PLUG, "가운데 단자 (ID 1 줄)"),),
    ),
    Step(
        "usb",
        "USB 선",
        "PC로 가는 USB 선 꽂기",
        do=(
            "USB A–C 5 m 선의 C 쪽을 어댑터의 USB-C 단자에 벽 쪽에서 꽂습니다.",
            "A 쪽은 집 PC로 가져갑니다. 오늘 영점은 PC에서 잽니다.",
        ),
        warn=("처음 전원 날에는 이 선을 컨트롤러 파이로 가는 짧은 선으로 바꿉니다.",),
        check=("USB-C가 끝까지 꽂혔다.",),
        adds=(USB,),
        moves=(Move((USB,), (-20.0, 0.0, 0.0)),),
        view=FROM_BACK,
        zoom=2.2,
        center=_AD_CENTER,
        labels=(Label(USB, "USB A–C 5 m (PC로)"),),
    ),
    Step(
        "cap",
        "어댑터 덮개",
        "어댑터 덮개 얹기",
        do=(
            "어댑터 덮개를 위에서 얹습니다. 벽이 없는 쪽이 몸통을 보고, 뒷벽의 홈으로 USB 선과 전원선이 지나갑니다.",
            "옆벽이 뒤 판의 옆 턱 바깥에 앉으면 됩니다. 나사는 없습니다.",
        ),
        check=("덮개가 뜨지 않고 앉았다.", "서보 선이 덮개 앞으로 눌리지 않고 나와 있다."),
        adds=(AD_COVER,),
        moves=(Move((AD_COVER,), (0.0, 32.0, 0.0)),),
        view=FROM_BACK,
        zoom=2.0,
        center=_AD_CENTER,
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "베이스 둘레 조립이 끝났습니다. 다 된 모습을 한 장 찍어 보내 주세요.",
            "전원선의 바나나 플러그는 가변 전원 옆에 둡니다. 다음 안내는 J1 영점입니다.",
        ),
        view=FRONT,
    ),
)

GUIDE = Guide(
    id="base-pi",
    kind="assembly",
    title="파이와 서보 어댑터",
    sub=f"책상 가운데({_CENTER_MM:.1f} mm씩) · 파이 둘(hori1, hori2)과 덮개 · 서보 어댑터 · 전원선 · J1 선(ID 1)",
    scene=scene,
    steps=STEPS,
    items=ITEMS_HERE,
    bundle="arm-black-5",
    printed_before=("fan_grille", "adapter_cover"),
)
