"""J1 영점: 전원 넣기, 방향 재기, 영점 핀으로 영점 굽기, 각도 한계 굽기, 확인, 끄기 (0024의 영점, 0034).

영점은 집 PC에서 잰다 (2026-11-25): 어댑터의 USB-C에 PC로 가는 5 m 선이 꽂혀 있다. 명령은 Claude가 돌리고, 사람이
핀을 꽂으며 Enter를 누르는 `zero --yes`는 사용자가 보는 터미널 창으로 띄운다.

서보가 움직이지 않는 안내다:
- 서보는 전원을 넣으면 토크가 꺼진 채 켜진다 (맨 서보 확인, 0024). 전원 직후의 목표 위치가 0이라 목표를 쓰지 않고
  토크를 켜면 0으로 달려갈 수 있는데, 이 안내에서는 토크를 켜지 않는다. 영점 명령은 토크가 켜져 있으면 하지 않고,
  값 적용(각도 한계 굽기)도 EEPROM만 쓴다.
- 관절을 돌리는 것은 사람 손이다. 방향은 손으로 + 쪽으로 돌려 재고, Claude가 설계로 본 방향(-1, data/robot.toml)과
  견준다.

J1의 + 쪽은 운동학의 뜻이다: 위에서 볼 때 반시계. 장면의 관절 각은 설계 각(위에서 볼 때 시계가 +)이라 부호가
반대다 (conventions.md의 설계 코드의 관절 각, J1 -).

장면 좌표는 J1의 서보 좌표다 (parts/base.py: x 로봇 앞, y 위, z 로봇 오른쪽).
"""

from __future__ import annotations

from dataclasses import replace

from horibot_design.guide.guides import base_pi
from horibot_design.guide.guides import j1_base as j1b
from horibot_design.guide.model import Guide, Item, Label, Move, ScenePart, Step, Use, View
from horibot_design.parts import base as b

SERVO_ID = 1
HAVE = "있는 것/"
DONE = Item(
    "assembled_base",
    "조립해 둔 베이스",
    "bought",
    "J1 베이스(ID 1 서보), 파이 둘, 서보 어댑터와 전원선",
)
OTHER: dict[str, Item] = {
    it.key: it
    for it in (
        DONE,
        Item("psu", "가변 전원", "tool", "전류 상한 3 A로 맞춰 둔 것"),
        Item(
            "power_cable", "전원선", "bought", "앞 안내에서 만들어 어댑터에 조여 둔 것", icon="wire"
        ),
        Item("pc", "집 PC", "tool", "USB 5 m 선이 꽂힌 것. Claude가 명령을 돌린다"),
        Item("longnose", "롱노즈", "tool", "핀을 곧게 당겨 뺀다", icon="tweezers"),
        Item("phone", "휴대폰", "tool", "사진을 찍어 대화창에 보낸다", icon="phone"),
    )
}


def scene() -> list[ScenePart]:
    out = [replace(p, name=f"{HAVE}{p.name}", key=DONE.key, count=False) for p in base_pi.scene()]
    pin = next(p for p in b.hardware(j1b.MODEL) if p.name.startswith("영점 핀"))
    out.append(ScenePart("영점 핀", pin.shape, "pin_msh3_20", "pin"))
    return out


J1_BODY = f"{HAVE}{base_pi.DONE_PREFIX}J1 몸통$"
SHELF = f"{HAVE}{base_pi.DONE_PREFIX}쉬는 자세 선반"
PIN = "영점 핀"
FROM_TOP = View((0, 1, 0), (1, 0, 0.3), 55, 0)
FROM_BACK_TOP = View((0, 1, 0), (-1, 0, 0.45), 42, 0)
_PIN_AT = (-b.R_PIN, b.ATOP + 6.2, 0.0)  # 뒤쪽 귀의 핀 구멍 위

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "가변 전원을 손이 바로 닿는 곳에 두고, 전원선의 바나나 플러그 둘을 그 옆에 둡니다. 가변 전원은 꺼 둡니다.",
            "USB 5 m 선이 집 PC에 꽂혀 있는지 봅니다. Claude가 PC에서 명령을 돌립니다.",
            "영점 핀(지름 3 mm 강철 핀)과 롱노즈를 둡니다.",
        ),
        check=(
            "가변 전원이 꺼져 있고 손이 닿는다.",
            "USB 선이 PC와 어댑터에 꽂혀 있다.",
            "영점 핀과 롱노즈가 있다.",
        ),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    Step(
        "power-on",
        "전원 넣기",
        "서보 전원 넣기",
        do=(
            "검정 플러그를 가변 전원의 검은 − 단자에 꽂습니다. 빨강 플러그는 아직 꽂지 않습니다.",
            "가변 전원을 켜고 화면이 12.0 V인지 봅니다.",
            "빨강 플러그를 빨간 + 단자에 꽂습니다. 화면의 전류를 읽어 대화창에 알려 주세요.",
            "Claude가 서보(ID 1)를 읽습니다. 다 읽었다는 말을 기다립니다.",
        ),
        warn=(
            "서보는 힘이 꺼진 채 켜지므로 아무것도 움직이지 않아야 합니다. 무엇이든 움직이거나, C.C 표시등이 켜지거나, "
            "타는 냄새가 나면 바로 빨강 플러그를 뺍니다.",
            "켜고 끄는 것은 언제나 빨강 플러그로 합니다. 노란 GND 단자는 쓰지 않습니다.",
            "다른 프로그램(LeRobot, Feetech 프로그램)으로 서보에 붙지 않습니다.",
        ),
        check=(
            "화면이 12.0 V이고 C.C 표시등이 꺼져 있다.",
            "아무것도 움직이지 않았다.",
            "Claude가 ID 1 서보를 읽었다고 했다.",
        ),
        stuck="화면을 사진으로 찍어 보내 주세요.",
        adds=(HAVE,),
        still=(HAVE,),
        uses=(Use("psu"), Use("power_cable", count=False), Use("pc")),
        view=View((0, 1, 0), (-1, 0, 0.35), 35, 0),
        zoom=2.0,
        center=(b.ADAPTER_CX, b.Y_FLOOR + 10.0, 0.0),
    ),
    Step(
        "direction",
        "방향 재기",
        "J1을 + 쪽으로 돌려 방향 재기",
        do=(
            "Claude가 PC에 영점 창을 띄웁니다. 창에 나오는 말대로 합니다.",
            '"영점 핀을 빼고 …"가 나오면, 핀이 꽂혀 있지 않으니 그대로 Enter를 누릅니다.',
            '"+ 쪽으로 20°쯤 돌린 채 Enter"가 나오면, 한 손으로 베이스를 누르고 J1 몸통을 위에서 볼 때 반시계 방향으로 '
            "20°쯤 돌린 채 다른 손으로 Enter를 누릅니다.",
        ),
        warn=("천천히 돌립니다. 서보 기어가 뻑뻑한 것이 정상입니다.",),
        check=('창에 "방향"이 나왔다.',),
        stuck="창에 나온 글을 사진으로 보내 주세요.",
        uses=(Use("pc", note="영점 창"),),
        turn=(0.0, -20.0),
        view=FROM_TOP,
        labels=(Label(J1_BODY, "+ 쪽: 위에서 볼 때 반시계"),),
    ),
    Step(
        "pin-in",
        "핀 꽂기",
        "영점 핀 꽂고 Enter",
        do=(
            "J1 몸통을 되돌려 선반이 정면(책상 앞)을 보게 합니다.",
            "J1 몸통의 뒤쪽 귀(책상 뒤 가장자리 쪽, 바닥 가까이)의 작은 구멍에 영점 핀을 위에서 꽂습니다. 핀이 돌림판을 "
            "지나 베이스 몸통까지 쑥 들어가면 맞는 자리입니다. 안 들어가면 J1 몸통을 조금씩 좌우로 돌려 맞춥니다.",
            '"영점 핀을 꽂고 …"가 나오면, J1 몸통을 위에서 볼 때 반시계 방향으로 살짝 민 채 Enter를 누릅니다.',
            "창에 오프셋을 구웠다는 말과 세 줄이 나오면 됩니다. 핀은 그대로 둡니다.",
        ),
        warn=("핀을 망치나 공구로 박지 않습니다. 손으로 쑥 들어가는 자리를 찾습니다.",),
        check=("핀이 귀 위로 3 mm쯤 나와 있다.", "창에 세 줄(위치 오프셋, 영점, 방향)이 나왔다."),
        stuck="핀을 꽂은 모습과 창을 사진으로 보내 주세요.",
        adds=(PIN,),
        uses=(Use("pc", note="영점 창"),),
        moves=(Move((PIN,), (0.0, 28.0, 0.0)),),
        view=FROM_BACK_TOP,
        zoom=1.6,
        center=_PIN_AT,
        labels=(Label(PIN, "영점 핀 (뒤쪽 귀)"),),
    ),
    Step(
        "apply",
        "한계 굽기",
        "각도 한계 굽기 (Claude)",
        do=(
            "Claude가 창의 세 줄을 로봇 설정에 옮기고, 잰 방향을 설계로 본 방향과 견줍니다.",
            "Claude가 값 적용(각도 한계와 보호 설정 굽기)을 돌려도 되는지 물으면 답합니다. 서보는 움직이지 않습니다.",
            "다 됐다는 말을 기다립니다. 핀은 그대로 둡니다.",
        ),
        check=("Claude가 값 적용이 끝났다고 했다.",),
        uses=(Use("pc"),),
        view=FROM_BACK_TOP,
        zoom=1.6,
        center=_PIN_AT,
    ),
    Step(
        "pin-out",
        "핀 빼기",
        "영점 핀 빼기",
        do=("롱노즈로 핀 머리를 잡고 곧게 위로 당겨 뺍니다.",),
        warn=("귀를 받침 삼아 젖히지 않습니다. 출력한 귀가 눌립니다.",),
        check=("핀을 뺐다.",),
        uses=(Use("longnose"),),
        view=FROM_BACK_TOP,
        zoom=1.6,
        center=_PIN_AT,
        labels=(Label(PIN, "곧게 위로 당겨 뺀다"),),
    ),
    Step(
        "check",
        "확인",
        "90° 돌려 읽어 보기",
        do=(
            "J1 몸통을 위에서 볼 때 시계 방향으로 돌려, 선반이 로봇 오른쪽(책상 앞에서 보면 왼쪽)을 보게 합니다 (90°쯤).",
            "Claude가 J1 각을 읽습니다. -90°쯤이면 맞습니다.",
            "선반이 정면을 보게 되돌립니다.",
        ),
        check=("Claude가 -90°쯤으로 읽었다고 했다.", "선반이 정면을 본다."),
        uses=(Use("pc"),),
        hide=(PIN,),
        turn=(0.0, 90.0),
        view=FROM_TOP,
        labels=(Label(SHELF, "선반이 로봇 오른쪽 (-90°)"),),
    ),
    Step(
        "power-off",
        "전원 끄기",
        "서보 전원 끄기",
        do=(
            "빨강 플러그를 가변 전원에서 뺀 뒤 가변 전원을 끕니다.",
            "Claude가 기록을 올립니다.",
        ),
        warn=("어깨 서보를 달기 전에 끕니다. 서보 선은 전원을 끈 채로 꽂고 뺍니다.",),
        check=("빨강 플러그를 뺐고 가변 전원이 꺼졌다.",),
        uses=(Use("psu"),),
        hide=(PIN,),
        view=View((0, 1, 0), (-1, 0, 0.35), 35, 0),
        zoom=2.0,
        center=(b.ADAPTER_CX, b.Y_FLOOR + 10.0, 0.0),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            f"J1(ID {SERVO_ID})의 영점과 각도 한계를 구웠습니다. 핀은 J2, J3, J4 영점에도 씁니다.",
            "다음 안내는 어깨(J2) 조립입니다. J2 영점은 위팔을 단 날 잽니다.",
        ),
        hide=(PIN,),
        view=FROM_TOP,
    ),
)

GUIDE = Guide(
    id="j1-zero",
    kind="measure",
    title="J1 영점",
    sub=f"서보 전원 넣기 · J1(ID {SERVO_ID}) 방향과 영점 · 각도 한계 · 집 PC에서",
    scene=scene,
    steps=STEPS,
    items=OTHER,
)
