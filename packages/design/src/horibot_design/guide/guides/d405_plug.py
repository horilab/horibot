"""D405 플러그 자리 재기: D405 받침의 선 클립이 따라가는 플러그 자리(camera_mount.PLUG_TOP)를 실제 값으로 바꾼다
(status.md의 사용자 순서 2). D405 받침을 뽑기 전에 한다.

플러그의 크기(튀어나온 길이, 몸통 길이와 두께)는 2026-11-15에 쟀고, 몸통이 옆면의 어디에 붙는지는 사진으로
어림했다 (옆면 위 모서리에서 몸통 위끝까지 7 mm, ±2). 그 거리를 캘리퍼스의 깊이 막대로 재고, 고무 끝에서 선이
나오는 자리(몸통 가운데로 봤다)를 사진으로 본다.

장면은 카메라 좌표다 (camera_mount의 camera()와 plug()를 받침 자리로 옮기기 전): x가 카메라의 세로(윗면 +x,
1/4 나사 구멍이 있는 바닥면 -x), y가 가로(커넥터가 있는 옆면 -y), z가 깊이(렌즈 면 z = 0).
"""

from __future__ import annotations

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
from horibot_design.parts import camera_mount as cam
from horibot_design.parts.shapes import box, cyl_x, cyl_z, solid

W, H, D = cam.CAM_W, cam.CAM_H, cam.CAM_D
TOP = H / 2  # 윗면
SIDE = -W / 2  # 커넥터가 있는 옆면
ZC = -D + cam.CONN_BACK  # 커넥터 가운데의 깊이
X1 = TOP - cam.PLUG_TOP  # 플러그 몸통 위끝 (어림)
X0 = X1 - cam.PLUG_LEN  # 고무 끝
Y_WIRE = SIDE - cam.PLUG_OUT + cam.PLUG_BODY / 2  # 선이 나오는 곳 (몸통 두께의 가운데)
TRIPOD_R = 3.2  # 바닥면의 1/4 나사 구멍 (그림에서 방향을 잡는 표시)
LENS_Y = 9.0  # 렌즈 둘 (그림에서 방향을 잡는 표시)

CAMERA_ITEM = Item(
    "d405", "D405 카메라", "bought", "깊이 카메라", thumb=View((1, 0, 0), (0.3, -1, 0.6), 20, 0)
)
CABLE_ITEM = Item(
    "usb2_cable",
    "얇은 USB 2 선",
    "bought",
    "D405에 쓸 1.55 m 선 (카메라 쪽 플러그가 ㄱ자)",
    icon="wire",
)
CALIPER = Item("caliper", "캘리퍼스", "tool", "깊이 막대로 잰다", icon="caliper")

CAMERA = "카메라 D405"
LENS = "카메라 렌즈"
PLUG = "ㄱ자 플러그"
WIRE = "USB 선"


def scene() -> list[ScenePart]:
    body = box(-H / 2, H / 2, -W / 2, W / 2, -D, 0.0)
    body = solid(body - cyl_x(TRIPOD_R, -H / 2 - 1.0, -H / 2 + 6.0, 0.0, -D / 2))
    lens = solid(cyl_z(3.0, 0.0, 0.6, 0.0, LENS_Y) + cyl_z(3.0, 0.0, 0.6, 0.0, -LENS_Y))
    plug = box(
        X0, X1, SIDE - cam.PLUG_OUT, SIDE, ZC - cam.PLUG_ACROSS / 2, ZC + cam.PLUG_ACROSS / 2
    )
    wire = cyl_x(cam.WIRE_D / 2, X0 - 30.0, X0, Y_WIRE, ZC)
    return [
        ScenePart(CAMERA, body, CAMERA_ITEM.key, "servo"),
        ScenePart(LENS, lens, CAMERA_ITEM.key, "metal", count=False),
        ScenePart(PLUG, solid(plug), CABLE_ITEM.key, "plug"),
        ScenePart(WIRE, solid(wire), CABLE_ITEM.key, "wire", count=False),
    ]


MEASURE_VIEW = View((1, 0, 0), (0.25, -1, 0.7), 12, 0)  # 커넥터가 있는 옆면을 비스듬히 앞에서
BOTTOM_VIEW = View((0, 0, 1), (-1, -0.6, 0), 15, 0)  # 바닥면 쪽에서 고무 끝을 올려다본다
BOTTOM_TAG = Tag((-H / 2 - 6.0, 6.0, -D / 2), "바닥면 (1/4 나사 구멍)")
TOP_TAG = Tag((TOP + 5.0, 6.0, -D / 2), "윗면")
LENS_TAG = Tag((0.0, LENS_Y + 6.0, 4.0), "렌즈 면")

STEPS: tuple[Step, ...] = (
    Step(
        "prep",
        "준비물",
        "준비물 꺼내 놓기",
        kind="prep",
        do=(
            "D405 카메라, 얇은 USB 2 선(1.55 m, 카메라 쪽 플러그가 ㄱ자로 꺾인 것), 캘리퍼스를 꺼냅니다.",
        ),
        check=("D405, 얇은 USB 2 선, 캘리퍼스",),
        stuck="모자라는 것이 있으면 무엇이 없는지 대화창에 적어 주세요.",
    ),
    Step(
        "measure",
        "재기",
        "윗면에서 플러그 몸통 위끝까지 재기",
        do=(
            "얇은 USB 2 선의 ㄱ자 플러그를 D405 옆면의 커넥터에 끝까지 꽂습니다. 선은 바닥면(1/4 나사 구멍이 있는 "
            "면) 쪽으로 나갑니다.",
            "캘리퍼스의 깊이 막대가 나오는 끝을 카메라 윗면의 커넥터 쪽 모서리(①)에 댑니다.",
            "깊이 막대를 옆면을 따라 내려 플러그 몸통의 위끝(②)에 닿게 하고 값을 읽습니다.",
            "값을 대화창에 적어 주세요. 사진으로 7 mm쯤이라고 어림해 둔 거리입니다.",
        ),
        warn=("깊이 막대가 기울면 값이 커집니다. 막대를 옆면에 붙여 곧게 세웁니다.",),
        check=("①에서 ②까지 잰 값을 대화창에 적었다.",),
        stuck="재는 자리가 헷갈리면 캘리퍼스를 댄 채로 옆에서 찍어 보내 주세요.",
        adds=(CAMERA, LENS, PLUG, WIRE),
        still=(CAMERA, LENS),
        moves=(Move((PLUG, WIRE), (0.0, -12.0, 0.0)),),
        uses=(Use(CALIPER.key),),
        view=MEASURE_VIEW,
        zoom=1.8,
        center=(TOP - 6.0, SIDE - 2.0, ZC),
        labels=(Label(PLUG, "ㄱ자 플러그", at=(X0 + 4.0, SIDE - cam.PLUG_OUT, ZC)),),
        tags=(TOP_TAG, BOTTOM_TAG, LENS_TAG),
        spots=(
            Spot(CAMERA, "①", ((TOP, SIDE, ZC),)),
            Spot(PLUG, "②", ((X1, SIDE - cam.PLUG_OUT / 2, ZC),)),
        ),
    ),
    Step(
        "photo",
        "사진",
        "선이 나오는 자리 찍기",
        do=(
            "플러그를 꽂은 채로, 선이 나오는 고무 끝(③)을 바로 밑에서 찍어 대화창에 보내 주세요.",
            "선이 고무 끝의 가운데에서 나오는지 Claude가 사진으로 봅니다.",
        ),
        check=("고무 끝 사진을 보냈다.",),
        stuck=fj.PHOTO,
        uses=(Use(CABLE_ITEM.key, note="꽂은 채로", count=False),),
        view=BOTTOM_VIEW,
        zoom=1.6,
        center=(X0, Y_WIRE, ZC),
        tags=(BOTTOM_TAG,),
        spots=(Spot(PLUG, "③", ((X0, Y_WIRE, ZC),)),),
    ),
    Step(
        "done",
        "끝",
        "다 됐습니다",
        kind="end",
        do=(
            "D405 플러그 자리 재기가 끝났습니다.",
            "Claude가 값을 D405 받침에 넣고, 선 클립의 자리를 다시 검사합니다.",
        ),
        view=MEASURE_VIEW,
    ),
)

GUIDE = Guide(
    id="d405-plug",
    kind="measure",
    title="D405 플러그 자리 재기",
    sub="D405 받침을 뽑기 전에 · 캘리퍼스로 하나, 사진 하나",
    scene=scene,
    steps=STEPS,
    items={CAMERA_ITEM.key: CAMERA_ITEM, CABLE_ITEM.key: CABLE_ITEM, CALIPER.key: CALIPER},
)
