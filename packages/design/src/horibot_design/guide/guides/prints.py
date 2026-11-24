"""출력 안내 정의 (0036의 옮기는 계획 2). 판마다의 할 일은 출력 준비 도구의 판 계획(data/prints/<묶음>/)에서 만든다.

다시 뽑을 판은 모자란 부품만 모아 새 묶음으로 내고(`slicing plan --name <묶음>-r1 ...`), 그 출력 안내의 bundles에
뽑는 차례대로 더한다. 조립 안내는 자기 묶음(Guide.bundle)을 담은 출력 안내에서 출력 부품의 판을 찾는다.
"""

from __future__ import annotations

from horibot_design.guide.model import PrintGuide

FIRST_JOINT = PrintGuide(
    id="first-joint-print",
    title="첫 관절 출력",
    sub="손목 몸통 둘과 속 부품, 공차 시험편",
    bundles=("first-joint",),
)

CABLE_MOTION = PrintGuide(
    id="cable-motion-print",
    title="선 돌려 보기 출력",
    sub=(
        "J6 서보 받침과 뒷덮개 둘 (J5, J6). 300 mm 선 시험의 가짜 아래팔 끝과, 부러진 받침을 바꿀 새 서보 받침 둘. "
        "선이 영점 핀 귀를 피하는 손목 2의 새 뒷덮개"
    ),
    bundles=("cable-motion", "cable-motion-300", "wrist2-cover"),
)

ST3120_COUPON = PrintGuide(
    id="st3120-coupon-print",
    title="ST-3120 혼 자리 시험편 출력",
    sub="어깨 서보(ID 2)의 혼에 덮어 보는 혼 자리 시험편 (검정). 어깨의 허브 판을 뽑기 전에 맞춤을 본다",
    bundles=("st3120-coupon",),
)

WRIST_END = PrintGuide(
    id="wrist-end-print",
    title="손목 끝 출력",
    sub="J6 플랜지와 도구 어댑터 판, 평행 그리퍼, D405 받침 (클립 없이, D405 선을 볼 시험용)",
    bundles=("wrist-end",),
)

ARM = PrintGuide(
    id="arm-print",
    title="팔 출력",
    sub=(
        "흰 판 둘(J1 몸통과 위팔, 아래팔과 쉬는 자세 선반)을 다 뽑고 검정으로 바꾼다. 검정 판 1, 2는 베이스, 어깨, "
        "J3·J4 속 부품, 덮개, 누르개, 글자 핀 시험편. 판 1의 어깨 허브 판은 퍼지 선 위에 뽑혀 그것만 다시 뽑는다. "
        "검정 판 3은 책상에 무는 판, 어댑터 덮개, 어깨 베어링(6808)을 미는 도구, 서보 받침 하나(J1, J3, J4에 셋이 든다). "
        "검정 판 4는 파이 판 둘과 글자 여덟(핀은 위팔 2.3, 아래팔 2.4 mm). 철망 판은 파이 덮개 철망 둘(봉지째인 "
        "팬의 M2.5 나사에 맞췄다). 판마다 밤 10시 전에 끝나게 건다"
    ),
    # 뽑는 차례로 둔다: 어깨 허브 판(arm-black-1-r1)은 검정 판 2 다음, 판 3 전에 뽑는다 (2026-11-23). 남은 검정은
    # 프리셋이 같아 판 둘로 모았다: J1 조립에 쓰는 판(arm-black-4)이 먼저, 글자를 살살 떼도록 글자 판(arm-black-5)이
    # 마지막이다 (2026-11-24, 사용자). 파이 덮개 철망(pi-grille)은 팬 나사를 잰 밤에 내서 그날 밤 뽑았다
    bundles=(
        "arm-white-1",
        "arm-white-2",
        "arm-black-1",
        "arm-black-1-r1",
        "arm-black-4",
        "arm-black-5",
        "pi-grille",
    ),
)
