"""출력 안내 정의 (0036의 옮기는 계획 2). 판마다의 할 일은 출력 준비 도구의 판 계획(data/prints/<묶음>/)에서 만든다.

다시 뽑을 판은 모자란 부품만 모아 새 묶음으로 내고(`slicing plan --name <묶음>-r1 ...`), 그 출력 안내의 bundles 끝에
더한다. 조립 안내는 자기 묶음(Guide.bundle)을 담은 출력 안내에서 출력 부품의 판을 찾는다.
"""

from __future__ import annotations

from horilab_design.guide.model import PrintGuide

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
