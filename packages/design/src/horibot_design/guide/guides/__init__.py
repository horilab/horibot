"""안내 정의. 안내 하나가 모듈 하나(출력 안내는 prints.py에 모은다)이고, 여기 ALL에 차례대로 둔다 (처음 화면의
목록 차례).

처음 화면에는 지금 쓸 안내만 둔다. 끝났거나 새 안내로 넘어간 안내는 목록에서 빼고 모듈은 남긴다: 서보 선 감아 돌려
보기(cable_wrap)와 서보로 선 돌려 보기(cable_motion)는 300 mm 선으로 돌려 보기(cable_motion_300)로, 그것은 새
뒷덮개로 다시 돌려 보기(cable_cover_300)로 넘어갔다. 선 시험은 2026-11-19에 끝나 그 안내와 출력 안내
(prints.CABLE_MOTION)도 뺐다. D405 플러그 자리 재기(d405_plug)도 2026-11-19에 끝났다. 베이스·어깨 전 재기
(base_measure)와 검정 판 전 재기(arm_measure)는 2026-11-22에 끝나 검정 판 3 전 재기(plug_measure)로 넘어갔다.
검정 판 3 전 재기와 글자 핀 맞춰 보기(letter_pegs)는 2026-11-23에, 파이 덮개 철망 전 재기(grille_measure)는
2026-11-24에 끝났다. 첫 관절(first_joint, 11-18)과 손목 끝(wrist_end, 11-19), ST-3120 혼 자리(st3120_horn, 11-21)와
그 출력 안내(prints.FIRST_JOINT, WRIST_END, ST3120_COUPON)는 2026-11-25에 뺐다: 아티팩트 한 판에 파일이 511개까지라
팔 안내 셋을 더할 자리를 냈다.
"""

from __future__ import annotations

from horibot_design.guide.guides import (
    base_pi,
    d405_tie,
    forearm,
    j1_base,
    j1_zero,
    j2_shoulder,
    prints,
    upper_arm,
    wrist_join,
)
from horibot_design.guide.model import Guide, PrintGuide

ALL: tuple[Guide | PrintGuide, ...] = (
    d405_tie.GUIDE,
    base_pi.GUIDE,
    j1_zero.GUIDE,
    j2_shoulder.GUIDE,
    upper_arm.GUIDE,
    forearm.GUIDE,
    wrist_join.GUIDE,
    prints.ARM,
    j1_base.GUIDE,
)

# 끝낸 안내에 남은, 알고 있는 검사 결과: (안내 id, 단계 id) → 까닭. guide check가 알리기만 하고 막지 않는다. 안내를 실제로
# 한 대로 고치면 지운다 (status.md의 조립).
KNOWN: dict[tuple[str, str], str] = {
    ("j1-base", "servo-in"): (
        "실제로는 누름 틀을 끼운 서보에 허브를 다시 달아 한 덩어리로 넣었다 (2026-11-24)"
    ),
    ("j1-base", "shelf"): "렌치를 넣는 방향을 그리지 않았다. 조립은 끝났다 (2026-11-24)",
    ("base-pi", "trays-on"): (
        "실제로는 hori2는 판 나사를 먼저 조이고 젠더를 꽂았고, hori1은 이 나사를 빼고 셋만 조였다 (2026-11-25)"
    ),
}
