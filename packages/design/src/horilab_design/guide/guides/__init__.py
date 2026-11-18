"""안내 정의. 안내 하나가 모듈 하나(출력 안내는 prints.py에 모은다)이고, 여기 ALL에 차례대로 둔다 (처음 화면의
목록 차례).

처음 화면에는 지금 쓸 안내만 둔다. 끝났거나 새 안내로 넘어간 안내는 목록에서 빼고 모듈은 남긴다: 서보 선 감아 돌려
보기(cable_wrap)와 서보로 선 돌려 보기(cable_motion)는 300 mm 선으로 돌려 보기(cable_motion_300)로 넘어갔다.
"""

from __future__ import annotations

from horilab_design.guide.guides import cable_motion_300, first_joint, prints
from horilab_design.guide.model import Guide, PrintGuide

ALL: tuple[Guide | PrintGuide, ...] = (
    first_joint.GUIDE,
    prints.FIRST_JOINT,
    prints.CABLE_MOTION,
    cable_motion_300.GUIDE,
)
