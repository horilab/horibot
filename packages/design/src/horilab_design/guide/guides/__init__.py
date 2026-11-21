"""안내 정의. 안내 하나가 모듈 하나(출력 안내는 prints.py에 모은다)이고, 여기 ALL에 차례대로 둔다 (처음 화면의
목록 차례).

처음 화면에는 지금 쓸 안내만 둔다. 끝났거나 새 안내로 넘어간 안내는 목록에서 빼고 모듈은 남긴다: 서보 선 감아 돌려
보기(cable_wrap)와 서보로 선 돌려 보기(cable_motion)는 300 mm 선으로 돌려 보기(cable_motion_300)로, 그것은 새
뒷덮개로 다시 돌려 보기(cable_cover_300)로 넘어갔다. 선 시험은 2026-11-19에 끝나 그 안내와 출력 안내
(prints.CABLE_MOTION)도 뺐다. D405 플러그 자리 재기(d405_plug)도 2026-11-19에 끝났다.
"""

from __future__ import annotations

from horilab_design.guide.guides import base_measure, first_joint, prints, st3120_horn, wrist_end
from horilab_design.guide.model import Guide, PrintGuide

ALL: tuple[Guide | PrintGuide, ...] = (
    base_measure.GUIDE,
    prints.ST3120_COUPON,
    st3120_horn.GUIDE,
    first_joint.GUIDE,
    prints.FIRST_JOINT,
    prints.WRIST_END,
    wrist_end.GUIDE,
)
