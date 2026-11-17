"""안내 정의. 안내 하나가 모듈 하나(출력 안내는 prints.py에 모은다)이고, 여기 ALL에 차례대로 둔다 (처음 화면의
목록 차례)."""

from __future__ import annotations

from horilab_design.guide.guides import cable_wrap, first_joint, prints
from horilab_design.guide.model import Guide, PrintGuide

ALL: tuple[Guide | PrintGuide, ...] = (first_joint.GUIDE, cable_wrap.GUIDE, prints.FIRST_JOINT)
