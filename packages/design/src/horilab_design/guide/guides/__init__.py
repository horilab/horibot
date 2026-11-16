"""안내 정의. 안내 하나가 모듈 하나이고, 여기 ALL에 차례대로 둔다 (처음 화면의 목록 차례)."""

from __future__ import annotations

from horilab_design.guide.guides import first_joint
from horilab_design.guide.model import Guide

ALL: tuple[Guide, ...] = (first_joint.GUIDE,)
