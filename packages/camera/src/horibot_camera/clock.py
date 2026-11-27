"""파이 시계와 PC 시계의 차이를 왕복으로 잰다 (docs/decisions/0039-camera-stream.md).

PC가 묻고(t1), 파이가 제 시각을 답하고(t2), PC가 받는다(t3). 파이 시각 t2는 PC 시각 (t1 + t3) / 2에 해당한다고
보고, 오차 한도는 왕복의 절반이다. 창(10초)마다 왕복이 가장 짧은 점을 골라, 고른 점들을 이은 꺾은선으로 PC 시각을
파이 시각으로 바꾼다: 파이 시계는 chrony가 고치는 시계라 한 직선에서 벗어날 수 있다. 첫 점 앞과 마지막 점 뒤는 그
점의 차이를 그대로 쓴다. PC 시각은 어느 시계든 되지만, 받은 때와 같은 시계(time.perf_counter 등)를 쓴다.
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass

WINDOW_S = 10.0


@dataclass(frozen=True)
class Anchor:
    """창 하나에서 고른 점."""

    pc: float  # PC 시각 (t1 + t3) / 2
    offset: float  # 파이 시각 − PC 시각
    bound: float  # 오차 한도: 왕복의 절반


class ClockFit:
    def __init__(self, window_s: float = WINDOW_S) -> None:
        self.window_s = window_s
        self._best: dict[int, Anchor] = {}
        self._keys: list[int] = []

    def add(self, t1: float, t2: float, t3: float) -> None:
        """PC 시각 t1에 묻고 t3에 받은 답 t2(파이 시각)를 더한다. 답이 물은 때보다 앞서면 버린다."""
        if t3 < t1:
            return
        pc = (t1 + t3) / 2
        anchor = Anchor(pc, t2 - pc, (t3 - t1) / 2)
        key = math.floor(pc / self.window_s)
        best = self._best.get(key)
        if best is None:
            bisect.insort(self._keys, key)
        if best is None or anchor.bound < best.bound:
            self._best[key] = anchor

    @property
    def anchors(self) -> list[Anchor]:
        return [self._best[k] for k in self._keys]

    def _around(self, pc: float) -> tuple[Anchor, Anchor] | None:
        anchors = self.anchors
        if not anchors:
            return None
        i = bisect.bisect_left([a.pc for a in anchors], pc)
        if i == 0:
            return anchors[0], anchors[0]
        if i == len(anchors):
            return anchors[-1], anchors[-1]
        return anchors[i - 1], anchors[i]

    def offset_at(self, pc: float) -> float | None:
        """PC 시각 pc에서의 차이 (파이 − PC). 아직 점이 없으면 None."""
        around = self._around(pc)
        if around is None:
            return None
        a, b = around
        if b.pc == a.pc:
            return a.offset
        return a.offset + (b.offset - a.offset) * (pc - a.pc) / (b.pc - a.pc)

    def to_pi(self, pc: float) -> float | None:
        offset = self.offset_at(pc)
        return None if offset is None else pc + offset

    def bound_at(self, pc: float) -> float | None:
        """그 시각의 오차 한도: 이은 두 점의 한도 가운데 큰 것."""
        around = self._around(pc)
        if around is None:
            return None
        return max(around[0].bound, around[1].bound)
