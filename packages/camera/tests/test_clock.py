"""왕복으로 시계 맞추기 (docs/decisions/0039-camera-stream.md)."""

import numpy as np
import pytest

from horibot_camera.clock import ClockFit


def _true_offset(pc: float) -> float:
    """파이 − PC: 100 s 차이에 50 ppm 흐름, 40초에 chrony가 2 ms 꺾는다."""
    return 100.0 + 50e-6 * pc + (0.002 * (pc - 40.0) / 20.0 if pc > 40.0 else 0.0)


def test_fit_follows_a_bending_offset_within_the_bound() -> None:
    rng = np.random.default_rng(3)
    fit = ClockFit()
    for t1 in np.arange(0.0, 80.0, 0.25):
        out = rng.uniform(0.0002, 0.003)  # 묻는 길
        back = rng.uniform(0.0002, 0.003)  # 답하는 길 (서로 달라 차이가 한쪽으로 쏠린다)
        t2 = t1 + out + _true_offset(t1 + out)
        fit.add(float(t1), float(t2), float(t1 + out + back))
    assert len(fit.anchors) == 8  # 10초 창 여덟
    for anchor in fit.anchors:
        assert abs(anchor.offset - _true_offset(anchor.pc)) <= anchor.bound + 1e-9
    # 고른 점 사이: 꺾인 곳을 낀 구간(최대 20초)만 직선에서 벗어난다 (기울기 차이 1e-4 × 20 s / 4 = 0.5 ms)
    for pc in np.arange(fit.anchors[0].pc, fit.anchors[-1].pc, 0.7):
        bound = fit.bound_at(float(pc))
        offset = fit.offset_at(float(pc))
        assert bound is not None and offset is not None
        assert bound < 0.001  # 창마다 가장 짧은 왕복을 고르면 0039의 기준(1 ms) 안이다
        assert abs(offset - _true_offset(float(pc))) < bound + 0.0006


def test_no_pings_no_answer_and_bad_pings_are_dropped() -> None:
    fit = ClockFit()
    assert fit.to_pi(1.0) is None
    assert fit.bound_at(1.0) is None
    fit.add(2.0, 50.0, 1.0)  # 받은 때가 물은 때보다 앞선다
    assert fit.to_pi(1.0) is None
    fit.add(1.0, 101.5, 1.002)
    assert fit.to_pi(1.001) == pytest.approx(101.5)
    assert fit.to_pi(500.0) == pytest.approx(500.0 + 100.499)  # 마지막 점 뒤는 마지막 차이
