"""x86/ARM의 미세 차감 오차가 관측 경로의 점 개수 차이로 커지지 않아야 한다."""
import math
import pytest
from arena_autonomy.local_path import _sample_count


@pytest.mark.parametrize('first',[0., .60409914267, .96520388004, 3., 7.])
def test_exact_2p05m_span_keeps_83_points_across_roundoff(first):
    last=first+2.05
    # r79 첫 불일치의 원인: 82에 매우 가까운 비율의 int 내림이 81이 됨.
    for left in (math.nextafter(first,-math.inf),first,math.nextafter(first,math.inf)):
        for right in (math.nextafter(last,-math.inf),last,math.nextafter(last,math.inf)):
            assert _sample_count(left,right)==83


@pytest.mark.parametrize('first',[0.,.96520388004,7.])
def test_genuinely_shorter_span_is_not_rounded_up(first):
    assert _sample_count(first,first+2.05-1e-9)==82
    assert _sample_count(first,first+2.05+1e-9)==83


def test_small_span_minimum_and_non_grid_span_are_unchanged():
    assert _sample_count(.4,.5)==12
    assert _sample_count(.4,.4+2.043)==82
