"""전방 순서 보조 평가: 가까운 옆차와 진짜 선행차/가림 기록을 구분한다."""
import math
import pytest
from traffic_front_target import front_target_at


def pose(x, y=0., yaw=0., t=1.):
    return [[t, x, y, yaw]]


def test_passed_side_car_can_be_the_actual_front_actor():
    r = front_target_at(pose(0), {'primary': pose(1.9), 'passed': pose(.9, .13)}, 1.)
    assert r['front_actor'] == 'passed'
    assert r['front_body_gap_m'] == pytest.approx(.7)
    assert r['actors']['primary']['body_gap_m'] == pytest.approx(1.7)


def test_closer_parallel_but_nonoverlapping_actor_is_not_selected():
    r = front_target_at(pose(0), {'primary': pose(1.9), 'parallel': pose(.4, .3)}, 1.)
    assert r['front_actor'] == 'primary'


def test_alongside_actor_is_reported_separately_and_not_a_front_success():
    r = front_target_at(pose(0), {'beside': pose(0., .22), 'rear': pose(-.5)}, 1.)
    assert r['front_actor'] is None
    assert r['alongside_actors'] == ['beside']


def test_rotated_global_frame_preserves_choice():
    r = front_target_at(pose(2., 4., math.pi/2),
                        {'primary': pose(2., 5.9, math.pi/2), 'passed': pose(1.87, 4.9, math.pi/2)}, 1.)
    assert r['front_actor'] == 'passed' and r['front_body_gap_m'] == pytest.approx(.7)


@pytest.mark.parametrize('bad', [[], pose(.4, t=.9), pose(.4, t=1.1), [[1., float('nan'), 0., 0.]]])
def test_missing_or_invalid_other_actor_cannot_disappear_from_evaluation(bad):
    r = front_target_at(pose(0), {'primary': pose(1.), 'unknown': bad}, 1.)
    assert not r['valid'] and r['stream'] == 'unknown'
