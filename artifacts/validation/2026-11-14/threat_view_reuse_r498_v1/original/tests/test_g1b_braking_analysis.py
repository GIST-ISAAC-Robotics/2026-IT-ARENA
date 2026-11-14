"""평가 전용 위치 차분은 누락 구간을 속도 0으로 채우지 않습니다."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from analyze_g1b_braking import diagnostic_summary, nearest_index, pose_speed


def test_pose_speed_from_positions_not_cached_dynamics():
    times = [i * .001 for i in range(101)]
    poses = [(t, .6*t, .8*t, 0.) for t in times]
    assert pose_speed(poses, times, .05) == pytest.approx(1.)
    assert pose_speed(poses, times, .001) is None
    assert pose_speed(poses, times, .099) is None


def test_pose_speed_refuses_missing_endpoints():
    assert pose_speed([], [], .05) is None
    assert pose_speed([(0., 0., 0.), (.1, .1, 0.)], [0., .1], .05) is None


def test_nearest_samples_are_bounded():
    assert nearest_index([1., 2., 3.], -1) == 0
    assert nearest_index([1., 2., 3.], 4) == 2
    assert nearest_index([1., 2., 3.], 1.6) == 1


def test_diagnostic_summary_missing_is_not_zero_slip():
    assert diagnostic_summary([], 1., .0325) == {'samples': 0, 'available': False}
    with pytest.raises(ValueError, match='missing'):
        diagnostic_summary([{'sim_time_s': 1.}], 1., .0325)


def test_diagnostic_summary_compares_same_time_wheel_and_body():
    row = dict(sim_time_s=1., left_speed_rad_s=10., right_speed_rad_s=30.,
               wheel_surface_speed_mps=.65, truth_longitudinal_mps=1., truth_roll_rad=0., truth_pitch_rad=.001)
    result = diagnostic_summary([row], 1., .0325)
    assert result['max_wheel_body_difference_mps'] == pytest.approx(.35)
    assert result['max_left_right_difference_mps'] == pytest.approx(.65)
    assert result['samples'] == 1
    with pytest.raises(ValueError, match='non-monotonic'):
        diagnostic_summary([row, row], 1., .0325)
