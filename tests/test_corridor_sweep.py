"""경로 접선 순간 정렬과 실제 현재 자세에서 출발하는 swept 감사를 구별한다."""
import math
import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from audit_corridor_sweep import sweep_poses, sweep_visibility, required_stopping_distance
from test_corridor_visibility import scene


@pytest.mark.parametrize('steering', [-.37, 0., .37])
def test_current_body_pose_is_exact_origin_not_path_tangent(steering):
    rows = sweep_poses(steering, .135, .7)
    np.testing.assert_array_equal(rows[0], [0.,0.,0.,0.])
    assert rows[-1,0] == .7
    assert np.max(np.diff(rows[:,0])) <= .04+1e-10


def test_straight_current_pose_sweep_has_expected_center():
    rows = sweep_poses(0., .135, .8)
    np.testing.assert_allclose(rows[:,0], rows[:,1], atol=1e-15)
    np.testing.assert_array_equal(rows[:,2:], np.zeros((len(rows),2)))
    result = sweep_visibility(scene(), 0., .8, FollowConfig())
    assert result['stopping_sweep_depth_supported']
    assert result['depth_supported_prefix_m'] == .8
    assert not result['actual_braking_verified']
    assert not result['driving_authorized']


def test_left_and_right_are_mirrored_and_rear_axle_follows_circle():
    left, right = [sweep_poses(a, .135, .8) for a in (.25,-.25)]
    np.testing.assert_allclose(left, right*[1,1,-1,-1], atol=1e-15)
    curvature = math.tan(.25)/.135
    rear_x = left[:,1]-.0675*np.cos(left[:,3])
    rear_y = left[:,2]-.0675*np.sin(left[:,3])
    np.testing.assert_allclose(rear_x, np.sin(left[:,3])/curvature-.0675)
    np.testing.assert_allclose(rear_y, (1-np.cos(left[:,3]))/curvature)


def test_first_return_failure_is_retained_as_failure():
    result = sweep_visibility(scene(.45), 0., .8, FollowConfig())
    assert not result['stopping_sweep_depth_supported']
    assert result['depth_supported_prefix_m'] < .35
    assert result['first_unsupported']


def test_measured_speed_not_replaced_by_smaller_target_for_stopping():
    cfg = FollowConfig()
    assert required_stopping_distance(1., .3, .1, cfg) == pytest.approx(.67)
    assert required_stopping_distance(.3, 1., .1, cfg) == pytest.approx(.67)
    assert required_stopping_distance(1., 1., .2, cfg) > required_stopping_distance(1., 1., .1, cfg)


@pytest.mark.parametrize('args', [(.4,.135,.7), (0.,0.,.7), (0.,.135,3.), (float('nan'),.135,.7)])
def test_invalid_or_expanded_sweep_is_not_accepted(args):
    with pytest.raises(ValueError):
        sweep_poses(*args)


def test_signed_reverse_not_used_as_forward_stopping_evidence():
    with pytest.raises(ValueError):
        required_stopping_distance(-.1, 0., .1, FollowConfig())
