import math

import numpy as np
import pytest

from arena_autonomy.corner_evidence import corner_anchor_evidence
from arena_autonomy.local_follow import FollowConfig


def reference():
    first=np.column_stack((np.linspace(-.25,.35,18),np.zeros(18)))
    theta=np.linspace(-math.pi/2,0,19)[1:]
    return np.vstack((first,np.column_stack((.35+.30*np.cos(theta),.30+.30*np.sin(theta)))))


def wall(start=-.72,end=.15,y=.425,count=100):
    return np.column_stack((np.linspace(start,end,count),np.full(count,y)))


def evidence(points=None,path=None,side='left'):
    return corner_anchor_evidence(wall() if points is None else points,
                                  reference() if path is None else path,side,FollowConfig())


def test_short_ahead_inner_wall_with_long_connected_current_anchor_supports_corner():
    points,path=wall(),reference()
    original=points.copy()
    result=evidence(points,path)
    assert result['supported'],result
    assert result['observed_length_m'] >= .65 and result['front_extent_m'] < .65
    assert result['minimum_inner_distance_m'] >= .345
    assert result['occupied_points']==len(points) and result['added_sensor_points']==0
    assert not result['is_drive_permission'] and not result['free_area_certified']
    assert not result['unknown_gaps_filled']
    np.testing.assert_array_equal(points,original)


def test_mirror_has_same_observed_geometry():
    a=evidence()
    b=evidence(wall()*[1,-1],reference()*[1,-1],'right')
    assert a['supported'] and b['supported']
    assert a['observed_length_m']==pytest.approx(b['observed_length_m'])


@pytest.mark.parametrize('points',[wall(start=-.1),wall(end=.05),wall(start=.1,end=.9),
    wall(y=.2),wall(y=.6),wall(count=8),np.empty((0,2)),np.array([[np.nan,0.]])])
def test_short_vehicle_anchor_behind_only_offset_or_invalid_never_supports(points):
    assert not evidence(points)['supported']


def test_missing_piece_is_not_joined_into_total_length():
    points=np.vstack((wall(-.72,-.4,count=30),wall(-.15,.15,count=30)))
    assert not evidence(points)['supported']


def test_shallow_two_faces_are_not_fitted_as_a_long_straight_anchor():
    points=wall()
    points[points[:,0]<-.2,1]+= .25*(points[points[:,0]<-.2,0]+.2)
    assert not evidence(points)['supported']


def test_right_angle_box_corner_is_not_a_straight_anchor():
    points=np.vstack((wall(-.2,.15,count=40),
                      np.column_stack((np.full(60,-.2),np.linspace(.1,.425,60)))))
    assert not evidence(points)['supported']


@pytest.mark.parametrize('points,path',[(None,0.),(0.,None),([],None),([[1.,2.,3.]],None)])
def test_malformed_shapes_fail_closed(points,path):
    assert not evidence(points,path)['supported']


def test_small_seeded_line_noise_does_not_alter_input_or_claim_free_space():
    points=wall()
    points[:,1]+=np.random.default_rng(51).uniform(-.01,.01,len(points))
    original=points.copy()
    result=evidence(points)
    assert result['supported'],result
    np.testing.assert_array_equal(original,points)


@pytest.mark.parametrize('path',[
    np.column_stack((np.linspace(-.25,1.8,36),np.zeros(36))),
    reference()*[1,-1],reference()[::-1],reference()+[0,.2]])
def test_straight_opposite_turn_reversed_or_offset_reference_rejected(path):
    assert not evidence(path=path)['supported']


def test_long_wall_in_future_path_cannot_be_ignored_to_approve_cutting_corner():
    assert not evidence(wall(end=.8))['supported']


def test_unrelated_points_remain_occupied_even_if_not_corner_anchor():
    points=np.vstack((wall(),[[.4,0.],[.42,0.],[.44,0.]]))
    result=evidence(points)
    assert result['supported']
    assert result['occupied_points']==103
    assert max(result['source_indices'])<100
