"""운영 미연결 거리 후보의 순수 기하 계약. 허용 출력은 경계 증거일 뿐이다."""
import math

import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from corner_distance_probe import corner_distance_evidence


def reference():
    straight=np.column_stack((np.linspace(-.2,0.,9),np.zeros(9)))
    theta=np.linspace(0.,math.pi/2,31)[1:]
    curve=np.column_stack((.425*np.sin(theta),.425*(1.-np.cos(theta))))
    return np.vstack((straight,curve))


def wall(start=-.8,end=0.,offset=.425):
    return np.column_stack((np.linspace(start,end,100),np.full(100,offset)))


def evidence(points=None,path=None,side='left'):
    return corner_distance_evidence(wall() if points is None else points,
        reference() if path is None else path,side,FollowConfig())


def test_observed_inner_endpoint_supports_quarter_turn_without_front_line_extension():
    p,path=wall(),reference()
    before,old_path=p.copy(),path.copy()
    result=evidence(p,path)
    assert result['supported'],result
    assert result['reference_ahead_m'] >= .65
    assert result['observed_extent_lower_bound_m'] >= .65
    assert not result['is_drive_permission'] and not result['free_area_certified']
    assert not result['unknown_gaps_filled'] and result['added_sensor_points']==0
    np.testing.assert_array_equal(p,before)
    np.testing.assert_array_equal(path,old_path)


def test_mirror_same_contract():
    a,b=evidence(),evidence(wall()*[1,-1],reference()*[1,-1],'right')
    assert a['supported'] and b['supported']
    assert a['maximum_offset_error_m']==pytest.approx(b['maximum_offset_error_m'])


@pytest.mark.parametrize('p', [wall(start=-.2),wall(start=-.3,end=.3),
    wall(offset=.2),wall(offset=.7),wall(end=1.),
    np.vstack((wall(-.8,-.45),wall(-.2,0.)))])
def test_short_wrong_offset_forward_crossing_and_gaps_never_support(p):
    assert not evidence(p)['supported']


@pytest.mark.parametrize('path', [reference()[::-1],reference()*[1,-1],reference()+[0.,.2],
    np.column_stack((np.linspace(-.2,1.,40),np.zeros(40))),reference()[:-10]])
def test_wrong_turn_short_supported_ahead_or_offset_rejected(path):
    assert not evidence(path=path)['supported']


@pytest.mark.parametrize('p,path',[(None,0.),(0.,None),([],None),([[math.nan,0.]],None),
                                 ([[1.,2.,3.]],None)])
def test_malformed_shapes_fail_closed(p,path):
    assert not evidence(p,path)['supported']


def test_noisy_short_object_cannot_inflate_length_by_zigzag_sum():
    p=wall(start=-.2)
    p[:,1]+=np.where(np.arange(len(p))%2,.03,-.03)
    assert np.linalg.norm(np.diff(p,axis=0),axis=1).sum() > .65
    assert not evidence(p)['supported']


def test_long_wall_and_center_object_kept_but_evidence_not_drive_permission():
    p=np.vstack((wall(),[[.4,0.],[.42,0.],[.44,0.]]))
    before=p.copy()
    result=evidence(p)
    assert result['supported']
    assert max(result['source_indices'])<100
    assert not result['is_drive_permission']
    np.testing.assert_array_equal(p,before)
