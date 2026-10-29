"""미연결 비교 도구의 경로 끝 보존과 구분 불가능한 사례의 범위."""
import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from probe_short_boundary_prefix import prefix_geometry


def straight(own_end=.55,other_y=.425):
    a=np.linspace(-.4,own_end,65)
    b=np.linspace(-.4,2.,160)
    return np.vstack((np.column_stack((a,a*0-.425)),np.column_stack((b,b*0+other_y))))


@pytest.mark.parametrize('side', ['left','right'])
def test_short_prefix_uses_only_own_observed_path(side):
    config=FollowConfig()
    points=straight()
    if side=='left':points[:,1]*=-1
    original=points.copy()
    result=prefix_geometry(points,side,config)
    assert result['supported']
    assert not result['runtime_connected'] and not result['driving_authorized']
    assert not result['wall_identity_certified'] and not result['memory_refresh_authorized']
    path,_=local_path(points,side,config.wall_offset_m)
    kept=np.asarray(result['path'])
    assert np.array_equal(kept,path[:len(kept)])
    assert np.array_equal(points,original)
    assert np.max(kept[:,0])<=.55+1e-9
    assert result['shared_span_m']>=.25


@pytest.mark.parametrize('end', [.1,.2])
def test_short_object_cannot_supply_a_driving_prefix(end):
    assert not prefix_geometry(straight(end),'right',FollowConfig())['supported']


def test_long_path_not_replaced_by_short_candidate():
    assert prefix_geometry(straight(1.3),'right',FollowConfig())['reason']=='not_a_short_boundary'


def test_opposite_shape_disagreement_not_waived_for_missing_anchor():
    assert not prefix_geometry(straight(other_y=.70),'right',FollowConfig())['supported']


def test_observed_gap_not_counted_as_supported_wall_length():
    points=straight()
    points=points[~((points[:,1]<0)&(points[:,0]>.10)&(points[:,0]<.35))]
    assert not prefix_geometry(points,'right',FollowConfig())['supported']


def test_short_boundary_identity_is_still_unknown():
    # 긴 물체의 옆면과 같은 기하라면 벽이라고 분류할 수 없다. 점유는 후속 별도 검사다.
    result=prefix_geometry(straight(),'right',FollowConfig())
    assert result['supported'] and not result['wall_identity_certified']
    assert not result['driving_authorized']


@pytest.mark.parametrize('points', [[],[[0,1,2]],[[float('nan'),0]],'not-points'])
def test_bad_input_has_no_path(points):
    result=prefix_geometry(points,'right',FollowConfig())
    assert not result['supported'] and result['path'] is None
