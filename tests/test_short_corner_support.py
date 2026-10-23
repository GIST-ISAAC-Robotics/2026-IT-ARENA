import numpy as np
import pytest

from probe_short_corner_support import point_support,compact_circle_fixture
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.corner_distance import corner_distance_evidence


@pytest.mark.parametrize('sign',[1,-1])
def test_compact_round_surface_supports_long_path_but_does_not_identify_wall(sign):
    p,path,info=compact_circle_fixture(sign)
    saved,original=p.copy(),path.copy()
    side='left' if sign==1 else 'right'
    result=point_support(p,path,side)
    assert info['footprint_length_m']<=.2 and info['footprint_width_m']<=.15
    assert result['observed_extent_m']<.15
    assert result['path_prefix_m']>=.65
    assert not corner_distance_evidence(p,path,side,FollowConfig())['supported']
    assert not result['is_wall_evidence'] and not result['is_drive_permission']
    assert not result['free_area_certified'] and not result['unknown_gaps_filled']
    assert result['added_sensor_points']==0
    np.testing.assert_array_equal(saved,p)
    np.testing.assert_array_equal(original,path)


def test_point_support_mirror_symmetry():
    p,path,_=compact_circle_fixture()
    left=point_support(p,path,'left')
    right=point_support(p*[1,-1],path*[1,-1],'right')
    assert left['path_prefix_m']==pytest.approx(right['path_prefix_m'])


def test_no_bridging_a_middle_path_mismatch():
    p,path,_=compact_circle_fixture()
    path[35:40,0]+=.4
    result=point_support(p,path,'left')
    assert result['path_prefix_m']<.65


@pytest.mark.parametrize('points,path,side',[
    ([],[[0,0],[1,0],[2,0]],'left'),
    ([[float('nan'),0],[1,2]],[[0,0],[1,0],[2,0]],'left'),
    ([[0,1],[1,1]],[[0,0],[0,0],[0,0]],'left'),
    ([[0,1],[1,1]],[[0,0],[1,0],[2,0]],'other'),
])
def test_malformed_geometry_rejected(points,path,side):
    with pytest.raises(ValueError):point_support(points,path,side)
