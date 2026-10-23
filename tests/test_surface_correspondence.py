from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.local_scene import LocalScene, SensorRay
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.surface_correspondence import SurfaceBounds, stationary_surface_candidates
from probe_corner_observability import fixed_history


def line(start=-.30,end=.30,y=.425,count=61):
    return np.column_stack((np.linspace(start,end,count),np.full(count,y)))


def scene(now,groups,epoch=0):
    rays=[]
    for group_index,points in enumerate(groups):
        for index,p in enumerate(points):
            delta=np.asarray(p)-[.06,0.]
            direction=delta/np.linalg.norm(delta)
            rays.append(SensorRay((.06,0.),tuple(p),now-.1+len(rays)*.0002,'return',
                                  'laser_frame',epoch,group_index*200+index,tuple(direction)))
    return LocalScene(now,rays[0].acquired_at_s if rays else None,
                      rays[-1].acquired_at_s if rays else None,'laser_frame',epoch,tuple(rays))


def match(old=None,new=None,bounds=None,history=None):
    return stationary_surface_candidates(scene(.2,[line()]) if old is None else old,
        scene(.3,[line(-.1,.1,count=21)]) if new is None else new,
        fixed_history() if history is None else history,
        SurfaceBounds(.03,.01) if bounds is None else bounds)


def test_short_partial_surface_can_match_but_never_certifies_identity_or_refresh():
    result=match()
    assert result['evaluated'] and len(result['links'])==1
    assert result['links'][0]['previous_points']==61
    assert result['links'][0]['current_points']==21
    for name in ('identity_certified','stationary_certified','wall_certified',
                 'free_area_certified','driving_authorized','memory_refresh_authorized'):
        assert result[name] is False
    assert result['links'][0]['previous_source_first_s']==pytest.approx(.1)
    assert result['links'][0]['current_source_first_s']==pytest.approx(.2)


def test_moved_surface_can_still_match_static_hypothesis():
    # 벽을 따라 움직인 표면도 부분 겹침이면 정적 가설과 구분되지 않는다.
    result=match(new=scene(.3,[line(-.05,.15,count=21)]))
    assert len(result['links'])==1 and not result['stationary_certified']


def test_transverse_motion_larger_than_assumed_error_not_matched():
    result=match(new=scene(.3,[line(-.1,.1,y=.6,count=21)]))
    assert result['evaluated'] and result['links']==[]
    assert result['unmatched_current']==[0]
    assert not result['free_area_certified']


def test_all_candidates_preserved_without_nearest_identity_selection():
    old=scene(.2,[line(y=.40),line(y=.45)])
    result=match(old=old)
    assert len(result['links'])==2 and result['ambiguous_current']==[0]
    assert not result['identity_certified']


def test_split_is_marked_ambiguous_not_two_known_objects():
    new=scene(.3,[line(-.2,-.1,count=11),line(.1,.2,count=11)])
    result=match(new=new)
    assert len(result['links'])==2 and result['ambiguous_previous']==[0]


def test_old_fragments_not_joined_to_cover_current_surface():
    old=scene(.2,[line(-.3,-.1,count=21),line(.1,.3,count=21)])
    result=match(old=old,new=scene(.3,[line(-.25,.25,count=51)]))
    assert result['links']==[]


def test_unobserved_old_endpoints_not_extrapolated():
    result=match(new=scene(.3,[line(.2,.5,count=31)]))
    assert result['links']==[]


def test_one_far_current_point_rejects_whole_fragment():
    points=line(-.1,.1,count=21)
    points[10,1]+=.08
    result=match(new=scene(.3,[points]))
    assert result['links']==[]


def test_input_rays_and_unknowns_unchanged():
    old,new=scene(.2,[line()]),scene(.3,[line(-.1,.1,count=21)])
    rays=list(new.rays)
    rays[7]=replace(rays[7],status='unknown',hit_xy_m=None)
    rays[15]=replace(rays[15],status='masked',hit_xy_m=None)
    new=replace(new,rays=tuple(rays))
    before=new
    result=match(old,new)
    assert new==before
    assert result['current_return_count']==19
    assert result['current_unknown_count']==result['current_masked_count']==1
    assert result['occupancy_points_removed']==0
    assert 7 not in result['current_supported_source_indices']
    assert 15 not in result['current_supported_source_indices']


@pytest.mark.parametrize('new,reason',[
    (scene(.2,[line()]),'reference_did_not_advance'),
    (scene(.5,[line()]),'source_age_exceeded'),
    (scene(.3,[line()],epoch=1),'source_epoch_or_frame_changed'),
    (scene(.3,[]),'empty_observation'),
])
def test_invalid_scene_pair_is_not_evaluated(new,reason):
    result=match(new=new)
    assert not result['evaluated'] and result['reason']==reason


def test_same_source_is_not_fresh_evidence():
    old=scene(.2,[line()])
    result=match(old,replace(old,t_ref_s=.3))
    assert result['reason']=='no_new_source_observation'


def test_capacity_is_explicit_and_fail_closed():
    result=match(bounds=SurfaceBounds(.03,.01,max_points=60))
    assert not result['evaluated'] and result['reason']=='surface_capacity_exceeded'


def test_motion_unavailable_is_not_silently_zero_motion():
    result=match(history=MotionHistory(MotionConfig()))
    assert not result['evaluated'] and result['reason'].startswith('ego_motion_unavailable:')


@pytest.mark.parametrize('kwargs',[dict(range_error_m=math.nan),dict(range_error_m=-.1),
    dict(ego_transport_error_m=True),dict(max_source_age_s=.31),dict(max_source_age_s=0.),
    dict(min_points=1),dict(max_points=2048),dict(max_fragments=True)])
def test_bounds_reject_invalid_or_relaxed_freshness(kwargs):
    with pytest.raises(ValueError):SurfaceBounds(**dict(dict(range_error_m=.03,ego_transport_error_m=.01),**kwargs))
