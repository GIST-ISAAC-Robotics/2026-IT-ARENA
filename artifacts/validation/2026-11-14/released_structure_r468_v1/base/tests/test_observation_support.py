from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.observation_support import (RecentScenes, depth_support, transport_scene,
    source_valid, outside_body, sweep_queries, sweep_support)
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_scan
from audit_corridor_visibility import bracket_depth
from test_local_follow import history
from test_corridor_visibility import scene
from test_linear_observation import unknown_at


def timed(now,epoch=0):
    return scene_from_scan([2.]*500,angle_min=-math.pi,angle_increment=math.tau/500,
        range_min=.05,range_max=12.,acquired_at_s=(now-.1+np.arange(500)*.0002).tolist(),
        t_ref_s=now,source_epoch=epoch)


@pytest.mark.parametrize('observation',[scene(),unknown_at(0),unknown_at(90),scene(.3)])
def test_vectorized_depth_matches_existing_audit(observation):
    q=np.array([[.2,0.],[.1,.08],[.3,.2],[-.2,0.],[.6,-.1]])
    padding=np.full(500,.055)
    expected=[r['supported'] for r in bracket_depth(observation,q,padding)]
    np.testing.assert_array_equal(depth_support(observation,q,padding),expected)


def test_cache_preserves_original_source_time_and_expires():
    c=RecentScenes(); h=history()
    sources,_=c.observe(timed(.4),h); assert len(sources)==1
    sources,_=c.observe(timed(.5),h); assert len(sources)==2
    assert sources[1].source_first_s==pytest.approx(.3) and sources[1].t_ref_s==.5
    sources,_=c.observe(replace(timed(.5),t_ref_s=.58),h)
    assert len(sources)==2 and sources[1].source_first_s==pytest.approx(.3)
    sources,_=c.observe(timed(.7),h)
    assert all(s.source_first_s>=.4-1e-9 for s in sources)


def test_time_reverse_epoch_and_invalid_count_clear_cache():
    c=RecentScenes(); h=history()
    c.observe(timed(.5),h)
    sources,reason=c.observe(timed(.4),h)
    assert not sources and reason=='observation_invalid_or_regressed'
    c.observe(timed(.5),h)
    sources,_=c.observe(timed(.6,epoch=1),h)
    assert len(sources)==1 and sources[0].source_epoch==1
    original=timed(.7,epoch=1)
    assert not source_valid(replace(original,rays=original.rays[:250]+original.rays[251:]))


def test_transport_uses_observed_motion_preserves_unknown_and_acquisition():
    original=unknown_at(90)
    moved=transport_scene(original,history(v=.5),.5)
    assert moved.source_first_s==original.source_first_s
    assert moved.rays[375].hit_xy_m is None and moved.rays[375].status=='unknown'
    assert moved.rays[375].ray_origin_xy_m[0]==pytest.approx(original.rays[375].ray_origin_xy_m[0]-.05)


def test_outside_body_does_not_discard_real_small_intrusion():
    assert outside_body([[-.05,-.05],[.05,-.05],[.05,.05],[-.05,.05]],.2,.15)==[]
    assert outside_body([[.05,.074],[.06,.074],[.06,.076],[.05,.076]],.2,.15)


def test_thin_unknown_intersections_have_explicit_queries():
    q,n=sweep_queries(unknown_at(90),.175,.4,FollowConfig())
    assert len(q)>100 and n>0


def test_straight_current_support_and_front_unknown_stop():
    cfg=FollowConfig()
    assert sweep_support([scene()],0.,.2,0.,0.,cfg)['supported']
    assert sweep_support([unknown_at(0),scene()],0.,.2,0.,0.,cfg)['reason']=='current_forward_unobserved'


def test_current_short_return_overrides_good_old_evidence():
    cfg=FollowConfig()
    result=sweep_support([scene(.25),scene()],0.,.2,0.,0.,cfg)
    assert not result['supported'] and result['reason']=='current_occupancy_overrides_history'


@pytest.mark.parametrize('distance',[.16,.20,.25,.30])
def test_new_front_surface_cannot_be_cleared_by_past_far_returns(distance):
    original=scene()
    rays=list(original.rays)
    # 현재 전면 7개 실제 반환을 새 물체 표면으로 대체한다. 예전 clear보다 우선한다.
    for index in range(247,254):
        r=rays[index]
        rays[index]=replace(r,hit_xy_m=tuple(np.array(r.ray_origin_xy_m)+distance*np.array(r.direction_xy)))
    current=replace(original,rays=tuple(rays))
    result=sweep_support([current,original],0.,.2,0.,0.,FollowConfig())
    assert not result['supported'] and result['reason']=='current_occupancy_overrides_history'


def test_old_only_or_epoch_mismatched_evidence_is_not_support():
    original=unknown_at(90)
    current=replace(original,source_epoch=1,rays=tuple(replace(r,source_epoch=1) for r in original.rays))
    output=sweep_support([current,scene()],0.,.2,.175,.175,FollowConfig())
    assert not output['supported']
    expired=replace(scene(),t_ref_s=.7)
    assert not sweep_support([current,expired],0.,.2,.175,.175,FollowConfig())['supported']


def test_missing_motion_history_cannot_transport_old_evidence():
    c=RecentScenes();h=history(end=.4)
    c.observe(timed(.4),h)
    sources,_=c.observe(timed(.6),h)
    assert len(sources)==1  # 현재 scene의 유효성은 호출자의 motion_scene가 별도 검사한다.


@pytest.mark.parametrize('speed,target,steer',[(-.01,.2,0.),(1.2,.2,0.),(0.,1.01,0.),(0.,.2,.38),(math.nan,.2,0.)])
def test_invalid_or_reverse_motion_fails(speed,target,steer):
    assert not sweep_support([scene()],speed,target,steer,0.,FollowConfig())['supported']
