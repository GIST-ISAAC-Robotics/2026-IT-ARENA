"""짧은 경로 선택형 연결: 정적 측정 점군/재구성 광선, 실제 폐루프는 별도."""
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import pytest

from arena_autonomy.corner_corridor import short_prefix_evidence
from arena_autonomy.corridor_observer import CorridorObserver,current_proposal
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.local_scene import LocalScene,SensorRay
from probe_short_boundary_prefix import prefix_geometry
from test_local_follow import history

POINTS=np.asarray(json.loads((Path(__file__).parent/'fixtures/short_prefix_boundary_points.json').read_text())['points'])


def config(enabled=True):
    return FollowConfig(boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate',
                        corridor_fallback_enabled=True,short_boundary_prefix_enabled=enabled)


def scene(now,*,epoch=0,obstacle=False,unknown=False,side='right'):
    points=POINTS.copy()
    if side=='left':points[:,1]*=-1
    if obstacle:points=np.vstack((points,[[.15,0.]]))
    rays=[];first=now-.1
    for i,p in enumerate(points):
        direction=p-[.06,0.];direction/=np.linalg.norm(direction)
        rays.append(SensorRay((.06,0.),tuple(p),first,'return','laser_frame',epoch,i,tuple(direction)))
    if unknown:rays.append(SensorRay((.06,0.),None,first,'unknown','laser_frame',epoch,len(rays),(1.,0.)))
    return LocalScene(now,first,first,'laser_frame',epoch,tuple(rays))


def test_default_off_and_dependent_options():
    assert not FollowConfig().short_boundary_prefix_enabled
    for options in ({'corridor_fallback_enabled':False},{'max_speed_mps':1.1},{'recent_observation_enabled':True}):
        with pytest.raises(ValueError):replace(config(),**options)


@pytest.mark.parametrize('value',[None,1,'true'])
def test_invalid_boolean_rejected(value):
    with pytest.raises(ValueError):replace(config(),short_boundary_prefix_enabled=value)


@pytest.mark.parametrize('side',['left','right'])
def test_integrated_geometry_matches_reviewed_shadow(side):
    points=POINTS.copy()
    if side=='left':points[:,1]*=-1
    original=prefix_geometry(points,side,config())
    actual=short_prefix_evidence(points,side,config())
    assert actual.pop('short_prefix') is True
    correction=actual.pop('origin_arc_correction_m')
    assert actual.pop('one_sided_lead_in_m')==pytest.approx(original.pop('one_sided_lead_in_m')+correction)
    assert actual==original


def test_current_proposal_keeps_default_rejection_and_limits_selected_speed():
    assert current_proposal(scene(.4),0.,'right',0.,.02,config(False))['reason']=='long_outer_reference_unavailable'
    result=current_proposal(scene(.4),0.,'right',0.,.02,config())
    assert result['short_prefix'] and result['point_occupancy_supported']
    assert 0<result['proposed_speed_mps']<=.35
    assert result['stop_budget_m']<=result['path_budget_m']
    assert not result['driving_authorized'] and not result['memory_refresh_authorized']


def test_current_high_speed_cannot_be_hidden_by_a_low_target():
    result=current_proposal(scene(.4),2.,'right',0.,.02,config())
    assert result['reason']=='short_prefix_stop_budget'
    assert not result['point_occupancy_supported'] and result['proposed_speed_mps']==0.


@pytest.mark.parametrize('kind',['occupied','unknown','stale','reverse','frame'])
def test_input_and_point_occupancy_stop_are_preserved(kind):
    sample=scene(.4,obstacle=kind=='occupied',unknown=kind=='unknown')
    if kind=='stale':sample=replace(sample,t_ref_s=1.)
    if kind=='frame':
        sample=replace(sample,source_frame_id='wrong',
                       rays=tuple(replace(ray,source_frame_id='wrong') for ray in sample.rays))
    result=current_proposal(sample,-.1 if kind=='reverse' else 0.,'right',0.,.02,config())
    assert not result['point_occupancy_supported'] and result['proposed_speed_mps']==0.


def test_distinct_confirmation_and_fault_recovery():
    observer=CorridorObserver(config());h=history()
    assert not observer.evaluate(scene(.4),h,0.,'right')['positive']
    assert not observer.evaluate(scene(.5),h,0.,'right')['positive']
    assert not observer.evaluate(scene(.5),h,0.,'right')['positive']
    accepted=observer.evaluate(scene(.6),h,0.,'right')
    assert accepted['positive'] and accepted['short_prefix']
    assert accepted['release_count']==2
    assert not observer.evaluate(scene(.7,unknown=True),h,0.,'right')['positive']
    assert not observer.evaluate(scene(.8),h,0.,'right')['positive']
    observer.reset()
    assert not observer.evaluate(scene(.9),h,0.,'right')['positive']


def test_selected_prefix_does_not_refresh_normal_wall_memory():
    ctl=LocalFollow(config());h=history()
    outputs=[ctl.command(scene(t),h,0.,'right') for t in (.4,.5,.6,.7)]
    assert any(o[0]>0 for o in outputs)
    for o in outputs:
        if o[0]>0:
            assert o[2]['boundary_selection']=='current_short_observed_prefix_candidate'
            assert o[0]<=.35 and o[2]['unfiltered_occupancy']
            assert not o[2]['boundary_memory_from_corridor']
    assert ctl.path is None
