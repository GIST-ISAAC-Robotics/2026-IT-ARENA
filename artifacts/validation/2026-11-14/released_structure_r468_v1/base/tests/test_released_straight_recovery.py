"""통과 후 작은 직진 후보의 관측·상태·실측 속도 경계. 실차 증명이 아니다."""
from dataclasses import replace
from unittest.mock import patch
import math
import numpy as np
import pytest

from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.pass_release_trial import PassReleaseSelector
import arena_autonomy.released_straight_recovery as recovery
from test_lateral_intent import straight, car
from test_local_follow import history


def case(extra=(), dropout=None):
    config=FollowConfig(partial_observation_policy='bounded_local_v4', parallel_hold_trial=True,
        pass_release_trial=True,guard_curve_trial=True,traffic_recovery_trial=True,
        piecewise_wall_trial=True,boundary_evidence_mode='corner_distance_candidate',
        occupancy_timing_mode='per_ray_candidate')
    owner=PassReleaseSelector(config)
    owner.sides['right'].update(seen=True,passed=True,present=False,clear=2,vanish='benign')
    scene,path=straight(0.,extra=extra,dropout=dropout)
    baseline=(.5,.25,dict(reason='local_follow',path_points=path.tolist(),boundary_from_memory=False,
        path_source_s=scene.source_first_s,partial_observation=dict(allowed=True,reason='fully_observed')))
    selected=(0.,.25,dict(selector_reason='large_motion_region_unobserved'))
    return owner,scene,history(),baseline,selected


def test_current_straight_uses_real_geometry_and_keeps_inputs():
    c,s,h,b,out=case()
    states={side:dict(values) for side,values in c.sides.items()}
    candidate=recovery.compare_released_straight(c,s,h,b,out,0.,0.,.02)
    assert candidate['accepted'] and candidate['reason']=='released_current_straight'
    assert 0<candidate['speed_mps']<=.2 and candidate['steering_rad']==0
    assert c.sides==states and out[0]==0 and b[0]==.5
    assert not candidate['unknown_filled'] and not candidate['body_standstill_verified']


@pytest.mark.parametrize('kind',['occupied','missing'])
def test_current_front_obstacle_or_missing_rays_cannot_be_bypassed(kind):
    c,s,h,b,out=case(extra=(car(.18,0.,0.),) if kind=='occupied' else (),
        dropout=(lambda deg: np.abs(deg)<5) if kind=='missing' else None)
    candidate=recovery.compare_released_straight(c,s,h,b,out,0.,0.,.02)
    assert not candidate['accepted']


@pytest.mark.parametrize('kind',['guard','solo','legacy','path_memory','path_old','baseline_hold','partial','stale'])
def test_not_a_replacement_for_lost_path_side_threat_or_stop(kind):
    c,s,h,b,out=case()
    if kind=='guard':c.sides['right']['present']=True
    elif kind=='solo':c.reset()
    elif kind=='legacy':c.follow_config=replace(c.follow_config,piecewise_wall_trial=False)
    elif kind=='path_memory':b[2]['boundary_from_memory']=True
    elif kind=='path_old':b[2]['path_source_s']-=.1
    elif kind=='baseline_hold':b=(0.,b[1],dict(b[2],reason='follow_gap_hold'))
    elif kind=='partial':b[2]['partial_observation']['reason']='bounded_side_missing'
    elif kind=='stale':s=replace(s,t_ref_s=s.t_ref_s+1.)
    assert recovery.compare_released_straight(c,s,h,b,out,0.,0.,.02) is None


@pytest.mark.parametrize('speed',[-.01,.36,math.nan])
def test_real_speed_is_not_replaced_with_stopped_target(speed):
    c,s,h,b,out=case()
    candidate=recovery.compare_released_straight(c,s,h,b,out,speed,.25,.02)
    assert not candidate['accepted'] and candidate['reason']=='outside_low_speed_scope'


def controlled_previous_region(*args,**kwargs):
    # 상태 전이 시험 전용. 실제 관측/점유는 위 시험과 저장 입력에서 비교한다.
    old=args[4]
    return dict(distance_m=.04,queries=10,unobserved=int(abs(old)>1e-9),
                masked_left=0,masked_right=0,masked_tail_extent_m=0.,masked_front_samples=0)


def test_presteer_never_uses_settled_hypothesis_for_positive_command():
    c,s,h,b,out=case()
    with patch.object(recovery,'motion_region',side_effect=controlled_previous_region):
        candidate=recovery.compare_released_straight(c,s,h,b,out,0.,.25,.02)
    assert candidate['accepted'] and candidate['reason']=='released_straight_presteer'
    assert candidate['speed_mps']==0. and candidate['steering_rad']==pytest.approx(.19)
    assert candidate['command']['previous_steering_rad']==.25
    assert candidate['command']['blocked']=='large_motion_region_unobserved'
    assert candidate['settled_hypothesis']['accepted'] and candidate['wheel_support']['supported']


@pytest.mark.parametrize('kind',['rotating','recent_wheel_motion','no_history','wheel_not_observed','geometry'])
def test_presteer_requires_recent_quiet_motion_wheel_observation_and_useful_route(kind):
    c,s,h,b,out=case()
    if kind=='rotating':h=history(w=.2)
    elif kind=='recent_wheel_motion':h=history(v=.1)
    elif kind=='no_history':h=None
    with patch.object(recovery,'motion_region',side_effect=controlled_previous_region):
        if kind=='wheel_not_observed':
            with patch.object(recovery.mx,'presteer_support',return_value=dict(supported=False)):
                candidate=recovery.compare_released_straight(c,s,h,b,out,0.,.25,.02)
        elif kind=='geometry':
            with patch.object(recovery.mx,'arc_checks',return_value=dict(geometry_ok=False,geometry=[])):
                candidate=recovery.compare_released_straight(c,s,h,b,out,0.,.25,.02)
        else:candidate=recovery.compare_released_straight(c,s,h,b,out,0.,.25,.02)
    assert not candidate['accepted']
