"""명시 현재 region proof의 작은 자기 가림만 수용하며 관측/가림을 구분한다."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace
import importlib
import math
import numpy as np
import pytest
from arena_autonomy import pass_release_trial as ps


def sample():
    frame=ps.mx.ExecutionFrame(np.array([[-1.,0.],[0.,0.],[1.,0.],[2.,0.]]))
    context=dict(queries=np.array([[.02,-.077],[.03,-.081],[.02,.077],[.03,.081]]),
        free=np.array([True,False,True,False]),masked=np.array([False,True,False,True]))
    sides={name:dict(current_m=.075) for name in ('left','right')}
    region=dict(queries=4,unobserved=0,masked_front_samples=0,masked_left=1,masked_right=1,masked_tail_extent_m=.006)
    return frame,context,sides,region


def check(data,**overrides):
    frame,context,sides,region=data
    return ps.observed_guard_approach(frame,context,sides,['right','left'],
        region=overrides.get('region',region),selector_config=overrides.get('selector_config',ps.SelectorConfig()),
        length_m=.2,width_m=.15)


def test_explicit_region_keeps_mask_separate_from_free_and_state():
    data=sample();before=deepcopy(data[1:]);out=check(data)
    assert out['accepted'] and out['bounded_mask_eligible'] and out['residual_hidden_object_risk']
    assert not out['unknown_filled']
    assert out['basis']=='observed_with_mask_budget'
    assert out['sides']=={side:dict(samples=2,currently_observed=1,known_masked=1,bounded_mask=1,unaccepted_masked=0,unknown=0,
        masked_x_span_m=0.,observed_before_mask=1,observed_after_mask=0) for side in ('left','right')}
    assert data[2:]==before[1:]
    for key in data[1]:np.testing.assert_array_equal(data[1][key],before[0][key])


def test_legacy_call_stays_strict_without_proof():
    frame,context,sides,_=sample()
    out=ps.observed_guard_approach(frame,context,sides,['right'])
    assert out==dict(accepted=False,unknown_filled=False,sides={'right':dict(samples=2,currently_observed=1)})
    assert not check(sample(),region=None)['accepted']
    assert not check(sample(),selector_config=None)['accepted']


@pytest.mark.parametrize('change',('unknown','front','over_budget','nan_tail','wrong_count','mismatched_mask'))
def test_invalid_or_unsafe_region_cannot_supply_bounded_permission(change):
    data=sample();region=data[3]
    if change=='unknown':region['unobserved']=1
    elif change=='front':region['masked_front_samples']=1
    elif change=='over_budget':region['masked_tail_extent_m']=.010001
    elif change=='nan_tail':region['masked_tail_extent_m']=float('nan')
    elif change=='wrong_count':region['queries']=5
    else:region['masked_left']=0
    out=check(data)
    assert not out['accepted'] and not out['bounded_mask_eligible']
    assert out['sides']['right']['unknown']==0
    assert out['sides']['right']['known_masked']==out['sides']['right']['unaccepted_masked']==1


def test_mask_only_one_side_cannot_borrow_free_from_other_side():
    data=sample();data[1]['free'][0]=False;data[1]['masked'][0]=True;data[3]['masked_right']=2
    out=check(data)
    assert not out['accepted'] and out['bounded_mask_eligible']
    assert out['sides']['right']['currently_observed']==0


def test_finite_block_or_general_unknown_cannot_be_relabelled_by_region_summary():
    data=sample();data[1]['masked'][1]=False;data[3]['masked_right']=0
    # The classification array wins even if a caller's summary says unknown=0.
    assert not check(data)['accepted']


@pytest.mark.parametrize('point',([.100001,-.081],[-.100001,-.081],[.03,-.085001]))
def test_query_geometry_cannot_hide_behind_region_summary(point):
    data=sample();data[1]['queries'][1]=point
    assert not check(data)['accepted']


def test_long_masked_span_is_rejected_without_requiring_bracketing_free():
    data=sample();data[1]['queries']=np.vstack((data[1]['queries'],[-.03,-.081]))
    data[1]['free']=np.append(data[1]['free'],False);data[1]['masked']=np.append(data[1]['masked'],True)
    data[3].update(queries=5,masked_right=2)
    assert not check(data)['accepted']  # .06m span exceeds existing .05m hidden-width budget.


@pytest.mark.parametrize('package',('arena_autonomy.current_path_recheck','arena_autonomy.race_runtime_policy.retained_recheck'))
@pytest.mark.parametrize('unknown',(False,True))
def test_absent_guard_checks_region_before_approach_and_keeps_identity(package,unknown,monkeypatch):
    module=importlib.import_module(package+'._guard')
    frame,ctx,sides,region=sample();cfg=SimpleNamespace(guard_curve_trial=True,traffic_recovery_trial=True,
        length_m=.2,width_m=.15,object_motion_bound_mps=.5,guard_observation_policy='selected_r181')
    normal=dict(selector_sides={name:dict(present=False,seen=True,passed=False) for name in sides},
        selector_guard_candidate=dict(accepted=True,guarded_sides={name:dict(anchor_m=.0) for name in sides}),
        partial_observation=dict(allowed=True,reason='fully_observed'),path_source_s=.9)
    output=(.15,0.,dict(normal=normal));old={};current=dict(speed_mps=0.,previous_steering_rad=0.,dt_s=.02,reference_s=1.)
    scene=SimpleNamespace(t_ref_s=1.,source_first_s=.9)
    monkeypatch.setattr(module,'supported',lambda _:True)
    monkeypatch.setattr(module,'forward_planning_speed',lambda c,v:v)
    monkeypatch.setattr(module.geometry,'rebase',lambda *a:dict(speed_mps=.15,steering_rad=0.,path_points=[[0.,0.],[1.,0.]],reference_s=1.,path_source_s=.9))
    monkeypatch.setattr(module,'current_forward_coverage',lambda *a:True)
    monkeypatch.setattr(ps,'lateral_config',lambda c:object())
    monkeypatch.setattr(ps,'threat_view',lambda *a:dict(frame=frame,left=dict(present=False),right=dict(present=False)))
    monkeypatch.setattr(ps,'command_arc',lambda *a:np.array([[0.,0.,0.]]))
    monkeypatch.setattr(ps,'extrema_arc',lambda *a:np.array([[0.,0.,0.]]))
    monkeypatch.setattr(ps,'side_extent',lambda f,p,s,c:.075 if len(p)==1 else .09)
    order=[]
    def motion(*a,context):
        order.append('region');context.update(deepcopy(ctx));context['padding']=np.zeros(4);r=deepcopy(region)
        if unknown:r['unobserved']=1;context['masked'][1]=False
        return r
    monkeypatch.setattr(ps,'motion_region',motion)
    monkeypatch.setattr(module,'bounded_rear_fringe',lambda *a:dict(accepted=False))
    original=ps.observed_guard_approach
    def observe(*a,**kw):order.append('approach');return original(*a,**kw)
    monkeypatch.setattr(ps,'observed_guard_approach',observe)
    before=deepcopy((output,current))
    result=module.rebase(output,old,current,object(),scene,object(),cfg)
    assert (output,current)==before
    if unknown:
        assert result['speed_mps']==0 and order==['region']
        assert result['reason']=='guard_rebase_toward_unreleased_side'
    else:
        assert result['speed_mps']==.15 and order==['region','approach']
        assert result['current_approach']['accepted'] and not result['state_transition']


@pytest.mark.parametrize('runtime',(False,True))
@pytest.mark.parametrize('change',('near_front','near_side','missing_front'))
def test_selected_policy_current_real_rays_keep_obstacle_and_unknown_stop(runtime,change):
    # Real saved current scene; only explicit selection and the stated ray fault change.
    from test_guard_observation_rebase import sample as saved_sample
    from arena_autonomy.follow_trace import restore_call
    from arena_autonomy.scan_input import scan_angles
    from arena_autonomy.scene_motion import motion_scene
    out,old,now,scene,history,cfg=saved_sample.__wrapped__()
    cfg=replace(cfg,guard_observation_policy='selected_r181')
    _,scan,_=restore_call(now,history.config)
    angles=scan_angles(scan)+history.config.lidar_yaw_rad;values=list(scan.ranges_m)
    if change=='near_side':
        for i,a in enumerate(angles):
            a=math.remainder(a,math.tau)
            if math.radians(65)<a<math.radians(135):values[i]=.07/math.sin(a)
    else:
        index=min(range(len(angles)),key=lambda i:abs(math.remainder(angles[i],math.tau)))
        values[index]=.05 if change=='near_front' else math.inf
    scan=replace(scan,ranges_m=tuple(values));now['scan']['measurements']['ranges_m']=values
    scene=motion_scene(scan,history,now['reference_s'],timing_verified=True,
        epoch=now['epoch'],rear_half_angle_deg=now['rear_half_angle_deg'])
    policy=importlib.import_module('arena_autonomy.race_runtime_policy.observation_policy' if runtime
                                  else 'arena_autonomy.guard_observation_policy')
    result=policy.rebase(out,old,now,scene,history,cfg)
    assert result['speed_mps']==0.
