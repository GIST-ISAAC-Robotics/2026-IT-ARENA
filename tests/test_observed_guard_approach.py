import numpy as np
import pytest
from dataclasses import replace
from types import SimpleNamespace
import arena_autonomy.pass_release_trial as m
from arena_autonomy.m3_execution_trial import ExecutionFrame
from arena_autonomy.local_follow import FollowConfig



def data():
    frame=ExecutionFrame(np.c_[np.linspace(-.3,1.,36),np.zeros(36)])
    context=dict(queries=np.array([[.12,-.09],[.15,-.11],[.12,.10]]),free=np.ones(3,bool),masked=np.zeros(3,bool))
    sides={'right':{'current_m':.075},'left':{'current_m':.075}}
    return frame,context,sides

def test_current_observed_toward_side_allows_without_claiming_unknown_free():
    frame,context,sides=data()
    out=m.observed_guard_approach(frame,context,sides,['right'])
    assert out['accepted'] and out['sides']['right']=={'samples':2,'currently_observed':2}
    assert out['unknown_filled'] is False

@pytest.mark.parametrize('masked',[False,True])
def test_blocked_or_unknown_or_structural_mask_on_approach_side_rejects(masked):
    frame,context,sides=data();context['free'][0]=False;context['masked'][0]=masked
    assert not m.observed_guard_approach(frame,context,sides,['right'])['accepted']

def test_two_guarded_sides_must_both_be_observed():
    frame,context,sides=data();context['free'][2]=False
    assert m.observed_guard_approach(frame,context,sides,['right'])['accepted']
    assert not m.observed_guard_approach(frame,context,sides,['right','left'])['accepted']

def test_missing_or_zero_approach_sample_is_not_a_permission():
    frame,context,sides=data();context['queries'][:,1]=0.
    assert not m.observed_guard_approach(frame,context,sides,['right'])['accepted']
    assert not m.observed_guard_approach(frame,{},sides,['right'])['accepted']

def test_left_mirror():
    frame,context,sides=data();context['queries']*=np.array([1,-1])
    assert m.observed_guard_approach(frame,context,sides,['left'])['accepted']

@pytest.mark.parametrize('current,recovery,free,expected',[
    (True,True,True,True),(False,True,True,False),(True,False,True,False),
    (True,True,False,False)])
def test_guard_branch_requires_current_path_enabled_policy_and_current_free(monkeypatch,current,recovery,free,expected):
    cfg=FollowConfig(partial_observation_policy='bounded_local_v4',parallel_hold_trial=True,
        pass_release_trial=True,guard_curve_trial=True,traffic_recovery_trial=recovery,
        boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate')
    c=m.PassReleaseSelector(cfg);c.sides['right'].update(seen=True,present=True,passed=False)
    c.guard_anchor['right']=-.1
    frame,context,_=data()
    def region(*args,context=None,**kw):
        context.update(queries=np.array([[.12,-.11],[.15,-.13]]),free=np.full(2,free),masked=np.zeros(2,bool),padding=np.ones(2)*.03)
        return dict(unobserved=0 if free else 2,masked_left=0,masked_right=0,masked_tail_extent_m=0.,masked_front_samples=0)
    monkeypatch.setattr(m,'motion_region',region)
    scene=SimpleNamespace(source_first_s=.9,t_ref_s=1.)
    detail=dict(reason='local_follow',boundary_from_memory=not current,path_source_s=.9 if current else .8)
    out=c._guard_curve(scene,{'frame':frame},(.35,-.15,detail),
        (0.,0.,{'reason':'parallel_hold_reference_not_straight'}),0.,0.,.02,.1,{},None)
    assert (out[0]>0)==expected
    assert c.mode=='GUARD' and c.sides['right']['present'] and not c.sides['right']['passed']
    if expected:
        assert out[2]['selector_guard_candidate']['current_approach']['accepted']

def test_baseline_hold_and_positive_parallel_are_not_replaced():
    cfg=FollowConfig(partial_observation_policy='bounded_local_v4',parallel_hold_trial=True,
        pass_release_trial=True,guard_curve_trial=True,boundary_evidence_mode='corner_distance_candidate',
        occupancy_timing_mode='per_ray_candidate')
    c=m.PassReleaseSelector(cfg)
    args=(None,None,(0.,0.,{'reason':'follow_gap_hold'}),(0.,0.,{'reason':'parallel_hold_reference_not_straight'}),0.,0.,.02,.1,{},None)
    assert c._guard_curve(*args) is None
    args=list(args);args[2]=(.35,0.,{'reason':'local_follow'});args[3]=(.2,0.,{'reason':'parallel_hold_follow'})
    assert c._guard_curve(*args) is None
