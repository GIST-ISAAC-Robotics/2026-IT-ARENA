"""거리 계약/양측 질의/단일 최종 검사 반례. 실행은 지정 담당자에게 맡긴다."""
import copy
from types import SimpleNamespace

import numpy as np
import pytest

from support import load
load('python')
import budget
import candidate


def test_inverse_preserves_measured_braking_and_minimum_distance():
    execution=budget.ps.mx.ExecutionConfig()
    accepted=budget.speed_from_distance(.03,0.,.10,.32,execution)
    assert accepted['accepted'] and 0<accepted['proposed_target_mps']<.10
    speed=accepted['proposed_target_mps']
    assert speed*.32+speed*speed/4<=.03
    rejected=budget.speed_from_distance(.03,.15,.10,.32,execution)
    assert not rejected['accepted'] and rejected['reason']=='measured_braking_distance_exceeds_budget'
    assert rejected['measured_distance_m']==pytest.approx(.053625)
    assert not budget.speed_from_distance(.009999,0.,.10,.32,execution)['accepted']


@pytest.mark.parametrize('sign',(-1,1))
def test_prefix_uses_actual_unknown_side_and_earliest_distance(sign):
    points=np.array([[-.05,sign*.080],[-.05,sign*.086]])
    result=budget.observed_prefix(points,np.array([.01,.0345]),np.array([0.,.01,.02,.03,.0345]),
        np.array([False,False]),np.array([False,False]),.207,.15,budget.ps.SelectorConfig())
    assert result['allowed_distance_m']==.03 and result['first_bad_distance_m']==.0345
    assert result['unknown_geometry_bad_queries']==1 and not result['unknown_filled']


def test_observed_far_geometry_does_not_gain_new_10mm_restriction():
    result=budget.observed_prefix(np.array([[.3,.3],[-.3,-.3]]),np.array([0.,0.]),np.array([0.,.01,.02]),
        np.array([True,True]),np.array([False,False]),.207,.15,budget.ps.SelectorConfig())
    assert result['first_bad_distance_m'] is None and result['allowed_distance_m']==.02


@pytest.mark.parametrize('point',([.11,.08],[-.12,.08],[-.05,.086]))
def test_masked_front_or_tail_still_bounds_prefix(point):
    result=budget.observed_prefix(np.array([point]),np.array([.01]),np.array([0.,.01,.02]),
        np.array([False]),np.array([True]),.207,.15,budget.ps.SelectorConfig())
    assert result['allowed_distance_m']==0 and result['masked_geometry_bad_queries']==1


def test_opposite_steering_poses_both_retained():
    cfg=SimpleNamespace(wheelbase_m=.128,length_m=.207,width_m=.15)
    distances=budget.fixed_distances(.0345,[-.4,.4],cfg)
    assert all(v in distances for v in (0.,.01,.02,.03,.0345))
    left=budget.poses_at_distances(.4,distances,cfg)
    right=budget.poses_at_distances(-.4,distances,cfg)
    assert np.allclose(left[:,0],right[:,0]) and np.allclose(left[:,1:],-right[:,1:])


def fixture(monkeypatch):
    output=[.45,.395,dict(handoff_state='FOLLOW',normal=dict(reason='local_follow',selector_mode='SOLO',
        selector_choice='baseline',boundary_from_memory=False,partial_observation=dict(allowed=True,reason='fully_observed')))]
    current=dict(speed_mps=0.,previous_steering_rad=.406,dt_s=.1,reference_s=10.,scan={'measurements':{}})
    before=dict(speed_mps=0.,steering_rad=.395,reference_s=10.,path_source_s=9.88,original_reference_s=9.99,
        path_points=[[0.,0.]],reason='route_current_large_motion_region_unobserved',current_route_proof={'accepted':False},
        current_relative_body_retry=dict(applied=False,cap_mps=.10,proof={'reason':'outside_small_body_side'}))
    proposal=dict(accepted=True,allowed_distance_m=.03,proposed_target_mps=.087,reason='single_distance_budget_proposal')
    region=dict(distance_m=.029734,queries=3,unobserved=1,masked_left=0,masked_right=0,masked_front_samples=0,masked_tail_extent_m=0.)
    proof=dict(accepted=True,reason='bounded_relative_body_nonreturn_risk',unknown_filled=False)
    calls=[]
    def propose(*args,**kwargs):calls.append(('proposal',args));return proposal
    def core(*args):
        calls.append(('core',args));return {**before,'speed_mps':args[0][0],'reason':'rebase_current_follow'}
    def motion(*args,context,**kwargs):
        calls.append(('motion',args));context.update(queries=np.array([[-.05,.08],[.2,0.],[.3,0.]]),
            free=np.array([False,True,True]),masked=np.array([False,False,False]),padding=np.array([.115]));return region
    def gap(*args,**kwargs):calls.append(('gap',args,kwargs));return proof
    monkeypatch.setattr(candidate,'propose',propose)
    monkeypatch.setattr(candidate.ps,'motion_region',motion)
    monkeypatch.setattr(candidate.ps,'lateral_config',lambda cfg:None)
    monkeypatch.setattr(candidate,'MeasurementScan',lambda **kwargs:None)
    monkeypatch.setattr(candidate,'bounded_body_nonreturn',gap)
    cfg=SimpleNamespace(length_m=.207,width_m=.15,max_steering_rad=.4578)
    return dict(output=output,current=current,before=before,proposal=proposal,region=region,proof=proof,calls=calls,core=core,cfg=cfg)


def run(f):
    return candidate.try_from_current_result(f['before'],f['output'],{},f['current'],None,None,f['cfg'],
        core_rebase=f['core'],require_current_region=True)


def test_one_proposal_one_final_check_same_speed_history_steering(monkeypatch):
    f=fixture(monkeypatch);saved=copy.deepcopy((f['before'],f['output'],f['current']))
    result=run(f);assert result['speed_mps']==.087 and result[candidate.DIAGNOSTIC]['applied']
    assert [c[0] for c in f['calls']]==['proposal','core','motion','gap']
    assert f['calls'][2][1][1:6]==(0.,.087,.395,.406,.1)
    assert f['calls'][3][2]=={'relative_neighbor':True}
    assert result['current_route_proof']['region']['unobserved']==1
    assert (f['before'],f['output'],f['current'])==saved


@pytest.mark.parametrize('case',('positive','guard','released','memory','partial','reverse','fast','dt','previous','wrong_stop'))
def test_original_positive_and_nontarget_are_identical(monkeypatch,case):
    f=fixture(monkeypatch)
    if case=='positive':f['before']['speed_mps']=.15
    if case in ('guard','released'):f['output'][2]['normal']['selector_mode']=case.upper()
    if case=='memory':f['output'][2]['normal']['boundary_from_memory']=True
    if case=='partial':f['output'][2]['normal']['partial_observation']['allowed']=False
    if case in ('reverse','fast'):f['current']['speed_mps']=-.001 if case=='reverse' else .151
    if case=='dt':f['current']['dt_s']=.101
    if case=='previous':f['current']['previous_steering_rad']=.46
    if case=='wrong_stop':f['before']['current_relative_body_retry']['proof']['reason']='unbounded_nonreturn'
    assert run(f) is f['before'] and not f['calls']


@pytest.mark.parametrize('case',('new_unknown','near_occupied','mask_front','mask_tail','distance','no_budget'))
def test_proposal_cannot_bypass_final_failure_or_retry_lower(monkeypatch,case):
    f=fixture(monkeypatch)
    if case in ('new_unknown','near_occupied'):
        f['proof'].update(accepted=False,reason='outside_small_body_side' if case=='new_unknown' else 'known_occupied_padding')
    if case=='mask_front':f['region'].update(masked_left=1,masked_front_samples=1)
    if case=='mask_tail':f['region'].update(masked_left=1,masked_tail_extent_m=.01001)
    if case=='distance':f['region']['distance_m']=.030001
    if case=='no_budget':f['proposal'].update(accepted=False,reason='measured_braking_distance_exceeds_budget')
    result=run(f)
    assert result['speed_mps']==0 and not result[candidate.DIAGNOSTIC]['applied']
    assert sum(c[0]=='core' for c in f['calls'])<=1 and sum(c[0]=='motion' for c in f['calls'])<=1


def test_final_fully_observed_region_needs_no_unknown_exception(monkeypatch):
    f=fixture(monkeypatch);f['region']['unobserved']=0
    result=run(f)
    assert result[candidate.DIAGNOSTIC]['applied'] and all(c[0]!='gap' for c in f['calls'])
