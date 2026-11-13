"""GUARD 저속 재검사 연결. 실제 저장 기하는 r451에서 별도로 대조한다."""
from copy import deepcopy
from dataclasses import replace
from types import SimpleNamespace

import pytest

from arena_autonomy import pass_release_trial as ps
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.race_runtime_policy import guard_crawl as gc, released_plan as rp


@pytest.fixture
def scenario(monkeypatch):
    cfg=FollowConfig(partial_observation_policy='bounded_local_v4',parallel_hold_trial=True,
        pass_release_trial=True,guard_curve_trial=True,traffic_recovery_trial=True,
        boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate')
    owner=ps.PassReleaseSelector(cfg)
    owner.sides['left'].update(seen=True,present=True)
    scene=SimpleNamespace(t_ref_s=1.,source_first_s=.9)
    baseline=(.7,.1,dict(reason='local_follow',boundary_from_memory=False,path_source_s=.9,
        partial_observation=dict(allowed=True,reason='fully_observed'),behavior='FOLLOW',path_points=[[0.,0.],[1.,0.]]))
    region=dict(unobserved=0,masked_front_samples=0,queries=100,masked_left=40,
        masked_right=0,masked_tail_extent_m=.012)
    detail=dict(reason='parallel_hold_reference_not_straight',behavior='HOLD',path_points=[[9.,9.]],
        selector_mode='GUARD',selector_choice='parallel',selector_reason=gc.REASONS[0],
        selector_guard_candidate=dict(accepted=False,trigger='parallel_hold_reference_not_straight',
            target_speed_mps=.35,region=deepcopy(region)),
        selector_guard_speed_recheck=dict(applied=False,attempts=[dict(target_mps=c,
            accepted=False,reason=gc.REASONS[0],region=deepcopy(region)) for c in (.25,.15)]))
    selected=(0.,.05,detail)
    state=dict(calls=[],accepted=True)
    monkeypatch.setattr(gc,'source_valid',lambda _:True)
    monkeypatch.setattr(ps,'threat_view',lambda *args:object())
    def check(*args,**kwargs):
        state['calls'].append((args,kwargs))
        return (.1 if state['accepted'] else 0.,.1,dict(selector_guard_candidate=dict(accepted=state['accepted']),
            selector_reason='guard_curve_observed_motion' if state['accepted'] else 'guard_curve_rejected_toward_left'))
    monkeypatch.setattr(owner,'_guard_curve_at_cap',check)
    def run(measured=0.,speed=0.,**changes):
        return gc.assess(owner,scene,baseline,selected,speed,.05,.02,changes.get('raw_scan',object()),measured=measured)
    return owner,scene,baseline,selected,state,run


def test_one_full_check_keeps_side_history_and_measured_motion(scenario):
    owner,scene,base,old,state,run=scenario
    before=deepcopy(owner.sides)
    result=run(measured=.13,speed=.13)
    assert result[0]==.1 and len(state['calls'])==1
    args,kwargs=state['calls'][0]
    assert args[2] is base and args[4:7]==(.13,.05,.02) and kwargs==dict(target_cap=.1)
    assert owner.sides==before and owner.mode=='GUARD'
    assert result[2]['selector_guard_crawl_retry']['state_transition'] is False
    assert result[2]['selector_guard_crawl_retry']['unknown_filled'] is False
    assert not set(('reason','behavior','path_points')).intersection(args[8])


@pytest.mark.parametrize('measured,speed',[(-.01,.01),(.16,.16),(float('nan'),0.),(0.,.01)])
def test_signed_motion_and_speed_scope(scenario,measured,speed):
    *_,state,run=scenario
    assert run(measured=measured,speed=speed)[0]==0.
    assert state['calls']==[]


@pytest.mark.parametrize('change',['memory','old_path','unobserved','front_mask','partial',
    'missing_retry','already_accepted','not_guard','negative_raw','no_recovery','no_source'])
def test_noneligible_inputs_do_not_trigger_extra_geometry(scenario,monkeypatch,change):
    owner,scene,base,old,state,run=scenario
    if change=='memory':base[2]['boundary_from_memory']=True
    elif change=='old_path':base[2]['path_source_s']=.8
    elif change=='partial':base[2]['partial_observation']['allowed']=False
    elif change=='missing_retry':old[2]['selector_guard_speed_recheck']['attempts'].pop()
    elif change=='already_accepted':old[2]['selector_guard_speed_recheck']['applied']=True
    elif change=='not_guard':owner.reset()
    elif change=='negative_raw':pass
    elif change=='no_recovery':owner.follow_config=replace(owner.follow_config,traffic_recovery_trial=False)
    elif change=='no_source':monkeypatch.setattr(gc,'source_valid',lambda _:False)
    else:old[2]['selector_guard_candidate']['region']['unobserved' if change=='unobserved' else 'masked_front_samples']=1
    result=run(measured=-.01,speed=.01) if change=='negative_raw' else run()
    assert result is old and state['calls']==[]


def test_failed_full_check_preserves_original_stop(scenario):
    owner,_,_,old,state,run=scenario
    before=deepcopy(owner.sides);state['accepted']=False
    result=run()
    assert result[:2]==old[:2] and result[2]['selector_reason']==old[2]['selector_reason']
    assert owner.sides==before
    assert not result[2]['selector_guard_crawl_retry']['applied']


def test_selected_policy_wiring_and_shadow_restore(scenario,monkeypatch):
    owner,scene,base,old,state,_=scenario
    monkeypatch.setattr(ps.PassReleaseSelector,'_select',lambda *a,**kw:old)
    restored=ps.PassReleaseSelector._select
    with rp.enabled(shadow=True):
        assert owner._select(scene,base,old,0.,.05,.02,raw_scan=object()) is old
    assert ps.PassReleaseSelector._select is restored
    with rp.enabled():
        result=owner._select(scene,base,old,0.,.05,.02,raw_scan=object())
    assert result[0]==.1 and len(state['calls'])==2
    assert ps.PassReleaseSelector._select is restored


def test_positive_original_output_is_not_replaced(scenario):
    owner,scene,base,old,state,_=scenario
    positive=(.2,old[1],old[2])
    assert gc.assess(owner,scene,base,positive,0.,.05,.02,object(),measured=0.) is positive
    assert state['calls']==[]


def test_real_guard_success_keeps_baseline_reason_behavior_and_path(scenario,monkeypatch):
    owner,scene,base,old,_,run=scenario
    base[2]['path_points']=[[-1.,0.],[0.,0.],[1.,0.],[2.,0.]]
    old[2]['selector_sides']=deepcopy(owner.sides)
    monkeypatch.delattr(owner,'_guard_curve_at_cap')
    monkeypatch.setattr(ps,'threat_view',lambda *a:dict(frame=ps.mx.ExecutionFrame(base[2]['path_points'])))
    monkeypatch.setattr(ps,'motion_region',lambda *a,**kw:dict(unobserved=0,
        masked_front_samples=0,queries=100,masked_left=40,masked_right=0,masked_tail_extent_m=.009))
    result=run()
    assert result[0]==.1 and result[2]['selector_choice']=='guard_baseline'
    assert result[2]['reason']==base[2]['reason']=='local_follow'
    assert result[2]['behavior']==base[2]['behavior']=='FOLLOW'
    assert result[2]['path_points']==base[2]['path_points']!=old[2]['path_points']
    from arena_autonomy.race_runtime_policy.retained_recheck._present import supported
    assert supported((result[0],result[1],dict(handoff_state='FOLLOW',normal=result[2])))
