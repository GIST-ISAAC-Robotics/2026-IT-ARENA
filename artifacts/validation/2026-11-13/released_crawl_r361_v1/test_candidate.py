"""선택 범위/실제 운동/전체영역의 의미 있는 거절 경계. 저장 재생은 replay.py."""
import copy
from dataclasses import replace
from types import SimpleNamespace

import pytest

import candidate
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.pass_release_trial import PassReleaseSelector


def case(monkeypatch):
    cfg = FollowConfig(parallel_hold_trial=True, pass_release_trial=True, guard_curve_trial=True,
        traffic_recovery_trial=True, partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate',
        wheelbase_m=.128, length_m=.207, width_m=.15)
    owner = PassReleaseSelector(cfg)
    owner.sides['right'].update(seen=True, passed=True, present=False, clear=2, vanish='rear')
    scene = SimpleNamespace(source_first_s=9.88, t_ref_s=10.)
    baseline = (.7, -.37, dict(reason='local_follow', boundary_from_memory=False, path_source_s=9.88,
        path_points=[[0., 0.], [1., -.5]], partial_observation=dict(allowed=True, reason='fully_observed')))
    region = dict(distance_m=.05, queries=100, unobserved=0, masked_front_samples=0,
                  masked_left=0, masked_right=30, masked_tail_extent_m=.0116)
    output = (0., -.35, dict(selector_mode='RELEASED', selector_choice='parallel', retained_passage=False,
        selector_reason='large_motion_masked_toward_right', reason='parallel_hold_reference_not_straight',
        selector_masked_speed_recheck=dict(applied=False, attempts=[dict(target_mps=t,
            region=copy.deepcopy(region), blocked='large_motion_masked_toward_right') for t in (.35, .25, .15)])))
    accepted = {**region, 'masked_tail_extent_m': .008}
    calls = []
    monkeypatch.setattr(candidate, 'source_valid', lambda s: True)
    def check(*args, **kwargs):
        calls.append((args, kwargs))
        return copy.deepcopy(accepted)
    monkeypatch.setattr(candidate.ps, 'motion_region', check)
    return owner, scene, baseline, output, accepted, calls


def test_full_check_preserves_real_input_state_and_baseline_steering(monkeypatch):
    o,s,b,old,r,calls = case(monkeypatch)
    saved = copy.deepcopy((o.sides,b,old))
    new,e = candidate.assess(o,s,b,old,.12,-.35,.1)
    assert e['applied'] and new[:2] == (.1,-.37)
    args,kwargs = calls[0]
    assert len(calls)==1 and args == (s,.12,.1,-.37,-.35,.1,o.follow_config,o.lat)
    assert 'context' in kwargs and (o.sides,b,old)==saved
    assert e['reference_s']==10. and e['source_first_s']==9.88
    assert not e['unknown_filled'] and not e['state_transition']
    assert new[2]['reason']=='local_follow' and new[2]['selector_region']==r


@pytest.mark.parametrize('kind', ['guard','solo','retained','present','moving','reverse','nan','bool',
    'baseline_hold','baseline_fault','memory','old_path','partial','partial_false','stale','zero_dt','long_dt',
    'low_baseline','positive_selected','unknown_reason','front_attempt','unknown_attempt','missing_attempt',
    'previous_accepted','reordered','smaller_cap','tail_inside','invalid_previous','invalid_steering','policy_off'])
def test_scope_exclusion_does_not_call_geometry(monkeypatch,kind):
    o,s,b,old,r,calls=case(monkeypatch);speed=0.;previous=-.35;dt=.1
    if kind=='guard':o.sides['right']['passed']=False
    if kind=='solo':o.reset()
    if kind=='retained':old[2]['retained_passage']=True
    if kind=='present':o.sides['right']['present']=True
    if kind=='moving':speed=.16
    if kind=='reverse':speed=-.01
    if kind=='nan':speed=float('nan')
    if kind=='bool':speed=True
    if kind=='baseline_hold':b=(0.,b[1],{**b[2],'reason':'follow_gap_hold'})
    if kind=='baseline_fault':b[2]['reason']='follow_input_stop'
    if kind=='memory':b[2]['boundary_from_memory']=True
    if kind=='old_path':b[2]['path_source_s']=9.7
    if kind=='partial':b[2]['partial_observation']['reason']='bounded_side_missing'
    if kind=='partial_false':b[2]['partial_observation']['allowed']=False
    if kind=='stale':s.t_ref_s=10.3
    if kind=='zero_dt':dt=0.
    if kind=='long_dt':dt=.2
    if kind=='low_baseline':b=(.15,b[1],b[2])
    if kind=='positive_selected':old=(.35,old[1],old[2])
    if kind=='unknown_reason':old[2]['selector_reason']='large_motion_region_unobserved'
    a=old[2]['selector_masked_speed_recheck']['attempts']
    if kind=='front_attempt':a[-1]['region']['masked_front_samples']=1
    if kind=='unknown_attempt':a[-1]['region']['unobserved']=1
    if kind=='missing_attempt':a.pop()
    if kind=='previous_accepted':old[2]['selector_masked_speed_recheck']['applied']=True
    if kind=='reordered':a.reverse()
    if kind=='smaller_cap':a[0]['target_mps']=.3
    if kind=='tail_inside':a[-1]['region']['masked_tail_extent_m']=.009
    if kind=='invalid_previous':previous=1.
    if kind=='invalid_steering':b=(b[0],1.,b[2])
    if kind=='policy_off':o.follow_config=replace(o.follow_config,traffic_recovery_trial=False)
    result,e=candidate.assess(o,s,b,old,speed,previous,dt)
    assert result is old and not e['eligible'] and not calls


@pytest.mark.parametrize('change', [{'unobserved':1}, {'masked_front_samples':1}, {'masked_tail_extent_m':.010001}])
def test_new_entire_region_still_rejects_hidden_or_large_motion(monkeypatch,change):
    o,s,b,old,r,calls=case(monkeypatch);r.update(change)
    result,e=candidate.assess(o,s,b,old,0.,-.35,.1)
    assert result is old and e['eligible'] and not e['applied'] and len(calls)==1


def test_raw_negative_cannot_be_hidden_by_existing_absolute_planning_speed(monkeypatch):
    o,s,b,old,r,calls=case(monkeypatch)
    token=candidate._MEASURED.set(-.061)
    try:result,e=candidate.assess(o,s,b,old,.061,-.35,.1)
    finally:candidate._MEASURED.reset(token)
    assert result is old and not calls


def test_import_is_off_and_context_restores_on_failure():
    original=candidate.ps.PassReleaseSelector._select
    from arena_autonomy.bypass_handoff import BypassHandoffFollow
    command=BypassHandoffFollow.command
    with pytest.raises(RuntimeError):
        with candidate.enabled():
            assert candidate.ps.PassReleaseSelector._select is not original
            raise RuntimeError('test interruption')
    assert candidate.ps.PassReleaseSelector._select is original
    assert BypassHandoffFollow.command is command
