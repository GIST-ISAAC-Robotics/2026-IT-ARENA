import copy
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest

spec=importlib.util.spec_from_file_location('r370_test_policy',Path(__file__).with_name('candidate.py'))
policy=importlib.util.module_from_spec(spec);spec.loader.exec_module(policy)


def fixture(monkeypatch):
    output=[.5,.28,dict(handoff_state='FOLLOW',normal=dict(selector_mode='SOLO',boundary_from_memory=False,
             partial_observation=dict(allowed=True,reason='fully_observed')))]
    current=dict(speed_mps=0.,reference_s=10.,previous_steering_rad=.27,dt_s=.02,scan={'measurements':{}})
    before=dict(speed_mps=0.,steering_rad=.28,reference_s=10.,reason='route_current_large_motion_region_unobserved',
                current_solo_gap_retry=dict(applied=False,cap_mps=.15),
                current_route_proof=dict(accepted=False,region={}))
    core=dict(speed_mps=.1,steering_rad=.28,reference_s=10.,reason='rebase_current_follow')
    region=dict(queries=2,unobserved=1,masked_left=1,masked_right=0,masked_front_samples=0,masked_tail_extent_m=.006)
    proof=dict(accepted=True,unknown_filled=False)
    calls=[]
    monkeypatch.setattr(policy.prior,'rebase',lambda *a,**k:before)
    def rebase(*args,**kwargs):calls.append(args);return core
    monkeypatch.setattr(policy.prior.prior.prior.base.candidate_policy,'rebase',rebase)
    def motion(*args,context,**kwargs):
        calls.append(args)
        context.update(queries=np.array([[-.04,.079],[-.04,.08]]),free=np.array([False,False]),
                       masked=np.array([False,True]),padding=np.array([.1]))
        return region
    monkeypatch.setattr(policy.ps,'motion_region',motion)
    monkeypatch.setattr(policy.ps,'lateral_config',lambda cfg:None)
    monkeypatch.setattr(policy,'MeasurementScan',lambda **kwargs:None)
    monkeypatch.setattr(policy,'thaw',lambda value:value)
    monkeypatch.setattr(policy,'bounded_body_nonreturn',lambda *a,**kw:proof)
    cfg=SimpleNamespace(length_m=.207,width_m=.15)
    return output,current,before,core,region,proof,calls,cfg


def run(parts):
    output,current,*_,cfg=parts
    return policy.rebase(output,{},current,None,None,cfg,require_current_region=True)


def test_actual_speed_previous_steer_and_current_occupancy_are_rechecked(monkeypatch):
    parts=fixture(monkeypatch);output,current,before,core,region,proof,calls,cfg=parts
    current['speed_mps']=.12
    saved=copy.deepcopy((output,current,before))
    result=run(parts)
    assert result['speed_mps']==.1 and result['current_relative_body_retry']['applied']
    assert result['current_route_proof']['region']==region
    assert calls[0][0][0]==.1 and calls[0][2] is current
    assert calls[1][1:6]==(.12,.1,.28,.27,.02)
    assert (output,current,before)==saved


@pytest.mark.parametrize('case',['positive','obstacle','guard','released','hold','memory','partial','low',
                                  'fast','reverse','nan','no_retry','wrong_cap','already_accepted'])
def test_unrelated_cases_are_unchanged(monkeypatch,case):
    parts=fixture(monkeypatch);out,current,before,_,_,_,calls,_=parts
    if case=='positive':before['speed_mps']=.2
    if case=='obstacle':before['reason']='rebase_current_collision'
    if case in ('guard','released'):out[2]['normal']['selector_mode']=case.upper()
    if case=='hold':out[2]['handoff_state']='HOLD'
    if case=='memory':out[2]['normal']['boundary_from_memory']=True
    if case=='partial':out[2]['normal']['partial_observation']['allowed']=False
    if case=='low':out[0]=.15
    if case in ('fast','reverse','nan'):current['speed_mps']={'fast':.16,'reverse':-.01,'nan':float('nan')}[case]
    if case=='no_retry':before.pop('current_solo_gap_retry')
    if case=='wrong_cap':before['current_solo_gap_retry']['cap_mps']=.2
    if case=='already_accepted':before['current_solo_gap_retry']['applied']=True
    assert run(parts) is before and not calls


@pytest.mark.parametrize('case',['core_stop','angle','time','front_mask','mask_extent','no_region','no_proof'])
def test_current_failures_still_stop(monkeypatch,case):
    parts=fixture(monkeypatch);_,_,_,core,region,proof,_,_=parts
    if case=='core_stop':core['speed_mps']=0.
    if case=='angle':core['steering_rad']=.3
    if case=='time':core['reference_s']=11.
    if case=='front_mask':region['masked_front_samples']=1
    if case=='mask_extent':region['masked_tail_extent_m']=.011
    if case=='no_region':region['queries']=0
    if case=='no_proof':proof['accepted']=False
    assert run(parts)['speed_mps']==0.
