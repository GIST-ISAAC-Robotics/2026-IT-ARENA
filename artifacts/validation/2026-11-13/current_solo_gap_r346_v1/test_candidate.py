import copy
from types import SimpleNamespace

import pytest
import candidate


def fixture(monkeypatch):
    normal=dict(selector_mode='SOLO',boundary_from_memory=False,
                partial_observation=dict(allowed=True,reason='fully_observed'))
    output=[.6,.2,dict(handoff_state='FOLLOW',normal=normal)]
    current=dict(speed_mps=0.,reference_s=10.,scan=dict(measurements={}),
                 previous_steering_rad=.2,dt_s=.02)
    before=dict(speed_mps=0.,steering_rad=.2,reference_s=10.,
        reason='route_current_large_motion_region_unobserved',current_route_proof=dict(
            region=dict(unobserved=123),original_path_source_s=9.9,source_timestamp_renewed=False))
    core=dict(speed_mps=.15,steering_rad=.2,reference_s=10.,reason='rebase_current_follow')
    bounded=dict(accepted=True,region=dict(unobserved=12,masked_front_samples=0),
                 unknown_filled=False,proof=dict(accepted=True))
    calls=[]
    monkeypatch.setattr(candidate.prior,'rebase',lambda *a,**k:before)
    def check_core(low,submitted,cur,*args):
        calls.append(('core',low,cur is current))
        return core
    monkeypatch.setattr(candidate.prior.base.candidate_policy,'rebase',check_core)
    monkeypatch.setattr(candidate,'MeasurementScan',lambda **kwargs:'raw')
    monkeypatch.setattr(candidate.prior.base.ps,'lateral_config',lambda cfg:'lateral')
    def check(*args):
        calls.append(('check',args))
        return bounded
    monkeypatch.setattr(candidate,'check',check)
    return output,current,before,core,bounded,calls


def run(output,current):
    return candidate.rebase(output,9.95,current,'scene','history',SimpleNamespace(),require_current_region=True)


def test_same_current_geometry_and_existing_bounded_check(monkeypatch):
    out,cur,before,core,bounded,calls=fixture(monkeypatch)
    originals=copy.deepcopy((out,cur,before))
    result=run(out,cur)
    assert result['speed_mps']==.15 and result['current_solo_gap_retry']['applied']
    assert result['current_route_proof']['region']['unobserved']==12
    assert result['current_route_proof']['original_path_source_s']==9.9
    assert not result['current_route_proof']['unknown_filled']
    assert calls[0][1][0]==.15 and calls[0][1][1:]==tuple(out[1:]) and calls[0][2]
    assert calls[1][1][2:7]==(0.,.15,.2,.2,.02)
    assert (out,cur,before)==originals


@pytest.mark.parametrize('kind',['positive','other_reason','guard','hold','memory','partial','notallowed',
                                 'fast','reverse','nan','zero','negative','already_low','noresult'])
def test_unrelated_cases_do_not_recheck(monkeypatch,kind):
    out,cur,before,core,bounded,calls=fixture(monkeypatch)
    if kind=='positive':before['speed_mps']=.6
    if kind=='other_reason':before['reason']='rebase_current_collision'
    if kind=='guard':out[2]['normal']['selector_mode']='GUARD'
    if kind=='hold':out[2]['handoff_state']='HOLD'
    if kind=='memory':out[2]['normal']['boundary_from_memory']=True
    if kind=='partial':out[2]['normal']['partial_observation']['reason']='unknown'
    if kind=='notallowed':out[2]['normal']['partial_observation']['allowed']=False
    if kind in ('fast','reverse','nan'):cur['speed_mps']={'fast':.16,'reverse':-.01,'nan':float('nan')}[kind]
    if kind in ('zero','negative','already_low'):out[0]={'zero':0.,'negative':-.1,'already_low':.15}[kind]
    if kind=='noresult':
        monkeypatch.setattr(candidate.prior,'rebase',lambda *a,**k:None)
        before=None
    assert run(out,cur) is before and not calls


@pytest.mark.parametrize('change',[dict(speed_mps=0.),dict(speed_mps=.2),dict(steering_rad=.21),dict(reference_s=10.1)])
def test_core_rejection_or_different_geometry_is_not_overridden(monkeypatch,change):
    out,cur,before,core,bounded,calls=fixture(monkeypatch)
    core.update(change)
    assert run(out,cur) is before and len(calls)==1


def test_bounded_rejection_remains_stop(monkeypatch):
    out,cur,before,core,bounded,calls=fixture(monkeypatch)
    bounded.update(accepted=False,blocked='front_gap_outside_scope')
    result=run(out,cur)
    assert result['speed_mps']==0 and not result['current_solo_gap_retry']['applied']
