from types import SimpleNamespace
from copy import deepcopy
import pytest
import candidate as c


def run(monkeypatch,*,success=.25,speed=0.,change=None):
    info=dict(target_speed_mps=.35,region=dict(unobserved=7,masked_front_samples=0,masked_tail_extent_m=.011),
        same_path_gap=dict(policy='guard_low_speed_body_nonreturn_r136',accepted=True))
    if change:change(info)
    original=(0.,.37,dict(selector_reason='guard_curve_rejected_large_motion_masked_toward_left',selector_guard_candidate=info))
    monkeypatch.setattr(c,'ORIGINAL',lambda *args:deepcopy(original))
    calls=[]
    def check(*args,target_cap):
        calls.append((args[4],args[5],args[6],target_cap))
        okay=target_cap<=success
        return (target_cap if okay else 0.,.37,dict(selector_guard_candidate=dict(accepted=okay,
            region=dict(unobserved=7,masked_tail_extent_m=.009 if okay else .011),
            same_path_gap=dict(accepted=okay)),selector_reason='checked_again'))
    owner=SimpleNamespace(follow_config=SimpleNamespace(traffic_recovery_trial=True),
        config=SimpleNamespace(guard_speed_cap_mps=.35),_guard_curve_at_cap=check)
    out=c.guard_recheck(owner,None,None,(.7,.37,dict(partial_observation=dict(allowed=True,reason='fully_observed'))),
        None,speed,.37,.1,.12,{},None)
    return out,calls,info


@pytest.mark.parametrize('success,expected',[(.25,.25),(.15,.15),(0.,0.)])
def test_full_check_is_repeated_without_changing_observation_or_measured_motion(monkeypatch,success,expected):
    out,calls,original=run(monkeypatch,success=success)
    assert out[0]==expected
    assert calls==([(0.,.37,.1,.25)] if success==.25 else [(0.,.37,.1,.25),(0.,.37,.1,.15)])
    assert original['region']['unobserved']==7
    assert not out[2]['selector_guard_speed_recheck']['unknown_filled']


@pytest.mark.parametrize('speed',[.35,.8])
def test_current_speed_is_not_cut_to_retry(monkeypatch,speed):
    out,calls,_=run(monkeypatch,speed=speed)
    assert out[0]==0. and not calls


@pytest.mark.parametrize('change',[
    lambda x:x['same_path_gap'].update(accepted=False),
    lambda x:x['same_path_gap'].update(policy='different'),
    lambda x:x['region'].update(masked_front_samples=1),
    lambda x:x['region'].update(unobserved=0),
    lambda x:x.update(target_speed_mps=.15)])
def test_unapproved_or_front_masked_region_is_not_retried(monkeypatch,change):
    out,calls,_=run(monkeypatch,change=change)
    assert out[0]==0. and not calls


def test_explicit_spawn_worker_starts_computes_and_exits(monkeypatch):
    import json
    from pathlib import Path
    import time
    import arena_autonomy.decision_process as dp
    trace=Path(__file__).resolve().parent.parent/'parallel_rebase_r133_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with trace.open() as stream:
        header=json.loads(next(stream))['data']
        first=next(json.loads(line) for line in stream if json.loads(line)['kind']=='call')
    monkeypatch.setattr(dp,'_worker',c.worker_entry)
    process=dp.DecisionProcess(header)
    try:
        process.offer(first['data']['input'])
        deadline=time.monotonic()+5.
        result=None
        while result is None and time.monotonic()<deadline:
            result=process.pump();time.sleep(.005)
        assert result is not None and process.fault is None
    finally:
        stopped=process.close()
    assert stopped==dict(forced=False,exitcode=0)
