"""만료 원인 전달·STOP 우선·실제 spawn의 참조/명령 분리."""
from dataclasses import replace
import json
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import pytest
from arena_autonomy.decision_process import DecisionProcess
from arena_autonomy.expiry_reference import reset_after_source_expiry
from arena_autonomy.fresh_submission import FreshStartDelivery
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from test_decision_delivery import Worker,data,poll
from test_decision_process import receive

ROOT=next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())


class ReasonWorker(Worker):
    invalidate=DecisionProcess.invalidate
    mark_source_expiry=DecisionProcess.mark_source_expiry
    def __init__(self):
        super().__init__()
        self.expiry_generation=None
        self.expiry_reference_trial=True
        self.stats=dict(invalidated=0)


def test_source_expiry_marks_only_current_generation_and_stop_removes_it():
    worker=ReasonWorker();gate=FreshStartDelivery(worker,retain_expired_reference=True)
    gate.offer(data(1.12,1.));worker.complete()
    assert poll(gate,1.31)['reason']=='decision_source_expired'
    assert worker.expiry_generation==worker.generation==1
    gate.invalidate()
    assert worker.expiry_generation is None and worker.generation==2


@pytest.mark.parametrize('now,changes',[(1.31,dict(motion_epoch=8)),(1.31,dict(sensor_epoch=8)),
    (1.31,dict(side='right')),(1.31,dict(previous_steering_rad=.4)),(.9,{})])
def test_other_invalidations_take_precedence_over_expired_time(now,changes):
    worker=ReasonWorker();gate=FreshStartDelivery(worker,retain_expired_reference=True)
    gate.offer(data(1.12,1.));worker.complete()
    result=poll(gate,now,**changes)
    assert result['kind']=='discarded' and result['reason']!='decision_source_expired'
    assert worker.expiry_generation is None


def test_default_does_not_ask_for_reference_retention():
    worker=ReasonWorker();gate=FreshStartDelivery(worker)
    gate.offer(data(1.12,1.));worker.complete();poll(gate,1.31)
    assert worker.expiry_generation is None
    with pytest.raises(ValueError,match='reason-aware'):
        FreshStartDelivery(Worker(),retain_expired_reference=True)
    with pytest.raises(ValueError,match='explicit'):
        FreshStartDelivery(worker,retain_expired_reference='true')


@pytest.mark.parametrize('value',[True,'true',1])
def test_config_requires_explicit_bool_and_junction(value):
    with pytest.raises(ValueError,match='expiry route reference'):
        FollowConfig(expiry_route_reference_trial=value)


def test_reset_keeps_reference_only_and_stop_erases_it():
    normal=LocalFollow(FollowConfig(junction_continuity=True,expiry_route_reference_trial=True))
    normal.path=np.array([[0.,0.],[1.,0.]])
    original=normal.path
    normal.path_ref=4.;normal.path_source=3.9;normal.path_wall='right'
    normal.epoch=2;normal.side='left';normal.last_ref=4.;normal.last_source=3.9
    normal.held=False
    owner=NS(normal=normal,reset=normal.reset)
    assert reset_after_source_expiry(owner)
    assert normal.path is None and normal.held and normal.path_wall is None
    assert normal.recent_path[3]=='right' and normal.recent_path[0] is not original
    assert (normal.epoch,normal.side,normal.last_ref,normal.last_source)==(2,'left',4.,3.9)
    owner.reset()
    assert normal.recent_path is None and normal.epoch is None


@pytest.mark.parametrize('enabled',[False,True])
def test_actual_spawn_retains_only_opted_in_expiry_and_first_request_hint(enabled):
    trace=ROOT/'artifacts/validation/2026-11-12/guard_approach_runtime_r182_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with trace.open() as stream:
        header=json.loads(next(stream))['data'];samples=[]
        for event in map(json.loads,stream):
            if event['kind']=='call':samples.append(event['data']['input'])
            if len(samples)==10:break
    header['follow_config']['expiry_route_reference_trial']=enabled
    runner=DecisionProcess(header)
    try:
        for sample in samples[:6]:
            runner.offer(sample);result=receive(runner)
            assert result.get('expiry_reference_retained',False) is False
        runner.invalidate();runner.mark_source_expiry()
        runner.offer(samples[6]);runner.pump()
        assert (runner.inflight.get('reset_reason')=='decision_source_expired') is enabled
        result=receive(runner)
        assert result.get('expiry_reference_retained',False) is enabled
        runner.offer(samples[7]);runner.pump()
        assert 'reset_reason' not in runner.inflight
        receive(runner)
        runner.invalidate();runner.mark_source_expiry();runner.invalidate()  # STOP wins.
        runner.offer(samples[8]);result=receive(runner)
        assert result.get('expiry_reference_retained',False) is False
    finally:
        cleanup=runner.close()
    assert cleanup==dict(forced=False,exitcode=0)
