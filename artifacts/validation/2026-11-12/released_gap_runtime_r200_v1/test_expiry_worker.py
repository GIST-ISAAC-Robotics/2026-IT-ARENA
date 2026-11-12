"""계산 만료와 명시 STOP/세대 변경을 구별하는 시험 연결의 경계."""
from types import SimpleNamespace as NS
import json
from pathlib import Path
import pytest
from expiry_worker import ReasonedWorker, ExpiryFreshDelivery
from test_decision_delivery import Worker,data,poll


def worker():
    inner=Worker();sent=[]
    inner.connection=NS(send=sent.append)
    outer=ReasonedWorker(inner)
    return outer,inner,sent


def test_source_expiry_is_only_next_generation_first_request():
    outer,inner,sent=worker();gate=ExpiryFreshDelivery(outer)
    assert gate.offer(data(1.12,1.));inner.complete()
    assert poll(gate,1.31)['reason']=='decision_source_expired'
    assert outer.expiry_generation==inner.generation==1
    inner.connection.send(dict(generation=1,sequence=2))
    inner.connection.send(dict(generation=1,sequence=3))
    assert sent==[dict(generation=1,sequence=2,reset_reason='decision_source_expired'),
                  dict(generation=1,sequence=3)]


def test_explicit_stop_after_expiry_clears_reference_request():
    outer,inner,sent=worker();gate=ExpiryFreshDelivery(outer)
    assert gate.offer(data(1.12,1.));inner.complete();poll(gate,1.31)
    gate.invalidate()
    inner.connection.send(dict(generation=inner.generation,sequence=2))
    assert sent==[dict(generation=2,sequence=2)]


@pytest.mark.parametrize('changes',[dict(motion_epoch=8),dict(sensor_epoch=8),dict(side='right'),
                                    dict(previous_steering_rad=.4)])
def test_other_invalidations_do_not_retain_reference(changes):
    outer,inner,sent=worker();gate=ExpiryFreshDelivery(outer)
    assert gate.offer(data(1.12,1.));inner.complete()
    assert poll(gate,1.18,**changes)['kind']=='discarded'
    assert outer.expiry_generation is None
    inner.connection.send(dict(generation=inner.generation,sequence=2))
    assert sent==[dict(generation=1,sequence=2)]


def test_unsolicited_old_generation_does_not_receive_hint():
    outer,inner,sent=worker();inner.generation=7
    outer.mark_source_expiry(dict(kind='discarded',reason='decision_source_expired'))
    inner.connection.send(dict(generation=6,sequence=1))
    assert sent==[dict(generation=6,sequence=1)]
    assert outer.expiry_generation==7


def test_actual_spawn_carries_only_expiry_reference_and_stop_resets_it(monkeypatch):
    import arena_autonomy.decision_process as process
    from expiry_worker import worker_entry
    from test_decision_process import receive
    trace=Path(__file__).resolve().parent.parent/'guard_approach_runtime_r182_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with trace.open() as stream:
        header=json.loads(next(stream))['data'];samples=[]
        for event in map(json.loads,stream):
            if event['kind']=='call':samples.append(event['data']['input'])
            if len(samples)==8:break
    header['follow_config'].update(guard_observation_policy='selected_r181', released_body_gap_trial=True)
    monkeypatch.setattr(process,'_worker',worker_entry)
    runner=ReasonedWorker(process.DecisionProcess(header))
    try:
        for sample in samples[:6]:
            runner.offer(sample);result=receive(runner)
            assert result['expiry_reference_retained'] is False
        runner.invalidate()
        runner.mark_source_expiry(dict(kind='discarded',reason='decision_source_expired'))
        runner.offer(samples[6]);result=receive(runner)
        assert result['expiry_reference_retained'] is True
        runner.invalidate()
        runner.offer(samples[7]);result=receive(runner)
        assert result['expiry_reference_retained'] is False
    finally:
        cleanup=runner.close()
    assert cleanup['forced'] is False and cleanup['exitcode']==0
