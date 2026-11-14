"""기존 새 회전 제출의 시간/세대/명령 간격 계약."""
import copy
import importlib.util
import json
from pathlib import Path
import pytest
from arena_autonomy.fresh_submission import FreshStartDelivery
from test_decision_delivery import Worker,data,poll


def test_old_complete_scan_waits_without_reset_or_timestamp_change():
    worker=Worker();gate=FreshStartDelivery(worker)
    old=data(1.18,1.);original=copy.deepcopy(old)
    assert not gate.offer(old)
    assert old==original and worker.generation==0 and gate.counts['submitted']==0
    assert gate.offer(data(1.22,1.1))


@pytest.mark.parametrize('reference',[1.10,1.12,1.124])
def test_completion_window_accepts_current_scan(reference):
    gate=FreshStartDelivery(Worker())
    assert gate.offer(data(reference,1.))


@pytest.mark.parametrize('reference',[1.09,1.125,1.18])
def test_future_or_old_scan_does_not_submit(reference):
    gate=FreshStartDelivery(Worker())
    assert not gate.offer(data(reference,1.))


def test_expiry_and_output_budget_remain_unchanged():
    worker=Worker();gate=FreshStartDelivery(worker)
    assert gate.offer(data(1.12,1.))
    worker.complete();assert poll(gate,1.28)['kind']=='candidate'
    assert gate.offer(data(1.32,1.2));worker.complete()
    assert poll(gate,1.35) is None
    assert poll(gate,1.38)['kind']=='candidate'
    assert gate.offer(data(1.42,1.3));worker.complete()
    assert poll(gate,1.61)['reason']=='decision_source_expired'


def test_first_source_alignment_and_busy_are_preserved():
    worker=Worker();gate=FreshStartDelivery(worker,align_first_source=True)
    assert not gate.offer(data(1.12,1.))
    assert gate.offer(data(1.22,1.1))
    assert not gate.offer(data(1.32,1.2)) and worker.pending is None


def test_missing_timestamp_contract_rejected():
    gate=FreshStartDelivery(Worker());raw=data()
    raw['scan']['measurements']['acquired_at_s']=[float('nan')]
    with pytest.raises(ValueError,match='acquisition'):gate.offer(raw)


def test_recorded_r182_submissions_match_preserved_prefix():
    root = Path(__file__).resolve().parents[1]
    old_path = root/'artifacts/validation/2026-11-11/fresh_submission_r114_v1/fresh_delivery.py'
    spec = importlib.util.spec_from_file_location('r184_original_fresh',old_path)
    old = importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    gates = [old.FreshStartDelivery(Worker(),align_first_source=True),
             FreshStartDelivery(Worker(),align_first_source=True)]
    trace = root/'artifacts/validation/2026-11-12/guard_approach_runtime_r182_v1/two_actor_run1/run1/follow_decisions.jsonl'
    calls = 0
    for event in map(json.loads,trace.open()):
        if event['kind'] == 'reset':
            for gate in gates:gate.invalidate();gate.worker.inflight=None
        if event['kind'] != 'call':continue
        value = event['data']['input']; before = copy.deepcopy(value)
        assert gates[0].offer(value) == gates[1].offer(value)
        assert value == before
        replies = []
        for gate in gates:
            if gate.worker.inflight is not None:gate.worker.complete()
            replies.append(gate.poll(value['reference_s']+.08,
                previous_steering_rad=value['previous_steering_rad'], side=value['side'],
                motion_epoch=value['epoch'], sensor_epoch=value['scan']['measurements']['source_epoch']))
        assert replies[0] == replies[1]
        assert gates[0].counts == gates[1].counts
        assert gates[0].boundary_source == gates[1].boundary_source
        assert gates[0].last_source == gates[1].last_source
        calls += 1
    assert calls == 439
