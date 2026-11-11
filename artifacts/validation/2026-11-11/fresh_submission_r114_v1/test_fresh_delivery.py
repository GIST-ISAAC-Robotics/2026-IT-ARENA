import copy
import pytest
from fresh_delivery import FreshStartDelivery
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
