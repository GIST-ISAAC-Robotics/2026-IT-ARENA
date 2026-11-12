import copy
import multiprocessing as mp
import time
import pytest
from delivery import Delivery, DelayedLane, BothConnections, Worker
from test_recheck_process import inputs
from arena_autonomy.decision_delivery import source_identity


def test_both_completion_pipes_are_not_consumed():
    pairs=[mp.Pipe(),mp.Pipe()]
    connection=BothConnections(*(p[0] for p in pairs))
    try:
        assert not connection.poll(0)
        for a,b in pairs:
            b.send('result');assert connection.poll(.1);assert a.recv()=='result'
            assert not connection.poll(0)
    finally:
        for pair in pairs:
            for c in pair:c.close()


class FakeLane:
    inflight=None
    def offer(self,*args):self.inflight={'sequence':1}
    def collect(self):
        if self.inflight is None:return None
        self.inflight=None
        return {'output':7}


def test_delayed_completion_occupies_slot_until_sim_time_and_wakes():
    now=[10.];wake=[];lane=DelayedLane(FakeLane(),lambda:now[0],.147,wake.append)
    lane.offer({});assert lane.collect() is None and lane.inflight
    assert wake==[10.147]
    with pytest.raises(RuntimeError):lane.offer({})
    now[0]=10.147
    row=lane.collect();assert row['output']==7 and lane.inflight is None
    assert row['minimum_result_age_s']==.147 and lane.collect() is None


def wait_stage(worker,stage):
    until=time.monotonic()+10
    while time.monotonic()<until:
        for row in worker.collect():
            if row['stage']==stage:return row
        assert not worker.fault
        time.sleep(.001)
    raise AssertionError('missing completion')


@pytest.fixture
def ready():
    header,calls=inputs();now=[calls[-1]['current_rebase']['input']['reference_s']+.001]
    worker=Worker(header,clock=lambda:now[0],delays=(0.,0.),wake=lambda _:None)
    delivery=Delivery(worker,lambda:now[0])
    try:
        for i,call in enumerate(calls):
            assert worker.offer_plan(call['input'])
            plan=wait_stage(worker,'plan')
            if i<len(calls)-1:worker.discard_plan((plan['generation'],plan['sequence']))
        current=calls[-1]['current_rebase']['input']
        assert delivery.offer_recheck(current)
        checked=wait_stage(worker,'current')
        assert checked['output']['speed_mps']>0
        yield delivery,checked,calls,now
    finally:
        result=worker.close();assert not result['forced'] and result['exitcode']==0


def state(row,now):
    c=row['input']
    return dict(now_s=now,previous_steering_rad=c['previous_steering_rad'],side=c['side'],
                motion_epoch=c['epoch'],sensor_epoch=source_identity(c)[0])


def test_candidate_preserves_source_and_consumes_once_without_retention(ready):
    delivery,row,_,now=ready
    item=delivery.poll(**state(row,now[0]))
    assert item['kind']=='candidate'
    assert item['input']==row['plan']['input'] and item['recheck_input']==row['input']
    assert item['command_reference_s']==row['input']['reference_s']
    assert delivery.worker.retained is None and delivery.worker.ready is None
    assert delivery.last_application==-float('inf')  # 노드의 실제 발행 뒤에만 갱신한다.
    assert delivery.poll(**state(row,now[0])) is None
    with pytest.raises(RuntimeError):delivery.offer_recheck(row['input'])


@pytest.mark.parametrize('field',('motion_epoch','sensor_epoch','previous_steering_rad','side','now_s'))
def test_changed_current_contract_is_discarded(ready,field):
    delivery,row,_,now=ready;s=state(row,now[0])
    s[field]=('right' if s[field]=='left' else 'left') if field=='side' else s[field]+1
    item=delivery.poll(**s)
    assert item['kind']=='discarded' and delivery.worker.retained is None


def test_stop_before_poll_drops_old_result(ready):
    delivery,row,_,now=ready;generation=delivery.worker.generation
    delivery.invalidate()
    assert delivery.worker.generation==generation+1
    assert delivery.poll(**state(row,now[0])) is None


def test_positive_interval_waits_without_renewing_reference(ready):
    delivery,row,_,now=ready
    delivery.last_application=now[0]-.05
    assert delivery.poll(**state(row,now[0])) is None
    now[0]+=.05
    assert delivery.poll(**state(row,now[0]))['command_reference_s']==row['input']['reference_s']
