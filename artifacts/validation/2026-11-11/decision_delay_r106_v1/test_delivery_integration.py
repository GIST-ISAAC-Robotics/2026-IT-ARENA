"""실제 배송 계약에 지연을 결합해 큐·출처·만료·STOP을 확인한다."""
import copy
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'src/arena_autonomy'))
from arena_autonomy.decision_delivery import DecisionDelivery
from delay_worker import DelayedWorker


class Worker:
    def __init__(self):
        self.inflight=self.pending=self.fault=self.result=None
        self.generation=0
    def offer(self,data):self.pending=copy.deepcopy(data)
    def pump(self):
        result,self.result=self.result,None
        if result is not None:self.inflight=None
        if self.inflight is None and self.pending is not None:
            self.inflight,self.pending=self.pending,None
        return result
    def invalidate(self):
        self.generation+=1;self.pending=None
    def complete(self):self.result=dict(output=[.5,.0,{}],generation=self.generation)


def sample(reference=1.,first=.9):
    return dict(scan=dict(representation='native_measurements_v1',
        measurements=dict(source_epoch=0,acquired_at_s=[first,first+.099])),
        reference_s=reference,previous_steering_rad=0.,epoch=0,dt_s=.1,side='left')


def poll(gate,now):
    return gate.poll(now,previous_steering_rad=0.,side='left',motion_epoch=0,sensor_epoch=0)


def test_measured_floor_keeps_one_active_source_and_original_reference():
    now=[1.];worker=Worker();delayed=DelayedWorker(worker,lambda:now[0],[(0.,.12)])
    gate=DecisionDelivery(delayed);assert gate.offer(sample())
    worker.complete();now[0]=1.04
    assert poll(gate,now[0]) is None
    assert not gate.offer(sample(1.1,1.0))
    now[0]=1.12;item=poll(gate,now[0])
    assert item['kind']=='candidate' and item['reference_s']==1.
    assert item['result']['injected_delay_s']==.12
    assert not gate.offer(sample())


def test_slow_delivery_is_discarded_and_stop_cancels_buffered_result():
    now=[1.];worker=Worker();delayed=DelayedWorker(worker,lambda:now[0],[(0.,.24)])
    gate=DecisionDelivery(delayed);gate.offer(sample());worker.complete()
    now[0]=1.1;assert poll(gate,now[0]) is None
    now[0]=1.24;item=poll(gate,now[0])
    assert item['kind']=='discarded' and item['reason']=='decision_source_expired'
    assert gate.counts['applied']==0 and worker.generation==1
    gate.offer(sample(1.3,1.2));worker.complete();now[0]=1.4
    assert poll(gate,now[0]) is None
    gate.invalidate();now[0]=1.6
    assert poll(gate,now[0]) is None and gate.active is None
