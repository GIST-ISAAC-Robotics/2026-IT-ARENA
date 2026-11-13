"""같은 명령의 timestamp를 반복 갱신하지 않는 회수 계약. ROS 발행은 별도."""
import copy
import pytest
from arena_autonomy.decision_delivery import DecisionDelivery


class Worker:
    def __init__(self):
        self.inflight = self.pending = self.fault = self.result = None
        self.generation = 0
    def offer(self, data):
        self.pending = copy.deepcopy(data)
    def pump(self):
        result, self.result = self.result, None
        if result is not None:self.inflight = None
        if self.inflight is None and self.pending is not None:
            self.inflight, self.pending = self.pending, None
        return result
    def invalidate(self):
        self.generation += 1
        self.pending = self.result = None
    def complete(self, speed=1.):
        self.result = dict(output=[speed, .2, {'reason':'test'}])


def data(reference=1.1, first=1.):
    return dict(scan=dict(representation='native_measurements_v1',
        measurements=dict(source_epoch=3,acquired_at_s=[first,first+.099])),
        reference_s=reference,previous_steering_rad=.1,epoch=2,dt_s=.1,side='left')


def poll(gate, now, **changes):
    return gate.poll(now, **(dict(previous_steering_rad=.1,side='left',motion_epoch=2,sensor_epoch=3)|changes))


def test_one_source_once_and_busy_does_not_queue_old_steering():
    worker=Worker();gate=DecisionDelivery(worker)
    original=data();assert gate.offer(original)
    original['previous_steering_rad']=9.
    assert not gate.offer(data(1.2,1.1)) and worker.pending is None
    worker.complete();result=poll(gate,1.18)
    assert result['kind']=='candidate' and result['reference_s']==1.1
    assert result['input']['previous_steering_rad']==.1
    assert poll(gate,1.19) is None
    assert not gate.offer(data())
    assert gate.offer(data(1.2,1.1))


@pytest.mark.parametrize('now,change,reason',[
    (1.31,{},'decision_source_expired'),
    (1.15,{'previous_steering_rad':.2},'decision_previous_request_changed'),
    (1.15,{'side':'right'},'decision_wall_side_changed'),
    (1.15,{'motion_epoch':4},'decision_epoch_changed'),
    (1.15,{'sensor_epoch':4},'decision_epoch_changed'),
    (1.09,{},'decision_clock_boundary')])
def test_completed_result_not_permission_after_context_changes(now,change,reason):
    worker=Worker();gate=DecisionDelivery(worker);gate.offer(data());worker.complete()
    result=poll(gate,now,**change)
    assert result['kind']=='discarded' and result['reason']==reason
    assert worker.generation==1 and gate.active is None


def test_fast_second_positive_waits_for_interval_but_zero_does_not():
    worker=Worker();gate=DecisionDelivery(worker);gate.offer(data());worker.complete()
    assert poll(gate,1.19)['kind']=='candidate'
    gate.offer(data(1.2,1.1));worker.complete()
    assert poll(gate,1.23) is None
    assert poll(gate,1.29)['kind']=='candidate'
    gate.offer(data(1.3,1.2));worker.complete(0.)
    assert poll(gate,1.31)['kind']=='candidate'


def test_invalidated_work_cannot_be_applied():
    worker=Worker();gate=DecisionDelivery(worker);gate.offer(data())
    gate.invalidate()
    assert not gate.offer(data(1.2,1.1))
    worker.inflight=None
    assert gate.offer(data(1.3,1.2))
    worker.complete();assert poll(gate,1.35)['kind']=='candidate'


def test_invalidation_does_not_grant_a_second_acceleration_step():
    worker=Worker();gate=DecisionDelivery(worker);gate.offer(data());worker.complete()
    assert poll(gate,1.19)['kind']=='candidate'
    gate.invalidate();gate.offer(data(1.2,1.1));worker.complete()
    assert poll(gate,1.23) is None
    assert poll(gate,1.29)['kind']=='candidate'


def test_command_reference_expires_independently_of_first_ray():
    worker=Worker();gate=DecisionDelivery(worker,command_timeout_s=.12)
    gate.offer(data());worker.complete()
    assert poll(gate,1.23)['reason']=='decision_command_expired'
