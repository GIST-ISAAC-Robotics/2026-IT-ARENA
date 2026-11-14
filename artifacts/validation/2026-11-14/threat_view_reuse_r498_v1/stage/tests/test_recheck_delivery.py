"""두 단계의 원천/세대/마지막 발행 경계. 허가/보호를 대신하지 않는다."""
import copy
import math

import pytest

from arena_autonomy.recheck_delivery import RecheckDelivery
from arena_autonomy import guard_observation_policy
from test_decision_delivery import data,poll


class Worker:
    def __init__(self):
        self.inflight=self.pending=self.fault=self.result=None
        self.generation=self.sequence=0;self.expiry_generation=None
    def offer(self,data):self.pending=('plan',copy.deepcopy(data))
    def offer_recheck(self,data,result):self.pending=('recheck',copy.deepcopy(data))
    def pump(self):
        result,self.result=self.result,None
        if result is not None:self.inflight=None
        if self.inflight is None and self.pending is not None:
            self.inflight,self.pending=self.pending,None;self.sequence+=1
        return result
    def invalidate(self):
        self.generation+=1;self.pending=self.result=None;self.expiry_generation=None
    def mark_source_expiry(self):self.expiry_generation=self.generation
    def complete(self,speed=1.,reference=None):
        stage,snapshot=self.inflight
        output=[speed,.2,{'reason':'test'}] if stage=='plan' else dict(speed_mps=speed,steering_rad=.2,
            reference_s=snapshot['reference_s'] if reference is None else reference,reason='current',path_points=[[1.,0.]])
        self.result=dict(output=output,stage=stage,generation=self.generation,sequence=self.sequence,
                         began_wall_s=100.,completed_wall_s=100.01)


@pytest.fixture
def gate(monkeypatch):
    monkeypatch.setattr(guard_observation_policy,'supported',lambda output:output[0]>0)
    return RecheckDelivery(Worker(),retain_expired_reference=True)


def begin(gate):
    assert gate.offer(data())
    gate.worker.complete()
    assert poll(gate,1.17)==dict(kind='needs_recheck')
    current=data(1.18,1.1);current['dt_s']=.02
    gate.offer_recheck(current)
    return current


def test_current_result_is_required_and_last_application_only_changes_after_publish(gate):
    current=begin(gate);current['reference_s']=-999
    assert poll(gate,1.185) is None
    assert gate.last_application==-math.inf
    gate.worker.complete()
    result=poll(gate,1.19)
    assert result['kind']=='candidate' and result['reference_s']==1.1 and result['command_reference_s']==1.18
    assert result['recheck_input']['reference_s']==1.18
    assert result['output'][2]['current_rebase']['reference_s']==1.18
    assert 'path_points' not in result['output'][2]['current_rebase']
    assert gate.last_application==-math.inf
    gate.last_application=1.19  # existing node records only after /drive publish.
    assert poll(gate,1.195) is None and gate.active is None
    assert not gate.offer(data())


@pytest.mark.parametrize('now,changed,reason',[(1.31,{},'decision_source_expired'),
    (1.19,{'previous_steering_rad':.2},'decision_previous_request_changed'),
    (1.19,{'side':'right'},'decision_wall_side_changed'),
    (1.19,{'motion_epoch':4},'decision_epoch_changed'),
    (1.19,{'sensor_epoch':4},'decision_epoch_changed'),
    (1.17,{},'decision_clock_boundary')])
def test_returned_recheck_cannot_skip_original_and_current_bounds(gate,now,changed,reason):
    begin(gate);gate.worker.complete()
    result=poll(gate,now,**changed)
    assert result['kind']=='discarded' and result['reason']==reason
    assert gate.active is None and gate.current is None and gate.worker.generation==1
    assert gate.last_application==-math.inf
    assert (gate.worker.expiry_generation==1)==(reason=='decision_source_expired')


@pytest.mark.parametrize('part',['epoch','sensor','side','steering'])
def test_current_snapshot_has_own_context_checks(gate,part):
    gate.offer(data());gate.worker.complete();assert poll(gate,1.17)['kind']=='needs_recheck'
    current=data(1.18,1.1)
    if part=='epoch':current['epoch']=9
    elif part=='sensor':current['scan']['measurements']['source_epoch']=9
    elif part=='side':current['side']='right'
    else:current['previous_steering_rad']=.3
    gate.offer_recheck(current);gate.worker.complete()
    assert poll(gate,1.19)['kind']=='discarded'


def test_positive_interval_and_zero_does_not_wait(gate):
    begin(gate);gate.worker.complete();gate.last_application=1.16
    assert poll(gate,1.19) is None
    assert poll(gate,1.26)['kind']=='candidate'
    gate.last_application=1.26
    assert gate.offer(data(1.3,1.2));gate.worker.complete(speed=0.)
    assert poll(gate,1.31)['kind']=='candidate'  # unsupported zero needs no recheck.
    assert gate.last_application==1.26


def test_invalidated_completed_and_pending_current_are_not_reused(gate):
    begin(gate);gate.invalidate()
    gate.worker.inflight=None  # actual process discards old generation on completion.
    assert gate.current is None and gate.ready is None and poll(gate,1.2) is None
    assert gate.offer(data(1.3,1.2))
    gate.worker.complete(speed=0.);assert poll(gate,1.31)['kind']=='candidate'


def test_collect_without_valid_sensors_only_drains_notification(gate):
    gate.offer(data());gate.worker.complete();gate.collect()
    assert gate.ready is not None and gate.worker.result is None and gate.worker.inflight is None
    assert gate.counts['applied']==0 and gate.last_application==-math.inf
    assert poll(gate,1.31)['reason']=='decision_source_expired'


def test_changed_current_reference_is_not_restamped(gate):
    begin(gate);gate.worker.complete(reference=1.19)
    with pytest.raises(RuntimeError,match='changed source reference'):poll(gate,1.2)


def test_stop_zero_from_recheck_does_not_wait_positive_interval(gate):
    begin(gate);gate.worker.complete(speed=0.);gate.last_application=1.18
    result=poll(gate,1.19)
    assert result['kind']=='candidate' and result['output'][0]==0
    assert result['output'][2]['reason']=='current'


def test_current_snapshot_can_only_be_offered_once(gate):
    current=begin(gate)
    with pytest.raises(RuntimeError):gate.offer_recheck(current)
