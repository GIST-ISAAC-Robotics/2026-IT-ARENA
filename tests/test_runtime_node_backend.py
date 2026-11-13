from types import SimpleNamespace as NS
import pytest
from arena_autonomy.runtime_node_backend import POLICY, validate_selection, control


def selection():
    return dict(execution='process_candidate', submission='fresh_native_v1', recheck_in_worker=True,
                retain_reference=True, numeric_execution='native_v1', hardware_shadow=False)


def test_default_off_preserves_existing_execution():
    assert not validate_selection('off', **selection())
    assert validate_selection(POLICY, **selection())


@pytest.mark.parametrize('key,value', [('execution','inline'),('submission','original_v1'),
    ('recheck_in_worker',False),('retain_reference',False),('numeric_execution','python'),('hardware_shadow',True)])
def test_registered_selection_before_spawn(key,value):
    options=selection();options[key]=value
    with pytest.raises(ValueError): validate_selection(POLICY, **options)


class Delivery:
    def __init__(self):
        self.worker=NS(fault=None,plan=NS(inflight=None),saved_plan=None)
        self.pending_publication=None;self.acks=[];self.offers=[];self.invalidated=0
    def acknowledge_publication(self,**data):
        self.acks.append(data);self.pending_publication=None
        return dict(retained=data['published'],reason='fake_publication_boundary')
    def invalidate(self):self.invalidated+=1
    def offer(self,snapshot):self.offers.append(snapshot)


def node():
    return NS(decision_delivery=Delivery(),decision_notice=None,now_s=lambda:1.,last_request_time_s=.9,
        last_speed=.2,last_steering=.1,command_was_published=False,scan=object(),side='right',rear_blind_half_angle=0.,
        motion=NS(history=NS(wheels=NS(values=[.3])),last_meta={'reference_stamp_s':.97},counts={'resets':2},
                  verified=False,timing_policy='host_estimated_v1'),follow_trace=NS(emit=lambda *args:None))


@pytest.mark.parametrize('published', [False,True])
def test_ack_follows_real_parent_publication(monkeypatch,published):
    n=node();n.decision_delivery.pending_publication={}
    def parent():
        n.command_was_published=published;n.decision_command_stamp_s=.96
        return 'result'
    assert control(n,parent)=='result'
    assert n.decision_delivery.acks==[dict(published=published,reference_s=.96,speed_mps=.2,steering_rad=.1,now_s=1.)]
    assert not n.decision_delivery.offers


def test_snapshot_original_stamp_and_observed_speed_only(monkeypatch):
    import arena_autonomy.follow_trace as trace
    n=node();calls=[]
    def snapshot(*args,**kwargs):calls.append((args,kwargs));return 'owned_snapshot'
    monkeypatch.setattr(trace,'snapshot_call',snapshot)
    def parent():n.async_inputs_ready=True
    control(n,parent)
    args,kwargs=calls[0]
    assert args[2:7]==(.97,2,.3,'right',.1)
    assert args[8] is False and kwargs==dict(timing_policy='host_estimated_v1')
    assert n.decision_delivery.offers==['owned_snapshot']
    n.motion.history.wheels.values=[];calls.clear();n.decision_delivery.offers.clear()
    control(n,parent)
    assert not calls and not n.decision_delivery.offers


def test_parent_error_invalidates_candidate():
    n=node()
    def parent():raise RuntimeError('publication failed')
    with pytest.raises(RuntimeError):control(n,parent)
    assert n.decision_delivery.invalidated==1


def test_worker_finally_closes_evaluator_then_pipe(monkeypatch):
    from arena_autonomy import runtime_dual_process as process
    events=[]
    class Pipe:
        def send(self,value):events.append(('send',value))
        def recv(self):return None
        def close(self):events.append('pipe_closed')
    class Evaluator:
        def close(self):events.append('evaluator_closed')
    monkeypatch.setattr(process.signal,'signal',lambda *args:None)
    process._worker('plan',None,None,Pipe(),{},1,lambda *args:Evaluator())
    assert events==[('send',{'ready':True}),'evaluator_closed','pipe_closed']


def test_worker_close_failure_still_closes_pipe(monkeypatch):
    from arena_autonomy import runtime_dual_process as process
    events=[]
    class Pipe:
        def send(self,value):pass
        def recv(self):return None
        def close(self):events.append('pipe_closed')
    class Evaluator:
        def close(self):raise RuntimeError('restore failed')
    monkeypatch.setattr(process.signal,'signal',lambda *args:None)
    with pytest.raises(RuntimeError,match='restore failed'):
        process._worker('plan',None,None,Pipe(),{},1,lambda *args:Evaluator())
    assert events==['pipe_closed']
