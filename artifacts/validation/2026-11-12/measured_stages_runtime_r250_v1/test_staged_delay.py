from copy import deepcopy
from types import SimpleNamespace
import pytest
from staged_delay import StagedDelay


class Worker:
    generation=0
    def __init__(self):self.answer=None;self.data=None
    def offer(self,data):self.data=deepcopy(data)
    def offer_recheck(self,data,plan):self.data=deepcopy(data)
    def pump(self):result=self.answer;self.answer=None;return result
    def invalidate(self):self.generation+=1;self.answer=None
    def close(self):return dict(forced=False,exitcode=0)


@pytest.mark.parametrize('stage,delay',[('plan',.15),('recheck',.035)])
def test_result_waits_for_actual_completion_and_registered_stage_age(stage,delay):
    w=Worker();now=SimpleNamespace(t=1.);proxy=StagedDelay(w,lambda:now.t,.15,.035)
    data={'reference_s':1.,'scan':{'stamp':.9}}
    if stage=='plan':proxy.offer(data)
    else:proxy.offer_recheck(data,{'sequence':1})
    now.t=1.+delay+.001
    assert proxy.pump() is None
    original=dict(stage=stage,generation=0,reference_s=1.,output=[.4,.1,{}],began_wall_s=20.,completed_wall_s=20.03)
    w.answer=deepcopy(original);now.t=1.+delay-.001
    assert proxy.pump() is None
    now.t=1.+delay
    result=proxy.pump()
    assert all(result[k]==v for k,v in original.items())
    assert result['injected_delay_s']==delay and w.data==data
    assert proxy.pump() is None


@pytest.mark.parametrize('held',[False,True])
def test_stop_cancels_held_or_pending_completion(held):
    w=Worker();proxy=StagedDelay(w,lambda:1.,.15,.035);proxy.offer({'reference_s':1.})
    if held:w.answer=dict(stage='plan',generation=0);assert proxy.pump() is None
    proxy.invalidate()
    assert proxy.pump() is None and proxy.current is proxy.held is None
    assert w.generation==1 and proxy.close()==dict(forced=False,exitcode=0)


def test_serial_stage_cannot_be_overwritten():
    w=Worker();proxy=StagedDelay(w,lambda:1.,.15,.035);proxy.offer({'reference_s':1.})
    with pytest.raises(RuntimeError):proxy.offer_recheck({'reference_s':1.01},{})
