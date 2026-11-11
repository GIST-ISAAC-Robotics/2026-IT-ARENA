"""차량을 움직이지 않는 지연 장치의 시간/STOP/세대 검사."""
import pytest
from delay_worker import DelayedWorker


class Worker:
    def __init__(self):self.generation=0;self.result=None;self.offered=[]
    def offer(self,data):self.offered.append(data)
    def pump(self):value=self.result;self.result=None;return value
    def invalidate(self):self.generation+=1;self.result=None
    def close(self):return 'closed'


def fixture():
    worker=Worker();now=[1.]
    gate=DelayedWorker(worker,lambda:now[0],[(.3,.18),(.4,.07)])
    return worker,now,gate


def test_actual_completion_and_sim_delay_both_required():
    worker,now,gate=fixture();gate.offer({'reference_s':1.})
    worker.result={'output':[1.,.2,{}],'completed_wall_s':21.}
    now[0]=1.16;assert gate.pump() is None
    now[0]=1.18;result=gate.pump()
    assert result['completed_wall_s']==21. and result['output']==[1.,.2,{}]
    assert result['injected_delay_s']==.18 and gate.pump() is None
    gate.offer({'reference_s':1.2});now[0]=1.4
    assert gate.pump() is None
    worker.result={'output':[0.,0.,{}]}
    assert gate.pump()['injected_delay_s']==.07


@pytest.mark.parametrize('held',[True,False])
def test_stop_cancels_held_or_running_result(held):
    worker,now,gate=fixture();gate.offer({'reference_s':1.})
    if held:
        worker.result={'output':[1.,.2,{}]};assert gate.pump() is None
    gate.invalidate();now[0]=1.5
    assert gate.pump() is None and gate.canceled==1 and worker.generation==1
    gate.offer({'reference_s':1.5});worker.result={'output':[0.,0.,{}]};now[0]=1.58
    assert gate.pump()['output'][0]==0.
    assert gate.close()=='closed'


def test_no_queue_or_second_early_result_and_old_generation_not_released():
    worker,now,gate=fixture();gate.offer({'reference_s':1.})
    with pytest.raises(RuntimeError):gate.offer({'reference_s':1.1})
    worker.result={'output':[1.,0.,{}]};gate.pump()
    worker.generation+=1;now[0]=1.4
    assert gate.pump() is None


@pytest.mark.parametrize('timeline',[[],[(0.,-1.)],[(0.,.1),(0.,.2)],[(0.,float('nan'))]])
def test_invalid_delay_profile_rejected(timeline):
    with pytest.raises(ValueError):DelayedWorker(Worker(),lambda:1.,timeline)
