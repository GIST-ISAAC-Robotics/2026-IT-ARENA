import pytest

from validate_g1_traffic import EvaluationCaptureGate, drain_actor_tail


class Clock:
    def __init__(self): self.t=0.
    def __call__(self): return self.t


def test_drain_receives_primary_and_extra_without_growing_ego_capture():
    gate=EvaluationCaptureGate();ego=[(164.009,0,0,0)]
    receive=gate.wrap(ego.append);gate.close()
    streams={'primary':[(163.95,0,0,0)],'extra':[(163.95,0,0,0)]}
    clock=Clock()
    def spin(timeout):
        clock.t+=timeout
        receive((165.,0,0,0))
        for rows in streams.values():rows.append((rows[-1][0]+.01,0,0,0))
    result=drain_actor_tail(ego[-1][0],streams,spin,clock=clock)
    assert result['completed'] and result['spin_calls']==6
    assert ego==[(164.009,0,0,0)]
    assert all(t>=164.009 for t in result['after_sim_s'].values())


def test_one_stopped_stream_times_out_without_inventing_rows():
    clock=Clock();streams={'primary':[(1.,0,0,0)],'extra':[]}
    def spin(timeout):clock.t+=timeout
    result=drain_actor_tail(1.,streams,spin,wall_timeout_s=.03,clock=clock)
    assert not result['completed'] and result['after_sim_s']['extra'] is None
    assert streams=={'primary':[(1.,0,0,0)],'extra':[]}


def test_complete_streams_do_not_spin_or_modify():
    streams={'primary':[(2.,0,0,0)]}
    def spin(timeout):raise AssertionError('unnecessary spin')
    result=drain_actor_tail(2.,streams,spin)
    assert result['completed'] and result['spin_calls']==0


@pytest.mark.parametrize('target,timeout,streams',[(float('nan'),1.,{'x':[]}),
    (1.,0.,{'x':[]}),(1.,1.,{})])
def test_bad_targets_rejected(target,timeout,streams):
    with pytest.raises(ValueError):drain_actor_tail(target,streams,lambda _:None,wall_timeout_s=timeout)
