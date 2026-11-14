from types import SimpleNamespace
import pytest
import test_traffic_path_recovery as original
from test_pass_release_trial import output

@pytest.fixture
def fixture(monkeypatch):
    return original.fixture.__wrapped__(monkeypatch)

def same_wait(ctl,*,t=1.02,source=.9,epoch=0,speed=0.):
    scene=SimpleNamespace(t_ref_s=t,source_first_s=source,source_epoch=epoch)
    baseline=(0.,.2,dict(reason='follow_same_scan_missing_wait',path_points=None,preferred_wall='right'))
    return ctl.select(scene,baseline,output(0.,.2,'parallel_hold_wait'),speed,.2,.02)

def test_same_scan_wait_preserves_history_but_never_motion(fixture):
    ctl,state,call=fixture
    call(1.,'follow_boundary_unconfirmed')
    out=same_wait(ctl)
    assert out[0]==0. and ctl.suspended_passage=={'left'} and not out[2]['restored_passage_history']
    assert call(1.04,source=.9)[0]==0.
    out=call(1.1)
    assert out[0]==.35 and out[2]['restored_passage_history']==['left']
    assert state['checks']==1 and not out[2]['restored_history_is_current_free_space']

@pytest.mark.parametrize('args',[dict(epoch=1),dict(source=.8),dict(source=.5),dict(speed=float('nan'))])
def test_discontinuous_same_scan_wait_clears_history(fixture,args):
    ctl,state,call=fixture;call(1.,'follow_boundary_unconfirmed')
    assert same_wait(ctl,**args)[0]==0. and not ctl.suspended_passage

def test_unfinished_passage_is_not_created(fixture):
    ctl,state,call=fixture;ctl.sides['left'].update(passed=False,clear=1)
    call(1.,'follow_boundary_unconfirmed');assert same_wait(ctl)[0]==0.
    assert not ctl.suspended_passage and call(1.1)[0]==0.

@pytest.mark.parametrize('cause',['current_threat','unobserved_motion_region'])
def test_wait_history_does_not_override_new_scan(cause,fixture):
    ctl,state,call=fixture;call(1.,'follow_boundary_unconfirmed');same_wait(ctl)
    if cause=='current_threat':state['present']=True
    else:state['unknown']=1
    out=call(1.1)
    assert out[0]==0.
    if cause=='current_threat':assert ctl.sides['left']['present'] and not ctl.sides['left']['passed']
    else:assert state['checks']==1
