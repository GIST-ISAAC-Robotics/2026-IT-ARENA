from types import SimpleNamespace
import numpy as np
import pytest
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
from controller import RelativeYieldFollow
from test_observer import row


def controller(enabled=True):
    execution=HandoffExecutionConfig(bypass=BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold'),
        preparation_policy='observed_entry_space_v1',persistence_window_policy='trim_before_compare_v1',
        follow_entry_space_enabled=True,reacquire_policy='known_hold_v1',entry_policy='corridor_local_v1',
        motion_pause_policy='bounded_motion_pause_v1')
    c=RelativeYieldFollow(execution=execution,allow_simulation_trial=True,yield_enabled=enabled)
    c.mode='BYPASS';c.bypass.guide.path=np.array([[0.,0.],[1.,0.]])
    for i in range(9):c.yield_observer.update(row(i))
    return c


@pytest.mark.parametrize('enabled,source,epoch,speed,phase,cap',[
    (True,1.9,1,.35,'entry',.2), (False,1.9,1,.35,'entry',.35),
    (True,2.3,1,.35,'entry',.35), (True,1.9,2,.35,'entry',.35),
    (True,1.7,1,.35,'entry',.35), (True,1.9,1,-.01,'entry',.35),
    (True,1.9,1,.35,'rejoin',.35), (True,1.9,1,1.1,'entry',.35)])
def test_cap_before_single_current_check_and_restore(monkeypatch,enabled,source,epoch,speed,phase,cap):
    c=controller(enabled);c.bypass.guide.phase=phase
    scene=SimpleNamespace(source_epoch=epoch,source_first_s=source,t_ref_s=source+.10)
    history=object();raw=object();calls=[]
    before=(c.bypass.guide.entry,c.bypass.guide.registered_entry)
    def fake(self,s,h,v,d,p,dt,**kwargs):
        calls.append(True)
        assert s is scene and h is history and v==speed and kwargs['raw_scan'] is raw
        assert self.bypass.guide.entry.speed_cap_mps==cap
        assert self.bypass.guide.registered_entry.speed_cap_mps==cap
        return 0.,.1,dict(handoff_state='HOLD')
    monkeypatch.setattr(BypassHandoffFollow,'command',fake)
    result=c.command(scene,history,speed,'left',.1,.02,raw_scan=raw)
    assert len(calls)==1 and result[:2]==(0.,.1)
    assert (c.bypass.guide.entry,c.bypass.guide.registered_entry)==before
    assert not c.yield_observer.detail['suggest_yield']


def test_exception_restores_entry_limits(monkeypatch):
    c=controller();before=(c.bypass.guide.entry,c.bypass.guide.registered_entry)
    def fail(*args,**kwargs):raise RuntimeError('injected failure')
    monkeypatch.setattr(BypassHandoffFollow,'command',fail)
    with pytest.raises(RuntimeError):
        c.command(SimpleNamespace(source_epoch=1,source_first_s=1.9,t_ref_s=2.),object(),.35,'left',0.,.02)
    assert (c.bypass.guide.entry,c.bypass.guide.registered_entry)==before


def test_reset_discards_advice():
    c=controller();assert c.yield_observer.detail['suggest_yield']
    c.reset();assert not c.yield_observer.detail['suggest_yield'] and c.last_yield_detail=={}
