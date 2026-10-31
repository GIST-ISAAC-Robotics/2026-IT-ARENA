"""초기 기준의 추가 허용이 현재 우회 준비 상태에 한정되는지 확인한다."""
from types import SimpleNamespace as NS
import pytest
import candidate as c
from arena_autonomy.bypass_handoff import handoff_baseline


@pytest.fixture
def setup(monkeypatch):
    obj=c.ReadyPairFollow(handoff_baseline(),allow_simulation_trial=True)
    scene=NS(t_ref_s=10.14,source_first_s=10.,source_frame_id='laser_frame',source_epoch=0)
    original=(None,dict(reason='straight_entry_only'))
    proposed=((0.,0.,-.15,1.5),dict(reason='current_tracked_pair_candidate'))
    monkeypatch.setattr(c.prior,'ORIGINAL_REFERENCE',lambda *_:original)
    monkeypatch.setattr(obj.reference_memory,'resolve',lambda *_:proposed)
    obj.monitor.result=dict(observed=True,ready=True)
    obj.monitor.motion_evidence=lambda:False
    def invoke(*args,**kwargs):
        refs=[m.straight_reference(scene,obj.config) for m in (c.handoff,c.entry,c.corridor)]
        assert refs[1:]==refs[:2]
        return 0.,0.,dict(reason='handoff_follow',reference=refs[0])
    monkeypatch.setattr(c.handoff.BypassHandoffFollow,'command',invoke)
    return obj,scene,original,proposed


def call(obj,scene):return obj.command(scene,None,0.,'left',0.,.02,raw_scan=object())


def test_ready_current_pair_is_added(setup):
    obj,scene,_,proposed=setup
    assert call(obj,scene)[2]['reference']==proposed
    assert obj.reference_use['fallback_calls']==3


@pytest.mark.parametrize('observed,ready,moving',[(False,True,False),(True,False,False),
    (True,True,True),(False,False,False),(None,True,False),(True,None,False)])
def test_incomplete_or_moving_evidence_keeps_original(setup,observed,ready,moving):
    obj,scene,original,_=setup
    obj.monitor.result=dict(observed=observed,ready=ready);obj.monitor.motion_evidence=lambda:moving
    assert call(obj,scene)[2]['reference']==original
    assert obj.reference_use['fallback_calls']==0 and obj.reference_use['withheld_calls']==3


def test_original_valid_is_never_replaced(setup,monkeypatch):
    obj,scene,_,_=setup
    original=((.01,0.,-.15,1.4),dict(reason='current_two_wall_straight_reference'))
    monkeypatch.setattr(c.prior,'ORIGINAL_REFERENCE',lambda *_:original)
    assert call(obj,scene)[2]['reference']==original
    assert obj.reference_use['fallback_calls']==0


def test_no_current_pair_keeps_original_failure(setup,monkeypatch):
    obj,scene,original,_=setup
    monkeypatch.setattr(obj.reference_memory,'resolve',lambda *_:(None,dict(reason='current_pair_unavailable')))
    assert call(obj,scene)[2]['reference']==original
    assert obj.reference_use['fallback_calls']==0


def test_gate_uses_monitor_updated_in_current_command(setup,monkeypatch):
    obj,scene,original,_=setup
    def invoke(*args,**kwargs):
        obj.monitor.result=dict(ready=False,observed=True)
        return 0.,0.,dict(reason='handoff_follow',reference=c.entry.straight_reference(scene,obj.config))
    monkeypatch.setattr(c.handoff.BypassHandoffFollow,'command',invoke)
    assert call(obj,scene)[2]['reference']==original


def test_aliases_restore_and_fault_clears_memory(setup,monkeypatch):
    obj,scene,_,_=setup
    before=[m.straight_reference for m in (c.handoff,c.entry,c.corridor)]
    obj.reference_memory.anchors={'left':object()}
    monkeypatch.setattr(c.handoff.BypassHandoffFollow,'command',lambda *a,**k:(0.,0.,dict(reason='handoff_input_fault')))
    call(obj,scene)
    assert not obj.reference_memory.anchors
    assert before==[m.straight_reference for m in (c.handoff,c.entry,c.corridor)]
