from dataclasses import replace
from types import SimpleNamespace
import pytest
import arena_autonomy.bypass_relative_yield_pause as module
from arena_autonomy.bypass_handoff import BypassHandoffFollow
from test_bypass_relative_yield import active,row,subject


def setup():
    owner=active();helper=module.MotionPauseRelativeYieldCommand()
    helper.observer=owner._relative_yield.observer;owner._relative_yield=helper
    return owner,helper


def scene(now,epoch=1):
    return SimpleNamespace(t_ref_s=now,source_first_s=round(now-.1,8),source_epoch=epoch)


def hold(owner,helper,now,reason='entry_reverse_motion_hold',speed=-.01):
    before=owner.bypass.guide.entry
    out=helper.command(owner,scene(now),speed,lambda:(0.,.01,dict(handoff_state='BYPASS',
        bypass=dict(reason=reason,geometry_retained=True))))
    assert out[:2]==(0.,.01) and owner.bypass.guide.entry==before
    return out[2]['relative_yield']


def test_v2_explicit_and_v1_preserved():
    old=subject();new=BypassHandoffFollow(execution=replace(old.execution,
        relative_yield_policy='bounded_relative_yield_v2'),allow_simulation_trial=True)
    assert isinstance(new._relative_yield,module.MotionPauseRelativeYieldCommand)
    assert not isinstance(old._relative_yield,module.MotionPauseRelativeYieldCommand)


def test_short_motion_wait_retains_only_slow_intent_then_current_recheck(monkeypatch):
    owner,helper=setup()
    for i in range(7):
        result=hold(owner,helper,2.+i*.1)
        assert result['motion_pause_intent_retained'] and not result['cap_requested']
    current=row(16);current.update(now=2.7,along_low=.22,along_high=.42)
    monkeypatch.setattr(module,'observation',lambda out,now:current)
    calls=[]
    def check():
        calls.append(owner.bypass.guide.entry.speed_cap_mps)
        return .2,.01,dict(handoff_state='BYPASS')
    out=helper.command(owner,scene(2.7),.2,check)
    assert calls==[.2] and out[:2]==(.2,.01)
    assert out[2]['relative_yield']['current_observer']['motion_pause_resumed']
    assert helper.pause is None and helper.observer.rows[-1]['source']==2.6


@pytest.mark.parametrize('change',['expired','epoch','source_gap','fault','phase','stop'])
def test_pause_does_not_survive_faults_or_expiry(change):
    owner,helper=setup();assert hold(owner,helper,2.)['motion_pause_intent_retained']
    if change=='stop':
        helper.reset();assert helper.pause is None and not helper.observer.detail['suggest_yield'];return
    now=2.1;state=scene(now);reason='entry_reverse_motion_hold'
    if change=='expired':
        for i in range(1,11):hold(owner,helper,2.+i*.1)
        state=scene(3.1)
    elif change=='epoch':state=scene(now,2)
    elif change=='source_gap':state=scene(2.4)
    elif change=='fault':reason='entry_reverse_motion_transport_unavailable'
    elif change=='phase':owner.bypass.guide.phase='rejoin'
    out=helper.command(owner,state,-.01,lambda:(0.,0.,dict(handoff_state='BYPASS',
        bypass=dict(reason=reason,geometry_retained=True))))
    assert not out[2]['relative_yield']['motion_pause_intent_retained']
    assert helper.pause is None and not helper.observer.detail['suggest_yield']


def test_resume_changed_face_is_rejected_without_clipping_current_hold(monkeypatch):
    owner,helper=setup();hold(owner,helper,2.)
    current=row(10);current.update(now=2.1,normal=.9)
    monkeypatch.setattr(module,'observation',lambda out,now:current)
    out=helper.command(owner,scene(2.1),.2,lambda:(0.,.12,dict(handoff_state='BYPASS')))
    assert out[:2]==(0.,.12) and not helper.observer.detail['suggest_yield']


def test_pause_does_not_extend_original_five_second_budget():
    owner,helper=setup();helper.observer.yield_since=-3.01
    assert not hold(owner,helper,2.)['motion_pause_intent_retained']
