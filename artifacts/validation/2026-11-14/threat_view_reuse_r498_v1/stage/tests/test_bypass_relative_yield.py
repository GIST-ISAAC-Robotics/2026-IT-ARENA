from dataclasses import asdict, replace
from types import SimpleNamespace
import numpy as np
import pytest
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.bypass_entry_follow import BypassExecutionConfig
from arena_autonomy.bypass_relative_yield import BoundedRelativeYield


def row(i):
    return dict(phase='entry', side='right', epoch=1, source=1+i*.1, now=1.15+i*.1,
        heading=0., speed=.35, normal=.35, along_low=.1, along_high=.3,
        method='parallel_face_median', blocked_by_current_returns=True,
        band_supported=False, wall_confirmations=3)


def subject(enabled=True):
    execution=HandoffExecutionConfig(bypass=BypassExecutionConfig(corridor_policy='corridor_v2_signed_hold'),
        preparation_policy='observed_entry_space_v1', persistence_window_policy='trim_before_compare_v1',
        follow_entry_space_enabled=True, reacquire_policy='known_hold_v1', entry_policy='corridor_local_v1',
        motion_pause_policy='bounded_motion_pause_v1',
        relative_yield_policy='bounded_relative_yield_v1' if enabled else 'off')
    return BypassHandoffFollow(execution=execution, allow_simulation_trial=True)


def active():
    c=subject(); c.mode='BYPASS'; c.bypass.guide.path=np.array([[0.,0.],[1.,0.]])
    for i in range(9): c._relative_yield.observer.update(row(i))
    assert c._relative_yield.observer.detail['suggest_yield']
    return c


def test_default_and_old_header_keep_off():
    assert HandoffExecutionConfig().relative_yield_policy=='off'
    old=asdict(subject(False).execution); del old['relative_yield_policy']
    assert HandoffExecutionConfig.from_mapping(old).relative_yield_policy=='off'
    assert subject(False)._relative_yield is None


@pytest.mark.parametrize('value',[True,None,'auto','always'])
def test_invalid_policy_rejected(value):
    with pytest.raises(ValueError): replace(subject().execution,relative_yield_policy=value)


def test_policy_dependencies_explicit():
    with pytest.raises(ValueError): HandoffExecutionConfig(relative_yield_policy='bounded_relative_yield_v1')
    with pytest.raises(ValueError): replace(subject().execution,motion_pause_policy='off')


@pytest.mark.parametrize('speed,epoch,source,phase,cap',[
    (.35,1,1.9,'entry',.2),(-.01,1,1.9,'entry',.35),(1.1,1,1.9,'entry',.35),
    (.35,2,1.9,'entry',.35),(.35,1,2.3,'entry',.35),(.35,1,1.9,'rejoin',.35)])
def test_single_current_check_uses_real_measurement_and_restores_limits(monkeypatch,speed,epoch,source,phase,cap):
    c=active(); c.bypass.guide.phase=phase
    scene=SimpleNamespace(source_epoch=epoch,source_first_s=source,t_ref_s=source+.1)
    history=object(); raw=object(); calls=[]; before=(c.bypass.guide.entry,c.bypass.guide.registered_entry)
    def command(s,h,v,side,p,dt,**kwargs):
        calls.append(1)
        assert s is scene and h is history and v==speed and kwargs['raw_scan'] is raw
        assert c.bypass.guide.entry.speed_cap_mps==cap
        assert c.bypass.guide.registered_entry.speed_cap_mps==cap
        return 0.,.12,dict(handoff_state='FOLLOW',reason='injected_current_hold')
    monkeypatch.setattr(c,'_prepared_command',command)
    out=c.command(scene,history,speed,'left',.12,.02,raw_scan=raw)
    assert out[:2]==(0.,.12) and calls==[1]
    assert (c.bypass.guide.entry,c.bypass.guide.registered_entry)==before
    assert not c._relative_yield.observer.detail['suggest_yield']


def test_exception_and_stop_reset(monkeypatch):
    c=active(); before=(c.bypass.guide.entry,c.bypass.guide.registered_entry)
    def fail(*args,**kwargs): raise RuntimeError('injected')
    monkeypatch.setattr(c,'_prepared_command',fail)
    with pytest.raises(RuntimeError):
        c.command(SimpleNamespace(source_epoch=1,source_first_s=1.9,t_ref_s=2.),object(),.35,'left',0.,.02)
    assert (c.bypass.guide.entry,c.bypass.guide.registered_entry)==before
    c.reset(); assert not c._relative_yield.observer.detail['suggest_yield']


def test_current_clear_band_waits_for_existing_rejoin_confirmation():
    o=BoundedRelativeYield()
    for i in range(9):o.update(row(i))
    for i in (9,10):
        r=row(i); r.update(band_supported=True,blocked_by_current_returns=False)
        assert o.update(r)['suggest_yield']
    r=row(11); r['phase']='rejoin'
    assert not o.update(r)['suggest_yield']


def test_far_face_only_continues_existing_advice_and_budget_expires():
    o=BoundedRelativeYield()
    for i in range(9):
        r=row(i); r.update(along_low=.6,along_high=.8)
        assert not o.update(r)['suggest_yield']
    o.reset()
    for i in range(9):o.update(row(i))
    for i in range(9,57):
        r=row(i); shift=min(.015*(i-8),.6)
        r.update(along_low=.1+shift,along_high=.3+shift)
        assert o.update(r)['suggest_yield']
    r=row(57);r.update(along_low=.7,along_high=.9)
    assert not o.update(r)['suggest_yield']
    assert o.detail['reason']=='continuation_budget_expired'


@pytest.mark.parametrize('change',[dict(epoch=2),dict(side='left'),dict(normal=.5),dict(method='low_order'),
    dict(source=3.,now=3.1),dict(speed=-.1),dict(heading=.2),dict(wall_confirmations=0)])
def test_fault_or_changed_face_clears_advice(change):
    o=active()._relative_yield.observer; r=row(9);r.update(change)
    assert not o.update(r)['suggest_yield']
