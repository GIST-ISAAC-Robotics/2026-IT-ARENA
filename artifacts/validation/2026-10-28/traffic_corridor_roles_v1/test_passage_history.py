"""상태/현재 영역 주입 반례. 센서 추론이나 실제 주행 성공의 시험은 아니다."""
from dataclasses import replace
from types import SimpleNamespace
import pytest
import arena_autonomy.pass_release_trial as ps
from passage_history_candidate import SuspendedPassageCandidate
from test_pass_release_trial import config,output,view


@pytest.fixture
def fixture(monkeypatch):
    cfg=replace(config(),parallel_hold_trial=True,pass_release_trial=True,
                guard_curve_trial=True,traffic_recovery_trial=True)
    ctl=SuspendedPassageCandidate(cfg)
    state=dict(present=False,unknown=0,checks=0,path=True)
    monkeypatch.setattr(ps,'threat_view',lambda *a:view(state['present'],-.1) if state['path'] else None)
    monkeypatch.setattr(ps,'band_check',lambda *a:dict(free=False))
    monkeypatch.setattr(ps,'side_extent',lambda *a:0.)
    monkeypatch.setattr(ps,'lateral_excursion',lambda *a:.005)
    def region(*a,**kw):
        state['checks']+=1
        return dict(unobserved=state['unknown'],masked_left=0,masked_right=0)
    monkeypatch.setattr(ps,'motion_region',region)
    ctl.sides['left'].update(seen=True,passed=True,clear=2,vanish='rear')
    def call(t,reason='local_follow',speed=0.,epoch=0,source=None,preferred='right'):
        path=None if reason=='follow_boundary_unconfirmed' else [[0.,0.],[1.,0.]]
        state['path']=path is not None
        b=(.8 if reason=='local_follow' else 0.,.2,dict(reason=reason,path_points=path,preferred_wall=preferred))
        scene=SimpleNamespace(source_first_s=t-.1 if source is None else source,t_ref_s=t,source_epoch=epoch)
        return ctl.select(scene,b,output(0.,.2,'parallel_hold_wait'),speed,.2,.02)
    return ctl,state,call


def test_completed_history_can_resume_only_through_current_region(fixture):
    ctl,state,call=fixture
    assert call(1.,'follow_boundary_unconfirmed')[0]==0.
    assert ctl.mode=='GUARD' and ctl.suspended_passage=={'left'}
    out=call(1.1)
    assert out[0]==.35 and out[2]['retained_passage']
    assert out[2]['restored_passage_history']==['left'] and state['checks']==1
    assert not out[2]['restored_history_is_current_free_space']


def test_unfinished_clearance_never_becomes_completed_history(fixture):
    ctl,state,call=fixture
    ctl.sides['left'].update(passed=False,clear=1)
    call(1.,'follow_boundary_unconfirmed')
    assert not ctl.suspended_passage and call(1.1)[0]==0.


@pytest.mark.parametrize('fault',['new_threat','unknown_region','reverse','epoch','time_regressed',
                                  'input_stop','reset','preferred_changed','stale','invalid_motion'])
def test_history_never_bypasses_current_or_discontinuous_inputs(fixture,fault):
    ctl,state,call=fixture
    call(1.,'follow_boundary_unconfirmed')
    kwargs={}
    if fault=='new_threat':state['present']=True
    if fault=='unknown_region':state['unknown']=1
    if fault=='reverse':kwargs['speed']=-.01
    if fault=='epoch':kwargs['epoch']=1
    if fault=='time_regressed':kwargs['source']=.8
    if fault=='input_stop':kwargs['reason']='follow_input_stop'
    if fault=='reset':ctl.invalidate('input_reset')
    if fault=='preferred_changed':kwargs['preferred']='left'
    if fault=='stale':kwargs['source']=.7
    if fault=='invalid_motion':kwargs['speed']=float('nan')
    assert call(1.1,**kwargs)[0]==0.


def test_reused_scan_cannot_restore_history(fixture):
    ctl,state,call=fixture
    call(1.,'follow_boundary_unconfirmed')
    assert call(1.02,source=.9)[0]==0.
    assert ctl.suspended_passage=={'left'}
    assert call(1.1)[0]==.35


def test_path_absence_and_wait_do_not_emit_motion(fixture):
    ctl,state,call=fixture
    for t in (1.,1.1,1.2,1.3):
        assert call(t,'follow_boundary_unconfirmed')[0]==0.
    out=call(1.4,'follow_release_wait')
    assert out[0]==0.
    assert call(1.5)[0]==.35
