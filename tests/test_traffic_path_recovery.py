"""상태/현재 영역 주입 반례. 센서 추론이나 실제 주행 성공의 시험은 아니다."""
from dataclasses import replace
from types import SimpleNamespace
import pytest
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.traffic_path_recovery_trial import SuspendedPassageSelector
from test_pass_release_trial import config,output,view


@pytest.fixture
def fixture(monkeypatch):
    cfg=replace(config(),parallel_hold_trial=True,pass_release_trial=True,
                guard_curve_trial=True,traffic_recovery_trial=True,
                traffic_path_recovery_trial=True,corridor_fallback_enabled=True)
    ctl=SuspendedPassageSelector(cfg)
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


def test_finite_reverse_between_boundary_waits_preserves_history_but_never_moves(fixture):
    ctl,state,call=fixture
    call(1.,'follow_boundary_unconfirmed')
    out=call(1.02,'follow_reverse_motion',speed=-.05,source=.9)
    assert out[0]==0. and ctl.suspended_passage=={'left'}
    assert call(1.1,'follow_boundary_unconfirmed')[0]==0.
    assert call(1.2)[0]==.35


def test_reverse_with_clock_regression_clears_history(fixture):
    ctl,state,call=fixture
    call(1.,'follow_boundary_unconfirmed')
    out=call(1.02,'follow_reverse_motion',speed=-.05,source=.8)
    assert out[0]==0. and not ctl.suspended_passage
    assert call(1.1)[0]==0.


def test_other_side_new_object_blocks_restored_left_history(fixture,monkeypatch):
    ctl,state,call=fixture
    call(1.,'follow_boundary_unconfirmed')
    observed=view()
    observed['right']=dict(present=True,points=4,rearmost_ds_m=-.1,frontmost_ds_m=.1)
    monkeypatch.setattr(ps,'threat_view',lambda *a:observed)
    out=call(1.1)
    assert out[0]==0. and ctl.mode=='GUARD'
    assert ctl.sides['right']['present'] and not ctl.sides['right']['passed']


def test_explicit_option_and_dependency_validation():
    from arena_autonomy.local_follow import FollowConfig,LocalFollow
    assert not FollowConfig().traffic_path_recovery_trial
    base=replace(config(),parallel_hold_trial=True,pass_release_trial=True,
                 guard_curve_trial=True,traffic_recovery_trial=True,corridor_fallback_enabled=True)
    for value in (None,1,'true'):
        with pytest.raises(ValueError):replace(base,traffic_path_recovery_trial=value)
    for changes in ({'traffic_recovery_trial':False},{'corridor_fallback_enabled':False}):
        with pytest.raises(ValueError):replace(base,traffic_path_recovery_trial=True,**changes)
    assert type(LocalFollow(base).pass_selector) is ps.PassReleaseSelector
    assert isinstance(LocalFollow(replace(base,traffic_path_recovery_trial=True)).pass_selector,
                      SuspendedPassageSelector)


def test_default_off_does_not_call_role_alternative(monkeypatch):
    import arena_autonomy.corridor_roles as roles
    from arena_autonomy.corridor_observer import current_proposal
    from test_corner_corridor_shadow import ring_scene
    cfg=replace(config(),corridor_fallback_enabled=True)
    def forbidden(*args):raise AssertionError('role candidate called while OFF')
    monkeypatch.setattr(roles,'role_aware_evidence',forbidden)
    result=current_proposal(ring_scene(.15),0.,'right',.2,.02,cfg)
    assert not result['geometry_supported']


@pytest.mark.parametrize('recovery,corridor',[(False,False),(False,True),(True,False)])
def test_cli_rejects_missing_prerequisites_before_launch(recovery,corridor):
    from validate_g1_traffic import traffic_path_recovery_arguments
    with pytest.raises(ValueError):traffic_path_recovery_arguments(True,recovery,corridor)
    assert traffic_path_recovery_arguments(False,recovery,corridor)==[]


def test_cli_and_runtime_dependencies_are_in_recording_manifests():
    from validate_g1_traffic import traffic_path_recovery_arguments,SOURCE_FILES
    from profile_follow_trace import SNAPSHOT_MODULES
    assert traffic_path_recovery_arguments(True,True,True)==['g1_traffic_path_recovery_trial:=true']
    for name in ('traffic_path_recovery_trial','corridor_roles'):
        assert name in SNAPSHOT_MODULES['arena_autonomy']
        assert f'src/arena_autonomy/arena_autonomy/{name}.py' in SOURCE_FILES
