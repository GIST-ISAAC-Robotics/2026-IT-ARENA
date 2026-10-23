"""시간축만 분리한 shadow 회귀. 실행 허가에는 사용하지 않는다."""
from dataclasses import replace
import numpy as np
import pytest

import probe_corner_temporal as temporal
from test_corner_corridor_shadow import ring_scene
from test_local_follow import history
from arena_autonomy.local_follow import FollowConfig


@pytest.fixture
def fixed_proposal(monkeypatch):
    x = np.linspace(-.15,1.,40)
    path = np.column_stack((x,.2*x*x))
    def shadow(scene, speed, side, previous, dt, config):
        return dict(reason='reverse_motion' if speed<0 else 'geometry_and_point_occupancy_only',
                    geometry_supported=True, preliminary_motion_supported=speed>=0,
                    proposed_speed_mps=.6 if speed>=0 else 0., steering_rad=.1,
                    geometry={'path':path.tolist()}, swept_clearance_m=1.)
    monkeypatch.setattr(temporal,'shadow_scene',shadow)
    return path


def call(controller,t,*,h=None,**kwargs):
    scene=ring_scene(now=t)
    if 'epoch' in kwargs:
        epoch=kwargs.pop('epoch')
        scene=replace(scene,source_epoch=epoch,rays=tuple(replace(r,source_epoch=epoch) for r in scene.rays))
    return controller.observe(scene,history() if h is None else h,kwargs.get('speed',0.),
                              kwargs.get('side','left'),0.,.02)


def test_requires_distinct_geometry_and_release_confirmations(fixed_proposal):
    ctl=temporal.TemporalCornerShadow()
    results=[call(ctl,t) for t in (.4,.5,.6)]
    assert [r['temporal_candidate'] for r in results]==[False,False,True]
    assert [r['release_count'] for r in results]==[0,1,2]
    assert all(r['driving_authorized'] is False for r in results)


def test_same_scan_never_counts_twice(fixed_proposal):
    ctl=temporal.TemporalCornerShadow()
    scene=ring_scene()
    for now in (.4,.42,.44):
        result=ctl.observe(replace(scene,t_ref_s=now),history(),0.,'left',0.,.02)
        assert not result['temporal_candidate']
        assert result['confirmations']==1


@pytest.mark.parametrize('change',[{'epoch':1},{'side':'right'}])
def test_epoch_or_side_change_starts_new_confirmation(fixed_proposal,change):
    ctl=temporal.TemporalCornerShadow()
    for t in (.4,.5,.6):call(ctl,t)
    result=call(ctl,.7,**change)
    assert not result['temporal_candidate'] and result['confirmations']==1
    assert result['reset_cause']=='epoch_or_side_changed'


def test_old_source_not_recent_reference_controls_cache_expiry(fixed_proposal):
    ctl=temporal.TemporalCornerShadow()
    call(ctl,.4)
    result=call(ctl,.61)
    assert result['previous_source_age_s']==pytest.approx(.31)
    assert result['confirmations']==1 and not result['previous_path_agrees']


def test_regressed_source_is_rejected_not_reconfirmed(fixed_proposal):
    ctl=temporal.TemporalCornerShadow()
    call(ctl,.5)
    result=call(ctl,.4)
    assert result['reason']=='reference_or_source_regressed'
    assert not result['temporal_candidate']


def test_signed_reverse_resets_confirmation(fixed_proposal):
    ctl=temporal.TemporalCornerShadow()
    for t in (.4,.5,.6):call(ctl,t)
    assert call(ctl,.7,speed=-.01)['confirmations']==0
    assert call(ctl,.8)['confirmations']==1


def test_missing_motion_history_cannot_correlate(fixed_proposal):
    from arena_autonomy.lidar_motion import MotionHistory
    ctl=temporal.TemporalCornerShadow()
    call(ctl,.4)
    result=call(ctl,.5,h=MotionHistory())
    assert result['confirmations']==1
    assert result['transport_error']


def test_displaced_path_above_current_limit_cannot_confirm(fixed_proposal,monkeypatch):
    ctl=temporal.TemporalCornerShadow()
    call(ctl,.4)
    monkeypatch.setattr(temporal,'transport_points',lambda p,*args:np.asarray(p)+[.5,0.])
    result=call(ctl,.5)
    assert result['transport_displacement_m']==pytest.approx(.5)
    assert result['confirmations']==1 and not result['previous_path_agrees']


def test_current_missing_geometry_does_not_run_on_old_path(fixed_proposal,monkeypatch):
    ctl=temporal.TemporalCornerShadow()
    for t in (.4,.5,.6):call(ctl,t)
    monkeypatch.setattr(temporal,'shadow_scene',lambda *args:dict(reason='front_unobserved',
                        geometry_supported=False,preliminary_motion_supported=False))
    result=call(ctl,.7)
    assert not result['temporal_candidate'] and result['confirmations']==0
    assert ctl.path is None


def test_reused_scan_changed_path_discards_previous_confirmations(fixed_proposal,monkeypatch):
    original=temporal.shadow_scene
    ctl=temporal.TemporalCornerShadow()
    for t in (.4,.5,.6):call(ctl,t)
    def changed(*args):
        result=original(*args)
        result['geometry']['path']=(fixed_proposal+[0.,.3]).tolist()
        return result
    monkeypatch.setattr(temporal,'shadow_scene',changed)
    scene=replace(ring_scene(now=.6),t_ref_s=.62)
    result=ctl.observe(scene,history(),0.,'left',0.,.02)
    assert result['reason']=='reused_scan_path_unconfirmed'
    assert not result['temporal_candidate'] and ctl.path is None


def test_occupancy_hold_requires_two_new_release_observations(fixed_proposal,monkeypatch):
    original=temporal.shadow_scene
    ctl=temporal.TemporalCornerShadow()
    for t in (.4,.5,.6):call(ctl,t)
    def blocked(*args):
        result=original(*args)
        result.update(reason='existing_occupancy_margin_hold',preliminary_motion_supported=False,
                      proposed_speed_mps=0.,swept_clearance_m=.20)
        return result
    monkeypatch.setattr(temporal,'shadow_scene',blocked)
    result=call(ctl,.7)
    assert not result['temporal_candidate'] and result['release_count']==0
    monkeypatch.setattr(temporal,'shadow_scene',original)
    assert not call(ctl,.8)['temporal_candidate']
    assert call(ctl,.9)['temporal_candidate']
