"""SOLO 기억과 현재 옆차를 함께 확인하는 저속 공백 범위."""
from dataclasses import replace
from types import SimpleNamespace
import pytest
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.traffic_solo_gap import eligible
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from test_partial_observation import sample
from test_local_follow import history


def config():
    return FollowConfig(partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate',
        parallel_hold_trial=True,pass_release_trial=True,guard_curve_trial=True,
        traffic_recovery_trial=True,oblique_gap_trial=True)


def selector(mode='SOLO'):
    return SimpleNamespace(mode=mode,lat=object(),sides={
        name:dict(seen=False,present=False) for name in ('left','right')})


@pytest.mark.parametrize('mode',['GUARD','RELEASED','unknown'])
def test_guard_and_released_cannot_enter(mode,monkeypatch):
    monkeypatch.setattr(ps,'threat_view',lambda *a:pytest.fail('not eligible'))
    assert not eligible(selector(mode),None,None,None,0.,None)


@pytest.mark.parametrize('field',['seen','present'])
def test_previous_threat_cannot_be_cleared_by_gap(field,monkeypatch):
    s=selector();s.sides['left'][field]=True
    monkeypatch.setattr(ps,'threat_view',lambda *a:pytest.fail('history gate first'))
    assert not eligible(s,None,None,None,0.,None)
    assert s.sides['left'][field]


@pytest.mark.parametrize('side',[None,'left','right','no_path'])
def test_current_threat_even_in_previous_solo_is_rejected(side,monkeypatch):
    view=None if side=='no_path' else {name:dict(present=name==side) for name in ('left','right')}
    monkeypatch.setattr(ps,'threat_view',lambda *a:view)
    assert eligible(selector(),None,(.2,0.,dict(path_points=[])),sample(.5,range(301,306)),0.,None)==(side is None)


@pytest.mark.parametrize('missing,explained',[((),None),(range(301,306),frozenset(range(301,306)))])
def test_fully_observed_or_explained_scan_avoids_extra_geometry(missing,explained,monkeypatch):
    monkeypatch.setattr(ps,'threat_view',lambda *a:pytest.fail('no additional geometry needed'))
    assert not eligible(selector(),None,None,sample(.5,missing),0.,explained)


def test_config_preserves_solo_and_explicit_traffic_only():
    assert not FollowConfig().oblique_gap_trial
    c=config();assert not LocalFollow(c).partial_policy.oblique_metric_enabled
    for field in ('traffic_recovery_trial','guard_curve_trial','pass_release_trial'):
        with pytest.raises(ValueError):replace(c,**{field:False})
    with pytest.raises(ValueError):replace(c,oblique_gap_trial='true')


def test_exception_does_not_leave_oblique_policy_on(monkeypatch):
    import arena_autonomy.traffic_solo_gap as scope
    owner=LocalFollow(config());h=history();raw=sample(.5,range(301,306))
    scene=motion_scene(raw,h,.6,timing_verified=True,rear_half_angle_deg=0.)
    monkeypatch.setattr(owner,'_path_command',lambda *a,**k:(.2,0.,dict(reason='local_follow',path_points=[[0.,0.],[2.,0.]])))
    monkeypatch.setattr(scope,'eligible',lambda *a:True)
    def fail(*a,**k):
        assert owner.partial_policy.oblique_metric_enabled
        raise RuntimeError('injected observation error')
    monkeypatch.setattr(owner.partial_policy,'evaluate',fail)
    with pytest.raises(RuntimeError,match='injected'):
        owner._partial_command(scene,h,0.,'left',0.,.02,raw)
    assert not owner.partial_policy.oblique_metric_enabled
