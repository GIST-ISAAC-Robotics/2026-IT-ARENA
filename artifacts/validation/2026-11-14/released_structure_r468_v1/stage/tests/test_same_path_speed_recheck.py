"""통과 후 제한 결측 후보의 실제 속도 축소/재검사. 센서는 실제 함수, 상태 분기만 주입."""
from dataclasses import replace
import math
import numpy as np
import pytest
from arena_autonomy import pass_release_trial as ps
from test_same_path_gap_probe import fixture
from test_pass_release_trial import config, output, view


def run(monkeypatch, *, target=.45, measured=.2, extra=None, enabled=True,
        reason='bounded_side_missing', allowed=True, checked_speed=.5,
        checked_steer=.2, queries=None, unobserved=1, tail=0., front=0., stop=False):
    raw, scene, proof, _ = fixture(extra)
    proof.update(reason=reason, allowed=allowed)
    proof['timed_missing_diagnostic'].update(checked_speed_mps=checked_speed,
        checked_steering_rad=checked_steer)
    ctl = ps.PassReleaseSelector(replace(config(), parallel_hold_trial=True,
        pass_release_trial=True, guard_curve_trial=True, traffic_recovery_trial=enabled))
    ctl.sides['left'].update(seen=True, passed=True, clear=2, vanish='benign')
    monkeypatch.setattr(ps, 'threat_view', lambda *args: view())
    monkeypatch.setattr(ps, 'band_check', lambda *args: dict(free=True))
    monkeypatch.setattr(ps, 'lateral_excursion', lambda *args: .04)
    calls = []
    q = np.array([[.06, .08]]) if queries is None else np.array(queries)
    def region(scene, speed, target, steer, previous, dt, cfg, lat, *, context=None):
        calls.append((speed, target, steer, previous))
        if context is not None:
            context.update(queries=q, free=np.full(len(q), not unobserved),
                           masked=np.zeros(len(q), dtype=bool), padding=np.full(500, .1))
        return dict(unobserved=unobserved, queries=len(q), masked_left=int(bool(tail or front)),
                    masked_right=0, masked_tail_extent_m=tail, masked_front_samples=front)
    monkeypatch.setattr(ps, 'motion_region', region)
    baseline = output(0. if stop else target, .2, 'follow_gap_hold' if stop else 'local_follow')
    baseline[2]['partial_observation'] = proof
    result = ctl.select(scene, baseline, output(0., reason='parallel_hold_partial_observation_outside_trial'),
                        measured, .2, .02, raw_scan=raw)
    return result, calls


@pytest.mark.parametrize('target', [.35000000000000003, .4, .45, .5])
def test_released_side_gap_rechecks_reduced_command(monkeypatch, target):
    result, calls = run(monkeypatch, target=target)
    assert result[:2] == (.35, .2)
    assert calls == [(.2, target, .2, .2), (.2, .35, .2, .2)]
    d=result[2]
    assert d['selector_reason']=='released_bounded_same_path_gap'
    assert d['selector_region']['unobserved']==1
    assert not d['selector_unknown_filled']
    assert d['selector_same_path_speed_recheck']['original_target_mps']==target


def test_real_measured_speed_is_not_lowered_with_target(monkeypatch):
    result,calls=run(monkeypatch, measured=.48)
    assert result[0]==.35
    assert all(c[0]==.48 for c in calls)


@pytest.mark.parametrize('kwargs', [
    dict(measured=.51), dict(checked_speed=.34), dict(checked_steer=.21),
    dict(extra={374:.09}), dict(extra={250:math.inf}),
    dict(extra={i:math.inf for i in range(373,379)}),
    dict(extra={375:math.nan}), dict(queries=[[.06,.5]]),
    dict(tail=.01001),dict(front=1),dict(measured=-.01),
    dict(stop=True),dict(allowed=False),dict(enabled=False),
])
def test_recheck_does_not_bypass_obstacle_unknown_motion_or_stop(monkeypatch,kwargs):
    assert run(monkeypatch, **kwargs)[0][0]==0.


@pytest.mark.parametrize('kwargs', [dict(target=.35), dict(unobserved=0),
                                  dict(reason='fully_observed'), dict(enabled=False)])
def test_unneeded_or_disabled_recheck_preserves_original_attempt(monkeypatch,kwargs):
    result,calls=run(monkeypatch,**kwargs)
    assert len(calls)==1
    assert 'selector_same_path_speed_recheck' not in result[2]


def test_actual_motion_region_uses_higher_measured_speed():
    raw,scene,_,_=fixture()
    cfg=config();lat=ps.lateral_config(cfg)
    high,_=ps.motion_queries(scene,.48,.45,.2,.2,.02,cfg,lat)
    reduced,_=ps.motion_queries(scene,.48,.35,.2,.2,.02,cfg,lat)
    quiet,_=ps.motion_queries(scene,.2,.35,.2,.2,.02,cfg,lat)
    assert high==reduced>quiet
