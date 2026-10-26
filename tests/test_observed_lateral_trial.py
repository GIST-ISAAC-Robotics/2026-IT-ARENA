"""조건부 관측 공간 후보의 오프라인 계약. 통과가 운영 승인/주행 증거는 아니다."""
from dataclasses import replace
import numpy as np

from arena_autonomy.lateral_intent import LateralIntentConfig
from arena_autonomy.observed_lateral_trial import ObservedLateralTrial, path_observation_queries
from arena_autonomy.observation_support import depth_support
from test_lateral_intent import GRASS, ROAD, straight, car


def test_default_off_never_reads_sensor():
    path = object()
    out = ObservedLateralTrial().step(None, path, None)
    assert out['mode'] == 'OFF' and out['path'] is path and not out['runtime_authorized']


def test_solo_is_passthrough_not_global_recenter_replacement():
    scene, path = straight(-.09)
    out = ObservedLateralTrial(GRASS).step(scene, path, .35)
    assert out['mode'] == 'BASELINE' and out['path'] is path
    assert out['hidden_future_risk_unresolved'] and not out['runtime_authorized']


def test_left_right_actor_only_proposes_away_from_observed_threat():
    for sign in (1., -1.):
        d0 = -sign*.09
        scene, path = straight(d0, extra=[car(-.40, sign*.13, d0)])
        out = ObservedLateralTrial(GRASS).step(scene, path, .35)
        side = 'left' if sign > 0 else 'right'
        assert out['threats'][side]['present']
        assert 'yield' in out['checks']
        assert sign*(out['checks']['yield']['target_m']-d0) < 0
        assert out['selected'] != 'center' and not out['runtime_authorized']
        if out['selected']:
            assert out['checks'][out['selected']]['eligible_for_trial']


def test_road_policy_retains_surface_failure_even_if_grass_policy_would_add_space():
    scene, path = straight(-.09, extra=[car(-.4, .13, -.09)])
    road = ObservedLateralTrial(ROAD).step(scene, path, .35)
    grass = ObservedLateralTrial(GRASS).step(scene, path, .35)
    assert 'yield' in grass['checks']
    assert abs(grass['checks']['yield']['target_m']) > ROAD.policy_limit_m()['selected_m']
    assert road['selected'] != 'yield'


def test_missing_path_does_not_release_active_intent_to_baseline():
    scene, path = straight(-.09, extra=[car(-.4, .13, -.09)])
    trial = ObservedLateralTrial(GRASS)
    trial.step(scene, path, .35)
    assert trial.active
    out = trial.step(scene, None, .35)
    assert out['mode'] == 'HOLD_PATH' and out['path'] is None and trial.active


def test_stale_input_resets_trial_and_cannot_authorize_motion():
    scene, path = straight(0.)
    trial = ObservedLateralTrial(GRASS)
    trial.active = True
    out = trial.step(replace(scene, t_ref_s=scene.t_ref_s+.4), path, .35)
    assert out['mode'] == 'HOLD_INPUT' and out['path'] is None and not trial.active


def test_thin_unknown_sector_is_queried_between_border_samples():
    scene, _ = straight(0., dropout=lambda a: (a > 78.1) & (a < 78.7))
    assert any(r.status == 'unknown' for r in scene.rays)
    q, sectors = path_observation_queries(scene, np.array([[.10, .08, .2]]), GRASS)
    support = depth_support(scene, q, np.full(500, .055))
    assert sectors > 0 and not support.all()


def test_all_unknown_rejects_input_instead_of_calling_absence_clear():
    scene, path = straight(0., dropout=lambda a: np.ones(len(a), dtype=bool))
    out = ObservedLateralTrial(GRASS).step(scene, path, .35)
    # source_valid checks format/timing, not return completeness; baseline still uses original controller gates.
    assert out['mode'] in ('BASELINE', 'HOLD_INPUT') and not out['runtime_authorized']
    trial = ObservedLateralTrial(GRASS)
    trial.step(straight(0.)[0], path, .35)
    trial.active = True
    out = trial.step(scene, path, .35)
    assert out['selected'] is None and out['path'] is None
