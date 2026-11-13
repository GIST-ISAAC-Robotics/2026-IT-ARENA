"""직진 노면 확약 구간 후보. 먼 경로 실패/가까운 실패를 구분하며 관측/간격은 그대로다."""
from dataclasses import replace
import math
import numpy as np
import pytest
import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import GRASS
from test_m3_small_center_straight import road_path, active
from test_m3_execution_trial import padding
from test_m3_observed_follow import timed_scene
from test_local_follow import history

ON = mx.ExecutionConfig(command_fallback=True, release_to_baseline=True, straight_surface_commitment=True)
OFF = replace(ON, straight_surface_commitment=False)


def straight(path):
    frame = mx.ExecutionFrame(path)
    return frame, mx.straight_candidate(frame, mx.ego_state(frame, GRASS), GRASS)


def test_typed_opt_in_dependencies():
    assert not mx.ExecutionConfig().straight_surface_commitment
    with pytest.raises(ValueError, match='bool'):
        replace(ON, straight_surface_commitment=1)
    for key in ('command_fallback', 'release_to_baseline', 'straight_start'):
        with pytest.raises(ValueError, match='requires'):
            replace(ON, **{key: False})


def test_far_curve_is_not_immediate_road_loss_and_margin_is_preserved():
    frame, candidate = straight(road_path(radius=2.))
    before = {k: v.copy() for k, v in candidate.items() if isinstance(v, np.ndarray)}
    full = mx.surface_check(frame, candidate, GRASS, current_pose=(0., 0., 0.))
    short = mx.straight_surface_prefix(candidate, .3125)
    nearby = mx.surface_check(frame, short, GRASS, current_pose=(0., 0., 0.))
    assert not full['ok'] and nearby['ok']
    assert nearby['edge_margin_m'] == full['edge_margin_m']
    assert short['curved'] == candidate['curved']
    assert short['progress'][-1] == .3125
    assert short['poses'][-1].tolist() == [.3125, 0., 0.]
    assert short['path'] is candidate['path']  # 조향용 경로를 단축하지 않는다.
    for key, value in before.items():
        np.testing.assert_array_equal(candidate[key], value)


@pytest.mark.parametrize('distance', [0., .025, .3125, 1.2])
def test_endpoint_present_once_without_rounding_down(distance):
    _, candidate = straight(road_path())
    prefix = mx.straight_surface_prefix(candidate, distance)
    assert prefix['progress'][-1] == distance
    assert np.sum(prefix['progress'] == distance) == 1
    assert np.all(np.diff(prefix['progress']) > 0)


@pytest.mark.parametrize('distance', [-.1, math.nan, math.inf, 2.])
def test_unavailable_or_invalid_horizon_rejected(distance):
    _, candidate = straight(road_path())
    with pytest.raises(ValueError):
        mx.straight_surface_prefix(candidate, distance)


def test_interpolated_endpoint_can_be_first_unsafe_pose():
    # road_only 합성 도로의 경계를 직선 운동 0.30625 m에 정확히 둔다.
    # 회전 사각형의 횡반폭은 L/2*sin(psi)+W/2*cos(psi); 끝을 .30으로 내리면 거절을 놓친다.
    psi, boundary = math.pi/6, .30625
    frame, candidate = straight(road_path(psi=psi))
    half_span = GRASS.length_m/2*math.sin(psi)+GRASS.width_m/2*math.cos(psi)
    edge = boundary*math.sin(psi)+half_span+GRASS.budget.edge_margin(False)[0]
    cfg = replace(GRASS, surface_policy='road_only', road_half_m=edge)
    coarse = mx.straight_surface_prefix(candidate, .30)
    exact = mx.straight_surface_prefix(candidate, .3125)
    assert mx.surface_check(frame, coarse, cfg, current_pose=(0., 0., 0.))['ok']
    assert not mx.surface_check(frame, exact, cfg, current_pose=(0., 0., 0.))['ok']


def test_nonstraight_and_nonfinite_geometry_cannot_use_linear_prefix():
    _, candidate = straight(road_path())
    for field, value in [('name', 'center'), ('poses', np.full_like(candidate['poses'], np.nan)),
                         ('steering_rad', np.ones_like(candidate['steering_rad']))]:
        bad = dict(candidate, **{field: value})
        with pytest.raises(ValueError):
            mx.straight_surface_prefix(bad, .30)


def test_surface_shortening_does_not_exempt_unknown_or_release_history():
    trial = active(ON, -.004)
    scene, path = timed_scene(.4, -.004, dropout=lambda deg: np.abs(deg) < 12.)
    out = trial.step(scene, [], path, .35, padding(scene), lambda *a: {'accepted': True, 'reason': None},
                     history=history())
    assert 'current_sweep_unobserved' in out['checks']['straight']['reasons']
    assert out['selected'] != 'straight' and trial.active
    trial.invalidate_release_history()
    out = trial.step(scene, [], path, .35, padding(scene), history=history())
    assert out['mode'] == 'HOLD_RELEASE_HISTORY'


def test_plan_horizon_grows_with_speed_and_age_and_must_fit_path():
    trial = active(ON, -.004)
    scene, path = timed_scene(.4, -.004)
    out = trial.step(scene, [], path, 2., padding(scene), history=history())
    assert out['checks']['straight']['reasons'] == ['straight_surface_horizon_unavailable']
    assert mx.committed_progress_m(.35, .2, GRASS) > mx.committed_progress_m(.35, .1, GRASS)
    assert mx.committed_progress_m(.5, .1, GRASS) > mx.committed_progress_m(.35, .1, GRASS)
