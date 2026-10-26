"""G2 평가기의 연속 이동 상계·바퀴 정책 반례. 경로 실행을 승인하지 않는다."""
from dataclasses import replace
import numpy as np
import pytest

from arena_autonomy.lateral_intent import LateralIntentConfig
from g0_geometry import TruthBox
from g2_geometry_probe import ProbeConfig, sample_path
from g2_surface_geometry import assess, geometry_at, interval_bounds, footprint_at
from surface_contact_eval import VehicleGeometry, steering_footprint, rectangle


@pytest.mark.parametrize('offset', [-.24, -.16, 0., .16, .24])
@pytest.mark.parametrize('transition', [.6, 1.2])
def test_interval_bound_covers_each_steered_wheel_vertex(offset, transition):
    cfg = LateralIntentConfig(surface_policy='two_wheel_grass')
    path = sample_path(offset, transition, ProbeConfig(step_m=.02))
    bounds = interval_bounds(path, cfg)
    previous = None
    for i, row in enumerate(path['samples']):
        _, wheels = geometry_at(row, cfg)
        if previous:
            for name in wheels:
                for shape, bound_key in (('patch', 'patch_m'), ('outline', 'footprint_m')):
                    old = np.asarray(previous[name][shape].exterior.coords)
                    new = np.asarray(wheels[name][shape].exterior.coords)
                    assert np.max(np.linalg.norm(new-old, axis=1)) <= bounds[bound_key][i-1]+1e-12
        previous = wheels


def test_center_blocker_remains_infeasible_with_current_error_budget():
    cfg = LateralIntentConfig(surface_policy='two_wheel_grass')
    for offset in (-.16, -.20, -.24):
        r = assess(sample_path(offset, 1.2, ProbeConfig(step_m=.01)), TruthBox(1.5, 0., .12), cfg)
        assert not r['geometry_conditions_met']
        assert not r['driving_authorized']
        assert not r['observation_verified'] and not r['tracking_verified']


def test_mirror_keeps_clearance_and_inner_wheels():
    a = assess(sample_path(-.16, 1.2, ProbeConfig(step_m=.01)), TruthBox(1.5, .15, .12))
    b = assess(sample_path(.16, 1.2, ProbeConfig(step_m=.01)), TruthBox(1.5, -.15, .12))
    for key in ('minimum_gap_lower_m', 'minimum_road_edge_lower_m', 'minimum_wall_edge_lower_m'):
        assert a[key] == pytest.approx(b[key], abs=1e-12)
    assert a['checks'] == b['checks']
    assert a['inner_pair_confirmed']['left'] == b['inner_pair_confirmed']['right']


def test_low_target_geometry_is_not_sensor_evidence():
    path = sample_path(-.16, 1.2, ProbeConfig(step_m=.01))
    assert assess(path, TruthBox(1.5, .15, .06)) == assess(path, TruthBox(1.5, .15, .12))


def test_no_static_discount_and_no_budget_relaxation():
    cfg = LateralIntentConfig(surface_policy='two_wheel_grass')
    path = sample_path(-.16, 1.2, ProbeConfig(step_m=.01))
    result = assess(path, TruthBox(1.5, .15, .12), cfg)
    assert result['required_gap_m'] == cfg.budget.lateral_clearance(True, result['curved_budget'])[0]
    tighter = replace(cfg, budget=replace(cfg.budget, range_bias_m=.06))
    other = assess(path, TruthBox(1.5, .15, .12), tighter)
    for key in ('body_and_wheel_gap', 'wall_margin', 'two_wheel_surface'):
        assert not other['checks'][key] or result['checks'][key]


def test_smaller_sampling_step_does_not_hide_coarse_bound():
    target = TruthBox(1.5, .15, .12)
    coarse = assess(sample_path(-.16, 1.2, ProbeConfig(step_m=.02)), target)
    fine = assess(sample_path(-.16, 1.2, ProbeConfig(step_m=.001)), target)
    assert fine['max_footprint_interval_bound_m'] < coarse['max_footprint_interval_bound_m']
    assert coarse['minimum_gap_lower_m'] <= fine['minimum_gap_lower_m']+1e-10
    assert coarse['minimum_road_edge_lower_m'] <= fine['minimum_road_edge_lower_m']+1e-10


@pytest.mark.parametrize('steer', [-.37, -.1, 0., .1, .37])
def test_component_distance_and_extent_match_union_without_union_construction(steer):
    geometry = VehicleGeometry()
    parts, _ = footprint_at(.2, -.1, .3, steer, geometry)
    union = steering_footprint(.2, -.1, .3, steer, geometry)
    assert parts.bounds == pytest.approx(union.bounds, abs=1e-14)
    for x,y in ((.2,-.1),(.5,.2),(-.5,0.),(.1,.1)):
        obstacle=rectangle(x,y,-.1,.2,.15)
        assert parts.distance(obstacle) == pytest.approx(union.distance(obstacle), abs=1e-14)
