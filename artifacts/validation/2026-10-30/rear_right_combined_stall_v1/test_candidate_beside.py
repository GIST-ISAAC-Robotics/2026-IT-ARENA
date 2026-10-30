"""오프라인 개입 후보의 국소 반례. 실제 swept_limit를 쓰고 광선별 팽창만 고정해 분기를 확인한다."""
import math
from types import SimpleNamespace

import numpy as np
import pytest

import arena_autonomy.measured_parallel_speed_trial as runtime
import arena_autonomy.pass_release_trial as ps
import candidate_speed as candidate
from arena_autonomy.local_follow import FollowConfig

CHAIN = dict(partial_observation_policy='bounded_local_v4', boundary_evidence_mode='corner_distance_candidate',
             occupancy_timing_mode='per_ray_candidate', parallel_hold_trial=True, pass_release_trial=True,
             guard_curve_trial=True, traffic_recovery_trial=True, traffic_path_recovery_trial=True,
             corridor_fallback_enabled=True, rear_lane_keep_trial=True, rear_lane_keep_policy='observed_measured_v1')
PATH = [[x/10, 0.] for x in range(11)]
FIXED = .025+.03


@pytest.fixture
def case(monkeypatch):
    state = dict(points=[], padding=[])
    for module in (runtime, candidate):
        monkeypatch.setattr(module, 'per_ray_padding', lambda *a: np.asarray(state['padding'], dtype=float))
    monkeypatch.setattr(ps, 'motion_region', lambda *a, **k: dict(distance_m=.6, queries=100, unobserved=0,
        masked_left=0, masked_right=0, masked_tail_extent_m=0., masked_front_samples=0))
    base = (1., 0., {'reason': 'local_follow', 'path_points': PATH, 'swept_clearance_m': 1.4,
                     'partial_observation': dict(allowed=True, reason='fully_observed')})

    def run(module, points, padding, speed=0.):
        state['points'], state['padding'] = points, padding
        scene = SimpleNamespace(t_ref_s=.1, source_first_s=0., source_epoch=0, occupied_endpoints_xy_m=points)
        return module.proposal(scene, base, speed, 0., .02, FollowConfig(**CHAIN))
    return run


def test_beside_age_only_overlap_no_longer_blocks(case):
    # 오른쪽 옆차 가까운 면 y=-0.2152(Gazebo 참값), 오래된 광선 팽창 0.146 → 기존은 거리 0 차단.
    points = [[x, -.2152] for x in (-.08, -.04, 0., .04, .08)]+[[1.5, 0.]]
    padding = [.146]*5+[.06]
    old = case(runtime, points, padding)
    new = case(candidate, points, padding)
    assert not old['eligible'] and old['reason'] == 'unchanged_gap_hold' and old['clearance_m'] == 0.
    assert new['eligible'] and new['beside_age_only_points'] == 5
    # 앞의 점(1.5 m)은 기존 팽창 그대로 여유를 정한다.
    assert new['clearance_m'] == pytest.approx(case(runtime, [[1.5, 0.]], [.06])['clearance_m'])


@pytest.mark.parametrize('y', [-.12, -.13])
def test_beside_point_inside_fixed_measurement_padding_still_blocks(case, y):
    new = case(candidate, [[0., y]], [.146])
    assert not new['eligible'] and new['clearance_m'] == 0. and new['beside_age_only_points'] == 0


@pytest.mark.parametrize('x', [.3, .6, 1.2])
def test_points_ahead_in_path_keep_full_age_padding(case, x):
    points, padding = [[x, -.10]], [.146]
    assert case(candidate, points, padding)['clearance_m'] == case(runtime, points, padding)['clearance_m']
    assert case(candidate, points, padding)['beside_age_only_points'] == 0


def test_ahead_beside_edge_with_age_padding_still_limits_sweep(case):
    # 차체 앞 모서리보다 앞(|x| > 길이/2+팽창)인 옆 점은 분리 대상이 아니다.
    points, padding = [[.30, -.20]], [.146]
    new, old = case(candidate, points, padding), case(runtime, points, padding)
    assert new['beside_age_only_points'] == 0 and new['clearance_m'] == old['clearance_m']


def test_no_beside_points_output_identical_except_count(case):
    points, padding = [[1.4, 0.], [.9, .30], [.5, -.33]], [.06, .1, .12]
    new, old = case(candidate, points, padding), case(runtime, points, padding)
    assert new.pop('beside_age_only_points') == 0
    assert new == old


def test_measured_overspeed_still_needs_own_region_and_stop_space(case):
    points = [[x, -.2152] for x in (-.08, 0., .08)]+[[.45, 0.]]
    result = case(candidate, points, [.146]*3+[.06], speed=1.05)
    assert not result['eligible'] and result['reason'] in ('measured_stop_space_insufficient', 'unchanged_gap_hold')
