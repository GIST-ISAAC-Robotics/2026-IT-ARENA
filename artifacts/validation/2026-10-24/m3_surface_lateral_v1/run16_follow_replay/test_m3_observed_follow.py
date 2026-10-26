"""M3 시험 연결의 순수 경계 검사. 실제 구동/폐루프 안전 증명은 아니다."""
from dataclasses import replace
import math

import numpy as np
import pytest

from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.m3_observed_follow import M3ObservedFollow, command_arc, command_observation
from test_lateral_intent import straight, GRASS
from test_local_follow import history


def timed_scene(now, d0=0., **kwargs):
    s, path = straight(d0, **kwargs)
    delta = now-s.t_ref_s
    return replace(s, t_ref_s=now, source_first_s=s.source_first_s+delta,
                   source_last_s=s.source_last_s+delta,
                   rays=tuple(replace(r, acquired_at_s=r.acquired_at_s+delta) for r in s.rays)), path


def ctl():
    return M3ObservedFollow(allow_simulation_trial=True)


def force_candidate(controller, path):
    # 격리된 출력/관문 시험에서만 경로 선별을 고정한다. 모든 점유 검사는 실제 함수를 쓴다.
    controller.base.command = lambda *a, **k: (0., 0., {'reason': 'follow_gap_hold',
        'behavior': 'HOLD', 'path_points': path.tolist(), 'boundary_age_s': .1})
    controller.lateral.active = True
    def selected(*args):
        controller.lateral.active = True
        return {'mode': 'HOLD_TRIAL', 'path': path, 'selected': 'hold'}
    controller.lateral.step = selected


def test_explicit_selection_and_bounded_configuration():
    with pytest.raises(ValueError):
        M3ObservedFollow()
    for cfg in (FollowConfig(), FollowConfig(occupancy_timing_mode='per_ray_candidate', max_speed_mps=1.1)):
        with pytest.raises(ValueError):
            M3ObservedFollow(cfg, allow_simulation_trial=True)


def test_solo_real_pipeline_retains_baseline_outputs():
    follow, baseline = ctl(), LocalFollow(FollowConfig(occupancy_timing_mode='per_ray_candidate'))
    h = history()
    for now in (.4, .5, .6, .7, .8):
        s, _ = timed_scene(now)
        expected = baseline.command(s, h, 0., 'left')
        got = follow.command(s, h, 0., 'left')
        assert got[:2] == expected[:2]
        assert got[2]['baseline_reason'] == expected[2]['reason']
        assert got[2]['simulation_trial_only'] and got[2]['hidden_future_risk_unresolved']
    assert got[0] > 0 and not follow.lateral.active


@pytest.mark.parametrize('kind', ['front_unknown', 'stale', 'reverse', 'invalid', 'boundary_missing'])
def test_existing_gate_cannot_be_overridden(kind):
    follow, h = ctl(), history()
    for now in (.4, .5, .6, .7):
        follow.command(timed_scene(now)[0], h, 0., 'left')
    s, _ = timed_scene(.8)
    speed = 0.
    if kind == 'front_unknown':
        rays = list(s.rays)
        rays[250] = replace(rays[250], status='unknown', hit_xy_m=None)
        s = replace(s, rays=tuple(rays))
    if kind == 'stale': s = replace(s, t_ref_s=1.2)
    if kind == 'reverse': speed = -.01
    if kind == 'invalid': speed = float('nan')
    if kind == 'boundary_missing':
        follow.reset()
        follow.base.command = lambda *a, **k: (0., 0., {'reason': 'follow_boundary_unconfirmed', 'behavior': 'HOLD'})
    follow.lateral.step = lambda *a: pytest.fail('invalid baseline reached lateral planner')
    result = follow.command(s, h, speed, 'left')
    assert result[0] == 0 and not result[2]['m3_selected']


def test_actual_candidate_output_requires_two_distinct_scans_after_hold():
    follow, h = ctl(), history()
    _, path = timed_scene(.4)
    force_candidate(follow, path)
    for now in (.4, .42, .44, .46):
        # 같은 원본 스캔의 현재 시각만 진행시킨다.
        s = replace(timed_scene(.4)[0], t_ref_s=now)
        out = follow.command(s, h, 0., 'left')
        assert out[0] == 0
    assert out[2]['reason'] == 'm3_release_wait'
    out = follow.command(timed_scene(.5)[0], h, 0., 'left')
    assert 0 < out[0] <= .35 and out[2]['reason'] == 'm3_observed_trial'
    assert out[2]['m3_command_observation']['distance_m'] > .2


def test_side_occupancy_kept_after_selected_path():
    follow = ctl()
    s, path = timed_scene(.4)
    force_candidate(follow, path)
    # 가까운 정면 반환을 추가하되 Ray 원형/순서를 보존한다.
    rays = list(s.rays)
    rays[250] = replace(rays[250], hit_xy_m=(.13, 0.))
    s = replace(s, rays=tuple(rays))
    out = follow.command(s, history(), .1, 'left')
    assert out[0] == 0
    assert out[2]['unfiltered_occupancy'] and out[2]['occupied_points'] == len(s.occupied_endpoints_xy_m)
    assert out[2]['reason'] in ('m3_command_unobserved', 'm3_gap_hold')


def test_path_loss_does_not_steer_toward_center_while_braking():
    follow, s = ctl(), timed_scene(.4)[0]
    follow.base.command = lambda *a: (.2, -.05, {'reason': 'local_follow', 'behavior': 'FOLLOW', 'path_points': []})
    follow.lateral.step = lambda *a: {'mode': 'HOLD_PATH', 'path': None, 'selected': None}
    out = follow.command(s, history(), .35, 'left', previous_steering=.08)
    assert out[:2] == (0., .08)
    assert out[2]['reason'] == 'm3_candidate_unavailable'


def test_command_arc_starts_at_actual_body_and_rear_axle_geometry():
    angle, distance = .2, .3
    poses = command_arc(angle, distance, GRASS)
    np.testing.assert_allclose(poses[0], [0., 0., 0.], atol=1e-15)
    yaw = math.tan(angle)/GRASS.wheelbase_m*distance
    assert poses[-1, 2] == pytest.approx(yaw)
    rear = poses[-1, :2]-GRASS.wheelbase_m/2*np.array([math.cos(yaw), math.sin(yaw)])
    radius = GRASS.wheelbase_m/math.tan(angle)
    np.testing.assert_allclose(rear, [radius*math.sin(yaw)-GRASS.wheelbase_m/2, radius*(1-math.cos(yaw))])


def test_observation_checks_previous_angle_and_unknown_sector():
    s, _ = timed_scene(.4, dropout=lambda a: (a > 78.1) & (a < 78.7))
    out = command_observation(s, .35, 0., .3, GRASS)
    assert not out['supported'] and out['unsupported'] > 0
