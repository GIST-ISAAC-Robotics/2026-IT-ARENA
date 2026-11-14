"""현재 위치 유지 후보의 운동·점유·새 관측 재개·입력 실패 경계."""
from dataclasses import replace
import json
import math

import numpy as np
import pytest

from arena_autonomy.follow_trace import FollowTrace, replay_events
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.parallel_lane_trial import ParallelLaneTrial, lateral_excursion, proposal
from arena_autonomy.scene_motion import motion_scene
from test_g1_partial_policy import corridor, call
from test_local_follow import history


def config(**kw):
    return FollowConfig(**({'partial_observation_policy': 'bounded_local_v4', 'parallel_hold_trial': True,
                           'boundary_evidence_mode': 'corner_distance_candidate',
                           'occupancy_timing_mode': 'per_ray_candidate'} | kw))


def base(offset=.11, slope=0.):
    x = np.linspace(-.1, 1.2, 40)
    return (0., .06, {'reason': 'follow_gap_hold', 'path_points': np.c_[x, slope*x+offset].tolist(),
                     'partial_observation': {'allowed': True, 'reason': 'fully_observed'}})


def scene(first=.5, obstacle=None):
    return motion_scene(corridor(first, obstacle=obstacle), history(), first+.1, timing_verified=True)


@pytest.mark.parametrize('offset', [-.11, .11])
def test_parallel_keeps_current_position_and_all_occupancy(offset):
    s = scene(); before = np.array(s.occupied_endpoints_xy_m)
    r = proposal(s, base(offset), 0., 0., .02, config())
    assert r['eligible'] and r['target_speed_mps'] == .35
    assert abs(r['steering_rad']) < 1e-12 and r['maximum_body_corner_lateral_m'] < 1e-12
    assert not r['unknown_filled'] and r['hidden_rear_risk_unresolved']
    np.testing.assert_array_equal(s.occupied_endpoints_xy_m, before)


@pytest.mark.parametrize('obstacle', ['front', 'side'])
def test_current_obstacle_is_not_erased_by_parallel_reference(obstacle):
    raw = corridor(.5, obstacle=.25 if obstacle == 'front' else None)
    if obstacle == 'side': raw.ranges[375] = .12
    s = motion_scene(raw, history(), .6, timing_verified=True)
    r = proposal(s, base(), 0., 0., .02, config())
    assert not r['eligible'] and r['reason'] == 'unchanged_gap_hold'
    assert r['unfiltered_occupancy'] and r['occupied_points'] == len(s.occupied_endpoints_xy_m)


@pytest.mark.parametrize('speed,previous', [(1., .2), (.35, .37)])
def test_present_motion_and_previous_steering_still_bound_lateral_motion(speed, previous):
    r = proposal(scene(), base(), speed, previous, .02, config())
    assert not r['eligible'] and r['reason'] == 'lateral_motion_not_small'


def test_analytic_lateral_extrema_include_dense_samples_both_directions():
    cfg = config()
    for speed in (.1, .35, 1.):
        for angle in (-.37, -.03, 0., .2, .37):
            limit = lateral_excursion(speed, .18, angle, -angle, .02, cfg)
            distance = speed*(.18+.1+.02)+speed*speed/4
            s = np.linspace(0., distance, 10001)
            observed = 0.
            for a in (angle, -angle):
                k = math.tan(a)/cfg.wheelbase_m
                yaw = s*k
                center = (1-np.cos(yaw))/k+cfg.wheelbase_m/2*np.sin(yaw) if k else s*0.
                for x in (-.1, .1):
                    for y in (-.075, .075):
                        observed = max(observed, float(np.max(abs(center+x*np.sin(yaw)+y*np.cos(yaw)-y))))
            assert limit >= observed-1e-12 and limit <= observed+1e-7


@pytest.mark.parametrize('fault', ['curve', 'missing_path', 'missing_scan', 'reverse', 'nan', 'partial', 'denied'])
def test_invalid_or_unobserved_case_never_uses_positive_baseline_as_fallback(fault):
    b = list(base()); b[0] = 1.; speed = 0.
    if fault == 'curve': b = list(base(slope=math.tan(math.radians(8))))
    elif fault == 'missing_path': b[2]['path_points'] = []
    elif fault == 'missing_scan': b[2]['reason'] = 'follow_partial_scan_missing'
    elif fault == 'reverse': speed = -.1
    elif fault == 'nan': speed = math.nan
    elif fault == 'partial': b[2]['partial_observation']['reason'] = 'side_missing'
    else: b[2]['partial_observation']['allowed'] = False
    ctl = ParallelLaneTrial(config())
    out = ctl.apply(scene(), b, speed, .01, .02)
    assert out[0] == 0 and out[1] == .01 and out[2]['behavior'] == 'HOLD'


def test_release_needs_two_distinct_fresh_scans_after_every_hold():
    ctl = ParallelLaneTrial(config())
    def step(first, obstacle=None):
        return ctl.apply(scene(first, obstacle), base(), 0., 0., .02)
    assert step(.4)[0] == 0
    assert step(.4)[0] == 0
    assert step(.5)[0] > 0
    assert step(.6, .25)[0] == 0
    assert step(.7)[0] == 0
    assert step(.7)[0] == 0
    assert step(.8)[0] > 0
    assert step(.5)[0] == 0  # 시각 역행은 새 확인 필요
    ctl.reset()
    assert step(.6)[0] == 0


def test_release_does_not_count_clearance_below_original_resume_threshold():
    ctl = ParallelLaneTrial(config())
    assert ctl.apply(scene(.4, .42), base(), 0., 0., .02)[0] == 0
    assert ctl.apply(scene(.5, .42), base(), 0., 0., .02)[0] == 0


@pytest.mark.parametrize('kw', [{'parallel_hold_trial': 1}, {'partial_observation_policy': 'strict'},
    {'occupancy_timing_mode': 'oldest'}, {'boundary_evidence_mode': 'connected'}])
def test_trial_configuration_is_explicit_and_bounded(kw):
    with pytest.raises(ValueError): config(**kw)
    assert not FollowConfig().parallel_hold_trial


def test_runtime_trace_roundtrip_includes_state_and_keeps_front_stop(tmp_path):
    h = history(); ctl = LocalFollow(config()); path = tmp_path/'parallel.jsonl'
    trace = FollowTrace(str(path), h.config, ctl.config)
    outputs = []
    for first in (.3, .4, .5, .6, .7, .8, .9):
        raw = corridor(first, obstacle=.25 if first >= .8 else None)
        out = call(ctl, raw, h)
        outputs.append(out)
        trace.record_call(raw, h, first+.1, 0, 0., 'left', 0., .02, True, 30., out)
    trace.close()
    assert any(o[0] > 0 and o[2]['parallel_hold_selected'] for o in outputs)
    assert all(o[0] <= .35 for o in outputs) and outputs[-1][0] == 0
    result = replay_events(json.loads(line) for line in path.read_text().splitlines())
    assert result['passed'] and result['calls'] == 7
