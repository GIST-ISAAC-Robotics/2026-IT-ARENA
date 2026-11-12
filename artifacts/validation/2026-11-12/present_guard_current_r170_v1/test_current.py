import copy
from dataclasses import replace
import json
import math
from pathlib import Path
import pytest
import current as candidate
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.scan_input import scan_angles

A = Path(__file__).resolve().parent


@pytest.fixture
def sample():
    trace = A.parent/'released_runtime_r168_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with trace.open() as f:
        header = json.loads(next(f))['data']
        item = next(row['data'] for row in map(json.loads, f)
                    if row['kind'] == 'call' and candidate.supported(row['data']['output'])
                    and row['data']['input']['speed_mps'] > 1.
                    and row['data']['output'][2]['normal']['selector_guard_candidate'].get('current_approach', {}).get('accepted'))
    old = item['input']; now = copy.deepcopy(old)
    _, scan, history = restore_call(now, MotionConfig(**header['motion_config']))
    scene = motion_scene(scan, history, now['reference_s'], timing_verified=True, epoch=now['epoch'],
                         rear_half_angle_deg=now['rear_half_angle_deg'])
    return item['output'], old, now, scan, scene, history, FollowConfig(**header['follow_config'])


def call(sample):
    out, old, now, scan, scene, history, cfg = sample
    return candidate.rebase(out, old, now, scene, history, cfg)


def test_actual_speed_not_target_used_while_braking(sample):
    before = copy.deepcopy(sample[:3])
    result = call(sample)
    assert result['reason'] == 'rebase_current_present_guard'
    assert 0 < result['speed_mps'] <= .35 < sample[2]['speed_mps']
    assert result['prediction_speed_mps'] == sample[2]['speed_mps']
    assert result['current_approach']['accepted']
    assert result['path_source_s'] == sample[0][2]['normal']['path_source_s']
    assert not result['state_transition'] and not result['source_permission_reused']
    assert sample[:3] == before


@pytest.mark.parametrize('kind', ['epoch', 'sensor_epoch', 'side', 'steering', 'path_age', 'missing_anchor', 'target', 'invalid_speed'])
def test_binding_and_limits(sample, kind):
    out, old, now, scan, scene, history, cfg = sample
    if kind == 'epoch': now['epoch'] += 1
    elif kind == 'sensor_epoch': now['scan']['measurements']['source_epoch'] += 1
    elif kind == 'side': now['side'] = 'right' if now['side'] == 'left' else 'left'
    elif kind == 'steering': now['previous_steering_rad'] += .1
    elif kind == 'path_age': out[2]['normal']['path_source_s'] -= 1.
    elif kind == 'missing_anchor': out[2]['normal']['selector_guard_candidate']['guarded_sides'] = {}
    elif kind == 'target': out[0] = .36
    else: now['speed_mps'] = float('nan')
    assert call(sample)['speed_mps'] == 0


@pytest.mark.parametrize('kind', ['close_front', 'missing_front', 'close_side'])
def test_current_obstacle_or_missing_data_overrides_previous_permission(sample, kind):
    out, old, now, scan, scene, history, cfg = sample
    angles = scan_angles(scan)+history.config.lidar_yaw_rad
    values = list(scan.ranges_m)
    if kind == 'close_side':
        for i,a in enumerate(angles):
            a = math.remainder(a, math.tau)
            if math.radians(70) < a < math.radians(130): values[i] = .10/math.sin(a)
    else:
        i = min(range(len(angles)), key=lambda i: abs(math.remainder(angles[i], math.tau)))
        values[i] = math.inf if kind == 'missing_front' else .05
    scan = replace(scan, ranges_m=tuple(values))
    scene = motion_scene(scan, history, now['reference_s'], timing_verified=True, epoch=now['epoch'],
                         rear_half_angle_deg=now['rear_half_angle_deg'])
    now['scan']['measurements']['ranges_m'] = values
    assert candidate.rebase(out, old, now, scene, history, cfg)['speed_mps'] == 0


def test_missing_guarded_side_not_relabelled_solo(sample):
    out, old, now, scan, scene, history, cfg = sample
    out[2]['normal']['selector_sides']['left']['seen'] = False
    out[2]['normal']['selector_sides']['left']['present'] = True
    assert not candidate.supported(out)
    assert call(sample) is None


def test_disable_keeps_candidate_out(sample):
    out, old, now, scan, scene, history, cfg = sample
    disabled = replace(cfg, traffic_recovery_trial=False, traffic_path_recovery_trial=False,
                       released_rear_sweep_trial=False, current_lane_preference_trial=False)
    assert candidate.rebase(out, old, now, scene, history, disabled)['speed_mps'] == 0


def test_signed_forward_policy_preserves_history_and_allows_negative_feedback(sample):
    out, old, now, scan, scene, history, cfg = sample
    now['speed_mps'] = -.01
    before = copy.deepcopy(now)
    result = call(sample)
    assert result['speed_mps'] > 0
    assert result['measured_speed_mps'] == -.01 and result['forward_planning_speed_mps'] == .01
    assert result['signed_history_preserved'] and now == before
    legacy = replace(cfg, signed_motion_policy='legacy_hold')
    assert candidate.rebase(out, old, now, scene, history, legacy)['speed_mps'] == 0
