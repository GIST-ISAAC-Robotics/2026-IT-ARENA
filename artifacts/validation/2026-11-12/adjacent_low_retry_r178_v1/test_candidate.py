import copy
import json
import math
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace as NS
import pytest

import candidate
from arena_autonomy import current_path_recheck as current_check
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, forward_planning_speed
from arena_autonomy.pass_release_trial import SelectorConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.scan_input import scan_angles

A = Path(__file__).resolve().parent


@pytest.fixture
def example():
    trace = A.parent/'released_parallel_runtime_r174_v1/two_actor_run1/run1/follow_decisions.jsonl'
    out = next(r['output'] for r in map(json.loads, (A/'expected.jsonl').open()) if r['seq'] == 593)
    with trace.open() as stream:
        header = json.loads(next(stream))['data']
        old = next(r['data'] for r in map(json.loads, stream) if r['seq'] == 593)
    now = copy.deepcopy(old['input'])
    _, scan, history = restore_call(now, MotionConfig(**header['motion_config']))
    cfg = FollowConfig(**header['follow_config'])
    scene = motion_scene(scan, history, now['reference_s'], timing_verified=True, epoch=now['epoch'],
                         rear_half_angle_deg=now['rear_half_angle_deg'])
    owner = NS(mode='GUARD', follow_config=cfg, config=SelectorConfig(), sides=out[2]['normal']['selector_sides'])
    baseline = [.6, out[1], out[2]['normal']]
    return owner, scene, baseline, old, out, now, scan, history, cfg


def test_retry_only_calls_existing_full_guard_and_never_reuses_old_permission(example, monkeypatch):
    owner, scene, base, old, out, now, scan, history, cfg = example
    original = old['output'][:2]+[old['output'][2]['normal']]
    planning_speed = forward_planning_speed(cfg, now['speed_mps'])
    assert candidate.eligible(owner, scene, base, planning_speed, original)
    monkeypatch.setattr(candidate, '_old_guard', lambda *a:original)
    calls = []
    def fail(*args, **kwargs):
        calls.append(kwargs['target_cap']); return original
    owner._guard_curve_at_cap = fail
    result = candidate.guard(owner, scene, None, base, None, planning_speed, now['previous_steering_rad'],
                             now['dt_s'], .117, {}, scan)
    assert calls == [.15] and result[:2] == tuple(old['output'][:2])
    assert not result[2]['selector_adjacent_low_retry']['applied']


@pytest.mark.parametrize('kind', ['fast_actual','present_side','memory_path','other_partial','already_checked','other_rejection'])
def test_no_new_scope_or_duplicate_probe(example, kind):
    owner, scene, base, old, out, now, scan, history, cfg = example
    speed = forward_planning_speed(cfg, now['speed_mps'])
    if kind == 'fast_actual': speed = .151
    elif kind == 'present_side': owner.sides['left']['present'] = True
    elif kind == 'memory_path': base[2]['boundary_from_memory'] = True
    elif kind == 'other_partial': base[2]['partial_observation']['reason'] = 'unbounded_missing'
    elif kind == 'already_checked': old['output'][2]['normal']['selector_horizon_recheck'] = {'attempts':[{'target_mps':.15}]}
    else: old['output'][2]['normal']['selector_reason'] = 'large_motion_masked_toward_left'
    assert not candidate.eligible(owner, scene, base, speed, old['output'][0:2]+[old['output'][2]['normal']])


@pytest.mark.parametrize('kind', ['clear','near_front','near_side','missing_front'])
def test_new_low_output_still_requires_current_obstacle_checks(example, kind):
    owner, scene, base, old, out, now, scan, history, cfg = example
    if kind != 'clear':
        values = list(scan.ranges_m)
        angles = scan_angles(scan)+history.config.lidar_yaw_rad
        if kind == 'near_side':
            for i,a in enumerate(angles):
                a = math.remainder(a, math.tau)
                if math.radians(70)<a<math.radians(130):values[i]=.10/math.sin(a)
        else:
            i = min(range(len(angles)), key=lambda i:abs(math.remainder(angles[i],math.tau)))
            values[i] = .05 if kind == 'near_front' else math.inf
        scan = replace(scan, ranges_m=tuple(values)); now['scan']['measurements']['ranges_m'] = values
        scene = motion_scene(scan, history, now['reference_s'], timing_verified=True, epoch=now['epoch'],
                             rear_half_angle_deg=now['rear_half_angle_deg'])
    result = current_check.rebase(out, old['input'], now, scene, history, cfg)
    assert result is not None
    if kind == 'clear': assert 0 < result['speed_mps'] <= .15
    else: assert result['speed_mps'] == 0.
    assert result['independent_guard_required']


def test_absent_guard_forward_policy_keeps_signed_history_and_legacy_hold(example):
    owner, scene, base, old, out, now, scan, history, cfg = example
    assert now['speed_mps'] < 0
    before = copy.deepcopy(now)
    result = current_check.rebase(out, old['input'], now, scene, history, cfg)
    assert 0 < result['speed_mps'] <= .15 and now == before
    legacy = replace(cfg, signed_motion_policy='legacy_hold')
    assert current_check.rebase(out, old['input'], now, scene, history, legacy)['speed_mps'] == 0.
    now['speed_mps'] = -.351
    assert current_check.rebase(out, old['input'], now, scene, history, cfg)['speed_mps'] == 0.
