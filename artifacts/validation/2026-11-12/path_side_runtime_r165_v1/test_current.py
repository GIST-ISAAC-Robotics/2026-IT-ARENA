import copy
import importlib.util
import json
import math
from pathlib import Path
from dataclasses import replace
import pytest
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.scan_input import scan_angles

A = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('r165_rebase_test', A/'rebase.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


@pytest.fixture
def sample():
    trace = A.parent/'adjacent_live_r152_v1/two_actor_run2/run1/follow_decisions.jsonl'
    with trace.open() as stream:
        header = json.loads(next(stream))['data']
        event = next(json.loads(line) for line in stream if json.loads(line)['seq']==632)
    with (A.parent/'path_side_gap_r164_v1/expected.jsonl').open() as stream:
        output = next(row['output'] for row in map(json.loads, stream) if row['seq']==632)
    submitted = event['data']['input']
    current = copy.deepcopy(submitted)
    data, scan, history = restore_call(current, MotionConfig(**header['motion_config']))
    scene = motion_scene(scan,history,data['reference_s'],timing_verified=True,epoch=data['epoch'],
                         rear_half_angle_deg=data['rear_half_angle_deg'])
    return output,submitted,current,scan,scene,history,FollowConfig(**header['follow_config'])


def test_current_guard_recalculates_same_small_gap(sample):
    out, old, now, scan, scene, history, cfg = sample
    r = m.rebase(out,old,now,scene,history,cfg)
    assert r['speed_mps'] == .15 and r['body_side_current_rechecked']
    assert r['current_gap_proof']['current_history_check']['accepted']
    assert not r['source_permission_reused'] and r['independent_guard_required']


@pytest.mark.parametrize('field', ['epoch', 'side', 'steering', 'path_age'])
def test_binding_and_path_expiration_are_not_relaxed(sample, field):
    out, old, now, scan, scene, history, cfg = sample
    if field == 'epoch': now['epoch'] += 1
    elif field == 'side': now['side'] = 'right' if now['side']=='left' else 'left'
    elif field == 'steering': now['previous_steering_rad'] += .1
    else: out[2]['normal']['path_source_s'] -= 1
    r = m.rebase(out,old,now,scene,history,cfg)
    assert r['speed_mps'] == 0


def test_current_new_close_return_blocks(sample):
    out, old, now, scan, scene, history, cfg = sample
    angles = scan_angles(scan)+history.config.lidar_yaw_rad
    index = min(range(len(angles)),key=lambda i:abs(math.atan2(math.sin(angles[i]),math.cos(angles[i]))))
    values = list(scan.ranges_m); values[index] = .05
    scan = replace(scan,ranges_m=tuple(values))
    scene = motion_scene(scan,history,now['reference_s'],timing_verified=True,epoch=now['epoch'],
                         rear_half_angle_deg=now['rear_half_angle_deg'])
    r = m.guard.rebase(out,old,now,scan,scene,history,cfg)
    assert r['speed_mps'] == 0


def test_original_side_present_never_becomes_permission(sample):
    out, old, now, scan, scene, history, cfg = sample
    out[2]['normal']['selector_sides']['left']['present'] = True
    assert not m.supported(out)
    assert m.rebase(out,old,now,scene,history,cfg) is None


def test_an_additional_current_front_gap_is_not_covered(sample):
    out, old, now, scan, scene, history, cfg = sample
    index = min(range(len(scene.rays)),key=lambda i:abs(math.atan2(scene.rays[i].direction_xy[1],scene.rays[i].direction_xy[0])))
    values = list(scan.ranges_m); values[index] = math.inf
    scan = replace(scan,ranges_m=tuple(values))
    scene = motion_scene(scan,history,now['reference_s'],timing_verified=True,epoch=now['epoch'],
                         rear_half_angle_deg=now['rear_half_angle_deg'])
    r = m.guard.rebase(out,old,now,scan,scene,history,cfg)
    assert r['speed_mps'] == 0
