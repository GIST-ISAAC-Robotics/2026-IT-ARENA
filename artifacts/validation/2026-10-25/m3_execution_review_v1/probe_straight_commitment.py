"""한 저장 장면의 확약 범위 노면 반사실과 실제 명령 검사. 운영 연결 없음."""
import hashlib
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np
import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.follow_trace import restore_call, motion_scene
from arena_autonomy.lidar_motion import MotionConfig


def probe(source):
    header = event = None
    for line in source.open(encoding='utf8'):
        row = json.loads(line)
        if row['kind'] == 'header':
            header = row['data']
        elif row['kind'] == 'call' and row['data']['input']['reference_s'] >= 32.5:
            event = row['data']
            break
    assert header and event
    data, scan, history = restore_call(event['input'], MotionConfig(**header['motion_config']))
    scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                         epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
    detail = event['output'][2]
    ctl = mx.M3ExecutionFollow(mx.FollowConfig(**header['follow_config']),
                              mx.ExecutionConfig(**header['execution_config']), allow_simulation_trial=True)
    cfg = ctl.lateral_config
    frame = mx.ExecutionFrame(detail['path_points'])
    state = mx.ego_state(frame, cfg)
    candidate = mx.straight_candidate(frame, state, cfg)
    grid = mx.ReturnGrid(scene, cfg)
    corridor = mx.corridor_reference(frame, grid.returns, state['s0']-cfg.band_behind_m,
                                    state['s0']+cfg.horizon_m, cfg)
    padding = mx.per_ray_padding(scene, .025+ctl.config.measurement_padding_m,
                                 ctl.config.object_motion_bound_mps)
    context = {'frame': frame, 'grid': grid, 'corridor': corridor,
               'age': scene.t_ref_s-scene.source_first_s, 'follow_padding': padding}
    speed = max(data['speed_mps'], ctl.trial_speed_cap_mps)
    committed = mx.committed_progress_m(speed, context['age'], cfg)
    original_surface = mx.surface_check

    def prefix_surface(frame_, candidate_, cfg_, current_pose=None):
        # 다음 표본까지 포함해 확약 끝을 부족하게 자르지 않는다. 여유/곡선 판정은 원본 유지.
        count = min(len(candidate_['progress']), int(np.searchsorted(candidate_['progress'], committed))+1)
        short = dict(candidate_)
        for key in ('poses', 'steering_rad', 'progress'):
            short[key] = candidate_[key][:count]
        return original_surface(frame_, short, cfg_, current_pose)

    original = mx.plan_check(scene, [], candidate, context, cfg, ctl.execution, speed)
    assert original['reasons'] == detail['m3_lateral']['checks']['straight']['reasons']
    with patch.object(mx, 'surface_check', prefix_surface):
        limited = mx.plan_check(scene, [], candidate, context, cfg, ctl.execution, speed)
    baseline = (*detail['baseline_output'], detail)
    command = ctl._evaluate_command('straight', candidate['path'], scene, [], history,
                                   data['speed_mps'], data['previous_steering_rad'], data['dt_s'],
                                   padding, baseline)
    return {'scope': 'one saved scene; surface-prefix counterfactual plus command helper; no controller state replay or drive',
            'reference_s': data['reference_s'], 'committed_m': committed,
            'prior_steering_rad': data['previous_steering_rad'], 'original_plan': original,
            'prefix_plan': limited, 'command_helper': command,
            'history_scene_support': 'empty list deliberately conservative; captured motion history retained',
            'limitations': ['No fresh-count/release state transition or independent ROS protection was executed.',
                           'A positive helper result is not a published drive or a proof of later turn recovery.',
                           'No production horizon, margin, physics, or defaults were changed.'],
            'input_sha256': hashlib.sha256(source.read_bytes()).hexdigest()}


if __name__ == '__main__':
    source, destination = map(Path, sys.argv[1:])
    if destination.exists():
        raise FileExistsError(destination)
    result = probe(source)
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({'original': result['original_plan']['reasons'],
                      'prefix': result['prefix_plan']['reasons'],
                      'command_accepted': result['command_helper']['accepted'],
                      'command_reason': result['command_helper']['reason'],
                      'target': result['command_helper'].get('target')}))
