"""run32 정지 자세 1판단의 실제 계획 질의 위치. 제어기 재주행/조건 변경 없이 센서 좌표로 분해한다."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from arena_autonomy.follow_trace import restore_call, motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.observation_support import depth_support
import arena_autonomy.m3_execution_trial as mx


def audit(path, target):
    header = event = None
    for line in path.open(encoding='utf8'):
        item = json.loads(line)
        if item['kind'] == 'header':
            header = item['data']
        if item['kind'] == 'call' and item['data']['input']['reference_s'] >= target:
            event = item['data']
            break
    data, scan, history = restore_call(event['input'], MotionConfig(**header['motion_config']))
    scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                         epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
    detail = event['output'][2]
    frame = mx.ExecutionFrame(detail['path_points'])
    execution = mx.ExecutionConfig(**header['execution_config'])
    ctl = mx.M3ExecutionFollow(mx.FollowConfig(**header['follow_config']), execution,
                               allow_simulation_trial=True)
    cfg = ctl.lateral_config
    state = mx.ego_state(frame, cfg)
    pad = mx.ray_padding(scene, cfg, execution.history_motion_bound_mps)
    output = {}
    for name, target_d in (('center', 0.), ('hold', state['d0'])):
        candidate = mx.anchored(mx.offset_candidate(frame, state, target_d, cfg, name))
        registered = detail['m3_lateral']['checks'][name]
        keep = candidate['progress'] <= registered['required_progress_m']+mx.GEOMETRY_EPS_M
        queries, thin = mx.body_queries(scene, candidate['poses'][keep], candidate['steering_rad'][keep], cfg)
        missing = queries[~depth_support(scene, queries, pad)]
        masked = mx.masked_only(scene, missing, pad)
        # 현재 차체 직사각형 밖으로의 최소 거리. 작은 값은 면제 근거가 아니다.
        outer = np.maximum(np.abs(missing)-[cfg.length_m/2, cfg.width_m/2], 0.)
        dist = np.linalg.norm(outer, axis=1)
        count_matches = len(missing) == registered['observation']['current_unsupported']
        output[name] = {
            'query_count': len(queries), 'recorded_queries': registered['observation']['queries'],
            'unsupported': len(missing), 'recorded_unsupported': registered['observation']['current_unsupported'],
            'matches_recorded_count': count_matches, 'masked_only_count': int(masked.sum()),
            'unsupported_xy_m': missing.tolist(), 'distance_outside_current_body_m': dist.tolist(),
            'required_progress_m': registered['required_progress_m'],
            'max_distance_outside_current_body_m': float(np.max(dist)) if len(dist) else None,
            'original_observation': registered['observation'], 'unknown_sector_intersections': thin,
        }
        assert count_matches and len(queries) == registered['observation']['queries']
    return {'scope': 'one stationary captured sensor scene; tiny unobserved protrusion is not free-space evidence',
            'reference_s': data['reference_s'], 'measured_speed_mps': data['speed_mps'],
            'rear_mask_half_angle_deg': data['rear_half_angle_deg'], 'state': state, 'candidates': output,
            'input_sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


if __name__ == '__main__':
    source, destination = map(Path, sys.argv[1:])
    if destination.exists():
        raise FileExistsError(destination)
    result = audit(source, 32.5)
    destination.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({name: {'unsupported': row['unsupported'], 'masked': row['masked_only_count'],
                             'max_outside_m': row['max_distance_outside_current_body_m']}
                      for name, row in result['candidates'].items()}))
