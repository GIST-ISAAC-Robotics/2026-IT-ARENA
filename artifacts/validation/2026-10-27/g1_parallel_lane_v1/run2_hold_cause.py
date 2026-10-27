"""평행 경로 간격 대기의 원인 분해(저장 판단 재구성, 읽기 전용).

`parallel_hold_unchanged_gap_hold`와 정지 판단마다 같은 장면·점별 여유로 `swept_limit`를 다시 계산하고, 점을 자차 옆면 밖
(|y|>폭/2)과 그 안으로 나누어 어느 쪽이 첫 간섭을 만들었는지 센다. 사후 정답(평가 전용)으로 그 시각 옆차의 종방향 위치를
붙인다. 제어 출력·입력을 바꾸지 않으며 폐루프 결론이 아니다.
"""
from bisect import bisect_right
from collections import Counter
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts'):
    sys.path.insert(0, str(ROOT/part))

import numpy as np  # noqa: E402
from arena_autonomy.follow_trace import restore_call  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig  # noqa: E402
from arena_autonomy.local_follow import FollowConfig  # noqa: E402
from arena_autonomy.local_path import swept_limit  # noqa: E402
from arena_autonomy.occupancy_timing import per_ray_padding  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402


def poses(path):
    return [json.loads(line) for line in path.open(encoding='utf-8')]


def relative(ego, actor, t):
    e = ego[max(0, bisect_right([r[0] for r in ego], t)-1)]
    a = actor[max(0, bisect_right([r[0] for r in actor], t)-1)]
    dx, dy = a[1]-e[1], a[2]-e[2]
    c, s = math.cos(e[3]), math.sin(e[3])
    return c*dx+s*dy, -s*dx+c*dy


def analyse(run_dir):
    lines = run_dir.joinpath('follow_decisions.jsonl').read_text(encoding='utf-8').splitlines()
    head = json.loads(lines[0])['data']
    motion, cfg = MotionConfig(**head['motion_config']), FollowConfig(**head['follow_config'])
    ego = poses(run_dir/'ego_poses.jsonl')
    side = json.loads((run_dir/'additional_actors.json').read_text(encoding='utf-8'))['poses']['eval_e3_side']
    rows, counts = [], Counter()
    for line in lines[1:-1]:
        event = json.loads(line)
        if event['kind'] != 'call':
            continue
        detail = event['data']['output'][2]
        if detail.get('reason') != 'parallel_hold_unchanged_gap_hold':
            continue
        data, scan, history = restore_call(event['data']['input'], motion)
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        returns = [i for i, r in enumerate(scene.rays) if r.status == 'return']
        points = np.asarray([scene.rays[i].hit_xy_m for i in returns], dtype=float)
        pad = per_ray_padding(scene, .025+cfg.measurement_padding_m, cfg.object_motion_bound_mps)
        trial = detail['parallel_hold_trial']
        steer, previous = trial['steering_rad'], data['previous_steering_rad']
        outside = np.abs(points[:, 1]) > cfg.width_m/2
        def limit(mask):
            if not np.any(mask):
                return 2.5
            return min(swept_limit(points[mask], a, cfg.wheelbase_m, horizon=2.5, margin=pad[mask],
                                   length=cfg.length_m, width=cfg.width_m) for a in (steer, previous))
        side_clear, lane_clear = limit(outside), limit(~outside)
        cause = ('side_returns' if side_clear <= cfg.hold_clearance_m < lane_clear else
                 'lane_returns' if lane_clear <= cfg.hold_clearance_m < side_clear else
                 'both' if max(side_clear, lane_clear) <= cfg.hold_clearance_m else 'neither_recomputed')
        counts[cause] += 1
        ages = np.asarray([scene.t_ref_s-scene.rays[i].acquired_at_s for i in returns])[outside]
        fx, fy = relative(ego, side, data['reference_s'])
        rows.append({'t': data['reference_s'], 'recorded_clearance_m': trial.get('clearance_m'),
                     'side_only_clearance_m': side_clear, 'lane_only_clearance_m': lane_clear, 'cause': cause,
                     'side_point_age_s': [float(np.min(ages)), float(np.max(ages))] if len(ages) else None,
                     'side_actor_forward_m': fx, 'side_actor_left_m': fy})
    return {'gap_hold_rows': len(rows), 'causes': dict(counts),
            'time_range_s': [rows[0]['t'], rows[-1]['t']] if rows else None,
            'side_actor_forward_range_m': [min(r['side_actor_forward_m'] for r in rows),
                                           max(r['side_actor_forward_m'] for r in rows)] if rows else None,
            'rows': rows}


out = {'scope': 'stored-input reconstruction of parallel-hold gap stops; truth only post hoc; not closed loop',
       'left': analyse(HERE/'run1_left'), 'right': analyse(HERE/'run2_right')}
(HERE/'run2_hold_cause.json').write_text(json.dumps(out, ensure_ascii=False, indent=1)+'\n', encoding='utf-8',
                                         newline='\n')
for side in ('left', 'right'):
    print(side, {k: v for k, v in out[side].items() if k != 'rows'})
