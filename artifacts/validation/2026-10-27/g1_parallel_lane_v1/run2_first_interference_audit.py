"""간격 대기에서 '처음 간섭한 반환'만 격리해 사후 정답 actor 상자·벽과 대조한다(평가 전용, 읽기 전용).

`run2_hold_cause.py`의 |y|>폭/2 분류는 옆차뿐 아니라 벽 반환도 포함한다. 여기서는 `swept_limit`과 같은 이산 진행에서
첫 간섭 거리의 간섭 반환을 모으고, 각 반환을 (1) 그 광선 취득 시각의 옆차 상자까지 거리, (2) 기준 경로 좌표의 벽
오프셋까지 거리로 분류한다. 정답 자세는 이 사후 분류에만 쓰며 제어·판정을 바꾸지 않는다.
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
from arena_autonomy.lateral_intent import PathFrame  # noqa: E402
from arena_autonomy.occupancy_timing import per_ray_padding  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402

ACTOR_TOL_M = .03   # 거리 오차·표면 표본 허용(평가 분류용). 판정선이 아니다.
WALL_TOL_M = .03    # side_threats의 벽 허용과 같은 값


def poses(path):
    return [json.loads(line) for line in path.open(encoding='utf-8')]


def at(series, t):
    k = max(0, bisect_right([r[0] for r in series], t)-1)
    return series[k]


def first_interference(points, margin, steering, cfg):
    """swept_limit와 같은 진행 격자에서 첫 간섭 거리와 그때의 간섭 점 색인."""
    curvature = math.tan(steering)/cfg.wheelbase_m
    for distance in np.arange(0., 2.5+.001, .04):
        yaw = curvature*distance
        if abs(curvature) < 1e-5:
            rx, ry = distance-cfg.wheelbase_m/2, 0.
        else:
            rx, ry = math.sin(yaw)/curvature-cfg.wheelbase_m/2, (1-math.cos(yaw))/curvature
        dx, dy = points[:, 0]-rx, points[:, 1]-ry
        lx = math.cos(yaw)*dx+math.sin(yaw)*dy-cfg.wheelbase_m/2
        ly = -math.sin(yaw)*dx+math.cos(yaw)*dy
        hit = (np.abs(lx) <= cfg.length_m/2+margin) & (np.abs(ly) <= cfg.width_m/2+margin)
        if np.any(hit):
            return float(distance), np.flatnonzero(hit)
    return None, np.zeros(0, dtype=int)


def box_distance(point, center, yaw, length=.20, width=.15):
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = point[0]-center[0], point[1]-center[1]
    lx, ly = c*dx+s*dy, -s*dx+c*dy
    return math.hypot(max(0., abs(lx)-length/2), max(0., abs(ly)-width/2))


def audit(run_dir):
    lines = run_dir.joinpath('follow_decisions.jsonl').read_text(encoding='utf-8').splitlines()
    head = json.loads(lines[0])['data']
    motion, cfg = MotionConfig(**head['motion_config']), FollowConfig(**head['follow_config'])
    ego = poses(run_dir/'ego_poses.jsonl')
    side = json.loads((run_dir/'additional_actors.json').read_text(encoding='utf-8'))['poses']['eval_e3_side']
    counts, rows = Counter(), []
    for line in lines[1:-1]:
        event = json.loads(line)
        if event['kind'] != 'call' or event['data']['output'][2].get('reason') != 'parallel_hold_unchanged_gap_hold':
            continue
        detail = event['data']['output'][2]
        data, scan, history = restore_call(event['data']['input'], motion)
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        rays = [r for r in scene.rays if r.status == 'return']
        points = np.asarray([r.hit_xy_m for r in rays], dtype=float)
        pad = per_ray_padding(scene, .025+cfg.measurement_padding_m, cfg.object_motion_bound_mps)
        trial = detail['parallel_hold_trial']
        frame = PathFrame(np.asarray(detail['path_points'], dtype=float)) if detail.get('path_points') else None
        # 기록 간격을 만든 조향(현재/직전) 중 더 짧은 쪽의 첫 간섭 반환
        best = None
        for angle in (trial['steering_rad'], data['previous_steering_rad']):
            distance, index = first_interference(points, pad, angle, cfg)
            if distance is not None and (best is None or distance < best[0]):
                best = (distance, index)
        if best is None:
            counts['no_interference_recomputed'] += 1
            continue
        e = at(ego, data['reference_s'])
        c0, s0 = math.cos(e[3]), math.sin(e[3])
        classes = Counter()
        ages = []
        for i in best[1]:
            ray = rays[i]
            a = at(side, ray.acquired_at_s)
            # 정답 actor 중심을 기준 시각 자차 좌표로 옮긴다(평가 전용; 운동 보정 차이는 오차로 남는다).
            dx, dy = a[1]-e[1], a[2]-e[2]
            center = (c0*dx+s0*dy, -s0*dx+c0*dy)
            d_actor = box_distance(points[i], center, a[3]-e[3])
            wall = False
            if frame is not None:
                _, l = frame.project(points[i:i+1])
                wall = bool(abs(abs(float(l[0]))-cfg.wall_offset_m) <= WALL_TOL_M)
            cls = 'actor' if d_actor <= ACTOR_TOL_M else ('wall' if wall else 'other')
            classes[cls] += 1
            ages.append(scene.t_ref_s-ray.acquired_at_s)
        dominant = classes.most_common(1)[0][0]
        counts[dominant] += 1
        rows.append({'t': data['reference_s'], 'first_distance_m': best[0], 'interfering_returns': int(len(best[1])),
                     'classes': dict(classes), 'age_s': [float(min(ages)), float(max(ages))]})
    ages = [a for r in rows for a in r['age_s']]
    return {'gap_hold_rows': len(rows), 'rows_by_dominant_class': dict(counts),
            'rows_all_interfering_actor': sum(set(r['classes']) == {'actor'} for r in rows),
            'interfering_age_range_s': [min(ages), max(ages)] if ages else None,
            'first_distance_range_m': [min(r['first_distance_m'] for r in rows), max(r['first_distance_m'] for r in rows)]
                                      if rows else None, 'rows': rows}


out = {'scope': 'evaluation-only isolation of first-interfering returns; truth box/wall classification post hoc',
       'actor_tolerance_m': ACTOR_TOL_M, 'wall_tolerance_m': WALL_TOL_M,
       'left': audit(HERE/'run1_left'), 'right': audit(HERE/'run2_right')}
(HERE/'run2_first_interference_audit.json').write_text(json.dumps(out, ensure_ascii=False, indent=1)+'\n',
                                                        encoding='utf-8', newline='\n')
for side in ('left', 'right'):
    print(side, {k: v for k, v in out[side].items() if k != 'rows'})
