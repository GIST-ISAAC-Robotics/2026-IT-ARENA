"""run17 감속을 만든 실제 반환의 사후 연관. actor 정답은 평가에만 쓰며 제어기 입력이 아니다."""
import argparse
from bisect import bisect_right
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import swept_limit, braking_speed
from arena_autonomy.occupancy_timing import per_ray_padding


def latest(rows, stamps, t):
    i = bisect_right(stamps, t)-1
    return None if i < 0 else rows[i]


def first_collision(points, steering, cfg, margin):
    curvature = math.tan(steering)/cfg.wheelbase_m
    for distance in np.arange(0., 2.501, .04):
        yaw = curvature*distance
        if abs(curvature) < 1e-5:
            rx, ry = distance-cfg.wheelbase_m/2, 0.
        else:
            rx = math.sin(yaw)/curvature-cfg.wheelbase_m/2
            ry = (1-math.cos(yaw))/curvature
        dx, dy = points[:, 0]-rx, points[:, 1]-ry
        x = math.cos(yaw)*dx+math.sin(yaw)*dy-cfg.wheelbase_m/2
        y = -math.sin(yaw)*dx+math.cos(yaw)*dy
        hit = (np.abs(x) <= cfg.length_m/2+margin) & (np.abs(y) <= cfg.width_m/2+margin)
        if np.any(hit):
            return max(0., float(distance)-.04), np.flatnonzero(hit)
    return 2.5, np.zeros(0, dtype=int)


def box_distance(point, pose):
    _, x, y, yaw = pose
    dx, dy = point-[x, y]
    a = math.cos(yaw)*dx+math.sin(yaw)*dy
    b = -math.sin(yaw)*dx+math.cos(yaw)*dy
    return math.hypot(max(0., abs(a)-.10), max(0., abs(b)-.075))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('run', type=Path)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        raise ValueError('new output required')
    raw = (args.run/'follow_decisions.jsonl').read_bytes()
    events = [json.loads(x) for x in raw.decode('utf8').splitlines()]
    h = events[0]['data']
    cfg, motion = FollowConfig(**h['follow_config']), MotionConfig(**h['motion_config'])
    ego = [json.loads(x) for x in (args.run/'ego_poses.jsonl').read_text().splitlines()]
    lead = [json.loads(x) for x in (args.run/'actor_poses.jsonl').read_text().splitlines()]
    extra = json.loads((args.run/'additional_actors.json').read_text())['poses']
    actors = {'primary_lead': lead, **extra}
    es = [r[0] for r in ego]
    stamps = {n: [r[0] for r in a] for n, a in actors.items()}
    report = json.loads((args.run/'report.json').read_text())
    end = report['case_window']['end_sim_s']
    rows, released, exact, sampled = [], None, 0, 0
    for event in events:
        if event['kind'] != 'call':
            continue
        data, out = event['data']['input'], event['data']['output']
        d, t = out[2], data['reference_s']
        lateral = d.get('m3_lateral', {})
        if lateral.get('released_to_baseline'):
            released = t
        if released is None or t > end or lateral.get('mode') != 'BASELINE' or out[0] <= 0:
            continue
        # fresh scan 하나당 첫 양수 판단. 동일 입력의 전체 제어기 재생은 이미 완료했다.
        if rows and math.isclose(rows[-1]['source_first_s'], d['source_first_s'], abs_tol=1e-10):
            continue
        data, scan, history = restore_call(data, motion)
        scene = motion_scene(scan, history, t, timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        points = np.asarray(scene.occupied_endpoints_xy_m)
        margin = per_ray_padding(scene, .025+cfg.measurement_padding_m, cfg.object_motion_bound_mps)
        choices = [first_collision(points, a, cfg, margin) for a in (out[1], data['previous_steering_rad'])]
        c = min(x[0] for x in choices)
        exact += math.isclose(c, d['swept_clearance_m'], abs_tol=1e-10)
        sampled += 1
        ids = np.unique(np.concatenate([i for value, i in choices if math.isclose(value, c)]))
        ep = latest(ego, es, t)
        labels = Counter()
        nearest = []
        ce, se = math.cos(ep[3]), math.sin(ep[3])
        for i in ids:
            q = points[i]
            world = np.array([ep[1]+ce*q[0]-se*q[1], ep[2]+se*q[0]+ce*q[1]])
            dist = {n: box_distance(world, latest(a, stamps[n], t)) for n, a in actors.items()}
            name = min(dist, key=dist.get)
            labels[name if dist[name] <= .08 else 'wall_or_unmatched'] += 1
            nearest.append(min(dist.values()))
        body_gaps = {}
        for name, a in actors.items():
            ap = latest(a, stamps[name], t)
            dx, dy = ap[1]-ep[1], ap[2]-ep[2]
            body_gaps[name] = {'relative_long_m': ce*dx+se*dy, 'relative_lat_m': -se*dx+ce*dy}
        age = t-scene.source_first_s
        available = max(0., c-cfg.standstill_gap_m)
        rows.append({'t': t, 'source_first_s': scene.source_first_s, 'speed_mps': data['speed_mps'],
                     'target_mps': out[0], 'clearance_m': c, 'hit_labels': dict(labels),
                     'headway_cap_mps': available/cfg.headway_s, 'braking_cap_mps': braking_speed(available, age),
                     'actor_centers_at_reference': body_gaps, 'min_hit_to_actor_m': min(nearest) if nearest else None})
    result = {'scope': 'post-run attribution by truth boxes; never input to controller; 80 mm association tolerance is diagnostic only',
              'input_sha256': hashlib.sha256(raw).hexdigest(), 'release_s': released,
              'first_collision_matches': exact, 'sampled_calls': sampled,
              'limiting_actor_counts': dict(Counter(k for r in rows for k in r['hit_labels'])),
              'headway_limited_calls': sum(math.isclose(r['target_mps'], r['headway_cap_mps'], abs_tol=1e-10) for r in rows),
              'rows': rows}
    result['input_unchanged'] = hashlib.sha256((args.run/'follow_decisions.jsonl').read_bytes()).hexdigest() == result['input_sha256']
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k: v for k,v in result.items() if k != 'rows'}, ensure_ascii=False))
    assert sampled and exact == sampled and result['input_unchanged']


if __name__ == '__main__':
    main()
