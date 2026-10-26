"""고정 저장 입력의 측방 위협이 관측 경로 외삽/긴 반환열에서 생기는지 분해한다.

센서와 당시 경로만 사용한다. 긴 반환열은 차량 아님/자유 공간으로 확정하지 않는다.
이 스크립트는 정책 변경이나 폐루프 주행이 아니다.
"""
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import screen_lateral as screen
import arena_autonomy.lateral_intent as li


def groups(scene):
    """인접 반환이 거리 오차 양끝+각도 분해능 안에서 연결되는 관측 묶음."""
    rays = [r for r in scene.rays if r.status == 'return']
    points = np.asarray([r.hit_xy_m for r in rays]).reshape(-1, 2)
    if not len(points):
        return points, [], np.zeros(0)
    labels = np.zeros(len(points), dtype=int)
    label = 0
    for k in range(1, len(rays)):
        a, b = rays[k-1:k+1]
        theta = abs(math.remainder(math.atan2(*a.direction_xy[::-1])-math.atan2(*b.direction_xy[::-1]), math.tau))
        depth = min(math.dist(a.ray_origin_xy_m, a.hit_xy_m), math.dist(b.ray_origin_xy_m, b.hit_xy_m))
        allowance = .060+1.5*depth*theta+.15*abs(b.acquired_at_s-a.acquired_at_s)
        if b.source_index-a.source_index != 1 or np.linalg.norm(points[k]-points[k-1]) > allowance:
            label += 1
        labels[k] = label
    extents = np.zeros(label+1)
    for j in range(label+1):
        p = points[labels == j]
        extents[j] = float(np.linalg.norm(np.max(p, axis=0)-np.min(p, axis=0)))
    return points, labels, extents


def run(name):
    motion, _, calls = screen.load_inputs(screen.RECORDS[name])
    cfg = li.LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass')
    rows = []
    with screen.TruthBlocked():
        for e in calls:
            if e['kind'] != 'call':
                continue
            path = e['data']['output'][2].get('path_points')
            if path is None:
                continue
            data, scan, history = screen.restore_call(screen.thaw(e['data']['input']), motion)
            scene = screen.motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                        epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            frame = li.PathFrame(path)
            state = li.ego_state(frame, cfg)
            points, labels, sizes = groups(scene)
            s, l = frame.project(points)
            ds = s-state['s0']
            # 元 side_threats의 후보 반환 집합만 대조한다.
            window = (ds >= -cfg.band_behind_m) & (ds <= cfg.band_ahead_m)
            interior = abs(l) < cfg.wall_offset_m-cfg.wall_tolerance_m
            wall = (abs(l-cfg.wall_offset_m) <= cfg.wall_tolerance_m) | (abs(l+cfg.wall_offset_m) <= cfg.wall_tolerance_m)
            long_limit = math.hypot(cfg.length_m, cfg.width_m)+2*cfg.budget.range_bias_m+cfg.budget.other_lateral_speed_mps*cfg.budget.observation_age_s
            row = {'t': scene.t_ref_s, 'path_first_x': float(frame.p[0, 0]), 'path_last_x': float(frame.p[-1, 0]), 's0': state['s0'], 's_end': frame.end_s}
            for side, sign in li.SIDES:
                chosen = window & interior & ~wall & (sign*(l-state['d0']) >= state['half_span_m'])
                short = chosen & (sizes[labels] <= long_limit)
                row[side] = {'present': bool(chosen.sum() >= cfg.min_threat_points), 'points': int(chosen.sum()),
                             'outside_path_support': int((chosen & ((s < 0) | (s > frame.end_s))).sum()),
                             'long_structure_points': int((chosen & ~short).sum()),
                             'remaining_compact_present': bool(short.sum() >= cfg.min_threat_points),
                             'remaining_compact_points': int(short.sum())}
                if chosen.sum() >= cfg.min_threat_points:
                    row[side]['groups'] = [{'n': int((chosen & (labels == k)).sum()), 'observed_span_m': float(sizes[k])} for k in np.unique(labels[chosen])]
            rows.append(row)
    summary = {'path_calls': len(rows)}
    for side, _ in li.SIDES:
        selected = [r[side] for r in rows if r[side]['present']]
        summary[side] = {'original_present': len(selected), 'compact_present': sum(r['remaining_compact_present'] for r in selected),
                         'all_outside_supported_path': sum(r['points'] == r['outside_path_support'] for r in selected),
                         'all_in_long_structure': sum(r['points'] == r['long_structure_points'] for r in selected)}
    return {'summary': summary, 'rows': rows}


result = {name: run(name) for name in ('e3_onroad_g1', 'e3_onroad_r0plus', 'solo_g1_20261016')}
out = Path(sys.argv[1])
out.write_text(json.dumps(result, indent=1), encoding='utf-8')
for name, data in result.items():
    print(name, json.dumps(data['summary']))
