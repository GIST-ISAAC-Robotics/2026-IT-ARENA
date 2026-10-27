"""run4/run5 RELEASED 뒤 G1 속도를 실제로 제한한 반환·팽창·제어 항을 분리한다(평가 전용, 읽기 전용).

판단 기록의 입력으로 scene을 재구성하고, 기록된 출력의 swept clearance를 같은 함수로 다시 계산해 일치를 확인한 뒤
`swept_limit`과 같은 진행 격자에서 첫 간섭 반환을 모은다. 각 반환은 그 광선 취득 시각의 정답 actor 상자(원래 앞차/추가
옆차)와 기준 경로의 벽 오프셋으로 **사후 분류**만 한다. 정답 자세는 제어·판정에 쓰지 않으며 기존 평가 판정을 바꾸지 않는다.

반사실 항목(`counterfactual_*`)은 분류된 옆차 반환만 뺀 점군으로 같은 clearance/목표 속도를 계산한 설명용 값이다.
실제 제어 출력이 아니며 그 점들이 없었을 때 차량이 실제로 그렇게 움직였다는 증거도 아니다.
"""
from bisect import bisect_right
from collections import Counter
import hashlib
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
from arena_autonomy.local_follow import FollowConfig, pursue_path  # noqa: E402
from arena_autonomy.local_path import braking_speed, swept_limit  # noqa: E402
from arena_autonomy.lateral_intent import PathFrame  # noqa: E402
from arena_autonomy.occupancy_timing import per_ray_padding  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402

SOURCE = HERE.parent/'g1_pass_release_integration_v1'
RUNS = {'run4_left': SOURCE/'run4_left', 'run5_right': SOURCE/'run5_right'}
ACTOR_TOL_M = .03   # 평가 분류 허용(기존 run2_first_interference_audit와 같은 값). 판정선이 아니다.
WALL_TOL_M = .03
END_WINDOW_S = 3.0  # 종료 직전 요약 창


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rows(path):
    return [json.loads(line) for line in path.open(encoding='utf-8')]


def at(series, times, t):
    return series[max(0, bisect_right(times, t)-1)]


def first_interference(points, margin, steering, cfg):
    """swept_limit과 같은 진행 격자의 첫 간섭 거리와 간섭 점 색인."""
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
            return max(0., float(distance)-.04), np.flatnonzero(hit)
    return 2.5, np.zeros(0, dtype=int)


def box_distance(point, center, yaw, length=.20, width=.15):
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = point[0]-center[0], point[1]-center[1]
    lx, ly = c*dx+s*dy, -s*dx+c*dy
    return math.hypot(max(0., abs(lx)-length/2), max(0., abs(ly)-width/2))


def to_ego(actor, ego):
    c, s = math.cos(ego[3]), math.sin(ego[3])
    dx, dy = actor[1]-ego[1], actor[2]-ego[2]
    return (c*dx+s*dy, -s*dx+c*dy), actor[3]-ego[3]


def target_terms(clearance, cap, age, cfg):
    available = max(0., clearance-cfg.standstill_gap_m)
    terms = {'cap_mps': cap, 'braking_mps': braking_speed(available, age), 'headway_mps': available/cfg.headway_s}
    binding = min(terms, key=terms.get)
    return min(terms.values()), binding, terms


def audit(name, run_dir):
    lines = run_dir.joinpath('follow_decisions.jsonl').read_text(encoding='utf-8').splitlines()
    head = json.loads(lines[0])['data']
    motion, cfg = MotionConfig(**head['motion_config']), FollowConfig(**head['follow_config'])
    ego = rows(run_dir/'ego_poses.jsonl')
    primary = rows(run_dir/'actor_poses.jsonl')
    side = json.loads((run_dir/'additional_actors.json').read_text(encoding='utf-8'))['poses']['eval_e3_side']
    et, pt, st = [r[0] for r in ego], [r[0] for r in primary], [r[0] for r in side]
    report = json.loads((run_dir/'report.json').read_text(encoding='utf-8'))
    end_s = report['case_window']['end_sim_s']
    released_s, out, mismatch = None, [], 0
    for line in lines[1:-1]:
        event = json.loads(line)
        if event['kind'] != 'call':
            continue
        speed_cmd, steer_cmd, detail = event['data']['output']
        if detail.get('selector_mode') == 'RELEASED' and released_s is None:
            released_s = event['data']['input']['reference_s']
        if released_s is None or detail.get('selector_choice') != 'baseline' or detail.get('reason') != 'local_follow':
            continue
        data, scan, history = restore_call(event['data']['input'], motion)
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        rays = [r for r in scene.rays if r.status == 'return']
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        if not np.allclose(points, np.asarray([r.hit_xy_m for r in rays], dtype=float)):
            raise ValueError('occupied endpoints differ from return rays')
        pad = per_ray_padding(scene, .025+cfg.measurement_padding_m, cfg.object_motion_bound_mps)
        previous = data['previous_steering_rad']
        clearance = min(swept_limit(points, a, cfg.wheelbase_m, horizon=2.5, margin=pad, length=cfg.length_m,
                                    width=cfg.width_m) for a in (steer_cmd, previous))
        if not math.isclose(clearance, detail['swept_clearance_m'], abs_tol=1e-9):
            mismatch += 1
            continue
        age = scene.t_ref_s-scene.source_first_s
        cap = pursue_path(np.asarray(detail['path_points']), data['speed_mps'], cfg,
                          max(age, detail['boundary_age_s']))[0]
        target, binding, terms = target_terms(clearance, cap, age, cfg)
        best = min((first_interference(points, pad, a, cfg) for a in (steer_cmd, previous)), key=lambda v: v[0])
        e = at(ego, et, data['reference_s'])
        frame = PathFrame(np.asarray(detail['path_points'], dtype=float))
        classes = Counter()
        labels = np.full(len(points), '', dtype=object)
        for i in range(len(points)):
            t_ray = rays[i].acquired_at_s
            p_center, p_yaw = to_ego(at(primary, pt, t_ray), e)
            s_center, s_yaw = to_ego(at(side, st, t_ray), e)
            if box_distance(points[i], p_center, p_yaw) <= ACTOR_TOL_M:
                labels[i] = 'primary'
            elif box_distance(points[i], s_center, s_yaw) <= ACTOR_TOL_M:
                labels[i] = 'side'
            else:
                _, l = frame.project(points[i:i+1])
                labels[i] = 'wall' if abs(abs(float(l[0]))-cfg.wall_offset_m) <= WALL_TOL_M else 'other'
        for i in best[1]:
            classes[labels[i]] += 1
        keep = labels != 'side'
        cf_clearance = min(swept_limit(points[keep], a, cfg.wheelbase_m, horizon=2.5, margin=pad[keep],
                                       length=cfg.length_m, width=cfg.width_m) for a in (steer_cmd, previous))
        cf_target, cf_binding, _ = target_terms(cf_clearance, cap, age, cfg)
        geometric = min(swept_limit(points, a, cfg.wheelbase_m, horizon=2.5, margin=.0, length=cfg.length_m,
                                    width=cfg.width_m) for a in (steer_cmd, previous))
        p_now, _ = to_ego(at(primary, pt, data['reference_s']), e)
        s_now, _ = to_ego(at(side, st, data['reference_s']), e)
        out.append({'t': data['reference_s'], 'command_mps': speed_cmd, 'measured_mps': data['speed_mps'],
                    'recomputed_target_mps': target, 'recorded_target_mps': detail['target_speed_before_hold_mps'],
                    'binding_term': binding, 'terms_mps': terms, 'swept_clearance_m': clearance,
                    'zero_padding_clearance_m': geometric,
                    'limiting_returns': dict(classes), 'returns_by_class': dict(Counter(labels.tolist())),
                    'limiting_padding_m': [float(np.min(pad[best[1]])), float(np.max(pad[best[1]]))]
                    if len(best[1]) else None,
                    'counterfactual_without_side_clearance_m': cf_clearance,
                    'counterfactual_without_side_target_mps': cf_target, 'counterfactual_binding_term': cf_binding,
                    'eval_primary_longitudinal_m': p_now[0], 'eval_primary_lateral_m': p_now[1],
                    'eval_side_longitudinal_m': s_now[0], 'eval_side_lateral_m': s_now[1]})
    for r in out:
        if not math.isclose(r['recomputed_target_mps'], r['recorded_target_mps'], abs_tol=1e-9):
            raise ValueError(f'target recomputation mismatch at {r["t"]}')

    def summary(sel):
        if not sel:
            return None
        dom = Counter(max(r['limiting_returns'], key=r['limiting_returns'].get) if r['limiting_returns'] else 'none'
                      for r in sel)
        return {'calls': len(sel), 'dominant_limiting_return': dict(dom),
                'calls_with_primary_returns': sum(r['returns_by_class'].get('primary', 0) > 0 for r in sel),
                'primary_returns_range': [min(r['returns_by_class'].get('primary', 0) for r in sel),
                                          max(r['returns_by_class'].get('primary', 0) for r in sel)], 'binding_term': dict(Counter(r['binding_term'] for r in sel)),
                'command_mps_range': [min(r['command_mps'] for r in sel), max(r['command_mps'] for r in sel)],
                'swept_clearance_m_range': [min(r['swept_clearance_m'] for r in sel), max(r['swept_clearance_m'] for r in sel)],
                'zero_padding_clearance_m_range': [min(r['zero_padding_clearance_m'] for r in sel),
                                                   max(r['zero_padding_clearance_m'] for r in sel)],
                'limiting_padding_m_range': [min(r['limiting_padding_m'][0] for r in sel if r['limiting_padding_m']),
                                             max(r['limiting_padding_m'][1] for r in sel if r['limiting_padding_m'])],
                'counterfactual_without_side_target_mps_range': [min(r['counterfactual_without_side_target_mps'] for r in sel),
                                                                 max(r['counterfactual_without_side_target_mps'] for r in sel)],
                'counterfactual_binding_term': dict(Counter(r['counterfactual_binding_term'] for r in sel)),
                'eval_side_lateral_m_range': [min(r['eval_side_lateral_m'] for r in sel), max(r['eval_side_lateral_m'] for r in sel)],
                'eval_side_longitudinal_m_range': [min(r['eval_side_longitudinal_m'] for r in sel),
                                                   max(r['eval_side_longitudinal_m'] for r in sel)]}
    end = [r for r in out if r['t'] >= end_s-END_WINDOW_S]
    return {'released_first_s': released_s, 'end_sim_s': end_s, 'clearance_recompute_mismatch': mismatch,
            'released_baseline_follow': summary(out), 'last_window': summary(end), 'last_window_s': END_WINDOW_S,
            'last_rows': out[-5:], 'rows': out,
            'inputs_sha256': {p.name: sha(p) for p in (run_dir/'follow_decisions.jsonl', run_dir/'ego_poses.jsonl',
                                                      run_dir/'actor_poses.jsonl', run_dir/'additional_actors.json')}}


result = {'scope': 'evaluation-only; returns classified post hoc against truth boxes at ray acquisition time; '
                   'counterfactual values are explanation only and not control output; original verdicts unchanged',
          'actor_tolerance_m': ACTOR_TOL_M, 'wall_tolerance_m': WALL_TOL_M}
for key, path in RUNS.items():
    result[key] = audit(key, path)
(HERE/'end_limit_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8',
                                         newline='\n')
for key in RUNS:
    r = result[key]
    print(key, json.dumps({k: r[k] for k in ('released_first_s', 'clearance_recompute_mismatch', 'last_window',
                                            'released_baseline_follow')},
                          ensure_ascii=False))
