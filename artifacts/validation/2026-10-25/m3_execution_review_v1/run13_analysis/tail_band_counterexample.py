"""꼬리 흔들림 띠(후방 마스크 경계)의 최소 반례와 '숨을 수 있는 물체 크기' 계산. 소스·판정 변경 없음.

1) 합성 직선 도로(벽만 있음, 기존 시험 장면): 같은 조향 원호의 발행 외곽 관측 결과를 각도별로 기록한다.
2) 현재 스캔에서 관측 자유가 아닌 영역 H(현재 차체 내부 제외)에 대해, 미지지 질의 q를 포함하고 H 안에 완전히
   들어가는 원판의 최대 지름을 표본 탐색한다. 물체 크기 가정 후보의 근거 수치이며 연속 증명이 아니다.
   과거 장면이나 미래 접근은 다루지 않는다(숨은 후방 차량의 미래 진입은 별도 미해결).
3) run13 저장 입력: 전 후보 명령 거절 판단의 명령 외곽 미지지 질의에 2)를 적용한다.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import locate_unsupported as L  # noqa: E402
import arena_autonomy.m3_execution_trial as m3  # noqa: E402
from arena_autonomy.observation_support import depth_support  # noqa: E402
from arena_autonomy.follow_trace import restore_call, freeze, replay_call, equal_output  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402
from arena_autonomy.local_follow import FollowConfig  # noqa: E402

RADII = (.0005, .001, .002, .003, .005, .0075, .01, .015, .02, .03, .05, .075)
DIRECTIONS = np.linspace(0., 2*math.pi, 48, endpoint=False)
OFFSETS = (0., .5, 1.)   # 원판 중심 = q + t·ρ·u (t=1이면 q가 원 둘레 위)


def disk_samples(center, rho):
    ring = np.column_stack((np.cos(DIRECTIONS[::2]), np.sin(DIRECTIONS[::2])))
    return np.vstack((center[None], center+rho*ring, center+.5*rho*ring))


def hideable_diameter(scene, q, pad, cfg):
    """q를 포함하고 '현재 관측 자유가 아님 ∧ 현재 차체 밖'인 표본만으로 이뤄진 원판의 최대 지름(표본 탐색)."""
    lx, wy = cfg.length_m/2, cfg.width_m/2
    best = 0.
    for rho in RADII:
        found = False
        for t in OFFSETS:
            for a in DIRECTIONS:
                center = q+t*rho*np.array([math.cos(a), math.sin(a)])
                pts = disk_samples(center, rho)
                if np.any((np.abs(pts[:, 0]) <= lx) & (np.abs(pts[:, 1]) <= wy)):
                    continue
                if np.any(depth_support(scene, pts, pad)):
                    continue
                found = True
                break
            if found:
                break
        if not found:
            break
        best = 2*rho
    return best


def synthetic():
    sys.path.insert(0, str(HERE.parents[4]/'tests'))
    from test_lateral_intent import GRASS
    from test_m3_execution_trial import two_scans, EXEC
    rows = []
    for v in (0., .35):
        scene, path, frames = two_scans(v)
        distance = m3.envelope_distance(.35, .1, .02, EXEC)
        pad = m3.ray_padding(scene, GRASS, EXEC.history_motion_bound_mps)
        for pair in ((0., 0.), (.01, .01), (.03, .03), (.06, .06), (-.06, -.06), (0., .06), (.06, .12), (.12, .12), (.3, .3)):
            check = m3.arc_checks(scene, frames, path, pair, distance, GRASS, EXEC)
            q, ok, cause = L.classify(scene, frames, m3.body_queries(scene, np.vstack([m3.command_arc(a, distance, GRASS) for a in sorted(set(pair))]),
                                                                    np.concatenate([np.full(len(m3.command_arc(a, distance, GRASS)), a) for a in sorted(set(pair))]), GRASS)[0],
                                      GRASS, EXEC)
            miss = q[~ok]
            widths = [hideable_diameter(scene, p, pad, GRASS) for p in miss[:12]]
            rows.append({'measured_speed_mps': v, 'steering_pair_rad': pair, 'envelope_m': distance,
                         'observation_ok': check['observation_ok'], 'geometry_ok': check['geometry_ok'],
                         'unsupported': check['observation']['unsupported'],
                         'history_reachable_rejections': check['observation']['history_reachable_rejections'],
                         'max_outside_body_m': float(np.max(np.maximum(np.abs(miss[:, 0])-GRASS.length_m/2,
                                                                       np.abs(miss[:, 1])-GRASS.width_m/2))) if len(miss) else None,
                         'hideable_diameter_m_max_of_first12': max(widths) if widths else None})
    return {'scene': 'tests straight(): walls only, sensor (0.06, 0), 60 deg rear mask; two scans 0.1 s apart',
            'rows': rows}


def stored(decisions, limit_per_call):
    raw = decisions.read_bytes()
    events = [json.loads(line) for line in raw.decode().splitlines()]
    head = events[0]['data']
    motion, config = MotionConfig(**head['motion_config']), FollowConfig(**head['follow_config'])
    execution = m3.ExecutionConfig(**head['execution_config'])
    m3.support, m3.arc_checks, m3.plan_check = L.wrapped_support, L.wrapped_arc, L.wrapped_plan
    m3.M3ExecutionFollow._evaluate_command = L.wrapped_eval
    follow = m3.M3ExecutionFollow(config, execution, allow_simulation_trial=True)
    inputs, exact, calls = {}, 0, 0
    for event in events[1:-1]:
        if event['kind'] == 'reset':
            follow.reset()
            continue
        L.CONTEXT.update(seq=event['seq'], stage=None, candidate=None)
        out = freeze(replay_call(follow, event['data']['input'], motion))
        calls += 1
        exact += equal_output(out, event['data']['output'])
        inputs[event['seq']] = (event['data']['input'], out[2]['reason'])
    m3.support, m3.arc_checks, m3.plan_check = L.ORIGINAL_SUPPORT, L.ORIGINAL_ARC, L.ORIGINAL_PLAN
    m3.M3ExecutionFollow._evaluate_command = L.ORIGINAL_EVAL
    lateral_cfg = follow.lateral_config
    per_call, widths = [], []
    for seq, (data, reason) in inputs.items():
        if reason != 'm3_command_unobserved':
            continue
        pts = {(round(u['x'], 7), round(u['y'], 7)) for r in L.RECORDS if r['seq'] == seq
               and r['stage'] == 'command_envelope' for u in r['unsupported']}
        if not pts:
            continue
        d, scan, history = restore_call(data, motion)
        scene = motion_scene(scan, history, d['reference_s'], timing_verified=d['timing_verified'],
                             epoch=d['epoch'], rear_half_angle_deg=d['rear_half_angle_deg'])
        pad = m3.ray_padding(scene, lateral_cfg, execution.history_motion_bound_mps)
        chosen = sorted(pts)[:limit_per_call]
        w = [hideable_diameter(scene, np.array(p), pad, lateral_cfg) for p in chosen]
        widths += w
        per_call.append({'seq': seq, 't': d['reference_s'], 'points_evaluated': len(chosen), 'points_total': len(pts),
                         'hideable_diameter_max_m': max(w)})
    return {'input_sha256': hashlib.sha256(raw).hexdigest(), 'calls': calls, 'exact_output_matches': exact,
            'focus_calls_evaluated': len(per_call), 'points_evaluated': len(widths),
            'hideable_diameter_m': {'max': max(widths) if widths else None,
                                    'p95': float(np.percentile(widths, 95)) if widths else None,
                                    'p50': float(np.percentile(widths, 50)) if widths else None},
            'per_call': per_call}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('decisions', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--limit-per-call', type=int, default=8)
    args = p.parse_args()
    if args.output.exists():
        raise ValueError('new output required')
    result = {'scope': 'counterexample + sampled hideable-size estimate; not a safety proof, not a control change',
              'radii_m': RADII, 'directions': len(DIRECTIONS), 'offsets': OFFSETS,
              'synthetic': synthetic(), 'stored_run13': stored(args.decisions, args.limit_per_call)}
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps({'synthetic': result['synthetic']['rows'],
                      'stored': {k: v for k, v in result['stored_run13'].items() if k != 'per_call'}},
                     ensure_ascii=False, indent=1))
    assert result['stored_run13']['exact_output_matches'] == result['stored_run13']['calls']


if __name__ == '__main__':
    main()
