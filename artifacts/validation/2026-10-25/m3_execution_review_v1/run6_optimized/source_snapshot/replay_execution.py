"""M3 유한 실행 후보를 저장 센서 입력에 연결한 개루프 대조. 새 궤적/폐루프/실차 시험이 아니다.

입력·직전 조향(`previous_steering_rad`)·측정 속도는 원래 G1/R0+ 기록의 값이다. 후보가 정지를 내도 기록
차량은 계속 움직였으므로, 이후 판단의 장면·속도·이력은 후보가 실제로 만들 상태가 아니다. 정답 자세는
계산 루프가 끝난 뒤 사후 거리 요약에만 연다.
"""
import argparse
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = next(p for p in HERE.parents if (p/'src/arena_autonomy').is_dir())
sys.path.insert(0, str(ROOT/'artifacts/validation/2026-10-24/m3_surface_lateral_v1'))
sys.path.insert(0, str(HERE))
import screen_lateral as screen  # noqa: E402
from arena_autonomy.local_follow import LocalFollow  # noqa: E402
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig  # noqa: E402
from arena_autonomy.m3_observed_follow import M3ObservedFollow  # noqa: E402
from arena_autonomy.follow_trace import freeze  # noqa: E402
from analyze_run17 import load_truth, at, rect_distance, to_world  # noqa: E402

VARIANTS = {
    'full': ExecutionConfig(),
    'no_history': ExecutionConfig(masked_history_support=False),
    'no_straight': ExecutionConfig(straight_start=False),
    'strict': ExecutionConfig(masked_history_support=False, straight_start=False),
    'frozen_run17': None,   # 동결 M3ObservedFollow(run17 판본)를 같은 하네스로 재호출
}
RECORDS = ('e3_onroad_g1', 'e3_onroad_r0plus', 'e3_stop_restart_g1', 'solo_g1_20261016')


def count(values):
    out = {}
    for v in values:
        out[v] = out.get(v, 0)+1
    return dict(sorted(out.items(), key=lambda kv: -kv[1]))


def run(name, execution, self_steering=False):
    motion, config, events = screen.load_inputs(screen.RECORDS[name])
    config = replace(config, occupancy_timing_mode='per_ray_candidate', recent_observation_enabled=False)
    candidate = (M3ObservedFollow(config, allow_simulation_trial=True) if execution is None else
                 M3ExecutionFollow(config, execution, allow_simulation_trial=True))
    baseline = LocalFollow(config)
    rows = []
    own = {'candidate': 0., 'baseline': 0.}
    with screen.TruthBlocked():
        for event in events:
            if event['kind'] == 'reset':
                candidate.reset()
                baseline.reset()
                own = {'candidate': 0., 'baseline': 0.}
                continue
            data, scan, history = screen.restore_call(screen.thaw(event['data']['input']), motion)
            scene = screen.motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                        epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            args = (scene, history, data['speed_mps'], data['side'], data['previous_steering_rad'], data['dt_s'])
            if self_steering:
                # 반사실 진단: 센서·속도는 기록 그대로, 직전 조향만 각 제어기 자신의 직전 출력이다.
                base = baseline.command(*args[:4], own['baseline'], args[5])
                started = time.perf_counter()
                out = candidate.command(*args[:4], own['candidate'], args[5])
                elapsed = 1e3*(time.perf_counter()-started)
                own = {'candidate': float(out[1]), 'baseline': float(base[1])}
            else:
                base = baseline.command(*args)
                started = time.perf_counter()
                out = candidate.command(*args)
                elapsed = 1e3*(time.perf_counter()-started)
            detail = out[2]
            env = detail.get('m3_envelope') or {}
            obs = env.get('observation') or {}
            rows.append({'t': scene.t_ref_s, 'speed': data['speed_mps'], 'previous': data['previous_steering_rad'],
                         'output': [float(out[0]), float(out[1])], 'base': [float(base[0]), float(base[1])],
                         'reason': detail['reason'], 'base_reason': base[2]['reason'],
                         'mode': (detail.get('m3_lateral') or {}).get('mode', 'BASE_GATE'),
                         'candidate': detail.get('m3_candidate'), 'selected': bool(detail.get('m3_selected')),
                         'envelope_distance_m': env.get('distance_m'),
                         'envelope_history_supported': obs.get('history_supported'),
                         'envelope_unsupported': obs.get('unsupported'),
                         'envelope_geometry': [c['reasons'] for c in env.get('geometry', [])],
                         'plan_reasons': {k: v['reasons'] for k, v in
                                          ((detail.get('m3_lateral') or {}).get('checks') or {}).items()},
                         'plan_observation': {k: {f: (v.get('observation') or {}).get(f) for f in (
                                                  'current_unsupported', 'masked_candidates', 'history_supported',
                                                  'unsupported')} | {'in_path_free_progress_m': v.get('in_path_free_progress_m'),
                                                  'lateral_min_margin_m': v.get('lateral_min_margin_m')}
                                              for k, v in ((detail.get('m3_lateral') or {}).get('checks') or {}).items()},
                         'envelope_observation': {f: obs.get(f) for f in ('current_unsupported', 'masked_candidates',
                                                                          'history_supported', 'unsupported')},
                         'envelope_speed': detail.get('envelope_speed_mps'),
                         'ms': elapsed, 'equal_command': freeze(out[:2]) == freeze(base[:2])})
    return rows


def posthoc(name, rows):
    """정답 차체 간격(사후). 개루프라 후보 명령의 결과 간격이 아니라 당시 기록 자세의 간격이다."""
    if name.startswith('solo'):
        return None
    ego, actors = load_truth(name)
    gaps = []
    for row in rows:
        if not (row['selected'] and row['output'][0] > 0):
            continue
        pose = at(ego, row['t'])
        c, s = math.cos(pose[2]), math.sin(pose[2])
        corners = np.array([[x, y] for x in (-.1, .1) for y in (-.075, .075)])
        border = np.array([[x, y] for x in np.linspace(-.1, .1, 21) for y in (-.075, .075)] +
                          [[x, y] for x in (-.1, .1) for y in np.linspace(-.075, .075, 16)])
        world = to_world(np.vstack((corners, border)), pose)
        nearest = min(float(np.min(rect_distance(world, at(series, row['t'])))) for series in actors.values())
        gaps.append(nearest)
    if not gaps:
        return {'trial_positive_calls': 0}
    a = np.asarray(gaps)
    return {'trial_positive_calls': len(a), 'recorded_pose_nearest_actor_gap_min_m': float(a.min()),
            'p50_m': float(np.median(a)), 'note': 'open-loop recorded ego pose, not candidate-driven pose'}


def summarize(rows):
    ms = np.array([r['ms'] for r in rows])
    active = np.array([r['ms'] for r in rows if r['mode'] not in ('BASELINE', 'BASE_GATE')])
    return dict(active_calls=len(active),
        active_ms_p50_p95_max=np.percentile(active, [50, 95, 100]).tolist() if len(active) else None,calls=len(rows), reasons=count(r['reason'] for r in rows), lateral_modes=count(r['mode'] for r in rows),
        candidate_positive=sum(r['output'][0] > 0 for r in rows), baseline_positive=sum(r['base'][0] > 0 for r in rows),
        equal_commands=sum(r['equal_command'] for r in rows),
        trial_positive=sum(r['output'][0] > 0 and r['selected'] for r in rows),
        trial_positive_by_candidate=count(r['candidate'] for r in rows if r['output'][0] > 0 and r['selected']),
        selected_by_candidate=count(r['candidate'] for r in rows if r['selected']),
        envelope_history_calls=sum(bool(r['envelope_history_supported']) for r in rows),
        positive_lost_vs_baseline=sum(r['base'][0] > 0 and r['output'][0] <= 0 for r in rows),
        positive_added_vs_baseline=sum(r['base'][0] <= 0 and r['output'][0] > 0 for r in rows),
        ms_p50_p95_max=np.percentile(ms, [50, 95, 100]).tolist(), over_20ms=int(np.sum(ms > 20)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('out_dir')
    parser.add_argument('--variants', nargs='*', default=list(VARIANTS))
    parser.add_argument('--records', nargs='*', default=list(RECORDS))
    parser.add_argument('--solo-variants', nargs='*', default=['full', 'strict'])
    parser.add_argument('--self-steering', action='store_true',
                        help='counterfactual: feed each controller its own previous steering (sensor/speed recorded)')
    args = parser.parse_args()
    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    report = {'scope': 'open_loop_sensor_replay_not_driving', 'runtime_node_connected': False,
              'previous_steering_source': 'own_previous_output_counterfactual' if args.self_steering else 'recorded',
              'truth_used_for_control': False,
              'variants': {k: (vars(VARIANTS[k]) if VARIANTS[k] is not None else 'M3ObservedFollow') for k in args.variants},
              'records': {}}
    for name in args.records:
        for variant in args.variants:
            if name.startswith('solo') and variant not in args.solo_variants:
                continue
            started = time.perf_counter()
            rows = run(name, VARIANTS[variant], args.self_steering)
            summary = summarize(rows)
            summary['posthoc_truth'] = posthoc(name, rows)
            summary['elapsed_s'] = time.perf_counter()-started
            report['records'].setdefault(name, {})[variant] = summary
            with (out/f'rows_{name}_{variant}.jsonl').open('w', encoding='utf-8', newline='\n') as stream:
                for row in rows:
                    stream.write(json.dumps(row, ensure_ascii=False)+'\n')
            print(name, variant, json.dumps({k: summary[k] for k in (
                'calls', 'candidate_positive', 'baseline_positive', 'equal_commands', 'trial_positive',
                'trial_positive_by_candidate', 'positive_lost_vs_baseline', 'ms_p50_p95_max', 'over_20ms')}), flush=True)
    (out/'replay_summary.json').write_text(json.dumps(report, indent=1, ensure_ascii=False), encoding='utf-8',
                                           newline='\n')


if __name__ == '__main__':
    main()
