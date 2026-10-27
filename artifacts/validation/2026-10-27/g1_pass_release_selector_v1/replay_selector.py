"""저장 판단 입력에서 상태 선택 시제품을 개루프로 대조한다. 새 선택대로 차량이 움직인 결과가 아니다.

- 기록 설정 그대로의 LocalFollow를 재생해 기록 출력과 같은지 먼저 확인한다(기준 출력 불변).
- 같은 호출에서 기존 G1 출력과 평행 후보 출력을 함께 얻는다. 기록이 평행 OFF면 같은 입력에 별도 ParallelLaneTrial을 적용한다.
- 선택기는 장면·두 출력·측정 속도·직전 조향만 받는다. 사후 정답은 해제 시각의 평가 주석에만 쓴다.
"""
import argparse
from bisect import bisect_right
from collections import Counter
from concurrent.futures import ProcessPoolExecutor
import hashlib
import json
import math
from pathlib import Path
import sys
import time

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts'):
    sys.path.insert(0, str(ROOT/part))
sys.path.insert(0, str(HERE))

from arena_autonomy.follow_trace import restore_call, freeze, equal_output, comparable_decision  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable  # noqa: E402
from arena_autonomy.local_follow import FollowConfig, LocalFollow  # noqa: E402
from arena_autonomy.parallel_lane_trial import ParallelLaneTrial  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402
import pass_release_selector as ps  # noqa: E402

INPUTS = {
    'run1_left': 'artifacts/validation/2026-10-27/g1_parallel_lane_v1/run1_left',
    'run2_right': 'artifacts/validation/2026-10-27/g1_parallel_lane_v1/run2_right',
    'stop_restart': 'artifacts/validation/2026-10-27/g1_partial_traffic_v1/run1_stop_restart',
    'run11_lap': 'artifacts/validation/2026-10-27/g1_partial_integration_v1/run11_lap',
}


def compact(output):
    speed, steer, detail = output
    return {'speed': float(speed), 'steer': float(steer), 'reason': detail.get('reason')}


def run(name):
    run_dir = ROOT/INPUTS[name]
    raw = (run_dir/'follow_decisions.jsonl').read_bytes()
    lines = raw.decode('utf-8').splitlines()
    head = json.loads(lines[0])['data']
    cfg = FollowConfig(**head['follow_config'])
    motion = MotionConfig(**head['motion_config'])
    follow = LocalFollow(cfg)
    captured = {}
    if follow.parallel_trial is not None:
        original = follow.parallel_trial.apply

        def apply(scene, output, speed, previous, dt):
            captured['baseline'] = output
            result = original(scene, output, speed, previous, dt)
            captured['parallel'] = result
            return result
        follow.parallel_trial.apply = apply
        extra = None
    else:
        extra = ParallelLaneTrial(cfg)
    selector = ps.PassReleaseSelector(cfg)
    rows, counts = [], Counter()
    for line in lines[1:-1]:
        event = json.loads(line)
        if event['kind'] == 'reset':
            follow.reset()
            if extra is not None:
                extra.reset()
            selector.invalidate('controller_reset_event')
            rows.append({'seq': event['seq'], 'reset': event['data'].get('reason')})
            continue
        data, scan, history = restore_call(event['data']['input'], motion)
        captured.clear()
        scene = None
        try:
            scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                                 epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
            output = follow.command(scene, history, data['speed_mps'], data['side'], data['previous_steering_rad'],
                                    data['dt_s'], **({'raw_scan': scan} if cfg.partial_observation_policy != 'strict'
                                                     else {}))
        except (MotionUnavailable, ValueError) as error:
            follow.reset()
            output = (0., 0., {'reason': 'follow_input_stop', 'behavior': 'HOLD', 'input_error': str(error)})
        recorded = event['data']['output']
        match = equal_output(comparable_decision(freeze(output)), comparable_decision(recorded))
        if extra is None:
            baseline = captured.get('baseline', output)
            parallel = captured.get('parallel', output)
        else:
            baseline = output
            parallel = (extra.apply(scene, output, data['speed_mps'], data['previous_steering_rad'], data['dt_s'])
                        if scene is not None else output)
        start = time.perf_counter()
        if scene is None:
            selected = selector._hold(data['previous_steering_rad'], {}, selector.invalidate('scene_unavailable'))
        else:
            selected = selector.select(scene, baseline, parallel, data['speed_mps'], data['previous_steering_rad'],
                                       data['dt_s'])
        cost = (time.perf_counter()-start)*1000
        sd = selected[2]
        evidence = sd.get('selector_evidence') or {}
        row = {'seq': event['seq'], 't': data['reference_s'], 'recorded_match': bool(match),
               'mode': sd.get('selector_mode', sd.get('selector_mode_before')), 'choice': sd.get('selector_choice'),
               'reason': sd.get('selector_reason'), 'out': compact(selected), 'baseline': compact(baseline),
               'parallel': compact(parallel), 'sides': sd.get('selector_sides'),
               'evidence': {k: v for k, v in evidence.items() if k != 'frame'},
               'excursion_m': sd.get('selector_excursion_m'), 'region': sd.get('selector_region'),
               'measured_speed_mps': data['speed_mps'], 'previous_steering_rad': data['previous_steering_rad'],
               'scene_age_s': None if scene is None else scene.t_ref_s-scene.source_first_s, 'cost_ms': cost}
        rows.append(row)
        counts[(row['mode'], row['choice'], row['reason'])] += 1
    unchanged = hashlib.sha256((run_dir/'follow_decisions.jsonl').read_bytes()).hexdigest() == hashlib.sha256(raw).hexdigest()
    return name, rows, counts, hashlib.sha256(raw).hexdigest(), unchanged


def poses(path):
    return [json.loads(line) for line in path.open(encoding='utf-8')]


def truth_at(name, t):
    """평가 전용: 해제 시각 actor의 자차 기준 위치(앞+/왼+)."""
    run_dir = ROOT/INPUTS[name]
    ego = poses(run_dir/'ego_poses.jsonl')
    e = ego[max(0, bisect_right([r[0] for r in ego], t)-1)]
    c, s = math.cos(e[3]), math.sin(e[3])
    actors = {'primary_lead': poses(run_dir/'actor_poses.jsonl')} if (run_dir/'actor_poses.jsonl').exists() else {}
    extra = run_dir/'additional_actors.json'
    if extra.exists():
        actors.update(json.loads(extra.read_text(encoding='utf-8'))['poses'])
    out = {}
    for actor, series in actors.items():
        if not series:
            continue
        a = series[max(0, bisect_right([r[0] for r in series], t)-1)]
        dx, dy = a[1]-e[1], a[2]-e[2]
        out[actor] = {'forward_m': round(c*dx+s*dy, 4), 'left_m': round(-s*dx+c*dy, 4)}
    return out


def summarize(name, rows, counts):
    calls = [r for r in rows if 'reset' not in r]
    guard = [r for r in calls if r['mode'] == 'GUARD']
    released = [r for r in calls if r['mode'] == 'RELEASED']
    transitions, previous = [], None
    for r in calls:
        if r['mode'] != previous:
            transitions.append({'t': r['t'], 'mode': r['mode'], 'reason': r['reason']})
            previous = r['mode']
    solo_mismatch = [r['t'] for r in calls if r['mode'] == 'SOLO' and r['choice'] == 'baseline'
                     and (abs(r['out']['speed']-r['baseline']['speed']) > 1e-12
                          or abs(r['out']['steer']-r['baseline']['steer']) > 1e-12)]
    guard_mismatch = [r['t'] for r in guard if r['choice'] == 'parallel'
                      and (abs(r['out']['speed']-r['parallel']['speed']) > 1e-12
                           or abs(r['out']['steer']-r['parallel']['steer']) > 1e-12)]
    fault_positive = [r['t'] for r in calls if r['choice'] == 'hold_fault' and r['out']['speed'] > 0]
    nonnormal_positive = [r['t'] for r in calls if r['reason'] == 'baseline_not_normal' and r['out']['speed'] > 0]
    first_release = next((r['t'] for r in released), None)
    out = {'calls': len(calls), 'resets': len(rows)-len(calls),
           'recorded_output_match': sum(r['recorded_match'] for r in calls),
           'modes': dict(Counter(r['mode'] for r in calls)),
           'choices': dict(Counter(r['choice'] for r in calls)),
           'reasons': dict(Counter(r['reason'] for r in calls)),
           'transitions': transitions[:40], 'transition_count': len(transitions),
           'solo_output_differs_from_baseline': solo_mismatch, 'guard_output_differs_from_parallel': guard_mismatch,
           'fault_positive_outputs': fault_positive, 'nonnormal_positive_outputs': nonnormal_positive,
           'positive': {'recorded_like': sum(r['out']['speed'] > 0 for r in calls),
                        'baseline': sum(r['baseline']['speed'] > 0 for r in calls),
                        'parallel': sum(r['parallel']['speed'] > 0 for r in calls),
                        'selected_above_parallel_cap': sum(r['out']['speed'] > .35+1e-9 for r in calls)},
           'cost_ms': {'max': max(r['cost_ms'] for r in calls),
                       'p95': sorted(r['cost_ms'] for r in calls)[int(.95*(len(calls)-1))]},
           'first_guard_t': next((r['t'] for r in guard), None), 'first_release_t': first_release}
    if first_release is not None:
        out['truth_at_first_release_evaluation_only'] = truth_at(name, first_release)
    first_guard = out['first_guard_t']
    if first_guard is not None:
        out['truth_at_first_guard_evaluation_only'] = truth_at(name, first_guard)
        trigger = next(r for r in guard)
        out['first_guard_evidence'] = trigger['evidence']
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=4)
    parser.add_argument('--inputs', nargs='+', choices=tuple(INPUTS), default=list(INPUTS))
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    sources = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in (
        'artifacts/validation/2026-10-27/g1_pass_release_selector_v1/pass_release_selector.py',
        'artifacts/validation/2026-10-27/g1_pass_release_selector_v1/replay_selector.py',
        'src/arena_autonomy/arena_autonomy/local_follow.py',
        'src/arena_autonomy/arena_autonomy/parallel_lane_trial.py',
        'src/arena_autonomy/arena_autonomy/m3_execution_trial.py',
        'src/arena_autonomy/arena_autonomy/lateral_intent.py')}
    report = {'scope': 'stored-input open-loop replay of an unconnected selector; the vehicle did not move along the '
                       'selected outputs; truth only for evaluation notes', 'sources': sources, 'inputs': {}}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for name, rows, counts, sha, unchanged in pool.map(run, args.inputs):
            with (args.output/f'{name}_rows.jsonl').open('w', encoding='utf-8', newline='\n') as stream:
                for row in rows:
                    stream.write(json.dumps(row, ensure_ascii=False)+'\n')
            entry = summarize(name, rows, counts)
            entry.update(input_sha256=sha, input_unchanged=unchanged)
            report['inputs'][name] = entry
            print(json.dumps({name: {k: entry[k] for k in ('calls', 'recorded_output_match', 'modes', 'choices',
                                                           'first_guard_t', 'first_release_t')}}), flush=True)
    report['sources_unchanged'] = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in sources.items())
    (args.output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=1)+'\n', encoding='utf-8',
                                            newline='\n')
    bad = [n for n, e in report['inputs'].items()
           if e['recorded_output_match'] != e['calls'] or e['solo_output_differs_from_baseline']
           or e['guard_output_differs_from_parallel'] or e['fault_positive_outputs'] or e['nonnormal_positive_outputs']
           or not e['input_unchanged']]
    print(json.dumps({'invariant_failures': bad, 'sources_unchanged': report['sources_unchanged']}))
    sys.exit(1 if bad or not report['sources_unchanged'] else 0)
