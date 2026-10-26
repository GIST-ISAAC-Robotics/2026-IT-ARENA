"""해제 관문 전제의 합성 반례(이동 자차). 결과 JSON과 불변식 검사. 폐루프·물리·실차 판정이 아니다.

판단기는 `ExecutionLateralTrial`(현행 장부) 또는 `UnionReleaseTrial`(시제품)이며 자차는 판단과 무관하게 등속이다.
정답 물체는 광선 투사·사후 간격에만 쓴다. 불변식 위반 시 exit 1.
"""
import argparse
from concurrent.futures import ProcessPoolExecutor
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts', 'tests'):
    sys.path.insert(0, str(ROOT/part))
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
import arena_autonomy.m3_execution_trial as mx  # noqa: E402
from arena_autonomy.lateral_intent import LateralIntentConfig  # noqa: E402
from arena_autonomy.occupancy_timing import per_ray_padding  # noqa: E402
import release_premises as rp  # noqa: E402
import synthetic_world as sw  # noqa: E402

CFG = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass')
EXECUTION = mx.ExecutionConfig(release_to_baseline=True)
TRIALS = {
    'ledger_current': None,
    'union_strict': rp.ReleasePremise(),
    'union_follow': rp.ReleasePremise(rear_follow=True),
    'union_follow_tail10': rp.ReleasePremise(rear_follow=True, tail_min_width_m=.010),
    'union_follow_tail20': rp.ReleasePremise(rear_follow=True, tail_min_width_m=.020),
    'union_follow_base_mask': rp.ReleasePremise(rear_follow=True, mask_equivalence=True),
}
EGO_V = .20
T0, T1 = 1.0, 9.0


def follower(world, dx, dy):
    return lambda t: (world.ego_x(t)+dx, world.ego_y+dy)


def pass_body(side):
    # t=1.0에 자차 중심 0.6 m 뒤, 횡 0.13 m(자차 −/+0.09 m, 차체 간격 0.07 m), 0.45 m/s(물체 상계 0.5 이하)
    x1 = EGO_V*1.0-.6
    return sw.Body(.20, .15, lambda t: (x1+.45*(t-1.0), .13*side), 'A')


def world_for(case):
    side = -1. if case.startswith('right') else 1.
    w = sw.World(EGO_V, -.09*side)
    if case == 'same_side_run37':
        w = sw.World(0., -.09)
        w.bodies = [sw.Body(.20, .10, lambda t: (max(-.65, .3-.4*(t-1.)), .08), 'A'),
                    sw.Body(.20, .10, lambda t: (.30+.4*abs(t-3.5), .24), 'B')]
        return w
    w.bodies = [pass_body(side)]
    if case == 'left_pass_hidden_tail':
        # 14×6 mm, 왼쪽 뒤 모서리 바깥 2 mm·후면 평면 앞 2 mm. 항상 후방 마스크 쐐기 안(같은 스캔 확인).
        w.bodies.append(sw.Body(.014, .006, follower(w, -.105, .080), 'H_tail'))
    if case in ('left_pass_hidden_behind', 'reapproach'):
        h = follower(w, -.45, .13)
        if case == 'reapproach':
            ego_x = w.ego_x
            h = lambda t, f=h: f(t) if t < 7.5 else (ego_x(7.5)-.45+.5*(t-7.5), w.ego_y+.13)
        w.bodies.append(sw.Body(.10, .10, h, 'H_behind'))
    if case == 'left_pass_unknown_beside':
        w.dropout = lambda t, deg: 4.5 <= t < 7.0 and 40. <= deg <= 150.
    return w


CASES = ['left_pass', 'right_pass', 'left_pass_hidden_tail', 'left_pass_hidden_behind', 'same_side_run37',
         'left_pass_unknown_beside', 'reapproach', 'timeline_break', 'motion_gap']


def scene_at(case, world, t):
    scene = world.scene(t)
    if case == 'timeline_break' and t >= 4.5:
        scene = replace(scene, source_epoch=1, rays=tuple(replace(r, source_epoch=1) for r in scene.rays))
    return scene


def run(args):
    case, trial_name = args
    world = world_for('left_pass' if case in ('timeline_break', 'motion_gap') else case)
    premise = TRIALS[trial_name]
    trial = (mx.ExecutionLateralTrial(CFG, EXECUTION) if premise is None
             else rp.UnionReleaseTrial(CFG, EXECUTION, 1., premise))
    t1 = 10.0 if case == 'reapproach' else T1
    rows = []
    for k in range(int(round((t1-T0)/.1))+1):
        t = round(T0+.1*k, 3)
        scene = scene_at(case, world, t)
        gap = (4.5, 4.8) if case == 'motion_gap' else None
        motion = world.history(t+.01, gap)
        out = trial.step(scene, [], world.path(), .35, per_ray_padding(scene, .055, .5), history=motion)
        gate = out.get('release_gate') or {}
        digest = hashlib.sha256(json.dumps([(r.status, r.hit_xy_m, r.acquired_at_s) for r in scene.rays]).encode()).hexdigest()
        rows.append({'t': t, 'mode': out['mode'], 'released': bool(out.get('released_to_baseline')),
                     'active': trial.active, 'present': [n for n, v in (out.get('threats') or {}).items() if v['present']],
                     'gate_reason': gate.get('reason'), 'memory_blocked': gate.get('memory_blocked') or {},
                     'scan_sha256': digest,
                     'truth': {b.name: world.relative(b, t) for b in world.bodies}})
    return case, trial_name, rows


def summarize(rows):
    released = [r['t'] for r in rows if r['released']]
    out = {'released_at': released, 'final_mode': rows[-1]['mode'],
           'memory_blocked': next((r['memory_blocked'] for r in reversed(rows) if r['memory_blocked']), {}),
           'present_times': [r['t'] for r in rows if r['present']][:1]+[r['t'] for r in rows if r['present']][-1:]}
    if released:
        first = next(r for r in rows if r['released'])
        out['truth_at_first_release'] = first['truth']
        after = [r for r in rows if r['t'] > first['t']]
        out['reactivated_after_release'] = [r['t'] for r in after if r['active']][:1]
        if out['reactivated_after_release']:
            re = next(r for r in after if r['active'])
            out['truth_at_reactivation'] = re['truth']
    return out


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--workers', type=int, default=12)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    sources = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in (
        'src/arena_autonomy/arena_autonomy/m3_execution_trial.py',
        'artifacts/validation/2026-10-26/m3_rear_residual_v1/release_premises.py',
        'artifacts/validation/2026-10-26/m3_rear_residual_v1/synthetic_world.py',
        'artifacts/validation/2026-10-26/m3_rear_residual_v1/counterexamples.py')}
    jobs = [(c, t) for c in CASES for t in TRIALS]
    results = {}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for case, trial, rows in pool.map(run, jobs):
            results.setdefault(case, {})[trial] = {'summary': summarize(rows), 'rows': rows}
            print(json.dumps({case: {trial: results[case][trial]['summary']}}, ensure_ascii=False), flush=True)
    checks = {}

    def same_rows(a, b, keys=('mode', 'released', 'gate_reason')):
        return all(x['scan_sha256'] == y['scan_sha256'] and all(x[k] == y[k] for k in keys) for x, y in zip(a, b))

    # 1) 구별불가: 숨은 물체를 더한 장면의 모든 스캔이 원래와 같고, 모든 판단기의 출력도 같다.
    for hidden in ('left_pass_hidden_tail', 'left_pass_hidden_behind'):
        checks[f'{hidden}_scans_identical'] = all(
            x['scan_sha256'] == y['scan_sha256'] for x, y in
            zip(results['left_pass']['union_strict']['rows'], results[hidden]['union_strict']['rows']))
        checks[f'{hidden}_decisions_identical'] = all(
            same_rows(results['left_pass'][t]['rows'], results[hidden][t]['rows']) for t in TRIALS)
    # 2) 엄격(현행 장부·합집합 엄격)은 숨은 물체 장면에서 해제하지 않는다 → 같은 입력의 정상 통과도 해제 불가.
    for case in ('left_pass', 'left_pass_hidden_tail', 'left_pass_hidden_behind'):
        for t in ('ledger_current', 'union_strict'):
            checks[f'{case}_{t}_no_release'] = not results[case][t]['summary']['released_at']
    # 3) 시각·세대 단절과 운동 이력 공백은 어느 전제에서도 해제하지 않는다.
    for case in ('timeline_break', 'motion_gap'):
        checks[f'{case}_no_release_any'] = all(not results[case][t]['summary']['released_at'] for t in TRIALS)
    # 4) 무응답 결측 동안에는 어느 전제도 해제하지 않는다.
    checks['unknown_no_release_during_dropout'] = all(
        not any(4.5 <= x < 7.0 for x in results['left_pass_unknown_beside'][t]['summary']['released_at'])
        for t in TRIALS)
    # 5) 같은 측면 두 물체(run37형): 엄격은 해제하지 않는다(현행 회귀와 같은 기대).
    for t in ('ledger_current', 'union_strict'):
        checks[f'same_side_{t}_no_release'] = not results['same_side_run37'][t]['summary']['released_at']
    # 6) 재접근: 해제가 있었다면 그 뒤 숨은 물체가 다가올 때 접촉 전(간격>0) 다시 측방 시험이 켜진다.
    for t in TRIALS:
        s = results['reapproach'][t]['summary']
        if s['released_at']:
            re = s.get('truth_at_reactivation')
            checks[f'reapproach_{t}_reactivated_before_contact'] = bool(re and re['H_behind']['body_gap_m'] > 0)
    report = {'scope': 'synthetic moving-ego straight road; open-loop decision trial; not closed loop or physics',
              'sources': sources, 'checks': checks,
              'results': {c: {t: v['summary'] for t, v in r.items()} for c, r in results.items()}}
    report['sources_unchanged'] = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in sources.items())
    args.output.mkdir(parents=True)
    (args.output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=1)+'\n', encoding='utf-8',
                                            newline='\n')
    (args.output/'rows.json').write_text(json.dumps({c: {t: v['rows'] for t, v in r.items()}
                                                     for c, r in results.items()}, ensure_ascii=False)+'\n',
                                         encoding='utf-8', newline='\n')
    print(json.dumps({'checks': checks, 'sources_unchanged': report['sources_unchanged']}, ensure_ascii=False))
    sys.exit(0 if all(checks.values()) and report['sources_unchanged'] else 1)
