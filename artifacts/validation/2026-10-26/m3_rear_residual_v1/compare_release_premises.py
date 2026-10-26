"""저장 판단 입력에서 해제 관문 전제별 출력을 비교한다(개루프). 폐루프 진행·안전 판정이 아니다.

- `plain`: 현행 소스 그대로. `strict`: 전제 판본의 모든 전제 False. 두 판본 전체 출력 해시 동일을 요구한다.
- 정답 자세(ego/actor)는 해제 시각의 사후 해석에만 읽는다. 제어기에 넣지 않는다.
"""
import argparse
from bisect import bisect_left
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
for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts', 'tests'):
    sys.path.insert(0, str(ROOT/part))
sys.path.insert(0, str(HERE))

import numpy as np  # noqa: E402
from arena_autonomy.follow_trace import replay_call, freeze, equal_output  # noqa: E402
from arena_autonomy.local_follow import FollowConfig  # noqa: E402
from arena_autonomy.lidar_motion import MotionConfig  # noqa: E402
import arena_autonomy.m3_execution_trial as mx  # noqa: E402
import release_premises as rp  # noqa: E402

VARIANTS = {
    'plain': None,
    'strict': rp.ReleasePremise(),
    'diag_no_expiry': rp.ReleasePremise(no_expiry_diagnostic=True),
    'follow': rp.ReleasePremise(rear_follow=True),
    'follow_tail10': rp.ReleasePremise(rear_follow=True, tail_min_width_m=.010),
    'follow_tail20': rp.ReleasePremise(rear_follow=True, tail_min_width_m=.020),
    'base_mask': rp.ReleasePremise(mask_equivalence=True),
    'follow_base_mask': rp.ReleasePremise(rear_follow=True, mask_equivalence=True),
    # 아래 *_noexp는 1 s 만료와 전제의 효과를 분리하는 진단 전용 조합이다(해결 후보 아님).
    'follow_noexp': rp.ReleasePremise(rear_follow=True, no_expiry_diagnostic=True),
    'follow_tail10_noexp': rp.ReleasePremise(rear_follow=True, tail_min_width_m=.010, no_expiry_diagnostic=True),
    'follow_tail20_noexp': rp.ReleasePremise(rear_follow=True, tail_min_width_m=.020, no_expiry_diagnostic=True),
    'follow_base_mask_noexp': rp.ReleasePremise(rear_follow=True, mask_equivalence=True, no_expiry_diagnostic=True),
}
# 합집합 상태기 시제품(release_premises.UnionReleaseTrial). 확인 만료 대신 간격·수명·칸 상한을 쓴다.
UNION = {
    'union_strict': rp.ReleasePremise(),
    'union_follow': rp.ReleasePremise(rear_follow=True),
    'union_follow_tail10': rp.ReleasePremise(rear_follow=True, tail_min_width_m=.010),
    'union_follow_tail20': rp.ReleasePremise(rear_follow=True, tail_min_width_m=.020),
    'union_base_mask': rp.ReleasePremise(mask_equivalence=True),
    'union_follow_base_mask': rp.ReleasePremise(rear_follow=True, mask_equivalence=True),
}


def replay(path, variant):
    raw = path.read_bytes()
    events = [json.loads(x) for x in raw.decode('utf8').splitlines()]
    head = events[0]['data']
    controller = mx.M3ExecutionFollow(FollowConfig(**head['follow_config']),
        mx.ExecutionConfig(**head['execution_config']), allow_simulation_trial=True)
    motion = MotionConfig(**head['motion_config'])
    rows = []
    if variant in UNION:
        rp.install_union(controller, UNION[variant])
        premise = None
    else:
        premise = VARIANTS[variant]
    context = rp.use_premise(premise) if premise is not None else None
    if context:
        context.__enter__()
    try:
        for event in events[1:-1]:
            if event['kind'] == 'reset':
                controller.reset()
                rows.append({'reset': True})
                continue
            start = time.perf_counter()
            result = replay_call(controller, event['data']['input'], motion)
            cost = time.perf_counter()-start
            frozen = freeze(result)
            lateral = result[2].get('m3_lateral') or {}
            gate = lateral.get('release_gate') or {}
            threats = lateral.get('threats') or {}
            sides = {k: {kk: v.get(kk) for kk in ('reason', 'release', 'records', 'blocked_cells', 'possible_cells',
                                                   'corridor_cells', 'raw_blocked_cells', 'exempt_rear',
                                                   'exempt_tail_size', 'exempt_mask')}
                     for k, v in (gate.get('sides') or {}).items()}
            rows.append({'t': event['data']['input']['reference_s'],
                         'sha': hashlib.sha256(json.dumps(frozen, sort_keys=True).encode()).hexdigest(),
                         'speed': float(result[0]), 'steer': float(result[1]), 'reason': result[2].get('reason'),
                         'mode': lateral.get('mode'), 'released': bool(lateral.get('released_to_baseline')),
                         'present': [k for k, v in threats.items() if v.get('present')],
                         'gate_reason': gate.get('reason'), 'memory_records': gate.get('memory_records'),
                         'memory_blocked': gate.get('memory_blocked') or {}, 'sides': sides,
                         'matches_recorded': equal_output(frozen, event['data']['output']), 'cost_s': cost})
    finally:
        if context:
            context.__exit__(None, None, None)
    return rows, hashlib.sha256(raw).hexdigest(), hashlib.sha256(path.read_bytes()).hexdigest() == hashlib.sha256(raw).hexdigest()


def poses(path):
    rows = []
    with open(path, encoding='utf8') as f:
        for line in f:
            rows.append(json.loads(line))
    return rows


def at(series, t):
    stamps = [r[0] for r in series]
    k = min(max(bisect_left(stamps, t), 0), len(series)-1)
    if k and abs(series[k-1][0]-t) < abs(series[k][0]-t):
        k -= 1
    return series[k]


def box_corners(x, y, yaw, length=.20, width=.15):
    c, s = math.cos(yaw), math.sin(yaw)
    local = np.array([[length/2, width/2], [length/2, -width/2], [-length/2, -width/2], [-length/2, width/2]])
    return local@np.array([[c, s], [-s, c]])+[x, y]


def box_gap(a, b):
    """두 볼록 사각형의 최소 거리(겹치면 0). 사후 평가 전용 근사(꼭짓점-변)."""
    def seg(p, q, r):
        d = r-q
        t = np.clip(np.dot(p-q, d)/np.dot(d, d), 0, 1)
        return float(np.linalg.norm(p-(q+t*d)))
    best = math.inf
    for P, Q in ((a, b), (b, a)):
        for p in P:
            for k in range(4):
                best = min(best, seg(p, Q[k], Q[(k+1) % 4]))
    return best


def posthoc(run_dir, t):
    """해제 시각의 실제 actor 상대 위치(자차 기준 앞+/왼쪽+)와 차체 간격. 평가 전용."""
    ego = at(poses(run_dir/'ego_poses.jsonl'), t)
    extra = json.loads((run_dir/'additional_actors.json').read_text(encoding='utf8'))['poses']
    lead = poses(run_dir/'actor_poses.jsonl')
    out = {'ego_pose_t': ego[0]}
    c, s = math.cos(ego[3]), math.sin(ego[3])
    for name, series in [('lead', lead)]+list(extra.items()):
        a = at(series, t)
        dx, dy = a[1]-ego[1], a[2]-ego[2]
        out[name] = {'pose_t': a[0], 'forward_m': c*dx+s*dy, 'left_m': -s*dx+c*dy,
                     'body_gap_m': box_gap(box_corners(*ego[1:]), box_corners(*a[1:]))}
    return out


def job(args):
    path, variant = args
    rows, sha, unchanged = replay(path, variant)
    return path, variant, rows, sha, unchanged


def summary(rows):
    calls = [r for r in rows if not r.get('reset')]
    cost = np.array([r['cost_s'] for r in calls])*1000
    released = [r['t'] for r in calls if r['released']]
    first_block = next((r['t'] for r in calls if r['memory_blocked']), None)
    reasons = Counter(r['gate_reason'] for r in calls if r['gate_reason'])
    return {'calls': len(calls), 'positive': sum(r['speed'] > 0 for r in calls),
            'matches_recorded': sum(r['matches_recorded'] for r in calls), 'released_at': released,
            'memory_blocked_first_t': first_block,
            'memory_blocked_reasons': dict(Counter(v for r in calls for v in r['memory_blocked'].values())),
            'max_memory_records': max((r['memory_records'] or 0) for r in calls),
            'gate_reasons': dict(reasons),
            'last_present_t': max((r['t'] for r in calls if r['present']), default=None),
            'active_calls': sum(r['mode'] not in (None, 'BASELINE', 'OFF') for r in calls),
            'cost_ms': {'p50': float(np.quantile(cost, .5)), 'p95': float(np.quantile(cost, .95)),
                        'max': float(cost.max())}}


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--plan', type=Path, required=True, help='JSON {input_path: [variants]}')
    parser.add_argument('--workers', type=int, default=12)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.mkdir(parents=True)
    plan = json.loads(args.plan.read_text(encoding='utf8'))
    sources = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in (
        'src/arena_autonomy/arena_autonomy/m3_execution_trial.py',
        'artifacts/validation/2026-10-26/m3_rear_residual_v1/release_premises.py',
        'artifacts/validation/2026-10-26/m3_rear_residual_v1/compare_release_premises.py')}
    jobs = [(ROOT/p, v) for p, variants in plan.items() for v in variants]
    results = {}
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        for path, variant, rows, sha, unchanged in pool.map(job, jobs):
            name = path.parent.parent.name
            (args.output/f'{name}__{variant}.json').write_text(json.dumps(rows, ensure_ascii=False)+'\n',
                                                               encoding='utf-8', newline='\n')
            entry = results.setdefault(name, {'sha256': sha, 'input_unchanged': True, 'variants': {}, 'rows': {}})
            entry['input_unchanged'] &= unchanged
            entry['variants'][variant] = summary(rows)
            entry['rows'][variant] = rows
            print(json.dumps({name: {variant: entry['variants'][variant]}}, ensure_ascii=False), flush=True)
    report = {'scope': 'stored-input open-loop replay; premise variants are unconnected prototypes, not adopted',
              'sources_before': sources, 'inputs': {}}
    for name, entry in results.items():
        rows = entry.pop('rows')
        base = [r.get('sha') for r in rows.get('plain', [])]
        for variant, rs in rows.items():
            other = [r.get('sha') for r in rs]
            entry['variants'][variant]['output_differences_vs_plain'] = (
                None if not base else sum(a != b for a, b in zip(base, other)) + abs(len(base)-len(other)))
        run_dir = ROOT/next(p for p in plan if Path(p).parent.parent.name == name)
        run_dir = run_dir.parent
        for variant, value in entry['variants'].items():
            if value['released_at']:
                value['posthoc_at_first_release'] = posthoc(run_dir, value['released_at'][0])
        report['inputs'][name] = entry
    report['sources_after_unchanged'] = all(
        hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in sources.items())
    (args.output/'summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=1)+'\n',
                                            encoding='utf-8', newline='\n')
    assert report['sources_after_unchanged']
    assert all(e['input_unchanged'] for e in report['inputs'].values())
    for name, e in report['inputs'].items():
        if 'strict' in e['variants'] and 'plain' in e['variants']:
            assert e['variants']['strict']['output_differences_vs_plain'] == 0, name
