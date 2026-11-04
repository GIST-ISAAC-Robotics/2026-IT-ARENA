"""출발 4 m의 시간 차이를 판단 사유·명령·경로로 나눈다. 11/1 선정 주행과 새 월드 run1을 같은 진행 위치로 비교. 평가 전용."""
from collections import Counter
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PRIOR = HERE.parents[1]/'2026-11-01/solo_corner_handoff_v1'
POST = (9.6886, 6.1064)


def load(run, metrics):
    begin = json.loads(metrics.read_bytes())['first_green_sample_s']
    rows = [r for r in map(json.loads, (run/'trajectory.jsonl').read_bytes().splitlines()) if r['light'] == 'green']
    t = np.array([r['time'] for r in rows])
    s = np.array([r['progress_m'] for r in rows])
    v = np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'], r['dynamics']['truth_lateral_mps']) for r in rows])
    x = np.array([r.get('x', r.get('pose', {}).get('x', np.nan)) for r in rows], dtype=float)
    y = np.array([r.get('y', r.get('pose', {}).get('y', np.nan)) for r in rows], dtype=float)
    calls = []
    for line in (run/'follow_decisions.jsonl').open(encoding='utf-8'):
        e = json.loads(line)
        if e['kind'] != 'call':
            continue
        i, o = e['data']['input'], e['data']['output']
        calls.append(dict(t=i['reference_s'], v=o[0], reason=o[2].get('reason'), path=o[2].get('observed_path_ahead_m'),
                          clearance=o[2].get('swept_clearance_m'), cap=o[2].get('speed_cap_reason', o[2].get('open_speed_reason')),
                          keys=sorted(o[2].keys())))
    return dict(begin=begin, t=t, s=s, v=v, x=x, y=y, calls=calls)


def window(run, lo, hi):
    keep = np.r_[True, np.diff(run['s']) > 0]
    ts, ss = run['t'][keep], run['s'][keep]
    a, b = np.interp([lo, hi], ss, ts)
    calls = [c for c in run['calls'] if a <= c['t'] <= b]
    reasons = Counter()
    for call in calls:
        reasons[call['reason']] += .02
    commands = np.array([c['v'] for c in calls]) if calls else np.array([0.])
    mask = (run['t'] >= a) & (run['t'] <= b)
    return dict(seconds=float(b-a), entry_since_green_s=float(a-run['begin']), v_min=float(run['v'][mask].min()),
                v_max=float(run['v'][mask].max()), command_mean=float(commands.mean()), command_max=float(commands.max()),
                reasons={k: round(v, 2) for k, v in reasons.most_common(6)},
                path_ahead_min=min((c['path'] for c in calls if c['path'] is not None), default=None),
                clearance_min=min((c['clearance'] for c in calls if c['clearance'] is not None), default=None))


def main():
    prior = load(PRIOR/'run1', PRIOR/'run1_metrics.json')
    now = load(HERE/'run1', HERE/'run1_metrics.json')
    edges = [(0., .5), (.5, 1.), (1., 1.5), (1.5, 2.), (2., 3.), (3., 4.)]
    result = dict(scope='평가 전용. 정답 자세·속도는 제어 입력이 아니다. 저장 입력 비교이며 반사실 주행이 아니다.',
                  sections=[dict(s0=a, s1=b, prior=window(prior, a, b), now=window(now, a, b)) for a, b in edges],
                  first_call_keys=now['calls'][0]['keys'] if now['calls'] else [])
    if np.isfinite(now['x']).all():
        distance = np.hypot(now['x']-POST[0], now['y']-POST[1])
        result['closest_center_to_post_m'] = float(distance.min())
    (HERE/'start_compare.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    for row in result['sections']:
        print(row['s0'], row['s1'], {k: row['prior'][k] for k in ('seconds', 'v_max', 'command_max', 'reasons')})
        print('    now', {k: row['now'][k] for k in ('seconds', 'v_max', 'command_max', 'reasons')})
    print(result.get('closest_center_to_post_m'), result['first_call_keys'][:40])


if __name__ == '__main__':
    main()
