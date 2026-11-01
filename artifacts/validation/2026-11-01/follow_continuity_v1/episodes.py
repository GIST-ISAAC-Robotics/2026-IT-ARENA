"""속도 명령이 G1 목표보다 크게 낮아진 구간을 원인별 사건으로 묶는다(사후 분석)."""
import csv
import json
import os
import sys
from pathlib import Path

import numpy as np


def load(name):
    path = Path(os.path.expanduser(f'~/arena_scratch/fc1/{name}.csv'))
    rows = list(csv.DictReader(path.open(encoding='utf-8')))
    for r in rows:
        for k in ('t', 'measured', 'out', 'g1', 'body', 'fes_cap', 'fixed_cap', 'axle_to_patch', 'rate', 'patch_x'):
            r[k] = float(r[k]) if r[k] not in ('', None) else float('nan')
    return rows


def episodes(rows, drop=.25):
    """G1 목표 대비 drop 이상 낮은 호출이 이어진 구간과 그 안의 제한 항목 시간."""
    out = []
    cur = None
    for a, b in zip(rows, rows[1:]):
        dt = b['t'] - a['t']
        if a['t'] < 0 or dt > .05:
            continue
        low = a['state'] == 'FOLLOW' and np.isfinite(a['g1']) and a['g1'] - a['out'] >= drop
        if low:
            if cur is None:
                cur = dict(start=a['t'], end=b['t'], bind={}, min_out=a['out'], body0=a['body'], g1_max=a['g1'],
                           axle0=a['axle_to_patch'])
            cur['end'] = b['t']
            cur['bind'][a['bind']] = cur['bind'].get(a['bind'], 0) + dt
            cur['min_out'] = min(cur['min_out'], a['out'])
            cur['g1_max'] = max(cur['g1_max'], a['g1'])
            cur['body1'] = b['body']
        elif cur is not None and a['t'] - cur['end'] > .12:
            out.append(cur)
            cur = None
    if cur is not None:
        out.append(cur)
    return out


if __name__ == '__main__':
    rows = load(sys.argv[1])
    limit = float(sys.argv[2]) if len(sys.argv) > 2 else 1e9
    rows = [r for r in rows if r['t'] <= limit]
    print(f"{'start':>6} {'end':>6} {'len':>5} {'g1max':>5} {'minout':>6} {'body0':>6} {'body1':>6} {'axle0':>6}  bind")
    for e in episodes(rows):
        bind = ' '.join(f"{k.replace('entry:','').replace('observed_entry_distance_reserve','reserve').replace('advancing_patch_close','closing')}={v:.2f}"
                        for k, v in sorted(e['bind'].items(), key=lambda x: -x[1]))
        print(f"{e['start']:6.2f} {e['end']:6.2f} {e['end']-e['start']:5.2f} {e['g1_max']:5.2f} {e['min_out']:6.3f} {e['body0']:6.3f} {e.get('body1',0):6.3f} {e['axle0']:6.3f}  {bind}")
    active = [r for r in rows if 0 <= r['t']]
    t = np.array([r['t'] for r in active])
    out = np.array([r['out'] for r in active])
    g1 = np.array([r['g1'] for r in active], dtype=float)
    valid = np.isfinite(g1) & (np.diff(t, append=t[-1]) <= .05)
    dt = np.diff(t, append=t[-1])
    print('command integral m', float(np.sum(out[valid] * dt[valid])), 'g1 integral m', float(np.sum(g1[valid] * dt[valid])))
