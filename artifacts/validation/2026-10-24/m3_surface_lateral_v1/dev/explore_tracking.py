"""탐색용: 단독 주행 정답 중심선 오차 분포(직선/곡선). 평가 전용."""
import json, sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[5]
cl = np.genfromtxt(ROOT/'src/arena_gazebo/worlds/it_arena_official/centerline.csv', delimiter=',', names=True)
xy = np.column_stack([cl['x_m'], cl['y_m']]); t = np.gradient(xy, axis=0)
ang = np.unwrap(np.arctan2(t[:, 1], t[:, 0])); k = np.abs(np.gradient(ang)/.02)
for run in sys.argv[1:]:
    rows = [json.loads(l) for l in open(ROOT/run/'trajectory.jsonl', encoding='utf-8')]
    sel, t0 = [], None
    for r in rows:
        if not r['autonomy'].get('started'):
            continue
        tt = r['t'] if 't' in r else r['time']
        t0 = tt if t0 is None else t0
        v = r.get('ground_speed_mps', r['autonomy'].get('speed_command_mps') or 0)
        if tt-t0 < 3 or (v or 0) < .3:
            continue
        i = int(np.argmin(np.abs(cl['s_m']-r['s'])))
        sel.append((r['centerline_error_m'], k[max(0, i-20):i+20].max(), v))
    e = np.array(sel)
    st, cu = e[e[:, 1] < .2], e[e[:, 1] >= .2]
    f = lambda a: np.percentile(a[:, 0], [50, 95, 100]).round(4).tolist() if len(a) else None
    print(run, 'n', len(e), 'straight', len(st), f(st), 'curved', len(cu), f(cu), 'speed p50', np.median(e[:, 2]).round(3))
