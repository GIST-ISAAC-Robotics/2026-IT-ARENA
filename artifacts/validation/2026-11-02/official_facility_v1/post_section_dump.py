"""기둥 구간 판단 필드 추출. 평가 전용."""
import json
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
name = sys.argv[1] if len(sys.argv) > 1 else 'run1'
lo, hi = float(sys.argv[2]), float(sys.argv[3])
rows = [r for r in map(json.loads, (HERE/name/'trajectory.jsonl').read_bytes().splitlines()) if r['light'] == 'green']
t = np.array([r['time'] for r in rows])
s = np.array([r['progress_m'] for r in rows])
keep = np.r_[True, np.diff(s) > 0]
a, b = np.interp([lo, hi], s[keep], t[keep])
last = None
for line in (HERE/name/'follow_decisions.jsonl').open(encoding='utf-8'):
    e = json.loads(line)
    if e['kind'] != 'call':
        continue
    time, output = e['data']['input']['reference_s'], e['data']['output']
    if not a <= time <= b:
        continue
    d = output[2]
    brief = {k: d.get(k) for k in d if isinstance(d.get(k), (bool, str, int, float, type(None)))}
    key = (round(output[0], 1), d.get('boundary_selection'), d.get('corridor_selected'), d.get('open_route'), d.get('reason'))
    if key != last:
        last = key
        print(round(time, 3), round(float(np.interp(time, t[keep], s[keep])), 3), round(output[0], 3), json.dumps(brief, ensure_ascii=False))
