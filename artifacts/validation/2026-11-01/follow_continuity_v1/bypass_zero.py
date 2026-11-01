"""BYPASS 중 낮은 명령 구간의 이유를 원본 판단에서 뽑는다(사후 분석)."""
import json
import sys
from pathlib import Path

run = Path(sys.argv[1])
green = json.loads((run / 'report.json').read_bytes())['case_window']['green_sim_s']
last = None
start = None
n = 0
with (run / 'follow_decisions.jsonl').open('rb') as f:
    for line in f:
        e = json.loads(line)
        if e['kind'] != 'call':
            continue
        d = e['data']
        t = d['input']['reference_s'] - green
        if t < 6 or t > 15:
            continue
        v, _, q = d['output']
        b = q.get('bypass') or {}
        y = q.get('relative_yield') or {}
        key = (q.get('handoff_state'), v < .05, q.get('reason'), b.get('reason'), b.get('entry_phase'),
               bool(y.get('final_command_clipped')), y.get('reason'))
        if key != last:
            if last is not None:
                print(f'{start:6.2f} n={n:3d} {last}')
            last, start, n = key, t, 0
        n += 1
print(f'{start:6.2f} n={n:3d} {last}')
