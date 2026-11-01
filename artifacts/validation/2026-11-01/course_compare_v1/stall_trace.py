"""BYPASS 정체 구간의 판단 이유를 시간순 구간으로 묶는다(사후 분석)."""
import json
from pathlib import Path
import sys

run = Path(sys.argv[1])
a, b = float(sys.argv[2]), float(sys.argv[3])
green = json.loads((run / 'report.json').read_bytes())['case_window']['green_sim_s']


def text(value):
    return None if value is None else str(value)[:60]


last = None
start = None
n = 0
vmax = 0.
with (run / 'follow_decisions.jsonl').open('rb') as f:
    for line in f:
        e = json.loads(line)
        if e['kind'] != 'call':
            continue
        d = e['data']
        t = d['input']['reference_s'] - green
        if t < a or t > b:
            continue
        v, _, q = d['output']
        byp = q.get('bypass') or {}
        y = q.get('relative_yield') or {}
        sel = byp.get('selected') or {}
        key = (q.get('handoff_state'), q.get('reason'), text(byp.get('reason')), byp.get('entry_phase'), byp.get('guide_mode'),
               bool(y.get('final_command_clipped')), text(y.get('reason')), text(sel.get('reason')), v < .001)
        if key != last:
            if last is not None:
                print(f'{start:7.2f} n={n:4d} vmax={vmax:.2f} {last}')
            last, start, n, vmax = key, t, 0, 0.
        n += 1
        vmax = max(vmax, v)
print(f'{start:7.2f} n={n:4d} vmax={vmax:.2f} {last}')
