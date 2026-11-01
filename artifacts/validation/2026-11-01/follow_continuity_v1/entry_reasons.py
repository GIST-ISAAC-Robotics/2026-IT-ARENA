"""앞차 정지 후 우회 진입을 막은 판단 이유를 집계한다(사후 분석)."""
from collections import Counter
import json
import sys
from pathlib import Path

run = Path(sys.argv[1])
a, b = float(sys.argv[2]), float(sys.argv[3])
green = json.loads((run / 'report.json').read_bytes())['case_window']['green_sim_s']
count = Counter()
samples = {}
with (run / 'follow_decisions.jsonl').open('rb') as f:
    for line in f:
        e = json.loads(line)
        if e['kind'] != 'call':
            continue
        d = e['data']
        t = d['input']['reference_s'] - green
        if t < a or t > b:
            continue
        q = d['output'][2]
        p = q.get('persistence') or {}
        byp = q.get('bypass') or {}
        adm = q.get('entry_admission') or {}
        key = (q.get('handoff_state'), bool(p.get('ready')), byp.get('reason'), adm.get('admitted'), adm.get('reason'))
        count[key] += 1
        samples.setdefault(key, (round(t, 2), round(d['input']['speed_mps'], 3), round(abs(d['input']['previous_steering_rad']), 3),
                                 (q.get('follow_entry_space') or {}).get('rear_axle_to_patch_m')))
for k, v in count.most_common():
    print(v, k, samples[k])
