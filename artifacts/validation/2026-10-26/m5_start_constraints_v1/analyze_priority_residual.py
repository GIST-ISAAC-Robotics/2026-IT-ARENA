"""저장 결과만 요약한다. 원인 분리용이며 후보 제어기를 재실행하지 않는다."""
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

base = Path(__file__).resolve().parent
source = base/'run20_straight_dynamics/data/report.json'
raw = source.read_bytes()
report = json.loads(raw)
results = []
for case in report['cases']:
    calls = case['calls']
    first = next((i for i, r in enumerate(calls) if r['reason']=='m3_command_unobserved'), None)
    selected = Counter()
    residual = Counter()
    for row in calls:
        meta = row['metadata']
        if row['request_mps'] > 0:
            selected[meta.get('m3_candidate', 'baseline')] += 1
        if row['reason'] == 'm3_command_unobserved':
            lateral = meta.get('m3_lateral', {})
            present = any(t['present'] for t in lateral.get('threats', {}).values())
            residual['threat_present' if present else 'threat_absent'] += 1
    chain = []
    if first is not None:
        for row in calls[max(0, first-6):first+3]:
            meta = row['metadata']; lateral = meta.get('m3_lateral', {})
            envelope = meta.get('m3_envelope', {})
            chain.append(dict(t_s=row['t_s'], frame=row['frame'], reason=row['reason'],
                request_mps=row['request_mps'], steering_deg=math.degrees(row['request_steering']),
                candidate=meta.get('m3_candidate'), measured_mps=row['measured_mps'],
                threats={k:v['present'] for k,v in lateral.get('threats',{}).items()},
                checks={k:dict(eligible=v.get('eligible_for_trial'), reasons=v.get('reasons'),
                    command=v.get('command')) for k,v in lateral.get('checks',{}).items()},
                unsupported=envelope.get('observation',{}).get('unsupported'),
                geometry_ok=envelope.get('geometry_ok'),
                d0=lateral.get('state',{}).get('d0')))
    results.append(dict(mode={k:case['summary'][k] for k in ('rounded','fault','straight_priority')},
        positive_candidates=dict(selected), unobserved=dict(residual), first_unobserved_chain=chain))
out = dict(source_sha256=hashlib.sha256(raw).hexdigest(), input_unchanged=source.read_bytes()==raw,
    scope='saved evidence only; causal counterfactual not yet tested', cases=results)
with (base/'priority_residual.json').open('x',encoding='utf-8') as f:
    json.dump(out,f,ensure_ascii=False,indent=2)
print(json.dumps([dict(mode=c['mode'],positive_candidates=c['positive_candidates'],
    unobserved=c['unobserved']) for c in results]))
