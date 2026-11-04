"""종료 뒤 최장 5 cm 정체 구간만 대조한다. 당시 실행/판정을 변경하지 않는다."""
from collections import Counter
import json
from pathlib import Path

HERE=Path(__file__).resolve().parent
RUN=HERE/'run1'
j=json.loads((HERE/'independent_review.json').read_bytes())
w=j['dense_progress']['max_nonprogress']
lo,hi=w['from_s'],w['to_s']
rows=[p for p in map(json.loads,(RUN/'trajectory.jsonl').read_bytes().splitlines()) if lo<=p['t']<=hi]
def counts(rows,key):
    return dict(Counter(key(r) for r in rows))
calls=[]
with (RUN/'follow_decisions.jsonl').open(encoding='utf8') as f:
    for line in f:
        e=json.loads(line)
        if e['kind']!='call':continue
        d=e['data'];t=d['input']['reference_s']
        if t>hi:break
        if t<lo:continue
        v,steer,outer=d['output'];n=outer.get('normal') or {}
        calls.append((v,outer.get('reason'),n.get('reason'),(n.get('current_lane_preference') or {}).get('applied',False)))
out=dict(window=w,summary_samples=len(rows),progress_range_m=[min(r['progress_m'] for r in rows),max(r['progress_m'] for r in rows)],
    safety_reasons=counts(rows,lambda r:r['safety_reason']),mcu_states=counts(rows,lambda r:r['mcu_state']),
    calls=len(calls),raw_zero=sum(c[0]<=1e-9 for c in calls),outer_reasons=counts(calls,lambda c:c[1]),
    normal_reasons=counts(calls,lambda c:c[2]),lane_preference_calls=sum(bool(c[3]) for c in calls),
    scope='저장된 시뮬 시각 창에서의 동시 관측. 창 이전 궤적의 영향이나 상대 지연과의 인과는 분리하지 않음.')
with (HERE/'stall_context.json').open('x',encoding='utf8') as f:json.dump(out,f,ensure_ascii=False,indent=2)
print(json.dumps(out,ensure_ascii=True))
