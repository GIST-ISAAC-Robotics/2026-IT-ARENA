"""정지 뒤 자연 재개의 기록상 조건 변화. 물체 신원이나 인과를 추정해 확정하지 않는다."""
import json
from pathlib import Path
BASE = Path(__file__).resolve().parent
last = None
chosen = []
for line in (BASE/'run1_course/follow_decisions.jsonl').open(encoding='utf-8'):
    row = json.loads(line)
    if row['kind'] != 'call':
        continue
    data = row['data']
    stamp = data['input']['reference_s']
    detail = data['output'][2]
    reason = detail.get('selector_reason')
    if stamp < 100.:
        continue
    record = dict(t_s=stamp, command=data['output'][:2], reason=reason,
                  mode=detail.get('selector_mode'), retained=detail.get('retained_passage'),
                  sides=detail.get('selector_sides'), region=detail.get('selector_region'),
                  partial=(detail.get('partial_observation') or {}).get('reason'),
                  rays_450_454=data['input']['scan']['ranges'][450:455])
    if reason == 'large_motion_region_unobserved':
        last = record
    elif last and data['output'][0] > 0.:
        chosen.append(last)
        chosen.append(record)
        break
result = dict(scope='recorded before/after; no causal identity inference', transition=chosen)
(BASE/'late_release_transition.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(result, ensure_ascii=False))
