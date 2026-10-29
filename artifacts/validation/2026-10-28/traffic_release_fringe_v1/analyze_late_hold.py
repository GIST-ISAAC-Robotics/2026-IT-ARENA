"""고정한 후반 기록의 거절 단계별 요약. 실시간 상태 조회/재생 아님."""
from collections import Counter
import json
from pathlib import Path
BASE = Path(__file__).resolve().parent
source = BASE/'late_hold_prefix.jsonl'
out = BASE/'late_hold_analysis.json'
if out.exists():
    raise FileExistsError(out)
counts = {k: Counter() for k in ('reason','selector_reason','selector_mode','partial','anchor','latch')}
examples, window, calls, positive = {}, [], 0, 0
for line in source.open(encoding='utf-8'):
    event = json.loads(line)
    if event['kind'] != 'call':
        continue
    data = event['data']
    meta = data['output'][2]
    partial = meta.get('partial_observation') or {}
    anchor = partial.get('anchor_diagnostic') or {}
    values = {k: meta.get(k) for k in ('reason','selector_reason','selector_mode')}
    values.update(partial=partial.get('reason'),anchor=anchor.get('reason'),latch=partial.get('latch_reason'))
    for key, value in values.items():
        counts[key][str(value)] += 1
    stamp = data['input']['reference_s']
    calls += 1
    positive += int(data['output'][0] > 0.)
    label = '|'.join(str(values[k]) for k in ('reason','selector_reason','partial','anchor'))
    if label not in examples:
        examples[label] = dict(reference_s=stamp,command=data['output'][:2],speed=data['input']['speed_mps'],
            previous_steering=data['input']['previous_steering_rad'],partial=partial,
            selector={k:v for k,v in meta.items() if k.startswith('selector_') or k in ('same_path_gap','retained_passage')})
    if not window or window[-1]['label'] != label:
        window.append(dict(first_s=stamp,last_s=stamp,calls=1,label=label))
    else:
        window[-1]['last_s'] = stamp
        window[-1]['calls'] += 1
report = dict(scope='captured prefix counts; no stateful replay or completed-run verdict',calls=calls,positive=positive,
              counts={k:dict(v) for k,v in counts.items()},windows=window,examples=examples)
out.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(dict(calls=calls,positive=positive,counts=report['counts'],
                     windows=[r for r in window if r['calls']>=10]),ensure_ascii=False))
