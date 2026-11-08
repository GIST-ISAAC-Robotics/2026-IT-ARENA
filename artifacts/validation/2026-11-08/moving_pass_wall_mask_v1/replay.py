"""r31 실패 입력의 새 후보와 기존 두 프로필 보존 대조. 주행 결과 아님."""
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path
from arena_autonomy.follow_trace import replay_events, equal_output, comparable_decision

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PREV = HERE.parent/'moving_pass_faster_v1'
MODE = sys.argv[1]
assert MODE in ('legacy','candidate')
sources = ({'faster': PREV/'straight_moving_lead/run1/follow_decisions.jsonl'} if MODE == 'candidate' else {
    'slow': ROOT/'artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/run1/follow_decisions.jsonl',
    'medium': HERE.parent/'medium_pass_connected_v1/straight_moving_lead/run1/follow_decisions.jsonl'})


def find(value,key):
    if isinstance(value,dict):
        for k,v in value.items():
            if k == key:
                yield v
            yield from find(v,key)
    elif isinstance(value,list):
        for v in value:
            yield from find(v,key)


summaries = {}
for label,path in sources.items():
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    with path.open() as stream:
        result = replay_events(json.loads(line) for line in stream if line.strip())
    rows = result.pop('outcomes')
    summary = {k:v for k,v in result.items() if k!='mismatches'}
    summary.update(original_mismatches=len(result['mismatches']),source_sha256=digest,
        zero_commands=sum(row['speed_mps']==0. for row in rows),
        reasons=dict(Counter(row['details'].get('reason') for row in rows)),
        entry_reasons=dict(Counter(v for row in rows for v in find(row['details'].get('moving_pass',{}),'reason'))))
    if MODE == 'legacy':
        with (PREV/f'current_{label}_rows.jsonl').open() as stream:
            old = [json.loads(line) for line in stream]
        assert len(old)==len(rows)
        def out(row):
            return [row['speed_mps'],row['steering_rad'],row['details']]
        summary['change_from_r31'] = sum(not equal_output(comparable_decision(out(a)),comparable_decision(out(b)))
                                        for a,b in zip(old,rows))
    else:
        with (HERE/'candidate_rows.jsonl').open('w') as stream:
            for row in rows:stream.write(json.dumps(row,ensure_ascii=False)+'\n')
    summaries[label] = summary
    assert digest==hashlib.sha256(path.read_bytes()).hexdigest()
    print(label,json.dumps(summary,ensure_ascii=False),flush=True)
(HERE/f'{MODE}_summary.json').write_text(json.dumps(summaries,ensure_ascii=False,indent=2)+'\n')
if MODE == 'legacy':
    assert all(s['change_from_r31']==0 for s in summaries.values())
