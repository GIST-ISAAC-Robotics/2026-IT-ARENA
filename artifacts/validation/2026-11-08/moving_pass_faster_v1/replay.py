"""기존 저속/중속 재현과 faster 후보의 고정 센서 입력 비교. 새 주행 결과 아님."""
import copy
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path
from arena_autonomy.follow_trace import replay_events

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
MODE = sys.argv[1]
assert MODE in ('legacy', 'faster', 'faster_v2')
SOURCES = {
    'slow': ROOT/'artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/run1/follow_decisions.jsonl',
    'medium': HERE.parent/'medium_pass_connected_v1/straight_moving_lead/run1/follow_decisions.jsonl',
}


def find(data, key):
    if isinstance(data, dict):
        for k, v in data.items():
            if k == key:
                yield v
            yield from find(v, key)
    elif isinstance(data, list):
        for v in data:
            yield from find(v, key)


summaries = {}
for label, path in SOURCES.items():
    if MODE.startswith('faster') and label != 'medium':
        continue
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    with path.open() as stream:
        events = [json.loads(line) for line in stream if line.strip()]
    if MODE.startswith('faster'):
        events = copy.deepcopy(events)
        events[0]['data']['execution_config']['moving_pass_profile'] = 'faster_v1'
    result = replay_events(events)
    details = [o['details'] for o in result['outcomes']]
    summary = {k: v for k, v in result.items() if k not in ('outcomes', 'mismatches')}
    summary.update(source_sha256=digest, mismatches=len(result['mismatches']),
        max_command_mps=max(o['speed_mps'] for o in result['outcomes']),
        zero_commands=sum(o['speed_mps'] == 0. for o in result['outcomes']),
        top_reasons=dict(Counter(d.get('reason', '') for d in details)),
        entry_reasons=dict(Counter(str(v) for d in details for v in find(d.get('moving_pass', {}), 'reason'))))
    summaries[label] = summary
    (HERE/f'{MODE}_{label}_summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    if MODE.startswith('faster'):
        with (HERE/f'{MODE}_rows.jsonl').open('w') as out:
            for row in result['outcomes']:
                out.write(json.dumps(row, ensure_ascii=False)+'\n')
    assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
    print(label, json.dumps(summary, ensure_ascii=False), flush=True)
    if MODE == 'legacy':
        assert result['passed'], 'legacy replay changed'
