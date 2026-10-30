"""연결 전 소스로 schema5 기록을 재생해 불일치가 원래 있었는지 본다. 현재 src를 경로에 넣지 않는다."""
import json
from pathlib import Path

import arena_autonomy.follow_trace as trace
from arena_autonomy.follow_trace import replay_events

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
print('module',trace.__file__)
result={}
for name in ('run1','run2'):
    path=ROOT/f'artifacts/validation/2026-10-29/g2_handoff_gazebo_v1/{name}/g2_decisions.jsonl'
    with path.open(encoding='utf-8') as stream:
        events=[json.loads(line) for line in stream if line.strip()]
    report=replay_events(events)
    result[name]=dict(calls=report['calls'],matched=report['matched'],passed=report['passed'])
    print(name,result[name],flush=True)
(BASE/'replay_before.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
