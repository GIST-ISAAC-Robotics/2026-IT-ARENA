"""연결 뒤 기존 G2/G1G2 판단 기록을 현재 코드로 재생한다. 주행 아님."""
import json
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from arena_autonomy.follow_trace import replay_events

PATHS=['artifacts/validation/2026-10-29/g2_entry_gazebo_v1/run1/g2_decisions.jsonl',
    'artifacts/validation/2026-10-29/g2_entry_left_v1/run1/g2_decisions.jsonl']
for folder in ('2026-10-29/g2_handoff_gazebo_v1',):
    PATHS+=[p.relative_to(ROOT).as_posix() for p in sorted((ROOT/'artifacts/validation'/folder).glob('*/g*_decisions.jsonl'))][:2]
result={}
for path in PATHS:
    target=ROOT/path
    if not target.exists():
        result[path]=dict(missing=True);continue
    with target.open(encoding='utf-8') as stream:
        events=[json.loads(line) for line in stream if line.strip()]
    report=replay_events(events)
    result[path]=dict(schema=events[0]['data'].get('schema'),calls=report['calls'],matched=report['matched'],passed=report['passed'])
    print(path,result[path],flush=True)
(BASE/'replay_old.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
raise SystemExit(0 if all(r.get('passed') for r in result.values() if not r.get('missing')) and any(r.get('passed') for r in result.values()) else 1)
