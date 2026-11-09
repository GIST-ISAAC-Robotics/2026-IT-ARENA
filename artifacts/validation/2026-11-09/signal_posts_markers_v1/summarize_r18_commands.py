"""기존 재생 행과 원 기록의 명령만 별도로 대조. 추가 제어기 실행 없음."""
import json
from pathlib import Path
from replay_tools import read_events
from arena_autonomy.follow_trace import equal_output
A=Path(__file__).resolve().parent;R=A.parents[3]
rows=json.loads((A/'r56_r18_rows.json').read_text())
record=[e['data'] for e in read_events(R/'artifacts/validation/2026-11-08/rear_face_motion_connected_v1/persistent_block/run1/follow_decisions.jsonl') if e['kind']=='call']
assert len(rows)==len(record)
changes=[]
for row,event in zip(rows,record):
    assert abs(row['t']-event['input']['reference_s'])<1e-8
    if not equal_output(row['before'],event['output'][:2]):
        changes.append(dict(t=row['t'],current=row['before'],recorded=event['output'][:2]))
out=dict(calls=len(rows),r56_vs_r55_command_changes=sum(not r['same'] for r in rows),
    r55_vs_historical_command_changes=len(changes),first_changes=changes[:5],
    scope='r56 effect separated from historical r18 vs current r55 changes')
(A/'r18_command_comparison.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=1))
