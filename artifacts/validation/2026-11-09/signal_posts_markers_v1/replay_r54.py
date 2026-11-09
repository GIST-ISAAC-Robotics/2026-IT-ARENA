"""r54 연결과 r53 원본 사본의 D2/B2 저장 입력 대조. 궤적 성공 판정은 아니다."""
import hashlib
import json
import sys
import types
import zipfile
from pathlib import Path
from replay_tools import build_controller, read_events
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision

A=Path(__file__).resolve().parent
old=types.ModuleType('arena_autonomy.junction_before_r54')
sys.modules[old.__name__]=old
with zipfile.ZipFile(A/'before_r54.zip') as z:
    code=z.read('src/arena_autonomy/arena_autonomy/local_follow.py')
exec(compile(code,'before_r54/local_follow.py','exec'),old.__dict__)
summary={}
for name in ('d_markers_blank2','b2_posts_candidates'):
    path=A/name/'follow_decisions.jsonl'
    before=hashlib.sha256(path.read_bytes()).hexdigest()
    rows=[]
    for event in read_events(path):
        d=event['data']
        if event['kind']=='header':
            current,mc=build_controller(d)
            baseline=old.LocalFollow(old.FollowConfig(**d['follow_config']))
        elif event['kind']=='reset':
            current.reset(); baseline.reset()
        elif event['kind']=='call':
            a=freeze(replay_call(baseline,d['input'],mc)); b=freeze(replay_call(current,d['input'],mc))
            rows.append(dict(t=d['input']['reference_s'],before_speed=a[0],speed=b[0],
                before_steering=a[1],steering=b[1],same_command=equal_output(a[:2],b[:2]),
                baseline_matches=equal_output(comparable_decision(a),comparable_decision(d['output'])),
                selection=b[2].get('boundary_selection'),wall=b[2].get('path_wall'),reason=b[2].get('reason'),
                switch_rejected=b[2].get('junction_wall_switch_rejected')))
    (A/f'r54_{name}_rows.json').write_text(json.dumps(rows,indent=1)+'\n')
    changed=[r for r in rows if not r['same_command']]
    summary[name]=dict(calls=len(rows),baseline_mismatches=sum(not r['baseline_matches'] for r in rows),
        changed_commands=len(changed),first_changes=changed[:3],
        zero_before=sum(r['before_speed']<=1e-6 for r in rows),zero_after=sum(r['speed']<=1e-6 for r in rows),
        source_unchanged=hashlib.sha256(path.read_bytes()).hexdigest()==before)
(A/'r54_replay_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=1))
