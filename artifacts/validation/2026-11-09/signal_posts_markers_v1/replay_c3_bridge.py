"""C3 원입력의 좁은 지주 벽참조 연결 선택 비교. 실제 주행 결과 아님."""
import copy,hashlib,json
from collections import Counter
from pathlib import Path
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
A=Path(__file__).resolve().parent
path=A/'c_stationary_posts/run3/follow_decisions.jsonl'
dest=A/'c3_bridge_replay';dest.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before_sha=sha(path);rows=[]
with (dest/'rows.jsonl').open('w') as stream:
    for event in read_events(path):
        d=event['data']
        if event['kind']=='header':
            baseline,mc=build_controller(d)
            changed=copy.deepcopy(d)
            assert not changed['follow_config']['narrow_occluder_bridge']
            changed['follow_config']['narrow_occluder_bridge']=True
            candidate,_=build_controller(changed)
        elif event['kind']=='reset':
            baseline.reset();candidate.reset()
        elif event['kind']=='call':
            before=freeze(replay_call(baseline,d['input'],mc))
            after=freeze(replay_call(candidate,d['input'],mc))
            row=dict(t=d['input']['reference_s'],before=before[:2],after=after[:2],
                same_command=equal_output(before[:2],after[:2]),
                baseline_matches=equal_output(comparable_decision(before),comparable_decision(d['output'])),
                before_reason=before[2].get('bypass',{}).get('reason'),
                after_reason=after[2].get('bypass',{}).get('reason'))
            rows.append(row);stream.write(json.dumps(dict(**row,detail=after[2]))+'\n')
summary=dict(calls=len(rows),baseline_mismatches=sum(not r['baseline_matches'] for r in rows),
    command_changes=sum(not r['same_command'] for r in rows),
    zeros_before=sum(r['before'][0]<=1e-6 for r in rows),
    zeros_after=sum(r['after'][0]<=1e-6 for r in rows),
    after_15=dict(calls=sum(r['t']>=15.1 for r in rows),
        zeros_before=sum(r['t']>=15.1 and r['before'][0]<=1e-6 for r in rows),
        zeros_after=sum(r['t']>=15.1 and r['after'][0]<=1e-6 for r in rows),
        candidate_reasons=dict(Counter(r['after_reason'] for r in rows if r['t']>=15.1))),
    source_sha256=before_sha,source_unchanged=before_sha==sha(path),
    limitation='Fixed sensor inputs; changed commands do not affect future poses in replay')
(dest/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary),flush=True)
