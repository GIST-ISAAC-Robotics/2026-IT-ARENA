"""r53 사본의 선택 순서만 바꿔 D2/B2 저장 입력을 대조한다. 실행 소스를 수정하지 않는다."""
import hashlib
import json
import sys
import types
from pathlib import Path
import numpy as np
from replay_tools import build_controller, read_events
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
source = ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py'
old = source.read_text()
start = old.index('                if not eligible and cfg.junction_continuity and carried is not None:')
end = old.index('                recent_confirmed = False', start)
block = old[start:end]
candidate = old[:start]+old[end:]
position = candidate.index('                if (not eligible and cfg.boundary_bridge\n')
candidate = candidate[:position]+block+candidate[position:]
module = types.ModuleType('arena_autonomy.junction_priority_probe')
module.__file__ = str(source)
sys.modules[module.__name__] = module
exec(compile(candidate, str(source), 'exec'), module.__dict__)
(A/'local_follow.priority_candidate.py').write_text(candidate, encoding='utf8', newline='\n')
summary = {}
for name in ('d_markers_blank2', 'b2_posts_candidates'):
    path = A/name/'follow_decisions.jsonl'
    before = hashlib.sha256(path.read_bytes()).hexdigest()
    rows=[]
    for event in read_events(path):
        data=event['data']
        if event['kind']=='header':
            base, mc=build_controller(data)
            changed=module.LocalFollow(module.FollowConfig(**data['follow_config']))
        elif event['kind']=='reset':
            base.reset(); changed.reset()
        elif event['kind']=='call':
            original=freeze(replay_call(base,data['input'],mc))
            output=freeze(replay_call(changed,data['input'],mc))
            paths=output[2].get('path_points')
            y=None
            if paths:
                pts=np.asarray(paths)
                y=float(pts[np.argmin(abs(pts[:,0]-.5)),1])
            rows.append(dict(t=data['input']['reference_s'],old_speed=original[0],speed=output[0],
                old_steering=original[1],steering=output[1],y_at_half=y,
                selection=output[2].get('boundary_selection'),wall=output[2].get('path_wall'),
                reason=output[2].get('reason'),
                same_command=equal_output(original[:2],output[:2]),
                baseline_matches=equal_output(comparable_decision(original),comparable_decision(data['output']))))
    (A/f'priority_{name}.json').write_text(json.dumps(rows,indent=1)+'\n')
    changed_rows=[r for r in rows if not r['same_command']]
    summary[name]=dict(calls=len(rows),baseline_mismatches=sum(not r['baseline_matches'] for r in rows),
        changed_commands=len(changed_rows),first_changes=changed_rows[:5],
        zero_before=sum(r['old_speed']<=1e-6 for r in rows),zero_after=sum(r['speed']<=1e-6 for r in rows),
        source_unchanged=hashlib.sha256(path.read_bytes()).hexdigest()==before)
(A/'priority_probe_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=1))
