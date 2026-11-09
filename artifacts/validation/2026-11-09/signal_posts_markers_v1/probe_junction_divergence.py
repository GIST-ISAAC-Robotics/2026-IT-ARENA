"""r54의 불필요한 벽 고정 없이 실제 분기 불일치 때만 반대 벽을 유지하는 사본 비교."""
from pathlib import Path
import sys
import types
import json
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
A=Path(__file__).resolve().parent
ROOT=A.parents[3]
path=ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py'
s=path.read_text()
needle="""if sufficient(opposite, other) and path_agrees(opposite, carried, cfg.boundary_agreement_m):
                        candidate, info, candidate_wall, eligible = opposite, other, other_side, True
                        detail['boundary_selection'] = 'junction_retained_wall_refresh'"""
replacement="""if (sufficient(opposite, other) and path_agrees(opposite, carried, cfg.boundary_agreement_m)
                            and (not sufficient(preferred, preferred_info)
                                 or not path_agrees_arc(preferred, opposite, cfg.boundary_agreement_m))):
                        candidate, info, candidate_wall, eligible = opposite, other, other_side, True
                        detail['boundary_selection'] = 'junction_retained_wall_refresh'"""
assert s.count(needle)==1
s=s.replace(needle,replacement)
(A/'local_follow.divergence_candidate.py').write_text(s,encoding='utf8',newline='\n')
m=types.ModuleType('arena_autonomy.junction_divergence_probe');sys.modules[m.__name__]=m
exec(compile(s,str(path),'exec'),m.__dict__)
summary={}
for name in ('b2_posts_candidates','d_markers_blank2'):
    rows=[]
    for event in read_events(A/name/'follow_decisions.jsonl'):
        d=event['data']
        if event['kind']=='header':
            _,mc=build_controller(d)
            c=m.LocalFollow(m.FollowConfig(**d['follow_config']))
        elif event['kind']=='reset': c.reset()
        elif event['kind']=='call':
            out=freeze(replay_call(c,d['input'],mc));rec=d['output']
            rows.append(dict(t=d['input']['reference_s'],speed=out[0],steering=out[1],
                same=equal_output(out[:2],rec[:2]),wall=out[2].get('path_wall'),
                selection=out[2].get('boundary_selection'),reason=out[2].get('reason')))
    (A/f'divergence_{name}_rows.json').write_text(json.dumps(rows,indent=1)+'\n')
    diff=[r for r in rows if not r['same']]
    summary[name]=dict(calls=len(rows),changed=len(diff),first=diff[:2],
        d2_window=[r for r in rows if 29.7<r['t']<30.3][::5] if name.startswith('d_') else None)
(A/'divergence_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=1))
