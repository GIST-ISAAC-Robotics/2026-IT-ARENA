"""r8 저장 주행에서 경로 샘플 간격의 영향을 분리. 공용 소스 변경 없음."""
import json
from pathlib import Path
import numpy as np
import arena_autonomy.corridor_observer as co
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import FollowConfig,LocalFollow,path_agrees
from arena_autonomy.lidar_motion import MotionConfig
from candidate import agrees

HERE=Path(__file__).resolve().parent
cases={}
x=np.linspace(.04,.35,8);p=np.c_[x,.2*x]
for name,q,expected in [('same_shifted_sampling',p+[.001,.0002],True),
                       ('lateral_mismatch',p+[0.,.09],False),
                       ('too_short',p[:5],False),('reverse',p[::-1],False),
                       ('too_few',p[::2],False),('past_window',p+[1.,0.],False)]:
    cases[name]=dict(actual=agrees(p,q,.08),expected=expected)
    assert cases[name]['actual']==expected,name
events=[json.loads(s) for s in (HERE.parent/'paired_prefix_v1/run1/follow_decisions.jsonl').read_text().splitlines()]
header=events[0]['data'];ctl=LocalFollow(FollowConfig(**header['follow_config']));motion=MotionConfig(**header['motion_config'])
assert ctl.config.piecewise_wall_trial
used=[]
def compare(a,b,t):
    old=path_agrees(a,b,t)
    new=old or agrees(a,b,t)
    if not old and new:used.append(True)
    return new
co.path_agrees=compare
rows=[]
for e in events[1:]:
    if e['kind']=='reset':ctl.reset()
    elif e['kind']=='call':
        q=e['data'];out=replay_call(ctl,q['input'],motion)
        rows.append(dict(t=q['input']['reference_s'],old_speed=q['output'][0],old_steer=q['output'][1],
                         speed=out[0],steer=out[1],reason=out[2].get('reason')))
co.path_agrees=path_agrees
summary=dict(calls=len(rows),comparisons_added=len(used),
    changed_commands=sum(abs(r['speed']-r['old_speed'])>1e-9 or abs(r['steer']-r['old_steer'])>1e-9 for r in rows),
    late_positive=sum(r['speed']>0 for r in rows if r['t']>=29),
    first_new_positive=next((r for r in rows if r['old_speed']==0 and r['speed']>0),None),
    counterexamples=cases,scope='frozen trajectory only; not new lap proof')
(HERE/'shadow_rows.json').write_text(json.dumps(rows,indent=1),encoding='utf-8')
(HERE/'shadow_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary))
