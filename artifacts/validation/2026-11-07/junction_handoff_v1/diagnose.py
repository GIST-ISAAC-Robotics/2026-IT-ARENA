"""r10 경계 인계 관찰; 주행 명령 변경 없이 후보 사유 기록."""
import json
from pathlib import Path
import numpy as np
from arena_autonomy import local_follow as follow
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'continuous_confirmation_v1/run1/follow_decisions.jsonl'
original=follow.local_path
rows=[]; current=None

def observed(points,side,*args,**kw):
    path,info=original(points,side,*args,**kw)
    if current is not None:
        current['wall_calls'].append(dict(side=side,n=len(points),info={k:v for k,v in info.items() if k!='path_points'},
           path=None if path is None else path.tolist()))
    return path,info
follow.local_path=observed
events=[json.loads(s) for s in SOURCE.read_text().splitlines()]
h=events[0]['data'];config=follow.FollowConfig(**h['follow_config']);motion=MotionConfig(**h['motion_config'])
ctl=follow.LocalFollow(config);calls=matched=0
for e in events[1:]:
    if e['kind']=='reset':ctl.reset()
    elif e['kind']=='call':
        q=e['data'];t=q['input']['reference_s']
        capture=24.9 <= t <= 28.5 and (current is None or True)
        current=dict(t=t,wall_calls=[],baseline_path=None if ctl.path is None else ctl.path.tolist(),
            baseline_source=ctl.path_source,corridor_path=None if ctl.corridor_observer.path is None else ctl.corridor_observer.path.tolist(),
            corridor_source=ctl.corridor_observer.path_source) if capture else None
        out=replay_call(ctl,q['input'],motion)
        matched+=int(np.allclose(out[:2],q['output'][:2],rtol=0,atol=1e-9));calls+=1
        if current and current['wall_calls']:
            current['output']=out;rows.append(current)
        current=None
(HERE/'diagnosis.json').write_text(json.dumps(rows,indent=1))
# An existing low-speed reacquire option is compared, not enabled in production here.
from dataclasses import replace
alt=follow.LocalFollow(replace(config,standing_wall_reacquire=True))
changed=[]
for e in events[1:]:
    if e['kind']=='reset':alt.reset()
    elif e['kind']=='call':
        q=e['data'];out=replay_call(alt,q['input'],motion)
        if not np.allclose(out[:2],q['output'][:2],rtol=0,atol=1e-9):
            changed.append(dict(t=q['input']['reference_s'],old=q['output'][:2],new=out[:2],reason=out[2].get('reason'),
              reference=out[2].get('bilateral_reference'),selection=out[2].get('boundary_selection')))
summary=dict(calls=calls,matched=matched,standing_option_changed=len(changed),standing_first=changed[:5],
 standing_late_positive=sum(r['t']>=28 and r['new'][0]>0 for r in changed),
 sample=[dict(t=r['t'],walls=[dict(side=w['side'],n=w['n'],info=w['info']) for w in r['wall_calls']],
  out=r['output'][:2],reason=r['output'][2].get('reason')) for r in rows if any(abs(r['t']-v)<.001 for v in [25.02,25.12,26.02,27.72,27.92,28.02,28.42])])
(HERE/'summary.json').write_text(json.dumps(summary,indent=2))
print(json.dumps(dict(calls=calls,matched=matched,standing_changed=len(changed),standing_late_positive=summary['standing_late_positive'])))
assert calls==matched
