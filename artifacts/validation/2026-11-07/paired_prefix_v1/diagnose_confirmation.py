"""r8 정지의 시간상 경로 확인을 출력 변경 없이 관찰한다."""
import json
from pathlib import Path
import numpy as np
from arena_autonomy.corridor_observer import CorridorObserver
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig

HERE=Path(__file__).resolve().parent
original=CorridorObserver.evaluate
rows=[]
def observed(self,scene,*a,**kw):
    before=dict(path_ref=self.path_ref,path_source=self.path_source,confirmations=self.confirmations,
                path=None if self.path is None else self.path.tolist())
    result=original(self,scene,*a,**kw)
    if scene.t_ref_s>=27.0 and (result.get('fresh') or scene.t_ref_s<27.5):
        rows.append(dict(t=scene.t_ref_s,before=before,result=result,
                         after_path=None if self.path is None else self.path.tolist()))
    return result
CorridorObserver.evaluate=observed
events=[json.loads(s) for s in (HERE/'run1/follow_decisions.jsonl').read_text().splitlines()]
header=events[0]['data'];ctl=LocalFollow(FollowConfig(**header['follow_config']));motion=MotionConfig(**header['motion_config'])
count=matched=0
for e in events[1:]:
    if e['kind']=='reset':ctl.reset()
    elif e['kind']=='call':
        q=e['data'];out=replay_call(ctl,q['input'],motion)
        matched+=int(abs(out[0]-q['output'][0])<1e-9 and abs(out[1]-q['output'][1])<1e-9);count+=1
(HERE/'confirmation_detail.json').write_text(json.dumps(rows,indent=1),encoding='utf-8')
summary=dict(calls=count,commands_matched=matched,late_rows=len(rows),
    samples=[{k:r[k] for k in ('t','result')}|{'before_source':r['before']['path_source'],
        'before_confirmations':r['before']['confirmations']} for r in rows if 27.5<r['t']<28.0])
for r in summary['samples']:r['result']={k:v for k,v in r['result'].items() if k!='path'}
(HERE/'confirmation_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary));assert count==matched
