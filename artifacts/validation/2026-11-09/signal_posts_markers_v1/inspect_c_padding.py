"""C1 저장 재생의 실제 시작 외곽 차단 반환을 진단한다. 패딩/출력은 변경하지 않는다."""
import json
from pathlib import Path
import numpy as np
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
from arena_autonomy.bypass_handoff_corridor_trial import ForecastGuide
A=Path(__file__).resolve().parent
source=A/'c_stationary_posts/run1/follow_decisions.jsonl'
original=ForecastGuide._padding
captures=[]

def traced(self,scene,config,side,wall,growth_s=0.):
    padding,evidence=original(self,scene,config,side,wall,growth_s)
    if 26.87<scene.t_ref_s<26.89 and growth_s==0.:
        rays=[r for r in scene.rays if r.status=='return']
        points=np.asarray([r.hit_xy_m for r in rays])
        inside=(abs(points[:,0])<=config.length_m/2+padding)&(abs(points[:,1])<=config.width_m/2+padding)
        captures.append(dict(t=scene.t_ref_s,side=side,
            face=evidence.get('corridor',{}).get('surface'),
            points=[dict(index=rays[i].source_index,xy_m=points[i].tolist(),padding_m=float(padding[i]),
                age_s=scene.t_ref_s-rays[i].acquired_at_s) for i in np.flatnonzero(inside)]))
    return padding,evidence
ForecastGuide._padding=traced
calls=mismatches=0
for event in read_events(source):
    d=event['data']
    if event['kind']=='header': c,mc=build_controller(d)
    elif event['kind']=='reset': c.reset()
    elif event['kind']=='call':
        out=freeze(replay_call(c,d['input'],mc));calls+=1
        mismatches+=not equal_output(comparable_decision(out),comparable_decision(d['output']))
        if d['input']['reference_s']>26.89:break
result=dict(scope='unchanged stored C command replay; points in current expanded body, not physical collision',
    calls=calls,mismatches=mismatches,captures=captures)
(A/'c_padding_diagnosis.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=1))
