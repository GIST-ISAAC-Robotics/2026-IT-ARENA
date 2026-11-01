"""실패 지점 앞에서 실제 차체 속도와 마지막 경로 변경을 비교한다."""
import json
from pathlib import Path
import numpy as np

HERE=Path(__file__).resolve().parent
CASES={'rise_only':HERE.parent/'race_acceleration_v1/run1',
       'turn_only':HERE.parent/'turn_preview_v1/run4','combined_failed':HERE/'run1'}


def one(folder):
    with (folder/'trajectory.jsonl').open() as f:
        rows=[x for x in map(json.loads,f) if x['light']=='green']
    goals=(40.,41.,42.,42.5,42.8,43.,43.25)
    out=[];i=0
    for goal in goals:
        while i+1<len(rows) and not rows[i]['progress_m']<goal<=rows[i+1]['progress_m']:i+=1
        if i+1==len(rows):break
        a,b=rows[i:i+2];frac=(goal-a['progress_m'])/(b['progress_m']-a['progress_m'])
        speed=lambda p:np.hypot(p['dynamics']['truth_longitudinal_mps'],p['dynamics']['truth_lateral_mps'])
        out.append(dict(progress_m=goal,t_s=a['time']+frac*(b['time']-a['time']),
            body_mps=float(speed(a)+frac*(speed(b)-speed(a))),state_before=a['autonomy']['state']))
    return out


if __name__=='__main__':
    result={k:one(v) for k,v in CASES.items()}
    result['scope']='평가용 같은 중심선 진행에서 처음 지날 때 실제 인접 속도 보간. 좌우 위치·스캔 위상·경로가 같다는 가정은 하지 않는다.'
    with (HERE/'last_corner_compare.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print({k:v[-3:] for k,v in result.items() if isinstance(v,list)})
