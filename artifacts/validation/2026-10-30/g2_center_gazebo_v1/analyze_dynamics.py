"""run1의 벽 간격, 모터축 속도와 차체 속도 차이, 잔디 구간 진행을 저장 기록으로 계산한다.

정답 자세는 평가에만 쓴다. 평균 모터축 속도만 있어 좌우 바퀴 헛돎은 직접 측정하지 못하고,
차체 속도와의 차이와 braking_diagnostics의 관절 속도로 간접 확인한다.
"""
import json
import math
from pathlib import Path

import numpy as np

BASE=Path(__file__).resolve().parent;RUN=BASE/'run1'
poses=np.array([json.loads(line) for line in (RUN/'ego_poses.jsonl').open(encoding='utf-8') if line.strip()],dtype=float)
series=json.loads((BASE/'run1_decision_series.json').read_text(encoding='utf-8'))
evaluation=json.loads((BASE/'run1_g2_evaluation.json').read_text(encoding='utf-8'))
green=evaluation['green_sim_s'];end=evaluation['window_end_sim_s']
origin=poses[0,1:3];heading=poses[0,3]
c,s=math.cos(heading),math.sin(heading)
along=(poses[:,1]-origin[0])*c+(poses[:,2]-origin[1])*s
lateral=-(poses[:,1]-origin[0])*s+(poses[:,2]-origin[1])*c
yaw=np.unwrap(poses[:,3])-heading
half_l,half_w=.10,.075
extent=np.abs(lateral)+half_l*np.abs(np.sin(yaw))+half_w*np.abs(np.cos(yaw))
inside=(poses[:,0]>=green)&(poses[:,0]<=end)
WALL=.42494  # preflight의 중심선 시작 자세 벽 간격 0.34994 m + 차체 반폭
gap=WALL-extent
index=np.flatnonzero(inside)[int(np.argmin(gap[inside]))]
# 0.1 s 구간 차체 속도
times=poses[:,0]
def body_speed(t,half=.05):
    a,b=np.searchsorted(times,[t-half,t+half])
    if b-a<2:return None
    path=np.sum(np.hypot(np.diff(poses[a:b,1]),np.diff(poses[a:b,2])))
    return float(path/(times[b-1]-times[a]))
rows=[]
for row in series:
    if green<=row['t']<=end:
        v=body_speed(row['t'])
        if v is not None:
            rows.append(dict(t=row['t'],measured=row['speed'],body=v,command=row['target'],phase=row['phase']))
windows={}
for name,(a,b) in dict(entry_on_grass=(green+2.,green+5.5),rejoin=(green+5.5,green+8.8),centered=(green+9.,green+18.)).items():
    part=[r for r in rows if a<=r['t']-0<=b]
    measured=np.array([r['measured'] for r in part]);body=np.array([r['body'] for r in part])
    windows[name]=dict(samples=len(part),mean_measured_mps=float(measured.mean()),mean_body_mps=float(body.mean()),
        ratio=float(measured.mean()/body.mean()),max_abs_difference_mps=float(np.max(np.abs(measured-body))))
moving=[r for r in rows if r['command']>0]
stalled=sum(r['body']<.03 for r in moving if r['t']>green+1.)
result=dict(scope='saved records, evaluation truth only; mean motor-shaft speed cannot show left/right wheel split',
    minimum_wall_gap_m=float(gap[index]),minimum_wall_gap_t_since_green_s=float(times[index]-green),
    lateral_at_minimum_m=float(lateral[index]),yaw_at_minimum_rad=float(yaw[index]),
    maximum_abs_yaw_rad=float(np.max(np.abs(yaw[inside]))),windows=windows,
    commanded_ticks_body_below_0p03=stalled,commanded_ticks=len(moving),
    command_zero_ticks_in_window=sum(r['command']==0. for r in rows))
(BASE/'run1_dynamics.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,indent=1))
