"""예측의 응답 가정을 기록과 맞춰 본다. 평가 전용이며 정답은 제어에 들어가지 않는다.

조향 명령→실제 조향 지연과 예측 경로→실제 궤적 차이를 속도대별로 센다. 고속 안전 증명이 아니다.
호출: response_check.py <run-folder> <출력.json>
"""
from collections import defaultdict
from hashlib import sha256
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.local_path import braking_speed
from arena_autonomy.turn_preview import rollout

folder=Path(sys.argv[1]);output=Path(sys.argv[2])
metrics=json.loads((folder/'run1_metrics.json').read_bytes())
begin=metrics['first_green_sample_s'];end=begin+metrics['lap_s_from_first_green_sample']

rows=[r for r in map(json.loads,(folder/'run1/trajectory.jsonl').read_bytes().splitlines()) if r['light']=='green' and begin<=r['time']<=end]
t=np.array([r['time'] for r in rows])
command=np.array([r['autonomy'].get('steering_command_rad',0.) for r in rows],dtype=float)
actual=np.array([.5*(r['dynamics']['left_steering_rad']+r['dynamics']['right_steering_rad']) for r in rows])
speed=np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'],r['dynamics']['truth_lateral_mps']) for r in rows])
grid=np.arange(t[0],t[-1],.005)
c=np.interp(grid,t,command);a=np.interp(grid,t,actual);v=np.interp(grid,t,speed)
lags={}
for name,mask in (('all',v>=0.),('over_1mps',v>1.),('over_1p5mps',v>1.5)):
    best=[]
    for shift in range(0,61):
        keep=mask[:-shift] if shift else mask
        x=c[:len(c)-shift] if shift else c
        y=a[shift:]
        best.append(float(np.mean((x[keep]-y[keep])**2)))
    lags[name]=dict(best_lag_s=int(np.argmin(best))*.005,rms_at_best_rad=math.sqrt(min(best)),rms_no_lag_rad=math.sqrt(best[0]))
rate=np.abs(np.diff(actual))/np.maximum(1e-3,np.diff(t))

poses=np.array([json.loads(line) for line in (folder/'run1/ego_poses.jsonl').read_text(encoding='utf-8').splitlines()])
poses=poses[np.argsort(poses[:,0],kind='stable')]
errors=defaultdict(list)
seen=set();config=None
for line in (folder/'run1/follow_decisions.jsonl').open(encoding='utf-8'):
    e=json.loads(line);d=e['data']
    if e['kind']=='header':config=d['follow_config']
    if e['kind']!='call':continue
    i=d['input'];target,steering,out=d['output'];now=i['reference_s']
    source=out.get('source_first_s')
    if not begin<=now<=end or source in seen or out.get('reason')!='local_follow' or not out.get('open_route_speed_enabled') or i['speed_mps']<.8:continue
    seen.add(source)
    span=out['swept_clearance_m']+.20;age=now-source
    if out['observed_path_ahead_m']<span:continue
    wanted=min(braking_speed(span-config['standstill_gap_m'],age),(span-config['standstill_gap_m'])/config['headway_s'])
    command_speed=max(wanted,target)
    predicted=rollout(np.asarray(out['path_points'],dtype=float),span,steering,i['speed_mps'],config,command_speed,age)
    if predicted is None:continue
    k=int(np.searchsorted(poses[:,0],now));x0,y0,yaw=poses[min(k,len(poses)-1),1:4]
    future=poses[(poses[:,0]>=now)&(poses[:,0]<=now+4.)]
    co,si=math.cos(yaw),math.sin(yaw)
    fx=co*(future[:,1]-x0)+si*(future[:,2]-y0);fy=-si*(future[:,1]-x0)+co*(future[:,2]-y0)
    gaps=np.hypot(np.diff(fx),np.diff(fy));arc=np.r_[0.,np.cumsum(gaps)]
    usable=predicted[predicted[:,3]<=arc[-1]-.03]
    if not len(usable):continue
    distance=np.min(np.hypot(usable[:,0][:,None]-fx,usable[:,1][:,None]-fy),axis=1)
    band='under_1p5' if command_speed<1.5 else ('1p5_to_2p2' if command_speed<2.2 else 'over_2p2')
    errors[band].append(float(np.max(distance)))
result=dict(scope='기록 비교. 조향 지연은 궤적 표본(약 46 ms) 보간이고 예측 차이는 해당 시점 경로로 만든 예측과 이후 실제 궤적의 최대 거리다. 후속 경로 갱신도 차이에 포함된다.',
    steering_lag=lags,steering_rate_p99_rad_s=float(np.percentile(rate,99)),
    rollout_vs_truth_m={k:dict(calls=len(v),p50=float(np.percentile(v,50)),p95=float(np.percentile(v,95)),max=float(np.max(v))) for k,v in sorted(errors.items())},
    script_sha256=sha256(Path(__file__).read_bytes()).hexdigest())
output.write_text(json.dumps(result,ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
print(json.dumps(result,ensure_ascii=False,indent=0))
