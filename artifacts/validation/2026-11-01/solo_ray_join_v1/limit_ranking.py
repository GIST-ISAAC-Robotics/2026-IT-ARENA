"""완주본 판단 기록의 속도 상한을 진행 거리에 올려 병목 순위를 본다. 평가 전용, 새 주행 예측 아님."""
from collections import defaultdict
from hashlib import sha256
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
RUN=HERE.parent/'solo_corridor_arc_v1'
sys.path.insert(0,str(ROOT/'src/arena_autonomy'))
from arena_autonomy.local_path import braking_speed as brake

metrics=json.loads((RUN/'run1_metrics.json').read_bytes())
start=metrics['first_green_sample_s'];end=start+metrics['lap_s_from_first_green_sample']
rows=[r for r in map(json.loads,(RUN/'run1/trajectory.jsonl').read_bytes().splitlines()) if r['light']=='green']
T=np.array([r['time'] for r in rows]);S=np.array([r['progress_m'] for r in rows])
V=np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'],r['dynamics']['truth_lateral_mps']) for r in rows])


def caps_of(out,inp,c):
    rear=np.asarray(out['path_points'],dtype=float).copy();rear[:,0]+=c['wheelbase_m']/2
    d=np.linalg.norm(rear,axis=1);nearest=int(np.argmin(d))
    ahead=np.flatnonzero((rear[:,0]>.1)&(np.arange(len(rear))>=nearest))
    look=min(.6,max(.3,.3+.12*abs(inp['speed_mps'])))
    target=rear[ahead[np.argmin(abs(d[ahead]-look))]]
    curvature=float(2*target[1]/max(.01,target@target))
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(rear,axis=0),axis=1))]
    remaining=float(arc[-1]-arc[nearest])
    dx=np.gradient(rear,axis=0);ddx=np.gradient(dx,axis=0)
    curves=abs(dx[:,0]*ddx[:,1]-dx[:,1]*ddx[:,0])/np.maximum(1e-8,np.linalg.norm(dx,axis=1)**3)
    preview=np.sqrt(c['lateral_acceleration_mps2']/np.maximum(.15,curves[nearest:])+4*np.maximum(0.,arc[nearest:]-arc[nearest]-.2))
    age=inp['reference_s']-out['source_first_s']
    available=max(0.,out['swept_clearance_m']-c['standstill_gap_m'])
    caps=dict(route_cap=c['open_route_speed_mps'] if out['open_route_speed_enabled'] else c['max_speed_mps'],
              path_stop=brake(remaining,max(age,out['boundary_age_s'])),
              path_curvature_preview=float(min(preview)),
              pursuit_curvature=math.sqrt(c['lateral_acceleration_mps2']/max(.15,abs(curvature))),
              constant_arc_stop=brake(available,age),constant_arc_headway=available/c['headway_s'])
    if 'observed_length_speed_cap_mps' in out:caps['path_length_blend']=out['observed_length_speed_cap_mps']
    return caps,remaining


samples=[]
for line in (RUN/'run1/follow_decisions.jsonl').open(encoding='utf-8'):
    e=json.loads(line);d=e['data']
    if e['kind']=='header':c=d['follow_config']
    if e['kind']!='call':continue
    inp=d['input'];v,steering,out=d['output'];t=inp['reference_s']
    if not start<=t<=end:continue
    row=dict(t=t,s=float(np.interp(t,T,S)),v=v,steering=steering,reason=out.get('reason'),truth=float(np.interp(t,T,V)),measured=inp['speed_mps'])
    if out.get('reason')=='local_follow' and out.get('path_points'):
        row['caps'],row['remaining']=caps_of(out,inp,c)
        row['preview']=out.get('turn_preview');row['lifted']=out.get('turn_preview')=='observed_turn_clear'
        row['clearance']=out['swept_clearance_m']
    samples.append(row)
GRID=np.arange(0.,float(S[np.searchsorted(T,end)]),.02)


def profile(drop):
    """drop에 든 상한을 뺀 위치별 목표. 비경로 판단은 기록 목표를 그대로 쓴다."""
    out=np.full(len(GRID),np.inf)
    for r in samples:
        value=r['v']
        if 'caps' in r and drop:
            kept=[x for k,x in r['caps'].items() if k not in drop]
            value=min(kept) if kept else r['v']
            if r['lifted'] and 'constant_arc_stop' not in drop:value=max(value,r['v'])
        i=int(r['s']/.02)
        if 0<=i<len(out):out[i]=min(out[i],value)
    known=np.isfinite(out)
    return np.interp(GRID,GRID[known],out[known])


def lap_time(cap,accel,decel):
    w=cap.copy();ds=.02
    w[0]=0.
    for i in range(1,len(w)):w[i]=min(w[i],math.sqrt(w[i-1]**2+2*accel*ds))
    for i in range(len(w)-2,-1,-1):w[i]=min(w[i],math.sqrt(w[i+1]**2+2*decel*ds))
    return float(np.sum(ds/np.maximum(.05,.5*(w[1:]+w[:-1])))),float(np.max(w))


# 가속/감속은 실측 속도에서 추정해 기준 랩을 맞춘다.
base=profile(())
dv=np.gradient(V,T)
accel=float(np.percentile(dv[dv>.2],80));decel=float(-np.percentile(dv[dv<-.2],20))
baseline=lap_time(base,accel,decel)
cases={'without_path_stop':('path_stop',),'without_path_blend':('path_length_blend',),
       'without_path_stop_and_blend':('path_stop','path_length_blend'),
       'without_constant_arc_stop':('constant_arc_stop',),'without_all_stop_like':('constant_arc_stop','path_stop','path_length_blend'),
       'without_curvature_limits':('path_curvature_preview','pursuit_curvature'),'route_cap_only_removed':('route_cap',)}
result=dict(scope='저장 경로의 상한을 진행 거리에 고정한 순위 진단. 상한 간 상호작용·조향·보호층을 예측하지 않으며 새 주행 시간이 아니다.',
    calibrated_accel_mps2=accel,calibrated_decel_mps2=decel,actual_lap_s=metrics['lap_s_from_first_green_sample'],
    model_baseline_s=baseline[0],model_baseline_peak_mps=baseline[1],cases={})
for name,drop in cases.items():
    t_,peak=lap_time(profile(drop),accel,decel)
    result['cases'][name]=dict(model_s=t_,delta_s=t_-baseline[0],peak_mps=peak)
bins=[0,.25,.5,.75,1.,1.25,1.5,1.75,2.,2.25,2.5,3.]
dt=np.diff(T);mid=.5*(V[1:]+V[:-1]);mask=(T[:-1]>=start)&(T[1:]<=end)
result['truth_speed_time_s']={f'{a}-{b}':float(np.sum(dt[mask&(mid>=a)&(mid<b)])) for a,b in zip(bins,bins[1:])}
sections=[]
edges=np.arange(0.,GRID[-1]+2.,2.)
for a,b in zip(edges,edges[1:]):
    m=(S>=a)&(S<b)&(T>=start)&(T<=end)
    if m.sum()<2:continue
    sub=[r for r in samples if a<=r['s']<b]
    top=defaultdict(float)
    for r in sub:
        if 'caps' in r:top[min(r['caps'],key=r['caps'].get)]+=.02
        else:top[r['reason']]+=.02
    sections.append(dict(s0=float(a),s1=float(b),seconds=float(T[m][-1]-T[m][0]),v_mean=float(np.mean(V[m])),v_min=float(np.min(V[m])),v_max=float(np.max(V[m])),
                         limit=sorted(((k,round(x,2)) for k,x in top.items()),key=lambda kv:-kv[1])[:3]))
result['sections']=sections
result['script_sha256']=sha256(Path(__file__).read_bytes()).hexdigest()
with (HERE/'limit_ranking.json').open('w',encoding='utf-8',newline='\n') as f:json.dump(result,f,ensure_ascii=False,indent=2)
print(json.dumps({k:v for k,v in result.items() if k!='sections'},ensure_ascii=False,indent=1))
for r in sections:print(r['s0'],r['s1'],round(r['seconds'],2),round(r['v_min'],2),round(r['v_mean'],2),round(r['v_max'],2),r['limit'])
