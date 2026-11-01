"""완주본의 묶인 상한별 시간과 다음 상한까지의 여유를 센다. 가속 모델 없는 상계 진단, 주행 예측 아님."""
from collections import Counter, defaultdict
from hashlib import sha256
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
RUN=Path(sys.argv[1]) if len(sys.argv)>1 else HERE.parent/'solo_corridor_arc_v1'
OUTPUT=Path(sys.argv[2]) if len(sys.argv)>2 else HERE/'next_limit.json'
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


by_limit=defaultdict(lambda:Counter());speed_at=defaultdict(list);open_false=Counter();open_rows=[];reasons=Counter();blend_floor=Counter()
for line in (RUN/'run1/follow_decisions.jsonl').open(encoding='utf-8'):
    e=json.loads(line);d=e['data']
    if e['kind']=='header':c=d['follow_config']
    if e['kind']!='call':continue
    inp=d['input'];v,steering,out=d['output'];t=inp['reference_s']
    if not start<=t<=end:continue
    dt=inp['dt_s']
    truth=float(np.interp(t,T,V));reasons[out.get('reason')]+=dt
    if out.get('reason')!='local_follow' or not out.get('path_points'):
        by_limit[str(out.get('reason'))]['seconds']+=dt
        continue
    caps,remaining=caps_of(out,inp,c)
    order=sorted(caps,key=caps.get)
    name=order[0]
    if out.get('turn_preview')=='observed_turn_clear':name+='+lifted'
    lower=max(.2,caps[order[0]]);upper=caps[order[1]]
    row=by_limit[name]
    row['seconds']+=dt;row['distance_m']+=truth*dt;row['saved_upper_s']+=dt*(1-lower/upper)
    speed_at[name].append(truth)
    if not out['open_route_speed_enabled']:
        cause=[]
        if out.get('boundary_from_memory'):cause.append('memory_path')
        if out.get('path_wall')!=inp['side']:cause.append('other_wall')
        if not cause:cause.append('front_gap_or_other')
        open_false['+'.join(cause)]+=dt
        open_rows.append(dict(t=t,s=float(np.interp(t,T,S)),cause='+'.join(cause),selection=out.get('boundary_selection'),truth=truth))
    if caps.get('path_length_blend',9)<=c['max_speed_mps']+1e-6:blend_floor['remaining_le_1p1_s']+=dt
result=dict(scope='묶인 상한의 시간과 다음 상한까지 여유. saved_upper_s는 가속·상호작용 없는 상계이며 주행 시간 예측이 아니다.',
    window_s=[start,end],limits={k:{**{a:round(b,3) for a,b in v.items()},'median_truth_mps':round(float(np.median(speed_at[k])),3) if speed_at[k] else None} for k,v in sorted(by_limit.items(),key=lambda kv:-kv[1]['seconds'])},
    reasons={k:round(v,3) for k,v in reasons.items()},open_route_off_s={k:round(v,3) for k,v in open_false.items()},blend_floor=dict(blend_floor),
    open_route_off_spans=[],script_sha256=sha256(Path(__file__).read_bytes()).hexdigest())
span=None
for r in open_rows:
    if span and r['t']-span['end']<.1 and r['cause']==span['cause']:span['end']=r['t'];span['s1']=r['s']
    else:
        span=dict(start=r['t'],end=r['t'],s0=r['s'],s1=r['s'],cause=r['cause'],selection=r['selection'],truth=r['truth']);result['open_route_off_spans'].append(span)
OUTPUT.write_text(json.dumps(result,ensure_ascii=False,indent=1),encoding='utf-8',newline='\n')
print(json.dumps({k:result[k] for k in('limits','open_route_off_s','blend_floor','reasons')},ensure_ascii=False,indent=0))
print([ (round(x['start'],2),round(x['end'],2),round(x['s0'],1),x['cause'],x['selection']) for x in result['open_route_off_spans'] if x['end']-x['start']>.1])
