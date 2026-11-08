"""r35 저장 입력의 RELEASED 이동 영역 재구성. 출력 변경/새 주행은 하지 않는다."""
from pathlib import Path
import json,math,hashlib
import numpy as np
from collections import Counter
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig,freeze,equal_output
from arena_autonomy.pass_release_trial import motion_region,lateral_config
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn

A=Path(__file__).resolve().parent
trace=A.parent/'c1_piecewise_noise_v1/B_c1_injection/run1/follow_decisions.jsonl'
targets=[60.36,60.60,64.0,67.0];rows=[];snapshots=[]
for line in trace.open():
    event=json.loads(line);d=event['data']
    if event['kind']=='header':header=d;cfg=FollowConfig(**d['follow_config']);mc=MotionConfig(**d['motion_config'])
    if event['kind']!='call' or not targets:continue
    now=d['input']['reference_s']
    if now+1e-8<targets[0]:continue
    targets.pop(0)
    i,scan,history=restore_call(d['input'],mc)
    scene=motion_scene(scan,history,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
    n=d['output'][2]['normal'];proof=n['partial_observation'];checked=proof['timed_missing_diagnostic']
    assert n['retained_passage'] and n['selector_mode']=='RELEASED'
    context={};target=.35;steer=checked['checked_steering_rad']
    region=motion_region(scene,i['speed_mps'],target,steer,i['previous_steering_rad'],i['dt_s'],cfg,lateral_config(cfg),context=context)
    assert equal_output(freeze(region),n['selector_region']),(now,region,n['selector_region'])
    q=context['queries'][~context['free'] & ~context['masked']];pad=context['padding']
    origins=np.asarray([r.ray_origin_xy_m for r in scene.rays]);directions=np.asarray([r.direction_xy for r in scene.rays])
    statuses=np.asarray([r.status for r in scene.rays]);counts=Counter();indices=set();records=[]
    for point in q:
        delta=point-origins;radius=np.linalg.norm(delta,axis=1)
        angles=np.arctan2(directions[:,0]*delta[:,1]-directions[:,1]*delta[:,0],np.sum(directions*delta,axis=1))
        lo=int(np.argmin(np.where(angles>=-1e-10,np.maximum(angles,0.),np.inf)))
        hi=int(np.argmin(np.where(angles<=1e-10,np.maximum(-angles,0.),np.inf)))
        indices.update((lo,hi));counts['/'.join(sorted([statuses[lo],statuses[hi]]))]+=1
        depths=[math.dist(scene.rays[k].ray_origin_xy_m,scene.rays[k].hit_xy_m) if statuses[k]=='return' else None for k in (lo,hi)]
        records.append(dict(xy=point.tolist(),pair=[lo,hi],statuses=statuses[[lo,hi]].tolist(),
            margins=[None if dep is None else dep-radius[k]-pad[k] for dep,k in zip(depths,(lo,hi))]))
    nonreturn=_bounded_nonreturn(scene,scan,q,pad,dict(accepted=False,maximum_gap_rad=math.radians(3),minimum_neighbor_range_m=.6))
    rows.append(dict(t=now,region_exact=True,region=region,unsupported_bounds=[q.min(0).tolist(),q.max(0).tolist()],pair_counts=dict(counts),
        involved_rays=[dict(index=k,status=statuses[k],angle_deg=math.degrees(math.atan2(*directions[k][::-1]))) for k in sorted(indices)],
        all_unknown=[dict(index=k,angle_deg=math.degrees(math.atan2(*directions[k][::-1]))) for k in np.flatnonzero(statuses=='unknown')],
        raw_nonreturn_test=nonreturn,selector_reason=n['selector_reason'],same_path_reason=n['same_path_gap']['reason']))
    snapshots.append(dict(t=now,header=header,input=d['input'],output=d['output'],queries=records,
        endpoints=list(scene.occupied_endpoints_xy_m),rays=[dict(origin=r.ray_origin_xy_m,direction=r.direction_xy,hit=r.hit_xy_m,status=r.status) for r in scene.rays]))
assert len(rows)==4
(A/'result.json').write_text(json.dumps(freeze(dict(scope='same recorded scene, exact selector_region; no driven result',trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),rows=rows)),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
(A/'snapshots.json').write_text(json.dumps(freeze(snapshots),ensure_ascii=False)+'\n',encoding='utf8')
print(json.dumps([dict(t=r['t'],pairs=r['pair_counts'],bounds=r['unsupported_bounds'],nonreturn=r['raw_nonreturn_test']['reason']) for r in rows],ensure_ascii=False))
