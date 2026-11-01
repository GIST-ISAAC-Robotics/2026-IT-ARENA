"""직선의 관측 벽이 실제로 끝났는지, 먼 반환 사이 간격에서 분리됐는지 조사. 평가 전용."""
import json
import math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_path import local_path

BASE=Path(__file__).resolve().parent


def examine(scene,side,offset):
    sign=1 if side=='left' else -1
    rays=[r for r in scene.rays if r.status=='return']
    pts=np.asarray([r.hit_xy_m for r in rays])
    order=np.argsort(-sign*np.arctan2(pts[:,1],pts[:,0]))
    p=pts[order]
    norms=np.linalg.norm(p,axis=1)
    steps=np.linalg.norm(np.diff(p,axis=0),axis=1)
    limits=.10+.035*np.minimum(norms[:-1],norms[1:])
    groups=np.split(np.arange(len(p)),np.flatnonzero(steps>limits)+1)
    candidates=[]
    for ids in groups:
        near=ids[(abs(p[ids,0])<.35)&(sign*p[ids,1]>.10)]
        if len(ids)<12 or not len(near):continue
        dist=np.linalg.norm(p[near]-[0.,sign*offset],axis=1)
        i=near[int(np.argmin(dist))];e=int(ids[-1]);score=float(dist.min())
        if score<=.35 and np.sum(steps[i:e])>=.28:candidates.append((score,e,ids))
    if not candidates:return {'reason':'no_group'}
    _,e,group=min(candidates,key=lambda g:g[0])
    if e==len(p)-1 or len(group)<5:return {'reason':'last_direction'}
    tail=p[group[-5:]];cent=tail.mean(axis=0)
    tangent=np.linalg.svd(tail-cent,full_matrices=False)[2][0]
    if np.dot(tangent,tail[-1]-tail[0])<0:tangent=-tangent
    normal=np.array([-tangent[1],tangent[0]])
    out=[];previous=e
    for j in range(e+1,min(e+9,len(p))):
        a=rays[order[previous]];b=rays[order[j]]
        out.append(dict(xy_m=p[j].tolist(),index_jump=abs(b.source_index-a.source_index),
            step_m=float(steps[j-1]),old_limit_m=float(limits[j-1]),
            residual_from_prior_wall_m=float(abs(np.dot(p[j]-cent,normal))),
            along_prior_wall_m=float(np.dot(p[j]-p[e],tangent))))
        previous=j
    _,meta=local_path(pts,side,offset,horizon=4.)
    return dict(reason='measured_end',side=side,wall_end_xy_m=p[e].tolist(),
        wall_ahead_m=meta.get('boundary_observed_ahead_m'),wall_raw_returns=len(group),
        prior_tangent_deg=math.degrees(math.atan2(tangent[1],tangent[0])),next_returns=out)


def sample(folder,stamp,side):
    with (folder/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        head=json.loads(next(stream))['data']
        row=next(r['data']['input'] for r in map(json.loads,stream) if r['kind']=='call'
                 and abs(r['data']['input']['reference_s']-stamp)<1e-7)
    args,scan,history=restore_call(row,MotionConfig(**head['motion_config']))
    scene=motion_scene(scan,history,args['reference_s'],timing_verified=args['timing_verified'],
                       epoch=args['epoch'],rear_half_angle_deg=args['rear_half_angle_deg'])
    return dict(run=folder.name,time=stamp,**examine(scene,side,head['follow_config']['wall_offset_m']))


if __name__=='__main__':
    a=json.loads((BASE/'ten_kmh_diagnostics.json').read_text())['highest_request']
    b=json.loads((BASE/'solo_10kmh_peak_request.json').read_text())
    rows=[sample(BASE/'solo_2p0',a['reference_s'],a['path_wall']),
          sample(BASE/'solo_10kmh',b['t'],b['path_wall'])]
    with (BASE/'straight_wall_end.json').open('x',encoding='utf-8') as f:
        json.dump(dict(scope='두 높은 명령 시점의 현재 스캔. 같은 직선인지 물체 정답으로 분류하지 않으며 제어 변경 없음.',samples=rows),f,indent=2,ensure_ascii=False)
