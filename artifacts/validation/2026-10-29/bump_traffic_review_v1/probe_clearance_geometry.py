"""run3 두 새 스캔의 실제 반환/결측과 빈 위치 질의 실패 원인을 보존한다."""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.observation_support import depth_support
from arena_autonomy.scan_topology import cyclic_count, adjacent_indices


def sha(path):
    with path.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def inspect(scene,point):
    grid=np.array([(x,y) for x in np.arange(-3,4)*.01 for y in np.arange(-3,4)*.01
        if x*x+y*y<=.03**2+1e-12])
    theta=np.arange(24)*math.tau/24
    queries=point+np.vstack((grid,.03*np.column_stack((np.cos(theta),np.sin(theta)))))
    padding=np.array([.03+2*(scene.t_ref_s-r.acquired_at_s) for r in scene.rays])
    support=depth_support(scene,queries,padding)
    origins=np.array([r.ray_origin_xy_m for r in scene.rays])
    directions=np.array([r.direction_xy for r in scene.rays])
    cycle=cyclic_count(scene,None)
    witnesses=[]
    for q,supported in zip(queries,support):
        delta=q-origins
        angles=np.arctan2(directions[:,0]*delta[:,1]-directions[:,1]*delta[:,0],
                         np.sum(directions*delta,axis=1))
        lo=int(np.argmin(np.where(angles>=-1e-10,np.maximum(angles,0.),np.inf)))
        hi=int(np.argmin(np.where(angles<=1e-10,np.maximum(-angles,0.),np.inf)))
        bounds=[]
        for i in (lo,hi):
            ray=scene.rays[i]
            distance=float(np.linalg.norm(delta[i]))
            depth=None if ray.hit_xy_m is None else math.dist(ray.ray_origin_xy_m,ray.hit_xy_m)
            bounds.append(dict(index=i,status=ray.status,angle_offset_deg=math.degrees(angles[i]),
                acquired_at_s=ray.acquired_at_s,ray_origin=ray.ray_origin_xy_m,hit=ray.hit_xy_m,
                query_radius_m=distance,depth_m=depth,padding_m=float(padding[i]),
                remaining_m=None if depth is None else depth-distance-padding[i]))
        adjacent=bool(adjacent_indices(np.array([lo]),np.array([hi]),cycle_count=cycle)[0])
        reasons=[]
        if not adjacent:reasons.append('nonadjacent')
        if abs(angles[lo])+abs(angles[hi])>.025:reasons.append('angular_gap')
        if np.any(np.sum(delta*delta,axis=1)<=1e-18):reasons.append('origin_overlap')
        if angles[lo]<-1e-10 or angles[hi]>1e-10:reasons.append('not_bracketed')
        if any(b['status']!='return' for b in bounds):reasons.append('missing_return')
        if any(b['remaining_m'] is not None and b['remaining_m']<=0 for b in bounds):reasons.append('depth_margin')
        assert bool(supported)==(not reasons), (q,supported,reasons)
        witnesses.append(dict(query=q.tolist(),supported=bool(supported),reasons=reasons,bounds=bounds))
    return dict(point=point.tolist(),supported=int(support.sum()),queries=len(queries),
        failed=[w for w in witnesses if not w['supported']],witnesses=witnesses)


def main():
    source=ROOT/'artifacts/validation/2026-10-28/bump_traffic_v1/run3/follow_decisions.jsonl'
    before=sha(source);rows=[]
    with source.open(encoding='utf-8') as stream:
        header=json.loads(next(stream))['data'];config=MotionConfig(**header['motion_config'])
        for line in stream:
            event=json.loads(line)
            if event['kind']!='call':continue
            data=event['data'];now=data['input']['reference_s']
            if now>134.83:break
            if not any(abs(now-t)<1e-7 for t in (134.72,134.82)):continue
            anchor=data['output'][2]['partial_observation']['anchor_diagnostic']
            inputs,scan,history=restore_call(data['input'],config)
            scene=motion_scene(scan,history,now,timing_verified=inputs['timing_verified'],
                rear_half_angle_deg=0.,epoch=inputs['epoch'])
            points=np.asarray(thaw(anchor['corrected_points']))
            rows.append(dict(t=now,observed=anchor['observed'],points=[inspect(scene,p) for p in points],
                current_returns=[r.hit_xy_m for r in scene.rays if r.status=='return']))
    assert len(rows)==2
    result=dict(scope='read-only saved raw-ray geometry; no new driving or changed margins',
        input_sha256=before,input_unchanged=sha(source)==before,script_sha256=sha(Path(__file__)),rows=rows)
    with (BASE/'clearance_geometry.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps([dict(t=r['t'],points=[dict(supported=p['supported'],queries=p['queries'],
        reasons=sorted({why for q in p['failed'] for why in q['reasons']})) for p in r['points']]) for r in rows]))


if __name__=='__main__':main()
