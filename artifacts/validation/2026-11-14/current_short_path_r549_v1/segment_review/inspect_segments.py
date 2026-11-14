"""r544 저장 입력의 관측 사슬 끝/원 광선만 분석. 제어기/새 주행 실행 없음."""
from pathlib import Path
import copy,hashlib,json,math,os,sys
for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):os.environ[key]='1'
import numpy as np
A=Path(__file__).resolve().parent;ROOT=A.parents[4]
P=ROOT/'artifacts/validation/2026-11-14/measured_steering_lap_r544_attempt2_v1/stall_extract'
sys.dont_write_bytecode=True
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_path import break_adjacent_occluders

def chain_details(scene,side,raw,bridge=True):
    returns=[r for r in scene.rays if r.status=='return']
    p=np.asarray([r.hit_xy_m for r in returns]);indices=np.asarray([r.source_index for r in returns])
    sign=1 if side=='left' else -1
    order=np.argsort(-sign*np.arctan2(p[:,1],p[:,0]));p=p[order];indices=indices[order]
    gap=np.linalg.norm(np.diff(p,axis=0),axis=1)
    limit=.10+.035*np.minimum(np.linalg.norm(p[:-1],axis=1),np.linalg.norm(p[1:],axis=1))
    broken=gap>limit
    original_p=p.copy();original_indices=indices.copy()
    remove,joins=break_adjacent_occluders(p,broken) if bridge else (np.zeros(len(p),dtype=bool),[])
    removed_sources=indices[remove].tolist()
    keep=~remove;new_index=np.cumsum(keep)-1
    p=p[keep];indices=indices[keep]
    gap=np.linalg.norm(np.diff(p,axis=0),axis=1)
    limit=.10+.035*np.minimum(np.linalg.norm(p[:-1],axis=1),np.linalg.norm(p[1:],axis=1));broken=gap>limit
    for before,_ in joins:broken[new_index[before]]=False
    split=np.r_[0,np.flatnonzero(broken)+1,len(p)];rows=[]
    for start,end in zip(split[:-1],split[1:]):
        wall=p[start:end];src=indices[start:end]
        if len(wall)<12:continue
        candidates=np.flatnonzero((abs(wall[:,0])<.35)&(sign*wall[:,1]>.10))
        if not len(candidates):continue
        scores=np.linalg.norm(wall[candidates]-[0.,sign*.505],axis=1)
        anchor=int(candidates[np.argmin(scores)]);score=float(np.min(scores))
        if score>.35:continue
        chord=wall[2:]-wall[:-2];middle=wall[1:-1]-wall[:-2]
        roughness=float(np.median(abs(chord[:,0]*middle[:,1]-chord[:,1]*middle[:,0])/np.maximum(1e-9,np.linalg.norm(chord,axis=1))))
        metric=wall
        if roughness>.003:
            padded=np.pad(wall,((3,3),(0,0)),mode='edge')
            metric=np.column_stack([np.convolve(padded[:,i],np.ones(7)/7,mode='valid') for i in (0,1)])
        arc=np.r_[0.,np.cumsum(np.maximum(1e-6,np.linalg.norm(np.diff(metric,axis=0),axis=1)))]
        next_item=None
        if end<len(p):next_item=dict(source_index=int(indices[end]),point=p[end].tolist(),gap_m=float(gap[end-1]),limit_m=float(limit[end-1]))
        last=int(src[-1]);near=[]
        for i in range(max(0,last-3),min(len(scene.rays),last+13)):
            r=scene.rays[i]
            near.append(dict(source_index=i,status=r.status,hit=r.hit_xy_m,origin=r.ray_origin_xy_m,
                angle_raw_rad=raw.angles_rad[i],range_raw_m=raw.ranges_m[i],acquired_at_s=r.acquired_at_s))
        original_position=int(np.flatnonzero(original_indices==last)[0])
        sorted_neighbors=[dict(source_index=int(original_indices[i]),point=original_p[i].tolist(),
            base_angle_rad=float(np.arctan2(original_p[i,1],original_p[i,0])),removed_from_path=bool(remove[i]))
            for i in range(max(0,original_position-2),min(len(original_p),original_position+18))]
        rows.append(dict(side=side,bridge=bridge,removed_source_indices=removed_sources,base_angle_sorted_neighbors=sorted_neighbors,
            samples=len(wall),anchor_index=int(src[anchor]),anchor=wall[anchor].tolist(),
            anchor_distance_score_m=score,roughness_m=roughness,observed_ahead_m=float(arc[-1]-arc[anchor]),
            first_source=int(src[0]),first_point=wall[0].tolist(),last_source=last,last_point=wall[-1].tolist(),
            next_angular_return=next_item,last_source_neighbors=near,source_indices=src.tolist(),points=wall.tolist()))
    return rows

def main():
    header=json.loads((P/'header.json').read_text())['record']['data'];motion=MotionConfig(**header['motion_config'])
    rows=[]
    for case in ('last_positive','first_zero_discarded','first_zero_candidate'):
        path=P/(case+'_current_input.json');saved=json.loads(path.read_text());data,scan,history=restore_call(saved,motion)
        scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],epoch=data['epoch'],
            rear_half_angle_deg=data['rear_half_angle_deg'],timing_policy=data.get('timing_policy','verified_only'))
        row=dict(case=case,reference_s=data['reference_s'],first_angle_rad=scan.angles_rad[0],last_angle_rad=scan.angles_rad[-1],
            status_counts={s:sum(r.status==s for r in scene.rays) for s in ('return','unknown','masked')},
            left=chain_details(scene,'left',scan),right=chain_details(scene,'right',scan),
            left_no_bridge_diagnostic=chain_details(scene,'left',scan,False))
        rows.append(row)
    (A/'segments.json').write_text(json.dumps(freeze(rows),ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    for row in rows:
        print(json.dumps({**{k:v for k,v in row.items() if k not in ('left','right','left_no_bridge_diagnostic')},
            'left':[{k:r[k] for k in ('observed_ahead_m','last_source','last_point','next_angular_return')} for r in row['left']],
            'right':[{k:r[k] for k in ('observed_ahead_m','last_source')} for r in row['right']],
            'left_no_bridge_diagnostic':[{k:r[k] for k in ('observed_ahead_m','last_source')} for r in row['left_no_bridge_diagnostic']]},ensure_ascii=False))
if __name__=='__main__':main()
