"""평가 전용: 그때 수신한 점을 고정한 채 실제로 지나간 차체 궤적과 당시 조향 원호를 비교."""
from collections import Counter
import argparse
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.occupancy_timing import per_ray_padding


def future_clear(poses,t,span,points,pad,cfg):
    # 나중에 보게 된 차체 자세는 이 평가에서만 사용한다.
    tail=poses[np.searchsorted(poses[:,0],t):np.searchsorted(poses[:,0],t+4.)]
    if len(tail)<2 or tail[0,0]-t>.005 or np.max(np.diff(tail[:,0]))>.025:return None
    steps=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(tail[:,1:3],axis=0),axis=1))]
    if steps[-1]<span:return None
    ds=np.r_[np.arange(0.,span,.02),span]
    c,s=np.cos(tail[0,3]),np.sin(tail[0,3])
    wx=tail[0,1]+c*points[:,0]-s*points[:,1];wy=tail[0,2]+s*points[:,0]+c*points[:,1]
    dx=wx-np.interp(ds,steps,tail[:,1])[:,None];dy=wy-np.interp(ds,steps,tail[:,2])[:,None]
    yaw=np.interp(ds,steps,np.unwrap(tail[:,3]));cy=np.cos(yaw)[:,None];sy=np.sin(yaw)[:,None]
    hit=np.flatnonzero(np.any((abs(cy*dx+sy*dy)<=cfg['length_m']/2+pad)&
                              (abs(-sy*dx+cy*dy)<=cfg['width_m']/2+pad),axis=1))
    return span if not len(hit) else max(0.,ds[hit[0]]-.02)


def measure(run):
    with (run/'ego_poses.jsonl').open() as f:poses=np.array([json.loads(x) for x in f])
    counts=Counter();examples=[];seen=set()
    with (run/'follow_decisions.jsonl').open() as f:
        for e in map(json.loads,f):
            d=e['data']
            if e['kind']=='header':motion=MotionConfig(**d['motion_config']);cfg=d['follow_config']
            if e['kind']!='call':continue
            v,_,o=d['output']
            if not (o.get('reason')=='local_follow' and o.get('behavior')=='FOLLOW' and .001<v<=1.001):continue
            key=o.get('source_first_s')
            if key in seen:continue
            seen.add(key);counts['distinct_scans']+=1
            arc=o['swept_clearance_m'];span=arc+.20
            if o['observed_path_ahead_m']<span:
                counts['observed_path_too_short_for_20cm_comparison']+=1;continue
            a,scan,h=restore_call(d['input'],motion)
            scene=motion_scene(scan,h,a['reference_s'],timing_verified=a['timing_verified'],epoch=a['epoch'],
                               rear_half_angle_deg=a['rear_half_angle_deg'])
            pad=per_ray_padding(scene,.025+cfg['measurement_padding_m'],cfg['object_motion_bound_mps'])
            fwd=future_clear(poses,a['reference_s'],span,np.asarray(scene.occupied_endpoints_xy_m),pad,cfg)
            if fwd is None:counts['future_pose_window_insufficient']+=1;continue
            gain=fwd-arc
            counts['actual_clear_through_extra_20cm' if gain>=.19999 else
                   'actual_reaches_arc_but_not_extra_20cm' if gain>=-.04 else 'actual_hits_earlier']+=1
            if gain>=.19999 and len(examples)<4:
                examples.append(dict(reference_s=a['reference_s'],constant_arc_m=arc,actual_frozen_cloud_clear_m=fwd))
    return dict(counts=counts,examples=examples,scope='단독 실측 시뮬 궤적에 같은 점별 여유와 차체 적용. 빠르게 갔을 때 같은 궤적·동적 차량에 대한 보장 아님. 미래 자세는 계획기에 입력하지 않는다.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();r=measure(a.run)
    with a.output.open('x',encoding='utf-8') as f:json.dump(r,f,indent=2,ensure_ascii=False)
    print(json.dumps(r,ensure_ascii=False))
