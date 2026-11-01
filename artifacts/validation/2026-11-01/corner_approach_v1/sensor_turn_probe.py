"""운영 노드에 연결하지 않은 조향 예측 비교. 입력은 당시 스캔·경로·속도·이전 조향뿐이다."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,pursue_path
from arena_autonomy.local_path import braking_speed
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.observation_support import depth_support
from arena_autonomy.scene_motion import motion_scene


def rollout(path,span,initial,speed,cfg,command,age):
    # 첫 취득/조향 반응 구간에는 직전 또는 발행 조향을 그대로 유지한다. 그 이후에도
    # 새 벽이 생긴다고 가정하지 않고 현재 경로에 같은 Pure Pursuit를 적용한다.
    ds=.03; speed=max(.25,abs(speed),command)
    L=cfg['wheelbase_m']; x=-L/2;y=yaw=0.;steer=initial
    poses=[]; hold=speed*(age+.10); look=min(.60,max(.30,.30+.12*speed))
    for a in np.arange(0.,span,ds):
        step=min(ds,span-a)
        co,si=math.cos(yaw),math.sin(yaw)
        if a>=hold:
            rel=path-np.array([x,y]);fx=co*rel[:,0]+si*rel[:,1];fy=-si*rel[:,0]+co*rel[:,1]
            dist=np.hypot(fx,fy);nearest=int(np.argmin(dist))
            ahead=np.flatnonzero((fx>.10)&(np.arange(len(fx))>=nearest))
            if len(ahead)<3 or np.max(dist[ahead])<.25:return None
            k=ahead[np.argmin(abs(dist[ahead]-look))]
            desired=np.clip(math.atan(L*2*fy[k]/max(.01,dist[k]**2)),-cfg['max_steering_rad'],cfg['max_steering_rad'])
            steer=float(np.clip(desired,steer-3*step/speed,steer+3*step/speed))
        curv=math.tan(steer)/L
        mid=yaw+step*curv/2
        x+=step*math.cos(mid); y+=step*math.sin(mid);yaw+=step*curv
        poses.append((x+L/2*math.cos(yaw),y+L/2*math.sin(yaw),yaw,a+step))
    return np.asarray(poses)


def classify(scene,path,span,arc,angle,prev,speed,cfg,old,age):
    new=min(braking_speed(span-cfg['standstill_gap_m'],age),(span-cfg['standstill_gap_m'])/cfg['headway_s'])
    points=np.asarray(scene.occupied_endpoints_xy_m)
    pad=per_ray_padding(scene,.025+cfg['measurement_padding_m'],cfg['object_motion_bound_mps'])
    fullpad=np.array([.025+cfg['measurement_padding_m']+
                      cfg['object_motion_bound_mps']*(scene.t_ref_s-r.acquired_at_s) for r in scene.rays])
    qs=[]
    for angle0 in set((angle,prev)):
        poses=rollout(path,span,angle0,speed,cfg,max(old,new),age)
        if poses is None:return 'path_ends_during_prediction'
        c,s=np.cos(poses[:,2:3]),np.sin(poses[:,2:3])
        dx=points[:,0]-poses[:,0:1];dy=points[:,1]-poses[:,1:2]
        if np.any((abs(c*dx+s*dy)<=cfg['length_m']/2+pad)&(abs(-s*dx+c*dy)<=cfg['width_m']/2+pad)):
            return 'observed_hit_remains'
        # 실제 반환이 아닌 빈 각도 방향을 20 cm 연장 근거로 쓰지 않는다.
        mask=poses[:,3]>=max(.20,arc-.12);x=poses[mask,0:1];y=poses[mask,1:2];c=c[mask];s=s[mask]
        for lx in (-cfg['length_m']/2,0.,cfg['length_m']/2):
            for ly in (-cfg['width_m']/2,0.,cfg['width_m']/2):
                qs.append(np.column_stack((x+c*lx-s*ly,y+s*lx+c*ly)))
    if not qs or not all(depth_support(scene,np.vstack(qs),fullpad)):return 'current_ray_support_missing'
    return 'additional_20cm_supported_in_sample'


def measure(run):
    counts=Counter();seen=set();matched=[]
    r=json.loads((run/'report.json').read_text(encoding='utf-8'))
    green=r.get('green_observed_sim_s')
    with (run/'follow_decisions.jsonl').open() as f:
        for e in map(json.loads,f):
            d=e['data']
            if e['kind']=='header':
                cfg=d['follow_config'];config=FollowConfig(**cfg);motion=MotionConfig(**d['motion_config'])
            if e['kind']!='call':continue
            v,angle,o=d['output']
            if o.get('handoff_state')=='BYPASS':continue
            normal=o.get('normal',o)
            if not (normal.get('reason')=='local_follow' and normal.get('behavior')=='FOLLOW' and .001<v<=1.001):continue
            key=normal.get('source_first_s')
            if key in seen:continue
            seen.add(key);counts['distinct_scans']+=1
            if not normal.get('open_route_speed_enabled'):counts['not_current_open_route']+=1;continue
            arc=normal['swept_clearance_m'];span=arc+.20
            if normal['observed_path_ahead_m']<span:counts['observed_path_too_short']+=1;continue
            a,scan,h=restore_call(d['input'],motion)
            scene=motion_scene(scan,h,a['reference_s'],timing_verified=a['timing_verified'],epoch=a['epoch'],
                               rear_half_angle_deg=a['rear_half_angle_deg'])
            label=classify(scene,np.asarray(normal['path_points']),span,arc,angle,a['previous_steering_rad'],
                           a['speed_mps'],cfg,v,scene.t_ref_s-scene.source_first_s)
            counts[label]+=1
            if label=='additional_20cm_supported_in_sample':
                age=scene.t_ref_s-scene.source_first_s
                cap=pursue_path(normal['path_points'],a['speed_mps'],config,max(age,normal['boundary_age_s']),
                                speed_cap=cfg['open_route_speed_mps'])[0]
                free=span-cfg['standstill_gap_m']
                proposed=min(cap,normal.get('observed_length_speed_cap_mps',cap),braking_speed(free,age),
                             free/cfg['headway_s'])
                matched.append(dict(reference_s=a['reference_s'],after_traffic_green_s=
                    None if green is None else a['reference_s']-green,old_command_mps=v,arc_m=arc,
                    baseline_normal_mps=normal['target_speed_before_hold_mps'],normal_candidate_mps=proposed,
                    steering_rad=angle))
    return dict(run=run.name,counts=counts,matched=matched,scope='저장 센서로 20 cm 더 살펴볼 수 있는 판단 표본. 명령 변경·폐루프 시험·동적 차량 충돌 보장이 아니다.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('out',type=Path)
    a=p.parse_args();r=measure(a.run)
    with a.out.open('x',encoding='utf-8') as f:json.dump(r,f,ensure_ascii=False,indent=2)
    print(json.dumps(r,ensure_ascii=False))
