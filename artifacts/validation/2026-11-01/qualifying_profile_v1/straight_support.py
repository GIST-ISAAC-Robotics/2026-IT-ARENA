"""반대 벽의 속도 상한을 실제 광선·기존 경로의 곡률까지 제한해 본 저장 입력 비교."""
from collections import Counter
import json
import math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,pursue_path
from arena_autonomy.local_path import local_path,braking_speed
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.turn_preview import _classify
from other_wall import BASE,OLD,overlap


def curve_cap(path,wheelbase,steering,lat):
    rear=path.copy();rear[:,0]+=wheelbase/2
    near=int(np.argmin(np.linalg.norm(rear,axis=1)))
    arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(rear,axis=0),axis=1))]
    dx=np.gradient(rear,axis=0);ddx=np.gradient(dx,axis=0)
    curve=np.abs(dx[:,0]*ddx[:,1]-dx[:,1]*ddx[:,0])/np.maximum(1e-8,np.linalg.norm(dx,axis=1)**3)
    return min(math.sqrt(lat/max(.15,abs(math.tan(steering)/wheelbase))),
               float(np.min(np.sqrt(lat/np.maximum(.15,curve[near:])+4*np.maximum(0.,arc[near:]-arc[near]-.2)))))


def value(scene,inp,details,steer,old,cfg):
    pts=np.asarray([r.hit_xy_m for r in scene.rays if r.status=='return'])
    side='right' if details['path_wall']=='left' else 'left'
    alt,_=local_path(pts,side,cfg.wall_offset_m,horizon=details['observed_search_horizon_m'])
    if alt is None:return old,'opposite_wall_missing'
    primary=np.asarray(details['path_points'])
    err=overlap(primary,alt)
    if err is None or err>cfg.measurement_padding_m:return old,'measured_paths_disagree'
    age=scene.t_ref_s-scene.source_first_s;speed=abs(inp['speed_mps'])
    alt_cap,_,length=pursue_path(alt,speed,cfg,age,speed_cap=cfg.open_route_speed_mps)
    if length<=details['observed_path_ahead_m']:return old,'not_longer'
    # 실제 Pure Pursuit 원하는 조향과 각도 변화 제한 후 발행하는 조향을 분리한다.
    _,wanted,_=pursue_path(primary,speed,cfg,age,speed_cap=cfg.open_route_speed_mps)
    cap=curve_cap(primary,cfg.wheelbase_m,wanted,cfg.lateral_acceleration_mps2)
    room=min(length,details['swept_clearance_m'])-cfg.standstill_gap_m
    proposed=min(cap,alt_cap,braking_speed(room,age),room/cfg.headway_s,details['observed_length_speed_cap_mps'])
    if proposed<=old+1e-7:return old,'other_limit'
    span=cfg.standstill_gap_m+max(.06+proposed*(age+.10)+proposed**2/4,proposed*cfg.headway_s)
    result=_classify(scene,alt,span,steer,inp['previous_steering_rad'],speed,vars(cfg),old,age)
    return (proposed if result=='observed_turn_clear' else old),result


def measure():
    path=OLD/'solo_10kmh/follow_decisions.jsonl'
    audit=json.loads((OLD/'solo_10kmh_metrics.json').read_bytes());start=audit['first_green_sample_s']
    end=start+audit['lap_s_from_first_green_sample'];reasons=Counter();seen=set();changes=[]
    with path.open(encoding='utf-8') as f:
        head=json.loads(next(f))['data'];cfg=FollowConfig(**head['follow_config']);motion=MotionConfig(**head['motion_config'])
        for event in map(json.loads,f):
            if event['kind']!='call':continue
            d=event['data'];inp=d['input'];t=inp['reference_s'];old,steer,status=d['output']
            if not start<=t<=end or not status.get('open_route_speed_enabled') or old<2.:continue
            src=status['source_first_s']
            if src in seen:continue
            seen.add(src);a,scan,h=restore_call(inp,motion)
            scene=motion_scene(scan,h,t,timing_verified=a['timing_verified'],epoch=a['epoch'],rear_half_angle_deg=a['rear_half_angle_deg'])
            new,why=value(scene,inp,status,steer,old,cfg);reasons[why]+=1
            if new>old+1e-7:changes.append(dict(t=t,old=old,new=new))
    return dict(scans=len(seen),reasons=dict(reasons),higher=len(changes),goal_scans=sum(r['new']>=cfg.open_route_speed_mps-1e-6 for r in changes),changes=changes,
        scope='현재 광선·주 경로 곡률/간격·반대 벽이 함께 지지할 때의 속도 후보. 기존 스캔별 첫 호출이며 실제 주행·조향 변경 아님.')


if __name__=='__main__':
    result=measure()
    with (BASE/'straight_support.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(result['reasons'],result['higher'],result['goal_scans'])
