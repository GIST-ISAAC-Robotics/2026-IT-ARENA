"""현재 반대쪽 벽에서 같은 통로가 더 멀리 보이는지 조사한다. 조향·실행 속도는 바꾸지 않는다."""
from collections import Counter
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.local_path import braking_speed,local_path
from arena_autonomy.scene_motion import motion_scene

BASE=Path(__file__).resolve().parent
OLD=BASE.parent/'corner_approach_v1'


def overlap(a,b):
    # x가 되돌아가는 커브에서는 직선의 보완 근거로 세지 않는다.
    a=a[a[:,0]>=0.];b=b[b[:,0]>=0.]
    if min(len(a),len(b))<3 or any(np.any(np.diff(p[:,0])<1e-6) for p in (a,b)):return None
    lo=max(a[0,0],b[0,0]);hi=min(a[-1,0],b[-1,0])
    if hi-lo<.65:return None
    grid=np.linspace(lo,hi,1+int((hi-lo)/.025))
    return float(np.max(np.abs(np.interp(grid,a[:,0],a[:,1])-np.interp(grid,b[:,0],b[:,1]))))


def study():
    audit=json.loads((OLD/'solo_10kmh_metrics.json').read_bytes())
    start=audit['first_green_sample_s'];end=start+audit['lap_s_from_first_green_sample']
    counts=Counter();seen=set();hits=[]
    with (OLD/'solo_10kmh/follow_decisions.jsonl').open(encoding='utf-8') as f:
        head=json.loads(next(f))['data'];cfg=FollowConfig(**head['follow_config']);motion=MotionConfig(**head['motion_config'])
        for raw in map(json.loads,f):
            if raw['kind']!='call':continue
            d=raw['data'];v,angle,details=d['output'];src=details.get('source_first_s');t=d['input']['reference_s']
            if not start<=t<=end or not details.get('open_route_speed_enabled') or v<2. or src in seen:continue
            seen.add(src);a,scan,h=restore_call(d['input'],motion)
            scene=motion_scene(scan,h,t,timing_verified=a['timing_verified'],epoch=a['epoch'],rear_half_angle_deg=a['rear_half_angle_deg'])
            pts=np.asarray([ray.hit_xy_m for ray in scene.rays if ray.status=='return'])
            opp='right' if details['path_wall']=='left' else 'left';age=t-src;goal=cfg.open_route_speed_mps
            other,info=local_path(pts,opp,cfg.wall_offset_m,horizon=details['observed_search_horizon_m'])
            if other is None:counts['no_other_observed_path']+=1;continue
            error=overlap(np.asarray(details['path_points']),other)
            if error is None or error>cfg.boundary_agreement_m:counts['different_or_nonstraight_path']+=1;continue
            cap,alt_steer,remain=pursue_path(other,abs(a['speed_mps']),cfg,age,speed_cap=goal)
            if remain<=details['observed_path_ahead_m']:counts['no_longer_path']+=1;continue
            space=details['swept_clearance_m']-cfg.standstill_gap_m
            upper=min(cap,braking_speed(space,age),space/cfg.headway_s)
            if upper<=v+1e-6:counts['no_additional_speed_budget']+=1;continue
            counts['possible_upper_gain']+=1
            hits.append(dict(t=t,old_mps=v,upper_mps=upper,original_path_m=details['observed_path_ahead_m'],
                other_path_m=remain,agreement_max_m=error,other_steer_difference_rad=alt_steer-angle))
    return dict(scans=len(seen),counts=dict(counts),upper_reaches_95pct=sum(r['upper_mps']>=.95*goal for r in hits),
                upper_reaches_goal=sum(r['upper_mps']>=goal-1e-6 for r in hits),samples=hits,
                scope='기존 10 km/h 단독 한 랩에서 요청 2 m/s 이상인 스캔당 첫 호출. 반대 벽 경로·기존 조향 원호를 쓴 낙관적 상한; 안전 판정이나 실제 명령이 아니다.')


if __name__=='__main__':
    out=study()
    with (BASE/'other_wall.json').open('x',encoding='utf-8') as f:json.dump(out,f,ensure_ascii=False,indent=2)
    print(out['counts'],out['upper_reaches_95pct'],out['upper_reaches_goal'])
