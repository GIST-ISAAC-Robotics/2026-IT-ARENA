"""같은 우회 경로의 Pure Pursuit 주시거리만 바꾼다. 저장 장면 비교이며 구동 연결 없음."""
from pathlib import Path
import json,math
import numpy as np
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig
from arena_autonomy.local_follow import pursue_path
from arena_autonomy.bypass_entry_trial import entry_motion_region
from arena_autonomy.local_path import swept_limit,braking_speed
from arena_autonomy.pass_release_trial import lateral_config,region_blocked,SelectorConfig
from arena_autonomy.stopped_entry_centering import stopped_shaft_window

A=Path(__file__).resolve().parent
d=json.loads((A.parent/'ops_traffic_centered_entry_v1/blocked_snapshot.json').read_bytes())
cfg=FollowConfig(**d['follow_config']);i,scan,history=restore_call(d['input'],MotionConfig(**d['motion_config']))
scene=motion_scene(scan,history,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
path=np.asarray(d['path']);points=np.asarray(d['points']);padding=np.asarray(d['padding'])
age=scene.t_ref_s-scene.source_first_s
original_cap,original_steer,remaining=pursue_path(path,0.,cfg,age)
rear=path.copy();rear[:,0]+=cfg.wheelbase_m/2;distance=np.linalg.norm(rear,axis=1)
nearest=int(np.argmin(distance));ahead=np.flatnonzero((rear[:,0]>.10)&(np.arange(len(rear))>=nearest))
assert len(ahead)>=3 and max(distance[ahead])>=.25 and stopped_shaft_window(history,scene.t_ref_s)
rows=[]
for lookahead in (.30,.24,.18,.12):
    target=rear[ahead[np.argmin(abs(distance[ahead]-lookahead))]]
    curvature=2*target[1]/max(.01,float(target@target));desired=float(np.clip(math.atan(cfg.wheelbase_m*curvature),-cfg.max_steering_rad,cfg.max_steering_rad))
    if lookahead==.30:assert abs(desired-original_steer)<1e-12
    for label,previous in (('actual_previous',i['previous_steering_rad']),('stationary_aligned_hypothesis',desired)):
        steering=float(np.clip(desired,previous-3*i['dt_s'],previous+3*i['dt_s']))
        gap=min(swept_limit(points,a,cfg.wheelbase_m,horizon=2.5,margin=padding,length=cfg.length_m,width=cfg.width_m) for a in (steering,previous))
        speed=min(.35,original_cap,math.sqrt(cfg.lateral_acceleration_mps2/max(.15,abs(curvature))),braking_speed(gap,age))
        region=entry_motion_region(scene,0.,speed,steering,previous,i['dt_s'],cfg,lateral_config(cfg),padding)
        blocked=region_blocked(region,SelectorConfig())
        rows.append(dict(lookahead_m=lookahead,case=label,desired_rad=desired,steering_rad=steering,previous_rad=previous,
                         speed_mps=speed,clearance_m=gap,region=region,blocked=blocked,
                         regional_candidate=speed>0 and blocked is None))
out=dict(scope='동일 저장 경로·현재 점유·시차 여유·관측 조건, 주시거리만 비교. 현재 조향으로는 여전히 0명령이며 정지 후 정렬 가설의 양수 후보는 실제 주행 성공이 아님.',reference_s=scene.t_ref_s,source_sha256=d['source_sha256'],rows=rows)
(A/'result.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8');print(json.dumps(rows))
