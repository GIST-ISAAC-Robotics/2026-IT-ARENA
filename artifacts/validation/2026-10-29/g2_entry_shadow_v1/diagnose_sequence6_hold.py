"""sequence6의 이상 센서 생산자를 재구성하여 정지 이유를 분리한다.

평가 자세는 광선 합성에만 사용한다. 제어기에는 센서/운동 이력만 전달하며
원래 명령과 수치가 일치하는지 먼저 대조한다. 벽 팽창 변경은 진단 전용이다.
"""
import hashlib
import json
import math
from pathlib import Path
import numpy as np
from arena_autonomy.bypass_entry_trial import EntryGuide, observed_straight_wall
from arena_autonomy.local_follow import FollowConfig, pursue_path
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_path import swept_limit, braking_speed
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.pass_release_trial import motion_region, lateral_config, region_blocked, SelectorConfig
from probe_bypass_entry_sequence import render_scan

BASE=Path(__file__).resolve().parent
OUTPUT=BASE/'sequence6_hold_diagnosis_v2.json'
if OUTPUT.exists():raise ValueError('preserve existing result')
cases=[]
for filename in ('y0.0_restart0.json','y0.0_restart1.json'):
    raw_bytes=(BASE/'sequence6'/filename).read_bytes(); saved=json.loads(raw_bytes)
    cfg=FollowConfig();guide=EntryGuide(cfg)
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06,history_s=5.))
    physical=[]
    for k in range(25):
        at=k*.02;physical.append(dict(t=at,rear_pose=[-.0675,0.,0.],speed=0.,steering=0.))
        history.add_mean_wheel(at,0.);history.add_gyro(at,0.)
    previous=0.;matched=0;diagnostics=[];max_difference=0.
    for k,r in enumerate(saved['records']):
        now=r['t'];cx,cy,yaw=r['evaluation']['center_pose']
        rear=[cx-.0675*math.cos(yaw),cy-.0675*math.sin(yaw),yaw]
        physical.append(dict(t=now,rear_pose=rear,speed=r['actual_speed_mps'],steering=previous))
        if k%5==0:scan=render_scan(physical,now,saved['target_y'],saved['target_restarts'])
        scene=motion_scene(scan,history,now,timing_verified=True,rear_half_angle_deg=30.)
        _,values=history.wheels.available(now);speed=float(values[-1])
        decision=guide.evaluate(scene,history,speed,previous,.02)
        selected=decision['selected'];recorded=r['selected']
        if (selected is None)!=(recorded is None):raise AssertionError(f'selection changed {filename} {now}')
        if selected:
            difference=max(abs(selected[key]-recorded[key]) for key in ('speed_mps','steering_rad','offset_m'))
            max_difference=max(max_difference,difference)
            if difference>1e-9:raise AssertionError(f'command changed {filename} {now}: {difference}')
        matched+=1
        if r['phase']=='entry' and any(abs(now-at)<1e-8 for at in (2.58,4.08,8.38)):
            fit,wall=observed_straight_wall(scene,cfg,guide.side)
            slope,intercept,lo,hi=fit;sign=1 if guide.side=='left' else -1
            norm=math.sqrt(1+slope*slope)
            points=np.asarray(scene.occupied_endpoints_xy_m).reshape(-1,2)
            physical_intercept=intercept+sign*cfg.wall_offset_m*norm
            shift_x=-sign*cfg.wall_offset_m*slope/norm
            wall_mask=(abs(points[:,1]-slope*points[:,0]-physical_intercept)/norm<=.015)&(
                points[:,0]>=lo+shift_x)&(points[:,0]<=hi+shift_x)
            span=float(np.ptp(points[wall_mask,0])) if np.any(wall_mask) else 0.
            if span<.65:wall_mask[:]=False
            line_mask=(abs(points[:,1]-slope*points[:,0]-physical_intercept)/norm<=.015)&(
                points[:,0]>=-.4)&(points[:,0]<=1.5)
            indices=np.flatnonzero(line_mask)
            indices=indices[np.argsort(points[indices,0])]
            gaps=np.linalg.norm(np.diff(points[indices],axis=0),axis=1)
            limits=.10+.035*np.minimum(np.linalg.norm(points[indices[:-1]],axis=1),np.linalg.norm(points[indices[1:]],axis=1))
            line_mask[:]=False
            for piece in np.split(indices,np.flatnonzero(gaps>limits)+1):
                if len(piece)>=12 and np.ptp(points[piece,0])>=.65:
                    line_mask[piece]=True
            age=now-scene.source_first_s
            cap,steer,_=pursue_path(guide.path,speed,cfg,age)
            steer=float(np.clip(steer,previous-.06,previous+.06))
            pads=per_ray_padding(scene,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
            modes=[]
            for name,pad in (('all_returns_may_move',pads),('outer_wall_static_assumption',
                    np.where(wall_mask,.025+cfg.measurement_padding_m,pads)),
                    ('long_current_collinear_returns_static_assumption',
                    np.where(line_mask,.025+cfg.measurement_padding_m,pads))):
                distances=[swept_limit(points,a,cfg.wheelbase_m,horizon=2.5,margin=pad,
                    length=cfg.length_m,width=cfg.width_m) for a in (steer,previous)]
                target=min(.35,cap,braking_speed(min(distances),age))
                # 관측 영역은 현재 기본 함수를 그대로 사용한다. 벽 free 팽창 할인 없음.
                region=motion_region(scene,speed,target,steer,previous,.02,cfg,lateral_config(cfg))
                modes.append(dict(mode=name,clearances=distances,target_speed_mps=target,
                    region=region,blocked=region_blocked(region,SelectorConfig())))
            diagnostics.append(dict(t=now,actual_speed=speed,steer=steer,previous=previous,
                occupied_points=len(points),outer_wall_points=int(wall_mask.sum()),
                current_collinear_points=int(line_mask.sum()),span_m=span,modes=modes))
        next_speed=float(np.clip(r['safe'][0],max(0.,r['actual_speed_mps']-2*.02),r['actual_speed_mps']+.02))
        previous=float(r['safe'][1]);physical[-1].update(speed=next_speed,steering=previous)
        history.add_mean_wheel(now,next_speed/.0325);history.add_gyro(now,next_speed*math.tan(previous)/.135)
    cases.append(dict(file=filename,source_sha256=hashlib.sha256(raw_bytes).hexdigest(),matched=matched,
        maximum_command_difference=max_difference,diagnostics=diagnostics))
OUTPUT.write_text(json.dumps(dict(scope='reconstructed_ideal_sensor_producer_not_original_bag',
    connected_to_drive=False,cases=cases),indent=2)+'\n',encoding='utf-8')
print(json.dumps(cases))
