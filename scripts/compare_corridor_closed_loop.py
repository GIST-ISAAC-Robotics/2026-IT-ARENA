#!/usr/bin/env python3
"""주행 미연결 회랑 후보의 합성 운동학 폐루프. ROS/Gazebo/실차 제어가 아니다."""
import argparse
from collections import Counter
from dataclasses import asdict
import json
import math
from pathlib import Path
from types import SimpleNamespace
import zipfile

import numpy as np
from shapely.geometry import Point, Polygon

from arena_autonomy.corridor_observer import CorridorObserver, compare_boundary_hold
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory, MotionUnavailable, advance_local
from arena_autonomy.lidar_observation import audit_front_observation
from arena_autonomy.local_follow import FollowConfig, LocalFollow, path_agrees
from arena_autonomy.local_path import braking_speed, mask_scan, swept_limit
from arena_autonomy.scene_motion import motion_scene, transport_points
from g0_geometry import Footprint
from probe_corner_corridor import ROOT, digest


SPEC = dict(inner_radius_m=.15, wall_spacing_m=.85, road_width_m=.45,
            dt_s=.01, control_hz=50, scan_hz=10, rays_per_scan=500, duration_s=8.,
            acceleration_mps2=.8, deceleration_mps2=2., steering_slew_rad_s=3.,
            range_noise_m=0., minimum_body_gap_m=.15, minimum_open_angle_rad=math.pi,
            feedback='ideal single mean-wheel equivalent and measured gyro, no count quantization',
            handoff='compare last issued path after measured-motion transport; stop experiment on disagreement',
            controller_truth_input=False, ros_connected=False, official_course=False,
            continuous_collision_verified=False, real_vehicle_verified=False)


def integrate(pose, speed, yaw_rate, dt, wheelbase=.135):
    """일정 twist의 후륜축 적분을 차체 중심으로 되돌리는 평가 전용 함수."""
    x,y,yaw = pose
    dx,dy,da = advance_local(speed,yaw_rate,dt)
    c,s = math.cos(yaw),math.sin(yaw)
    return np.array([x-wheelbase/2*c+c*dx-s*dy+wheelbase/2*math.cos(yaw+da),
                     y-wheelbase/2*s+s*dx+c*dy+wheelbase/2*math.sin(yaw+da),yaw+da])


def poses_at(truth, stamps):
    """각 10 ms 구간의 실제 ZOH twist로 취득 중 자세 생성. 미래 표본 보간 없음."""
    if min(stamps) < truth[0,0]-1e-9 or max(stamps) > truth[-1,0]+1e-9:
        raise ValueError('acquisition outside saved plant history')
    indices=np.maximum(0,np.searchsorted(truth[:,0],stamps,side='right')-1)
    return np.array([integrate(truth[i,1:4],truth[i,4],truth[i,5],max(0.,t-truth[i,0]))
                     for i,t in zip(indices,stamps)])


def circle_hits(origins, directions, center, radius):
    delta=origins-np.asarray(center)
    along=np.sum(delta*directions,axis=1)
    disc=along*along-np.sum(delta*delta,axis=1)+radius*radius
    root=np.sqrt(np.maximum(0.,disc))
    near,far=-along-root,-along+root
    return np.where(disc>=0,np.where(near>=0,near,np.where(far>=0,far,np.inf)),np.inf)


def box_hits(origins, directions, center, yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    rot=np.array([[c,-s],[s,c]])
    o,d=(origins-np.asarray(center))@rot,directions@rot
    near,far=np.full(len(o),-np.inf),np.full(len(o),np.inf)
    valid=np.ones(len(o),dtype=bool)
    for axis,half in enumerate((.10,.075)):
        parallel=np.abs(d[:,axis])<1e-14
        valid &= ~parallel | (np.abs(o[:,axis])<=half)
        a,b=np.full(len(o),-np.inf),np.full(len(o),np.inf)
        np.divide(-half-o[:,axis],d[:,axis],out=a,where=~parallel)
        np.divide(half-o[:,axis],d[:,axis],out=b,where=~parallel)
        near,far=np.maximum(near,np.minimum(a,b)),np.minimum(far,np.maximum(a,b))
    return np.where(valid & (far>=near) & (far>=0),np.where(near>=0,near,far),np.inf)


def environment(sign, kind):
    radius=SPEC['inner_radius_m']
    center=np.array([0.,sign*(radius+.425)])
    box=None
    if kind=='blocked':
        angle=.95
        box=(np.array([(radius+.425)*math.sin(angle),
                       sign*(radius+.425)*(1-math.cos(angle))]),sign*angle)
    elif kind!='open':
        raise ValueError('unknown bounded case')
    return center,box


def scan_at(now, truth, sign, kind):
    n=SPEC['rays_per_scan']
    first,increment=now-.1,.1/n
    stamps=first+np.arange(n)*increment
    poses=poses_at(truth,stamps)
    yaw=poses[:,2]
    origins=poses[:,:2]+.06*np.column_stack((np.cos(yaw),np.sin(yaw)))
    bearings=-math.pi+np.arange(n)*math.tau/n+yaw
    directions=np.column_stack((np.cos(bearings),np.sin(bearings)))
    center,box=environment(sign,kind)
    ranges=np.minimum(circle_hits(origins,directions,center,SPEC['inner_radius_m']),
                      circle_hits(origins,directions,center,SPEC['inner_radius_m']+.85))
    if box is not None:
        ranges=np.minimum(ranges,box_hits(origins,directions,*box))
    ranges=np.where(ranges<.05,np.nan,ranges)  # 최소 거리 안의 반환을 뒤벽으로 대체하지 않는다.
    nanos=round(first*1e9)
    return SimpleNamespace(header=SimpleNamespace(frame_id='laser_frame',
        stamp=SimpleNamespace(sec=nanos//10**9,nanosec=nanos%10**9)),ranges=ranges.tolist(),
        angle_min=-math.pi,angle_increment=math.tau/n,range_min=.05,range_max=12.,time_increment=increment)


def footprint_audit(pose,sign,kind):
    center,box=environment(sign,kind)
    polygon=Polygon(Footprint().polygon(*pose))
    near=polygon.distance(Point(center))
    far=max(np.linalg.norm(np.asarray(p)-center) for p in polygon.exterior.coords)
    inside=near>=SPEC['inner_radius_m']+.20 and far<=SPEC['inner_radius_m']+.65
    collision=near<SPEC['inner_radius_m'] or far>SPEC['inner_radius_m']+.85
    gap=None
    if box is not None:
        other=Polygon(Footprint().polygon(*box[0],box[1]))
        gap=polygon.distance(other)
        collision |= polygon.intersection(other).area>1e-10
    return dict(inside_road=bool(inside),collision=bool(collision),body_gap_m=gap)


def handoff_agreement(previous, path, now, history, config):
    if previous is None:
        return None
    when,old_path=previous
    if now-when>config.memory_ttl_s or now<when:
        return None
    # 이전 발행 경로는 전환 비교 전용. 현재 경계 증거나 새 경로로 사용하지 않는다.
    carried=transport_points(old_path,history,when,now)
    return bool(path_agrees(np.asarray(path),carried,config.boundary_agreement_m))


def run_case(sign,kind,mode,duration=None,world=None):
    if sign not in (-1,1) or mode not in ('baseline','corridor'):
        raise ValueError('bounded case required')
    cfg=FollowConfig(boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate')
    follow,observer=LocalFollow(cfg),CorridorObserver(cfg)
    history=MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-.0675,lidar_x_m=.06))
    side='left' if sign==1 else 'right'
    pose=np.zeros(3) if world is None else world.initial_pose()
    speed=steer=requested=commanded=0.
    truth,rows,handoffs=[],[],[]
    previous_path=previous_mode=None
    scan=None
    failure=None
    dt=SPEC['dt_s']
    steps=round((duration or SPEC['duration_s'])/dt)
    for index in range(steps+1):
        now=index*dt
        yaw_rate=speed*math.tan(steer)/cfg.wheelbase_m
        truth.append([now,*pose,speed,yaw_rate])
        history.add_mean_wheel(now,speed/.0325)
        history.add_gyro(now,yaw_rate)
        if index>=10 and index%10==0:
            scan=mask_scan(scan_at(now,np.asarray(truth),sign,kind) if world is None else
                           world.scan_at(now,np.asarray(truth)))
        if index%2==0 and scan is not None:
            try:
                scene=motion_scene(scan,history,now,timing_verified=True)
                base=follow.command(scene,history,speed,side,commanded,.02)
                proposal=observer.evaluate(scene,history,speed,side,commanded,.02)
                selection=compare_boundary_hold(base,proposal)
                selected=mode=='corridor' and selection['selected']
                value=selection['proposed_speed_mps'] if selected else base[0]
                angle=selection['proposed_steering_rad'] if selected else base[1]
                origin='corridor' if selected else 'baseline'
                path=proposal['path'] if selected else base[2].get('path_points')
                if value>0 and path is not None:
                    if previous_mode is not None and origin!=previous_mode:
                        agree=handoff_agreement(previous_path,path,now,history,cfg)
                        handoffs.append(dict(time_s=now,from_mode=previous_mode,to_mode=origin,path_agrees=agree))
                        if agree is False:
                            failure='path_disagreement_at_mode_transition'
                    previous_path,previous_mode=(now,np.asarray(path)),origin
                # 0명령 중에도 마지막 양수 경로를 전환 감사에만 보존한다.
                # handoff_agreement의 0.3초 만료를 적용하며 제어/경계 기억으로 사용하지 않는다.
                angle=float(np.clip(angle,commanded-.06,commanded+.06))
                points=np.asarray(scene.occupied_endpoints_xy_m)
                observed=audit_front_observation(scan.ranges,scan.angle_min,scan.angle_increment,.05,12.)
                gap=min(swept_limit(points,a,cfg.wheelbase_m,length=cfg.length_m,width=cfg.width_m)
                        for a in (angle,commanded))
                safe=braking_speed(gap,now-scene.source_first_s) if observed.reason=='ok' else 0.
                requested=min(value,safe,requested+SPEC['acceleration_mps2']*.02)
                commanded=angle
                row=dict(t=now,pose=pose.tolist(),speed_mps=speed,command_mps=requested,
                         command_steering_rad=commanded,actual_steering_rad=steer,
                         baseline_reason=base[2]['reason'],observer_reason=proposal['reason'],
                         selected=selected,guard_cap_mps=safe)
            except (MotionUnavailable,ValueError) as error:
                follow.reset(); observer.reset()
                requested=0.
                previous_path=previous_mode=None
                row=dict(t=now,pose=pose.tolist(),speed_mps=speed,command_mps=0.,selected=False,
                         baseline_reason='input_stop',observer_reason='input_stop',error=str(error))
            rows.append(row)
        audit=footprint_audit(pose,sign,kind) if world is None else world.audit(pose)
        if not audit['inside_road'] or audit['collision'] or (audit['body_gap_m'] is not None and audit['body_gap_m']<.15):
            failure='sampled_footprint_or_gap_violation'
        if failure or index==steps:
            break  # 평가 전용으로 시험 중단. 정답을 제어기에 제공하거나 회피에 사용하지 않음.
        pose=integrate(pose,speed,yaw_rate,dt)
        speed+=float(np.clip(requested-speed,-SPEC['deceleration_mps2']*dt,SPEC['acceleration_mps2']*dt))
        steer+=float(np.clip(commanded-steer,-SPEC['steering_slew_rad_s']*dt,SPEC['steering_slew_rad_s']*dt))
    if world is None:
        center,_=environment(sign,kind)
        angles=np.unwrap(np.arctan2(np.asarray(truth)[:,2]-center[1],np.asarray(truth)[:,1]-center[0]))
        progress=float(sign*(angles[-1]-angles[0]))
        progress_key,minimum='progress_angle_rad',math.pi
    else:
        progress=world.progress(np.asarray(truth))
        progress_key,minimum='progress_distance_m',world.minimum_progress_m
    audits=[footprint_audit(r[1:4],sign,kind) if world is None else world.audit(r[1:4]) for r in truth]
    gaps=[a['body_gap_m'] for a in audits if a['body_gap_m'] is not None]
    tail=[r for r in rows if r['t']>=now-.5]
    hold=bool(tail) and all(r['command_mps']==0 and abs(r['speed_mps'])<.05 for r in tail)
    checks=dict(no_sampled_collision=not any(a['collision'] for a in audits),
                inside_sampled_road=all(a['inside_road'] for a in audits),
                minimum_body_gap=not gaps or min(gaps)>=.15,
                no_handoff_disagreement=not any(h['path_agrees'] is False for h in handoffs))
    checks['half_course_progress' if kind=='open' else 'stationary_hold_last_half_second']=progress>=minimum if kind=='open' else hold
    return dict(side=side,kind=kind,mode=mode,checks=checks,passed=all(checks.values()),
                failure=failure,elapsed_sim_s=now,**{progress_key:progress},
                minimum_body_gap_m=min(gaps) if gaps else None,selected_calls=sum(r['selected'] for r in rows),
                baseline_reasons=dict(Counter(r['baseline_reason'] for r in rows)),
                observer_reasons=dict(Counter(r['observer_reason'] for r in rows)),handoffs=handoffs,
                final_speed_mps=speed,trace=rows,ros_connected=False,real_vehicle_verified=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    sources={str(p.relative_to(ROOT)):digest(p) for folder in ('src/arena_autonomy/arena_autonomy',
             'src/arena_vehicle_interface/arena_vehicle_interface') for p in (ROOT/folder).glob('*.py')}
    for p in ('scripts/compare_corridor_closed_loop.py','scripts/g0_geometry.py',
              'scripts/probe_corner_corridor.py','tests/test_corridor_closed_loop.py'):
        sources[p]=digest(ROOT/p)
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in sources:archive.write(ROOT/p,p)
    results=[]
    for sign in (1,-1):
        for kind in ('open','blocked'):
            for mode in ('baseline','corridor'):
                result=run_case(sign,kind,mode)
                name=f'{sign}_{kind}_{mode}.json'
                with (args.output/name).open('x') as stream:json.dump(result.pop('trace'),stream,allow_nan=False)
                result.update(trace=name,trace_sha256=digest(args.output/name))
                results.append(result)
                print(json.dumps(result),flush=True)
    report=dict(scope='synthetic_kinematic_feedback_comparison_only',spec=SPEC,
                follow_config=asdict(FollowConfig(boundary_evidence_mode='corner_distance_candidate',occupancy_timing_mode='per_ray_candidate')),
                source_sha256=sources,source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()),results=results,
                runtime_changed=False,independent_ros_protection_tested=False,official_lap_verified=False)
    with (args.output/'report.json').open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    return 0 if report['source_unchanged'] else 2  # 모형 실패는 결과에 보존. 도구 성공과 주행 통과를 분리.


if __name__=='__main__':raise SystemExit(main())
