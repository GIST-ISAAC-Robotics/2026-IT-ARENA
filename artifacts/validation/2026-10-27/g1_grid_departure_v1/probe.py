"""현재 G1 선택기의 6초 그리드 이상 운동 비교. 공식 신호/ROS/MCU/실차 시험 아님."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import heapq
import json
import math
from pathlib import Path
import sys
import time
import zipfile

import numpy as np
import yaml
from sensor_msgs.msg import LaserScan
from shapely.geometry import MultiLineString, Polygon

ROOT=Path(__file__).resolve().parents[4]
for part in ('src/arena_autonomy','src/arena_vehicle_interface','scripts'):
    sys.path.insert(0,str(ROOT/part))
from arena_autonomy.follow_trace import freeze, snapshot_call
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_path import mask_scan
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.scene_motion import motion_scene
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec
from audit_grid_departure_motion import audit
from audit_local_pursuit_road import road_polygon
from g0_geometry import Footprint, TruthBox
from g2_surface_geometry import footprint_at
from probe_grid_departure_observation import motion_history
from probe_grid_follow import CAMERA, VEHICLE, RUNTIME, registry, wall_sections
from simulate_grid_departure import (IdealPlant, IndependentGuard, event_key,
    grid_peer_velocities, acquired_ranges, peer_active_time, stamp)
from surface_contact_eval import VehicleGeometry


def simulate(slot,others,segments,road,vehicle,sensor,cfg,policy):
    wheelbase=cfg.wheelbase_m
    lidar=vehicle['sensors']['lidar_2d']
    assert vehicle['sensors']['drive_feedback']['drive_motor_encoder']['counts_per_motor_revolution']==28
    assert vehicle['drivetrain']['motor']['gear_ratio']==15.
    assert vehicle['drivetrain']['wheel_radius_m']==.0325
    assert sensor['rate_hz']==10. and lidar['samples_per_scan']==500
    plant=IdealPlant(slot,wheelbase);spec=MotorEncoderSpec(28,15.,.0325)
    histories=[motion_history(wheelbase,sensor) for _ in range(2)]
    ctl=LocalFollow(cfg);guard=IndependentGuard(histories[1],cfg)
    # 독립 관측 상태: 제어기의 partial_policy 객체/승인 결과를 전달하지 않는다.
    guard.node.partial_policy=PartialObservationPolicy(allow_narrow_forward=True,
        surface_translation_enabled=True,lateral_metric_enabled=True)
    guard.node.previous_safe_steering=0.
    footprint=Footprint(vehicle['footprint']['length_m'],vehicle['footprint']['width_m'])
    peers=[TruthBox(s['x'],s['y'],.12,footprint,s['yaw_rad'],label=f"grid_{s['index']}") for s in others]
    velocities=grid_peer_velocities(slot,others,vehicle['footprint']['width_m'],policy)
    geometry=VehicleGeometry(wheelbase_m=wheelbase,track_width_m=vehicle['drivetrain']['track_width_m'])
    walls=MultiLineString(segments.tolist())
    events=[];event_id=0
    def enqueue(when,kind,value=None):
        nonlocal event_id
        heapq.heappush(events,event_key(when,event_id,kind,value));event_id+=1
    for i in range(601):enqueue(i*.01,'motor')
    for i in range(math.floor(6.*208)+1):enqueue(i/208,'gyro')
    for i in range(27500):enqueue(.5+i*.0002,'ray',i)
    for i in range(270):enqueue(.6018+i*.02,'control')
    old_count=0;last_steering=0.;latest=None;started=None;failures=0
    poses=[];times=[];scans=[];calls=[];measures=[]
    while events:
        ns,_,_,kind,value=heapq.heappop(events);now=ns*1e-9
        if now>6.:break
        plant.advance(now)
        if kind=='motor':
            count=spec.counts_from_motor_angle(plant.travel/.0325*15.)
            rate=spec.motor_rad_s_from_counts(count-old_count,.01) if now else 0.
            old_count=count;enqueue(now+.002,'motor_delivery',(now,spec.mean_wheel_rad_s(rate)))
            measures.append(dict(t_s=now,counts=count,motor_rad_s=rate))
        elif kind=='gyro':enqueue(now+.002,'gyro_delivery',(now,plant.gyro))
        elif kind in ('motor_delivery','gyro_delivery'):
            for h in histories:
                if kind=='motor_delivery':h.add_mean_wheel(*value)
                else:h.add_gyro(*value)
        elif kind=='ray':
            poses.append(plant.center);times.append(now)
            if len(poses)==500:
                values=acquired_ranges(poses,times,peers,velocities,segments,sensor,lidar['range_min_m'],lidar['range_max_m'])
                scan=LaserScan(angle_min=-math.pi,angle_increment=math.tau/500,time_increment=.0002,
                    scan_time=.1,range_min=lidar['range_min_m'],range_max=lidar['range_max_m'],ranges=values.tolist())
                scan.header.stamp=stamp(times[0]);scan.header.frame_id='laser_frame'
                raw=mask_scan(scan,sensor['masked_rear_half_angle_deg'])
                enqueue(now+.002,'scan_delivery',(raw,value//500))
                scans.append(dict(frame=value//500,first_s=times[0],ranges=freeze(raw.ranges),delivered_s=now+.002))
                poses=[];times=[]
        elif kind=='scan_delivery':latest=value
        elif kind=='control':
            if latest is None:continue
            scan,frame=latest;_,speeds=histories[0].wheels.available(now)
            measured=float(speeds[-1]) if len(speeds) else 0.
            captured=snapshot_call(scan,histories[0],now,0,measured,'left',last_steering,.02,True,sensor['masked_rear_half_angle_deg'])
            began=time.perf_counter_ns()
            try:
                scene=motion_scene(scan,histories[0],now,timing_verified=True,rear_half_angle_deg=sensor['masked_rear_half_angle_deg'])
                request=ctl.command(scene,histories[0],measured,'left',last_steering,.02,raw_scan=scan)
            except MotionUnavailable as error:
                failures+=1;request=(0.,last_steering,dict(reason=str(error)))
            explicit=now>=5.6
            if explicit:ctl.reset();request=(0.,last_steering,dict(reason='explicit_test_stop'))
            cost=(time.perf_counter_ns()-began)/1e6
            safe,status=guard.command(now,scan,request[0],request[1])
            plant.speed,plant.steering=safe.drive.speed,safe.drive.steering_angle
            last_steering=request[1]
            if plant.speed>0. and started is None:started=now
            x,y,yaw=plant.center;outline,wheels=footprint_at(x,y,yaw,plant.steering,geometry)
            active=peer_active_time(now)
            moved=[Polygon(p.footprint.polygon(p.x_m+v[0]*active,p.y_m+v[1]*active,p.yaw_rad)) for p,v in zip(peers,velocities)]
            overlap=any(q.intersection(p).area>1e-10 for q in outline.components for p in moved)
            gap=min(outline.distance(p) for p in moved);wall_gap=outline.distance(walls)
            outside=sum(w['patch'].difference(road).area>1e-10 for w in wheels.values())
            calls.append(dict(t_s=now,frame=frame,explicit_stop=explicit,captured_input=captured,
                measured_mps=measured,request_mps=request[0],request_steering=request[1],safe_mps=plant.speed,
                safe_steering=plant.steering,reason=request[2]['reason'],safety_reason=status['reason'],
                metadata=request[2],safety=status,controller_cost_ms=cost,truth_pose=[x,y,yaw],
                truth_travel_m=plant.travel,peer_overlap=overlap,minimum_peer_gap_m=gap,
                wall_gap_m=wall_gap,wheels_outside_road=outside))
            if overlap or wall_gap<=1e-9 or outside>2:
                plant.speed=0.;break
    summary=dict(controller='current_g1',config=asdict(cfg),rounded=False,fault=False,peer_motion_policy=policy,
        peer_stop_s=None,peer_resume_s=None,first_motion_s=started,calls=len(calls),progress_m=plant.travel,
        motion_input_failures=failures,minimum_peer_gap_m=min(c['minimum_peer_gap_m'] for c in calls),
        minimum_wall_gap_m=min(c['wall_gap_m'] for c in calls),maximum_wheels_outside_road=max(c['wheels_outside_road'] for c in calls),
        explicit_stop_checked=any(c['explicit_stop'] for c in calls),
        stopped_after_request=all(c['safe_mps']==0. for c in calls if c['explicit_stop']),
        reasons=dict(Counter(c['reason'] for c in calls)),safety_reasons=dict(Counter(c['safety_reason'] for c in calls)),
        scope='6s ideal bicycle and separate in-process guard; no ROS/MCU/physical brake or official start proof')
    return dict(summary=summary,calls=calls,scans=scans,motor_measurements=measures)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):parser.error('validation directory required')
    out.mkdir(parents=True,exist_ok=False)
    vehicle=yaml.safe_load(VEHICLE.read_text())['vehicle'];sensor=yaml.safe_load(CAMERA.read_text())['lidar']
    slots=registry(json.loads((RUNTIME/'scene.json').read_text()),(RUNTIME/'world.sdf').read_bytes(),vehicle)[:5]
    segments=wall_sections((RUNTIME/'world.sdf').read_bytes(),sensor['xyz_m'][2]);road=road_polygon(RUNTIME/'world.sdf')
    reference=ROOT/'artifacts/validation/2026-10-27/g1_guard_curve_v1/run2_corner/follow_decisions.jsonl'
    with reference.open() as f:header=json.loads(next(f))['data']
    cfg=FollowConfig(**header['follow_config'])
    files={Path(__file__),CAMERA,VEHICLE,RUNTIME/'world.sdf',RUNTIME/'scene.json',reference}
    files.update((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    for m in tuple(sys.modules.values()):
        name=getattr(m,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':files.add(p)
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in files:
            if p!=reference:z.write(p,p.relative_to(ROOT).as_posix())
    rows=[]
    for name,index,indices,policy in (('front_wait',2,(2,5),'ahead_only'),
            ('front_neighbor_depart',2,(2,5),'ahead_or_other_lane'),
            ('rear_neighbor_depart',5,(2,5),'ahead_or_other_lane'),
            ('middle_five_depart',1,(2,5,1,4,0),'ahead_or_other_lane')):
        selected=[s for s in slots if s['index'] in indices];ego=next(s for s in selected if s['index']==index)
        result=simulate(ego,[s for s in selected if s!=ego],segments,road,vehicle,sensor,cfg,policy)
        with (out/(name+'.json')).open('x') as f:json.dump(freeze(result),f,allow_nan=False)
        checked=audit(result,selected,MultiLineString(segments.tolist()),road,
            VehicleGeometry(wheelbase_m=cfg.wheelbase_m,track_width_m=vehicle['drivetrain']['track_width_m']),ego_index=index)
        rows.append(dict(name=name,ego_grid_index=index,summary=result['summary'],continuous_audit=checked))
    unchanged=all(hashlib.sha256((ROOT/n).read_bytes()).hexdigest()==h for n,h in hashes.items())
    report=dict(scope='exploratory same G1 configuration in grid; independent autonomous vehicles not simulated',
        rows=rows,sources_unchanged=unchanged,source_sha256=hashes)
    with (out/'summary.json').open('x',encoding='utf-8') as f:json.dump(freeze(report),f,ensure_ascii=False,indent=2,allow_nan=False)
    print(json.dumps(freeze(report['rows']),ensure_ascii=False))
    return 0 if unchanged and all(r['summary']['calls']==270 and r['summary']['explicit_stop_checked'] and
        r['summary']['stopped_after_request'] and r['continuous_audit']['passed'] for r in rows) else 1


if __name__=='__main__':raise SystemExit(main())
