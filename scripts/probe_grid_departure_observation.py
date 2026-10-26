#!/usr/bin/env python3
"""정지 자차와 움직이는 옆 열의 순차 LiDAR. 첫 출발 후보 관측만, 폐루프 아님."""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile

import numpy as np
import yaml

from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_scene import scene_from_scan
from arena_autonomy.startup_straight_candidate import StartupStraightCandidate
from arena_autonomy import rounded_occupancy_candidate
from arena_autonomy.lidar_motion import MotionHistory, MotionConfig
from arena_autonomy.m3_execution_trial import presteer_motion_quiet
from probe_grid_follow import ROOT, CAMERA, VEHICLE, RUNTIME, registry, wall_sections, json_value
from probe_corner_visibility_grid import cast_segments
from g0_geometry import Footprint, TruthBox, ray_box_distance
from analyze_grid_start_constraints import placement_checks


def moving_ranges(ego, peers, velocities, segments, sensor, stamps, minimum, maximum, bias=0.):
    """각 광선의 취득 때 상자를 이동시킨다. 벽/자차는 고정, 라벨/정답은 제어기에 주지 않는다."""
    if len(peers)!=len(velocities) or not math.isfinite(bias):raise ValueError('registered peer motion required')
    stamps=np.asarray(stamps,dtype=float)
    if len(stamps)!=500 or not np.isfinite(stamps).all() or np.any(np.diff(stamps)<=0):
        raise ValueError('ordered 500 ray timestamps required')
    c,s=math.cos(ego['yaw_rad']),math.sin(ego['yaw_rad'])
    sx,sy,z=sensor['xyz_m']
    origin=(ego['x']+c*sx-s*sy,ego['y']+s*sx+c*sy,z)
    angles=-math.pi+np.arange(len(stamps))*math.tau/len(stamps)+ego['yaw_rad']
    raw=cast_segments(segments,origin[:2],angles)
    labels=['wall' if math.isfinite(d) else None for d in raw]
    for i,(angle,t) in enumerate(zip(angles,stamps)):
        direction=(math.cos(angle),math.sin(angle),0.)
        for box,(vx,vy) in zip(peers,velocities):
            moved=replace(box,x_m=box.x_m+vx*(t-.5),y_m=box.y_m+vy*(t-.5))
            hit=ray_box_distance(origin,direction,moved,0.,maximum)
            if hit is not None and hit < raw[i]:raw[i],labels[i]=hit,box.label
    # 먼저 물리적 최소거리 가림을 판정한다. bias로 근접 무효를 유효화하지 않는다.
    ranges=np.where(raw<minimum,np.nan,np.where(raw<=maximum,raw+bias,np.inf))
    ranges=np.where(np.isfinite(ranges)&(ranges<minimum),np.nan,ranges)
    ranges=np.where(np.isfinite(ranges)&(ranges>maximum),np.inf,ranges)
    return ranges,labels


def motion_history(wheelbase,sensor):
    return MotionHistory(MotionConfig(wheel_radius_m=.0325,rear_axle_x_m=-wheelbase/2,
                                 lidar_x_m=sensor['xyz_m'][0],lidar_y_m=sensor['xyz_m'][1]))


def append_stationary_measurements(history,now):
    """현재까지의 합성 100 Hz 정지 측정만 추가해 원래 2초 이력 창을 유지한다."""
    first=0 if not history.wheels.times else round(history.wheels.times[-1]*100)+1
    for i in range(first,math.floor(now*100)+1):
        history.add_mean_wheel(i*.01,0.);history.add_gyro(i*.01,0.)


def evaluate(ego,others,segments,vehicle,sensor,*,bias=0.,delay=0.,fault=False,selection='opposite_column'):
    if sensor['rate_hz']!=10. or vehicle['sensors']['lidar_2d']['samples_per_scan']!=500:
        raise ValueError('registered 500 rays / 10 Hz required')
    if selection not in ('opposite_column','ahead_only'):raise ValueError('registered motion selection required')
    cfg=FollowConfig(wheelbase_m=vehicle['drivetrain']['wheelbase_m'],occupancy_timing_mode='per_ray_candidate',
                     boundary_evidence_mode='corner_distance_candidate',corridor_fallback_enabled=True)
    base=LocalFollow(cfg); h=motion_history(cfg.wheelbase_m,sensor)
    observers=[StartupStraightCandidate(cfg,enabled=True,rounded_occupancy=r) for r in (False,True)]
    footprint=Footprint(vehicle['footprint']['length_m'],vehicle['footprint']['width_m'])
    boxes=[TruthBox(p['x'],p['y'],.12,footprint,p['yaw_rad'],label=f"grid_{p['index']}") for p in others]
    # 평가용: 같은 열의 뒤차는 움직이지 않고 옆 열만 진행 방향으로 이동한다.
    # 앞/뒤 차 선택은 합성 장면 정의일 뿐 제어기 입력이나 출발 순서 규칙이 아니다.
    c,s=math.cos(ego['yaw_rad']),math.sin(ego['yaw_rad'])
    def moving(p):
        if selection=='ahead_only':return (p['x']-ego['x'])*c+(p['y']-ego['y'])*s>0.
        return abs(-(p['x']-ego['x'])*s+(p['y']-ego['y'])*c)>.1
    velocities=[(.5*math.cos(p['yaw_rad']),.5*math.sin(p['yaw_rad'])) if moving(p) else (0.,0.) for p in others]
    lidar=vehicle['sensors']['lidar_2d']; rows=[]; scans=[]; invalid=0; missing_motion=0
    for frame in range(30):
        stamps=.5+frame*.1+np.arange(500)*.0002
        ranges,labels=moving_ranges(ego,boxes,velocities,segments,sensor,stamps,lidar['range_min_m'],lidar['range_max_m'],bias)
        injected=bool(fault and 18<=frame<22)
        if injected:ranges[250]=np.nan
        scans.append(dict(first_s=float(stamps[0]),ranges=json_value(ranges),fault=injected))
        for repeat in range(5):
            now=float(stamps[-1]+.002+delay+repeat*.02)
            append_stationary_measurements(h,now)
            motion_ready=presteer_motion_quiet(h,now)
            missing_motion+=int(not motion_ready)
            scene=scene_from_scan(ranges.tolist(),angle_min=-math.pi,angle_increment=math.tau/500,
                range_min=lidar['range_min_m'],range_max=lidar['range_max_m'],acquired_at_s=stamps.tolist(),
                t_ref_s=now,sensor_xy_m=tuple(sensor['xyz_m'][:2]),rear_half_angle_deg=sensor['masked_rear_half_angle_deg'])
            baseline=base.command(scene,h,0.,'left',0.,.02)
            candidates=[o.evaluate(scene,h,baseline,0.) for o in observers]
            peer_now=[dict(p,x=p['x']+v[0]*(now-.5),y=p['y']+v[1]*(now-.5)) for p,v in zip(others,velocities)]
            geometry=placement_checks(ego,peer_now,footprint); invalid+=int(not geometry['nonoverlapping'])
            rows.append(dict(t_s=now,frame=frame,repeat=repeat,fault=injected,
                baseline_speed_mps=baseline[0],baseline_reason=baseline[2]['reason'],
                candidate=candidates,nonoverlapping=geometry['nonoverlapping'],motion_ready=motion_ready))
    return dict(slot=ego['index'],selection=selection,bias_m=bias,additional_delay_s=delay,fault_selected=fault,
        scans=scans,calls=rows,overlap_calls=invalid,missing_motion_calls=missing_motion,driving_authorized=False,
        summaries=[dict(rounded=r,selected_calls=sum(c['candidate'][i]['selected'] for c in rows),
            first_selected_s=next((c['t_s'] for c in rows if c['candidate'][i]['selected']),None),
            during_fault_selected=sum(c['candidate'][i]['selected'] for c in rows if c['fault']),
            reason_counts=dict(Counter(c['candidate'][i]['reason'] for c in rows))) for i,r in enumerate((False,True))])


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();out=args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):p.error('new validation subdirectory required')
    out.mkdir(parents=True,exist_ok=False)
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    slots=registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')),(RUNTIME/'world.sdf').read_bytes(),vehicle)[:5]
    segments=wall_sections((RUNTIME/'world.sdf').read_bytes(),sensor['xyz_m'][2])
    sources={Path(__file__),CAMERA,VEHICLE,RUNTIME/'scene.json',RUNTIME/'world.sdf',
             ROOT/'tests/test_grid_departure_observation.py',ROOT/'tests/test_rounded_occupancy_candidate.py',
             ROOT/'tests/test_startup_straight_state.py'}
    for module in tuple(sys.modules.values()):
        f=getattr(module,'__file__',None)
        if f:
            path=Path(f).resolve()
            if path.is_relative_to(ROOT) and path.suffix=='.py':sources.add(path)
    frozen={path.relative_to(ROOT).as_posix():path.read_bytes() for path in sorted(sources)}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for name,data in frozen.items():z.writestr(name,data)
    cases=[(ego,0.,0.,False,'ahead_only') for ego in slots]
    second=next(s for s in slots if s['index']==5)
    cases += [(second,b,d,f,'ahead_only') for b,d,f in ((.03,0.,False),(-.03,0.,False),(0.,.05,False),(0.,0.,True))]
    results=[evaluate(ego,[s for s in slots if s['index']!=ego['index']],segments,vehicle,sensor,
                      bias=b,delay=d,fault=f,selection=selection) for ego,b,d,f,selection in cases]
    report=dict(scope='moving peers, stationary ego, sequential synthetic rays; not closed-loop departure',
        cases=results,driving_authorized=False,source_sha256={n:hashlib.sha256(d).hexdigest() for n,d in frozen.items()},
        sources_unchanged=all((ROOT/n).read_bytes()==d for n,d in frozen.items()))
    (out/'report.json').write_text(json.dumps(json_value(report),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    summary=[{k:v for k,v in r.items() if k not in ('calls','scans')} for r in results]
    (out/'summary.json').write_text(json.dumps(json_value(summary),ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    invalid=sum(c['overlap_calls'] for c in results)
    missing_motion=sum(c['missing_motion_calls'] for c in results)
    fault_positive=sum(s['during_fault_selected'] for c in results for s in c['summaries'])
    print(json.dumps(dict(cases=len(results),sources_unchanged=report['sources_unchanged'],
                         overlap_calls=invalid,missing_motion_calls=missing_motion,during_fault_selected=fault_positive)))
    return 0 if report['sources_unchanged'] and not invalid and not fault_positive and not missing_motion else 1


if __name__=='__main__':raise SystemExit(main())
