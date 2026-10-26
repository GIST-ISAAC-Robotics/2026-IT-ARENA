#!/usr/bin/env python3
"""M5 저장 이상 운동을 1 ms 간격+이동 상계로 대조. 제어 입력/실차 인증 아님."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

import numpy as np
import yaml
from shapely.geometry import MultiLineString, Polygon

from g0_geometry import Footprint
from g2_surface_geometry import footprint_at
from g2_tracking_probe import bicycle_step
from probe_grid_follow import ROOT, CAMERA, VEHICLE, RUNTIME, registry, wall_sections
from audit_local_pursuit_road import road_polygon
from surface_contact_eval import VehicleGeometry, ackermann_wheel_angles


def interval_bound(speed, steering, duration, geometry):
    """자차 꼭짓점의 후축에 대한 최대 반경을 사용한 구간 이동 길이 상계."""
    radius = max(math.hypot((geometry.body_length_m+geometry.wheelbase_m)/2, geometry.body_width_m/2),
        math.hypot(geometry.wheelbase_m, geometry.track_width_m/2)+
        math.hypot(geometry.wheel_radius_m, geometry.wheel_width_m/2))
    return abs(speed)*duration*(1+abs(math.tan(steering)/geometry.wheelbase_m)*radius)


def audit(case, slots, walls, road, geometry, *, ego_index=5):
    if len({s['index'] for s in slots})!=len(slots) or sum(s['index']==ego_index for s in slots)!=1:
        raise ValueError('unique slots and one selected ego required')
    start = next(s for s in slots if s['index'] == ego_index)
    others = [s for s in slots if s['index'] != ego_index]
    expected=[start['x'],start['y'],start['yaw_rad']]
    if not np.allclose(case['calls'][0]['truth_pose'],expected,atol=1e-10,rtol=0.):
        raise ValueError('recorded first pose disagrees with audit start')
    cosine, sine = math.cos(start['yaw_rad']), math.sin(start['yaw_rad'])
    moving = [(s['x']-start['x'])*cosine+(s['y']-start['y'])*sine > 0 for s in others]
    footprint = Footprint(geometry.body_length_m, geometry.body_width_m)
    initial = dict(t_s=0., truth_pose=[start['x'], start['y'], start['yaw_rad']], safe_mps=0., safe_steering=0.)
    # 마지막 control 뒤 .02초도 종료 0출력으로 확인한다. 원래 시험 종료는 6초다.
    rows = [initial]+case['calls']
    min_peer = min_wall = math.inf
    max_outside = count = 0
    endpoint_error = 0.
    at_min = None

    def observe(pose, when, steering, ego_bound, peer_dt):
        nonlocal min_peer, min_wall, max_outside, count, at_min
        x, y, yaw = pose
        outline, wheels = footprint_at(x, y, yaw, steering, geometry)
        for other, enabled in zip(others, moving):
            speed = .5 if enabled else 0.
            traveled = speed*max(0., when-.5)
            shape = Polygon(footprint.polygon(other['x']+traveled*math.cos(other['yaw_rad']),
                other['y']+traveled*math.sin(other['yaw_rad']), other['yaw_rad']))
            lower = outline.distance(shape)-ego_bound-speed*peer_dt
            if lower < min_peer:
                min_peer, at_min = lower, dict(t_s=when, peer_slot=other['index'])
        min_wall = min(min_wall, outline.distance(walls)-ego_bound)
        outside = sum(not road.covers(w['patch'].buffer(ego_bound)) if ego_bound else
                      not road.covers(w['patch']) for w in wheels.values())
        max_outside = max(max_outside, outside)
        count += 1

    previous_steer = 0.
    for index, row in enumerate(rows):
        end = rows[index+1]['t_s'] if index+1 < len(rows) else 6.
        dt = end-row['t_s']
        if dt <= 0:
            raise ValueError('ordered movement knots required')
        x, y, yaw = row['truth_pose']
        speed, steer = row['safe_mps'], row['safe_steering']
        old_angles, new_angles = ackermann_wheel_angles(previous_steer, geometry), ackermann_wheel_angles(steer, geometry)
        angle_change = max(abs(a-b) for a,b in zip(old_angles, new_angles))
        # 순간 조향의 중간 바퀴 외곽을 외접 이동 반경으로 감싼다. 차체에도 같은 여유를 적용해 보수적이다.
        spin = math.hypot(geometry.wheel_radius_m, geometry.wheel_width_m/2)*angle_change
        observe((x,y,yaw),row['t_s'],previous_steer,spin,0.)
        rear = (x-geometry.wheelbase_m/2*math.cos(yaw), y-geometry.wheelbase_m/2*math.sin(yaw), yaw)
        parts = max(1, math.ceil(dt/.001))
        step = dt/parts
        bound = interval_bound(speed,steer,step,geometry)
        for i in range(parts):
            rx,ry,yaw = rear
            center = (rx+geometry.wheelbase_m/2*math.cos(yaw),ry+geometry.wheelbase_m/2*math.sin(yaw),yaw)
            observe(center,row['t_s']+i*step,steer,bound,step)
            rear = bicycle_step(*rear,speed,steer,step,geometry.wheelbase_m)
        if index+1 < len(rows):
            rx,ry,yaw = rear
            expected = rows[index+1]['truth_pose']
            error = max(abs(rx+geometry.wheelbase_m/2*math.cos(yaw)-expected[0]),
                abs(ry+geometry.wheelbase_m/2*math.sin(yaw)-expected[1]),abs(yaw-expected[2]))
            endpoint_error = max(endpoint_error,error)
        previous_steer = steer
    return dict(rounded=case['summary']['rounded'],fault=case['summary']['fault'],
        straight_priority=case['summary'].get('straight_priority',False),samples=count,
        hold_priority=case['summary'].get('hold_priority',False),
        minimum_peer_gap_lower_m=min_peer,minimum_wall_gap_lower_m=min_wall,
        maximum_outside_wheels_upper=max_outside,minimum_peer_location=at_min,
        reconstructed_endpoint_max_error=endpoint_error,
        passed=min_peer>0 and min_wall>0 and max_outside<=2 and endpoint_error<1e-8,
        scope='continuous bound in recorded ideal model only; not required 97/115mm lateral-budget acceptance')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    paths=[args.input,CAMERA,VEHICLE,RUNTIME/'world.sdf',RUNTIME/'scene.json',Path(__file__),
           ROOT/'tests/test_grid_departure_motion_audit.py']
    before={str(p.resolve().relative_to(ROOT)):p.read_bytes() for p in paths}
    with zipfile.ZipFile(args.output.with_suffix('.sources.zip'),'x',zipfile.ZIP_DEFLATED) as archive:
        for n,b in before.items():
            if n.endswith('.py'):archive.writestr(n,b)
    report=json.loads(args.input.read_text(encoding='utf-8'))
    for p in paths[1:5]:
        name=p.relative_to(ROOT).as_posix()
        if hashlib.sha256(p.read_bytes()).hexdigest()!=report['source_sha256'][name]:
            raise ValueError(f'geometry changed since run: {name}')
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar']
    slots=registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')),(RUNTIME/'world.sdf').read_bytes(),vehicle)[:5]
    walls=MultiLineString(wall_sections((RUNTIME/'world.sdf').read_bytes(),sensor['xyz_m'][2]).tolist())
    geometry=VehicleGeometry(wheelbase_m=vehicle['drivetrain']['wheelbase_m'],track_width_m=vehicle['drivetrain']['track_width_m'])
    road=road_polygon(RUNTIME/'world.sdf')
    cases=[audit(c,slots,walls,road,geometry) for c in report['cases']]
    unchanged=all((ROOT/n).read_bytes()==b for n,b in before.items())
    result=dict(cases=cases,inputs_unchanged=unchanged,passed=unchanged and all(c['passed'] for c in cases),
        input_sha256={n:hashlib.sha256(b).hexdigest() for n,b in before.items()})
    with args.output.open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
