"""G2 1회 시험의 배치를 실행 전에 공식 도로·벽·신호 시험본과 대조한다. 주행 증거가 아니다."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

import numpy as np
from shapely.strtree import STRtree

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
for d in ('scripts','src/arena_autonomy','src/arena_vehicle_interface'):sys.path.insert(0,str(ROOT/d))
from audit_local_pursuit_road import road_polygon
from build_experimental_track import obstacle_polygons, rectangle
from audit_start_signal_geometry import rotation
from traffic_road_route import check_onroad_route
import g1_traffic_eval as ev

SIGNAL=ROOT/'artifacts/validation/2026-10-27/official_departure_v2'


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    output=BASE/'preflight.json'
    if output.exists():raise FileExistsError(output)
    manifest=json.loads((BASE/'fixture/manifest.json').read_text(encoding='utf-8'))
    config=json.loads((BASE/'g2_config_v2_frame.json').read_text(encoding='utf-8'))
    changed=[p for p,h in manifest['sources'].items() if sha(ROOT/p)!=h]
    v2_changed=[p for p,h in manifest['v2_signal_manifest_sources'].items() if sha(ROOT/p)!=h]
    world=ROOT/manifest['world_directory']
    scene=json.loads((world/'scene.json').read_text(encoding='utf-8'))
    route=np.genfromtxt(world/'centerline.csv',delimiter=',',names=True)
    centerline=ev.Centerline(route['x_m'],route['y_m'],route['s_m'],scene['track']['lap_length_m'])
    road=road_polygon(world/'world.sdf');obstacles=obstacle_polygons(world/'world.sdf');tree=STRtree(obstacles)
    case=config['cases'][0];spawn=manifest['spawn']
    s0=spawn['s_m']+case['actor_start_center_gap_m']
    def pose_at(cl,s,schedule,elapsed,lateral):
        x,y,yaw=cl.pose(s);return x-lateral*math.sin(yaw),y+lateral*math.cos(yaw),yaw
    try:
        actor_route=check_onroad_route(config,centerline,case,s0,road,obstacles,
            lambda c,cl,k,s:{'actor_start_s_m':s,'actor_end_s_m':s},pose_at)
        actor_error=None
    except ValueError as error:
        actor_route,actor_error=None,str(error)
    actor=rectangle(*pose_at(centerline,s0,[],0.,case['actor_lateral_offset_m']),.20,.15)
    ego=rectangle(spawn['x'],spawn['y'],spawn['yaw_rad'],.20,.15)
    # 합성 sequence 최종 진행 약3.41 m + 정지 여유1 m까지 같은 직선인지 본다.
    corner=centerline.unwrap(config['first_corner_s_m'],spawn['s_m'])
    if corner<spawn['s_m']:corner+=centerline.lap
    straight=[]
    for along in np.arange(spawn['s_m']-.5,spawn['s_m']+4.41,.05):
        x,y,yaw=centerline.pose(along)
        straight.append(dict(s=float(along),yaw=float(yaw)))
    yaws=np.unwrap([r['yaw'] for r in straight])
    # v2 신호 시험본의 광원 10개에 대한 뒤쪽30 cm 카메라 시선(상자 형상·표적 포함).
    spec=importlib.util.spec_from_file_location('v2_preflight',SIGNAL/'preflight.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    import xml.etree.ElementTree as ET
    boxes=module.boxes(ET.parse(world/'world.sdf').find('world'))
    ax,ay,ayaw=pose_at(centerline,s0,[],0.,case['actor_lateral_offset_m'])
    boxes.append(('g2_target',np.array([ax,ay,.07]),rotation(0.,0.,ayaw),np.array([.10,.075,.07])))
    v2=json.loads((world/'manifest.json').read_text(encoding='utf-8'))
    camera=np.array([spawn['x'],spawn['y'],0.])+rotation(0.,0.,spawn['yaw_rad'])@np.array([-.075,0.,.30])
    lamps={l['name']:module.hits(camera,np.asarray(l['xyz_m']),boxes) for l in v2['lamps']}
    result=dict(
        scope='static layout/route/line-of-sight check before the single G2 run; not a driving result',
        sources_unchanged=not changed and not v2_changed,changed_sources=changed+v2_changed,
        spawn=spawn,ego_inside_road=bool(road.contains(ego)),ego_wall_distance_m=float(min(ego.distance(obstacles[int(i)]) for i in tree.query(ego.buffer(1.)))),
        actor_start_s_m=s0,actor_route=actor_route,actor_route_error=actor_error,
        actor_outside_road_area_m2=float(actor.difference(road).area),
        actor_road_edge_distance_m=float(actor.distance(road.boundary)),
        initial_body_gap_m=float(ego.distance(actor)),
        straight_ahead_m=float(corner-spawn['s_m']),straight_heading_span_rad=float(np.ptp(yaws)),
        planned_travel_m=4.41,
        signal_lamp_blocking=lamps,signal_unobstructed=all(not v for v in lamps.values()),
        scan_plane_crosses_target=.095<.14)
    result['passed']=bool(result['sources_unchanged'] and result['ego_inside_road'] and actor_error is None
        and result['straight_ahead_m']>=5.4 and result['straight_heading_span_rad']<.02 and result['signal_unobstructed'])
    result['source_sha256']={str(p.relative_to(ROOT)):sha(p) for p in (Path(__file__),BASE/'shape.py',
        BASE/'g2_config_v2_frame.json',*sorted((BASE/'fixture').glob('*')))}
    with output.open('x',encoding='utf-8',newline='\n') as stream:stream.write(json.dumps(result,ensure_ascii=False,indent=2))
    print(json.dumps({k:result[k] for k in ('passed','sources_unchanged','changed_sources','ego_inside_road','ego_wall_distance_m',
        'actor_route_error','actor_outside_road_area_m2','actor_road_edge_distance_m','initial_body_gap_m','straight_ahead_m',
        'straight_heading_span_rad','signal_unobstructed')},ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
