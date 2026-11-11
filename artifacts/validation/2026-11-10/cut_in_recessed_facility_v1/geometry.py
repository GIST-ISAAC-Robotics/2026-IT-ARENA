"""기존 합류 각본이 현재 시설과 겹치지 않는지 실행 전 확인한다. 평가 전용."""
import hashlib,json,math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
F=ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
P=ROOT/'artifacts/validation/2026-11-08/cut_in_early_newcourse_v2'
sys.path[:0]=[str(F),str(ROOT/'scripts')]
from replay_tools import load_launch
from build_experimental_track import rectangle
import validate_g1_traffic as ev


def main():
    world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
    cfg=ev.load_config(P/'new_track_course.json');case=cfg['cases'][0]
    line,scene=ev.load_centerline(world);s0=ev.actor_start_s(cfg,scene,0,case)
    route=ev.check_route(cfg,line,case,s0,ev.road_polygon(world/'world.sdf'),ev.obstacle_polygons(world/'world.sdf'))
    tree=ET.parse(world/'world.sdf')
    signal=load_launch()._signal_posts(tree.getroot().find('world'),'stl_61cm_provisional','official_v2026_11_06')
    parts=[(p['name'],rectangle(*p['center_xy_m'],signal['beam_yaw_rad'],p['along_m'],p['across_m'])) for p in signal['links']]
    meta=json.loads((F/'marker_posts_provisional.json').read_bytes())
    parts.extend((p['name'],rectangle(*p['center'],p['yaw'],p['length'],p['width'])) for p in meta['links'])
    rows={n:dict(minimum_m=math.inf,overlap_samples=0) for n,_ in parts}
    for t in np.arange(0.,case['duration_s']+.001,.005):
        x,y,yaw=ev.primary_pose_at(cfg,line,s0,case,float(t));body=rectangle(x,y,yaw,.2,.15)
        for n,part in parts:
            rows[n]['minimum_m']=min(rows[n]['minimum_m'],body.distance(part))
            rows[n]['overlap_samples']+=int(body.intersection(part).area>1e-9)
    files=[Path(__file__),P/'new_track_course.json',world/'world.sdf',F/'marker_posts_provisional.json',
        ROOT/'scripts/validate_g1_traffic.py',ROOT/'src/arena_bringup/launch/simulation.launch.py']
    result=dict(passed=all(r['overlap_samples']==0 for r in rows.values()),route=route,facility_clearance=rows,
        sources={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
        scope='5 ms actor schedule geometry only; no ego drive or continuous collision proof')
    with (A/'geometry.json').open('x') as f:f.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));assert result['passed']


if __name__=='__main__':main()
