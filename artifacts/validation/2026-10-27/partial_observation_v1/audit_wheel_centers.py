"""차체 일부 이탈과 네 바퀴 중심의 노면 위치를 분리한 사후 평가. 제어 입력 아님."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import yaml
from shapely.geometry import Point, box
from shapely.prepared import prep

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from audit_local_pursuit_road import road_polygon


def wheel_points(x,y,yaw,wheelbase,track):
    c,s = math.cos(yaw),math.sin(yaw)
    return [(x+c*a-s*b,y+s*a+c*b) for a,b in
            ((wheelbase/2,track/2),(wheelbase/2,-track/2),(-wheelbase/2,track/2),(-wheelbase/2,-track/2))]


def counts(road,x,y,yaw,wheelbase=.135,track=.135):
    return [not road.covers(Point(p)) for p in wheel_points(x,y,yaw,wheelbase,track)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    args = parser.parse_args()
    road_test = box(-2,-.225,2,.225)
    assert sum(counts(road_test,0,.155,0))==0   # 차체(.075반폭)는 밖으로 나가지만 바퀴 중심은 안이다.
    assert sum(counts(road_test,0,.20,0))==2
    assert sum(counts(road_test,0,.31,0))==4
    run = args.run.resolve()
    world = run/'runtime_world.sdf'
    vehicle = ROOT/'src/arena_description/config/vehicle.yaml'
    cfg = yaml.safe_load(vehicle.read_text())['vehicle']['drivetrain']
    road = road_polygon(world)
    prepared = prep(road)
    dense = run/'ego_poses.jsonl'
    trajectory = run/'trajectory.jsonl'
    if dense.is_file():
        poses = [json.loads(line) for line in dense.read_text().splitlines()]
        input_path = dense
    else:
        records = [json.loads(line) for line in trajectory.read_text().splitlines()]
        poses = [[r['time'],r['x'],r['y'],r['yaw']] for r in records]
        input_path = trajectory
    assert poses and all(b[0]>a[0] for a,b in zip(poses,poses[1:])), 'strictly increasing pose times required'
    hist = Counter()
    events = []
    active = None
    closest = math.inf
    for stamp,x,y,yaw in poses:
        wheels = wheel_points(x,y,yaw,cfg['wheelbase_m'],cfg['track_width_m'])
        outside = [i for i,p in enumerate(wheels) if not prepared.covers(Point(p))]
        count = len(outside)
        hist[count] += 1
        closest = min(closest,min(Point(p).distance(road.boundary) for p in wheels))
        if count>=3:
            if active is None:
                active = dict(first_sample_s=stamp,last_sample_s=stamp,samples=0,max_wheels=count)
                events.append(active)
            active.update(last_sample_s=stamp,samples=active['samples']+1,max_wheels=max(active['max_wheels'],count))
        else:
            active = None
    for event in events:
        event['sampled_span_s'] = event['last_sample_s']-event['first_sample_s']
    report = dict(scope='truth_only_sampled_planar_wheel_centers_vs_road_not_tire_contact_or_referee_penalty',
        source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                       for p in (Path(__file__),world,vehicle,input_path,ROOT/'scripts/audit_local_pursuit_road.py')},
        pose_source=input_path.name,includes_start_stop=True,samples=len(poses),
        wheel_order=['front_left','front_right','rear_left','rear_right'],
        dimensions_m=dict(wheelbase=cfg['wheelbase_m'],track=cfg['track_width_m'],wheel_width=cfg['wheel_width_m']),
        max_sample_gap_s=max((b[0]-a[0] for a,b in zip(poses,poses[1:])),default=0.),
        wheel_count_histogram={str(i):hist[i] for i in range(5)},
        sampled_three_plus_events=events,minimum_center_to_road_boundary_m=closest,
        limitations=['does not inspect tire contact width or wheel lift, roll/pitch/suspension',
            'sampled poses do not prove absence of events between samples',
            'outside road may be grass or another surface; official penalty is not inferred',
            'keeps the original body-footprint audit and original run verdict unchanged'])
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(report,stream,ensure_ascii=False,indent=2)
    print(json.dumps({k:report[k] for k in ('pose_source','samples','max_sample_gap_s','wheel_count_histogram','sampled_three_plus_events')}))


if __name__ == '__main__':
    main()
