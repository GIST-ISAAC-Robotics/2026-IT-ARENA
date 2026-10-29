"""시각 보정 사본에 남은 두 점의 평가용 귀속. 운영 판단 입력은 바꾸지 않는다."""
import hashlib
import json
import math
from pathlib import Path
import sys
import zipfile
import numpy as np
from shapely.geometry import Point, Polygon

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from diagnose_front_unknown import wall_geometry
from g0_geometry import Footprint
from helpers import interpolated,world,local
from probe_timed_missing_sweep import timed_missing_sweep


def main():
    output=BASE/'report.json'
    if output.exists():raise FileExistsError(output)
    run=ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course'
    diagnostic=BASE.parent/'timed_latch_v1/policy_replay_v1/report.json'
    partial=json.loads(diagnostic.read_text())['first_latch'];e=partial['latch_evidence']
    ctx=e['correspondence_context'];calls={}
    with (run/'follow_decisions.jsonl').open() as stream:
        header=json.loads(next(stream))['data'];motion=MotionConfig(**header['motion_config'])
        for line in stream:
            row=json.loads(line)
            if row['kind']!='call':continue
            d=row['data'];t=d['input']['reference_s']
            if t>e['reference_s']+1e-8:break
            if abs(t-e['previous_reference_s'])<1e-8:calls['previous']=d
            if abs(t-e['reference_s'])<1e-8:calls['current']=d
    scenes={};scans={}
    for label,data in calls.items():
        d,scan,h=restore_call(data['input'],motion);scans[label]=scan
        sc=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],
            epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        scenes[label]=[r for r in sc.rays if r.status=='return' and np.linalg.norm(r.hit_xy_m)<=3.]
    old=np.asarray(ctx['previous_points_at_reference_xy']);carried=np.asarray(ctx['previous_points_now_xy'])
    assert np.allclose(old,[r.hit_xy_m for r in scenes['previous']],rtol=0,atol=1e-9)
    poses=np.asarray([json.loads(l) for l in (run/'ego_poses.jsonl').open()])
    actors=np.asarray([json.loads(l) for l in (run/'actor_poses.jsonl').open()])
    boxes,_,_=wall_geometry(run/'runtime_world.sdf')
    walls=[Polygon(b.footprint.polygon(b.x_m,b.y_m,b.yaw_rad)) for b in boxes]
    result=[]
    for witness in e['witnesses']:
        idx=int(np.argmin(np.linalg.norm(carried-witness['previous_point_now_xy'],axis=1)))
        assert np.linalg.norm(carried[idx]-witness['previous_point_now_xy'])<1e-9
        ray=scenes['previous'][idx];scan=scans['previous']
        angle=scan.angle_min+ray.source_index*scan.angle_increment+motion.lidar_yaw_rad
        acquired=[motion.lidar_x_m+scan.ranges[ray.source_index]*math.cos(angle),
                  motion.lidar_y_m+scan.ranges[ray.source_index]*math.sin(angle)]
        p=world(acquired,interpolated(poses,ray.acquired_at_s,.01))
        actor=interpolated(actors,ray.acquired_at_s,.05)
        wall_dist=[shape.distance(Point(p)) for shape in walls]
        current_rays=scenes['current'];nearest=min(current_rays,key=lambda r:np.linalg.norm(
            np.asarray(r.hit_xy_m)-witness['nearest_current_xy']))
        assert np.linalg.norm(np.asarray(nearest.hit_xy_m)-witness['nearest_current_xy'])<1e-8
        current_scan=scans['current'];missing_t=e['source_first_s']+witness['source_index']*current_scan.time_increment
        geometry=timed_missing_sweep([witness['previous_point_now_xy']],[ray.acquired_at_s],
            [witness['origin_xy']],[witness['direction_rad']],[missing_t],e['reference_s'],
            half_angle=current_scan.angle_increment/2,required=partial['history_distance_m'],
            speed=e['sweep_speed_mps'],age=e['reference_s']-e['source_first_s'],
            steering=e['steering_rad'],previous_steering=e['previous_steering_rad'])
        result.append(dict(**witness,previous_source_index=ray.source_index,previous_acquired_s=ray.acquired_at_s,
            raw_hit_world_xy=p.tolist(),actor_boundary_distance_m=Polygon(Footprint().polygon(*actor)).boundary.distance(Point(p)),
            wall_distance_m=min(wall_dist),nearest_current_ray_index=nearest.source_index,
            nearest_current_acquired_s=nearest.acquired_at_s,timed_geometry=geometry))
    paths=[Path(__file__),BASE/'helpers.py',diagnostic,run/'follow_decisions.jsonl',run/'ego_poses.jsonl',
           run/'actor_poses.jsonl',run/'runtime_world.sdf',ROOT/'scripts/probe_timed_missing_sweep.py',
           ROOT/'scripts/probe_joint_missing_sweep.py',ROOT/'scripts/diagnose_front_unknown.py',
           *sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    with zipfile.ZipFile(BASE/'source.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in paths:
            if p.suffix=='.py':z.write(p,p.relative_to(ROOT).as_posix())
    report=dict(scope='post-run truth attribution only; no operational adoption or driving',
        reference_s=e['reference_s'],witnesses=result,source_sha256=hashes,
        source_unchanged=all(hashlib.sha256((ROOT/r).read_bytes()).hexdigest()==h for r,h in hashes.items()))
    output.write_text(json.dumps(freeze(report),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(freeze({k:v for k,v in report.items() if k!='source_sha256'})))


if __name__=='__main__':main()
