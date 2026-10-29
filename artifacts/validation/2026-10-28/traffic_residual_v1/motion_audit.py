"""모터축/gyro 이동과 평가 정답의 차이를 분리한다. 제어기에 정답을 넣지 않는다."""
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene,transport_points
from helpers import interpolated,world,local

def main():
    output=BASE/'motion_audit.json'
    if output.exists():raise FileExistsError(output)
    report_path=BASE/'report.json';first=json.loads(report_path.read_text())
    old=np.asarray([w['previous_point_now_xy'] for w in first['witnesses']])
    old_world=np.asarray([w['raw_hit_world_xy'] for w in first['witnesses']])
    run=ROOT/'artifacts/validation/2026-10-27/short_prefix_connection_v1/run1_course'
    poses=np.asarray([json.loads(l) for l in (run/'ego_poses.jsonl').open()])
    trace=run/'follow_decisions.jsonl';rows=[];last_source=None
    with trace.open() as stream:
        motion=MotionConfig(**json.loads(next(stream))['data']['motion_config'])
        for line in stream:
            row=json.loads(line)
            if row['kind']!='call':continue
            inp=row['data']['input'];now=inp['reference_s']
            if now<first['reference_s']:continue
            if now>first['reference_s']+1.:break
            d,scan,h=restore_call(inp,motion)
            source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
            if source==last_source:continue
            last_source=source
            scene=motion_scene(scan,h,now,timing_verified=d['timing_verified'],epoch=d['epoch'],
                rear_half_angle_deg=d['rear_half_angle_deg'])
            rays=[r for r in scene.rays if r.status=='return']
            ego=interpolated(poses,now,.01);carried=transport_points(old,h,first['reference_s'],now)
            truth_local=local(old_world,ego);raw_world=[]
            for ray in rays:
                angle=scan.angle_min+ray.source_index*scan.angle_increment+motion.lidar_yaw_rad
                p=[motion.lidar_x_m+scan.ranges[ray.source_index]*math.cos(angle),
                   motion.lidar_y_m+scan.ranges[ray.source_index]*math.sin(angle)]
                raw_world.append(world(p,interpolated(poses,ray.acquired_at_s,.01)))
            current=np.asarray([r.hit_xy_m for r in rays]);raw_world=np.asarray(raw_world)
            nearest=np.argmin(np.linalg.norm(old_world[:,None]-raw_world[None],axis=2),axis=1)
            rows.append(dict(reference_s=now,source_s=source,unknown_rays=sum(r.status=='unknown' for r in scene.rays),
                transported_point_error_m=np.linalg.norm(carried-truth_local,axis=1).tolist(),
                transported_point_error_xy=(carried-truth_local).tolist(),
                pipeline_nearest_m=np.min(np.linalg.norm(carried[:,None]-current[None],axis=2),axis=1).tolist(),
                truth_projected_nearest_m=np.linalg.norm(old_world-raw_world[nearest],axis=1).tolist(),
                truth_nearest_ray_ids=[rays[i].source_index for i in nearest]))
    sources=[Path(__file__),BASE/'helpers.py',report_path,trace,run/'ego_poses.jsonl']
    result=dict(scope='post-run evaluation of sensor projection and odometry; truth is not a controller input',
        samples=rows,source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in sources})
    output.write_text(json.dumps(freeze(result),indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='source_sha256'}))

if __name__=='__main__':main()
