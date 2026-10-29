"""직전 정체의 후방 광선과 정적 코스 기하 대조. 정답은 사후 진단에만 사용한다."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from diagnose_front_unknown import pose, pose_window, wall_geometry, triangle_distance, raw_kind
from g0_geometry import ray_box_distance

BASE = Path(__file__).resolve().parent
RUN = BASE.parent/'traffic_recovery_connection_v1/run1_course'
out = BASE/'rear_nonreturn_geometry.json'
if out.exists():
    raise FileExistsError(out)
wanted = (40., 136.7, 136.72)
selected = []
trace = RUN/'follow_decisions.jsonl'
for line in trace.open(encoding='utf-8'):
    row = json.loads(line)
    if row['kind'] == 'header':
        motion = MotionConfig(**row['data']['motion_config'])
    if row['kind'] == 'call' and any(abs(row['data']['input']['reference_s']-t) < 1e-7 for t in wanted):
        selected.append(row['data'])
        if len(selected) == len(wanted):
            break
assert len(selected) == len(wanted)
poses = np.asarray([json.loads(line) for line in (RUN/'ego_poses.jsonl').open()])
assert poses.ndim == 2 and poses.shape[1] == 4 and np.all(np.diff(poses[:, 0]) > 0)
vehicle = ET.parse(RUN/'runtime_vehicle.sdf').find('model')
sensor = pose(vehicle.find("link[@name='lidar_link']"))
chassis = pose(vehicle.find("link[@name='chassis']"))
report = json.loads((RUN/'report.json').read_text())
height = pose(vehicle)[2]+sensor[2]
boxes, triangles, meshes = wall_geometry(RUN/'runtime_world.sdf')
inputs = [trace, RUN/'ego_poses.jsonl', RUN/'runtime_vehicle.sdf', RUN/'runtime_world.sdf', RUN/'report.json', *meshes]
sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
hashes = {str(p): sha(p) for p in inputs}
results = []
for call in selected:
    data, scan, _ = restore_call(call['input'], motion)
    first = scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
    rays = []
    for index in range(449, 455):
        acquired = first+index*scan.time_increment
        for sample in pose_window(poses, acquired):
            _, x, y, yaw = sample
            c, s = math.cos(yaw), math.sin(yaw)
            origin = (x+c*sensor[0]-s*sensor[1], y+s*sensor[0]+c*sensor[1], height)
            angle = yaw+sensor[5]+scan.angle_min+index*scan.angle_increment
            direction = (math.cos(angle), math.sin(angle), 0.)
            hits = [(d, b.label) for b in boxes if (d := ray_box_distance(origin, direction, b, 0., 100.)) is not None]
            distance, label = min(hits, default=(None, None))
            rays.append(dict(index=index, acquired_s=acquired, pose_sample_s=float(sample[0]),
                raw_kind=raw_kind(scan.ranges[index], scan.range_min, scan.range_max),
                raw_m=float(scan.ranges[index]) if math.isfinite(scan.ranges[index]) else None,
                collision_m=distance, collision_label=label,
                visual_two_sided_m=triangle_distance(origin, direction, triangles),
                visual_front_m=triangle_distance(origin, direction, triangles, True),
                origin_xyz_m=origin, angle_rad=angle))
    results.append(dict(reference_s=data['reference_s'], range_max_m=scan.range_max, rays=rays))
missing = [r for row in results for r in row['rays'] if r['raw_kind'] == 'positive_infinity']
result = dict(scope='post-run static-wall planar intersections at acquisition-window endpoints; not renderer/actor replay',
    assumptions=['nominal SDF model z plus lidar offset; no measured heave/pitch/roll at acquisition',
                 'moving vehicles and all non-wall objects excluded; finite shorter returns may hit them',
                 'no interpolation or continuous-time guarantee'],
    sensor_height_m=height, examples=results, input_sha256=hashes,
    input_unchanged=all(sha(Path(p)) == h for p, h in hashes.items()),
    missing_pose_samples=len(missing),
    missing_beyond_range=sum(r['collision_m'] is not None and r['collision_m'] > results[0]['range_max_m'] for r in missing),
    controller_received_truth=False, runtime_changed=False)
out.write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
print(json.dumps(dict(missing_pose_samples=len(missing), missing_beyond_range=result['missing_beyond_range'],
    example_40=[{k:r[k] for k in ('index','raw_kind','raw_m','collision_m','visual_front_m')}
                for r in results[0]['rays'][::2]], input_unchanged=result['input_unchanged'])))
