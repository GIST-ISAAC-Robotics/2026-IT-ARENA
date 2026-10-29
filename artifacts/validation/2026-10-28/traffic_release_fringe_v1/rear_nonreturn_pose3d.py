"""제동 진단의 실제 roll/pitch/z를 사용한 사후 광선 대조. 제어 입력 아님."""
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from diagnose_front_unknown import pose, wall_geometry, triangle_distance, raw_kind
from g0_geometry import ray_box_distance
BASE = Path(__file__).resolve().parent
RUN = BASE.parent/'traffic_recovery_connection_v1/run1_course'
out = BASE/'rear_nonreturn_pose3d.json'
if out.exists():
    raise FileExistsError(out)
wanted, selected = (40., 136.7, 136.72), []
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
poses = [json.loads(line) for line in (RUN/'braking_diagnostics.jsonl').open()]
times = np.asarray([r['sim_time_s'] for r in poses])
assert np.all(np.diff(times) > 0)
vehicle = ET.parse(RUN/'runtime_vehicle.sdf').find('model')
sensor, chassis = pose(vehicle.find("link[@name='lidar_link']")), pose(vehicle.find("link[@name='chassis']"))
assert all(abs(v) < 1e-10 for v in sensor[3:]+chassis[3:])
offset = np.asarray(sensor[:3])-chassis[:3]
boxes, triangles, meshes = wall_geometry(RUN/'runtime_world.sdf')
files = [trace, RUN/'braking_diagnostics.jsonl', RUN/'runtime_vehicle.sdf', RUN/'runtime_world.sdf', *meshes]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
hashes = {str(p): sha(p) for p in files}


def rotation(roll, pitch, yaw):
    c, s = np.cos([roll,pitch,yaw]), np.sin([roll,pitch,yaw])
    rx = np.array([[1.,0.,0.],[0.,c[0],-s[0]],[0.,s[0],c[0]]])
    ry = np.array([[c[1],0.,s[1]],[0.,1.,0.],[-s[1],0.,c[1]]])
    rz = np.array([[c[2],-s[2],0.],[s[2],c[2],0.],[0.,0.,1.]])
    return rz@ry@rx


results = []
for call in selected:
    data, scan, _ = restore_call(call['input'], motion)
    first = scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
    casts = []
    for index in range(449,455):
        acquired = first+index*scan.time_increment
        lo = int(np.searchsorted(times, acquired-.0021, side='right')-1)
        hi = int(np.searchsorted(times, acquired, side='left'))
        assert 0 <= lo <= hi < len(times) and times[hi]-times[lo] <= .041
        for row in (poses[lo], poses[hi]):
            xyz = np.asarray([row['truth_'+axis+'_m'] for axis in ('x','y','z')])
            angles = [row['truth_'+axis+'_rad'] for axis in ('roll','pitch','yaw')]
            r = rotation(*angles)
            origin = xyz+r@offset
            angle = scan.angle_min+index*scan.angle_increment
            direction = r@np.array([math.cos(angle),math.sin(angle),0.])
            hits = [(d,b.label) for b in boxes if (d:=ray_box_distance(origin,direction,b,0.,100.)) is not None]
            distance, label = min(hits, default=(None,None))
            floor = float(-origin[2]/direction[2]) if direction[2] < 0 else None
            casts.append(dict(index=index,acquired_s=acquired,pose_s=row['sim_time_s'],
                max_pose_window_s=float(times[hi]-times[lo]),roll_pitch_deg=np.degrees(angles[:2]).tolist(),
                raw_kind=raw_kind(scan.ranges[index],scan.range_min,scan.range_max),
                raw_m=float(scan.ranges[index]) if math.isfinite(scan.ranges[index]) else None,
                wall_collision_m=distance,wall_label=label,
                visual_front_m=triangle_distance(origin,direction,triangles,True),
                infinite_z0_plane_m=floor,origin_xyz_m=origin.tolist(),direction_xyz=direction.tolist()))
    results.append(dict(reference_s=data['reference_s'],range_max_m=scan.range_max,casts=casts))
result=dict(scope='post-run static wall and diagnostic infinite z=0 plane; recorded 3D chassis pose endpoints',
    limitations=['pose sample brackets up to 41 ms, not exact acquisition pose or continuous bound',
                 'moving actors and non-wall objects excluded; z=0 plane is diagnostic, not mesh raycast',
                 'no GPU replay, reflection or renderer verification'],
    input_sha256=hashes,input_unchanged=all(sha(Path(p))==h for p,h in hashes.items()),
    controller_received_truth=False,runtime_changed=False,examples=results)
out.write_text(json.dumps(result,indent=2,allow_nan=False),encoding='utf-8')
print(json.dumps(dict(example_40=[{k:v[k] for k in ('index','raw_kind','raw_m','wall_collision_m','infinite_z0_plane_m','roll_pitch_deg')}
                                 for v in results[0]['casts'][::2]],input_unchanged=result['input_unchanged'])))
