"""기존 기록의 방지턱 상대 바퀴 위치와 운동 차이. 평가 전용, 접촉력 증명 아님."""
import argparse
from bisect import bisect_left
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

BASE = Path(__file__).resolve().parent
CASES = ('g1_forward_same_path_gazebo_v1', 'g2_motion_pause_gazebo_v1', 'g2_motion_pause_level_v1')


def sha(p):
    with p.open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()


def rotation(q):
    x, y, z, w = [q[k] for k in ('x', 'y', 'z', 'w')]
    if not all(math.isfinite(v) for v in (x, y, z, w)) or abs(x*x+y*y+z*z+w*w-1) > 1e-6:
        raise ValueError('invalid quaternion')
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def transform(r, v):
    return [sum(row[i]*v[i] for i in range(3)) for row in r]


def nearest(rows, times, t):
    i = bisect_left(times, t)
    j = min(range(max(0, i-1), min(len(times), i+1)), key=lambda j: abs(times[j]-t))
    if abs(times[j]-t) > .004:
        raise ValueError('pose/diagnostic sample not time aligned')
    return rows[j]


def wheel_state(pose, diagnostic, offsets, bump, radius, width):
    r = rotation(pose['orientation']); p = pose['position']
    c, s = math.cos(bump[5]), math.sin(bump[5])
    result = {}
    for name, offset in offsets.items():
        center = [p[k]+v for k,v in zip(('x','y','z'), transform(r, offset))]
        steer = diagnostic[('left' if 'left' in name else 'right')+'_steering_rad'] if name.startswith('front') else 0.
        # Cylinder axis after local -pi/2 roll is +Y; steer rotates it about local Z.
        axis = transform(r, (-math.sin(steer), math.cos(steer), 0.))
        local_axis = (c*axis[0]+s*axis[1], -s*axis[0]+c*axis[1], axis[2])
        extents = [radius*math.sqrt(max(0.,1-a*a)) + width/2*abs(a) for a in local_axis]
        dx,dy = center[0]-bump[0],center[1]-bump[1]
        along,lateral = c*dx+s*dy,-s*dx+c*dy
        result[name] = dict(along_m=along,lateral_m=lateral,center_z_m=center[2],
            bottom_z_m=center[2]-extents[2],
            bump_xy_projection_overlap=abs(along)<=.025+extents[0] and abs(lateral)<=.225+extents[1])
    return result


def analyze(case_name):
    run = BASE.parent/case_name/'run1'
    paths = [run/n for n in ('bump_pose_trace.json','braking_diagnostics.jsonl','runtime_vehicle.sdf','runtime_world.sdf')]
    hashes={p.name:sha(p) for p in paths}
    poses=json.loads(paths[0].read_text()); times=[p['time'] for p in poses]
    assert all(b>a for a,b in zip(times,times[1:])), 'unordered poses'
    rows=[json.loads(line) for line in paths[1].open()]
    v=ET.parse(paths[2]).getroot(); world=ET.parse(paths[3]).getroot()
    b=world.findall(".//model/link[@name='safety_bump_0']")
    assert len(b)==1
    bump=list(map(float,b[0].findtext('pose').split()))
    assert bump[3:5]==[0.,0.]
    offsets={}; radius=width=None
    for name in ('front_left','front_right','rear_left','rear_right'):
        link=v.find(f".//link[@name='{name}_wheel']")
        val=list(map(float,link.findtext('pose').split())); offsets[name]=val[:3]
        assert abs(val[3]+math.pi/2)<1e-12 and val[4:]==[0.,0.]
        cylinder=link.find('collision/geometry/cylinder')
        rr,ww=float(cylinder.findtext('radius')),float(cylinder.findtext('length'))
        if radius is not None: assert (rr,ww)==(radius,width)
        radius,width=rr,ww
    selected=[row for row in rows if 21<=row['sim_time_s']<=45]
    sampled=[]; overlap_first={name:None for name in offsets}; sustained=[]
    for row in selected:
        t=row['sim_time_s']; pose=nearest(poses,times,t)
        wheels=wheel_state(pose,row,offsets,bump,radius,width)
        for wheel,info in wheels.items():
            if info['bump_xy_projection_overlap'] and overlap_first[wheel] is None: overlap_first[wheel]=t
        if abs(row['wheel_surface_speed_mps']-row['truth_longitudinal_mps'])>.2:
            sustained.append(t)
    for t in (22.,23.,23.323,24.,24.6,25.,26.,28.,29.,30.,31.,40.,41.625):
        row=min(selected,key=lambda r:abs(r['sim_time_s']-t));pose=nearest(poses,times,row['sim_time_s'])
        keys=('sim_time_s','applied_target_mps','left_speed_rad_s','right_speed_rad_s','wheel_surface_speed_mps',
              'truth_longitudinal_mps','truth_roll_rad','truth_pitch_rad','truth_yaw_rad','left_steering_rad','right_steering_rad')
        sampled.append(dict(**{k:row[k] for k in keys},model_origin_z_m=pose['position']['z'],
                            wheels=wheel_state(pose,row,offsets,bump,radius,width)))
    intervals=[]
    for lo,hi in ((21.,23.),(23.,25.),(25.,30.),(30.,40.)):
        ss=[r for r in selected if lo<=r['sim_time_s']<=hi]
        def integral(key):
            return sum(.5*(a[key]+b[key])*(b['sim_time_s']-a['sim_time_s']) for a,b in zip(ss,ss[1:]))
        intervals.append(dict(window_s=[lo,hi],wheel_integral_m=integral('wheel_surface_speed_mps'),
            body_longitudinal_integral_m=integral('truth_longitudinal_mps'),
            max_sample_gap_s=max(b['sim_time_s']-a['sim_time_s'] for a,b in zip(ss,ss[1:]))))
    return dict(run=case_name,source_sha256=hashes,input_unchanged=all(sha(p)==hashes[p.name] for p in paths),
        bump_pose=bump,wheel_radius_m=radius,wheel_width_m=width,projection_first_s=overlap_first,
        first_large_wheel_body_disagreement_s=min(sustained) if sustained else None,
        intervals=intervals,samples=sampled,
        scope='rigid wheel geometry projection and measured simulation diagnostics; no measured normal loads or hardware claim')


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=dict(cases=[analyze(n) for n in CASES],script_sha256=sha(Path(__file__)))
    with args.output.open('x') as f:json.dump(result,f,indent=2,allow_nan=False)
    print(json.dumps([dict(case=c['run'],unchanged=c['input_unchanged'],projection=c['projection_first_s'],
                          disagreement=c['first_large_wheel_body_disagreement_s'],intervals=c['intervals']) for c in result['cases']]))
