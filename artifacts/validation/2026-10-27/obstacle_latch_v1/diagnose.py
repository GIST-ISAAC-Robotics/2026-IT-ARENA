"""최초 소실 래치의 센서점과 평가 정답을 대조한다. 제어/보호 입력은 변경하지 않는다."""
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from shapely.geometry import Point,Polygon

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
for folder in ('scripts','src/arena_autonomy','src/arena_vehicle_interface'):
    sys.path.insert(0,str(ROOT/folder))
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene,transport_points
from diagnose_front_unknown import wall_geometry
from g0_geometry import Footprint

RUN=BASE.parent/'short_prefix_connection_v1/run1_course'
OUT=BASE/'diagnosis_v1.json'
if OUT.exists():raise FileExistsError(OUT)

def interpolated(rows,t,maximum_gap):
    i=int(np.searchsorted(rows[:,0],t))
    if i==0 or i==len(rows) or rows[i,0]-rows[i-1,0]>maximum_gap:raise ValueError('pose interpolation unavailable')
    a,b=rows[i-1],rows[i];f=(t-a[0])/(b[0]-a[0])
    yaw=a[3]+f*((b[3]-a[3]+math.pi)%math.tau-math.pi)
    return np.r_[a[1:3]+f*(b[1:3]-a[1:3]),yaw]

def world(p,pose):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return np.asarray(p)@np.array([[c,s],[-s,c]])+pose[:2]

def local(p,pose):
    c,s=math.cos(pose[2]),math.sin(pose[2])
    return (np.asarray(p)-pose[:2])@np.array([[c,-s],[s,c]])

calls=[];first=None
with (RUN/'follow_decisions.jsonl').open() as stream:
    header=json.loads(next(stream))['data'];motion=MotionConfig(**header['motion_config'])
    for line in stream:
        row=json.loads(line)
        if row['kind']!='call':continue
        data=row['data'];t=data['input']['reference_s']
        if t<128.9:continue
        if t>130.5:break
        calls.append(data)
        p=data['output'][2].get('partial_observation') or {}
        if first is None and p.get('latch_evidence'):first=p
assert first is not None
e=first['latch_evidence'];ctx=e['correspondence_context']
previous=min(calls,key=lambda q:abs(q['input']['reference_s']-e['previous_reference_s']))
assert abs(previous['input']['reference_s']-e['previous_reference_s'])<1e-8
d,scan,h=restore_call(previous['input'],motion)
scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
rays=[r for r in scene.rays if r.status=='return' and np.linalg.norm(r.hit_xy_m)<=3.]
old=np.asarray(ctx['previous_points_at_reference_xy']);carried=np.asarray(ctx['previous_points_now_xy'])
assert np.allclose(old,np.asarray([r.hit_xy_m for r in rays]),rtol=0,atol=1e-9)
poses=np.asarray([json.loads(line) for line in (RUN/'ego_poses.jsonl').open()])
actors=np.asarray([json.loads(line) for line in (RUN/'actor_poses.jsonl').open()])
boxes,triangles,meshes=wall_geometry(RUN/'runtime_world.sdf')
walls=[Polygon(b.footprint.polygon(b.x_m,b.y_m,b.yaw_rad)) for b in boxes]
ego_now=interpolated(poses,e['reference_s'],.01)
ego_old=interpolated(poses,e['previous_reference_s'],.01)
truth_se2=np.r_[local(ego_old[:2],ego_now),(ego_old[2]-ego_now[2]+math.pi)%math.tau-math.pi]
rows=[]
for w in e['witnesses']:
    idx=int(np.argmin(np.linalg.norm(carried-w['previous_point_now_xy'],axis=1)))
    assert np.linalg.norm(carried[idx]-w['previous_point_now_xy'])<1e-9
    ray=rays[idx];n=ray.source_index;stamp=ray.acquired_at_s
    pose=interpolated(poses,stamp,.01);actor=interpolated(actors,stamp,.05)
    angle=scan.angle_min+n*scan.angle_increment+motion.lidar_yaw_rad
    acquired=[motion.lidar_x_m+scan.ranges[n]*math.cos(angle),motion.lidar_y_m+scan.ranges[n]*math.sin(angle)]
    point=world(acquired,pose);shape=Polygon(Footprint().polygon(*actor))
    rows.append(dict(**w,previous_source_index=n,previous_acquired_s=stamp,
        raw_hit_world_xy=point.tolist(),actor_pose_at_acquisition=actor.tolist(),
        actor_boundary_distance_m=shape.boundary.distance(Point(point)),
        wall_collision_distance_m=min(p.distance(Point(point)) for p in walls)))
retained=np.asarray([w['previous_point_now_xy'] for w in e['witnesses']])
reobservations=[]
for data in calls:
    t=data['input']['reference_s']
    if t<e['reference_s'] or abs((t-e['reference_s'])/.1-round((t-e['reference_s'])/.1))>1e-5:continue
    d,scan,h=restore_call(data['input'],motion)
    sc=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    p=transport_points(retained,h,e['reference_s'],t)
    current=np.asarray(sc.occupied_endpoints_xy_m)
    dist=np.min(np.linalg.norm(p[:,None]-current[None],axis=2),axis=1)
    reobservations.append(dict(t_s=t,nearest_current_distances_m=dist.tolist(),
        within_2cm=int(np.count_nonzero(dist<=.02)),raw_reason=(data['output'][2].get('partial_observation') or {}).get('reason')))
trace=json.loads((RUN/'trace.json').read_text())
safety_first=next(q for q in trace if (q.get('partial_observation_trial',{}).get('safety') or {}).get('latch_evidence'))
safety=safety_first['partial_observation_trial']['safety']
inputs=[Path(__file__),RUN/'follow_decisions.jsonl',RUN/'trace.json',RUN/'ego_poses.jsonl',RUN/'actor_poses.jsonl',RUN/'runtime_world.sdf',
        ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py',ROOT/'src/arena_autonomy/arena_autonomy/lidar_safety.py']
report=dict(scope='post-run sensor/ground-truth attribution only; no truth enters controller; interpolation is diagnostic',
    first_controller={k:v for k,v in first.items() if k!='latch_evidence'},witnesses=rows,
    estimated_ego_se2=ctx['estimated_reference_to_now_se2'],truth_ego_se2=truth_se2.tolist(),
    ego_se2_error=(np.asarray(ctx['estimated_reference_to_now_se2'])-truth_se2).tolist(),
    first_safety={k:v for k,v in safety.items() if k!='latch_evidence'},
    first_safety_witnesses={k:v for k,v in safety['latch_evidence'].items() if k!='correspondence_context'},
    retained_point_reobservations=reobservations,
    source_sha256={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})
OUT.write_text(json.dumps(freeze(report),ensure_ascii=False,indent=2),encoding='utf-8')
fig,ax=plt.subplots(figsize=(8,7),layout='constrained')
current=np.asarray(ctx['current_return_points_xy'])
ax.scatter(carried[:,0],carried[:,1],s=6,c='#b9a0d6',label='Previous returns, ego-motion compensated')
ax.scatter(current[:,0],current[:,1],s=7,c='#333333',label='Current returns')
ax.scatter(retained[:,0],retained[:,1],s=65,c='#d73027',marker='x',label='Points causing latch')
w=e['witnesses'][0];origin=np.asarray(w['origin_xy']);v=np.array([math.cos(w['direction_rad']),math.sin(w['direction_rad'])])
ax.plot(*np.array([origin,origin+v*1.2]).T,c='#e69f00',lw=2,label='Missing ray (no synthetic range)')
actor=interpolated(actors,e['reference_s'],.05);outline=local(Footprint().polygon(*actor),ego_now)
outline=np.vstack((outline,outline[0]));ax.plot(outline[:,0],outline[:,1],c='#009e73',ls='--',label='Lead box at reference time (evaluation only)')
ego=np.array([[-.1,-.075],[.1,-.075],[.1,.075],[-.1,.075],[-.1,-.075]])
ax.plot(ego[:,0],ego[:,1],c='#0072b2',label='Ego outline')
ax.set(xlim=(-.2,1.),ylim=(-.6,1.2),aspect='equal',xlabel='Forward (m)',ylabel='Left (m)',title=f"First latch at t={e['reference_s']:.3f} s")
ax.grid(alpha=.25);ax.legend(fontsize=8);fig.savefig(BASE/'first_latch.png',dpi=160)
print(json.dumps(freeze({k:report[k] for k in ('witnesses','ego_se2_error','retained_point_reobservations')}),ensure_ascii=False))
