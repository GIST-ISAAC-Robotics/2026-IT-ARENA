"""고정 입력의 잘린 위협 예시를 원시 광선/시각에서 재구성한다. 제어 변경 없음."""
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import relative_poses
from arena_autonomy.recovery_geometry import timed_missing_sweep
from arena_autonomy.partial_observation import short_observed_surface
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial

trace=BASE.parent/'traffic_path_recovery_connection_v1/run1_course/follow_decisions.jsonl'
with trace.open('rb') as stream:before=hashlib.file_digest(stream,'sha256').hexdigest()
frames=[];previous=None
with trace.open() as stream:
    header=json.loads(next(stream))['data']
    for line in stream:
        row=json.loads(line)
        if row['kind']!='call':continue
        ref=row['data']['input']['reference_s']
        if 131.0<=ref<=133.0:frames.append(row)
target=next(r for r in frames if abs(r['data']['input']['reference_s']-131.68)<1e-8)
partial=target['data']['output'][2]['partial_observation'];evidence=partial['latch_evidence']
old_call=next(r for r in frames if abs(r['data']['input']['reference_s']-evidence['previous_reference_s'])<1e-8)
motion=MotionConfig(**header['motion_config'])
data,raw,history=restore_call(target['data']['input'],motion)
old_data,old_raw,old_history=restore_call(old_call['data']['input'],motion)
first=raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9;now=data['reference_s']
old_first=old_raw.header.stamp.sec+old_raw.header.stamp.nanosec*1e-9
old_points,old_ids,_=old_history.project(old_raw.ranges,old_raw.angle_min,old_raw.angle_increment,
    old_raw.range_min,old_raw.range_max,old_first,old_raw.time_increment,old_data['reference_s'],mode='both',timing_verified=True)
keep=np.linalg.norm(old_points,axis=1)<=3.
context=evidence['correspondence_context']
assert np.allclose(old_points[keep],context['previous_points_at_reference_xy'],rtol=0,atol=1e-8)
times=old_first+old_ids[keep]*old_raw.time_increment
carried=np.asarray(context['previous_points_now_xy'])
current,ids,_=history.project(raw.ranges,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,
    first,raw.time_increment,now,mode='both',timing_verified=True)
assert np.allclose(current,context['current_return_points_xy'],rtol=0,atol=1e-8)
values=np.asarray(raw.ranges);step=raw.angle_increment
angles=(raw.angle_min+np.arange(len(values))*step+motion.lidar_yaw_rad+math.pi)%math.tau-math.pi
valid=np.isfinite(values)&(values>=raw.range_min)&(values<=raw.range_max)
unknown=np.flatnonzero((~valid)&(np.abs(angles)<=math.radians(100)+1e-6))
unknown=unknown[np.argsort(angles[unknown])]
stamps=first+unknown*raw.time_increment
poses=relative_poses(history,stamps,now);directions=angles[unknown]+poses[:,2]
c,s=np.cos(poses[:,2]),np.sin(poses[:,2])
origins=poses[:,:2]+np.column_stack((c*motion.lidar_x_m-s*motion.lidar_y_m,s*motion.lidar_x_m+c*motion.lidar_y_m))
timed=partial['timed_missing_diagnostic']
result=timed_missing_sweep(carried,times,origins,directions,stamps,now,step/2,
    timed['required_m'],timed['checked_speed_mps'],timed['checked_age_s'],
    timed['checked_steering_rad'],timed['checked_previous_steering_rad'],
    wheelbase=timed['wheelbase_m'],length=timed['length_m'],width=timed['width_m'])
threat=np.asarray(result['lower_distance_m'])<=np.asarray(result['before_radius_m'])+1e-10
initial=int(threat.sum());reobserved=[]
for index in np.flatnonzero(np.any(threat,axis=0)):
    distance=float(np.min(np.linalg.norm(current-carried[index],axis=1)))
    if distance<=.02 or (distance<=.04 and short_observed_surface(carried[index],current,ids) is not None):
        threat[:,index]=False;reobserved.append(int(index))
fit=partial.get('surface_correspondence') or {}
if fit.get('accepted'):
    shift=np.asarray(fit['translation_at_reference_m'])
    for index in np.flatnonzero(np.any(threat,axis=0)):
        point=carried[index]+shift
        distance=float(np.min(np.linalg.norm(current-point,axis=1)))
        if distance<=.02 or (distance<=.04 and short_observed_surface(point,current,ids) is not None):
            threat[:,index]=False
pairs=np.argwhere(threat);unique=np.flatnonzero(np.any(threat,axis=0))
assert len(pairs)==evidence['pairs']
assert all(int(unknown[ray])==w['source_index'] and np.allclose(carried[index],w['previous_point_now_xy'])
    for (ray,index),w in zip(pairs[:8],evidence['witnesses']))
probe=SurfaceAnchorTrial();probe.start(carried[unique],current,ids,now,first)
observations=[]
for row in frames:
    if row['data']['input']['reference_s']<=now:continue
    inp,scan,h=restore_call(row['data']['input'],motion)
    t=inp['reference_s'];source=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9
    points,indices,_=h.project(scan.ranges,scan.angle_min,scan.angle_increment,scan.range_min,scan.range_max,
        source,scan.time_increment,t,mode='both',timing_verified=True)
    obs=probe.observe(points,indices,h,t,source)
    observations.append(dict(t_s=t,source_s=source,**obs))
    if probe.ended:break
summary=dict(scope='sensor-only reconstruction and standalone reobservation probe; no drive authorization',
    input_sha256=before,latch_time_s=now,pairs_before_reobservation=initial,
    stored_pairs=evidence['pairs'],stored_examples=len(evidence['witnesses']),
    reconstructed_pairs=len(pairs),unique_points=len(unique),all_unique_point_indices=unique.tolist(),
    all_unique_points=carried[unique].tolist(),missing_indices=unknown.tolist(),
    original_anchor_reason=partial.get('anchor_diagnostic',{}).get('reason'),
    original_surface_fit=fit,observations=observations,
    complete_set_reobserved=any(o.get('matched') for o in observations))
with trace.open('rb') as stream:summary['input_unchanged']=hashlib.file_digest(stream,'sha256').hexdigest()==before
with (BASE/'threat_reconstruction.json').open('x',encoding='utf-8') as stream:json.dump(freeze(summary),stream,indent=2)
with (BASE/'target_frames.json').open('x',encoding='utf-8') as stream:json.dump(frames,stream)
print(json.dumps({k:v for k,v in summary.items() if k not in ('observations','original_surface_fit')},default=int))
print(json.dumps([dict(t_s=o['t_s'],reason=o['reason'],matched=o['matched'],observed=o.get('observed')) for o in observations]))
