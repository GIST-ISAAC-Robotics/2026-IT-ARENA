"""위협점과 배경 앵커를 같은 원래 스캔에 묶는 미연결 비교."""
import json
from pathlib import Path
import sys
import numpy as np
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.surface_anchor_trial import SurfaceAnchorTrial

reconstruction=json.loads((BASE/'threat_reconstruction.json').read_text())
frames=json.loads((BASE/'target_frames.json').read_text())
trace=BASE.parent/'traffic_path_recovery_connection_v1/run1_course/follow_decisions.jsonl'
with trace.open() as stream:header=json.loads(next(stream))['data']
motion=MotionConfig(**header['motion_config'])
target=next(r for r in frames if abs(r['data']['input']['reference_s']-131.68)<1e-8)
evidence=target['data']['output'][2]['partial_observation']['latch_evidence']
old_call=next(r for r in frames if abs(r['data']['input']['reference_s']-evidence['previous_reference_s'])<1e-8)
data,raw,h=restore_call(old_call['data']['input'],motion)
source=raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9
points,ids,_=h.project(raw.ranges,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,
    source,raw.time_increment,data['reference_s'],mode='both',timing_verified=True)
keep=np.linalg.norm(points,axis=1)<=3.;points,ids=points[keep],ids[keep]
assert np.allclose(points,evidence['correspondence_context']['previous_points_at_reference_xy'])
unique=reconstruction['all_unique_point_indices']
probe=SurfaceAnchorTrial();probe.start(points[unique],points,ids,data['reference_s'],source)
results=[]
for row in frames:
    if row['data']['input']['reference_s']<131.68-1e-9:continue
    inp,raw,h=restore_call(row['data']['input'],motion)
    t=inp['reference_s'];s=raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9
    p,i,_=h.project(raw.ranges,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,
        s,raw.time_increment,t,mode='both',timing_verified=True)
    obs=probe.observe(p,i,h,t,s)
    results.append(dict(t_s=t,source_s=s,**obs))
    if probe.ended:break
report=dict(scope='standalone same-source anchor probe; no controller/independent guard authorization',
    old_reference_s=data['reference_s'],old_source_s=source,latch_time_s=131.68,
    retained_points=len(unique),complete_set_reobserved=any(o['matched'] for o in results),
    first_matched_s=next((o['t_s'] for o in results if o['matched']),None),observations=results)
with (BASE/'anchor_reference_comparison.json').open('x') as stream:json.dump(freeze(report),stream,indent=2)
print(json.dumps({k:v for k,v in report.items() if k!='observations'}))
print(json.dumps([dict(t_s=r['t_s'],reason=r['reason'],matched=r['matched'],observed=r.get('observed'),
    distances=r.get('nearest_distances_m')) for r in results if r['reason']!='same_scan_no_confirmation']))
