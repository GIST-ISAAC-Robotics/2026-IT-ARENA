"""미연결 진단: 정합을 하지 않은 원래 위협점의 현재 실제 표면 대응."""
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.partial_observation import short_observed_surface
from arena_autonomy.partial_surface_alignment import surface_translation

BASE=Path(__file__).resolve().parent
saved=json.loads((BASE/'target_frames.json').read_text(encoding='utf-8'))
motion=MotionConfig(**saved['header']['motion_config']);frames=saved['frames']
latched=next(r for r in frames if abs(r['data']['input']['reference_s']-129.88)<1e-8)
evidence=latched['data']['output'][2]['partial_observation']['latch_evidence']
oldrow=next(r for r in frames if abs(r['data']['input']['reference_s']-evidence['previous_reference_s'])<1e-8)
olddata,oldraw,oldh=restore_call(oldrow['data']['input'],motion)
old=np.asarray(evidence['correspondence_context']['previous_points_at_reference_xy'])
# 저장된 현재 운반 위협과 정확히 일치하는 원래 점을 찾는다. 이 실행은 위협쌍이1개다.
data,raw,h=restore_call(latched['data']['input'],motion)
carried=transport_points(old,h,olddata['reference_s'],data['reference_s'])
ret=np.asarray(evidence['witnesses'][0]['previous_point_now_xy'])
index=int(np.argmin(np.linalg.norm(carried-ret,axis=1)))
assert np.linalg.norm(carried[index]-ret)<1e-9
rows=[];last=None
for row in frames:
    data,raw,h=restore_call(row['data']['input'],motion)
    t=data['reference_s'];source=raw.header.stamp.sec+raw.header.stamp.nanosec*1e-9
    if t<129.88-1e-9 or source==last:continue
    last=source
    current,ids,_=h.project(raw.ranges,raw.angle_min,raw.angle_increment,raw.range_min,raw.range_max,
        source,raw.time_increment,t,mode='both',timing_verified=True)
    moved=transport_points(old,h,olddata['reference_s'],t);point=moved[index]
    distance=np.linalg.norm(current-point,axis=1);nearest=int(np.argmin(distance))
    short=short_observed_surface(point,current,ids)
    fit=surface_translation(moved,current,ids)
    rows.append(dict(t_s=t,source_s=source,anchor_age_s=t-olddata['reference_s'],point_xy=point.tolist(),
        nearest_distance_m=float(distance[nearest]),nearest_xy=current[nearest].tolist(),
        short_surface=short,direct_observed=bool(distance[nearest]<=.02 or distance[nearest]<=.04 and short is not None),
        fit=fit))
report=dict(scope='diagnostic only; unchanged recorded input; no controller or guard connection',
    old_reference_s=olddata['reference_s'],original_point_index=index,original_point_xy=old[index].tolist(),rows=rows)
with (BASE/'identity_comparison.json').open('x',encoding='utf-8') as f:json.dump(freeze(report),f,ensure_ascii=False,indent=2)
print(json.dumps([dict(t_s=r['t_s'],distance=r['nearest_distance_m'],short=r['short_surface'] is not None,
    direct=r['direct_observed'],fit=r['fit']['reason']) for r in rows]))
