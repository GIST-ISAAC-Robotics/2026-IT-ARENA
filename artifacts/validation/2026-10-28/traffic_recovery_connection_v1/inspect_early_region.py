"""진행 중 고정된 한 호출의 이동 영역만 재구성한다. 새 주행/상태ful 재생이 아니다."""
from pathlib import Path
import json,math
from collections import Counter
import numpy as np
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,pursue_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.rear_fringe_trial import bounded_rear_fringe
from arena_autonomy.pass_release_trial import motion_region,lateral_config,lateral_excursion
BASE=Path(__file__).resolve().parent
for line in (BASE/'run1_early_hold_prefix.jsonl').open():
 row=json.loads(line)
 if row['kind']=='header':header=row['data']
 if row['kind']=='call' and abs(row['data']['input']['reference_s']-40.)<1e-7:call=row['data'];break
m=call['output'][2];cfg=FollowConfig(**header['follow_config'])
d,raw,h=restore_call(call['input'],MotionConfig(**header['motion_config']))
scene=motion_scene(raw,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
_,steer,_=pursue_path(np.asarray(m['path_points']),d['speed_mps'],cfg,m['boundary_age_s'])
steer=float(np.clip(steer,d['previous_steering_rad']-3*d['dt_s'],d['previous_steering_rad']+3*d['dt_s']))
if m.get('corridor_selected'):steer=m['corridor_candidate']['steering_rad']
context={};region=motion_region(scene,d['speed_mps'],.35,steer,d['previous_steering_rad'],d['dt_s'],cfg,lateral_config(cfg),context=context)
q=context['queries'][~context['free'] & ~context['masked']];pad=context['padding']
origins=np.asarray([r.ray_origin_xy_m for r in scene.rays]);directions=np.asarray([r.direction_xy for r in scene.rays]);ids=np.asarray([r.source_index for r in scene.rays])
counts=Counter();samples=[]
for point in q:
 delta=point-origins;angle=np.arctan2(directions[:,0]*delta[:,1]-directions[:,1]*delta[:,0],np.sum(directions*delta,axis=1))
 low=np.where(angle>=-1e-10,np.maximum(angle,0.),np.inf);high=np.where(angle<=1e-10,np.maximum(-angle,0.),np.inf)
 lo,hi=int(np.argmin(low)),int(np.argmin(high));pair=[scene.rays[lo],scene.rays[hi]]
 label=('seam' if abs(int(ids[lo])-int(ids[hi]))==len(ids)-1 else 'other',pair[0].status,pair[1].status)
 counts[str(label)]+=1
 if len(samples)<10:samples.append(dict(q=point.tolist(),indices=[int(ids[lo]),int(ids[hi])],statuses=[r.status for r in pair],angle_sum=float(low[lo]+high[hi]),padding=pad[[lo,hi]].tolist()))
r=dict(scope='one captured frame; no planner-state replay',reference_s=d['reference_s'],steering=steer,recomputed=region,recorded=m['selector_region'],counts=dict(counts),unobserved_bounds=[q.min(axis=0).tolist(),q.max(axis=0).tolist()] if len(q) else [],samples=samples,expected_excursion=m['selector_excursion_m'],excursion=lateral_excursion(max(d['speed_mps'],.35),scene.t_ref_s-scene.source_first_s,steer,d['previous_steering_rad'],d['dt_s'],cfg))
r['existing_guard_rear_fringe']=bounded_rear_fringe(scene,raw,q,pad,cfg.length_m,cfg.width_m)
(BASE/'early_region_with_fringe_analysis.json').write_text(json.dumps(freeze(r),indent=2),encoding='utf-8')
print(json.dumps(r))
