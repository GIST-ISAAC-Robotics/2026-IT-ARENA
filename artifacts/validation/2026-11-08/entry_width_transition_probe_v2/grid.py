"""r39 첫 진입의 편이/실제 곡률 조합 비교. 주행 연결 없음."""
from pathlib import Path
from dataclasses import replace
import json,math
import numpy as np
from arena_autonomy.follow_trace import restore_call,MotionConfig,FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.bypass_entry_trial import entry_path,preview_clearance,preview_point_gap,straight_reference,EntryConfig
from arena_autonomy.occupancy_timing import per_ray_padding
A=Path(__file__).resolve().parent
for line in (A.parent/'short_entry_transition_v1/run1/follow_decisions.jsonl').open():
 e=json.loads(line);d=e['data']
 if e['kind']=='header':head=d
 if e['kind']=='call' and abs(d['input']['reference_s']-13.02)<.001:
  raw=d;break
cfg=FollowConfig(**head['follow_config']);motion=MotionConfig(**head['motion_config'])
inp,scan,history=restore_call(raw['input'],motion)
sc=motion_scene(scan,history,inp['reference_s'],timing_verified=inp['timing_verified'],epoch=inp['epoch'],rear_half_angle_deg=inp['rear_half_angle_deg'])
ref,proof=straight_reference(sc,cfg)
wall=proof['walls']['right'];reference=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
pts=np.asarray(sc.occupied_endpoints_xy_m);pad=per_ray_padding(sc,.025+cfg.measurement_padding_m,cfg.object_motion_bound_mps)
rows=[]
for offset in (-.27,-.29,-.305,-.3091170558204061):
 for length in (.6,.65,.7,.75,.8,.9):
  entry=EntryConfig(transitions_m=(length,));path=entry_path(reference,offset,cfg,entry,length)
  d=np.gradient(path,axis=0);dd=np.gradient(d,axis=0)
  curv=np.abs(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/np.maximum(1e-8,np.linalg.norm(d,axis=1)**3)
  steer=float(np.max(np.arctan(cfg.wheelbase_m*curv)))
  rows.append(dict(offset_m=offset,length_m=length,steering_rad=steer,steerable=steer<=cfg.max_steering_rad,
   progress_m=preview_clearance(path,pts,pad,cfg),gap_m=preview_point_gap(path,pts,pad,cfg),
   aged_gap_m=preview_point_gap(path,pts,pad+.1*cfg.object_motion_bound_mps,cfg)))
out=dict(scope='single current scan, geometry only, not command permission',max_steering_rad=cfg.max_steering_rad,rows=rows)
(A/'grid_result.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf8')
print(json.dumps([r for r in rows if r['steerable']],indent=2))
