"""r39 첫 진입의 편이/실제 곡률 조합 비교. 주행 연결 없음."""
from pathlib import Path
from dataclasses import replace
import json,math
import numpy as np
from arena_autonomy.follow_trace import restore_call,MotionConfig,FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.bypass_entry_trial import entry_path,preview_clearance,preview_point_gap,straight_reference,EntryConfig,current_outer_wall_padding
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
fixed,evidence=current_outer_wall_padding(sc,cfg,'right',wall)
fixed_aged,_=current_outer_wall_padding(sc,cfg,'right',wall,growth_s=.1)
from roomy_candidate import AdaptiveCorridorEntryConfig,compare_roomy_path
entry=AdaptiveCorridorEntryConfig(transitions_m=(.6,.7,.8,.9))
row=raw['output'][2]['bypass']['corridor_plan']['rows']['right']
result=compare_roomy_path(sc,cfg,entry,row,wall,.08)
(A/'roomy_result.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
print(json.dumps(result,indent=2))
