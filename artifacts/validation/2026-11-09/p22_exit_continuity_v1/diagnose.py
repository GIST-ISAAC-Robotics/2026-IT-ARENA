import json,math
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene,transport_points
from arena_autonomy.p22_observation import segments
from extend_path import retained_poses

A=Path(__file__).resolve().parent;source=A.parent/'p22_route_connected_v1/run1/follow_decisions.jsonl'
prev=None;rows=[]
for l in source.open():
 e=json.loads(l);d=e['data']
 if e['kind']=='header':mc=MotionConfig(**d['motion_config'])
 if e['kind']!='call':continue
 raw,scan,h=restore_call(d['input'],mc);t=raw['reference_s']
 q=d['output'][2].get('p22_trial',{})
 if q.get('phase')=='exit' and prev is not None:
  path=transport_points(prev[1],h,prev[0],t);p=retained_poses(path)[-1]
  scene=motion_scene(scan,h,t,timing_verified=raw['timing_verified'],epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
  segs=segments(scene)
  print('time',t,'tail',path[-1].tolist(),'tail_heading',math.degrees(p.yaw),'steer',p.steer)
  for s in segs:
   if abs(s['heading_deg'])<15 and abs(s['m'][1])<.5:
    print('line',s['m'].tolist(),'heading',s['heading_deg'],'span',s['span'],'tail_line_offset',float((path[-1]-s['m'])@s['n']))
  break
 if q.get('phase')=='entry':prev=(t,np.asarray(q['path_points']))
