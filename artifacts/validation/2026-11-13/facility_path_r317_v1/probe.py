from pathlib import Path
import hashlib, json, sys
import numpy as np
A=Path(__file__).resolve().parent; R=A.parents[3]
sys.path[:0]=[str(A),str(R/'src/arena_autonomy'),str(R/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
import geometry
p=A.parent/'facility_arc_drive_r314_v1/run1/follow_decisions.jsonl'
with p.open() as f:
    head=json.loads(next(f))['data']; calls=[e['data'] for line in f if (e:=json.loads(line))['kind']=='call']
cfg=FollowConfig(**head['follow_config']); mc=MotionConfig(**head['motion_config'])
rows=[]
for c in calls:
    if not 20.8 <= c['input']['reference_s'] <= 23.0: continue
    d,scan,h=restore_call(c['input'],mc)
    scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    out=c['output'][2]; path=out.get('path_points')
    row=dict(time=d['reference_s'],speed=d['speed_mps'],reason=out['reason'])
    if path:
        chosen,proof=geometry.choose(path,scene.occupied_endpoints_xy_m,geometry.selection_padding(scene,cfg),cfg,d['speed_mps'],scene.t_ref_s-scene.source_first_s)
        row.update(proof=proof)
    rows.append(row)
(A/'probe.json').write_text(json.dumps(dict(source_sha256=hashlib.sha256(p.read_bytes()).hexdigest(),rows=rows),indent=2)+'\n')
for row in rows:
    proof=row.get('proof',{})
    if abs(row['time']*10-round(row['time']*10))<1e-4 or not proof:
        print({**row,'proof':{k:v for k,v in proof.items() if k!='attempts'}})
