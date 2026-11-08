"""기존 저장 입력의 진입 제안과 실제 점유 거절 대조. 제어기/월드 변경 없음."""
from pathlib import Path
import json,math,hashlib
import numpy as np
from arena_autonomy.follow_trace import restore_call,MotionConfig,FollowConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.known_structure import parse_sectors
from arena_autonomy.moving_pass_wall_padding import current_outer_and_rear_padding
from arena_autonomy.bypass_entry_trial import current_outer_wall_padding
A=Path(__file__).resolve().parent
sources={
 'r32':'moving_pass_wall_mask_v1/straight_moving_lead/run1/follow_decisions.jsonl',
 'right20':'faster_onroad_space_v2/straight_moving_lead/run1/follow_decisions.jsonl',
 'medium':'medium_recessed_pass_v1/straight_moving_lead/run1/follow_decisions.jsonl'}
out={}
for key,rel in sources.items():
 p=A.parent/rel; header=None; entries=[]; holds=[]; allcalls=0
 for line in p.open(encoding='utf8'):
  e=json.loads(line);d=e['data']
  if e['kind']=='header':header=d
  if e['kind']!='call':continue
  allcalls+=1;m=d['output'][2].get('moving_pass',{}); en=m.get('entry',{})
  if en.get('pass_width') and en.get('selected') and en['selected']['offset_m']:
   entries.append((d,en))
  candidates=en.get('candidates',[])
  if d['output'][0]==0 and len(candidates)==1 and candidates[0]['reason']=='known_occupied_command_hold':holds.append((d,en))
 cfg=FollowConfig(**header['follow_config']);mc=MotionConfig(**header['motion_config']);sectors=parse_sectors(cfg.known_structure_sectors_json)
 rows=[]
 for i in sorted(set([0,len(holds)//2,len(holds)-1])) if holds else []:
  d,en=holds[i];args,scan,hist=restore_call(d['input'],mc);t=args['reference_s']
  scene=motion_scene(scan,hist,t,timing_verified=args['timing_verified'],epoch=args['epoch'],rear_half_angle_deg=args['rear_half_angle_deg'])
  row=en['candidates'][0];side=row['current_outer_wall'];wall=en['reference']['walls'][side]
  pad,proof=current_outer_and_rear_padding(scene,cfg,side,wall,raw_scan=scan,sectors=sectors) if header['execution_config']['moving_pass_profile']=='faster_v1' else current_outer_wall_padding(scene,cfg,side,wall)
  rays=[r for r in scene.rays if r.status=='return'];pts=np.asarray([r.hit_xy_m for r in rays]);base=.025+cfg.measurement_padding_m
  overlap=(abs(pts[:,0])<=cfg.length_m/2+pad)&(abs(pts[:,1])<=cfg.width_m/2+pad)
  fixed=(abs(pts[:,0])<=cfg.length_m/2+base)&(abs(pts[:,1])<=cfg.width_m/2+base)
  a,b=wall['slope'],wall['intercept_m']; norm=math.hypot(1.,a);sign=1 if side=='left' else -1
  residual=abs((pts[:,1]-a*pts[:,0]-b)/norm-sign*cfg.wall_offset_m)
  taken=np.flatnonzero(overlap)
  rows.append(dict(t=t,speed=args['speed_mps'],command=d['output'][:2],fixed_overlap_count=int(fixed.sum()),timing_overlap_count=int(overlap.sum()),
    aligned_timing_count=int(np.sum(overlap&(residual<=.015))),
    wall_slope=a,wall_intercept=b,rear_witness=wall.get('rear_geometry_witness'),
    points=[dict(index=rays[j].source_index,xy=pts[j].tolist(),padding=float(pad[j]),age=t-rays[j].acquired_at_s,wall_residual=float(residual[j]),is_fixed=bool(pad[j]<=base+1e-10)) for j in taken]))
 out[key]=dict(trace=rel,sha256=hashlib.sha256(p.read_bytes()).hexdigest(),calls=allcalls,entry_candidates=len(entries),occupied_holds=len(holds),
  first_entry=None if not entries else dict(t=entries[0][0]['input']['reference_s'],speed=entries[0][0]['input']['speed_mps'],selected=entries[0][1]['selected'],width=entries[0][1]['pass_width']),
  representative_holds=rows)
with (A/'audit.json').open('x',encoding='utf8') as f:json.dump(out,f,indent=2)
print(json.dumps({k:dict(calls=v['calls'],holds=v['occupied_holds'],entry=v['first_entry'],samples=[{kk:vv for kk,vv in r.items() if kk not in ('points','rear_witness')} for r in v['representative_holds']]) for k,v in out.items()}))
