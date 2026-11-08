"""0명령의 저장 프레임에서 비접근 측방 이동 검사가 설명 가능한지 확인한다. 주행 연결 없음."""
from collections import Counter
from hashlib import sha256
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import restore_call, motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.known_structure import parse_sectors
from arena_autonomy.moving_pass_wall_padding import current_outer_and_rear_padding
from arena_autonomy.parallel_side_departure import limited_clearance
from arena_autonomy.bypass_entry_trial import entry_motion_region, lateral_config, region_blocked, SelectorConfig
from arena_autonomy.local_path import braking_speed

HERE=Path(__file__).resolve().parent
TRACE=HERE.parent/'moving_pass_wall_mask_v1/straight_moving_lead/run1/follow_decisions.jsonl'
rows=[]
for line in TRACE.open(encoding='utf8'):
    e=json.loads(line); d=e['data']
    if e['kind']=='header':
        mc=MotionConfig(**d['motion_config']); cfg=FollowConfig(**d['follow_config'])
        sectors=parse_sectors(cfg.known_structure_sectors_json)
    if e['kind']!='call' or d['output'][0]!=0:
        continue
    info=d['output'][2]
    if info.get('reason')!='moving_pass_current_path_unavailable':
        continue
    entry=info['moving_pass']['entry']; cands=entry.get('candidates',[])
    if len(cands)!=1 or cands[0]['reason']!='known_occupied_command_hold':
        continue
    row=cands[0]; side=row['current_outer_wall']; wall=entry['reference']['walls'][side]
    x,scan,hist=restore_call(d['input'],mc)
    scene=motion_scene(scan,hist,x['reference_s'],timing_verified=x['timing_verified'],
                       epoch=x['epoch'],rear_half_angle_deg=x['rear_half_angle_deg'])
    pts=np.asarray(scene.occupied_endpoints_xy_m); prev=x['previous_steering_rad']
    speed=x['speed_mps']; dt=x['dt_s']; age=scene.t_ref_s-scene.source_first_s
    pad,_=current_outer_and_rear_padding(scene,cfg,side,wall,raw_scan=scan,sectors=sectors)
    results={}
    # stop_then_straight는 현재의 직전 조향을 무시하는 반사실 진단. 명령 근거로 쓰지 않는다.
    for mode,angle,prior in [('current',row['steering_rad'],prev),
                             ('rate_limited_straight',float(np.clip(0.,prev-3*dt,prev+3*dt)),prev),
                             ('stop_then_straight_diagnostic',0.,0.)]:
        clear,why=limited_clearance(pts,(angle,prior),pad,speed,age,dt,cfg)
        target=min(.35,braking_speed(clear,age)) if clear>0 else 0.
        blocked=None
        if target>0:
            region=entry_motion_region(scene,speed,target,angle,prior,dt,cfg,lateral_config(cfg),pad)
            blocked=region_blocked(region,SelectorConfig())
        results[mode]=dict(steering=angle,prior=prior,clearance=clear,geometry=why['reason'],
                            region_blocked=blocked,attempted_speed=target,
                            geometry_and_region=bool(target>0 and blocked is None))
    rows.append(dict(t=x['reference_s'],source=scene.source_first_s,speed=speed,results=results))
counts={k:dict(Counter((str(r['results'][k]['geometry'])+' / '+str(r['results'][k]['region_blocked']))
                        for r in rows)) for k in rows[0]['results']}
out=dict(scope='fixed input geometry only; no change to commands or source',total=len(rows),
         counts=counts,rows=rows,trace_sha256=sha256(TRACE.read_bytes()).hexdigest())
(HERE/'departure_result.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(dict(total=len(rows),counts=counts),ensure_ascii=False))
