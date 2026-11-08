"""분석 장면을 한 번 고정한 뒤 속도별 짧은 원호·관측 범위를 비교한다. 제어 연결 없음."""
import json,runpy
from pathlib import Path
from arena_autonomy.follow_trace import restore_call,motion_scene,MotionConfig,FollowConfig
from arena_autonomy.bypass_entry_trial import entry_motion_region
from arena_autonomy.local_path import swept_limit,braking_speed
from arena_autonomy.pass_release_trial import lateral_config,region_blocked,SelectorConfig
import numpy as np

A=Path(__file__).resolve().parent;cached=A/'blocked_snapshot.json'
if not cached.exists():
    env=runpy.run_path(str(A/'blocked_geometry.py'))
    snapshot=dict(input=env['target']['input'],follow_config=vars(env['cfg']),motion_config=vars(env['m']),
                  points=env['points'].tolist(),padding=env['padding'].tolist(),path=env['candidate']['path_points'],
                  source_sha256=env['result']['trace_sha256'],matched_calls=env['count'])
    cached.write_text(json.dumps(snapshot,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
d=json.loads(cached.read_bytes());cfg=FollowConfig(**d['follow_config']);i,scan,history=restore_call(d['input'],MotionConfig(**d['motion_config']))
scene=motion_scene(scan,history,i['reference_s'],timing_verified=i['timing_verified'],epoch=i['epoch'],rear_half_angle_deg=i['rear_half_angle_deg'])
points,padding=np.asarray(d['points']),np.asarray(d['padding']);rows=[]
for steer in (-.37,-.35,-.33,-.31,-.13452914372133695):
    clearance=swept_limit(points,steer,cfg.wheelbase_m,horizon=2.5,margin=padding,length=cfg.length_m,width=cfg.width_m)
    for speed in (.35,.25,.15,.08):
        cap=braking_speed(clearance,scene.t_ref_s-scene.source_first_s)
        region=entry_motion_region(scene,0.,speed,steer,steer,i['dt_s'],cfg,lateral_config(cfg),padding)
        rows.append(dict(steering_rad=steer,target_mps=speed,clearance_m=clearance,braking_cap_mps=cap,
                         region=region,blocked=region_blocked(region,SelectorConfig()),
                         regional_checks_pass=cap>=speed and region_blocked(region,SelectorConfig()) is None))
out=dict(scope='동일 저장 센서 장면의 정지 후 조향 정렬 가설. 실제 정렬/추종/반복 진행/양수 속도 허가 아님. 원 반응·제동·가림 예산은 불변.',source_sha256=d['source_sha256'],rows=rows)
(A/'blocked_snapshot_comparison.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps([(r['steering_rad'],r['target_mps'],r['regional_checks_pass'],r['blocked']) for r in rows]))
