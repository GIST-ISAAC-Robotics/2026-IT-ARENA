"""동일 입력·동일 프로파일러 OFF 조건의 순수 관측 함수 AB/BA 비용 대조."""
from collections import Counter
import argparse
import cProfile
import json
from pathlib import Path
import sys
import time
import types
import zipfile

import numpy as np

from arena_autonomy import observation_support as current
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest

ROOT=Path.cwd()
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--reference',type=Path,required=True)
options=parser.parse_args()
OUT=options.output.resolve()
OUT.mkdir(parents=True,exist_ok=False)
CAPTURE=ROOT/'artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl'
FROZEN=options.reference.resolve()
assert not (OUT/'report.json').exists()
before=digest(CAPTURE)
names=['src/arena_autonomy/arena_autonomy/'+n+'.py' for n in
       ('observation_support','observation_lateral','linear_observation','local_follow','scene_motion','local_scene','lidar_motion','local_path')]
names.append('scripts/profile_recent_observation.py')
hashes={n:digest(ROOT/n) for n in names}
with zipfile.ZipFile(OUT/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
    for n in names:z.write(ROOT/n,n)
with zipfile.ZipFile(FROZEN) as z:
    source=z.read('src/arena_autonomy/arena_autonomy/observation_support.py').decode()
old=types.ModuleType('frozen_observation_support')
exec(compile(source,'frozen_observation_support','exec'),old.__dict__)
cache=current.RecentScenes();last=None;scenes=[]
with CAPTURE.open() as stream:
    for line in stream:
        e=json.loads(line)
        if e['kind']=='header':
            motion=MotionConfig(**e['data']['motion_config']);cfg=FollowConfig(**e['data']['follow_config'])
        if e['kind']!='call':continue
        data,scan,h=restore_call(e['data']['input'],motion)
        key=(data['epoch'],scan.header.stamp.sec,scan.header.stamp.nanosec)
        if key==last:continue
        last=key
        scene=motion_scene(scan,h,data['reference_s'],timing_verified=data['timing_verified'],
                          epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        needed=e['data']['output'][2]['reason']=='follow_front_unobserved'
        sources,_=cache.observe(scene,h,transport=needed)
        if needed:
            for label,angle in [('recorded',data['previous_steering_rad']),('left',.175417)]:
                scenes.append((e['seq'],label,(sources,data['speed_mps'],.2,angle,data['previous_steering_rad'],cfg)))
            if len(scenes)==16:break
kwargs={'protected_sensor_pose':(motion.lidar_x_m,motion.lidar_y_m,motion.lidar_yaw_rad)}
rows=[];outputs={}
for seq,label,args in scenes:
    for module in (old,current):module.sweep_support(*args,**kwargs)
    for repeat in range(3):
        order=[('old',old),('new',current)] if repeat%2==0 else [('new',current),('old',old)]
        for name,module in order:
            begin=time.perf_counter_ns(); result=module.sweep_support(*args,**kwargs)
            elapsed=(time.perf_counter_ns()-begin)/1e6
            rows.append(dict(seq=seq,label=label,repeat=repeat,version=name,elapsed_ms=elapsed,result=result))
            outputs[(seq,label,name)]=result
profile=cProfile.Profile()
profile.enable()
for _,_,args in scenes:current.sweep_support(*args,**kwargs)
profile.disable();profile.dump_stats(str(OUT/'current.pstats'))
summary={}
for name in ('old','new'):
    vals=[r['elapsed_ms'] for r in rows if r['version']==name and r['result']['reason']!='observation_invalid_state']
    summary[name]=dict(count=len(vals),p50=float(np.percentile(vals,50)),p95=float(np.percentile(vals,95)),
                       maximum=max(vals),over20=sum(v>20 for v in vals))
report=dict(scope='same_input_AB_BA_function_only_not_ROS_deadline',samples=rows,summary_ms=summary,
            decisions_same=all(outputs[(s,l,'old')]['supported']==outputs[(s,l,'new')]['supported'] for s,l,_ in scenes),
            details_same=all(outputs[(s,l,'old')]==outputs[(s,l,'new')] for s,l,_ in scenes),
            source_sha256=hashes,input_sha256=before,input_unchanged=before==digest(CAPTURE),
            source_unchanged=all(digest(ROOT/n)==h for n,h in hashes.items()),frozen_sha256=digest(FROZEN))
(OUT/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('samples','source_sha256')}))
