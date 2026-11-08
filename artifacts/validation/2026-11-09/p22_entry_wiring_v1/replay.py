"""연결 클래스의 첫 허가와 ID20없는 본선 명령을 원리별 비교한다."""
import hashlib
import json
from pathlib import Path
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.p22_trial import P22Entry

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
D=ROOT/'artifacts/validation/2026-11-07/junction_handoff_v1/run1'
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
src=D/'follow_decisions.jsonl';before=digest(src)
frames=[]
for e in json.loads((D/'trace.json').read_bytes()):
    au=e['autonomy'];v=au.get('vision') or {};lr=v.get('last_result') or {}
    if lr.get('stamp') is not None:frames.append((e['time'],au['marker_ids'],lr['stamp'],v['epoch']))
frames.sort(key=lambda v:v[0]);i=0;calls=0;mismatch=[];admitted=[];before_start=[];active=True;latest=None
for line in src.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':
        mc=MotionConfig(**d['motion_config']);cfg=FollowConfig(**d['follow_config'])
        baseline=LocalFollow(cfg);off=P22Entry(LocalFollow(cfg));on=P22Entry(LocalFollow(cfg))
    if e['kind']=='reset':
        baseline.reset();off.reset();on.reset();continue
    if e['kind']!='call':continue
    raw,scan,h=restore_call(d['input'],mc);t=raw['reference_s']
    scene=motion_scene(scan,h,t,timing_verified=raw['timing_verified'],epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
    while i<len(frames) and frames[i][0]<=t:latest=frames[i];i+=1
    if latest:
        tm,ids,stamp,epoch=latest
        off.frame([v for v in ids if v!=20],stamp,tm,epoch)
        if active:on.frame(ids,stamp,tm,epoch)
    args=(scene,h,raw['speed_mps'],raw['side'],raw['previous_steering_rad'],raw['dt_s'])
    base=baseline.command(*args,raw_scan=scan);without=off.command(*args,raw_scan=scan)
    calls+=1
    if tuple(base[:2])!=tuple(without[:2]):mismatch.append(t)
    if active:
        res=on.command(*args,raw_scan=scan)
        if (res[2].get('p22_trial') or {}).get('first_command_admitted'):
            admitted.append(dict(t=t,speed=res[0],steer=res[1],first=res[2]));active=False
        elif tuple(base[:2])!=tuple(res[:2]):before_start.append(t)
out=dict(calls=calls,without_id20_mismatches=mismatch,before_admission_mismatches=before_start,
         admitted=admitted,input_unchanged=digest(src)==before,input_sha256=before)
(A/'replay_summary.json').write_text(json.dumps(out,indent=2)+'\n',encoding='utf8')
print({k:v for k,v in out.items() if k!='admitted'})
print('admitted',[(q['t'],q['speed'],q['steer']) for q in admitted])
assert not mismatch and not before_start and out['input_unchanged']
assert len(admitted)==1 and abs(admitted[0]['t']-21.92)<1e-8 and admitted[0]['speed']==.65
