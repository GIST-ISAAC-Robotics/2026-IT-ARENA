"""r49 입구 기록 불변 및 첫 출구 허가. 허가 뒤 반사실 명령을 주행으로 해석하지 않는다."""
import hashlib,json
from pathlib import Path
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.p22_trial import P22Entry
from arena_autonomy.p22_route_trial import P22Route

A=Path(__file__).resolve().parent
source=A.parent/'p22_entry_wiring_v1/run1/follow_decisions.jsonl'
sha=lambda:hashlib.sha256(source.read_bytes()).hexdigest()
before=sha();count=0;mismatches=[];first_exit=None;prefix=[]
for line in source.open():
    e=json.loads(line);k=e['kind'];d=e['data']
    if k=='header':
        mc=MotionConfig(**d['motion_config']);cfg=FollowConfig(**d['follow_config'])
        old=P22Entry(LocalFollow(cfg));new=P22Route(LocalFollow(cfg))
    elif k=='p22_frame':old.frame(**d);new.frame(**d)
    elif k=='reset':old.reset(force=d['reason']=='autonomy_reset');new.reset(force=d['reason']=='autonomy_reset')
    elif k=='call':
        count+=1;t=d['input']['reference_s'];out=replay_call(old,d['input'],mc)
        if not equal_output(comparable_decision(freeze(out)),comparable_decision(d['output'])):mismatches.append(t)
        if first_exit is None:
            trial=replay_call(new,d['input'],mc)
            if new.phase=='exit':
                first_exit=dict(t=t,speed=trial[0],steer=trial[1],path=new.path.tolist(),detail=freeze(trial[2]))
            elif out[:2]!=trial[:2]:prefix.append(t)
out=dict(calls=count,r49_mismatches=mismatches,before_exit_mismatches=prefix,first_exit=first_exit,input_unchanged=sha()==before)
(A/'replay_summary.json').write_text(json.dumps(out,indent=2)+'\n')
print({k:v for k,v in out.items() if k!='first_exit'})
print('first_exit',None if first_exit is None else {k:first_exit[k] for k in ('t','speed','steer')})
assert not mismatches and not prefix and first_exit and first_exit['speed']>0 and out['input_unchanged']
