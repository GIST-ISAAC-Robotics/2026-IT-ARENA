"""완료된 기존 입력 5호출로 비교기의 연결만 확인한다."""
import json
from pathlib import Path
from compare_trace import ROOT,BASE,FollowConfig,LocalFollow,MotionConfig,replay_call,ShadowSafety,RecoveryPolicy
from perturb import CASES,transform

trace=BASE.parent/'official_traffic_v1/run1_course/follow_decisions.jsonl'
with trace.open(encoding='utf-8') as stream:
    header=json.loads(next(stream))['data']; rows=[]
    for line in stream:
        event=json.loads(line)
        if event['kind']=='call': rows.append(event['data']['input'])
        if len(rows)==5: break
config=FollowConfig(**header['follow_config']); motion=MotionConfig(**header['motion_config'])
result={}
for case in CASES:
    follow=LocalFollow(config); guard=ShadowSafety(RecoveryPolicy(True,True,True),config,rows[0]['rear_half_angle_deg'])
    records=[]
    for row in rows:
        changed=transform(row,case)
        command=replay_call(follow,changed,motion)
        safe,status=guard.step(changed,motion,command[:2])
        assert 0<=safe[0]<=command[0]+1e-12
        records.append(dict(command=command[:2],safe=safe,reason=status.get('reason')))
    result[case.name]=records
with (BASE/'smoke_v1.json').open('x',encoding='utf-8') as stream:
    json.dump(dict(scope='first five saved calls; wiring only',results=result),stream,indent=2)
print('5 cases x 5 saved calls: controller / independent guard connected')
