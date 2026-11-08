"""기록 고정 재생: OFF 원출력 일치, ON 제어각 피드백은 제자리에서 양수 요청까지. 실제 주행 증명 아님."""
import json,math
from pathlib import Path
from arena_autonomy.follow_trace import MotionConfig,FollowConfig,replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
A=Path(__file__).resolve().parent;p=A.parent/'ops_traffic_resume_v1/run1/follow_decisions.jsonl'
off=on=None;previous=None;events=[];mismatch=0;count=0;first=None;applied=False
for line in p.open():
 r=json.loads(line);d=r['data'];kind=r['kind']
 if kind=='header':
    m=MotionConfig(**d['motion_config'])
    off,on=[BypassHandoffFollow(FollowConfig(**d['follow_config']),HandoffExecutionConfig.from_mapping({**d['execution_config'],'stopped_entry_centering':flag}),allow_simulation_trial=True) for flag in (False,True)]
 elif kind=='reset':off.reset();on.reset();previous=None;applied=False
 elif kind=='call':
    i=d['input'];t=i['reference_s']
    if t>=20:break
    bo=freeze(replay_call(off,i,m));count+=1
    mismatch+=not equal_output(comparable_decision(bo),comparable_decision(d['output']))
    if first is not None:continue
    ci={**i,'previous_steering_rad':previous} if applied else i
    v,st,detail=replay_call(on,ci,m)
    if detail.get('stopped_entry_centering') and v==0:
        assert i['speed_mps']==0 and abs(st-ci['previous_steering_rad'])<=3*i['dt_s']+1e-8 and abs(st)<=abs(ci['previous_steering_rad'])+1e-8
        if not applied:events.append(dict(first_align_s=t,original_steering_rad=ci['previous_steering_rad']))
        applied=True
    if applied:
        previous=st
        if v>0:
            first=dict(t=t,command_mps=v,previous_rad=ci['previous_steering_rad'],steering_rad=st,reason=detail['reason'],admission=detail.get('entry_admission',{}).get('admitted'))
            break
assert count>100 and mismatch==0 and events and first and first['admission'] is True and first['reason']=='handoff_entry' and abs(first['previous_rad'])<=.08
out=dict(off_prefix_matched=count,off_mismatch=mismatch,align=events,first_actual_policy_request=first,scope='open-loop fixed sensor poses; zero-target steering fed back as last command; stop at first nonzero command')
(A/'replay_check.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps(out))
