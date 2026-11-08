"""실제 연결 옵션 OFF 동일성/ON 고정입력 대조. 원시 피드백은 변하지 않는다."""
from dataclasses import replace
import hashlib,json
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig

HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'persistent_block_newcourse_v1/persistent_block/run1/follow_decisions.jsonl'
events=[json.loads(l) for l in SOURCE.read_text().splitlines()]
h=events[0]['data'];mc=MotionConfig(**h['motion_config'])

def replay(enabled):
    ec=HandoffExecutionConfig.from_mapping(h['execution_config'])
    ec=replace(ec,bypass=replace(ec.bypass,rear_wall_witness=enabled))
    ctl=BypassHandoffFollow(FollowConfig(**h['follow_config']),ec,allow_simulation_trial=True)
    rows=[]
    for event in events[1:]:
        if event['kind']=='reset':ctl.reset()
        elif event['kind']=='call':
            d=event['data'];out=freeze(replay_call(ctl,d['input'],mc));old=d['output']
            rows.append(dict(t=d['input']['reference_s'],before=old[:2],after=out[:2],
                same=equal_output(comparable_decision(out),comparable_decision(old)),
                numeric_same=equal_output(out[:2],old[:2]),reason=out[2].get('bypass',{}).get('reason')))
    return rows

off=replay(False);assert all(r['same'] for r in off)
on=replay(True);changes=[r for r in on if not r['numeric_same']]
positive=sum(r['before'][0]==0 and r['after'][0]>0 for r in changes)
assert positive>0
result=dict(scope='fixed recorded feedback, not actual bypass completion',source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
    baseline_calls=len(off),baseline_all_equal=True,changes=len(changes),positive_from_zero=positive,first_changes=changes[:4])
(HERE/'replay_result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in result.items() if k!='first_changes'},ensure_ascii=False))
