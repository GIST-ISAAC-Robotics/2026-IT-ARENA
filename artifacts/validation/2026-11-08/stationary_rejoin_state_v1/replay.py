"""연결한 단일 상태 전달 수정이 앞서 비교한 후보와 같은 명령인지 검사한다."""
import hashlib,json
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
HERE=Path(__file__).resolve().parent
OLD=HERE.parent/'stationary_wall_witness_v1'
SOURCE=OLD/'persistent_block/run1/follow_decisions.jsonl'
events=[json.loads(l) for l in SOURCE.read_text().splitlines()]
shadow=json.loads((OLD/'phase_shadow.json').read_text())
changes={r['t']:r['after'] for r in shadow['changes']}
h=events[0]['data'];mc=MotionConfig(**h['motion_config'])
ctl=BypassHandoffFollow(FollowConfig(**h['follow_config']),HandoffExecutionConfig.from_mapping(h['execution_config']),allow_simulation_trial=True)
count=0
for event in events[1:]:
    if event['kind']=='reset':ctl.reset()
    elif event['kind']=='call':
        d=event['data'];out=freeze(replay_call(ctl,d['input'],mc));expected=changes.get(d['input']['reference_s'],d['output'][:2])
        assert equal_output(out[:2],expected),(d['input']['reference_s'],out[:2],expected)
        count+=1
result=dict(calls=count,all_numeric_match_prior_shadow=True,changes=len(changes),
    source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),scope='fixed input; not actual return')
(HERE/'replay_result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result))
