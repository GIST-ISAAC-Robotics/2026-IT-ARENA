"""a2 입력에서 원래 판단 재현과 조기 복귀 정책의 상태/명령을 비교한다."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys

from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call, comparable_decision, equal_output, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig

root = Path.cwd()
source = root/'artifacts/validation/2026-11-02/official_forward_v1/a2/follow_decisions.jsonl'
report = json.loads((source.parent/'report.json').read_bytes())
green = report['evaluation']['t_green']
rows = [json.loads(line) for line in source.open()]
header = rows[0]['data']
motion = MotionConfig(**header['motion_config'])
config = FollowConfig(**header['follow_config'])
execution = HandoffExecutionConfig.from_mapping(header['execution_config'])
off = BypassHandoffFollow(config,execution,allow_simulation_trial=True)
on = BypassHandoffFollow(config,replace(execution,reacquire_policy='observed_rejoin_v2'),allow_simulation_trial=True)
counts = dict(calls=0,off_mismatch=0,raised=0,lowered=0,new_hold=0,old_hold_raised=0,steering_change=0,mode_change=0)
details = []
for row in rows[1:]:
    if row['kind'] == 'reset':
        off.reset(); on.reset()
    elif row['kind'] == 'call':
        data = row['data']
        a = freeze(replay_call(off,data['input'],motion))
        b = freeze(replay_call(on,data['input'],motion))
        counts['calls'] += 1
        counts['off_mismatch'] += not equal_output(comparable_decision(a),comparable_decision(data['output']))
        diff = b[0]-a[0]
        counts['raised'] += diff>1e-10
        counts['lowered'] += diff< -1e-10
        counts['new_hold'] += b[0]<=0<a[0]
        counts['old_hold_raised'] += a[0]<=0<b[0]
        counts['steering_change'] += abs(a[1]-b[1])>1e-10
        counts['mode_change'] += a[2].get('handoff_state')!=b[2].get('handoff_state')
        if abs(diff)>1e-10:
            details.append(dict(t=data['input']['reference_s']-green,before=a[0],after=b[0],
                                reserve=b[2].get('follow_entry_space'),before_mode=a[2].get('handoff_state'),after_mode=b[2].get('handoff_state'),before_reason=a[2].get('reason'),after_reason=b[2].get('reason')))
output = dict(scope='fixed sensor input comparison, not counterfactual vehicle progress',counts=counts,
              capture_footer=rows[-1],input_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),differences=details)
target = Path(sys.argv[1])
with target.open('x',encoding='utf-8') as f:
    json.dump(output,f,ensure_ascii=False,indent=2)
print(json.dumps(counts))
raise SystemExit(1 if counts['off_mismatch'] else 0)
