"""저장 입력에서 조향 가능 전이 선택의 판단 차이만 본다. 반사실 주행이 아니다. 인자: 실행 폴더, 출력."""
from dataclasses import replace
import json
from pathlib import Path
import sys

from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import comparable_decision, equal_output, freeze, replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig

run, target = Path(sys.argv[1]), Path(sys.argv[2])
source = run/'follow_decisions.jsonl'
green = json.loads((run/'report.json').read_bytes())['evaluation']['t_green']
rows = [json.loads(line) for line in source.open(encoding='utf-8')]
header = rows[0]['data']
motion = MotionConfig(**header['motion_config'])
config = FollowConfig(**header['follow_config'])
execution = HandoffExecutionConfig.from_mapping(header['execution_config'])
off = BypassHandoffFollow(config, execution, allow_simulation_trial=True)
on = BypassHandoffFollow(config, replace(execution, bypass=replace(execution.bypass, steerable_transition=True)),
                         allow_simulation_trial=True)
counts = dict(calls=0, off_mismatch=0, raised=0, lowered=0, mode_change=0)
changes = []
for row in rows[1:]:
    if row['kind'] == 'reset':
        off.reset()
        on.reset()
    elif row['kind'] == 'call':
        data = row['data']
        a = freeze(replay_call(off, data['input'], motion))
        b = freeze(replay_call(on, data['input'], motion))
        counts['calls'] += 1
        counts['off_mismatch'] += not equal_output(comparable_decision(a), comparable_decision(data['output']))
        counts['raised'] += b[0]-a[0] > 1e-10
        counts['lowered'] += b[0]-a[0] < -1e-10
        if a[2].get('handoff_state') != b[2].get('handoff_state'):
            counts['mode_change'] += 1
            if len(changes) < 3:
                selected = (b[2].get('bypass') or {}).get('selected')
                changes.append(dict(t=data['input']['reference_s']-green, before=a[0], after=b[0], state=b[2].get('handoff_state'),
                                    reason=b[2].get('reason'), selected=selected,
                                    admission=(b[2].get('entry_admission') or {}).get('consistent')))
output = dict(scope='fixed sensor input; decisions after the first difference are not a counterfactual run', counts=counts, first_changes=changes)
with target.open('x', encoding='utf-8') as stream:
    json.dump(output, stream, ensure_ascii=False, indent=2, default=str)
print(json.dumps(output, ensure_ascii=False, default=str)[:900])
