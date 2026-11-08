"""r9 실제 연결 출력과 앞서 저장한 메모리 후보 출력 대조."""
import json
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig

HERE=Path(__file__).resolve().parent
events=[json.loads(s) for s in (HERE.parent/'traffic_surface_alignment_v1/straight_moving_lead/run2/follow_decisions.jsonl').read_text().splitlines()]
expected=json.loads((HERE/'shadow_rows.json').read_text())
header=events[0]['data']
ctl=BypassHandoffFollow(FollowConfig(**header['follow_config']),HandoffExecutionConfig.from_mapping(header['execution_config']),allow_simulation_trial=True)
motion=MotionConfig(**header['motion_config']);count=matched=0
for e in events[1:]:
    if e['kind']=='reset':ctl.reset()
    elif e['kind']=='call':
        q=e['data'];out=replay_call(ctl,q['input'],motion);r=expected[count]
        matched+=int(abs(out[0]-r['speed'])<1e-9 and abs(out[1]-r['steer'])<1e-9);count+=1
summary=dict(calls=count,matched_shadow=matched)
(HERE/'replay_connected.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary));assert count==matched==len(expected)
