"""실제 연결 코드의 출력이 검토한 shadow 출력과 일치하는지 확인한다."""
import json
from pathlib import Path
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig

HERE=Path(__file__).resolve().parent
results={}
for name,relative in [('old_off','course_section_v1/run1'),('corner_on','piecewise_corner_v1/run1')]:
    events=[json.loads(s) for s in (HERE.parent/relative/'follow_decisions.jsonl').read_text().splitlines()]
    expected=json.loads((HERE/(name+'_shadow_rows.json')).read_text())
    header=events[0]['data'];ctl=LocalFollow(FollowConfig(**header['follow_config']));motion=MotionConfig(**header['motion_config'])
    count=matched=old_matched=0
    for e in events[1:]:
        if e['kind']=='reset':ctl.reset()
        elif e['kind']=='call':
            q=e['data'];out=replay_call(ctl,q['input'],motion);saved=expected[count]
            matched+=int(abs(out[0]-saved['speed'])<1e-9 and abs(out[1]-saved['steer'])<1e-9)
            old_matched+=int(abs(out[0]-q['output'][0])<1e-9 and abs(out[1]-q['output'][1])<1e-9)
            count+=1
    results[name]=dict(calls=count,shadow_command_matched=matched,original_command_matched=old_matched)
(HERE/'replay_connected.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(json.dumps(results))
assert all(x['calls']==x['shadow_command_matched'] for x in results.values())
assert results['old_off']['calls']==results['old_off']['original_command_matched']
