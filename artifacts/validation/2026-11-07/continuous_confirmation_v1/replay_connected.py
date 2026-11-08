"""실제 직각 short-prefix 연결과 사전 후보 대조, OFF 기존 명령 보존."""
import json
from pathlib import Path
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
HERE=Path(__file__).resolve().parent
summary={}
for label,source,expected in [('r8',HERE.parent/'paired_prefix_v1/run1/follow_decisions.jsonl',HERE/'shadow_rows.json'),
                              ('off',HERE.parent/'course_section_v1/run1/follow_decisions.jsonl',None)]:
    events=[json.loads(s) for s in source.read_text().splitlines()]
    header=events[0]['data'];ctl=LocalFollow(FollowConfig(**header['follow_config']));motion=MotionConfig(**header['motion_config'])
    rows=json.loads(expected.read_text()) if expected else None
    count=matched=0
    for e in events[1:]:
        if e['kind']=='reset':ctl.reset()
        elif e['kind']=='call':
            q=e['data'];out=replay_call(ctl,q['input'],motion)
            target=(rows[count]['speed'],rows[count]['steer']) if rows else q['output'][:2]
            matched+=int(abs(out[0]-target[0])<1e-9 and abs(out[1]-target[1])<1e-9);count+=1
    summary[label]=dict(calls=count,expected_commands_matched=matched)
(HERE/'replay_connected.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary));assert all(r['calls']==r['expected_commands_matched'] for r in summary.values())
