"""저장된 입력은 그대로, 후보 ON 출력만 비교. 폐루프 진행을 증명하지 않는다."""
import dataclasses
import json
from pathlib import Path

from arena_autonomy.follow_trace import replay_events, replay_call
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig

HERE = Path(__file__).resolve().parent
events = [json.loads(s) for s in (HERE.parent/'course_section_v1/run1/follow_decisions.jsonl').read_text().splitlines()]
baseline = replay_events(iter(events))
header = events[0]['data']
cfg = dataclasses.replace(FollowConfig(**header['follow_config']), piecewise_wall_trial=True)
follow = LocalFollow(cfg)
motion = MotionConfig(**header['motion_config'])
rows = []
for event in events[1:]:
    if event['kind'] == 'reset':
        follow.reset()
    elif event['kind'] == 'call':
        out = replay_call(follow, event['data']['input'], motion)
        old = event['data']['output']
        rows.append(dict(t=event['data']['input']['reference_s'], old_speed=old[0], speed=out[0],
                         steer=out[1], reason=out[2].get('reason'), details=out[2]))
summary = dict(off_replay={k:baseline[k] for k in ('calls','matched')},
               changed=sum(abs(r['old_speed']-r['speed'])>1e-9 for r in rows),
               after_135_positive=sum(r['t']>=13.5 and r['speed']>0 for r in rows),
               samples=[{k:v for k,v in r.items() if k!='details'} for r in rows if abs(r['t']-13.52)<.011 or abs(r['t']-20.22)<.011])
(HERE/'replay_summary.json').write_text(json.dumps(summary, indent=2), encoding='utf8')
(HERE/'replay_rows.json').write_text(json.dumps(rows, indent=1, default=str), encoding='utf8')
print(json.dumps(summary))
