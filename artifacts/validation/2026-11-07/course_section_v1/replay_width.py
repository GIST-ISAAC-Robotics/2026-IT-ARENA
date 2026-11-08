"""같은 정지 센서 기록에 구/신 단면만 바꿔 경로 불일치 원인을 비교한다. 개루프다."""
from collections import Counter
import json
from pathlib import Path
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.local_follow import LocalFollow,FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'b0390_course_baseline_v1/run1/follow_decisions.jsonl'
events=[json.loads(x) for x in SOURCE.read_text().splitlines()]
h=events[0]['data'];result={}
for name,overrides in [('old',{}),('new',dict(wall_offset_m=.505,road_half_m=.305))]:
    controller=LocalFollow(FollowConfig(**{**h['follow_config'],**overrides}))
    motion=MotionConfig(**h['motion_config']);reasons=Counter();positive=0;count=0;matched=0;first=None
    for row in events[1:]:
        if row['kind']=='reset':controller.reset();continue
        if row['kind']!='call':continue
        sample=row['data'];out=replay_call(controller,sample['input'],motion)
        count+=1;reasons[out[2].get('reason')]+=1
        if out[0]>0:
            positive+=1
            if first is None:first=dict(t=sample['input']['reference_s'],speed=out[0],steering=out[1],reason=out[2]['reason'])
    result[name]=dict(calls=count,positive_commands=positive,reasons=dict(reasons),first_positive=first)
result['scope']='same saved stationary inputs; positive commands do not prove moving progress'
(HERE/'replay_width.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf8')
print(json.dumps(result,ensure_ascii=False))
