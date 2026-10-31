import json
from pathlib import Path
from arena_autonomy.local_follow import FollowConfig
from continuous_prefix import refine

root=Path(__file__).resolve().parent
source=root.parent/'g2_relative_yield_course_video_v1/progress_review3/review.json'
saved=json.loads(source.read_text(encoding='utf-8'))
cfg=FollowConfig(**saved['follow_config'])
rows=[]
for s in saved['snapshots']:
    out=refine(s['short_evidence'],cfg)
    rows.append(dict(time_s=s['time_s'],side=s['side'],baseline=s['short_evidence']['reason'],
        supported=out['supported'],reason=out['reason'],continuous_prefix=out.get('continuous_prefix')))
result=dict(scope='current stored geometry only; no command or driving change',rows=rows)
with (root/'probe1.json').open('x',encoding='utf-8') as stream:
    json.dump(result,stream,ensure_ascii=False,indent=2)
print(json.dumps(result,ensure_ascii=False))
