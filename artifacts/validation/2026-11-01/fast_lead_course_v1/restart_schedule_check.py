"""고정 0.35 m/s 전용 후처리와 별개로 실제 등록 각본을 대조한다."""
import json,math
from pathlib import Path
from hashlib import sha256

b=Path(__file__).resolve().parent;run=b/'run1'
r=json.loads((run/'report.json').read_bytes());green=r['case_window']['green_sim_s']
cfg=json.loads((b/'course.json').read_bytes());schedule=cfg['cases'][0]['schedule']
poses=[json.loads(line) for line in (run/'actor_poses.jsonl').read_text().splitlines()]
before=[p for p in poses if green+8.1<=p[0]<=green+8.9]
after=[p for p in poses if green+9.05<=p[0]<=green+10.05]
def covered(rows,start,end):
    return (len(rows)>=3 and abs(rows[0][0]-start)<=.05 and abs(rows[-1][0]-end)<=.05
        and all(0<b[0]-a[0]<=.05 for a,b in zip(rows,rows[1:])))
def travel_at(t):
    return sum(max(0.,min(t,s['to_s'])-s['from_s'])*s['speed_mps'] for s in schedule)
assert covered(before,green+8.1,green+8.9) and covered(after,green+9.05,green+10.05)
observed=sum(math.dist(a[1:3],b[1:3]) for a,b in zip(after,after[1:]))
expected=travel_at(after[-1][0]-green)-travel_at(after[0][0]-green)
spread=max(math.dist(p[1:3],before[-1][1:3]) for p in before)
answer=dict(scope='실제 기록된 평가 상대의 각본 재출발. 제어 입력/실물 증명이 아님.',
    old_fixed_0p35_check=False,coverage=True,stationary_spread_m=spread,observed_travel_m=observed,
    scheduled_travel_m=expected,error_m=observed-expected,
    tracking_error_bound_m=cfg['acceptance']['harness']['actor_moving_tracking_error_max_m'],
    passed=spread<=.01 and abs(observed-expected)<=cfg['acceptance']['harness']['actor_moving_tracking_error_max_m'],
    inputs={n:sha256((run/n).read_bytes()).hexdigest() for n in ('report.json','actor_poses.jsonl')})
with (b/'restart_schedule_check.json').open('x',encoding='utf-8') as f:json.dump(answer,f,ensure_ascii=False,indent=2)
print(json.dumps(answer,ensure_ascii=False))
