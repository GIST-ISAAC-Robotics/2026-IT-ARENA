"""assess.py 보충 진행 계산을 r22 run1 저장 자세로 시험한다(평가 스크립트 점검 전용)."""
import json,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import assess
run=assess.ROOT/'artifacts/validation/2026-11-08/traffic_lap_anchor_v1/two_actor_course_lap/run1'
r=json.loads((run/'report.json').read_bytes())
t0=r['case_window']['green_sim_s']
for end in (t0+44.,r['case_window']['end_sim_s']):
    c=assess.supplementary_progress(run,r,t0,end)
    print(json.dumps({k:c[k] for k in ('valid','progress_m','ego_start_s_m','ego_end_s_m','actor_end_s_m','projection_jumps','reference_s_m')}))
print('r22 stored full_course_progress',json.dumps(r['evaluation']['full_course_progress']))
