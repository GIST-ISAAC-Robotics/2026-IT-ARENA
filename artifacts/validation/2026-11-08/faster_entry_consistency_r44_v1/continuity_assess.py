"""추월 성공과 구분하는 추종 지속 판정. 원 평가를 덮어쓰지 않는다."""
import json
from pathlib import Path

A = Path(__file__).resolve().parent/'straight_moving_lead'
report = json.loads((A/'run1/report.json').read_bytes())
assess = json.loads((A/'assessment_run1.json').read_bytes())
trace = [json.loads(s) for s in (A/'run1/follow_decisions.jsonl').open()]
tg, te = [report['case_window'][k] for k in ('green_sim_s','end_sim_s')]
calls = [e['data'] for e in trace if e['kind']=='call' and tg<=e['data']['input']['reference_s']<=te]
# 실제 추월 거부 경로가 사용됐는지 확인. 진행/간격은 평가용 정답만 사용.
rejections = sum(c['output'][2].get('moving_pass',{}).get('entry',{}).get('reason') ==
    'moving_pass_sustained_side_width_unavailable' for c in calls)
positive_rejections = sum(c['output'][0]>0 and c['output'][2].get('moving_pass',{}).get('entry',{}).get('reason') ==
    'moving_pass_sustained_side_width_unavailable' for c in calls)
metrics = report['evaluation']['case_metrics']
progress = report['evaluation']['progress']
result = dict(scope='same 10s fast-lead scene; continued following, NOT completed overtaking',
    common=assess['common'], motion=assess['motion'], raw_gates=report['gates'],
    original_pass_behavior=assess['behavior'], extra=assess['extra'],
    rejected_entry_calls=rejections, positive_follow_rejections=positive_rejections,
    case_metrics=metrics, progress=progress)
result['continuity'] = dict(
    rejection_exercised=rejections>0 and positive_rejections>0,
    no_entry=assess['extra']['entry_after_green_s'] is None,
    progresses_6m=progress['at_window_end_m']>=6.,
    no_permanent_stop=assess['motion']['longest_below_005_after_motion_s']<=.5,
    finishes_behind_within_1p2m=-1.2<=assess['extra']['final_relative'][1]<0.)
result['passed'] = all(result['common'].values()) and all(result['continuity'].values())
(A/'continuity_run1.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(dict(passed=result['passed'], continuity=result['continuity'], case_metrics=metrics)))
