"""run2 평가기 중간 단계 누락을 실제 정책/상태/자세 기록으로 대조한다. 원본 실패 불변."""
import json
from pathlib import Path
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from race_ops_eval import Course

run=HERE/'run2'
report=json.loads((run/'report.json').read_bytes())
old=json.loads((HERE/'run2_analysis.json').read_bytes())
history=json.loads((run/'ops_history.json').read_bytes())
steps=report['steps']
first=next(s for s in steps if s['step']=='ordinary_stopped')
start=next(s for s in steps if s['step']=='press_resume_start' and s['stage']=='queued')
stop=[s for s in steps if s['step']=='press_stop'][-1]
command=next(c for c in report['final_state']['commands'] if c['request']==start['request'])
result=next(h for h in history if (h['race'].get('last_resume') or {}).get('request')==start['request'])
binding=start['binding']
same=all(all(h['binding'][k]==binding[k] for k in ('session','race_id','race_epoch')) for h in history
         if h['wall']>=start['wall_s'])
course=Course()
begin_s=course.nearest(*first['truth'][1:3])[0]
end_s=course.nearest(*stop['truth'][1:3])[0]
gates={k:old['gates'][k] for k in ('visual_start','moving_before_stop','no_reset','one_accepted_resume',
    'first_continuous_stop','final_continuous_stop','stop_policy_and_mcu','pose_coverage',
    'conservative_surface','main_course','source_unchanged','cleanup')}
gates.update(policy_accepted=command['policy']['accepted'] is True,
             motion_permission=command['stage']=='permitted' and result['race']['last_resume']['outcome']=='permitted',
             same_race=same,progress_before_final_stop=end_s-begin_s>=3.)
data=dict(behavior_confirmed=all(gates.values()),gates=gates,
    original_runtime_exit=1,original_analysis_passed=old['passed'],original_error=report['error'],
    resume_request=start['request'],policy=command['policy'],resume=result['race']['last_resume'],
    input_boundary_s=result['race']['last_resume']['boundary_s'],first_permitted_status_s=result['driving']['sim_s'],
    boundary_to_first_permitted_status_s=result['driving']['sim_s']-result['race']['last_resume']['boundary_s'],
    progress=dict(first_stop_s_m=begin_s,final_stop_request_s_m=end_s,delta_m=end_s-begin_s),
    scope='동일 저장 증거의 행동 확인. 평가기 정상 완료는 실패로 유지. 전체 랩/실물/공용망/모든 복구 증명 아님')
(HERE/'run2_reconciled.json').write_text(json.dumps(data,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(data,ensure_ascii=False))
raise SystemExit(0 if data['behavior_confirmed'] else 1)
