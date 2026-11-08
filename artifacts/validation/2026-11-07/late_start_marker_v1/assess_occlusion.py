"""가림 후 출발 기능의 보충 판정. 기존 운영 도구 raw passed를 덮어쓰지 않는다."""
import json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'scripts'))
from g1_traffic_eval import stop_pose_evidence
HERE=Path(__file__).resolve().parent
n=sys.argv[1]
p=HERE/('occlusion_run'+n)
r=json.loads((p/'report.json').read_text())
steps={s['step']:s for s in r.get('steps',[])}
trace=json.loads((p/'autonomy_trace.json').read_text())
poses=[json.loads(s) for s in (p/'ego_poses.jsonl').read_text().splitlines() if s.strip()]
started=next((s for s in trace if s.get('started')),None)
stop=steps.get('press_stop',{}).get('truth',[None])[0]
dense=stop_pose_evidence([s[:4] for s in poses],poses[-1][0],.5) if poses else dict(passed=False)
removed=steps.get('screen_removed',{}).get('removed_sim_s')
progress=steps.get('late_start_progress',{})
unchanged=json.loads((HERE/('unchanged_v'+n+'.json')).read_text())
gates=dict(
 execution_completed=r.get('completed') is True and 'error' not in r,
 red_occluded=steps.get('blocked_red',{}).get('marker_ids')==[] and steps.get('blocked_red',{}).get('signal')=='late_markers_unavailable',
 green_occluded=steps.get('blocked_green',{}).get('marker_ids')==[] and steps.get('blocked_green',{}).get('signal')=='late_markers_unavailable',
 no_start_while_occluded=removed is not None and not any(s.get('started') for s in trace if s['sim_time_s']<removed),
 actual_late_start=started is not None and started.get('signal')=='late_green_confirmed',
 green_window=0<progress.get('green_request_to_start_s',99)<5,
 progress_one_m=progress.get('progress_m',0)>=1,
 explicit_stop=stop is not None and r.get('final_state',{}).get('race',{}).get('latch')=='explicit_stop',
 dense_actual_stop=bool(dense.get('passed')),
 source_unchanged=r.get('source_unchanged') is True and unchanged['changed']==[],
 clean_exit=r.get('remaining_test_pids')==[] and all(x==0 for x in r.get('exit_codes',[1])))
result=dict(scope='one optical screen and current RGB/B3 late start; not multi-car racing or physical camera proof',
 raw_passed=r.get('passed'),passed=all(gates.values()),gates=gates,first_started=started and {k:started.get(k) for k in ('sim_time_s','signal','marker_ids')},
 reveal_to_first_start_s=started['sim_time_s']-removed if started and removed is not None else None,
 progress_m=progress.get('progress_m'),dense_stop=dense,source_count=unchanged['count'])
(p/'assessment.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(result,ensure_ascii=False))
