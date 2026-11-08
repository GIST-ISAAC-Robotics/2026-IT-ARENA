"""등록된 최소 운영 목표 사후 평가. 실제 차체 정답은 이 평가에만 사용한다."""
from pathlib import Path
import hashlib,json,math
import numpy as np
from g1_traffic_eval import rectangle
from audit_local_pursuit_road import road_polygon
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];run=HERE/'run1'
read=lambda p:json.loads(p.read_bytes())
r=read(run/'report.json');reg=read(HERE/'registered.json');steps=r.get('steps',[])
rows=[json.loads(l) for l in (run/'ego_poses.jsonl').read_bytes().splitlines()]
byname=lambda name:[s for s in steps if s['step']==name]
commands=[s['step'][6:] for s in steps if s['step'].startswith('press_')]
holds=byname('first_stop_hold')+byname('final_stop_hold');resumes=byname('resume_closed');progress=byname('post_resume_progress')
hold_stats=[]
for h in holds:
    a=h['from_sim_s'];b=h['to_sim_s'];w=[x for x in rows if a<=x[0]<=b]
    speeds=[math.dist(x[1:3],y[1:3])/(y[0]-x[0]) for x,y in zip(w,w[1:])]
    hold_stats.append(dict(step=h['step'],duration_s=b-a,drift_m=h['drift_m'],samples=len(w),max_pose_gap_s=max([y[0]-x[0] for x,y in zip(w,w[1:])] or [math.inf]),peak_speed_mps=max(speeds or [math.inf])))
road=road_polygon(ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf')
outside=sum(rectangle(row[1],row[2],row[3],.20,.15).difference(road).area>1e-6 for row in rows)
changed=[p for p,h in reg['sources'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
# 종료 helper의 구조는 원 보고서에 보존하고, 실제 남은 프로세스와 정상 exit를 별도로 대조한다.
gates=dict(completed=r.get('completed') is True,visual_start=any(s.get('source')=='visual' for s in byname('visual_started')),
    no_motion_before_prepare=bool(byname('no_motion_before_prepare')) and all(s['truth_speed']<.02 for s in byname('no_motion_before_prepare')),
    b0390_profile=(r.get('local_sensor_config') or {}).get('sha256')==hashlib.sha256((ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml').read_bytes()).hexdigest(),
    no_reset='recovery_reset' not in commands,one_resume_after_stop=commands.count('resume_start')==2,
    two_stops=commands.count('stop')==2,holds=len(hold_stats)==2 and all(h['duration_s']>=.49 and h['drift_m']<=.02 and h['max_pose_gap_s']<=.12 and h['peak_speed_mps']<=.05 for h in hold_stats),
    resume=len(resumes)==1 and resumes[0]['last']['outcome']=='permitted' and resumes[0]['sim_seconds']<=3.,
    progress=len(progress)==1 and progress[0]['progress_m']>=1.2,
    same_race=bool(byname('same_race_resumed')),body_on_road=outside==0,source_unchanged=not changed and r['source_unchanged'],
    no_remaining_process=not r.get('remaining_test_pids'),normal_exit=all(x==0 for x in r.get('exit_codes',[]) if x is not None))
# 재배치/RESET을 포함하므로 옛 "no_reset" 게이트 대신 명시 사전 조건을 평가한다.
gates.pop('no_reset')
gates['one_recovery_reset']=commands.count('recovery_reset')==1
placed=byname('placement_verified');starts=byname('press_resume_start');req=byname('requested_place')
before_place=byname('before_place')
gates['placed_pose_verified']=bool(len(placed)==len(req)==len(before_place)==1 and
    math.dist(placed[0]['truth'][1:3],(req[0]['x'],req[0]['y']))<.04 and
    abs(math.atan2(math.sin(placed[0]['truth'][3]-req[0]['yaw']),math.cos(placed[0]['truth'][3]-req[0]['yaw'])))<.05 and
    math.dist(placed[0]['truth'][1:3],before_place[0]['truth'][1:3])>1.0)
delay=(starts[-1]['truth'][0]-placed[0]['truth'][0]) if placed and starts else None
gates['immediate_request_after_placed']=delay is not None and 0<=delay<=.3
gates['bounded_settling']=len(holds)==2 and all(h['passed'] and h['duration_s']>=.5-1e-9 and h['from_stop_request_s']<=1.6 for h in holds)
gates['movement_along_new_heading']=len(progress)==1 and progress[0]['progress_m']>=1.2 and abs(progress[0]['lateral_m'])<.2
gates['same_binding']=all(s['initial_binding'].get(k) is not None and s['initial_binding'].get(k)==s['current_binding'].get(k)
   for s in byname('same_race_resumed') for k in ('session','race_id','race_epoch')) and len(byname('same_race_resumed'))==1
gates['precedence']=(commands==['resume_start','prepare_start','stop','recovery_reset','resume_start','stop'])
gates['normal_exit']=r.get('exit_codes')==[0,0]
original_windows=[s for s in steps if s['step'].endswith('_original_window')]
gates['original_windows_retained']=len(original_windows)==2 and all(s['window'] is not None for s in original_windows)
result=dict(scope='newcourse simulated judge teleport at test s12 then one START; no official coordinate, hardware/public WiFi/contact-event proof',gates=gates,passed=all(gates.values()),hold_stats=hold_stats,original_windows=original_windows,placement_to_request_s=delay,stable_ready=[h.get('stationary_ready') for h in holds],resumes=resumes,post_progress=progress,body_outside_samples=outside,pose_samples=len(rows),sources=len(reg['sources']),changed=changed,exit_codes=r.get('exit_codes'),gazebo_stop_service=r.get('gazebo_stop_service'),cleanup=r.get('auxiliary_cleanup'))
(HERE/'assessment.json').write_text(json.dumps(result,ensure_ascii=False,indent=1)+'\n',encoding='utf-8');print(json.dumps(result,ensure_ascii=True))

raise SystemExit(0 if result["passed"] else 1)
