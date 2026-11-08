"""원시 시각·명령을 보존하고 새 코스 운영과 실제 상태를 평가한다. 평가 전용."""
from pathlib import Path
import bisect,hashlib,json,math
from g1_traffic_eval import rectangle,contact_connection_valid
from surface_contact_eval import SurfaceMap,evaluate_pose
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3];run=HERE/'run1'
read=lambda p:json.loads(p.read_bytes())
r=read(run/'report.json');reg=read(HERE/'registered.json');steps=r.get('steps',[])
byname=lambda n:[s for s in steps if s['step']==n]
commands=[s['step'][6:] for s in steps if s['step'].startswith('press_')]
rows=[json.loads(l) for l in (run/'ego_poses.jsonl').read_bytes().splitlines()]
holds=byname('first_stop_hold')+byname('final_stop_hold');resumes=byname('resume_closed');progress=byname('post_resume_progress')
binding=byname('same_race_resumed');visual=byname('visual_started');end=holds[-1]['to_sim_s'] if holds else rows[-1][0]
start=visual[0]['truth'][0] if visual else rows[0][0]
window=[p for p in rows if start<=p[0]<=end]
actors=[];bad=0
for line in (run/'actor_poses.jsonl').read_bytes().splitlines():
    try:d=json.loads(line)
    except ValueError:bad+=1;continue
    h=d.get('header',{}).get('stamp',{});t=float(h.get('sec',0))+float(h.get('nsec',0))*1e-9
    for p in d.get('pose',[]):
        if p.get('name')!='eval_g1b_actor':continue
        pos,q=p['position'],p['orientation'];yaw=math.atan2(2*(q.get('w',1)*q.get('z',0)+q.get('x',0)*q.get('y',0)),1-2*(q.get('y',0)**2+q.get('z',0)**2))
        if not actors or t>actors[-1][0]:actors.append((t,pos['x'],pos['y'],yaw))
contacts=[];contact_messages=0
for line in (run/'contacts.jsonl').read_bytes().splitlines():
    try:d=json.loads(line)
    except ValueError:bad+=1;continue
    contact_messages+=1
    h=d.get('header',{}).get('stamp',{});t=float(h.get('sec',0))+float(h.get('nsec',0))*1e-9
    for c in d.get('contact',[]):
        pair=(c['collision1']['name'],c['collision2']['name'])
        if start<=t<=end and any('arena_car' in n for n in pair):contacts.append(dict(t=t,pair=pair))
index=[p[0] for p in actors];gaps=[];coverage=True;max_pose_age=0.
for row in window:
    k=bisect.bisect_right(index,row[0])-1
    if k<0 or row[0]-index[k]>.05+1e-9:coverage=False;continue
    a=actors[k];max_pose_age=max(max_pose_age,row[0]-a[0]);gaps.append(rectangle(*row[1:4],.2,.15).distance(rectangle(*a[1:4],.2,.15)))
surface=SurfaceMap.from_world(run/'runtime_world.sdf')
max_off=wall_samples=body_out=0
for row in window:
    values=[evaluate_pose(surface,*row[1:4],a) for a in (0.,-.45,.45)]
    max_off=max(max_off,*(v['off_counts']['any'] for v in values))
    wall_samples+=any(v['wall_wheels'] for v in values)
    body_out+=values[0]['body_outside_road_area_m2']>1e-6
changed=[p for p,h in reg['sources'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest()!=h]
probe=byname('contact_source');conn=dict(topic='/eval/g1b_actor/contact',output=probe[0]['output'],returncode=probe[0]['returncode'],reader_alive_at_graph_audit=probe[0]['reader_alive_at_graph_audit']) if probe else {}
hold_stats=[]
for h in holds:
    w=[p for p in rows if h['from_sim_s']<=p[0]<=h['to_sim_s']]
    dt=[b[0]-a[0] for a,b in zip(w,w[1:])]
    speeds=[math.dist(a[1:3],b[1:3])/d for a,b,d in zip(w,w[1:],dt)]
    hold_stats.append(dict(duration_s=w[-1][0]-w[0][0],peak_mps=max(speeds or [math.inf]),max_gap_s=max(dt or [math.inf]),drift_m=max(math.dist(w[0][1:3],p[1:3]) for p in w),from_stop_request_s=h['from_stop_request_s']))
gates=dict(completed=r.get('completed') is True,visual_start=len(visual)==1 and visual[0]['source']=='visual',
    no_reset='recovery_reset' not in commands,command_order=commands==['resume_start','prepare_start','stop','resume_start','stop'],
    dense_stop=len(hold_stats)==2 and all(h['duration_s']>=.5-1e-9 and h['peak_mps']<=.05 and h['max_gap_s']<=.12 and h['drift_m']<=.02 and h['from_stop_request_s']<=1.6 for h in hold_stats),
    resume=len(resumes)==1 and resumes[0]['last']['outcome']=='permitted' and resumes[0]['sim_seconds']<=3.,
    progress=len(progress)==1 and progress[0]['progress_m']>=1.2,
    same_binding=len(binding)==1 and all(binding[0]['initial_binding'].get(k) is not None and binding[0]['initial_binding'][k]==binding[0]['current_binding'].get(k) for k in ('session','race_id','race_epoch')),
    actor_pose_coverage=coverage and len(gaps)>100 and max([b[0]-a[0] for a,b in zip(actors,actors[1:])] or [math.inf])<=.05,
    contact_source=bool(probe and contact_connection_valid(conn)),no_contact=not contacts,
    positive_sample_gap=bool(gaps) and min(gaps)>0.,raw_parse=bad==0,
    wheels=max_off<=2 and wall_samples==0,source_unchanged=not changed and r['source_unchanged'],
    no_remaining_process=not r.get('remaining_test_pids'),normal_exit=r.get('exit_codes')==[0,0,0,0])
# Contact topic may emit only when touching. Publisher/subscriber graph and actual actor pose coverage accompany an empty contact file.
out=dict(scope='stationary front actor STOP START without RESET; no moving-target or hardware proof',gates=gates,passed=all(gates.values()),commands=commands,holds=hold_stats,resumes=resumes,progress=progress,
    contact_messages=contact_messages,contacts=contacts,actor_pose_samples=len(actors),ego_pose_samples=len(window),actor_pose_max_age_s=max_pose_age,min_sample_gap_m=min(gaps) if gaps else None,
    max_off_wheels_three_steer_samples=max_off,wall_wheel_samples=wall_samples,body_outside_samples=body_out,steering_scope='0,+/-.45 rad discrete evaluation; no measured joint trace',
    sources=len(reg['sources']),changed=changed,exit_codes=r.get('exit_codes'),remaining=r.get('remaining_test_pids'))
(HERE/'assessment.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf-8');print(json.dumps(out,ensure_ascii=True))
raise SystemExit(0 if out['passed'] else 1)
