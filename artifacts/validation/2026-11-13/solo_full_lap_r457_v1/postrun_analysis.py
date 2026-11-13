"""r457 원 기록 집계. 제어기·수치 커널·새 주행 실행 없음."""
import collections
import hashlib
import json
import math
from pathlib import Path
import statistics

A=Path(__file__).resolve().parent;P=A/'run1';O=A/'analysis_v1';O.mkdir(exist_ok=True)
read=lambda p:json.loads(p.read_bytes())
def lines(p):
    with p.open(encoding='utf8') as f:return [json.loads(s) for s in f if s.strip()]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write(n,v):(O/n).write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
def count(v):return dict(collections.Counter(v))
def small(v):return {k:x for k,x in v.items() if x is None or isinstance(x,(str,int,float,bool))}
def stats(v):
    v=sorted(v)
    return dict(n=len(v),median=statistics.median(v),p95=v[max(0,math.ceil(.95*len(v))-1)],maximum=max(v)) if v else dict(n=0)
t=lines(P/'follow_decisions.jsonl');traj=lines(P/'trajectory.jsonl');r=read(P/'report.json');act=read(P/'actuation_trace.json')
by=lambda k:[e for e in t if e['kind']==k]
sub=by('runtime_plan_submitted');done=by('runtime_plan_completed');calls=by('call');pub=by('route_publication')
key=lambda e:(e['data']['generation'],e['data']['sequence'])
sm={key(e):e for e in sub};dm={key(e):e for e in done}
normal=lambda e:e['data']['output'][2].get('normal',{})
current=[e for e in calls if e['data'].get('current_rebase')]
ckey=lambda e:(e['data']['current_rebase']['compute']['generation'],e['data']['current_rebase']['compute']['sequence'])
pkey=lambda e:(e['data']['compute']['generation'],e['data']['compute']['sequence'])
new_reason='guard_curve_current_approach_observed_with_mask_budget'
newplans={k:e for k,e in dm.items() if normal(e).get('selector_reason')==new_reason}
cur_index=collections.defaultdict(list)
for e in current:cur_index[e['data']['current_rebase']['output']['reference_s']].append(e)
joined=[]
for e in pub:
    d=e['data'];matches=[c for c in cur_index[d['reference_s']] if c['seq']<e['seq']]
    c=max(matches,key=lambda x:x['seq']) if matches else None
    joined.append(dict(publication_seq=e['seq'],publication_line=e['seq']+1,**d,
        current_trace_seq=c['seq'] if c else None,current_key=list(ckey(c)) if c else None,
        plan_key=list(pkey(c)) if c else None,plan_selector_reason=normal(dm[pkey(c)]).get('selector_reason') if c and pkey(c) in dm else None,
        current_reason=c['data']['current_rebase']['output'].get('reason') if c else None,
        checked_speed_agrees=(c is not None and abs(c['data']['current_rebase']['output']['speed_mps']-d['checked_speed_mps'])<1e-9) if 'checked_speed_mps' in d else None))
write('publication_current_plan_join.json',joined)
cross=next(x for x in traj if x['progress_m']>20.18)
area=[x for x in traj if 19<=x['progress_m']<=21]
window=(area[0]['t'],area[-1]['t'])
newcur=[e for e in current if pkey(e) in newplans]
newpub=[e for e in joined if e['plan_selector_reason']==new_reason]
lastpos=max(i for i,x in enumerate(traj) if x['drive_speed_mps']>0)
firstzero=traj[lastpos+1]
faults=[e for e in current if e['data']['current_rebase']['output'].get('reason')=='route_current_large_motion_masked_toward_right']
firstfault=min(faults,key=lambda e:e['data']['current_rebase']['output']['reference_s'])
f=firstfault['data'];fk=pkey(firstfault);fd=dm[fk];fs=sm[fk]
fault_input=f['current_rebase']['input'];fault_output=f['current_rebase']['output'];fault_compute=f['current_rebase']['compute']
zero_plans=[e for e in done if traj[lastpos]['t']<=sm[key(e)]['data']['input']['reference_s']<=fault_output['reference_s'] and e['data']['output'][0]==0]
zero_chain=[dict(plan_key=list(key(e)),reference_s=sm[key(e)]['data']['input']['reference_s'],
    submission_trace_seq=sm[key(e)]['seq'],submission_line_1based=sm[key(e)]['seq']+1,
    completion_trace_seq=e['seq'],completion_line_1based=e['seq']+1,output=e['data']['output'][:2],
    reason=normal(e).get('reason'),selector_reason=normal(e).get('selector_reason'),
    recorded_current_call_count=sum(pkey(c)==key(e) for c in current)) for e in zero_plans]
selected=[x for e in zero_plans for x in (sm[key(e)],e)]+[fs,fd,firstfault]
with (O/'first_stall_raw_events.jsonl').open('w',encoding='utf8') as out:
    for e in selected:out.write(json.dumps(e,ensure_ascii=False)+'\n')
write('first_stall_anchor.json',dict(
    first_persistent_zero_trajectory=firstzero,last_positive_trajectory=traj[lastpos],
    initial_plan_zero_chain=zero_chain,
    first_rejection=dict(trace_seq=firstfault['seq'],line_1based=firstfault['seq']+1,
        plan_key=list(fk),current_key=list(ckey(firstfault)),
        plan_submission_trace_seq=fs['seq'],plan_submission_line_1based=fs['seq']+1,
        plan_completion_trace_seq=fd['seq'],plan_completion_line_1based=fd['seq']+1,
        input=small(fault_input),output=small(fault_output),current_compute=small(fault_compute),
        output_diagnostics={k:v for k,v in fault_output.items() if k not in small(fault_output) and k not in ('path','scene','raw_scene')},
        plan_output=fd['data']['output'][:2],plan_normal=small(normal(fd))),
    paths=dict(original='run1/follow_decisions.jsonl',plan_input=f'line {fs["seq"]+1}: data.input',
        current_input=f'line {firstfault["seq"]+1}: data.current_rebase.input',
        current_worker_full_input=f'line {firstfault["seq"]+1}: data.current_rebase.compute.input',
        current_worker_plan=f'line {firstfault["seq"]+1}: data.current_rebase.compute.plan',
        selected_raw='analysis_v1/first_stall_raw_events.jsonl')))

def summarize_window(lo,hi):
    ds=[e for e in done if lo<=sm[key(e)]['data']['input']['reference_s']<=hi]
    cs=[e for e in current if lo<=e['data']['current_rebase']['output']['reference_s']<=hi]
    ps=[e for e in joined if lo<=e['at_s']<=hi]
    return dict(sim_window=[lo,hi],plan_count=len(ds),plan_selector_reasons=count(normal(e).get('selector_reason') for e in ds),
        current_count=len(cs),current_reasons=count(e['data']['current_rebase']['output'].get('reason') for e in cs),
        parent_publications=len(ps),parent_positive=sum(e['speed_mps']>0 for e in ps),
        new_reason_plan_count=sum(key(e) in newplans for e in ds),new_reason_current_count=sum(pkey(e) in newplans for e in cs),
        new_reason_parent_positive=sum(e['plan_selector_reason']==new_reason and e['speed_mps']>0 for e in ps))

maskrows=[]
for k,e in newplans.items():
    n=normal(e);g=n['selector_guard_candidate'];cs=[c for c in newcur if pkey(c)==k];ps=[p for p in newpub if p['plan_key']==list(k)]
    maskrows.append(dict(plan_key=list(k),trace_seq=e['seq'],line_1based=e['seq']+1,input_reference_s=sm[k]['data']['input']['reference_s'],
        measured_speed_mps=sm[k]['data']['input']['speed_mps'],target_mps=e['data']['output'][0],
        region=g.get('region'),current_approach=g.get('current_approach'),
        current_trace_seqs=[c['seq'] for c in cs],current_keys=[list(ckey(c)) for c in cs],
        current_reasons=count(c['data']['current_rebase']['output']['reason'] for c in cs),
        parent_publication_seqs=[p['publication_seq'] for p in ps],parent_positive=sum(p['speed_mps']>0 for p in ps)))
write('r459_mask_budget_connections.json',maskrows)
planwall=[1000*(e['data']['completed_wall_s']-e['data']['began_wall_s']) for e in done]
uniquecurrent={ckey(e):e for e in current}
curwall=[1000*(e['data']['current_rebase']['compute']['completed_wall_s']-e['data']['current_rebase']['compute']['began_wall_s']) for e in uniquecurrent.values()]
first_new=min(sm[k]['data']['input']['reference_s'] for k in newplans) if newplans else None
last_new=max(sm[k]['data']['input']['reference_s'] for k in newplans) if newplans else None
after=[x for x in traj if last_new is not None and x['t']>last_new and x['t']<firstzero['t']]
accel={str(limit):next(({k:x[k] for k in ('t','progress_m','drive_speed_mps','safe_speed_mps','ground_speed_mps')} for x in after if x['drive_speed_mps']>limit and x['safe_speed_mps']>limit and x['ground_speed_mps']>limit),None) for limit in (.35,.75,1.)}
diag=read(P/'actuation_diagnostics.json')
summary=dict(
    run_count=1,run_exit=int((A/'execution_v1/run.exit').read_text()),original_verdict={k:r.get(k) for k in ('passed','harness_valid','run_valid','abort','error','gates')},
    progress=r['evaluation']['full_course_progress'],case_metrics=r['evaluation']['case_metrics'],crossed_old_stall=cross,
    r459=dict(selector_reason=new_reason,plan_count=len(newplans),input_reference_range=[first_new,last_new],current_count=len(newcur),
        current_reasons=count(e['data']['current_rebase']['output']['reason'] for e in newcur),
        parent_positive_publications=sum(e['speed_mps']>0 for e in newpub),
        region_ranges={name:[min(x['region'][name] for x in maskrows),max(x['region'][name] for x in maskrows)] for name in ('masked_tail_extent_m','unobserved','masked_front_samples')},
        publication_join_unmatched=sum(e['current_key'] is None for e in joined),publication_speed_disagreements=sum(e['checked_speed_agrees'] is False for e in joined),
        acceleration_after_new_reason_before_final_stop=accel,
        scope='same-run plan/current/reference-key publication association; actor removal and policy change are confounded, no causal speedup claim'),
    around_20_18m=dict(progress_window_m=[19,21],whole=summarize_window(*window),before=summarize_window(window[0],cross['t']),after=summarize_window(cross['t'],window[1])),
    final_stall=dict(first_persistent_zero_sim_s=firstzero['t'],first_rejection_sim_s=fault_output['reference_s'],
        initial_plan_zero_chain=zero_chain,
        plan_key=list(fk),current_key=list(ckey(firstfault)),first_call_trace_seq=firstfault['seq'],
        current_rejection_count=len(faults),range=[min(e['data']['current_rebase']['output']['reference_s'] for e in faults),max(e['data']['current_rebase']['output']['reference_s'] for e in faults)],
        plan_selector_reasons=count(normal(dm[pkey(e)]).get('selector_reason') for e in faults),
        positive_plan_count=sum(dm[pkey(e)]['data']['output'][0]>0 for e in faults),
        parent_positive_after_first_zero=sum(e['at_s']>=firstzero['t'] and e['speed_mps']>0 for e in joined),
        anchor='analysis_v1/first_stall_anchor.json'),
    complete_trace=dict(kinds=count(e['kind'] for e in t),submissions=len(sub),completions=len(done),unique_submissions=len(sm),unique_completions=len(dm),
        unmatched_submission_keys=sorted(sm.keys()-dm.keys()),unmatched_completion_keys=sorted(dm.keys()-sm.keys()),
        wall_submit_disagreements=sum(sm[k]['data']['submitted_wall_s']!=dm[k]['data']['submitted_wall_s'] for k in sm.keys()&dm.keys()),
        contiguous_seq=[e['seq'] for e in t]==list(range(t[0]['seq'],t[-1]['seq']+1)),footer=t[-1]),
    computation=dict(plan_wall_ms=stats(planwall),current_wall_ms=stats(curwall),wall_elapsed_s=r['wall_elapsed_s'],
        sim_green_s=r.get('green_observed_sim_s'),stop_request_sim_s=r.get('stop_request_sim_s'),scope='observed WSL simulation workload, not Jetson; wall milliseconds not sim milliseconds'),
    guard_mcu=dict(reasons=count(x['reason'] for x in act),command_expired_samples=sum(x['reason']=='command_expired' for x in act),diagnostics=diag,
        final_mcu=act[-1],safety_reasons=count(x['safety_reason'] for x in traj)),
    stop=dict(checks=r['stop_checks'],final_output=r['actuation']['final_output'],early_stop=read(A/'early_stop.json'),sequence=read(A/'stop_sequence.json')),
    contact_surface=dict(no_actor=r['evaluation']['no_actor'],wall_collision_samples=r['wall_collision_samples'],dense_wall=r['evaluation']['dense_wall_overlap'],
        road=r['evaluation']['road'],wheel_centers=r['evaluation']['wheel_centers'],contact_evidence=r['contact_evidence']),
    cleanup=dict(shutdown=r['shutdown_audit'],shutdown_clean=r['shutdown_clean'],process_exit_codes=r['process_exit_codes'],owned=read(A/'execution_v1/owned_processes_after.json')),
    integrity=read(A/'integrity.json'),scope='one early-stopped actor0 simulation; no repeat/new runtime or hardware claim')
write('summary.json',summary)
sealed=[p for p in P.iterdir() if p.is_file()]+[A/n for n in ('prepared.json','sources.zip','registered.json','integrity.json','runtime_audit.json','stop_sequence.json','root_execution_review.json','early_stop.json')]
sealed += [p for p in (A/'execution_v1').iterdir() if p.is_file() and p.suffix in ('.json','.log','.exit')]
write('original_evidence_sha256.json',{p.relative_to(A).as_posix():sha(p) for p in sealed})
print(json.dumps({k:summary[k] for k in ('r459','around_20_18m','final_stall','computation')},ensure_ascii=False,indent=2))
