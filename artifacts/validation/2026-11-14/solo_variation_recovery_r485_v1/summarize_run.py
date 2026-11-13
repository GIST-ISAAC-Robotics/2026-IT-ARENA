"""원 단일 주행을 봉인하고 첫 보호 정지 주변의 원 기록만 발췌한다. 재생/발행 없음."""
from pathlib import Path
import collections,hashlib,json
A=Path(__file__).resolve().parent;C=A/'cases/start_left20_yaw_plus3';R=C/'run1';F=A/'failure_inputs'
def read(p):return json.loads(p.read_bytes())
def write(p,v):p.write_text(json.dumps(v,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for chunk in iter(lambda:f.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()

def main():
    F.mkdir(exist_ok=False)
    report=read(R/'report.json');trace=[json.loads(s) for s in (R/'trajectory.jsonl').open()]
    blocked=[r for r in trace if r.get('progress_m',0)>39 and r.get('drive_speed_mps',0)>0
        and r.get('safe_speed_mps')==0 and r.get('safety_reason')=='lateral_metric_outside_trial']
    first=blocked[0];low,high=49.5,50.65
    # 좌표/자세/진행률은 평가 기록이며 제어 재생 입력이 아니다.
    write(F/'evaluation_observation.json',dict(first_sample=first,
        scope='evaluation/diagnosis only; truth pose and progress must not feed control',
        nearby=[r for r in trace if low<=r['t']<=high]))
    scans=[]
    for line,s in enumerate((R/'scans_received.jsonl').open(),1):
        d=json.loads(s)
        if low<=d['stamp_s']<=high:scans.append(dict(source_line=line,data=d))
    write(F/'raw_scans.json',scans)
    mcu=[d for d in read(R/'actuation_trace.json') if low*1e6<=d.get('now_us',0)<=high*1e6]
    write(F/'mcu_feedback_and_targets.json',mcu)
    statuses=[]
    for line,s in enumerate((R/'autonomy_status.jsonl').open(),1):
        d=json.loads(s)
        if low<=d.get('sim_time_s',-1)<=high:statuses.append(dict(source_line=line,data=d))
    write(F/'autonomy_commands.json',statuses)
    selected=[];summary=[];headers={};kind_count=collections.Counter();positive_plan=positive_current=published=0
    for line,s in enumerate((R/'follow_decisions.jsonl').open(),1):
        row=json.loads(s);kind=row['kind'];d=row['data'];kind_count[kind]+=1
        if kind in ('runtime_evaluator_header','runtime_factory_policy'):headers[kind]=row
        ref=d.get('reference_s',d.get('input',{}).get('reference_s'))
        if kind=='runtime_plan_completed':
            n=d['output'][2].get('normal') or {};ref=n.get('reference_stamp_s')
            if ref is not None and ref>=50.2 and d['output'][0]>0:positive_plan+=1
        if kind=='call' and d.get('current_rebase'):
            cur=d['current_rebase']['output']
            if cur.get('reference_s',-1)>=50.2 and cur.get('speed_mps',0)>0:positive_current+=1
        if kind=='route_publication' and ref is not None and ref>=50.2 and d.get('speed_mps',0)>0:published+=1
        if ref is not None and low<=ref<=high:
            selected.append(dict(source_line=line,record=row))
            n=(d.get('output') or [None,None,{}])[2].get('normal',{}) if kind in ('call','runtime_plan_completed') else {}
            c=d.get('current_rebase',{});meta=c.get('compute',{});out=c.get('output',{})
            summary.append(dict(source_line=line,seq=row['seq'],kind=kind,reference_s=ref,
                generation=d.get('generation'),sequence=d.get('sequence'),
                plan_speed=d.get('output',[None])[0] if kind in ('call','runtime_plan_completed') else None,
                plan_reason=n.get('reason'),plan_partial=(n.get('partial_observation') or {}).get('reason'),
                current_key=[meta.get('generation'),meta.get('sequence')] if c else None,
                current_speed=out.get('speed_mps'),current_reason=out.get('reason'),
                published_speed=d.get('speed_mps') if kind=='route_publication' else None,
                published_steering=d.get('steering_rad') if kind=='route_publication' else None))
    with (F/'follow_records.jsonl').open('x',encoding='utf-8') as f:
        for row in selected:f.write(json.dumps(row,ensure_ascii=False)+'\n')
    write(F/'runtime_headers.json',headers);write(F/'record_index.json',summary)
    before=[x for x in mcu if x.get('now_us',0)<=50.22e6]
    positive=[x for x in before if x.get('target_speed_mps',0)>0]
    zero=next(x for x in mcu if x.get('now_us',0)>=50.2e6 and x.get('target_speed_mps')==0)
    index=dict(window_s=[low,high],first_guard_zero_trajectory_s=first['t'],
        first_guard_zero_progress_m=first['progress_m'],raw_scan_records=len(scans),follow_records=len(selected),
        last_positive_mcu_before_zero=positive[-1],first_mcu_zero=zero,
        prior_safe_steering_candidate=positive[-1]['steering_rad'],
        guard_state_exact=False,
        unavailable=['full /safety/status partial_observation was not saved in this run trace',
            'exact independent guard timer reference/command receipt/previous_safe_steering callback order',
            'independent guard PartialObservationPolicy/StructureAware internal state'],
        usable='Original plan/current records retain raw scan, signed wheel/gyro history, timestamps, epoch, command. MCU targets are observed downstream commands, not an exact guard-state snapshot.',
        scope='first-failure saved-input handoff; no whole-guard replay or safety-policy proof',
        sources={p.name:dict(path=str(p),sha256=sha(p)) for p in
            (R/'follow_decisions.jsonl',R/'scans_received.jsonl',R/'actuation_trace.json',R/'autonomy_status.jsonl',R/'trajectory.jsonl')})
    write(F/'README.json',index)
    observations=[json.loads(s) for s in (C/'execution_v1/observations.jsonl').open()]
    stop_trigger=next(x for x in observations if x.get('stop_required'))
    result=dict(passed=report['passed'],full_lap=report['evaluation']['gates']['full_lap'],
        progress_m=report['evaluation']['full_course_progress']['progress_m'],lap_length_m=44.64,
        max_progress_m=max(r.get('progress_m',0) for r in trace),
        longest_stationary_sim_s=report['evaluation']['case_metrics']['longest_stationary_s'],
        green_sim_s=report['green_observed_sim_s'],first_motion_sim_s=report['evaluation']['case_metrics']['first_motion_sim_s'],
        first_guard_zero=dict(t=first['t'],progress_m=first['progress_m'],drive_speed_mps=first['drive_speed_mps'],safe_speed_mps=first['safe_speed_mps'],reason=first['safety_reason']),
        positive_drive_guard_zero_samples=len(blocked),plan_positive_records_after_50p2=positive_plan,
        current_positive_records_after_50p2=positive_current,positive_route_publications_after_50p2=published,
        stop_trigger=stop_trigger,stop=report['stop'],abort=report['abort'],
        wall_collision_samples=report['wall_collision_samples'],dense_wall_overlap_samples=report['evaluation']['dense_wall_overlap']['overlap_samples'],
        max_off_wheel_centers=report['evaluation']['wheel_centers']['max_off_wheels'],
        deadline_diagnostics=len(read(R/'actuation_diagnostics.json')),
        sensor_only_inputs=report['sensor_only_inputs'],safety_sensor_only_inputs=report['safety_sensor_only_inputs'],
        shutdown_clean=report['shutdown_clean'],shutdown_children=report['shutdown_audit']['clean_child_count'],
        source_unchanged=report['source_unchanged'],integrity=read(C/'integrity.json'),
        owned_processes=read(C/'execution_v1/owned_processes_after.json'),
        frozen_input_count=len(read(A/'execution_prepared.json')['frozen_inputs']),
        wall_elapsed_s=report['wall_elapsed_s'],follow_kinds=dict(kind_count),
        actual_run_count=1,analysis_started_new_run=False,repeat_performed=False,wsl_returned=True,
        conclusion='r481 plan/current produce positive commands; independent safety lateral veto causes new sustained stop; original failed verdict retained')
    write(A/'execution_summary.json',result)
    print(json.dumps({k:result[k] for k in ('progress_m','longest_stationary_sim_s','first_guard_zero',
        'plan_positive_records_after_50p2','current_positive_records_after_50p2','positive_route_publications_after_50p2',
        'wall_collision_samples','shutdown_clean','source_unchanged','frozen_input_count')}))

if __name__=='__main__':main()
