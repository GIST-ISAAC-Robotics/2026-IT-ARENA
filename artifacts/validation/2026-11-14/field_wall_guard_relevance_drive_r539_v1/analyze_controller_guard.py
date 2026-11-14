"""r530 종료 후 기록 집계. 동결 실행 소스/원 보고서는 변경하지 않는다."""
import collections,hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;R=A/'run1'
read=lambda p:json.loads(Path(p).read_bytes())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    report=read(R/'report.json');before=sha(R/'report.json');start=report['case_window']['green_sim_s'];end=report['case_window']['end_sim_s']
    plans=[];publications=[];pending=None
    with (R/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row=json.loads(line);data=row.get('data') or {}
            if row['kind'] in ('runtime_plan_completed','call'):
                out=data['output'];meta=out[2];normal=meta.get('normal') or {}
                item=dict(seq=row['seq'],speed_mps=out[0],handoff_state=meta.get('handoff_state'),behavior=normal.get('behavior'),
                    selector_mode=normal.get('selector_mode'),reason=normal.get('reason'),helper=bool(normal.get('current_wall_reacquire')))
                if row['kind']=='runtime_plan_completed':plans.append(item)
                else:
                    item['command_reference_s']=(data.get('delivery') or {}).get('command_reference_s');pending=item
            elif row['kind']=='route_publication':
                matched=pending is not None and pending['command_reference_s']==data.get('reference_s')
                publications.append(dict(seq=row['seq'],**data,matched_preceding_call=matched,call=pending if matched else None));pending=None
            elif row['kind']=='reset':pending=None
    def normal(item):return item['handoff_state']=='FOLLOW' and item['reason']=='local_follow' and not item['helper'] and item['speed_mps']>0
    helper_pubs=[p for p in publications if p['call'] and p['call']['helper']]
    normal_pubs=[p for p in publications if p['call'] and normal(p['call'])]
    rows=[];selected={};partial_counts=collections.Counter();memory_counts=collections.Counter();ray_counts=collections.Counter();zero_positive=[]
    with (R/'safety_status.jsonl').open(encoding='utf-8') as stream:
        for number,line in enumerate(stream,1):
            row=json.loads(line);d=row['data'];reference=d.get('reference_s')
            if reference is None or not start<=reference<=end:continue
            partial=d.get('partial_observation') or {};memory=partial.get('retained_memory_command') or {};ray=memory.get('current_ray_clearance') or {}
            partial_counts[partial.get('reason')]+=1
            if memory:memory_counts[memory.get('reason')]+=1;ray_counts[ray.get('reason')]+=1
            compact=dict(line=number,reference_s=reference,receipt_monotonic_s=row['receipt_monotonic_s'],reason=d.get('reason'),safe_speed_mps=d.get('safe_speed_mps'),
                partial_reason=partial.get('reason'),memory_reason=memory.get('reason'),requested_speed_mps=memory.get('requested_speed_mps'),
                memory_cap_mps=memory.get('speed_cap_mps'),normal_cap_restored=memory.get('normal_cap_restored'),retained_points=memory.get('retained_points'),
                original_reference_s=memory.get('original_reference_s'),ray_reason=ray.get('reason'),active_points_ray_cleared=memory.get('active_points_ray_cleared'),
                current_raw_clearance_m=memory.get('current_raw_clearance_m'),current_memory_clearance_m=memory.get('current_memory_clearance_m'))
            rows.append(compact)
            if d.get('reason')=='known_obstacle_latched' and 'first_latch' not in selected:selected['first_latch']=(number,row)
            if (d.get('safe_speed_mps') or 0)>0 and 'first_positive_guard' not in selected:selected['first_positive_guard']=(number,row)
            if memory.get('normal_cap_restored') and 'first_memory_normal_cap_restored' not in selected:selected['first_memory_normal_cap_restored']=(number,row)
            if (memory.get('requested_speed_mps') or 0)>0 and (d.get('safe_speed_mps') or 0)<=0:zero_positive.append(compact)
            if memory.get('reason')=='retained_memory_current_sweep_crawl' and (memory.get('requested_speed_mps') or 0)>.1:
                selected['last_requested_above_crawl_cap']=(number,row)
    intervals=[];active=None
    for row in rows:
        capped=row['memory_reason']=='retained_memory_current_sweep_crawl' and (row['requested_speed_mps'] or 0)>.100001 and (row['memory_cap_mps'] or 0)<=.100001
        if capped:
            if active is None:active=dict(first_reference_s=row['reference_s'],last_reference_s=row['reference_s'],first_line=row['line'],last_line=row['line'],samples=1)
            else:active.update(last_reference_s=row['reference_s'],last_line=row['line'],samples=active['samples']+1)
        elif active is not None:intervals.append(active);active=None
    if active is not None:intervals.append(active)
    for item in intervals:item['sample_span_s']=item['last_reference_s']-item['first_reference_s']
    extracted=A/'guard_event_extract';extracted.mkdir()
    paths={}
    for name,(line,row) in selected.items():
        path=extracted/(name+'.json');path.write_text(json.dumps(dict(source_file='run1/safety_status.jsonl',source_line=line,raw=row),indent=2)+'\n',encoding='utf-8')
        paths[name]=dict(path=path.relative_to(A).as_posix(),sha256=sha(path),line=line,reference_s=row['data'].get('reference_s'))
    trace=read(R/'trace.json');active_trace=[r for r in trace if start<=r['t']<=end]
    result=dict(scope='Stored trace only, no replay; command publication versus independent guard output separated',
        original_passed=report['passed'],original_report_sha256=before,
        correction_to_generic_collection='positive_normal_follow_plans there counts normal.selector_mode FOLLOW. This run selector_mode GUARD is independent of handoff FOLLOW; do not use that counter for normal driving recovery.',
        normal_plan_definition='handoff_state FOLLOW, normal.reason local_follow, positive plan speed, no current_wall_reacquire helper',
        plans=len(plans),helper_plans=sum(p['helper'] for p in plans),normal_positive_plans=sum(normal(p) for p in plans),zero_plans=sum(p['speed_mps']==0 for p in plans),
        publications=len(publications),unmatched_publications=sum(not p['matched_preceding_call'] for p in publications),
        helper_publications=helper_pubs,normal_positive_publications=len(normal_pubs),first_normal_publication=normal_pubs[0] if normal_pubs else None,
        guard_partial_reasons=dict(partial_counts),guard_memory_reasons=dict(memory_counts),guard_ray_reasons=dict(ray_counts),
        current_guard_zero_with_positive_requested_memory_command=zero_positive,
        low_guard_cap_intervals=intervals,guard_interval_scope='Consecutive saved guard status sample spans, not continuous-time proof or sum of actual stationary durations',
        guard_rows=rows,selected_raw_events=paths,
        raw_metrics=report['evaluation']['case_metrics'],raw_final_progress_m=report['evaluation']['progress']['at_window_end_m'],
        raw_final_gap_m=report['evaluation']['progress']['sampled_gap_at_window_end_m'],
        peak_sampled_ground_speed_mps=max(r['ground_speed_mps'] for r in active_trace),
        trajectory_positive_drive_zero_safe_rows=[{k:r.get(k) for k in ('t','drive_speed_mps','safe_speed_mps','safety_reason')} for r in active_trace if r['drive_speed_mps']>0 and r['safe_speed_mps']==0],
        source_clock_contract='reference_s and trace t are simulation seconds; receipt_monotonic_s is evaluator host time',
        raw_report_unchanged=sha(R/'report.json')==before)
    (A/'controller_guard_supplement.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:result[k] for k in ('plans','helper_plans','normal_positive_plans','zero_plans','publications','unmatched_publications','normal_positive_publications','low_guard_cap_intervals','raw_final_progress_m','raw_final_gap_m')}))


if __name__=='__main__':main()
