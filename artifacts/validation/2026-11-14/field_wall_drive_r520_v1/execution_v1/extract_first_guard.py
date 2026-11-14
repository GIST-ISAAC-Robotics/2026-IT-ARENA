"""r520 최초 guard latch/재획득 출력의 저장 원문을 발췌한다. 제어·guard 재실행은 하지 않는다."""
import collections
import hashlib
import json
import math
from pathlib import Path

A=Path(__file__).resolve().parent.parent;RUN=A/'run1';OUT=A/'failure_inputs'
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()


def lines(path):
    raw=path.read_bytes().splitlines(keepends=True)
    return raw,[json.loads(line) for line in raw]


def save(path,value):
    with path.open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')


def helper(meta):
    normal=meta.get('normal',{})
    return meta.get('current_wall_reacquire') or normal.get('current_wall_reacquire')


def front_summary(row,motion):
    pts=[]
    for i,(r,angle) in enumerate(zip(row['ranges'],row['angles_rad'])):
        if not isinstance(r,(int,float)) or not math.isfinite(r) or r<=0:continue
        angle+=motion['lidar_yaw_rad']
        x=motion['lidar_x_m']+r*math.cos(angle);y=motion['lidar_y_m']+r*math.sin(angle)
        if 0<=x<=1.2 and abs(y)<=.35:pts.append(dict(ray_index=i,x_m=x,y_m=y,range_m=r,acquired_at_s=row['acquired_at_s'][i]))
    compact=dict(stamp_s=row['stamp_s'],acquisition_first_s=min(row['acquired_at_s']),acquisition_last_s=max(row['acquired_at_s']),
        source_frame_id=row['source_frame_id'],representation=row['representation'],raw_count=len(row['ranges']),
        finite_count=sum(isinstance(r,(int,float)) and math.isfinite(r) for r in row['ranges']),
        inspection_front_count=len(pts),inspection_front_min_x_m=min((p['x_m'] for p in pts),default=None),
        inspection_front_xy_bounds=None if not pts else [[min(p[k] for p in pts) for k in ('x_m','y_m')],[max(p[k] for p in pts) for k in ('x_m','y_m')]])
    return compact,pts


def main():
    OUT.mkdir(exist_ok=False)
    report=read(RUN/'report.json');report_hash=sha(RUN/'report.json')
    trace=read(RUN/'trace.json')
    index=next(i for i,r in enumerate(trace) if r['safety_reason']=='known_obstacle_latched')
    latch_time=trace[index]['t'];previous=trace[index-1]
    fraw,records=lines(RUN/'follow_decisions.jsonl')
    calls=[r for r in records if r['kind']=='call'];plans=[r for r in calls if r['data']['compute']['stage']=='plan']
    pubs=[r for r in records if r['kind']=='route_publication']
    header=next(r for r in records if r['kind']=='header')['data'];motion=header['motion_config']
    selected_plan=plans[:5]
    # 최초 판정 전 reset/header도 보존한다. 단독 행을 상태 포함 재생으로 오인하지 않는다.
    max_seq=max(r['seq'] for r in selected_plan)
    (OUT/'controller_prefix.jsonl').write_bytes(b''.join(raw for raw,row in zip(fraw,records) if row['seq']<=max_seq))
    sraw,scans=lines(RUN/'scans_received.jsonl')
    selected_scans=[r for r in scans if latch_time-.4<=max(r['acquired_at_s'])<=latch_time+.5]
    limit=max(max(r['acquired_at_s']) for r in selected_scans)
    (OUT/'raw_scan_prefix.jsonl').write_bytes(b''.join(raw for raw,row in zip(sraw,scans) if max(row['acquired_at_s'])<=limit))
    (OUT/'selected_raw_scans.jsonl').write_bytes(b''.join(raw for raw,row in zip(sraw,scans) if row in selected_scans))
    summaries=[]
    with (OUT/'front_raw_body_points.jsonl').open('x',encoding='utf-8') as stream:
        for scan in selected_scans:
            compact,points=front_summary(scan,motion);summaries.append(compact)
            stream.write(json.dumps(dict(stamp_s=scan['stamp_s'],points=points))+'\n')
    _,ego=lines(RUN/'ego_poses.jsonl');_,actor=lines(RUN/'actor_poses.jsonl')
    def truth(t):
        e=min(ego,key=lambda r:abs(r[0]-t));a=min(actor,key=lambda r:abs(r[0]-t));dx,dy=a[1]-e[1],a[2]-e[2]
        c,s=math.cos(e[3]),math.sin(e[3])
        return dict(ego_pose=e,actor_pose=a,actor_in_ego_xy_m=[c*dx+s*dy,-s*dx+c*dy],evaluation_only=True)
    around=[]
    for row in trace[max(0,index-2):index+7]:around.append(dict(**row,truth=truth(row['t'])))
    save(OUT/'guard_timeline.json',dict(first_latched_trace_index=index,first_latched_t=latch_time,previous_t=previous['t'],rows=around,
        timing_resolution='trace approx100ms; not exact latch transition timestamp',
        full_guard_internal_witness_recorded=False,guard_evidence_fields=['safety_reason','safety_clearance_m','drive_speed_mps','safe_speed_mps'],
        replay_inputs='Full original scans and controller prefix/history/config preserved. Exact guard callback receipt ordering/internal latch witness was not logged.'))
    helper_plans=[r for r in plans if helper(r['data']['output'][2])]
    positive_helper=[r for r in helper_plans if r['data']['output'][0]>0]
    normal_positive=[r for r in plans if r['data']['output'][0]>0 and r['data']['output'][2].get('behavior')=='FOLLOW' and not helper(r['data']['output'][2])]
    handoffs=[r for r in plans if r['data']['output'][2].get('moving_pass',{}).get('reason')=='returned_to_current_follow_after_wall_reacquire']
    published=[]
    for pub in pubs:
        ref=pub['data']['reference_s']
        matching=[r for r in calls if abs(r['data'].get('current_rebase',{}).get('output',{}).get('reference_s',math.inf)-ref)<1e-8 or abs(r['data']['input']['reference_s']-ref)<1e-8]
        published.append(dict(publication=pub,matched_call_seqs=[r['seq'] for r in matching],
            matched_helper=any(helper(r['data']['output'][2]) for r in matching)))
    compact_plans=[]
    for row in selected_plan:
        d=row['data'];speed,steer,meta=d['output'];normal=meta.get('normal',{})
        compact_plans.append(dict(seq=row['seq'],runtime_sequence=d['compute']['sequence'],reference_s=d['input']['reference_s'],
            speed_mps=speed,steering_rad=steer,reason=meta.get('reason'),behavior=meta.get('behavior'),handoff_state=meta.get('handoff_state'),
            helper=helper(meta),normal_reason=normal.get('reason'),boundary_selection=normal.get('boundary_selection'),
            underlying_reason=normal.get('underlying_reason'),swept_clearance_m=normal.get('swept_clearance_m'),
            occupancy_padding_m=normal.get('occupancy_padding_m'),occupied_points=normal.get('occupied_points'),
            front_patch=meta.get('persistence'),delivery=d['delivery'],
            current_rebase_output=d.get('current_rebase',{}).get('output'),truth=truth(d['input']['reference_s'])))
    window=[r for r in trace if report['case_window']['green_sim_s']<=r['t']<=report['case_window']['end_sim_s']]
    source_files=['follow_decisions.jsonl','scans_received.jsonl','trajectory.jsonl','trace.json','ego_poses.jsonl','actor_poses.jsonl',
        'autonomy_status.jsonl','runtime_world.sdf','runtime_local_sensors.yaml','report.json','simulation.log']
    result=dict(scope='One early-stopped invalid r520 run; observation extraction only, no replay or attribution',
        run_count=1,first_latched_trace_t=latch_time,previous_trace=dict(t=previous['t'],safety_reason=previous['safety_reason']),
        first_controller_plan_t=plans[0]['data']['input']['reference_s'],latch_observed_before_first_plan=True,
        helper_plan_rows=len(helper_plans),positive_helper_plan_rows=len(positive_helper),all_plan_rows=len(plans),
        top_plan_reason_counts=dict(collections.Counter(r['data']['output'][2].get('reason') for r in plans)),
        normal_reason_counts=dict(collections.Counter(r['data']['output'][2].get('normal',{}).get('reason') for r in plans)),
        zero_normal_baseline_reason_counts=dict(collections.Counter(str(r['data']['output'][2].get('normal',{}).get('baseline_reason')) for r in plans if r['data']['output'][0]<=0)),
        positive_normal_follow_plan_rows=len(normal_positive),explicit_reacquire_to_normal_follow_handoffs=len(handoffs),
        publications=published,first_plans=compact_plans,
        sampled_window_guard_counts=dict(collections.Counter(r['safety_reason'] for r in window)),
        peak_sampled_drive_mps=max(r['drive_speed_mps'] for r in window),peak_sampled_safe_mps=max(r['safe_speed_mps'] for r in window),
        initial_actor_gap_m=trace[0]['sampled_gap_m'],first_latched_evaluation_truth=truth(latch_time),
        raw_front_point_inspection=dict(body_transform=motion,roi='0<=body x<=1.2m, abs(body y)<=.35m; inspection ROI, not guard/acceptance threshold',
            motion_compensated=False,scan_to_guard_callback_match_proven=False,summary=summaries),
        original_source_hashes={name:sha(RUN/name) for name in source_files},
        selected_source_hashes={name:sha(OUT/name) for name in ['controller_prefix.jsonl','raw_scan_prefix.jsonl','selected_raw_scans.jsonl','front_raw_body_points.jsonl','guard_timeline.json']},
        original_report_sha256=report_hash,original_report_unchanged=sha(RUN/'report.json')==report_hash,
        low_speed_publication_is_not_motion_or_guard_acceptance=True,
        known_obstacle_latched_is_recorded_reason_not_proven_geometric_collision=True,
        normal_follow_enum_is_not_proof_of_normal_recovery=True,
        texture_asset_errors_preserved=True,controller_reexecuted=False,guard_reexecuted=False)
    save(OUT/'summary.json',result)
    print(json.dumps({k:result[k] for k in ('first_latched_trace_t','first_controller_plan_t','helper_plan_rows','positive_helper_plan_rows',
        'positive_normal_follow_plan_rows','explicit_reacquire_to_normal_follow_handoffs','publications','peak_sampled_drive_mps','peak_sampled_safe_mps')},ensure_ascii=False))


if __name__=='__main__':main()
