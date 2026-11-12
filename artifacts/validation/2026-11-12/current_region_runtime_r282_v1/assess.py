"""r274 기존 시설 판정과 병렬/원 시각/현재 관측 증거 감사."""
import importlib.util
import json
import statistics
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
assert not sys.argv[1:]
D=A/'two_actor_run1';RUN=D/'run1'
BASE=ROOT/'artifacts/validation/2026-11-11/native_traffic_baseline_r87_v1'
F=ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
sys.path[:0]=[str(F),str(BASE),str(ROOT/'scripts')]
from traffic_posts_assess import assess
from native_decision_gates import decision_gates


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m


def summary(values):
    q=sorted(values)
    return dict(count=len(q),minimum=min(q),p50=statistics.median(q),
        p95=q[min(len(q)-1,int(.95*len(q)))],maximum=max(q)) if q else dict(count=0)


def main():
    assert not (D/'assessment.json').exists() and not (D/'combined_assessment.json').exists()
    assess(D)
    shape=module('r114_shape_audit',ROOT/'artifacts/validation/2026-11-10/combined_recessed_facility_v1/audit_shapes.py')
    shape.A=A;shape.D=D;shape.RUN=RUN
    shape.evaluate=module('r114_native_outline',BASE/'outline_native.py').evaluate
    shape.main()
    read=lambda p:json.loads(p.read_bytes())
    r=read(RUN/'report.json');acq=read(RUN/'acquisition.json')
    gates,age=decision_gates(RUN,acq)
    rows=[json.loads(line) for line in (RUN/'follow_decisions.jsonl').read_text('utf8').splitlines()]
    calls=[x['data'] for x in rows if x['kind']=='call']
    candidates=[c for c in calls if c.get('delivery',{}).get('kind')=='candidate']
    discarded=[c for c in calls if c.get('delivery',{}).get('kind')=='discarded']
    applies=[c['input']['scan']['measurements']['acquired_at_s'][0]+c['delivery']['source_age_s'] for c in candidates]
    positive_intervals=[]
    for i in range(1,len(candidates)):
        if candidates[i].get('current_rebase',{}).get('output',{}).get('speed_mps',candidates[i]['output'][0])>0:positive_intervals.append(applies[i]-applies[i-1])
    injected=[c['compute'] for c in calls if 'minimum_result_age_s' in c.get('compute',{})]
    gates.update(common_expiry_reference_selected=rows[0]['data']['follow_config'].get('expiry_route_reference_trial') is True,
        parallel_runtime_registered=any(e['kind']=='parallel_runtime_policy' and e['data'].get('submission')=='latest_unused_native_v1' and e['data'].get('retained_fallback') is False for e in rows),
        latest_submission_age=bool(calls) and all(0<=c['input']['reference_s']-c['input']['scan']['measurements']['acquired_at_s'][0]<.3 and max(c['input']['scan']['measurements']['acquired_at_s'])<=c['input']['reference_s'] for c in calls),
        plan_delay_applied=bool(injected),
        worker_current_selected=any(e['kind']=='current_path_recheck_policy' and e['data'].get('execution')=='worker_event_v1' for e in rows),
        native_input_verified=r.get('native_acquisition',{}).get('verified') is True,
        no_additional_impairment=bool(acq) and 'lidar_test_profile' not in r and all('impairment' not in a for a in acq),
        process_execution=any(e['kind']=='reset' and e['data'].get('execution')=='process_candidate' for e in rows),
        all_calls_have_delivery=bool(calls) and len(candidates)+len(discarded)==len(calls),
        planning_dt_0p1=bool(calls) and all(c['input']['dt_s']==.1 for c in calls),
        receiving_parent_50hz=read(D/'recessed_config.json')['launch']['control_rate_hz']==50.,
        unchecked_source_age=all(0<=c['delivery']['source_age_s']<.3 for c in candidates if 'current_rebase' not in c),
        unchecked_reference_age=all(0<=t-c['input']['reference_s']<.2 for c,t in zip(candidates,applies) if 'current_rebase' not in c),
        positive_dt_budget=all(x>=.1-1e-8 for x in positive_intervals))
    checked=[c for c in calls if 'current_rebase' in c]
    accepted_checked=[c for c in candidates if 'current_rebase' in c]
    profile=read(A/'delivery_profile.json')
    gates.update(plan_minimum_age=bool(injected) and all(c['minimum_result_age_s']==profile['plan_delay_s'] and c['result_available_sim_s']-c['submitted_sim_s']>=profile['plan_delay_s']-1e-8 for c in injected),
        recheck_minimum_age=bool(checked) and all(c['current_rebase']['compute']['minimum_result_age_s']==profile['recheck_delay_s'] and c['current_rebase']['compute']['result_available_sim_s']-c['current_rebase']['compute']['submitted_sim_s']>=profile['recheck_delay_s']-1e-8 for c in checked))
    gates.update(recheck_used=bool(checked),
        recheck_result_identity=all(c['current_rebase']['compute']['stage']=='current' and
            c['current_rebase']['compute']['generation']==c['compute']['generation'] and
            c['current_rebase']['compute']['sequence']>c['compute']['sequence'] and
            c['current_rebase']['compute']['plan']['sequence']==c['compute']['sequence'] for c in checked),
        current_source_preserved=all(c['current_rebase']['output']['reference_s']==c['current_rebase']['input']['reference_s'] for c in checked),
        accepted_current_age=all(0 <= (c['input']['scan']['measurements']['acquired_at_s'][0]+c['delivery']['source_age_s'])-c['current_rebase']['input']['reference_s']<.2 and
            0 <= (c['input']['scan']['measurements']['acquired_at_s'][0]+c['delivery']['source_age_s'])-c['current_rebase']['input']['scan']['measurements']['acquired_at_s'][0]<.3 for c in accepted_checked))
    positive_checked=[c for c in accepted_checked if c['current_rebase']['output']['speed_mps']>0]
    gates.update(current_region_proven=bool(positive_checked) and all(c['current_rebase']['output'].get('current_route_proof',{}).get('accepted') is True for c in positive_checked),
        no_retained_fallback=all(c['current_rebase']['compute']['route_check']['retained'] is False for c in checked),
        original_geometry_age=all(0<=c['input']['scan']['measurements']['acquired_at_s'][0]+c['delivery']['source_age_s']-c['output'][2]['normal']['path_source_s']<.5 for c in positive_checked),
        no_source_renewal=all(c['current_rebase']['output']['current_route_proof']['source_timestamp_renewed'] is False and c['current_rebase']['output']['current_route_proof']['old_occupancy_permission_reused'] is False for c in positive_checked))
    safety=[json.loads(line) for line in (D/'full_safety.jsonl').read_text().splitlines()]
    stale=[row for row in safety if row.get('command_stale') is True]
    gates.update(stale_command_output_zero=bool(safety) and all(row['safe_speed_mps']==0. for row in stale),
        guard_reference_complete=bool(safety) and all(isinstance(row.get('reference_s'),(float,int)) for row in safety))
    gates.update(traffic_oblique_selected=rows[0]['data']['follow_config'].get('oblique_gap_trial') is True,
        independent_oblique_evaluated=any(((row.get('partial_observation') or {}).get('lateral_metric') or {}).get('oblique_trial') is True for row in safety))
    fixture=read(D/'assessment.json');outline=read(D/'outline_assessment.json');structure=outline['structure']
    gates.update(facility_goal=fixture['passed'],
        actor_structure_unchanged=structure['exact_expected'] and structure['unchanged_nonvisual'] and structure['visual_counts']==[3,1],
        snapshot_unchanged=read(D/'extra_integrity.json')['snapshot_unchanged'],
        registered_sources_unchanged=read(D/'extra_integrity.json')['changed']==[])
    result=dict(passed=all(gates.values()),gates=gates,scan_age_s=age,
        progress=fixture['course'],longest_low_speed_s=fixture['longest_low_speed_s'],raw_pass=r['passed'],
        delivery=dict(candidates=len(candidates),discarded=len(discarded),
            discard_reasons={reason:sum(c['delivery'].get('reason')==reason for c in discarded)
                             for reason in sorted({c['delivery'].get('reason','') for c in discarded})},
            application_source_age_s=summary([c['delivery']['source_age_s'] for c in candidates]),
            positive_interval_s=summary(positive_intervals),
            computation_wall_ms=summary([1000*(c['compute']['completed_wall_s']-c['compute']['began_wall_s']) for c in calls])),
        recheck=dict(count=len(checked),accepted=len(accepted_checked),computation_wall_ms=summary([c['current_rebase']['computation_wall_ms'] for c in checked]),result_wait_wall_ms=summary([1000*(c['current_rebase']['compute']['received_wall_s']-c['current_rebase']['compute']['completed_wall_s']) for c in checked])),
        guard_observation=dict(stale_command_calls=len(stale),
            stale_command_with_observation=sum(row.get('partial_observation') is not None for row in stale),
            reasons={reason:sum(row.get('reason')==reason for row in safety) for reason in sorted({row.get('reason','') for row in safety})}),
        scope='PC explicit parallel runtime/current-region recheck with r248 stage p95 minimum-age injection; not a joint p95 or worst-case Jetson/hardware proof')
    with (D/'combined_assessment.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
