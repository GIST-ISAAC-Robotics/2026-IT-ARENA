"""r114 기존 시설 판정·입력·배송 지연을 구분한 감사."""
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
        if candidates[i]['output'][0]>0:positive_intervals.append(applies[i]-applies[i-1])
    injected=[c['compute'] for c in calls if 'injected_delay_s' in c.get('compute',{})]
    gates.update(common_expiry_reference_selected=rows[0]['data']['follow_config'].get('expiry_route_reference_trial') is True and any(e['kind']=='expiry_reference_policy' for e in rows),delay_profile_registered=any(e['kind']=='delivery_delay_profile' for e in rows),
        fresh_submission_registered=any(e['kind']=='delivery_delay_profile' and e['data'].get('submission_policy')=='fresh_completed_scan_r114' and e['data'].get('completion_window_s')==.025 for e in rows),
        fresh_submission_age=bool(calls) and all(0<=c['input']['reference_s']-max(c['input']['scan']['measurements']['acquired_at_s'])<=.025+1e-9 for c in calls),
        all_results_have_injection=bool(calls) and len(injected)==len(calls),
        releases_not_early=bool(calls) and all(c['compute'].get('released_sim_s',-1)+1e-8>=
            c['input']['reference_s']+c['compute'].get('injected_delay_s',0) for c in calls),
        native_input_verified=r.get('native_acquisition',{}).get('verified') is True,
        no_additional_impairment=bool(acq) and 'lidar_test_profile' not in r and all('impairment' not in a for a in acq),
        process_execution=any(e['kind']=='reset' and e['data'].get('execution')=='process_candidate' for e in rows),
        all_calls_have_delivery=bool(calls) and len(candidates)+len(discarded)==len(calls),
        planning_dt_0p1=bool(calls) and all(c['input']['dt_s']==.1 for c in calls),
        receiving_parent_50hz=read(D/'recessed_config.json')['launch']['control_rate_hz']==50.,
        accepted_source_age=bool(candidates) and all(0<=c['delivery']['source_age_s']<.3 for c in candidates),
        accepted_reference_age=bool(candidates) and all(0<=t-c['input']['reference_s']<.2 for c,t in zip(candidates,applies)),
        positive_dt_budget=all(x>=.1-1e-8 for x in positive_intervals))
    safety=[json.loads(line) for line in (D/'full_safety.jsonl').read_text().splitlines()]
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
        injected_delay_s=summary([c['injected_delay_s'] for c in injected]),
        scope='single simulation with measured-cost delivery floor; costs belong to an older saved trajectory; not full CPU emulation, hardware timing, or all-condition proof')
    with (D/'combined_assessment.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
