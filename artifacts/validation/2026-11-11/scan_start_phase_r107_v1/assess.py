"""r107 기존 시설 판정·첫 제출 위상·비동기 적용을 구분해 감사한다."""
import importlib.util
import json
import statistics
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3];D=A/'two_actor_run1';RUN=D/'run1'
BASE=A.parent/'native_traffic_baseline_r87_v1'
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
    shape=module('r105_shape_audit',ROOT/'artifacts/validation/2026-11-10/combined_recessed_facility_v1/audit_shapes.py')
    shape.A=A;shape.D=D;shape.RUN=RUN
    shape.evaluate=module('r105_native_outline',BASE/'outline_native.py').evaluate
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
    gates.update(first_source_alignment=any(e['kind']=='reset' and e['data'].get('align_first_source') is True for e in rows),
        native_input_verified=r.get('native_acquisition',{}).get('verified') is True,
        no_additional_impairment=bool(acq) and 'lidar_test_profile' not in r and all('impairment' not in a for a in acq),
        process_execution=any(e['kind']=='reset' and e['data'].get('execution')=='process_candidate' for e in rows),
        all_calls_have_delivery=bool(calls) and len(candidates)+len(discarded)==len(calls),
        planning_dt_0p1=bool(calls) and all(c['input']['dt_s']==.1 for c in calls),
        receiving_parent_50hz=read(D/'recessed_config.json')['launch']['control_rate_hz']==50.,
        accepted_source_age=bool(candidates) and all(0<=c['delivery']['source_age_s']<.3 for c in candidates),
        accepted_reference_age=bool(candidates) and all(0<=t-c['input']['reference_s']<.2 for c,t in zip(candidates,applies)),
        positive_dt_budget=all(x>=.1-1e-8 for x in positive_intervals))
    fixture=read(D/'assessment.json');outline=read(D/'outline_assessment.json');structure=outline['structure']
    gates.update(facility_goal=fixture['passed'],
        actor_structure_unchanged=structure['exact_expected'] and structure['unchanged_nonvisual'] and structure['visual_counts']==[3,1],
        snapshot_unchanged=read(D/'extra_integrity.json')['snapshot_unchanged'])
    result=dict(passed=all(gates.values()),gates=gates,scan_age_s=age,
        progress=fixture['course'],longest_low_speed_s=fixture['longest_low_speed_s'],raw_pass=r['passed'],
        delivery=dict(candidates=len(candidates),discarded=len(discarded),
            discard_reasons={reason:sum(c['delivery'].get('reason')==reason for c in discarded)
                             for reason in sorted({c['delivery'].get('reason','') for c in discarded})},
            application_source_age_s=summary([c['delivery']['source_age_s'] for c in candidates]),
            positive_interval_s=summary(positive_intervals),
            computation_wall_ms=summary([1000*(c['compute']['completed_wall_s']-c['compute']['began_wall_s']) for c in calls])),
        scope='single slow-clock simulation with async receiving parent; no injected Jetson wall-delay, no full real-time/hardware proof; wire stamp behavior separately unit tested')
    with (D/'combined_assessment.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print(json.dumps(result));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
