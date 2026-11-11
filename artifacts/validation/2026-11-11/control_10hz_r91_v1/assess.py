"""추가 스트레스 없는 native 기준. 기존 시설/정지/소스 판정을 보존한다."""
import importlib.util
import json
import math
import statistics
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3];D=A/'two_actor_run4';RUN=D/'run1'
F=ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
sys.path[:0]=[str(F),str(ROOT/'scripts')]
from traffic_posts_assess import assess
from native_decision_gates import decision_gates


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
    return m


def main():
    assert not (D/'assessment.json').exists() and not (D/'combined_assessment.json').exists()
    assess(D)
    shape=module('shape_audit',ROOT/'artifacts/validation/2026-11-10/combined_recessed_facility_v1/audit_shapes.py')
    shape.A=A;shape.D=D;shape.RUN=RUN
    shape.evaluate=module('native_outline',A/'outline_native.py').evaluate
    shape.main()
    read=lambda p:json.loads(p.read_bytes())
    r=read(RUN/'report.json');acq=read(RUN/'acquisition.json')
    gates,age=decision_gates(RUN,acq)
    gates['native_input_verified']=r.get('native_acquisition',{}).get('verified') is True
    gates['no_additional_impairment']=bool(acq) and 'lidar_test_profile' not in r and all('impairment' not in a for a in acq)
    refs=[]; dts=[]
    with (RUN/'follow_decisions.jsonl').open(encoding='utf8') as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='call':
                refs.append(event['data']['input']['reference_s'])
                dts.append(event['data']['input']['dt_s'])
    intervals=[b-a for a,b in zip(refs,refs[1:]) if b>a]
    gates['control_dt_0p1_verified']=bool(dts) and all(math.isclose(v,.1,abs_tol=1e-12,rel_tol=0) for v in dts)
    gates['control_config_10hz']=read(D/'recessed_config.json')['launch']['control_rate_hz']==10.
    fixture=read(D/'assessment.json');outline=read(D/'outline_assessment.json')
    structure=outline['structure']
    gates.update(facility_goal=fixture['passed'],
        actor_structure_unchanged=structure['exact_expected'] and structure['unchanged_nonvisual'] and structure['visual_counts']==[3,1],
        snapshot_unchanged=read(D/'extra_integrity.json')['snapshot_unchanged'])
    result=dict(passed=all(gates.values()),gates=gates,scan_age_s=age,
        progress=fixture['course'],longest_low_speed_s=fixture['longest_low_speed_s'],raw_pass=r['passed'],
        control_period_s=dict(calls=len(refs),dt_min=min(dts) if dts else None,dt_max=max(dts) if dts else None,
            interval_p50=statistics.median(intervals) if intervals else None,interval_max=max(intervals) if intervals else None),
        scope='10Hz control closed-loop nominal facility traffic; no processing-delay model, real-time, or Jetson proof')
    with (D/'combined_assessment.json').open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
