"""추가 스트레스 없는 native 기준. 기존 시설/정지/소스 판정을 보존한다."""
import importlib.util
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3];D=A/'two_actor_run1';RUN=D/'run1'
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
    fixture=read(D/'assessment.json');outline=read(D/'outline_assessment.json')
    structure=outline['structure']
    gates.update(facility_goal=fixture['passed'],
        actor_structure_unchanged=structure['exact_expected'] and structure['unchanged_nonvisual'] and structure['visual_counts']==[3,1],
        snapshot_unchanged=read(D/'extra_integrity.json')['snapshot_unchanged'])
    result=dict(passed=all(gates.values()),gates=gates,scan_age_s=age,
        progress=fixture['course'],longest_low_speed_s=fixture['longest_low_speed_s'],raw_pass=r['passed'],
        scope='functional native baseline with ideal simulated ranges; additional radial noise and transport jitter both absent; not stress or real sensor proof')
    with (D/'combined_assessment.json').open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
