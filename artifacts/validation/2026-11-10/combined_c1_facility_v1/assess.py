"""기존 시설·형상·C1 주입 판정 재사용. 기존 출력/평가기 바이트 불변."""
import importlib.util,json,sys
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3];D=A/'two_actor_run1';RUN=D/'run1'
F=ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
sys.path[:0]=[str(F),str(ROOT/'scripts')]
from traffic_posts_assess import assess


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m);return m


def main():
    assert not (D/'assessment.json').exists() and not (D/'combined_assessment.json').exists()
    assess(D)
    shape=module('shape_audit',A.parent/'combined_recessed_facility_v1/audit_shapes.py')
    shape.A=A;shape.D=D;shape.RUN=RUN
    shape.main()
    stress=module('registered_stress_assess',ROOT/'artifacts/validation/2026-11-08/masked_motion_speed_v1/assess.py')
    read=lambda p:json.loads(p.read_bytes())
    r=read(RUN/'report.json');acq=read(RUN/'acquisition.json')
    received=[json.loads(s) for s in (RUN/'scans_received.jsonl').open()]
    gates,details=stress.injection_gates('B',RUN,r,acq,received)
    decision,age=stress.decision_gates(RUN,acq);gates.update(decision)
    fixture=read(D/'assessment.json');outline=read(D/'outline_assessment.json')
    structure=outline['structure']
    gates.update(facility_goal=fixture['passed'],
        actor_structure_unchanged=structure['exact_expected'] and structure['unchanged_nonvisual'] and structure['visual_counts']==[3,1],
        snapshot_unchanged=read(D/'extra_integrity.json')['snapshot_unchanged'])
    result=dict(passed=all(gates.values()),gates=gates,injection=details,scan_age_s=age,
        progress=fixture['course'],longest_low_speed_s=fixture['longest_low_speed_s'],raw_pass=r['passed'],
        shape_observation_scope='8mm geometric classifier retained as diagnostic only; injected 30mm errors exceed 25mm recess separation, so these noisy counts do not prove individual recessed returns')
    with (D/'combined_assessment.json').open('x') as stream:stream.write(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
