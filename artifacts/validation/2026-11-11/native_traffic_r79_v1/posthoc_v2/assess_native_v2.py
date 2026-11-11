"""r79 원 assess의 균일 각도/시각 가정 실패를 보존한 저장자료 후처리. 재주행 없음."""
import hashlib,importlib.util,json,sys
from pathlib import Path
ROOT=next(p for p in Path(__file__).resolve().parents if (p/'AGENTS.md').is_file() and (p/'src/arena_autonomy').is_dir())
A=ROOT/'artifacts/validation/2026-11-11/native_traffic_r79_v1'
D=A/'two_actor_run1';RUN=D/'run1';OUT=A/'posthoc_v2'
sys.path[:0]=[str(A),str(ROOT/'scripts')]
read=lambda p:json.loads(p.read_bytes())
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    obj=importlib.util.module_from_spec(spec);sys.modules[name]=obj;spec.loader.exec_module(obj)
    return obj

def main():
    assert (D/'assessment.json').is_file()
    assert not (D/'combined_assessment.json').exists()
    assert not read(D/'extra_integrity.json')['changed']
    inputs=[RUN/'report.json',RUN/'follow_decisions.jsonl',RUN/'acquisition.json',
        RUN/'scans_received.jsonl',D/'assessment.json',D/'surface.json',D/'extra_sources.zip']
    before={p.relative_to(ROOT).as_posix():sha(p) for p in inputs}
    OUT.mkdir(exist_ok=False)
    for name in ('recessed_config.json','additional_actors.json'):
        (OUT/name).write_bytes((D/name).read_bytes())
    adapter=Path(__file__).with_name('outline_native.py')
    native=module('outline_native_posthoc',adapter)
    shape_path=ROOT/'artifacts/validation/2026-11-10/combined_recessed_facility_v1/audit_shapes.py'
    shape=module('shape_audit_native_posthoc',shape_path)
    shape.A=A;shape.D=OUT;shape.RUN=RUN;shape.evaluate=native.evaluate
    shape.main()
    stress_path=ROOT/'artifacts/validation/2026-11-08/masked_motion_speed_v1/assess.py'
    stress=module('stress_native_posthoc',stress_path)
    from native_decision_gates import decision_gates
    report=read(RUN/'report.json');acq=read(RUN/'acquisition.json')
    received=[json.loads(line) for line in (RUN/'scans_received.jsonl').open()]
    gates,details=stress.injection_gates('B',RUN,report,acq,received)
    decisions,ages=decision_gates(RUN,acq);gates.update(decisions)
    gates['native_input_verified']=report.get('native_acquisition',{}).get('verified') is True
    fixture=read(D/'assessment.json');outline=read(OUT/'outline_assessment.json')
    structure=outline['structure']
    gates.update(facility_goal=fixture['passed'],
        actor_structure_unchanged=structure['exact_expected'] and structure['unchanged_nonvisual'] and structure['visual_counts']==[3,1],
        snapshot_unchanged=read(D/'extra_integrity.json')['snapshot_unchanged'])
    gates['posthoc_inputs_unchanged']=all(sha(ROOT/path)==value for path,value in before.items())
    result=dict(passed=all(gates.values()),gates=gates,injection=details,scan_age_s=ages,
        progress=fixture['course'],longest_low_speed_s=fixture['longest_low_speed_s'],raw_pass=report['passed'],
        scope='separate posthoc audit after original outline evaluator KeyError sec; actual run/report/source unchanged; not a new driving trial',
        shape_observation_scope='8mm classifier is diagnostic only; 30mm injected error exceeds 25mm recess separation, so noisy counts do not prove individual recessed returns',
        input_sha256=before,evaluator_sha256={p.relative_to(ROOT).as_posix():sha(p) for p in
            (Path(__file__),adapter,shape_path,stress_path,A/'native_decision_gates.py')})
    (OUT/'combined_assessment.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    for path in (Path(__file__),adapter,Path(__file__).with_name('test_outline_native.py')):
        (OUT/path.name).write_bytes(path.read_bytes())
    print(json.dumps({k:v for k,v in result.items() if k not in ('input_sha256','evaluator_sha256')},ensure_ascii=False))
    return int(not result['passed'])

if __name__=='__main__':raise SystemExit(main())
