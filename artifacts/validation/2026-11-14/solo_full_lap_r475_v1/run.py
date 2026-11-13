"""r475 actor0 최신 기하 단독1랩. root review 뒤 고정 명령으로1회만 실행."""
import inspect
import json
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A),str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare import arguments,verify_prepared,write_new,sha
from launch_adapter import instrument_start

def configure(target):
    from shapely.affinity import rotate,translate
    from facility_geometry import load,slab_projection
    F=ROOT/'artifacts/validation/2026-11-13/old_signal_610_r326_v1'
    record=json.loads((F/'fixture.json').read_bytes());mesh=F/'support_frame_m.stl'
    assert record['original_sources_unchanged'] and sha(mesh)==record['mesh_sha256']
    _,_,triangles,_=load(mesh);shape=slab_projection(triangles,.0325,.0925)
    shapes=[translate(rotate(shape,p['yaw'],origin=(0,0),use_radians=True),p['x'],p['y']) for p in record['locations']]
    original=target.obstacle_polygons
    target.obstacle_polygons=lambda path:[*original(path),*shapes]
    original_start=instrument_start(target.start_demo_process,A/'launch_command.json')
    def start(*args,**kwargs):
        bound=inspect.signature(original_start).bind(*args,**kwargs);bound.apply_defaults()
        extra=list(bound.arguments['extra_arguments'])
        assert sum(x.startswith('evaluation_actor_sdf:=') for x in extra)==1
        extra+=['g1_narrow_occluder_bridge:=true','g1_junction_continuity:=true','evaluation_course_markers:=blank_20_30']
        bound.arguments['extra_arguments']=extra
        write_new(A/'launch_extra.json',extra)
        return original_start(*bound.args,**bound.kwargs)
    target.start_demo_process=start

def main():
    assert Path.cwd()==ROOT and not sys.argv[1:]
    assert not (A/'registered.json').exists() and not (A/'run1').exists(),'one execution only'
    prepared=verify_prepared()
    review=json.loads((A/'root_execution_review.json').read_bytes())
    assert review['root_reviewed'] and review['execution_limit']==1 and review['repeat_on_failure'] is False
    assert review['prepared_sha256']==sha(A/'prepared.json')
    import solo_runner as target
    configure(target);args=arguments();assert args==prepared['args']
    write_new(A/'registered.json',dict(args=args,frozen_inputs=prepared['frozen_inputs'],actor_count=0,
        source_archive_sha256=prepared['source_archive_sha256'],baseline='r469',ground_truth_evaluation_only=True,
        one_execution_only=True,repeat_on_failure=False))
    code=None
    try:
        sys.argv=args;code=target.main();return code
    finally:
        changed=[p for p,h in prepared['frozen_inputs'].items() if not (ROOT/p).exists() or sha(ROOT/p)!=h]
        write_new(A/'integrity.json',dict(validation_returncode=code,changed_inputs=changed,input_count=len(prepared['frozen_inputs']),
            archive_unchanged=sha(A/'sources.zip')==prepared['source_archive_sha256']))
        trace=A/'run1/follow_decisions.jsonl'
        if trace.exists():
            from audit_runtime import audit
            write_new(A/'runtime_audit.json',audit(trace,A/'expected_header.json'))
        report=A/'run1/report.json'
        if report.exists():
            from audit_runtime import stop_sequence
            write_new(A/'stop_sequence.json',stop_sequence(json.loads(report.read_bytes()),
                json.loads((A/'early_stop.json').read_bytes()) if (A/'early_stop.json').exists() else {}))

if __name__=='__main__':raise SystemExit(main())
