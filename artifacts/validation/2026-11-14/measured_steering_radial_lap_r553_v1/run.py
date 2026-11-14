"""r544 actor0 대표 전체랩. r530 검토/소스동결/별도 root 실행인계 후 단1회."""
import inspect,json,sys
from pathlib import Path

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A),str(ROOT/'scripts'),str(ROOT/'tests')]
from prepare import arguments,read,sha,verify_prepared,verify_traffic_review,write_new
from launch_adapter import instrument_start
from field_wall_geometry import DERIVED,SOURCE_SHA,verify_derived


def configure(target,source_inputs):
    from shapely.affinity import rotate,translate
    from facility_geometry import load,slab_projection
    fixture=ROOT/'artifacts/validation/2026-11-13/old_signal_610_r326_v1'
    record=read(fixture/'fixture.json');mesh=fixture/'support_frame_m.stl'
    assert record['original_sources_unchanged'] and sha(mesh)==record['mesh_sha256']
    _,_,triangles,_=load(mesh);shape=slab_projection(triangles,.0325,.0925)
    shapes=[translate(rotate(shape,p['yaw'],origin=(0,0),use_radians=True),p['x'],p['y']) for p in record['locations']]
    assert len(shapes)==3;verify_derived()
    original=target.obstacle_polygons;expected=original(DERIVED)
    def field_obstacles(path):
        assert sha(Path(path))==SOURCE_SHA
        return [*expected,*shapes]
    target.obstacle_polygons=field_obstacles
    evaluate=target.solo_evaluation.evaluate_run
    def evaluate_runtime(**kwargs):
        runtime=original(A/'run1/runtime_world.sdf')
        equal=len(runtime)==len(expected) and all(x.equals_exact(y,1e-12) for x,y in zip(runtime,expected))
        receipt=dict(passed=equal,expected_static_walls=len(expected),runtime_static_walls=len(runtime),
            additional_facilities=3,derived_world_sha256=sha(DERIVED),runtime_world_sha256=sha(A/'run1/runtime_world.sdf'),
            scope='r492 same static box polygon comparison; evaluation only')
        write_new(A/'evaluation_geometry.json',receipt)
        result=evaluate(**kwargs)
        result.setdefault('harness',{}).setdefault('checks',{})['runtime_wall_matches_evaluator']=equal
        result['harness']['valid']=result['harness'].get('valid') is True and equal
        result['variation_geometry']=receipt
        return result
    target.solo_evaluation.evaluate_run=evaluate_runtime
    original_paths=target.source_paths
    local=A.relative_to(ROOT).as_posix()+'/'
    files=[p for p in source_inputs if not p.endswith('.zip') and
           (p.startswith(('src/','scripts/','config/',local)) or 'old_signal_610_r326_v1/' in p)]
    files += [(A/n).relative_to(ROOT).as_posix() for n in ('field_world.sdf','facilities_only.sdf','resource_manifest.json','resource_check.json','field_geometry_audit.json')]
    target.source_paths=lambda *args,**kwargs:list(dict.fromkeys([*original_paths(*args,**kwargs),*files]))
    start_original=instrument_start(target.start_demo_process,A/'launch_command.json')
    def start(*args,**kwargs):
        bound=inspect.signature(start_original).bind(*args,**kwargs);bound.apply_defaults()
        extra=list(bound.arguments['extra_arguments'])
        assert sum(v.startswith('evaluation_actor_sdf:=') for v in extra)==1
        extra+=['g1_narrow_occluder_bridge:=true','g1_junction_continuity:=true','evaluation_course_markers:=blank_20_30']
        bound.arguments['extra_arguments']=extra;write_new(A/'launch_extra.json',extra)
        return start_original(*bound.args,**bound.kwargs)
    target.start_demo_process=start


def main():
    assert Path.cwd()==ROOT and not sys.argv[1:]
    assert not any((A/p).exists() for p in ('registered.json','run1')),'one execution only'
    prepared=verify_prepared();verify_traffic_review();review=read(A/'root_execution_review.json')
    assert review['root_reviewed'] and review['execution_limit']==1 and review['repeat_on_failure'] is False
    assert review['wsl_exclusive_owner_confirmed'] and review['prepared_sha256']==sha(A/'prepared.json')
    assert review['source_archive_sha256']==sha(A/'sources.zip')
    import solo_runner as target
    configure(target,prepared['frozen_inputs']);args=arguments();assert args==prepared['args']
    write_new(A/'registered.json',dict(args=args,frozen_inputs=prepared['frozen_inputs'],source_archive_sha256=prepared['source_archive_sha256'],
        actor_count=0,facilities=3,one_execution_only=True,repeat_on_failure=False,ground_truth_evaluation_only=True,
        geometry_claim='Only first-straight two walls are curved/gapped; full circuit is not uniformly deformed'))
    code=None
    try:
        sys.argv=args;code=target.main();return code
    finally:
        changed=[p for p,h in prepared['frozen_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
        write_new(A/'integrity.json',dict(validation_returncode=code,changed_inputs=changed,input_count=len(prepared['frozen_inputs']),
            observational_changes=[p for p,h in prepared.get('observed_inputs',{}).items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h],
            observational_count=len(prepared.get('observed_inputs',{})),
            archive_unchanged=sha(A/'sources.zip')==prepared['source_archive_sha256']))
        if (A/'run1/report.json').exists():
            from audit_runtime import stop_sequence
            write_new(A/'stop_sequence.json',stop_sequence(read(A/'run1/report.json'),read(A/'early_stop.json') if (A/'early_stop.json').exists() else {}))


if __name__=='__main__':raise SystemExit(main())
