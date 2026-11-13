"""동결·nominal 성공·주 담당 검토 뒤 변형 하나를 한 번만 실행한다."""
import inspect
import json
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(A), str(ROOT/'scripts'), str(ROOT/'tests')]
from prepare_execution import CASES, arguments, case_dir, nominal_success, read, verify_prepared
from variation_launch import sha, write_new
from launch_adapter import instrument_start


def configure(target, name, evidence):
    from shapely.affinity import rotate, translate
    from facility_geometry import load, slab_projection
    fixture = ROOT/'artifacts/validation/2026-11-13/old_signal_610_r326_v1'
    record = read(fixture/'fixture.json')
    mesh = fixture/'support_frame_m.stl'
    assert record['original_sources_unchanged'] and sha(mesh) == record['mesh_sha256']
    _, _, triangles, _ = load(mesh)
    shape = slab_projection(triangles, .0325, .0925)
    shapes = [translate(rotate(shape, p['yaw'], origin=(0,0), use_radians=True), p['x'], p['y'])
              for p in record['locations']]
    assert len(shapes) == 3
    original = target.obstacle_polygons
    variant = A/'connection_v1'/f'{name}.sdf'
    expected_walls = original(variant)
    target.obstacle_polygons = lambda path: [*expected_walls, *shapes]
    original_evaluate = target.solo_evaluation.evaluate_run
    def evaluate(**kwargs):
        # launch 전 STRtree와 종료 평가가 같은 물리 월드를 사용했는지 실제 runtime에서 대조한다.
        runtime = original(evidence/'run1/runtime_world.sdf')
        equal = len(runtime) == len(expected_walls) and all(
            x.equals_exact(y, 1e-12) for x, y in zip(runtime, expected_walls))
        audit = dict(passed=equal, expected_static_walls=len(expected_walls), runtime_static_walls=len(runtime),
            additional_facilities=len(shapes), raw_variant_sha256=sha(variant),
            runtime_world_sha256=sha(evidence/'run1/runtime_world.sdf'),
            comparison='same static box polygons; old610 three STL slabs retained from baseline')
        write_new(evidence/'evaluation_geometry.json', audit)
        result = original_evaluate(**kwargs)
        result.setdefault('harness', {}).setdefault('checks', {})['runtime_wall_matches_evaluator'] = equal
        result['harness']['valid'] = result['harness'].get('valid') is True and equal
        result['variation_geometry'] = audit
        return result
    target.solo_evaluation.evaluate_run = evaluate
    original_start = instrument_start(target.start_demo_process, evidence/'launch_command.json', name)
    def start(*args, **kwargs):
        bound = inspect.signature(original_start).bind(*args, **kwargs)
        bound.apply_defaults()
        extra = list(bound.arguments['extra_arguments'])
        assert sum(x.startswith('evaluation_actor_sdf:=') for x in extra) == 1
        extra += ['g1_narrow_occluder_bridge:=true', 'g1_junction_continuity:=true', 'evaluation_course_markers:=blank_20_30']
        bound.arguments['extra_arguments'] = extra
        write_new(evidence/'launch_extra.json', extra)
        return original_start(*bound.args, **bound.kwargs)
    target.start_demo_process = start


def main():
    assert Path.cwd() == ROOT and len(sys.argv) == 2 and sys.argv[1] in CASES
    name = sys.argv[1]
    evidence = case_dir(name)
    assert not any((evidence/p).exists() for p in ('registered.json', 'run1')), 'one execution only'
    prepared = verify_prepared()
    nominal_success()
    review = read(evidence/'root_execution_review.json')
    assert review['root_reviewed'] and review['case'] == name and review['execution_limit'] == 1
    assert review['repeat_on_failure'] is False and review['wsl_exclusive_owner_confirmed'] is True
    assert review['prepared_sha256'] == sha(A/'execution_prepared.json')
    assert review['source_archive_sha256'] == prepared['source_archive_sha256']
    import solo_runner as target
    configure(target, name, evidence)
    args = arguments(name)
    assert args == prepared['args'][name]
    write_new(evidence/'registered.json', dict(case=name, args=args, frozen_inputs=prepared['frozen_inputs'],
        source_archive_sha256=prepared['source_archive_sha256'], actor_count=0, one_execution_only=True,
        repeat_on_failure=False, ground_truth_evaluation_only=True))
    code = None
    try:
        sys.argv = args
        code = target.main()
        return code
    finally:
        changed = [p for p,h in prepared['frozen_inputs'].items() if not (ROOT/p).exists() or sha(ROOT/p) != h]
        write_new(evidence/'integrity.json', dict(validation_returncode=code, changed_inputs=changed,
            archive_unchanged=sha(A/'execution_sources.zip') == prepared['source_archive_sha256']))
        trace = evidence/'run1/follow_decisions.jsonl'
        if trace.exists():
            from audit_runtime import audit
            write_new(evidence/'runtime_audit.json', audit(trace, A/'expected_header.json'))
        if (evidence/'run1/report.json').exists():
            from audit_runtime import stop_sequence
            write_new(evidence/'stop_sequence.json', stop_sequence(read(evidence/'run1/report.json'),
                read(evidence/'early_stop.json') if (evidence/'early_stop.json').exists() else {}))


if __name__ == '__main__':
    raise SystemExit(main())
