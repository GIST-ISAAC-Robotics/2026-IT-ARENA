"""r406 평가/시설을 유지하고 crop-edge 선택만 추가한 final4가림 제한 단1회."""
import copy
from datetime import datetime, timezone
import inspect
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
F = A.parent/'old_signal_610_r326_v1'
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'tests')]
from prepare import arguments, read, sha, verify_prepared, write_new
from launch_adapter import instrument_start


def configure(target):
    from shapely.affinity import rotate, translate
    from facility_geometry import load, slab_projection
    record = read(F/'fixture.json')
    assert record['original_sources_unchanged']
    mesh = F/'support_frame_m.stl'
    assert sha(mesh) == record['mesh_sha256']
    _, _, triangles, _ = load(mesh)
    shape = slab_projection(triangles, .0325, .0925)
    shapes = [translate(rotate(shape, p['yaw'], origin=(0, 0), use_radians=True), p['x'], p['y'])
              for p in record['locations']]
    original = target.obstacle_polygons
    target.obstacle_polygons = lambda path: [*original(path), *shapes]
    original_start = instrument_start(target.start_demo_process, A/'launch_command.json')

    def start(*args, **kwargs):
        bound = inspect.signature(original_start).bind(*args, **kwargs)
        bound.apply_defaults()
        extra = list(bound.arguments['extra_arguments'])
        indices = [i for i, value in enumerate(extra) if value.startswith('evaluation_actor_sdf:=')]
        assert len(indices) == 1 and not any(x.startswith('signal_posts:=') for x in extra)
        index = indices[0]
        actor = Path(extra[index].split(':=', 1)[1])
        tree = ET.parse(actor)
        for model in ET.parse(F/'supports_trial.sdf').getroot().findall('model'):
            model.set('name', 'eval_r406_'+model.get('name'))
            tree.getroot().append(copy.deepcopy(model))
        destination = actor.with_name('actors_and_old610.sdf')
        ET.indent(tree)
        tree.write(destination, encoding='utf8', xml_declaration=True)
        extra[index] = 'evaluation_actor_sdf:='+str(destination)
        extra += ['g1_narrow_occluder_bridge:=true', 'g1_junction_continuity:=true',
                  'evaluation_course_markers:=blank_20_30']
        bound.arguments['extra_arguments'] = extra
        write_new(A/'launch_extra.json', extra)
        return original_start(*bound.args, **bound.kwargs)
    target.start_demo_process = start


def main():
    assert Path.cwd() == ROOT and not sys.argv[1:], 'fixed relative command from repository root required'
    assert not (A/'registered.json').exists() and not (A/'run1').exists(), 'one execution only'
    prepared = verify_prepared()
    import validate_g1_traffic as target
    configure(target)
    args = arguments()
    assert args == prepared['args']
    prepared_sha = sha(A/'prepared.json')
    write_new(A/'registered.json', dict(at_utc=datetime.now(timezone.utc).isoformat(), args=args,
        input_sha256=prepared['frozen_inputs'], source_archive_sha256=prepared['source_archive_sha256'],
        prepared_sha256=prepared_sha, parent=str((A.parent/'final_runtime_drive_r406_v1').relative_to(ROOT)),
        scope=read(A/'config.json')['scope'], source_timestamps_unchanged=True,
        ground_truth_evaluation_only=True, causal_comparison=False, default_changed=False,
        independent_guard_bypassed=False, runtime_policy='cad_runtime_r377_v1',
        current_corner_budget='r396_v1', marker_z_m=.09, known_structure_sectors=4,
        crop_edge_component_trial=True,
        node_launch_prefix=False, candidate_artifact_imports=False))
    code = None
    try:
        sys.argv = args
        code = target.main()
        return code
    finally:
        changed = [p for p,h in prepared['frozen_inputs'].items() if not (ROOT/p).is_file() or sha(ROOT/p) != h]
        write_new(A/'integrity.json', dict(validation_returncode=code, changed_inputs=changed,
            input_count=len(prepared['frozen_inputs']), prepared_unchanged=sha(A/'prepared.json')==prepared_sha,
            archive_unchanged=sha(A/'sources.zip')==prepared['source_archive_sha256']))
        trace = A/'run1/follow_decisions.jsonl'
        if trace.is_file():
            from audit_runtime import audit
            write_new(A/'runtime_audit.json', audit(trace, A/'expected_header.json'))


if __name__ == '__main__':
    raise SystemExit(main())
