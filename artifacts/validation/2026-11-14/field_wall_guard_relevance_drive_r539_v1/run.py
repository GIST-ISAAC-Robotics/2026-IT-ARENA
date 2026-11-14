"""r530: 원 교통 평가기 + r475 시설/실행 연결. 제어·판정 함수는 변경하지 않는다."""
import copy
import inspect
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(A), str(ROOT / 'scripts'), str(ROOT / 'tests')]
from prepare import read, sha, verify_prepared, write_new
from launch_adapter import instrument_start
from field_wall_geometry import DERIVED, SOURCE_SHA, verify_derived


def compose_facilities(actor_text):
    root = ET.fromstring(actor_text)
    facility = ET.parse(A / 'facilities_only.sdf').getroot()
    assert [m.get('name') for m in root.findall('model')] == ['eval_g1b_actor']
    assert len(facility.findall('model')) == 3
    for model in facility.findall('model'):
        root.append(copy.deepcopy(model))
    return ET.tostring(root, encoding='unicode')


def configure(target):
    capture = read(A/'safety_capture_preparation.json')
    assert sha(Path(target.__file__)) == capture['local_runner_sha256']
    assert sha(ROOT/capture['common_validator_path']) == capture['common_validator_sha256']
    # r475와 같은 평가용 시설 외곽. actor 정답/경로를 제어 입력으로 전달하지 않는다.
    from shapely.affinity import rotate, translate
    from facility_geometry import load, slab_projection
    fixture = ROOT / 'artifacts/validation/2026-11-13/old_signal_610_r326_v1'
    record = read(fixture / 'fixture.json')
    mesh = fixture / 'support_frame_m.stl'
    assert record['original_sources_unchanged'] and sha(mesh) == record['mesh_sha256']
    _, _, triangles, _ = load(mesh)
    shape = slab_projection(triangles, .0325, .0925)
    shapes = [translate(rotate(shape, p['yaw'], origin=(0, 0), use_radians=True), p['x'], p['y'])
              for p in record['locations']]
    verify_derived()
    # 실제 source_snapshot에도 실행 harness/파생월드/외부 자산을 포함한다.
    from world_resources import verify_resources
    source_paths = target.source_paths
    local_files = [*A.glob('*.py'), *(A/'launch').glob('*.py'),
        *[A/n for n in ('field_world.sdf','facilities_only.sdf','resource_manifest.json','field_geometry_audit.json')]]
    paths = [p.relative_to(ROOT).as_posix() for p in local_files]
    paths += [r['repository_path'] for r in verify_resources()['asset_references']]
    target.source_paths = lambda *args, **kwargs: list(dict.fromkeys([*source_paths(*args, **kwargs), *paths]))
    original_obstacles = target.obstacle_polygons
    def field_obstacles(path):
        assert sha(Path(path)) == SOURCE_SHA
        return original_obstacles(DERIVED)
    target.obstacle_polygons = lambda path: [*field_obstacles(path), *shapes]
    original_actor = target.actor_sdf
    target.actor_sdf = lambda config, pose: compose_facilities(original_actor(config, pose))
    original_start = instrument_start(target.start_demo_process, A / 'launch_command.json')

    def start(*args, **kwargs):
        bound = inspect.signature(original_start).bind(*args, **kwargs)
        bound.apply_defaults()
        extra = list(bound.arguments['extra_arguments'])
        assert sum(x.startswith('evaluation_actor_sdf:=') for x in extra) == 1
        extra += ['g1_narrow_occluder_bridge:=true', 'g1_junction_continuity:=true',
                  'evaluation_course_markers:=blank_20_30']
        bound.arguments['extra_arguments'] = extra
        write_new(A / 'launch_extra.json', extra)
        return original_start(*bound.args, **bound.kwargs)

    target.start_demo_process = start


def main():
    assert Path.cwd() == ROOT and not sys.argv[1:]
    assert not (A / 'registered.json').exists() and not (A / 'run1').exists(), 'one execution only'
    prepared = verify_prepared()
    review = read(A / 'root_execution_review.json')
    assert review['root_reviewed'] and review['execution_limit'] == 1 and review['repeat_on_failure'] is False
    assert review['prepared_sha256'] == sha(A / 'prepared.json')
    assert review['source_archive_sha256'] == sha(A / 'sources.zip')
    import traffic_runner as target
    configure(target)
    args = read(A / 'arguments.json')
    assert args == prepared['args']
    write_new(A / 'registered.json', dict(args=args, frozen_inputs=prepared['frozen_inputs'],
        source_archive_sha256=prepared['source_archive_sha256'], actor_count=1, facilities=3,
        case='right_side_start', duration_sim_s=12, ground_truth_evaluation_only=True,
        one_execution_only=True, repeat_on_failure=False, full_lap_claim=False))
    code = None
    try:
        sys.argv = args
        code = target.main()
        return code
    finally:
        changed = [p for p, digest in prepared['frozen_inputs'].items()
                   if not (ROOT / p).is_file() or sha(ROOT / p) != digest]
        write_new(A / 'integrity.json', dict(validation_returncode=code, changed_inputs=changed,
            input_count=len(prepared['frozen_inputs']),
            archive_unchanged=sha(A / 'sources.zip') == prepared['source_archive_sha256']))


if __name__ == '__main__':
    raise SystemExit(main())
