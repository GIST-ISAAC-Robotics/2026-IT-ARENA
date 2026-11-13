"""최종 CAD 조향과 r375 RELEASED parallel 현재 상대 거리 검사의 제한 교통1회."""
import copy
from datetime import datetime, timezone
import hashlib
import inspect
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
BASE = A.parent/'relative_body_cad_drive_r371_v1'
F = A.parent/'old_signal_610_r326_v1'
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'tests')]
import validate_g1_traffic as target
from launch_prefix import instrument_start
from prepare import configuration, arguments, verify_prepared


def configure():
    # 평가용 형상은 앞선 r333과 같은 실제 차체 높이 slab를 사용한다.
    # 이 함수의 경로/정답은 제어기로 전송하지 않는다.
    import importlib.util
    path = A.parent/'old610_common_corner_r333_v1/run.py'
    spec = importlib.util.spec_from_file_location('r377_previous_shape', path)
    previous = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(previous)
    from shapely.affinity import rotate, translate
    record = json.loads((F/'fixture.json').read_bytes())
    assert record['original_sources_unchanged']
    mesh = F/'support_frame_m.stl'
    assert hashlib.sha256(mesh.read_bytes()).hexdigest() == record['mesh_sha256']
    _, _, triangles, _ = previous.fixture.analysis.audit.load(mesh)
    shape = previous.slab_projection(triangles, .0325, .0925)
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
            model.set('name', 'eval_r377_'+model.get('name'))
            tree.getroot().append(copy.deepcopy(model))
        destination = actor.with_name('actors_and_old610.sdf')
        ET.indent(tree)
        tree.write(destination, encoding='utf8', xml_declaration=True)
        extra[index] = 'evaluation_actor_sdf:='+str(destination)
        extra += ['g1_narrow_occluder_bridge:=true', 'g1_junction_continuity:=true',
                  'evaluation_course_markers:=blank_20_30']
        bound.arguments['extra_arguments'] = extra
        (A/'launch_extra.json').write_text(json.dumps(extra, indent=2)+'\n')
        return original_start(*bound.args, **bound.kwargs)
    target.start_demo_process = start


def main():
    assert not sys.argv[1:] and not (A/'registered.json').exists(), 'one registered execution'
    out = A/'run1'
    assert not out.exists()
    verify_prepared()
    config = configuration()
    (A/'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    overlay = json.loads((BASE/'actors.json').read_bytes())
    (A/'actors.json').write_text(json.dumps(overlay, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    args = arguments()
    configure()
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    files = {p for p in (ROOT/'src').rglob('*') if p.is_file()
             and p.suffix in ('.py', '.yaml', '.xml', '.xacro', '.sdf', '.cpp', '.hpp', '.stl')}
    files.update((ROOT/'scripts').glob('*.py'))
    # 실행 전 parent가 공통 src(r362 포함)의 변경 종료/동결을 검토한다.
    # 검증에 사용한 테스트 원문까지 이번 실행의 소스 archive에 남긴다.
    files.update((ROOT/'tests').rglob('*.py'))
    files.update((A.parent/'released_crawl_r361_v1').glob('*.py'))
    frozen = json.loads((A.parent/'current_deadline_r325_v1/runtime_source_manifest_complete.json').read_bytes())
    files.update(ROOT/p for p in frozen)
    for folder in (A, A.parent/'current_ros_bridge_r336_v1', A.parent/'current_deceleration_r334_v1',
                   A.parent/'current_deadline_r325_v1', A.parent/'current_phase_r323_v1',
                   A.parent/'current_solo_low_r343_v1', A.parent/'current_solo_gap_r346_v1',
                   A.parent/'solo_crawl_retry_r352_v1', A.parent/'current_corner_r370_v1',
                   A.parent/'released_current_r375_v1'):
        files.update(p for p in folder.rglob('*') if p.is_file() and p.suffix in ('.py', '.json')
                     and p.stat().st_size < 1000000)
    files.add(ROOT/'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml')
    files.update([F/'fixture.json', F/'support_frame_m.stl', F/'supports_trial.sdf',
                  A.parent/'old610_common_corner_r333_v1/run.py',
                  ROOT/'artifacts/validation/2026-11-12/signal_post_revision_r291_v1/prepare_fixture.py',
                  ROOT/'config/tests/c1_standard10_native_angles_v1.json'])
    hashes = {p.relative_to(ROOT).as_posix(): sha(p) for p in sorted(files)}
    with zipfile.ZipFile(A/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in hashes: archive.write(ROOT/name, name)
    registration = dict(at_utc=datetime.now(timezone.utc).isoformat(), args=args, input_sha256=hashes,
        source_archive_sha256=sha(A/'sources.zip'), parent=str(BASE.relative_to(ROOT)),
        scope=config['scope'], source_timestamps_unchanged=True, ground_truth_evaluation_only=True,
        geometry_assumptions=['220mm actor height; lower actors not covered', 'r326 facility positions provisional'],
        acceptance='original r282 gap/surface/collision/STOP/full-lap checks unchanged; failures retained',
        causal_comparison=False, default_changed=False, independent_guard_bypassed=False)
    registration['prepared_comparison'] = json.loads((A/'prepared.json').read_bytes())
    registration['selected_plan_policy'] = dict(policy='r361',
        sha256=sha(A.parent/'released_crawl_r361_v1/candidate.py'),
        behavior_changes='r375 current RELEASED parallel relative-body check; r371 steering/geometry/speed unchanged',
        current_policy='r375_released_relative_body', original_trial='relative_body_cad_drive_r371_v1',
        child_installation_evidence='worker_policy_receipts/plan_*_installed.json')
    (A/'registered.json').write_text(json.dumps(registration, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    code = None
    try:
        sys.argv = args
        code = target.main()
        return code
    finally:
        (A/'integrity.json').write_text(json.dumps(dict(validation_returncode=code,
            changed_inputs=[name for name, digest in hashes.items() if not (ROOT/name).is_file() or sha(ROOT/name) != digest],
            input_count=len(hashes), archive_unchanged=sha(A/'sources.zip')==registration['source_archive_sha256']), indent=2)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
