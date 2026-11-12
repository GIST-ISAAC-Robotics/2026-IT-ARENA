"""확정 구형610 지주/CAD 배치에서 r336 ROS 인계 후보의 제한 교통1회."""
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
BASE = ROOT/'artifacts/validation/2026-11-12/current_region_runtime_r282_v1/two_actor_run1'
F = A.parent/'old_signal_610_r326_v1'
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'tests')]
import validate_g1_traffic as target
from launch_prefix import instrument_start


def configure():
    # 평가용 형상은 앞선 r333과 같은 실제 차체 높이 slab를 사용한다.
    # 이 함수의 경로/정답은 제어기로 전송하지 않는다.
    import importlib.util
    path = A.parent/'old610_common_corner_r333_v1/run.py'
    spec = importlib.util.spec_from_file_location('r349_previous_shape', path)
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
            model.set('name', 'eval_r349_'+model.get('name'))
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
    parent = json.loads((BASE/'registered.json').read_bytes())
    config = json.loads((BASE/'recessed_config.json').read_bytes())
    config['scope'] = 'r346 정지 근처 SOLO 기존 공백 검사 연결 + r347 확정 CAD 센서 구조. r336 ROS 인계·확정 구형610/CAD·220mm 상대 두 대·80초 1회. 지연 무주입/RTF0.25이며 실시간/실차 증명이 아니다.'
    config['launch']['entry_g1'] = 'r349 course.launch.py plus explicit r346 current policy'
    config['actor']['height_m'] = .220
    config['actor']['visible_section'] = '상대 높이220mm는 팀 차량과 비슷한 높이의 시험 가정. C1 Z134.6mm가 가로지르며 더 낮은 상대를 대신 검증하지 않는다.'
    config['vehicle_and_sensor_unchanged'] = '현재 CAD C1 X75.4/Z134.6mm·새 지주·전방배선6mm·마스크 움직임3mm 여유; 축거/윤거135/135mm 전체 기구는 아직 기존 근사, B0390축198.363mm. 단일모터28count·2.5*81/17·65mm 유지.'
    (A/'config.json').write_text(json.dumps(config, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    overlay = json.loads((BASE/'additional_actors.json').read_bytes())
    (A/'actors.json').write_text(json.dumps(overlay, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    args = list(parent['args'])
    for flag, value in [('--config', str(A/'config.json')), ('--actor-overlay', str(A/'actors.json')),
                        ('--output', str(out)), ('--local-sensor-config',
                         str(ROOT/'src/arena_description/config/b0390_c1_cad_20261113_final_frontwire_6mm.yaml'))]:
        args[args.index(flag)+1] = value
    configure()
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    files = {p for p in (ROOT/'src').rglob('*') if p.is_file()
             and p.suffix in ('.py', '.yaml', '.xml', '.xacro', '.sdf', '.cpp', '.hpp', '.stl')}
    files.update((ROOT/'scripts').glob('*.py'))
    frozen = json.loads((A.parent/'current_deadline_r325_v1/runtime_source_manifest_complete.json').read_bytes())
    files.update(ROOT/p for p in frozen)
    for folder in (A, A.parent/'current_ros_bridge_r336_v1', A.parent/'current_deceleration_r334_v1',
                   A.parent/'current_deadline_r325_v1', A.parent/'current_phase_r323_v1',
                   A.parent/'current_solo_low_r343_v1', A.parent/'current_solo_gap_r346_v1'):
        files.update(p for p in folder.rglob('*') if p.is_file() and p.suffix in ('.py', '.json')
                     and p.stat().st_size < 1000000)
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
