"""공개 50 mm 방지턱의 저속 단독 시험. 운영 기본값은 변경하지 않는다."""
import argparse
import difflib
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import types
import zipfile
from trial_contract import route_contract, validate_result

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
SIGNAL = ROOT / 'artifacts/validation/2026-10-27/official_departure_v2'
FIXTURE = SIGNAL / 'fixture'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def patch_evaluator(original):
    replacements = [
        ('{"red", "yellow", "green"}.issubset(signal_states)',
         '{"red", "green"}.issubset(signal_states)'),
        ("args.speed_profile != 'g1_compare_1mps'):",
         "args.speed_profile not in ('g1_compare_1mps', 'cautious', 'brisk')):"),
        ("or args.lidar_acquisition != 'sequential' or not args.disable_speed_bump):",
         "or args.lidar_acquisition != 'sequential'):"),
        ('    drive_stamps = []',
         '    drive_stamps = []\n    bump_poses, bump_imu, bump_scans = [], [], []'),
        ('        state[key] = message', '''        state[key] = message
        if key == 'imu':
            bump_imu.append(dict(time=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
                orientation=[getattr(message.orientation, k) for k in ('x','y','z','w')],
                angular_velocity=[getattr(message.angular_velocity, k) for k in ('x','y','z')],
                linear_acceleration=[getattr(message.linear_acceleration, k) for k in ('x','y','z')]))'''),
        ('        state["scan"] = message', '''        state["scan"] = message
        bump_scans.append(dict(time=message.header.stamp.sec + message.header.stamp.nanosec * 1e-9,
            angle_min=message.angle_min, angle_increment=message.angle_increment,
            time_increment=message.time_increment, scan_time=message.scan_time,
            range_min=message.range_min, range_max=message.range_max,
            ranges=[float(v) if math.isfinite(v) else str(v) for v in message.ranges]))'''),
        ('                            state["pose"] = pose', '''                            bump_poses.append(dict(time=pose['time'], position=pose['position'],
                                                   orientation=pose['orientation']))
                            state["pose"] = pose'''),
        ('            trace.append(record)', '''            trace.append(record)
            # 평가용 사진 저장 조건이며 제어 노드에 정답 자세를 전달하지 않는다.
            bump = scene['speed_bumps']['bumps'][0]
            if math.hypot(position[0]-bump['x'], position[1]-bump['y']) < .7:
                label = 'bump_rgb_' + str(int(now * 4))
                if label not in pictures:
                    cv2.imwrite(str(output / (label + '.png')),
                                cv2.cvtColor(image_array(state['rgb']), cv2.COLOR_RGB2BGR))
                    pictures.add(label)'''),
        ('        (output / "trace.json").write_text', '''        (output / 'bump_pose_trace.json').write_text(json.dumps(bump_poses), encoding='utf-8')
        (output / 'bump_imu_trace.json').write_text(json.dumps(bump_imu), encoding='utf-8')
        (output / 'bump_scan_trace.json').write_text(json.dumps(bump_scans), encoding='utf-8')
        (output / "trace.json").write_text'''),
    ]
    result = original
    for old, new in replacements:
        if result.count(old) != 1:
            raise ValueError('evaluator signature mismatch: ' + old[:100])
        result = result.replace(old, new)
    compile(result, 'effective_bump_evaluator.py', 'exec')
    return result


def build_launch(output, profile='cautious'):
    if profile not in ('cautious', 'brisk'):
        raise ValueError('unregistered bump speed profile')
    spec = importlib.util.spec_from_file_location('departure_adapter', SIGNAL / 'run.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    launch, settings, args = module.wrapper_launch(output)
    # 기존 launch 생성 결과만 바꾼다. 운영 파일 자체는 편집하지 않는다.
    launch = launch.replace("'speed_bump': 'false'", "'speed_bump': 'true'")
    launch = launch.replace("'speed_profile': 'g1_compare_1mps'", "'speed_profile': " + repr(profile))
    launch = launch.replace(str(SIGNAL / 'signal_driver.py'), str(SIGNAL / 'signal_driver_v2.py'))
    args.update(speed_bump='true', speed_profile=profile)
    compile(launch, 'bump_trial.launch.py', 'exec')
    return launch, settings, args


def prepare(out, profile, scope, laps=1):
    if out.parent != BASE or out.exists():
        raise ValueError('fresh direct child run directory required')
    manifest = json.loads((FIXTURE / 'manifest.json').read_text(encoding='utf-8'))
    scene = json.loads((FIXTURE / 'scene.json').read_text(encoding='utf-8'))
    contract = route_contract(scope, laps, float(scene['track']['lap_length_m']))
    for relative, digest in manifest['sources'].items():
        if sha(ROOT / relative) != digest:
            raise ValueError('fixture source changed: ' + relative)
    source = ROOT / 'scripts/validate_basic_autonomy.py'
    original = source.read_text(encoding='utf-8')
    changed = patch_evaluator(original)
    launch, settings, args = build_launch(out, profile)
    prefix = BASE / out.name
    launch_path = prefix.with_suffix('.launch.py')
    diff_path = BASE / (out.name + '_evaluator.diff')
    for path, text in [(launch_path, launch), (diff_path, ''.join(difflib.unified_diff(
            original.splitlines(True), changed.splitlines(True), fromfile='base_evaluator',
            tofile='bump_trial_evaluator')))]:
        with path.open('x', encoding='utf-8') as f:
            f.write(text)
    paths = {Path(__file__), source, launch_path, diff_path,
             SIGNAL / 'run.py', SIGNAL / 'signal_driver.py', SIGNAL / 'signal_driver_v2.py',
             SIGNAL / 'native/build/material_client', *FIXTURE.rglob('*'),
             *ROOT.glob('src/**/*.py'), *ROOT.glob('src/**/*.yaml'), *ROOT.glob('src/**/*.xacro'),
             *ROOT.glob('scripts/*.py'), *BASE.glob('*.py'), BASE / 'README.md'}
    hashes = {str(p.relative_to(ROOT)): sha(p) for p in sorted(paths) if p.is_file() and '__pycache__' not in p.parts}
    with zipfile.ZipFile(BASE / (out.name + '_trial_sources.zip'), 'x', zipfile.ZIP_DEFLATED) as z:
        for relative in hashes:
            z.write(ROOT / relative, relative)
        z.writestr('effective_bump_evaluator.py', changed)
    metadata = dict(controller_parameters=settings, launch_arguments=args, source_sha256=hashes, trial_scope=scope,
                    route_contract=contract,
                    output=str(out), launch_path=str(launch_path), fixture_manifest=manifest,
                    scope='single car, published 50mm collision approximation; cautious 0.35 or brisk 0.70m/s command cap; no hardware proof')
    with (BASE / (out.name + '_prepared.json')).open('x', encoding='utf-8') as f:
        json.dump(metadata, f, ensure_ascii=False, indent=2)
    return metadata


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--speed-profile', choices=('cautious','brisk'), default='cautious')
    parser.add_argument('--scope', choices=('crossing','course'), default='crossing')
    parser.add_argument('--laps', type=int, default=1)
    options = parser.parse_args()
    out = options.output.resolve()
    metadata_path = BASE / (out.name + '_prepared.json')
    if options.prepare_only:
        result = prepare(out, options.speed_profile, options.scope, options.laps)
        print(json.dumps(dict(prepared=True, files=len(result['source_sha256']), output=str(out))))
        return 0
    if out.parent != BASE or out.exists():
        raise ValueError('fresh prepared run required')
    metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
    if metadata['output'] != str(out):
        raise ValueError('prepared output mismatch')
    if metadata['launch_arguments']['speed_profile'] != options.speed_profile:
        raise ValueError('prepared speed profile mismatch')
    if metadata['trial_scope'] != options.scope:
        raise ValueError('prepared route scope mismatch')
    scene = json.loads((FIXTURE / 'scene.json').read_text(encoding='utf-8'))
    contract = route_contract(options.scope, options.laps, float(scene['track']['lap_length_m']))
    if metadata.get('route_contract') != contract:
        raise ValueError('prepared explicit route contract missing or changed; preserve old trial, prepare a new one')
    for relative, digest in metadata['source_sha256'].items():
        if sha(ROOT / relative) != digest:
            raise ValueError('prepared source changed: ' + relative)
    source = ROOT / 'scripts/validate_basic_autonomy.py'
    changed = patch_evaluator(source.read_text(encoding='utf-8'))
    for relative in ('scripts', 'src/arena_autonomy', 'src/arena_vehicle_interface'):
        sys.path.insert(0, str(ROOT / relative))
    evaluator = types.ModuleType('bump_trial_evaluator')
    evaluator.__file__ = str(source)
    sys.modules[evaluator.__name__] = evaluator
    exec(compile(changed, str(source), 'exec'), evaluator.__dict__)
    evaluator.DEMO_WORLD = FIXTURE
    def start(log, *pos, **kw):
        return subprocess.Popen(['ros2', 'launch', metadata['launch_path']], cwd=ROOT,
            stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
    evaluator.start_demo_process = start
    sys.argv = [str(source), '--output', str(out), '--actuation-mode', 'virtual_mcu', '--g1-follow',
        '--record-follow-decisions', '--g1-boundary-evidence', 'corner_distance_candidate',
        '--g1-occupancy-timing', 'per_ray_candidate', '--g1-corridor-fallback',
        '--partial-observation-policy', 'bounded_local_v4', '--autonomy-mode', 'local_pursuit',
        '--grid-slot', '3', '--speed-profile', options.speed_profile, '--lidar-acquisition', 'sequential',
        '--lidar-compensation', 'both', '--lidar-rate-hz', '10', '--control-rate-hz', '50',
        '--render-backend', 'wsl_nvidia', '--sensor-wall-timeout-s', '30',
        '--max-sim-seconds', '300' if options.scope=='course' else '60',
        '--wall-timeout-s', '3600' if options.scope=='course' else '1800', '--stop-wall-timeout-s', '180',
        '--shutdown-mode', 'service']
    sys.argv += contract['arguments']
    code = evaluator.main()
    report = json.loads((out/'report.json').read_text(encoding='utf-8'))
    metadata['route_contract_check'] = validate_result(contract, report)
    metadata.update(argv=sys.argv, base_exit=code, source_unchanged=all(
        sha(ROOT / r) == h for r, h in metadata['source_sha256'].items()))
    (out / 'trial_overlay.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    if not metadata['source_unchanged']:
        raise RuntimeError('source changed during trial')
    return code if metadata['route_contract_check']['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
