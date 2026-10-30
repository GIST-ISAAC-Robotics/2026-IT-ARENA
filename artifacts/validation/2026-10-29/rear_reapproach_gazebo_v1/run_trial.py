"""후방 사각 소실·재접근의 공식 직선 제한 비교. 후보 OFF/ON의 실행 입력만 달리한다.

기존 운영 파일/판정선은 수정하지 않는다. 모든 준비 파일은 새 이름으로 보존한다.
"""
import argparse
import difflib
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
import subprocess
import sys
import types
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
SIGNAL = ROOT / 'artifacts/validation/2026-10-27/official_departure_v2'
FIXTURE = SIGNAL / 'fixture'
SIDE = 'left'
CONFIG = BASE / 'left_config.json'
OVERLAY = BASE / 'left_overlay.json'
CANDIDATE = False
GRID_SLOT = 3


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def replace_once(text, before, after):
    if text.count(before) != 1:
        raise ValueError('source signature changed: ' + before[:100])
    return text.replace(before, after)


def effective_sources():
    evaluator = (ROOT / 'scripts/g1_traffic_eval.py').read_text(encoding='utf-8')
    harness = (ROOT / 'scripts/validate_g1_traffic.py').read_text(encoding='utf-8')
    new_eval = replace_once(evaluator,
        "{'red', 'yellow', 'green'}.issubset(report.get('signal_states', []))",
        "{'red', 'green'}.issubset(report.get('signal_states', []))")
    needle = "            light = state['light']\n"
    new_harness = replace_once(harness, needle, needle +
        "            if (output/'signal_driver_failure.json').exists():\n"
        "                report['error'] = 'signal_driver_failure: see recorded receipt'\n"
        "                break\n")
    new_harness = replace_once(new_harness,
        "                raise TimeoutError('green not observed in time')",
        "                report['error'] = 'TimeoutError: green not observed in time'\n                break")
    new_harness = replace_once(new_harness,
        "        state['autonomy'] = status\n",
        "        state['autonomy'] = status\n"
        "        status['observed_vision_mode'] = status.get('vision', {}).get('mode')\n"
        "        status['observed_signal_mode'] = status.get('vision', {}).get('signal_mode')\n"
        "        status['observed_race_supervisor'] = status.get('race') is not None\n")
    return [(evaluator, new_eval, 'g1_traffic_eval.py'),
            (harness, new_harness, 'validate_g1_traffic.py')]


def launch_source(output):
    spec = importlib.util.spec_from_file_location('official_departure_base', SIGNAL / 'run.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    _, settings, arguments = module.wrapper_launch(output)
    settings.update(g1_traffic_recovery_trial=True, g1_traffic_path_recovery_trial=True,
                    start_signal_mode='official_4red_local_candidate',
                    g1_rear_lane_keep_trial=CANDIDATE)
    arguments.update(evaluation_actor_sdf=str(output / 'evaluation_actor.sdf'), grid_slot=str(GRID_SLOT))
    rows = [f'SetParameter(name={key!r},value={value!r})' for key, value in settings.items()]
    rows += ['IncludeLaunchDescription(PythonLaunchDescriptionSource(' +
             repr(str(FIXTURE / 'simulation.launch.py')) + '),launch_arguments=' +
             repr(arguments) + '.items())',
             'ExecuteProcess(cmd=' + repr(['/usr/bin/python3', '-B',
             str(SIGNAL / 'signal_driver_v2.py'), str(output)]) + ',output="screen")']
    text = ('from launch import LaunchDescription\n'
            'from launch.actions import IncludeLaunchDescription, ExecuteProcess\n'
            'from launch.launch_description_sources import PythonLaunchDescriptionSource\n'
            'from launch_ros.actions import SetParameter\n'
            'def generate_launch_description():\n    return LaunchDescription([' + ',\n        '.join(rows) + '])\n')
    compile(text, 'official_traffic.launch.py', 'exec')
    return text, settings, arguments


def validate_launch_call(effective, settings, arguments, output):
    expected = dict(speed_profile='g1_compare_1mps', lidar_rate_hz=10, lidar_acquisition='sequential',
                    lidar_compensation='both', control_rate_hz=50, grid_slot=GRID_SLOT,
                    autonomy_mode='local_pursuit', actuation_mode='virtual_mcu',
                    speed_bump=False, collision_detector='configured', g1_follow=True,
                    r0_plus=False, lidar_test_profile='', motion_test_profile='',
                    record_inputs=False, runtime_artifacts=str(output))
    if any(effective.get(k) != v for k, v in expected.items()):
        raise ValueError('requested launch differs from registered official traffic trial')
    requested = {}
    for item in effective.get('extra_arguments', ()):
        key, value = item.split(':=', 1)
        if key in requested:
            raise ValueError('duplicate requested parameter: ' + key)
        requested[key] = value
    required = {'evaluation_actor_sdf', 'partial_observation_policy', 'g1_parallel_hold_trial',
        'g1_pass_release_trial', 'g1_guard_curve_trial', 'g1_traffic_recovery_trial',
        'g1_traffic_path_recovery_trial', 'g1_boundary_evidence_mode', 'g1_trace_path',
        'g1_occupancy_timing_mode', 'g1_corridor_fallback_enabled',
        'g1_recent_observation_enabled', 'g1_short_boundary_prefix_enabled'}
    if set(requested) != required:
        raise ValueError('missing or unexpected trial parameter')
    for key, value in requested.items():
        actual = settings.get(key, arguments.get(key))
        actual = str(actual).lower() if isinstance(actual, bool) else str(actual)
        if actual != value:
            raise ValueError('launch parameter mismatch: ' + key)
    return True


def write_new(path, content):
    with path.open('x', encoding='utf-8') as stream:
        stream.write(content)


def prepare(output):
    if output.parent != BASE or output.exists():
        raise ValueError('fresh direct child output required')
    manifest = json.loads((FIXTURE / 'manifest.json').read_text(encoding='utf-8'))
    if any(sha(ROOT / p) != digest for p, digest in manifest['sources'].items()):
        raise ValueError('official signal fixture input changed')
    launch, settings, arguments = launch_source(output)
    launch_path = BASE / (output.name + '.launch.py')
    write_new(launch_path, launch)
    changes = effective_sources()
    patches = []
    for original, changed, filename in changes:
        compile(changed, filename, 'exec')
        patch = BASE / (output.name + '_' + filename + '.diff')
        write_new(patch, ''.join(difflib.unified_diff(original.splitlines(True), changed.splitlines(True),
            fromfile='original/' + filename, tofile='official_traffic/' + filename)))
        patches.append(patch)
    paths = {p for folder in ('src', 'scripts', 'config', 'tests') for p in (ROOT / folder).rglob('*')
             if p.is_file() and '__pycache__' not in p.parts and p.suffix not in ('.pyc', '.pyo')}
    paths.update(p for p in FIXTURE.rglob('*') if p.is_file())
    paths.update(BASE.glob('*.py'))
    paths.update([CONFIG, OVERLAY, BASE / 'DRIVE_PLAN.md', BASE / 'preparation.json'])
    paths.update([Path(__file__), SIGNAL / 'run.py', SIGNAL / 'signal_driver.py',
        SIGNAL / 'signal_driver_v2.py', SIGNAL / 'native/build/material_client', launch_path, *patches])
    before = {p.relative_to(ROOT).as_posix(): sha(p) for p in sorted(paths)}
    with zipfile.ZipFile(BASE / (output.name + '_sources.zip'), 'x', zipfile.ZIP_DEFLATED) as archive:
        for relative in before:
            archive.write(ROOT / relative, relative)
        for _, changed, filename in changes:
            archive.writestr('effective_' + filename, changed)
    metadata = dict(side=SIDE, candidate=CANDIDATE, source_sha256=before, output=str(output), launch_path=str(launch_path),
        controller_parameters=settings, launch_arguments=arguments, fixture=manifest,
        scope='12s alongside-to-rear-blind-to-reapproach box actor; registered straddle route and historical gates preserved; candidate default OFF; not independent vehicles or hardware proof',
        route=dict(case='slow_lead', duration_s=12, minimum_laps=None,
                   config=str(CONFIG), actor_overlay=str(OVERLAY), seed=0))
    write_new(BASE / (output.name + '_prepared.json'), json.dumps(metadata, ensure_ascii=False, indent=2))
    return metadata


def verify_runtime(output, metadata, report):
    """요청값뿐 아니라 실제 판단기 헤더·신호 ACK·영상 출발 결과를 확인한다."""
    with (output / 'follow_decisions.jsonl').open(encoding='utf-8') as stream:
        header = json.loads(next(stream))['data']['follow_config']
    names = dict(g1_follow_enabled=None, g1_trace_path=None, race_control_mode=None,
                 start_signal_mode=None, partial_observation_policy='partial_observation_policy')
    for key in metadata['controller_parameters']:
        if key.startswith('g1_') and key not in names:
            names[key] = key[3:]
    mismatches = {key: dict(expected=value, actual=header.get(names[key]))
                  for key, value in metadata['controller_parameters'].items()
                  if names.get(key) is not None and header.get(names[key]) != value}
    sequence = [json.loads(line) for line in (output / 'signal_sequence.jsonl').read_text(encoding='utf-8').splitlines()]
    stages = [row['stage'] for row in sequence if 'stage' in row]
    receipts = [receipt for row in sequence if 'stage' in row for receipt in row.get('receipts', [])]
    expected_stages = ['green_probe', 'off', 'red1', 'red2', 'red3', 'red4', 'green']
    # follow_config에는 영상/운영 모드가 없으므로 실제 상태 토픽의 작은 필드만 별도 보존한다.
    # 평가기만 이 정보를 읽으며 제어 신호로 되돌리지 않는다.
    with (output / 'autonomy_status.jsonl').open(encoding='utf-8') as stream:
        statuses = [json.loads(line) for line in stream]
    expected_signal = metadata['controller_parameters']['start_signal_mode']
    vision_modes = {row.get('observed_vision_mode') for row in statuses}
    signal_modes = {row.get('observed_signal_mode') for row in statuses}
    legacy_rows = all(row.get('observed_race_supervisor') is False for row in statuses)
    checks = dict(actual_follow_parameters=not mismatches,
        actual_vision_mode=bool(statuses) and vision_modes == {'process'},
        actual_signal_mode=bool(statuses) and signal_modes == {expected_signal},
        actual_legacy_operation=bool(statuses) and legacy_rows,
        exact_applied_stages=stages == expected_stages,
        all_lamp_acks=len(receipts) == 70 and all(all(r.get(k) is True for k in
            ('parsed', 'executed', 'service_result', 'applied')) for r in receipts),
        no_signal_failure=not (output / 'signal_driver_failure.json').exists(),
        no_false_start=report.get('false_start') is False)
    return dict(passed=all(checks.values()), checks=checks, mismatches=mismatches,
                stages=stages, receipt_count=len(receipts), status_rows=len(statuses),
                observed_vision_modes=sorted(vision_modes, key=str),
                observed_signal_modes=sorted(signal_modes, key=str),
                scope='applied traffic options and visual signal fixture; driving verdict remains separate')


def run(output):
    metadata = json.loads((BASE / (output.name + '_prepared.json')).read_text(encoding='utf-8'))
    if output.parent != BASE or output.exists() or metadata['output'] != str(output) or metadata['side'] != SIDE or metadata['candidate'] != CANDIDATE:
        raise ValueError('fresh prepared output required')
    def unchanged():
        return all((ROOT / p).is_file() and sha(ROOT / p) == digest
                   for p, digest in metadata['source_sha256'].items())
    if not unchanged():
        raise ValueError('prepared source changed')
    for relative in ('scripts', 'src/arena_autonomy', 'src/arena_vehicle_interface'):
        sys.path.insert(0, str(ROOT / relative))
    modules = []
    for _, changed, filename in effective_sources():
        module = types.ModuleType('official_traffic_' + Path(filename).stem)
        module.__file__ = str(ROOT / 'scripts' / filename)
        sys.modules[module.__name__] = module
        exec(compile(changed, module.__file__, 'exec'), module.__dict__)
        modules.append(module)
    evaluator, validator = modules
    validator.ev = evaluator
    validator.DEMO_WORLD = FIXTURE
    from straddle_route import wrap_route_check
    validator.check_route = wrap_route_check(validator)
    original_load = validator.load_centerline
    validator.load_centerline = lambda: original_load(FIXTURE)
    original_start = validator.start_demo_process
    def start(log, *pos, **kw):
        bound = inspect.signature(original_start).bind(log, *pos, **kw)
        bound.apply_defaults()
        validate_launch_call(bound.arguments, metadata['controller_parameters'], metadata['launch_arguments'], output)
        return subprocess.Popen(['ros2', 'launch', '--noninteractive', metadata['launch_path']], cwd=ROOT,
            stdin=subprocess.DEVNULL, stdout=log, stderr=log, start_new_session=True)
    validator.start_demo_process = start
    sys.argv = [validator.__file__, '--config', str(CONFIG), '--case', 'slow_lead',
        '--seed', '0', '--mode', 'g1', '--output', str(output), '--actor-overlay', str(OVERLAY),
        '--record-follow-decisions', '--record-braking-diagnostics',
        '--boundary-evidence', 'corner_distance_candidate', '--occupancy-timing', 'per_ray_candidate',
        '--corridor-fallback', '--short-boundary-prefix', '--partial-observation-policy', 'bounded_local_v4',
        '--parallel-hold-trial', '--pass-release-trial', '--guard-curve-trial',
        '--traffic-recovery-trial', '--traffic-path-recovery-trial']
    code = validator.main()
    metadata.update(argv=sys.argv, source_unchanged=unchanged(), evaluator_exit=code)
    report = json.loads((output / 'report.json').read_text(encoding='utf-8'))
    try:
        metadata['runtime_contract'] = verify_runtime(output, metadata, report)
    except (KeyError, ValueError, StopIteration, OSError) as error:
        metadata['runtime_contract'] = dict(passed=False, error=f'{type(error).__name__}: {error}')
    (output / 'trial_overlay.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8')
    return code if metadata['source_unchanged'] and metadata['runtime_contract']['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--side', choices=('left',), default='left')
    parser.add_argument('--candidate', choices=('off','on'), required=True)
    args = parser.parse_args()
    SIDE = args.side
    CANDIDATE = args.candidate == 'on'
    GRID_SLOT = 3 if SIDE == 'left' else 0
    CONFIG = BASE / (SIDE + '_config.json')
    OVERLAY = BASE / (SIDE + '_overlay.json')
    output = args.output.resolve()
    if args.prepare_only:
        result = prepare(output)
        print(json.dumps(dict(prepared=True, files=len(result['source_sha256']), output=str(output))))
    else:
        raise SystemExit(run(output))
