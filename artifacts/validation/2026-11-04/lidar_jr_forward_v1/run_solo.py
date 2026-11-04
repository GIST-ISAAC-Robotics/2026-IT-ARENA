"""11/3 선정 단독 1랩 등록 인자에 센서 장착 YAML만 바꿔 실행한다. 자동 재실행하지 않는다.

--profile 없음 = 기본 b0183_c1.yaml(R, 같은 빌드 대조). --max-sim-seconds는 짧은 실패 확인용.
"""
import argparse
from copy import deepcopy
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PRIOR = ROOT/'artifacts/validation/2026-11-03/solo_post_free_v1/run1_registered.json'
ENV = ROOT/'artifacts/validation/2026-11-01/turn_preview_v1/environment.sh'
BASELINE = HERE.parent/'lidar_mount_speed_v4'/'jr50_2_registered.json'
SOURCES = ('src/arena_autonomy/arena_autonomy/local_scene.py', 'src/arena_autonomy/arena_autonomy/local_path.py',
           'src/arena_autonomy/arena_autonomy/local_pursuit.py', 'src/arena_autonomy/arena_autonomy/lidar_safety.py',
           'src/arena_bringup/launch/simulation.launch.py', 'scripts/validate_basic_autonomy.py',
           'src/arena_description/config/b0183_c1.yaml', 'src/arena_description/models/arena_car/model.sdf.xacro',
           'config/tests/lidar_mount_adapted_r2_sha256.json',
           'src/arena_autonomy/arena_autonomy/known_structure.py',
           'src/arena_autonomy/arena_autonomy/partial_observation.py',
           'src/arena_autonomy/arena_autonomy/local_follow.py',
           'config/tests/lidar_mount_pilot_r3_sha256.json',
           'config/tests/lidar_mount_speed_r4_sha256.json',
           'tests/test_structure_route_speed.py', 'tests/test_source_provenance_r0_plus_b3.py',
           'scripts/validate_basic_autonomy.py', 'src/arena_bringup/launch/demo.launch.py',
           'src/arena_bringup/launch/local_pursuit.launch.py', 'src/arena_bringup/launch/g1_follow.launch.py')


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def register(output, profile, max_sim):
    prior = json.loads(PRIOR.read_bytes())
    baseline = json.loads(BASELINE.read_bytes())
    args = list(prior['args'])
    assert args[-2] == '--output' and '--local-sensor-config' not in args
    args[-1] = str(output)
    if max_sim is not None:
        args[args.index('--max-sim-seconds')+1] = str(max_sim)
    if profile:
        args[-2:-2] = ['--local-sensor-config', str(profile)]
    args += ['--pilot-marker-z', '0.140']
    files = set(SOURCES) | {p for p in prior['source_sha256'] if (ROOT/p).is_file()}
    files |= {p.relative_to(ROOT).as_posix() for p in HERE.glob('*.py')}
    if profile:
        files.add(Path(profile).resolve().relative_to(ROOT).as_posix())
        data = yaml.safe_load(Path(profile).read_text(encoding='utf-8'))
        reference = yaml.safe_load(Path(baseline['profile']).read_text(encoding='utf-8'))
        lidar = data['lidar']
        expected = deepcopy(reference)
        name = Path(profile).name
        expected['lidar']['xyz_m'][0] = .060
        if name == 'jr60p20.yaml':
            expected['lidar']['scan_plane_posts'][0][0] = .020
            expected['lidar']['scan_plane_posts'][1][0] = .020
        elif name != 'jr60.yaml':
            raise ValueError('pilot expects jr60.yaml or jr60p20.yaml')
        if data != expected:
            raise ValueError('profile differs from v4 JR50 in fields beyond approved coordinates')
        posts = {tuple(post[:2]) for post in lidar.get('scan_plane_posts', []) if post[0] < 0}
        wires = {tuple(wire[:2]) for wire in lidar.get('scan_plane_wires', [])}
        if wires - posts:
            raise ValueError(f'synthetic rear wires must share rear-post centers: {sorted(wires - posts)}')
        expected_args = list(baseline['args'])
        expected_args[expected_args.index('--local-sensor-config') + 1] = str(profile)
        expected_args[expected_args.index('--output') + 1] = str(output)
        if args != expected_args:
            raise ValueError('launch arguments differ from v4 JR50 beyond output/profile paths')
    source_hashes = {p: digest(ROOT/p) for p in sorted(files)}
    baseline_runtime = {p: value for p, value in baseline['source_sha256'].items()
                        if p.startswith(('src/', 'scripts/', 'install/'))}
    current_runtime = {p: value for p, value in source_hashes.items()
                       if p.startswith(('src/', 'scripts/', 'install/'))}
    if current_runtime != baseline_runtime:
        changed = sorted(p for p in set(current_runtime) | set(baseline_runtime)
                         if current_runtime.get(p) != baseline_runtime.get(p))
        raise ValueError(f'runtime sources differ from v4 JR50: {changed}')
    return dict(args=args, prior_registered_sha256=digest(PRIOR), profile=profile,
                source_sha256=source_hashes,
                baseline_registered_sha256=digest(BASELINE),
                scope='v4 JR50 same launch inputs, world and controller. Only approved lidar x / optional front-post x differs; 75 mm scan plane and marker/wire geometry retained.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('name')
    parser.add_argument('--profile', default='')
    parser.add_argument('--max-sim-seconds', type=float)
    parser.add_argument('--prepare-only', action='store_true')
    options = parser.parse_args()
    output = HERE/options.name
    registered = HERE/f'{options.name}_registered.json'
    if output.exists():
        raise FileExistsError('기존 실행')
    profile = str((HERE/'profiles'/options.profile).resolve(strict=True)) if options.profile else ''
    data = register(output, profile, options.max_sim_seconds)
    if options.prepare_only:
        with registered.open('x', encoding='utf-8', newline='\n') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
    else:
        if data != json.loads(registered.read_bytes()):
            raise ValueError('등록 후 입력 변경')
        with (HERE/f'{options.name}_console.log').open('x') as stream:
            result = subprocess.run(['bash', str(ENV), sys.executable, '-B', *data['args']], cwd=ROOT,
                                    stdout=stream, stderr=subprocess.STDOUT)
        raise SystemExit(result.returncode)
