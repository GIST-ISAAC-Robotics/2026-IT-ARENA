"""11/3 선정 단독 1랩 등록 인자에 센서 장착 YAML만 바꿔 실행한다. 자동 재실행하지 않는다.

--profile 없음 = 기본 b0183_c1.yaml(R, 같은 빌드 대조). --max-sim-seconds는 짧은 실패 확인용.
"""
import argparse
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
        lidar = data['lidar']
        posts = {tuple(post[:2]) for post in lidar.get('scan_plane_posts', []) if post[0] < 0}
        wires = {tuple(wire[:2]) for wire in lidar.get('scan_plane_wires', [])}
        if wires - posts:
            raise ValueError(f'synthetic rear wires must share rear-post centers: {sorted(wires - posts)}')
    return dict(args=args, prior_registered_sha256=digest(PRIOR), profile=profile,
                source_sha256={p: digest(ROOT/p) for p in sorted(files)},
                scope='11/3 선정 단독 1랩과 같은 인자·월드·속도 프로필. 공통 구조 적응 정책, 75 mm 광축. 센서 장착 YAML만 다르다.')


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
