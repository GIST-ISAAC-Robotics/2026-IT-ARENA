"""공식 v2026.11.01 파생 월드의 선정 35.281초 구성 단독 대표 1랩. 자동 재실행하지 않는다.

11/1 등록 인자에서 팀 시험본 입력(--team-signal-prelap)만 빼고 출력·빨강 유지 시간만 지정한다.
제어 프로필은 그대로다. 실패 뒤 원인을 고친 재시험은 --attempt로 다른 폴더에 쓴다.
"""
import argparse
from hashlib import sha256
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PRIOR = HERE.parents[1]/'2026-11-01/solo_corner_handoff_v1'
ENV = PRIOR.parent/'turn_preview_v1/environment.sh'
PROFILE = 'g1_open_10kmh_solo_gap05_arc_preview_bridge'
RED4_HOLD_S = 1.6  # 11/1 시험 신호 발생기의 마지막 빨강 유지 시간
NEW_INPUTS = ('assets/track/official/v2026.11.01/it_arena_track_v2026.11.01.zip',
              'config/tracks/official_v2026.11.01.yaml', 'scripts/build_official_track.py',
              'scripts/official_2x5_facilities.py', 'scripts/prelap_signal_eval.py',
              'scripts/validate_basic_autonomy.py', 'src/arena_bringup/launch/simulation.launch.py',
              'src/arena_gazebo/scripts/traffic_light_controller.py', 'src/arena_gazebo/src/visual_material_client.cpp',
              'src/arena_gazebo/CMakeLists.txt', 'tests/test_official_track.py', 'tests/test_official_signal_control.py',
              'install/arena_gazebo/lib/arena_gazebo/visual_material_client')


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def register(output):
    prior = json.loads((PRIOR/'registered.json').read_bytes())
    spec = importlib.util.spec_from_file_location('registered_speed_profiles', ROOT/'src/arena_bringup/launch/simulation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if module.SPEED_PROFILES[PROFILE] != prior['trial']:
        raise ValueError('선정 프로필이 11/1 등록값과 다르다')
    args = [value for value in prior['args'] if value != '--team-signal-prelap']
    if len(args) != len(prior['args'])-1 or args[-2] != '--output' or args[args.index('--speed-profile')+1] != PROFILE:
        raise ValueError('기준 실행 인자 변경')
    args[-1] = str(output)
    args[-2:-2] = ['--red-duration-s', str(RED4_HOLD_S)]
    files = {p for p in prior['source_sha256'] if (ROOT/p).is_file()} | set(NEW_INPUTS)
    files |= {p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_gazebo/worlds/it_arena_official').rglob('*') if p.is_file()}
    files |= {p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
    files |= {Path(__file__).resolve().relative_to(ROOT).as_posix()}
    return dict(args=args, source_sha256={p: digest(ROOT/p) for p in sorted(files)}, trial=PROFILE,
                prior_registered_sha256=digest(PRIOR/'registered.json'),
                missing_from_prior=sorted(p for p in prior['source_sha256'] if not (ROOT/p).is_file()),
                scope='새 공식 파생 월드·공식 2×5 신호의 단독 1랩. 그리드 3, C1 순차 10 Hz, 8.970588:1, 28카운트, 65 mm, 영상 조건은 11/1과 같다.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--attempt', default='run1')
    options = parser.parse_args()
    output = HERE/options.attempt
    registered = HERE/f'{options.attempt}_registered.json'
    if output.exists():
        raise FileExistsError('기존 실행')
    data = register(output)
    if options.prepare_only:
        with registered.open('x', encoding='utf-8', newline='\n') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
    else:
        if data != json.loads(registered.read_bytes()):
            raise ValueError('등록 후 입력 변경')
        with (HERE/f'{options.attempt}_console.log').open('x') as stream:
            result = subprocess.run(['bash', str(ENV), sys.executable, '-B', *data['args']], cwd=ROOT,
                                    stdout=stream, stderr=subprocess.STDOUT)
        with (HERE/f'{options.attempt}_exit.log').open('x') as stream:
            stream.write(f'ACTUAL_EXIT_CODE={result.returncode}\n')
        raise SystemExit(result.returncode)
