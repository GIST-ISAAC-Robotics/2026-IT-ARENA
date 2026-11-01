"""직전 단독 완주 구성의 시간 간격만 0.5초로 비교하는 1회. 자동 재실행하지 않는다."""
import argparse
from hashlib import sha256
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from prelap_signal_eval import fixture_inputs  # noqa: E402

BASE = HERE.parent/'solo_route_v1'
ENV = HERE.parent/'turn_preview_v1/environment.sh'
OLD, NEW = 'g1_open_10kmh_solo', 'g1_open_10kmh_solo_gap05'


def digest(path):
    return sha256(path.read_bytes()).hexdigest()


def register():
    prior = json.loads((BASE/'registered.json').read_bytes())
    spec = importlib.util.spec_from_file_location('registered_speed_profiles', ROOT/'src/arena_bringup/launch/simulation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    old, new = module.SPEED_PROFILES[OLD], module.SPEED_PROFILES[NEW]
    if new != {**old, 'g1_solo_headway_s': .5} or old != prior['trial']:
        raise ValueError('비교 구성 변경은 단독 시간 간격 하나여야 한다')
    args = list(prior['args'])
    index = args.index('--speed-profile')+1
    if args[index] != OLD or args[-2:] != ['--output', str(BASE/'run1')]:
        raise ValueError('기준 실행 인자 변경')
    args[index], args[-1] = NEW, str(HERE/'run1')
    files = set(prior['source_sha256'])
    files.update(p.relative_to(ROOT).as_posix() for p in (
        Path(__file__).resolve(), HERE/'before.zip', BASE/'registered.json', ENV,
        ROOT/'config/tests/solo_headway_r1_sha256.json', ROOT/'tests/test_solo_route_mode.py'))
    sources = fixture_inputs()
    sources.update({p: digest(ROOT/p) for p in sorted(files)})
    changed = sorted(p for p, h in prior['source_sha256'].items() if sources[p] != h)
    return dict(args=args, source_sha256=sources, baseline=old, trial=new, changed_since_solo=changed,
                scope='직전 solo 실행과 같은 2×5·ID45 이동·방지턱 OFF·촬영·감속비·10 Hz 입력의 단독 1회. '
                      'changed_since_solo는 단독 시간 간격 연결과 검사 등록 차이다.')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--prepare-only', action='store_true')
    options = parser.parse_args()
    if (HERE/'run1').exists():
        raise FileExistsError('기존 실행')
    data, registered = register(), HERE/'registered.json'
    if options.prepare_only:
        with registered.open('x', encoding='utf-8', newline='\n') as stream:
            json.dump(data, stream, ensure_ascii=False, indent=2)
    else:
        if data != json.loads(registered.read_bytes()):
            raise ValueError('등록 후 입력 변경')
        with (HERE/'run1_console.log').open('x') as stream:
            result = subprocess.run(['bash', str(ENV), sys.executable, '-B', *data['args']], cwd=ROOT,
                                    stdout=stream, stderr=subprocess.STDOUT)
        with (HERE/'run1_exit.log').open('x') as stream:
            stream.write(f'ACTUAL_EXIT_CODE={result.returncode}\n')
        raise SystemExit(result.returncode)
