"""빠른 앞차 급정지 각본을 기준(closing)과 후보(low_motion)로 한 번씩 실행한다."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
from g1_handoff_eval import launch_arguments
from validate_g1_traffic import load_config, source_paths

ENV = BASE.parent / 'turn_preview_v1/environment.sh'
CONFIG = BASE / 'fast_stop.json'


def register(arm):
    cfg = load_config(CONFIG)
    launch_arguments('g1g2', arm, cfg['launch'], True)
    args = [str(BASE / 'fast_stop_entry.py'), '--mode', 'g1g2', '--handoff-reserve', arm, '--config', str(CONFIG),
            '--case', 'stop_restart', '--seed', '0', '--record-follow-decisions', '--output', str(BASE / ('fast_stop_' + arm))]
    paths = set(source_paths(CONFIG))
    paths.update(p.relative_to(ROOT).as_posix() for p in (ROOT / 'src/arena_autonomy/arena_autonomy').glob('*.py'))
    paths.update(p.relative_to(ROOT).as_posix() for p in (Path(__file__).resolve(), BASE / 'fast_stop_entry.py', ENV))
    return dict(args=args, source_sha256={p: sha256((ROOT / p).read_bytes()).hexdigest() for p in sorted(paths)},
                scope='같은 파생 각본의 기준·후보 비교. 고정 여유 적용 조건만 다르다. 소스 변경 후 재사용 금지.')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('arm', choices=('closing', 'low_motion'))
    p.add_argument('--prepare-only', action='store_true')
    a = p.parse_args()
    name = 'fast_stop_' + a.arm
    reg = BASE / (name + '_registered.json')
    if (BASE / name).exists():
        raise FileExistsError('실행 원본 덮어쓰기 금지')
    data = register(a.arm)
    if a.prepare_only:
        with reg.open('x', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    else:
        if json.loads(reg.read_bytes()) != data:
            raise ValueError('실행 등록 후 변경')
        with (BASE / (name + '_console.log')).open('x') as f:
            r = subprocess.run(['bash', str(ENV), sys.executable, '-B', *data['args']], cwd=ROOT, stdout=f,
                               stderr=subprocess.STDOUT)
        with (BASE / (name + '_exit.log')).open('x') as f:
            f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
