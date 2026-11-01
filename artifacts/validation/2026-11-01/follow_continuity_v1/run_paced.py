"""이동 가정 하한 후보(paced)를 코너·정지·빠른 급정지에서 한 번씩 실행한다."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
LEAD = BASE.parent / 'rolling_lead_v1'
sys.path[:0] = [str(LEAD), str(ROOT / 'scripts')]
from course import CONFIG as COURSE, expected, geometry
from g1_handoff_eval import launch_arguments
from validate_g1_traffic import load_config, source_paths

ENV = BASE.parent / 'turn_preview_v1/environment.sh'
OUTPUTS = dict(curve=LEAD / 'curve_paced', stop=LEAD / 'stop_paced', fast=BASE / 'fast_stop_paced')
CONFIGS = dict(curve=COURSE, stop=BASE.parent / 'race_follow_pair_v1/stop_restart.json', fast=BASE / 'fast_stop.json')


def register(which):
    config = CONFIGS[which]
    output = OUTPUTS[which]
    case = 'first_curve' if which == 'curve' else 'stop_restart'
    cfg = load_config(config)
    launch_arguments('g1g2', 'paced', cfg['launch'], True)
    if which == 'curve' and (cfg != expected() or geometry(cfg) != json.loads((LEAD / 'geometry.json').read_bytes())):
        raise ValueError('원래 코너 각본·판정 변화')
    entry = str(BASE / 'fast_stop_entry.py') if which == 'fast' else 'scripts/validate_g1_traffic.py'
    args = [entry, '--mode', 'g1g2', '--handoff-reserve', 'paced', '--config', str(config), '--case', case,
            '--seed', '0', '--record-follow-decisions', '--output', str(output)]
    inputs = {}
    if which == 'curve':
        from race_traffic_course import inputs as facility
        request = SimpleNamespace(config=config, actor_overlay=None, mode='g1g2', seed=0,
                                  record_follow_decisions=True, output=output, case=case)
        inputs.update(facility(request, cfg, cfg['cases'][0]))
        args.append('--team-signal-course')
    paths = set(source_paths(config))
    paths.update(p.relative_to(ROOT).as_posix() for p in (ROOT / 'src/arena_autonomy/arena_autonomy').glob('*.py'))
    paths.update(p.relative_to(ROOT).as_posix() for p in (Path(__file__).resolve(), ENV, BASE / 'fast_stop_entry.py',
                                                          BASE / 'candidate_replay_v2.json'))
    inputs.update({p: sha256((ROOT / p).read_bytes()).hexdigest() for p in sorted(paths)})
    return dict(args=args, source_sha256=inputs,
                scope='기존 교통 구성에서 전진율 없는 이동 조각의 상한만 바꿈. 소스 변경 후 재사용 금지.')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('case', choices=tuple(OUTPUTS))
    p.add_argument('--prepare-only', action='store_true')
    a = p.parse_args()
    name = a.case + '_paced'
    reg = BASE / (name + '_registered.json')
    if OUTPUTS[a.case].exists():
        raise FileExistsError('실행 원본 덮어쓰기 금지')
    data = register(a.case)
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
