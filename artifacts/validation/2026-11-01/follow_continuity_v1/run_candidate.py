"""고정 여유를 낮은 이동 조각에만 쓰는 후보의 코너·정지 비교. 소스·각본을 등록한 실행만 한 번 수행한다."""
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
from course import CONFIG, expected, geometry
from g1_handoff_eval import launch_arguments
from validate_g1_traffic import load_config, source_paths

ENV = BASE.parent / 'turn_preview_v1/environment.sh'


def register(which):
    curve = which == 'curve'
    # 코너 평가기는 등록된 상대 각본 폴더 아래의 출력만 받는다. 정지 각본도 같은 곳에 모은다.
    output = LEAD / (which + '_low_motion')
    config = CONFIG if curve else BASE.parent / 'race_follow_pair_v1/stop_restart.json'
    case = 'first_curve' if curve else 'stop_restart'
    reference = LEAD / 'run1' if curve else BASE.parent / 'race_follow_closing_v1/stop'
    report = json.loads((reference / 'report.json').read_bytes())
    if not report['run_valid'] or not report['source_unchanged'] or report['case'] != case:
        raise ValueError('유효한 기준 실행이 없음')
    cfg = load_config(config)
    launch_arguments('g1g2', 'low_motion', cfg['launch'], True)
    if curve and (cfg != expected() or geometry(cfg) != json.loads((LEAD / 'geometry.json').read_bytes())):
        raise ValueError('원래 코너 각본·판정 변화')
    args = ['scripts/validate_g1_traffic.py', '--mode', 'g1g2', '--handoff-reserve', 'low_motion',
            '--config', str(config), '--case', case, '--seed', '0', '--record-follow-decisions', '--output', str(output)]
    inputs = {}
    if curve:
        from race_traffic_course import inputs as facility
        request = SimpleNamespace(config=config, actor_overlay=None, mode='g1g2', seed=0,
                                  record_follow_decisions=True, output=output, case=case)
        inputs.update(facility(request, cfg, cfg['cases'][0]))
        args.append('--team-signal-course')
    paths = set(source_paths(config))
    paths.update(p.relative_to(ROOT).as_posix() for p in (ROOT / 'src/arena_autonomy/arena_autonomy').glob('*.py'))
    paths.update(p.relative_to(ROOT).as_posix() for p in (Path(__file__).resolve(), ENV, reference / 'report.json',
                                                          BASE / 'candidate_replay_v1.json'))
    inputs.update({p: sha256((ROOT / p).read_bytes()).hexdigest() for p in sorted(paths)})
    return dict(args=args, source_sha256=inputs, reference=reference.relative_to(ROOT).as_posix(),
                scope='기존 1.5 m/s·간격·노면·STOP·상대 각본 유지. 고정 여유 적용 조건만 바꿈. 소스 변경 후 재사용 금지.')


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('case', choices=('curve', 'stop'))
    p.add_argument('--prepare-only', action='store_true')
    a = p.parse_args()
    reg = BASE / (a.case + '_registered.json')
    if (LEAD / (a.case + '_low_motion')).exists():
        raise FileExistsError('실행 원본 덮어쓰기 금지')
    data = register(a.case)
    if a.prepare_only:
        with reg.open('x', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    else:
        if json.loads(reg.read_bytes()) != data:
            raise ValueError('실행 등록 후 변경')
        with (BASE / (a.case + '_console.log')).open('x') as f:
            r = subprocess.run(['bash', str(ENV), sys.executable, '-B', *data['args']], cwd=ROOT, stdout=f,
                               stderr=subprocess.STDOUT)
        with (BASE / (a.case + '_exit.log')).open('x') as f:
            f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
