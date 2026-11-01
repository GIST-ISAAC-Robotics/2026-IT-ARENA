"""대표 155초 코스를 paced로 한 번 실행한다. 소스·각본을 등록한 뒤에만 시작한다."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
import race_traffic_course as course

course.BASE = BASE
course.ENTRY = BASE / 'course.launch.py'
from traffic_actors import overlay_case
from validate_g1_traffic import source_paths

ENV = BASE.parent / 'turn_preview_v1/environment.sh'
OUTPUT = BASE / 'run_paced'
REGISTERED = BASE / 'run_paced_registered.json'
CASE = 'stop_restart_full_course'


def register():
    cfg = json.loads(course.CONFIG.read_text(encoding='utf-8'))
    request = SimpleNamespace(mode='g1g2', config=course.CONFIG, actor_overlay=course.OVERLAY, seed=0, case=CASE,
                              record_follow_decisions=True, output=OUTPUT)
    sources = course.inputs(request, cfg, overlay_case(cfg['cases'][0], course.OVERLAY))
    paths = set(source_paths(course.CONFIG, actor_overlay=course.OVERLAY))
    paths.update(p.relative_to(ROOT).as_posix() for p in (ROOT / 'src/arena_autonomy/arena_autonomy').glob('*.py'))
    paths.update([Path(__file__).resolve().relative_to(ROOT).as_posix(), 'scripts/g1_course_eval.py',
                  (BASE / 'course_entry.py').relative_to(ROOT).as_posix(), ENV.relative_to(ROOT).as_posix(),
                  'artifacts/validation/2026-11-01/follow_continuity_v1/course_replay_v2.json'])
    sources.update({k: sha256((ROOT / k).read_bytes()).hexdigest() for k in sorted(paths)})
    args = [str(BASE / 'course_entry.py'), '--mode', 'g1g2', '--handoff-reserve', 'paced', '--team-signal-course',
            '--config', str(course.CONFIG), '--actor-overlay', str(course.OVERLAY), '--case', CASE, '--seed', '0',
            '--record-follow-decisions', '--output', str(OUTPUT)]
    return dict(args=args, source_sha256=sources,
                scope='mixed_course_v1 run1과 같은 2×5 신호·1.5 m/s·앞차/왼쪽 차·155초에서 추종 여유 선택만 paced. 독립 3대 경기 아님.')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepare-only', action='store_true')
    a = p.parse_args()
    if OUTPUT.exists():
        raise FileExistsError('이미 실행한 출력')
    data = register()
    if a.prepare_only:
        with REGISTERED.open('x', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        print('REGISTERED', len(data['source_sha256']))
    else:
        if data != json.loads(REGISTERED.read_text(encoding='utf-8')):
            raise ValueError('등록 후 소스·각본 변경')
        with (BASE / 'run_paced_console.log').open('x') as log:
            result = subprocess.run(['bash', str(ENV), sys.executable, '-B', *data['args']], cwd=ROOT, stdout=log,
                                    stderr=subprocess.STDOUT)
        with (BASE / 'run_paced_exit.log').open('x') as f:
            f.write(f'ACTUAL_EXIT_CODE={result.returncode}\n')
        raise SystemExit(result.returncode)
