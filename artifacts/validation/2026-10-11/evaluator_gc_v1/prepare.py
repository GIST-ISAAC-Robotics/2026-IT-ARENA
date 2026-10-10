"""실행 전 계약·계획·소스 사본 동결. 기존 파일은 덮어쓰지 않는다."""
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write(path, value):
    with path.open('x', encoding='utf-8') as f:
        json.dump(value, f, ensure_ascii=False, indent=2)


if __name__ == '__main__':
    parent = ROOT/'config/tests/scheduled_command_r2_sha256.json'
    contract = json.loads(parent.read_text(encoding='utf-8'))
    prior = contract['approved_evaluation_scripts']['scripts/probe_braking_contact.py']['after']
    contract.update(parent_contract=parent.relative_to(ROOT).as_posix(), parent_contract_sha256=sha(parent),
                    scope='2026-10-11 evaluation GC native/defer only; controller/protection/world/plugin unchanged')
    contract['approved_changes'] = {}
    contract['approved_implementation_changes'] = {}
    contract['approved_evaluation_scripts'] = {
        'scripts/probe_braking_contact.py': {'before': prior, 'after': sha(ROOT/'scripts/probe_braking_contact.py'),
                                           'reason': 'bounded opt-in evaluator GC scope; native default'},
        'scripts/scheduled_command.py': contract['approved_evaluation_scripts']['scripts/scheduled_command.py'],
    }
    # 변경되지 않은 파일은 새 before/after 변경인 것처럼 기록하지 않는다.
    scheduled = contract['approved_evaluation_scripts'].pop('scripts/scheduled_command.py')
    contract['sha256']['scripts/scheduled_command.py'] = scheduled['after']
    contract['implementation_sha256']['scripts/evaluation_gc.py'] = sha(ROOT/'scripts/evaluation_gc.py')
    path = ROOT/'config/tests/scheduled_command_r3_sha256.json'
    write(path, contract)
    common = ['--surface', 'official', '--zero-speed-mode', 'passive_damping', '--command-schedule', 'sim_grid',
              '--schedule-phase-ms', '10', '--command-profile',
              'artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json', '--plugin-command-trace']
    for name, mode in [('a1_native', 'native'), ('b1_defer', 'defer'), ('b2_defer', 'defer'), ('a2_native', 'native')]:
        write(BASE/f'plan_{name}.json', {'base': BASE.relative_to(ROOT).as_posix(),
              'purpose': 'pre-registered evaluator GC ABBA; each case requires parent review before next',
              'cases': [{'name': name, 'expect': 'pass', 'args': [*common, '--evaluation-gc', mode]}]})
    files = [ROOT/p for p in {**contract['sha256'], **contract['implementation_sha256']}]
    files += [ROOT/'scripts/probe_braking_contact.py', ROOT/'tests/test_evaluation_gc.py',
              ROOT/'tests/test_physics_step_option.py', BASE/'README.md', BASE/'prepare.py', path]
    with zipfile.ZipFile(BASE/'pre_source.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for p in files:
            z.write(p, p.relative_to(ROOT).as_posix())
    write(BASE/'pre_manifest.json', {'sha256': {p.relative_to(ROOT).as_posix(): sha(p) for p in files},
          'git_head': subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip()})
