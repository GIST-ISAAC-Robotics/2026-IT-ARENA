"""r6 소스 고정 실행 등록. 인자는 상위 accepted_runs.json 그대로(판정선 불변), 바뀐 것은 launch 가드뿐.

사용: python3 -B prepare_runs.py register <slow_lead|straight_moving_lead>  → r6/<case>/registered.json·source_snapshot.zip
"""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[4]
PARENT = HERE.parent
REL, PREL = HERE.relative_to(ROOT).as_posix(), PARENT.relative_to(ROOT).as_posix()
ENV = 'artifacts/validation/2026-11-01/turn_preview_v1/environment.sh'
PARENT_FILES = ('new_track_course.json', 'proposed_runs.json', 'accepted_runs.json', 'geometry_probe.json')
OWN = ('prepare_runs.py', 'make_scripts.py', 'prepare_contract.py', 'guard_check.py', 'run.py', 'run.sh', 'assess.py',
       'before.zip', 'install_check.json')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def register(case):
    accepted = json.loads((PARENT/'accepted_runs.json').read_text(encoding='utf-8'))['runs'][case]
    folder = HERE/case
    folder.mkdir(exist_ok=False)
    names = subprocess.check_output(['git', 'ls-files', '-co', '--exclude-standard', 'src', 'scripts', 'config'],
                                    cwd=ROOT, text=True).splitlines()
    files = sorted({p for p in names if (ROOT/p).is_file()} | {f'{PREL}/{n}' for n in PARENT_FILES}
                   | {f'{REL}/{n}' for n in OWN} | {ENV})
    with zipfile.ZipFile(folder/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in files:
            bundle.write(ROOT/path, path)
    data = dict(
        registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope=f'v2026.11.06 출발 직선 {case} 재실행 1회(run2). 재실행 원인 변화: G1/G2 launch 가드가 명시 허용된 새 공식 track을 '
              '받도록 r6 계약으로 확장. 실행 인자·각본·판정선은 accepted_runs.json/new_track_course.json 그대로',
        previous_failure=f'{PREL}/{case}/run1' if (PARENT/case/'run1').exists() else None,
        contract='config/tests/handoff_track_guard_r6_sha256.json', environment=ENV, args=accepted['args'],
        reason=accepted['reason'], command=f'wsl.exe -e bash {REL}/run.sh {case}',
        sources={p: sha(ROOT/p) for p in files}, snapshot_sha256=sha(folder/'source_snapshot.zip'))
    with (folder/'registered.json').open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
    print('registered', case, len(files))


if __name__ == '__main__':
    assert sys.argv[1] == 'register'
    register(sys.argv[2])
