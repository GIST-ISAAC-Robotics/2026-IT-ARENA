"""새 직선 대표 2회의 최종 선택·실행 등록(실행 전 고정). proposed_runs.json은 제안 원본으로 보존한다.

사용: python3 -B prepare_runs.py accept           # accepted_runs.json
      python3 -B prepare_runs.py install-check    # checks/install_check.json (빌드 필요 여부)
      python3 -B prepare_runs.py register <case>  # <case>/registered.json + <case>/source_snapshot.zip
"""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
REL = HERE.relative_to(ROOT).as_posix()
CASES = ('slow_lead', 'straight_moving_lead')
ENV = 'artifacts/validation/2026-11-01/turn_preview_v1/environment.sh'
SENSOR = 'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'
OWN = ('new_track_course.json', 'proposed_runs.json', 'accepted_runs.json', 'geometry_probe.json',
       'prepare_runs.py', 'run.py', 'run.sh', 'assess.py')


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save(path, data):
    with Path(path).open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n')


def accept():
    proposed = read(HERE/'proposed_runs.json')
    slow = [a for a in proposed['runs']['slow_lead'] if a != '--moving-pass-trial']
    assert len(slow) == len(proposed['runs']['slow_lead'])-1 and '--handoff-continuous-pace' in slow
    moving = proposed['runs']['straight_moving_lead']
    assert '--moving-pass-trial' in moving and '--handoff-continuous-pace' in moving
    save(HERE/'accepted_runs.json', dict(
        decided='2026-11-07 GPT 설계 결정(작업 총괄 지시)', proposal='proposed_runs.json',
        proposal_sha256=sha(HERE/'proposed_runs.json'),
        runs=dict(
            slow_lead=dict(args=slow, change_from_proposal='--moving-pass-trial 제거',
                           reason='순수 추종 대조. continuous pace 등 나머지 r16 준비 상태·감속비·센서·물성 유지'),
            straight_moving_lead=dict(args=moving, change_from_proposal='없음',
                                      reason='r16 동작(moving pass ON) 새 직선 확인')),
        order=list(CASES), rule='각 장면 1회, 동시 Gazebo 1개, run1 실행기/신호/환경 결함 없음 확인 뒤 run2'))


def install_check():
    """symlink 설치의 share·실행 파일이 현재 src와 같고 C++ 산출물이 원본보다 새로운지 확인한다."""
    rows, bad = 0, []
    for package, sub in (('arena_bringup', 'launch'), ('arena_bringup', 'config'), ('arena_description', 'config'),
                         ('arena_description', 'models'), ('arena_gazebo', 'worlds/it_arena_official_v2026_11_06'),
                         ('arena_gazebo', 'worlds/it_arena_official')):
        for path in (ROOT/'src'/package/sub).rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts:
                rows += 1
                installed = ROOT/'install'/package/'share'/package/path.relative_to(ROOT/'src'/package)
                if not installed.is_file() or sha(installed) != sha(path):
                    bad.append(path.relative_to(ROOT).as_posix())
    controller = ROOT/'install/arena_gazebo/lib/arena_gazebo/traffic_light_controller.py'
    native = {name: (ROOT/'build/arena_gazebo'/name).stat().st_mtime
              for name in ('libarena-single-motor-system.so', 'visual_material_client')}
    sources = max(p.stat().st_mtime for p in (ROOT/'src/arena_gazebo').rglob('*') if p.suffix in ('.cpp', '.hpp'))
    egg = {name: (ROOT/'install'/name/'lib/python3.12/site-packages').glob('*.egg-link').__next__().read_text().split()[0]
           for name in ('arena_autonomy', 'arena_vehicle_interface')}
    result = dict(share_files_checked=rows, share_mismatch=bad,
                  traffic_light_controller_symlink=str(controller.resolve()) == str(ROOT/'src/arena_gazebo/scripts/traffic_light_controller.py'),
                  native_outputs_newer_than_cpp_sources=all(t > sources for t in native.values()),
                  python_develop_links=egg)
    result['build_needed'] = bool(bad) or not result['traffic_light_controller_symlink'] \
        or not result['native_outputs_newer_than_cpp_sources']
    save(HERE/'checks/install_check.json', result)
    return 1 if result['build_needed'] else 0


def register(case):
    assert case in CASES
    folder = HERE/case
    folder.mkdir(exist_ok=False)
    run = read(HERE/'accepted_runs.json')['runs'][case]
    names = subprocess.check_output(['git', 'ls-files', '-co', '--exclude-standard', 'src', 'scripts', 'config'],
                                    cwd=ROOT, text=True).splitlines()
    files = sorted({p for p in names if (ROOT/p).is_file()} | {f'{REL}/{name}' for name in OWN} | {ENV})
    with zipfile.ZipFile(folder/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in files:
            bundle.write(ROOT/path, path)
    save(folder/'registered.json', dict(
        registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope=f'v2026.11.06 출발 직선 {case} 1회. 제어기 변경 없음, 평가 각본만 새 track 등록',
        environment=ENV, sensor=SENSOR, sensor_sha256=sha(ROOT/SENSOR), args=run['args'], reason=run['reason'],
        command=f'wsl.exe -e bash {REL}/run.sh {case}',
        sources={p: sha(ROOT/p) for p in files}, snapshot_sha256=sha(folder/'source_snapshot.zip')))
    print('registered', case, len(files))
    return 0


if __name__ == '__main__':
    action = sys.argv[1]
    if action == 'accept':
        accept()
    elif action == 'install-check':
        raise SystemExit(install_check())
    elif action == 'register':
        raise SystemExit(register(sys.argv[2]))
    else:
        raise ValueError(action)
