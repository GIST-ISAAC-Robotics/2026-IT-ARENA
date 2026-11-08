"""v3 묶음(v2 픽스처 footer 수정): 검사 → v1과 runtime 해시 대조 → 격리 추출 → 문서상 profile_follow_trace.py CLI 실제 실행.

v1/v2 archive·결과는 읽기만 한다. v1 CLI 결함 재현은 v2에서 마쳤다.
Jetson·ROS·Gazebo를 실행하지 않는다. 전체 3,168판단 재생은 반복하지 않는다(v1 parity.json 참조).
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[4]
V1 = HERE.parent/'bundle_manifest.json'
MANIFEST = HERE/'bundle_manifest.json'
ARCHIVE = REPO/'build/jetson_follow_20261108_v3.tar.gz'
V1_ARCHIVE = REPO/'build/jetson_follow_20261108_v1.tar.gz'
TARGET = REPO/'build/jetson_follow_20261108_v3_extract'
V1_REPRO = REPO/'build/jetson_follow_20261108_v1_cli_repro'
EXPECTED_CHANGED = {'scripts/profile_follow_trace.py'}
FIXTURES = json.loads((HERE/'fixtures.json').read_text(encoding='utf-8'))
FIXTURE_DIR = 'artifacts/validation/2026-11-08/jetson_bundle_refresh_v1/v3/fixtures'


def clean_env(root):
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(('PYTHON', 'ROS', 'AMENT', 'COLCON', 'RMW', 'CYCLONEDDS', 'FASTRTPS'))}
    # 문서상 실행: PYTHONPATH=src/arena_autonomy:src/arena_vehicle_interface:scripts python3 scripts/profile_follow_trace.py
    env.update(PYTHONNOUSERSITE='1', PYTHONPATH=':'.join(
        str(root/part) for part in ('src/arena_autonomy', 'src/arena_vehicle_interface', 'scripts')))
    return env


def cli(root, trace, out, label):
    command = [sys.executable, '-B', 'scripts/profile_follow_trace.py', str(trace), '--input-mode', 'stream',
               '--warmups', '0', '--repeats', '1', '--output', out]
    done = subprocess.run(command, cwd=root, env=clean_env(root), capture_output=True, text=True, timeout=600)
    (HERE/f'cli_{label}.log').open('x', encoding='utf-8').write(
        f'$ (cwd={root}) {" ".join(command)}\nexit={done.returncode}\n--- stdout\n{done.stdout}\n--- stderr\n{done.stderr}')
    return done.returncode, done.stderr.strip().splitlines()[-1:] if done.stderr.strip() else []


def main():
    sys.path.insert(0, str(REPO/'scripts'))
    from verify_follow_profile_bundle import verify
    manifest = json.loads(MANIFEST.read_text(encoding='utf-8'))
    check = verify(ARCHIVE, manifest)
    with (HERE/'verify_result.json').open('x', encoding='utf-8') as stream:
        json.dump(check, stream, ensure_ascii=False, indent=2)
    assert check['passed'] and check['dependency_closure']['checked'], check
    old, new = json.loads(V1.read_text(encoding='utf-8'))['files'], manifest['files']
    common = set(old) & set(new)
    changed = sorted(p for p in common if old[p]['sha256'] != new[p]['sha256'])
    runtime = dict(v1_files=len(old), v3_files=len(new), changed=changed, removed=sorted(set(old)-set(new)),
                   added=sorted(set(new)-set(old)),
                   identical_except_expected=set(changed) == EXPECTED_CHANGED and not set(old)-set(new)
                   and all(p.startswith(FIXTURE_DIR) for p in set(new)-set(old)),
                   v1_parity_traces_same_bytes=all(old[p] == new[p] for p in common if p.endswith('.jsonl')))
    TARGET.mkdir(exist_ok=False)
    with tarfile.open(ARCHIVE, 'r:gz') as archive:
        archive.extractall(TARGET, filter='data')
    runs = {}
    for name in FIXTURES:
        trace = TARGET/FIXTURE_DIR/name
        code, tail = cli(TARGET, trace, f'artifacts/validation/local_cli_v3/{name[:-6]}', f'v3_{name[:-6]}')
        report = json.loads((TARGET/f'artifacts/validation/local_cli_v3/{name[:-6]}/report.json').read_text(encoding='utf-8'))
        runs[name] = dict(exit=code, **{k: report.get(k) for k in (
            'passed', 'baseline_matched', 'timed_calls', 'controller_scope', 'input_unchanged', 'source_unchanged',
            'source_snapshot_omitted', 'source_snapshot_complete')}, snapshot_files=len(report.get('source_sha256', {})))
        # 같은 환경에서 runpy로 같은 CLI 파일을 __main__으로 실행해 실제 import 위치를 기록한다.
        out = f'artifacts/validation/local_cli_v3/{name[:-6]}_modules'
        done = subprocess.run([sys.executable, '-B', str(HERE.parent/'v2/cli_modules.py'), str(trace), out, str(REPO)],
                              cwd=TARGET, env=clean_env(TARGET), capture_output=True, text=True, timeout=600)
        runs[name]['module_check'] = json.loads(done.stdout.strip().splitlines()[-1]) if done.returncode == 0 else dict(
            exit=done.returncode, stderr=done.stderr[-2000:])
    # v1 CLI 결함 재현은 v2/cli_result.json에 보존했으므로 반복하지 않는다.
    result = dict(passed=runtime['identical_except_expected'] and runtime['v1_parity_traces_same_bytes']
                  and all(r['exit'] == 0 and r['passed'] and r['source_unchanged']
                          and r['source_snapshot_omitted'] == ['tests/test_follow_profile.py']
                          and r['module_check'].get('from_extract_only') for r in runs.values()),
                  archive=ARCHIVE.relative_to(REPO).as_posix(), runtime_hash_comparison=runtime, cli_runs=runs,
                  v1_cli_repro='see ../v2/cli_result.json',
                  scope='minimal documented CLI entrypoint check on derived 100-call fixtures; not Jetson/ROS')
    with (HERE/'cli_result.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
