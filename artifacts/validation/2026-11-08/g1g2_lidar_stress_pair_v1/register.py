"""A(무주입)·B(등록 C1 주입)를 같은 소스 스냅숏 하나로 실행 전에 함께 등록한다.

사용: python3 -B register.py --check   (아무것도 쓰지 않고 목록·해시·계약만 확인)
      python3 -B register.py           (registered.json·source_snapshot.zip·두 run 폴더 생성, 재실행 거부)
"""
import datetime, hashlib, json, subprocess, sys, zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
CONTRACT = 'config/tests/g1g2_lidar_stress_r34_sha256.json'
PROFILE = 'config/tests/lidar_uniform30_delay50_jitter20.json'
RUNS = {'A': 'A_no_injection', 'B': 'B_c1_injection'}
HELPERS = ('PLAN.md', 'register.py', 'run.py', 'run.sh', 'assess.py', 'compare.py', 'check_runtime.py',
           'pair_args.json', 'new_track_course.json', 'actor_overlay.json', 'before.zip')
EXTRA = ('tests/test_g1g2_lidar_stress_cli.py', 'tests/test_source_provenance_r0_plus_b3.py')


def plan():
    pair = json.loads((HERE/'pair_args.json').read_text(encoding='utf-8'))
    common = pair['common_args']
    assert common[0] == 'scripts/validate_g1_traffic.py' and common.count('OUTPUT') == 1
    assert '--lidar-test-profile' not in common and '--motion-test-profile' not in common
    assert pair['b_only_args'] == ['--lidar-test-profile', PROFILE]
    r27 = json.loads((ROOT/pair['r27_registered']).read_text(encoding='utf-8'))
    for copy, original in pair['copied_inputs'].items():
        assert sha(ROOT/copy) == sha(ROOT/original) == r27['sources'][original], copy
    expected = list(r27['args'])
    for copy, original in pair['copied_inputs'].items():
        expected[expected.index(original)] = copy
    assert common == expected, 'common args must equal r27 args with only copied input paths'
    contract = json.loads((ROOT/CONTRACT).read_text(encoding='utf-8'))
    for path, change in contract['approved_evaluation_scripts'].items():
        assert sha(ROOT/path) == change['after'], path
    assert sha(ROOT/contract['parent_contract']) == contract['parent_contract_sha256']
    assert sha(ROOT/contract['before_snapshot']) == contract['before_snapshot_sha256']
    files = set(subprocess.check_output(['git', 'ls-files', '-co', '--exclude-standard', 'src', 'scripts', 'config'],
                                        cwd=ROOT, text=True).splitlines())
    files = sorted({p for p in files if (ROOT/p).is_file()} | set(EXTRA)
                   | {(HERE/p).relative_to(ROOT).as_posix() for p in HELPERS})
    for p in (CONTRACT, PROFILE, *pair['copied_inputs'], 'scripts/validate_g1_traffic.py'):
        assert p in files, p
    runs = {k: dict(folder=(HERE/v).relative_to(ROOT).as_posix(),
                    args=common+(pair['b_only_args'] if k == 'B' else []))
            for k, v in RUNS.items()}
    assert runs['B']['args'][:len(common)] == runs['A']['args']
    return pair, files, runs


def main(check):
    pair, files, runs = plan()
    print(json.dumps(dict(sources=len(files), runs=runs), ensure_ascii=False, indent=1))
    if check:
        print('CHECK_ONLY: nothing written')
        return 0
    if (HERE/'registered.json').exists() or (HERE/'source_snapshot.zip').exists() or any((HERE/v).exists() for v in RUNS.values()):
        print('refused: already registered')
        return 2
    with zipfile.ZipFile(HERE/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for p in files:
            z.write(ROOT/p, p)
    data = dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                scope=f'r27 0.8m/s two leads 60s one lap; A no injection, B adds only {PROFILE}; '
                      f'same source snapshot ({CONTRACT} sha256 {sha(ROOT/CONTRACT)})',
                args_source=pair['r27_registered'], runs=runs,
                sources={p: sha(ROOT/p) for p in files}, snapshot_sha256=sha(HERE/'source_snapshot.zip'))
    (HERE/'registered.json').write_bytes((json.dumps(data, ensure_ascii=False, indent=2)+'\n').encode())
    for v in RUNS.values():
        (HERE/v).mkdir()
    print('registered', len(files))
    return 0


if __name__ == '__main__':
    sys.exit(main('--check' in sys.argv[1:]))
