"""r48 생산 소스·인자가 그대로인지 확인하고 실행 입력을 등록한다. 사용: register.py [--check]

과거 계약·등록 해시는 읽기만 한다. 하나라도 어긋나면 고치지 않고 중단한다.
"""
from pathlib import Path
import datetime, hashlib, json, subprocess, sys, zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
R48 = ROOT/'artifacts/validation/2026-11-08/observed_corner_continuity_r48_v1'
BASE = ROOT/'artifacts/validation/2026-11-08/grid_pack_newcourse_v1'
CASE = 'five_vehicle_grid_staggered'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
rel = lambda p: Path(p).relative_to(ROOT).as_posix()


def main(check):
    for name in ('geometry.exit', 'regression.exit'):
        assert (A/name).read_text().strip() == '0', name
    previous = json.loads((R48/'registered.json').read_bytes())
    changed = [p for p, h in previous['sources'].items() if not (ROOT/p).is_file() or sha(ROOT/p) != h]
    contract = json.loads((ROOT/previous['contract']).read_bytes())
    contract_ok = (sha(ROOT/contract['parent_contract']) == contract['parent_contract_sha256']
                   and sha(ROOT/contract['before_snapshot']) == contract['before_snapshot_sha256']
                   and all(sha(ROOT/p) == d['after'] for p, d in contract['approved_changes'].items())
                   and all(sha(ROOT/p) == d for p, d in contract['new_dependencies'].items())
                   and sha(R48/'source_snapshot.zip') == previous['snapshot_sha256'])
    if changed or not contract_ok:
        print(json.dumps(dict(refused='r48 provenance differs; past hashes are not edited', changed=changed[:20],
                              contract_ok=contract_ok)))
        return 5
    for name in ('new_track_course.json', 'additional_actors.json'):
        derived = json.loads((A/name).read_bytes())['derived_from']
        assert sha(ROOT/derived['path']) == derived['sha256'], name
    args = list(previous['args'])
    args[args.index('--config')+1] = rel(A/'new_track_course.json')
    args[args.index('--case')+1] = CASE
    assert '--actor-overlay' not in args
    args += ['--actor-overlay', rel(A/'additional_actors.json')]
    files = set(subprocess.check_output(['git', 'ls-files', '-co', '--exclude-standard', 'src', 'scripts', 'config'],
                                        cwd=ROOT, text=True).splitlines())
    files |= set(previous['sources'])
    files |= {rel(p) for p in A.iterdir() if p.suffix in ('.py', '.sh', '.md', '.json')
              and p.name not in ('registered.json', 'geometry.json')}
    files |= {rel(A/'geometry.json'), rel(BASE/'new_track_course.json'), rel(BASE/'additional_actors.json'),
              rel(R48/'registered.json'), previous['contract'],
              'tests/test_official_multi_actor_course.py', 'tests/test_traffic_course_track.py',
              'tests/test_traffic_actors.py', 'tests/test_official_traffic_grid.py'}
    files = sorted(p for p in files if (ROOT/p).is_file())
    new_since_r48 = sorted(set(files)-set(previous['sources']))
    print(json.dumps(dict(r48_sources=len(previous['sources']), r48_changed=0, contract_ok=True,
                          files=len(files), new_since_r48=new_since_r48, args=args), ensure_ascii=False))
    if check:
        return 0
    assert not any((A/n).exists() for n in ('registered.json', 'source_snapshot.zip', 'run1'))
    with zipfile.ZipFile(A/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as z:
        for p in files:
            z.write(ROOT/p, p)
    data = dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                scope='r48 controller unchanged; official 5-slot grid, staggered evaluator-only starts grid2 .2/grid1 .8/grid0 1.4/grid4 2.0 s at 0.5 m/s, 12 s window, 180 mm assumed actor height',
                args_source=rel(R48/'registered.json'), args=args, contract=previous['contract'],
                new_since_r48=new_since_r48, sources={p: sha(ROOT/p) for p in files},
                snapshot_sha256=sha(A/'source_snapshot.zip'), runs={'CASE': dict(folder=rel(A))})
    with open(A/'registered.json', 'x', encoding='utf-8', newline='\n') as f:
        f.write(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
    print('registered', len(files))
    return 0


if __name__ == '__main__':
    sys.exit(main('--check' in sys.argv))
