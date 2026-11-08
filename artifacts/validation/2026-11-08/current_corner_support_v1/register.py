import datetime
import hashlib
import json
import subprocess
import sys
import zipfile
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
contract = 'config/tests/current_corner_support_r41_sha256.json'
c = json.loads((ROOT/contract).read_bytes())
assert sha(ROOT/c['parent_contract']) == c['parent_contract_sha256']
assert sha(ROOT/c['before_snapshot']) == c['before_snapshot_sha256']
for p, row in c['approved_changes'].items(): assert sha(ROOT/p) == row['after']
for p, digest in c['new_dependencies'].items(): assert sha(ROOT/p) == digest
legacy = A.parent/'fragment_boundary_connected_v1/right_parallel_first_curve/registered.json'
old = json.loads(legacy.read_bytes())
args = list(old['args'])
args[args.index('--config')+1] = (A/'new_track_course.json').relative_to(ROOT).as_posix()
assert args.count('--handoff-boundary-refresh') == 1
assert '--handoff-standing-wall-reacquire' in args
assert sha(A/'new_track_course.json') == sha(A.parent/'fragment_boundary_connected_v1/new_track_course.json')
evaluation = json.loads((A/'evaluation_source.json').read_bytes())
for p, digest in evaluation.items(): assert sha(ROOT/p) == digest
geometry = json.loads((A/'geometry.json').read_bytes())
assert geometry['initial_ego_actor_gap_m'] >= .08
assert geometry['route']['actor_max_off_wheel_centers'] <= 2
files = set(subprocess.check_output(['git', 'ls-files', '-co', '--exclude-standard', 'src', 'scripts', 'config'], cwd=ROOT, text=True).splitlines())
files |= {p.relative_to(ROOT).as_posix() for p in A.iterdir() if p.suffix in ('.py', '.sh', '.md', '.json', '.zip')}
files |= set(evaluation) | {'tests/test_corner_boundary_support.py', 'tests/test_source_provenance_r0_plus_b3.py'}
files = sorted(p for p in files if (ROOT/p).is_file())
print('r41 sources', len(files), 'same r30 scenario and criteria')
if '--check' in sys.argv: raise SystemExit(0)
assert not any((A/p).exists() for p in ('registered.json', 'source_snapshot.zip', 'run1'))
with zipfile.ZipFile(A/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as z:
    for p in files: z.write(ROOT/p, p)
r = dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    scope='r41 current two-face support of valid retained corner; r30 same32s side actor',
    args_source=legacy.relative_to(ROOT).as_posix(), args=args, contract=contract,
    sources={p:sha(ROOT/p) for p in files}, snapshot_sha256=sha(A/'source_snapshot.zip'),
    runs={'CASE':dict(folder=A.relative_to(ROOT).as_posix())})
(A/'registered.json').write_text(json.dumps(r, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
