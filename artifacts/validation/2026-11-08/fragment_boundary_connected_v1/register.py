"""r27 실제 인자를 계승하고 단일 첫 코너 옆차 장면을 실행 전에 고정한다."""
import datetime
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path
HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
old = ROOT/'artifacts/validation/2026-11-08/side_corner_live_v1/right_parallel_first_curve/registered.json'
args = json.loads(old.read_bytes())['args']
args[args.index('--config')+1] = (HERE/'new_track_course.json').relative_to(ROOT).as_posix()
args[args.index('--case')+1] = 'right_parallel_first_curve'
assert '--actor-overlay' not in args
args.append('--handoff-boundary-refresh')
assert '--handoff-standing-wall-reacquire' in args
assert not any(a.startswith('artifacts/validation/2026-11-08/traffic_faster_reacquire_live_v1') for a in args)
contract = ROOT/'config/tests/fragmented_wall_refresh_r30_sha256.json'
c = json.loads(contract.read_bytes())
for p, change in {**c['approved_changes'], **c['approved_evaluation_scripts']}.items():
    assert sha(ROOT/p) == change['after'], p
for p, digest in c['new_dependencies'].items():
    assert sha(ROOT/p) == digest, p
geometry = json.loads((HERE/'geometry.json').read_bytes())
assert geometry['initial_ego_actor_gap_m'] >= .08
assert geometry['route']['actor_max_off_wheel_centers'] <= 2
folder = HERE/'right_parallel_first_curve'
folder.mkdir(exist_ok=False)
files = set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'], cwd=ROOT, text=True).splitlines())
files = sorted({p for p in files if (ROOT/p).is_file()} | {(HERE/p).relative_to(ROOT).as_posix()
    for p in ('register.py','run.py','run.sh','assess.py','geometry.py','new_track_course.json','PLAN.md')})
with zipfile.ZipFile(folder/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
    for p in files:
        archive.write(ROOT/p, p)
result = dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    scope='r30 observed fragment path and retained current wall; same r28 32s side-corner actor',
    args_source=old.relative_to(ROOT).as_posix(), args=args, contract=contract.relative_to(ROOT).as_posix(),
    sources={p:sha(ROOT/p) for p in files}, snapshot_sha256=sha(folder/'source_snapshot.zip'))
(folder/'registered.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print('registered', len(files))
