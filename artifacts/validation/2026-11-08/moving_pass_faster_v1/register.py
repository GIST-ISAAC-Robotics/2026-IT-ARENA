"""0.70m/s 상대·10초·faster_v1의 새 코스 직선 대표를 실행 전에 고정한다."""
import datetime
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
comparison = json.loads((HERE/'current_comparison_summary.json').read_bytes())
assert all(row['change_from_before'] == 0 for row in comparison.values())
contract = ROOT/'config/tests/moving_pass_faster_r31_sha256.json'
c = json.loads(contract.read_bytes())
for row_group in ('approved_changes', 'approved_evaluation_scripts'):
    for path, row in c[row_group].items():
        assert sha(ROOT/path) == row['after'], path
old = HERE.parent/'medium_pass_connected_v1/straight_moving_lead/registered.json'
args = json.loads(old.read_bytes())['args']
args[args.index('--config')+1] = (HERE/'new_track_course.json').relative_to(ROOT).as_posix()
args[args.index('--moving-pass-profile')+1] = 'faster_v1'
args.append('--handoff-standing-wall-reacquire')
assert '--handoff-boundary-refresh' not in args
course = json.loads((HERE/'new_track_course.json').read_bytes())
assert course['cases'][0]['schedule'][0]['speed_mps'] == .7
assert course['cases'][0]['duration_s'] == 10.
assert .7*10. < 8.08  # 기존 grid3 상대 시작~첫 코너의 평가용 직선 길이.
folder = HERE/'straight_moving_lead'
folder.mkdir(exist_ok=False)
files = set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],
                                   cwd=ROOT, text=True).splitlines())
helpers = ('register.py','run.py','run.sh','assess.py','new_track_course.json','PLAN.md')
files = sorted({p for p in files if (ROOT/p).is_file()} | {(HERE/p).relative_to(ROOT).as_posix() for p in helpers})
with zipfile.ZipFile(folder/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
    for path in files:
        archive.write(ROOT/path, path)
data = dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
    scope='single 0.70m/s lead, 10s straight; explicit faster_v1, two-wheel surface, independent STOP unchanged',
    args=args, contract=contract.relative_to(ROOT).as_posix(),
    sources={p:sha(ROOT/p) for p in files}, snapshot_sha256=sha(folder/'source_snapshot.zip'))
(folder/'registered.json').write_text(json.dumps(data, ensure_ascii=False, indent=2)+'\n')
print('registered',len(files))
