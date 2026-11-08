"""r32 같은 상대 배치에서 r44 진입/추종 지속 1회. 기존 추월 판정은 보존."""
from datetime import datetime, timezone
from pathlib import Path
import hashlib
import json
import subprocess
import zipfile

HERE = Path(__file__).resolve().parent; ROOT = HERE.parents[3]
R32 = HERE.parent/'moving_pass_wall_mask_v1'
sha = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert (HERE/'tests_v2.exit').read_text().strip() == '0'
assert (HERE/'legacy_compare_v1.exit').read_text().strip() == '0'
replay = json.loads((HERE/'replay.json').read_bytes())
assert all(replay[k]['expected_mismatch']==0 for k in ('r32','right20','recessed_medium'))
legacy = json.loads((HERE/'legacy_compare.json').read_bytes())
assert all(v['before_after_mismatch']==0 for v in legacy.values())
assert sha(HERE/'new_track_course.json') == sha(R32/'new_track_course.json')
assert sha(HERE/'assess.py') == sha(R32/'assess.py')
case = 'straight_moving_lead'
old = json.loads((R32/case/'registered.json').read_bytes())
args = list(old['args']); i = args.index('--config')+1
assert args[i] == (R32/'new_track_course.json').relative_to(ROOT).as_posix()
args[i] = (HERE/'new_track_course.json').relative_to(ROOT).as_posix()
assert args[args.index('--moving-pass-profile')+1] == 'faster_v1'
d = HERE/case; d.mkdir(exist_ok=False)
files = set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],
    cwd=ROOT, text=True).splitlines())
helpers = ['register.py','new_track_course.json','assess.py','run.py','run.sh','check_runtime.py',
           'continuity_assess.py','PLAN.md']
files = sorted({f for f in files if (ROOT/f).is_file()} | {(HERE/f).relative_to(ROOT).as_posix() for f in helpers})
with zipfile.ZipFile(d/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as z:
    for f in files: z.write(ROOT/f, f)
record = dict(registered_utc=datetime.now(timezone.utc).isoformat(),
    scope='same r32 0.70m/s right12cm scene; r44 sustained-entry proposal with unchanged FOLLOW fallback',
    args=args, sources={f:sha(ROOT/f) for f in files}, snapshot_sha256=sha(d/'source_snapshot.zip'),
    r32_evaluator_sha256=sha(R32/'assess.py'), r32_args_identical_except_config=True)
(d/'registered.json').write_text(json.dumps(record, indent=2)+'\n', encoding='utf8')
print('registered', len(files))
