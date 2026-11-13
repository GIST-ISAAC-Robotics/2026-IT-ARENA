#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
ARTIFACT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/estimated_observation_r444_v1'
RUN='/home/jinhyeong/it-arena-r444-verify-b'
python3 - "$ARTIFACT" "$RUN" <<'PY'
from pathlib import Path
import sys,zipfile
a,r=map(Path,sys.argv[1:]);assert not r.exists(),r
r.mkdir()
with zipfile.ZipFile(a/'validation_sources_v2.zip') as z:
    assert all(not Path(n).is_absolute() and '..' not in Path(n).parts for n in z.namelist())
    z.extractall(r)
PY
cd "$RUN"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$RUN/src/arena_autonomy:$RUN/src/arena_vehicle_interface:${PYTHONPATH:-}"
set +e
python3 -B run_validation.py
CODE=$?
set -e
python3 - "$ARTIFACT" "$RUN" "$CODE" <<'PY'
from pathlib import Path
import hashlib,json,os,shutil,sys
a,r=map(Path,sys.argv[1:3]);out=a/'verify_native_v2';out.mkdir(exist_ok=False)
for name in ('execution.json','units.log','units.xml','saved.log'):
    if (r/name).exists():shutil.copy2(r/name,out/name)
if (r/'saved_result').exists():shutil.copytree(r/'saved_result',out/'saved_result')
remaining=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) in (os.getpid(),os.getppid()):continue
    try:
        cwd=(p/'cwd').resolve(strict=True)
        if cwd.is_relative_to(r):remaining.append(dict(pid=int(p.name),command=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode()))
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
manifest=json.loads((r/'manifest.json').read_bytes())
changed=[p for p,w in manifest.items() if hashlib.sha256((r/p).read_bytes()).hexdigest()!=w]
receipt=dict(exitcode=int(sys.argv[3]),owned_remaining=remaining,input_changes=changed,
    run_path=str(r),ros_nodes_started=False,devices_opened=False)
(out/'receipt.json').write_text(json.dumps(receipt,indent=2))
print(json.dumps(receipt))
PY
exit "$CODE"
