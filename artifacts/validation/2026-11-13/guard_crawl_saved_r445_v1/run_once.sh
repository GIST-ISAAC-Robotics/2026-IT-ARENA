#!/bin/bash
set -e
bundle='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/guard_crawl_saved_r445_v1'
root=/home/jinhyeong/it-arena-r445-guard-a
test ! -e "$root"
mkdir "$root"
python3 -m zipfile -e "$bundle/sources_v1.zip" "$root"
cd "$root"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$root/src/arena_autonomy:$root/src/arena_vehicle_interface"
set +e
python3 -B replay_guard.py > run.log 2>&1 &
pid=$!
echo "$pid" > run.pid
wait "$pid"
code=$?
echo "$code" > run.exit
set -e
cp -r result_v1 "$bundle/result_v1"
cp run.log run.pid run.exit "$bundle/"
python3 - "$pid" "$code" "$root" "$bundle" <<'PY'
import hashlib,json,pathlib,sys
pid,code,root,bundle=int(sys.argv[1]),int(sys.argv[2]),pathlib.Path(sys.argv[3]),pathlib.Path(sys.argv[4])
manifest=json.loads((root/'manifest.json').read_text())
changed=[n for n,h in manifest.items() if hashlib.sha256((root/n).read_bytes()).hexdigest()!=h]
remaining=[]
for proc in pathlib.Path('/proc').iterdir():
    if not proc.name.isdigit():continue
    try:cmd=(proc/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
    except (FileNotFoundError,PermissionError,ProcessLookupError):continue
    if proc.name==str(pid) or ('replay_guard.py' in cmd and not cmd.startswith(('bash ', '/bin/bash '))):remaining.append(dict(pid=int(proc.name),cmd=cmd))
receipt=dict(exitcode=code,child_pid=pid,child_reaped=True,source_changes=changed,remaining_owned_work=remaining,ros_started=False)
(bundle/'handoff_audit.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt))
assert not changed and not remaining
PY
exit "$code"
