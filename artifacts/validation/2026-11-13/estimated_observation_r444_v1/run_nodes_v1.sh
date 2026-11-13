#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
ARTIFACT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/estimated_observation_r444_v1'
PROFILE='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2'
RUN='/home/jinhyeong/it-arena-r444-verify-b'
python3 - "$ARTIFACT" "$PROFILE" "$RUN" <<'PY'
from pathlib import Path
import hashlib,json,shutil,sys
a,p,r=map(Path,sys.argv[1:]);out=r/'node_check';out.mkdir(exist_ok=False)
manifest=json.loads((r/'manifest.json').read_bytes())
assert all(hashlib.sha256((r/n).read_bytes()).hexdigest()==w for n,w in manifest.items())
shutil.copy2(a/'verify_nodes.py',out/'verify_nodes.py')
for name in ('runtime_profile.yaml','bindings.json'):shutil.copy2(p/name,out/name)
(out/'inputs.json').write_text(json.dumps({f.name:hashlib.sha256(f.read_bytes()).hexdigest() for f in out.iterdir()},indent=2))
PY
cd "$RUN"
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$RUN/src/arena_autonomy:$RUN/src/arena_vehicle_interface:${PYTHONPATH:-}"
export ROS_DOMAIN_ID=144
set +e
timeout --signal=TERM 90s python3 -B node_check/verify_nodes.py > node_check/execution.log 2>&1
CODE=$?
set -e
python3 - "$ARTIFACT" "$RUN" "$CODE" <<'PY'
from pathlib import Path
import hashlib,json,os,shutil,sys
a,r=map(Path,sys.argv[1:3]);out=a/'verify_nodes_v1';shutil.copytree(r/'node_check',out)
remaining=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) in (os.getpid(),os.getppid()):continue
    try:
        if (p/'cwd').resolve(strict=True).is_relative_to(r):remaining.append(dict(pid=int(p.name),command=(p/'cmdline').read_bytes().replace(b'\0',b' ').decode()))
    except (FileNotFoundError,PermissionError,ProcessLookupError):pass
manifest=json.loads((r/'manifest.json').read_bytes())
changed=[n for n,w in manifest.items() if hashlib.sha256((r/n).read_bytes()).hexdigest()!=w]
node_manifest=json.loads((r/'node_check/inputs.json').read_bytes())
node_changes=[n for n,w in node_manifest.items() if hashlib.sha256((r/'node_check'/n).read_bytes()).hexdigest()!=w]
receipt=dict(exitcode=int(sys.argv[3]),owned_remaining=remaining,input_changes=changed,node_input_changes=node_changes)
(out/'receipt.json').write_text(json.dumps(receipt,indent=2));print(json.dumps(receipt))
if (out/'report.json').exists():print((out/'report.json').read_text())
PY
exit "$CODE"
