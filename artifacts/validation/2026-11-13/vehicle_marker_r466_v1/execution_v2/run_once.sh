#!/usr/bin/env bash
# root의 WSL 소유권 배정 뒤 한 번만 실행. 준비 단계에서는 호출하지 않는다.
set -eo pipefail
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
artifact="$repo/artifacts/validation/2026-11-13/vehicle_marker_r466_v1"
receipt="$artifact/execution_v2"
work='/home/jinhyeong/it-arena-r466-ros-b'
test ! -e "$receipt/run1"
test ! -e "$work"
mkdir "$receipt/once_gate"
printf '{"shell_pid":%s,"started_utc":"%s"}\n' "$$" "$(date -u +%FT%TZ)" > "$receipt/started.json"
source /opt/ros/jazzy/setup.bash
source "$repo/install/setup.bash"
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=96
export PYTHONDONTWRITEBYTECODE=1
mkdir "$work"
python3 -B - "$receipt" "$work" <<'PY'
import hashlib,json,sys,zipfile
from pathlib import Path
receipt,work=map(Path,sys.argv[1:])
spec=json.loads((receipt/'prepared.json').read_bytes())
archive=receipt/'source.zip'
assert hashlib.sha256(archive.read_bytes()).hexdigest()==spec['source_archive_sha256']
with zipfile.ZipFile(archive) as z:
    names=z.namelist()
    assert all(not Path(n).is_absolute() and '..' not in Path(n).parts for n in names)
    z.extractall(work)
for row in spec['files']:
    assert hashlib.sha256((work/row['path']).read_bytes()).hexdigest()==row['sha256'],row['path']
PY
export PYTHONPATH="$work/src/arena_autonomy:$work/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
set +e
timeout --signal=INT --kill-after=5s 30s python3 -B "$work/validation/run_ros_validation.py" \
  --profile "$work/config/vehicle_marker_approx_b0390.json" \
  --output "$work/run1" --import-tests "$work/tests/test_local_follow.py" \
  > "$receipt/stdout.log" 2> "$receipt/stderr.log"
test_exit=$?
set -e
python3 -B - "$work" "$receipt" "$test_exit" <<'PY'
import hashlib,json,os,shutil,sys
from pathlib import Path
work,receipt=map(Path,sys.argv[1:3]);code=int(sys.argv[3]);spec=json.loads((receipt/'prepared.json').read_bytes())
result_path=work/'run1/result.json';result=json.loads(result_path.read_bytes()) if result_path.exists() else {}
alive=[]
for pid in result.get('child_processes',[]):
    try:os.kill(pid,0)
    except ProcessLookupError:continue
    alive.append(pid)
frozen=all(hashlib.sha256((work/row['path']).read_bytes()).hexdigest()==row['sha256'] for row in spec['files'])
if (work/'run1').exists():shutil.copytree(work/'run1',receipt/'run1')
audit=dict(test_exit_code=code,source_snapshot_unchanged=frozen,reported_child_pids_alive=alive,
    result_present=bool(result),ros_nodes_destroyed=result.get('nodes_destroyed'),
    vision_cleanup=result.get('vision_cleanup'),scope='owned Python and VisionProcess, no hardware processes started')
(receipt/'completion.json').write_text(json.dumps(audit,indent=2)+'\n')
print(json.dumps(audit,indent=2))
PY
exit "$test_exit"
