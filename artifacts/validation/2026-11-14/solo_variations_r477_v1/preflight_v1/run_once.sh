#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
a='artifacts/validation/2026-11-14/solo_variations_r477_v1'
r="$a/preflight_v1"
mkdir "$r/once_gate"
printf '%s\n' "$$" > "$r/shell.pid"
date --utc --iso-8601=seconds > "$r/started_utc.txt"
finish() {
  code=$?
  trap - EXIT
  printf '%s\n' "$code" > "$r/session.exit"
  python3 -B - "$r" "$$" "$code" <<'PY'
import datetime,json,os,sys
from pathlib import Path
out=Path(sys.argv[1]);shell=int(sys.argv[2]);rows=[]
for path in Path('/proc').iterdir():
    if not path.name.isdigit() or int(path.name) in (os.getpid(),shell):continue
    try:
        args=[x.decode(errors='replace') for x in (path/'cmdline').read_bytes().split(b'\0') if x]
        text=' '.join(args)
        if 'solo_variations_r477_v1' in text and any(x in text for x in ('runtime_preflight.py','prepare_execution.py','preflight_v1/run_once.sh')):
            rows.append(dict(pid=int(path.name),argv=args))
    except (FileNotFoundError,PermissionError):pass
value=dict(at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),session_exit=int(sys.argv[3]),
    remaining_owned_preflight_work=rows,scope='r477 synchronous preflight/freeze processes; no ROS nodes or Gazebo started')
(out/'completion.json').write_text(json.dumps(value,indent=2)+'\n')
print(json.dumps(value))
assert not rows
PY
  audit=$?
  date --utc --iso-8601=seconds > "$r/finished_utc.txt"
  if [ "$code" -ne 0 ]; then exit "$code"; fi
  exit "$audit"
}
trap finish EXIT
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$a/runtime_preflight.py" > "$r/preflight.stdout" 2> "$r/preflight.stderr"
python3 -B "$a/prepare_execution.py" --freeze > "$r/freeze.stdout" 2> "$r/freeze.stderr"
cat "$r/freeze.stdout"
