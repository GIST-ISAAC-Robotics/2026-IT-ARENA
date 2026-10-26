#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONDONTWRITEBYTECODE=1
B=artifacts/validation/2026-10-26/m5_start_constraints_v1
O="$B/run47_gazebo_transfer"
[ "$(cat "$B/run46_ros_preflight/exit_code.txt")" = 0 ] || exit 92
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
python3 -B - "$O" <<'PY'
import importlib,hashlib,json,subprocess,sys
from pathlib import Path
out=Path(sys.argv[1]);root=Path.cwd()
tokens=('gz sim','gzserver','parameter_bridge','local_pursuit','lidar_safety','validate_g1_traffic.py')
processes=[line for line in subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
           if any(token in line for token in tokens)]
rows=[]
for name in ('local_pursuit','m3_execution_trial','polygon_batch','rounded_occupancy_candidate','lidar_safety'):
    module=importlib.import_module('arena_autonomy.'+name)
    installed=Path(module.__file__);source=root/'src/arena_autonomy/arena_autonomy'/(name+'.py')
    rows.append(dict(module=name,installed=str(installed),match=installed.read_bytes()==source.read_bytes(),
                     sha256=hashlib.sha256(source.read_bytes()).hexdigest()))
(out/'preflight.json').write_text(json.dumps(dict(processes=processes,imports=rows),indent=2))
assert not processes, 'preexisting simulation; do not interrupt it'
assert all(row['match'] for row in rows), 'installed module differs from source'
PY
pre=$?
printf '%s\n' "$pre" > "$O/preflight_exit_code.txt"
if [ "$pre" -ne 0 ]; then exit "$pre"; fi
CMD=(python3 -B scripts/validate_g1_traffic.py --case stop_restart --seed 0 --mode m3 --m3-command-fallback --m3-release-to-baseline --m3-small-center-straight --m3-straight-surface-commitment --m3-rounded-occupancy --m3-straight-with-threat --m3-defer-center-on-straight --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --actor-overlay config/tests/e3_stop_restart_v1.json --output "$O/run")
printf '%q ' "${CMD[@]}" > "$O/command.txt"
printf '\n' >> "$O/command.txt"
"${CMD[@]}" > "$O/stdout.txt" 2> "$O/stderr.txt"
run=$?
printf '%s\n' "$run" > "$O/run_exit_code.txt"
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
if [ -f "$O/run/follow_decisions.jsonl" ]; then
    python3 -B scripts/replay_follow_trace.py "$O/run/follow_decisions.jsonl" --output "$O/follow_trace_replay.json" > "$O/replay_stdout.txt" 2> "$O/replay_stderr.txt"
    printf '%s\n' "$?" > "$O/replay_exit_code.txt"
fi
if [ -f "$O/run/report.json" ]; then
    python3 -B scripts/surface_contact_eval.py "$O/run" --output "$O/surface_contact_eval.json" > "$O/surface_stdout.txt" 2> "$O/surface_stderr.txt"
    printf '%s\n' "$?" > "$O/surface_exit_code.txt"
fi
python3 -B - "$O" <<'PY'
import json,subprocess,sys
from pathlib import Path
out=Path(sys.argv[1])
tokens=('gz sim','gzserver','parameter_bridge','local_pursuit','lidar_safety','validate_g1_traffic.py')
rows=[line for line in subprocess.check_output(['ps','-eo','pid,args'],text=True).splitlines()
      if any(token in line for token in tokens)]
(out/'residual_processes.json').write_text(json.dumps(rows,indent=2))
PY
printf '%s\n' "$run" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$run"
