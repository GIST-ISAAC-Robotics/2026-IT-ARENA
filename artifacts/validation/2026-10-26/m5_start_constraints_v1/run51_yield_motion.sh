#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
B=artifacts/validation/2026-10-26/m5_start_constraints_v1
O="$B/run51_yield_motion"
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
cp src/arena_autonomy/arena_autonomy/m3_execution_trial.py "$O/source.py"
python3 -B scripts/replay_follow_trace.py "$B/run47_gazebo_transfer/run/follow_decisions.jsonl" --output "$O/gazebo_saved_replay.json" > "$O/replay_stdout.txt" 2> "$O/replay_stderr.txt"
replay=$?
printf '%s\n' "$replay" > "$O/replay_exit_code.txt"
python3 -B scripts/probe_grid_deferred_center.py --output "$O/data" > "$O/stdout.txt" 2> "$O/stderr.txt"
motion=$?
printf '%s\n' "$motion" > "$O/motion_exit_code.txt"
python3 -B - "$O" <<'PY'
import json,sys
from pathlib import Path
out=Path(sys.argv[1]);r=json.loads((out/'gazebo_saved_replay.json').read_text())
source=Path('src/arena_autonomy/arena_autonomy/m3_execution_trial.py')
valid=r['calls']>0 and r['input_unchanged'] and source.read_bytes()==(out/'source.py').read_bytes()
(out/'comparison_integrity.json').write_text(json.dumps(dict(comparison_completed=valid,
    saved_outputs_match=r['passed'],source_unchanged=source.read_bytes()==(out/'source.py').read_bytes(),
    scope='saved trajectory comparison; any changed actions need new trajectory evidence')))
assert valid
PY
integrity=$?
result=$motion
if [ "$integrity" -ne 0 ]; then result=$integrity; fi
printf '%s\n' "$result" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$result"
