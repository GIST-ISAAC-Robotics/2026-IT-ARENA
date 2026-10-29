#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
BASE="$PWD/artifacts/validation/2026-10-28/traffic_recovery_connection_v1"
OLD="$PWD/artifacts/validation/2026-10-28/traffic_residual_v1"
export PYTHONPATH="$BASE:$OLD:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$BASE/runtime_replay.log"
set +e
/usr/bin/python3 -B "$BASE/replay_runtime.py" > "$BASE/runtime_replay.log" 2>&1
code=$?
printf '%s\n' "$code" > "$BASE/runtime_replay.exit"
exit "$code"
