#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
BASE="$PWD/artifacts/validation/2026-10-28/traffic_residual_v1"
set +e
/usr/bin/python3 -B -m pytest -q "$BASE/test_safety_shadow.py" > "$BASE/safety_tests.log" 2>&1
code=$?
printf '%s\n' "$code" > "$BASE/safety_tests.exit"
if [ "$code" -ne 0 ]; then exit "$code"; fi
/usr/bin/python3 -B "$BASE/replay_safety.py" > "$BASE/safety_replay.log" 2>&1
code=$?
printf '%s\n' "$code" > "$BASE/safety_replay.exit"
exit "$code"
