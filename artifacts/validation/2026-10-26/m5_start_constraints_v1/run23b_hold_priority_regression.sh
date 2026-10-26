#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m5_start_constraints_v1/run23b_hold_priority_regression
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
cp src/arena_autonomy/arena_autonomy/m3_execution_trial.py "$O/controller.py"
cp tests/test_m3_straight_before_hold.py "$O/test_source.py"
sha256sum src/arena_autonomy/arena_autonomy/m3_execution_trial.py tests/test_m3_straight_before_hold.py scripts/simulate_grid_departure.py > "$O/sources_before.sha256"
python3 -B -m pytest -q tests/test_m3*.py tests/test_grid_departure_dynamics.py tests/test_grid_departure_motion_audit.py > "$O/stdout.txt" 2> "$O/stderr.txt"
result=$?
sha256sum -c "$O/sources_before.sha256" > "$O/source_check.txt" || result=92
printf '%s\n' "$result" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$result"
