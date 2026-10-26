#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m5_start_constraints_v1/run18_wrap_regression
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
cp src/arena_autonomy/arena_autonomy/m3_execution_trial.py "$O/controller.py"
cp tests/test_m3_parallel_straight.py "$O/test_source.py"
sha256sum src/arena_autonomy/arena_autonomy/m3_execution_trial.py tests/test_m3_parallel_straight.py > "$O/sources_before.sha256"
python3 -B -m pytest -q tests/test_m3*.py tests/test_grid_departure_dynamics.py tests/test_grid_departure_motion_audit.py tests/test_paired_wall_interval_probe.py > "$O/stdout.txt" 2> "$O/stderr.txt"
code=$?
printf '%s\n' "$code" > "$O/exit_code.txt"
sha256sum -c "$O/sources_before.sha256" > "$O/source_check.txt"
date -Is > "$O/end.txt"
exit "$code"
