#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m5_start_constraints_v1/run36_batch_regression
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
cp src/arena_autonomy/arena_autonomy/observed_lateral_trial.py "$O/source.py"
cp src/arena_autonomy/arena_autonomy/polygon_batch.py "$O/batch_source.py"
cp tests/test_polygon_batch.py "$O/test_batch_source.py"
cp tests/test_path_observation_prefilter.py "$O/test_query_source.py"
python3 -B -m pytest -q tests/test_polygon_batch.py tests/test_path_observation_prefilter.py tests/test_observed_lateral_trial.py tests/test_m3_execution_trial.py tests/test_m3_command_selection.py tests/test_m3_surface_commitment.py tests/test_m3_parallel_straight.py > "$O/stdout.txt" 2> "$O/stderr.txt"
result=$?
printf '%s\n' "$result" > "$O/exit_code.txt"
cmp -s src/arena_autonomy/arena_autonomy/observed_lateral_trial.py "$O/source.py" || exit 92
cmp -s src/arena_autonomy/arena_autonomy/polygon_batch.py "$O/batch_source.py" || exit 92
date -Is > "$O/end.txt"
exit "$result"
