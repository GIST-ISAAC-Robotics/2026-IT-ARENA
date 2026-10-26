#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m5_start_constraints_v1/run7
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
python3 -B -m pytest -q tests/test_grid_departure_observation.py tests/test_rounded_occupancy_candidate.py tests/test_startup_straight_state.py tests/test_startup_straight_candidate.py tests/test_grid_start_constraints.py tests/test_grid_follow_probe.py > "$O/tests_stdout.txt" 2> "$O/tests_stderr.txt"
checked=$?
printf '%s\n' "$checked" > "$O/tests_exit_code.txt"
if [ "$checked" -ne 0 ]; then exit "$checked"; fi
python3 -B scripts/probe_grid_departure_observation.py --output "$O/data" > "$O/probe_stdout.txt" 2> "$O/probe_stderr.txt"
result=$?
printf '%s\n' "$result" > "$O/probe_exit_code.txt"
date -Is > "$O/end.txt"
exit "$result"




