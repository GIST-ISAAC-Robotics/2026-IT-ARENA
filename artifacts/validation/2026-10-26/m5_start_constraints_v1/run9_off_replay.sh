#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m5_start_constraints_v1/run9_off_replay
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
python3 -B -m pytest -q tests/test_m3*.py tests/test_lateral_intent.py tests/test_observed_lateral_trial.py tests/test_scan_topology.py tests/test_startup_straight_state.py tests/test_rounded_occupancy_candidate.py > "$O/tests_stdout.txt" 2> "$O/tests_stderr.txt"
code=$?
printf '%s\n' "$code" > "$O/tests_exit_code.txt"
if [ "$code" -ne 0 ]; then exit "$code"; fi
python3 -B artifacts/validation/2026-10-26/m3_cost_preserving_v1/verify_integrated.py --full --output "$O/data" > "$O/stdout.txt" 2> "$O/stderr.txt"
code=$?
printf '%s\n' "$code" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$code"
