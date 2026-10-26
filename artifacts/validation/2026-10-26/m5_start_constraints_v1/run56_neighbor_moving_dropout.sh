#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m5_start_constraints_v1/run56_neighbor_moving_dropout
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
cp scripts/simulate_grid_departure.py "$O/simulation_source.py"
cp scripts/audit_grid_departure_motion.py "$O/audit_source.py"
cp scripts/probe_grid_neighbor_departure.py "$O/probe_source.py"
cp tests/test_grid_neighbor_departure.py "$O/test_source.py"
python3 -B -m pytest -q tests/test_grid_neighbor_departure.py tests/test_grid_departure_dynamics.py tests/test_grid_departure_motion_audit.py tests/test_grid_departure_impairment.py > "$O/tests_stdout.txt" 2> "$O/tests_stderr.txt"
result=$?
printf '%s\n' "$result" > "$O/tests_exit_code.txt"
if [ "$result" -eq 0 ]; then
    python3 -B scripts/probe_grid_neighbor_departure.py --moving-dropout-only --output "$O/data" > "$O/stdout.txt" 2> "$O/stderr.txt"
    result=$?
fi
printf '%s\n' "$result" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$result"
