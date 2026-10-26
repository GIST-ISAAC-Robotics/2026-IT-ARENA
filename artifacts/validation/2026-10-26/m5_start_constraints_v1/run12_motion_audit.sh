#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/tests:$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
O=artifacts/validation/2026-10-26/m5_start_constraints_v1/run12_motion_audit
mkdir "$O" || exit 91
python3 -B -m pytest -q tests/test_grid_departure_motion_audit.py > "$O/tests_stdout.txt" 2> "$O/tests_stderr.txt"
code=$?
printf '%s\n' "$code" > "$O/tests_exit_code.txt"
if [ "$code" -ne 0 ]; then exit "$code"; fi
python3 -B scripts/audit_grid_departure_motion.py artifacts/validation/2026-10-26/m5_start_constraints_v1/run11_dynamics_ordered/data/report.json --output "$O/summary.json" > "$O/stdout.txt" 2> "$O/stderr.txt"
code=$?
printf '%s\n' "$code" > "$O/exit_code.txt"
exit "$code"
