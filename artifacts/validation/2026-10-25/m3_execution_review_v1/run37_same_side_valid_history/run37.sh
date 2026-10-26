#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH='src/arena_autonomy:src/arena_vehicle_interface:scripts:tests'
D=artifacts/validation/2026-10-25/m3_execution_review_v1/run37_same_side_valid_history
[ ! -e "$D/result.json" ] || exit 91
cp artifacts/validation/2026-10-25/m3_execution_review_v1/probe_same_side_v2.py "$D/probe_source.py"
python3 -B "$D/probe_source.py" "$D/result.json" > "$D/stdout.txt" 2> "$D/stderr.txt"
result_code=$?
printf '%s\n' "$result_code" > "$D/exit_code.txt"
exit "$result_code"
