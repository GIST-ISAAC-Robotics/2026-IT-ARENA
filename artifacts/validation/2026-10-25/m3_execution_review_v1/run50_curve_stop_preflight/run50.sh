#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
A=artifacts/validation/2026-10-25/m3_execution_review_v1
D="$A/run50_curve_stop_preflight"
[ ! -e "$D/exit_code.txt" ] || exit 91
cp "$A/curve_stop_preflight.py" "$D/preflight_source.py"
cp config/tests/g1_curve_stop_restart_v1.json "$D/config.json"
python3 -B "$D/preflight_source.py" "$D/result.json" > "$D/stdout.txt" 2> "$D/stderr.txt"
result_code=$?
printf '%s\n' "$result_code" > "$D/exit_code.txt"
exit "$result_code"
