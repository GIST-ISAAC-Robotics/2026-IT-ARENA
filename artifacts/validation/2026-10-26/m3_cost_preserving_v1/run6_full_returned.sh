#!/usr/bin/env bash
set +u
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:${PYTHONPATH:-}"
BASE=artifacts/validation/2026-10-26/m3_cost_preserving_v1
O="$BASE/run6_full_returned"
mkdir "$O" || exit 91
date -Is > "$O/start.txt"
python3 -B "$BASE/compare_returned.py" --full --output "$O/data" > "$O/stdout.txt" 2> "$O/stderr.txt"
code=$?
printf '%s\n' "$code" > "$O/exit_code.txt"
date -Is > "$O/end.txt"
exit "$code"
