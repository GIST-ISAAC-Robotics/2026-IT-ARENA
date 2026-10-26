#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
set +u
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export PYTHONPATH="scripts:src/arena_autonomy:src/arena_vehicle_interface:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
A=artifacts/validation/2026-10-25/m3_execution_review_v1
D="$A/run58_narrow_lead_preflight"
[ ! -e "$D/exit_code.txt" ] || exit 91
cp "$A/narrow_lead_preflight.py" "$D/preflight_source.py"
cp config/tests/m2_narrow_offset_lead_v1.json "$D/config.json"
set +e
python3 -B "$D/preflight_source.py" "$D/result.json" > "$D/stdout.txt" 2> "$D/stderr.txt"
code=$?
printf '%s\n' "$code" > "$D/exit_code.txt"
exit "$code"
