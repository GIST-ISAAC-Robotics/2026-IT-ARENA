#!/usr/bin/env bash
set -uo pipefail
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
A=artifacts/validation/2026-10-25/m3_execution_review_v1
D=$A/run32_corner_after_threat
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="src/arena_autonomy:src/arena_vehicle_interface:scripts:tests:${PYTHONPATH:-}"
python3 -B "$A/corner_after_threat_preflight.py" "$D/preflight.json" > "$D/preflight_stdout.txt" 2> "$D/preflight_stderr.txt"
echo $? > "$D/preflight_exitcode.txt"
