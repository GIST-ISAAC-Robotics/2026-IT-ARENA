#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
a='artifacts/validation/2026-11-14/measured_steering_lap_r544_attempt2_v1'
test ! -e "$a/prepared.json"
test ! -e "$a/run1"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$a/preflight.py" > "$a/preflight/import_final2.log" 2>&1
python3 -B "$a/prepare.py" --freeze > "$a/preflight/freeze.json"
python3 -B "$a/prepare.py" --check > "$a/preflight/final_check.json"
cat "$a/preflight/freeze.json"
