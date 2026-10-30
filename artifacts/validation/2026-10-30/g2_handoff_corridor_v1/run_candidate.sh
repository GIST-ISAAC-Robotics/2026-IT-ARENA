#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:${PYTHONPATH:-}"
base=artifacts/validation/2026-10-30/g2_handoff_corridor_v1
test ! -e "$base/candidate1.log"
python3 -B -m pytest -q "$base/test_candidate.py" > "$base/tests1.log" 2>&1
python3 -B "$base/inspect_candidate.py" > "$base/candidate1.log" 2>&1
