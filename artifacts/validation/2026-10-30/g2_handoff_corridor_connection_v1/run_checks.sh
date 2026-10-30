#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:${PYTHONPATH:-}"
base=artifacts/validation/2026-10-30/g2_handoff_corridor_connection_v1
test ! -e "$base/tests1.log"
python3 -B -m pytest -q tests/test_bypass*.py tests/test_g2*.py tests/test_source_provenance_r0_plus_b3.py > "$base/tests1.log" 2>&1
python3 -B "$base/baseline.py" after > "$base/after.log" 2>&1
python3 -B "$base/check_candidate_replay.py" > "$base/candidate_replay1.log" 2>&1
colcon build --packages-select arena_autonomy > "$base/build1.log" 2>&1
