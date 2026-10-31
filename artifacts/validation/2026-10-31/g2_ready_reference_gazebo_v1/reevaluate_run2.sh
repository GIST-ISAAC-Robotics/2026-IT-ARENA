#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-31/g2_ready_reference_gazebo_v1
test ! -e "$base/run2_evaluation_v2.json"
python3 -B -m pytest -q -p no:cacheprovider "$base/test_evaluate_v2.py" > "$base/run2_evaluation_v2_tests.log" 2>&1
python3 -B "$base/evaluate_v2.py" "$base/run2" --output "$base/run2_evaluation_v2.json" > "$base/run2_evaluation_v2.log" 2>&1
