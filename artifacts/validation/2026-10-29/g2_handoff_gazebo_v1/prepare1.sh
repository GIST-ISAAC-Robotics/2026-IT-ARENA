#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/g2_handoff_gazebo_v1
test ! -e "$base/build1.log" || exit 73
python3 -B "$base/test_g2_eval_core.py" > "$base/evaluator_tests1.log" 2>&1
colcon build --packages-select arena_autonomy --symlink-install > "$base/build1.log" 2>&1
source install/local_setup.bash
ros2 pkg executables arena_autonomy > "$base/installed_entries1.log"
python3 -B "$base/prepare_fixture.py" > "$base/preparation1.log" 2>&1
python3 -B "$base/run.py" --prepare-only --output "$PWD/$base/run1" > "$base/run1_prepare.log" 2>&1
