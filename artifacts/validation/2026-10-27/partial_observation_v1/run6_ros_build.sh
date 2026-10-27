#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-27/partial_observation_v1
test ! -e "$base/run6_ros.log"
python3 -B scripts/validate_sensor_input_ros.py --partial-observation-policy bounded_side_v1 --output "$base/run6_ros" > "$base/run6_ros.log" 2>&1
colcon build --symlink-install --packages-select arena_autonomy arena_bringup arena_vehicle_interface > "$base/run6_build.log" 2>&1
cat "$base/run6_ros.log"
tail -2 "$base/run6_build.log"
