#!/usr/bin/env bash
set -e
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
cd "$repo"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$repo/src/arena_autonomy:$repo/src/arena_vehicle_interface${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m pytest \
  tests/test_local_pursuit.py \
  tests/test_lidar_safety.py \
  tests/test_lidar_observation.py \
  tests/test_lidar_motion.py \
  tests/test_rotating_lidar.py \
  -q
