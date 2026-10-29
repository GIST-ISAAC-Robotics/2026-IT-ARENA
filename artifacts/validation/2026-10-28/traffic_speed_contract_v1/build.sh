#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
colcon build --packages-select arena_autonomy --symlink-install > artifacts/validation/2026-10-28/traffic_speed_contract_v1/build.log 2>&1
