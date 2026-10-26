#!/usr/bin/env bash
set -o pipefail
repo="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
cd "$repo" || exit 90
source /opt/ros/jazzy/setup.bash || exit 90
source install/setup.bash || exit 90
colcon build --packages-select arena_autonomy --symlink-install
