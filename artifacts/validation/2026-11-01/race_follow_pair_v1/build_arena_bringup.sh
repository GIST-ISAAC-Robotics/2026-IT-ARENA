#!/usr/bin/env bash
set -o pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
colcon build --packages-select arena_bringup --symlink-install 2>&1 | tee artifacts/validation/2026-11-01/race_follow_pair_v1/colcon_build_arena_bringup.log
result=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$result" >> artifacts/validation/2026-11-01/race_follow_pair_v1/colcon_build_arena_bringup.log
exit "$result"