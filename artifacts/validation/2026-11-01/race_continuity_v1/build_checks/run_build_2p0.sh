#!/usr/bin/env bash
set -o pipefail
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
printf '%s\n' '$ colcon build --packages-select arena_bringup arena_vehicle_interface --symlink-install'
colcon build --packages-select arena_bringup arena_vehicle_interface --symlink-install 2>&1 | tee artifacts/validation/2026-11-01/race_continuity_v1/build_checks/colcon_build_2p0.log
rc=${PIPESTATUS[0]}
printf 'ACTUAL_EXIT_CODE=%s\n' "$rc" | tee -a artifacts/validation/2026-11-01/race_continuity_v1/build_checks/colcon_build_2p0.log
exit "$rc"
