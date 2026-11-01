#!/usr/bin/env bash
cd "/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local" || exit 90
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
set -o pipefail
printf '%s\n' '$ colcon test --packages-select arena_gazebo --event-handlers console_direct+'
colcon test --packages-select arena_gazebo --event-handlers console_direct+ 2>&1 | tee artifacts/validation/2026-11-01/race_continuity_v1/build_checks/colcon_test.log
rc=${PIPESTATUS[0]}
printf 'ACTUAL_EXIT_CODE=%s\n' "$rc" | tee -a artifacts/validation/2026-11-01/race_continuity_v1/build_checks/colcon_test.log
printf '%s\n' '$ colcon test-result --test-result-base build/arena_gazebo --verbose'
colcon test-result --test-result-base build/arena_gazebo --verbose 2>&1 | tee artifacts/validation/2026-11-01/race_continuity_v1/build_checks/colcon_test_result.log
rr=${PIPESTATUS[0]}
printf 'ACTUAL_EXIT_CODE=%s\n' "$rr" | tee -a artifacts/validation/2026-11-01/race_continuity_v1/build_checks/colcon_test_result.log
if [ "$rc" -ne 0 ]; then exit "$rc"; else exit "$rr"; fi
