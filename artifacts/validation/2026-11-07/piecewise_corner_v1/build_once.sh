#!/usr/bin/env bash
set +u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 2
set +e
source /opt/ros/jazzy/setup.bash
source_status=$?
if [ "$source_status" -ne 0 ]; then
  printf 'SOURCE_EXIT_CODE=%s\n' "$source_status" > artifacts/validation/2026-11-07/piecewise_corner_v1/build.exit
  exit "$source_status"
fi
colcon build --symlink-install --packages-select arena_autonomy arena_bringup > artifacts/validation/2026-11-07/piecewise_corner_v1/build.log 2>&1
build_status=$?
printf 'ACTUAL_EXIT_CODE=%s\n' "$build_status" > artifacts/validation/2026-11-07/piecewise_corner_v1/build.exit
exit "$build_status"
