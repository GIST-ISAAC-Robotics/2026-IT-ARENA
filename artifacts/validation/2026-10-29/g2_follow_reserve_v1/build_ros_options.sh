#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/g2_follow_reserve_v1
test ! -e "$base/build1.log" || exit 73
set +e
colcon build --packages-select arena_autonomy --event-handlers console_direct+ > "$base/build1.log" 2>&1
result=$?
echo "$result" > "$base/build1.exit"
test "$result" -eq 0 || exit "$result"
set -e
source install/local_setup.bash
export ROS_DOMAIN_ID=226
export ROS_LOCALHOST_ONLY=1
test ! -e "$base/ros_options1.log" || exit 73
set +e
python3 -B "$base/check_ros_options.py" > "$base/ros_options1.log" 2>&1
result=$?
echo "$result" > "$base/ros_options1.exit"
exit "$result"
