#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
# 빌드 전 source에서 남은 develop/build 경로를 이 검사 프로세스에서만 제거한다.
export PYTHONPATH=''
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export ROS_DOMAIN_ID=226
export ROS_LOCALHOST_ONLY=1
base=artifacts/validation/2026-10-29/g2_follow_reserve_v1
test ! -e "$base/ros_options2.log" || exit 73
set +e
python3 -B "$base/check_ros_options.py" --output ros_options2 > "$base/ros_options2.log" 2>&1
result=$?
echo "$result" > "$base/ros_options2.exit"
exit "$result"
