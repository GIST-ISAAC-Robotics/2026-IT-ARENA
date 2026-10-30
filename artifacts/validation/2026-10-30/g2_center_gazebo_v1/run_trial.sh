#!/usr/bin/env bash
# 등록된 중앙 표적 Gazebo 1회. 출력 폴더가 이미 있으면 실행하지 않는다.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
set +u
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_center_gazebo_v1
test ! -e "$base/run1" || exit 93
pgrep -af 'gz sim|ros2 launch|g2_entry_trial|lidar_safety' > "$base/run1_preexisting_processes.txt"
timeout 1200 /usr/bin/python3 -B "$base/run.py" --output "$PWD/$base/run1" > "$base/run1_trial.log" 2>&1
code=$?
printf '%s\n' "$code" > "$base/run1_exit.txt"
sleep 3
pgrep -af 'gz sim|ros2 launch|g2_entry_trial|lidar_safety' > "$base/run1_residual_processes.txt"
exit "$code"
