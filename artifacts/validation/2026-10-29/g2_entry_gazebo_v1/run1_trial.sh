#!/usr/bin/env bash
# G2 run1: 등록한 1회 실행. 실패해도 자동 반복하지 않는다.
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/g2_entry_gazebo_v1
test ! -e "$base/run1" || exit 90
ps -eo pid,etime,cmd | grep -E "gz sim|ros2|parameter_bridge|arena_" | grep -v grep > "$base/run1_preexisting_processes.txt"
/usr/bin/python3 -B "$base/run.py" --output "$PWD/$base/run1" > "$base/run1_trial.log" 2>&1
result=$?
echo "$result" > "$base/run1_exit.txt"
sleep 3
ps -eo pid,etime,cmd | grep -E "gz sim|ros2|parameter_bridge|arena_" | grep -v grep > "$base/run1_residual_processes.txt"
exit "$result"
