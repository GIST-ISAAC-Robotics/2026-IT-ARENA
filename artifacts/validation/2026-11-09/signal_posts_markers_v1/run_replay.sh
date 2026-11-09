#!/usr/bin/env bash
# 사용: run_replay.sh <script.py> <label>  (WSL ROS Jazzy, 저장 입력 재생만)
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-09/signal_posts_markers_v1'
set +u; source /opt/ros/jazzy/setup.bash; source ../../../../install/local_setup.bash; set -u
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/../../../../src/arena_autonomy:$PWD/../../../../src/arena_vehicle_interface:$PWD${PYTHONPATH:+:$PYTHONPATH}"
python3 -B "$1" > "checks/$2.log" 2>&1
result=$?
echo "ACTUAL_EXIT_CODE=$result" > "checks/$2.exit"
exit "$result"
