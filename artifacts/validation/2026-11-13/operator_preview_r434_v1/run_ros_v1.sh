#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
root=/home/jinhyeong/it-arena-r434-preview-a
bundle='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/operator_preview_r434_v1'
test ! -e "$root"
mkdir "$root"
python3 -m zipfile -e "$bundle/ros_sources_v2.zip" "$root"
cd "$root"
export PYTHONPATH="$root:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=234 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOCALHOST_ONLY=1
python3 -B ros_preview_probe.py --output "$root/result_v1" > "$root/run_v1.log" 2>&1
