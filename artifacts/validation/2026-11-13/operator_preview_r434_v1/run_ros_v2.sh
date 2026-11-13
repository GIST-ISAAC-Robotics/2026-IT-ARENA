#!/bin/bash
set -e
source /opt/ros/jazzy/setup.bash
root=/home/jinhyeong/it-arena-r434-preview-b
bundle='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/operator_preview_r434_v1'
test ! -e "$root"
mkdir "$root"
python3 -m zipfile -e "$bundle/ros_sources_v2.zip" "$root"
cd "$root"
export PYTHONPATH="$root:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1 ROS_DOMAIN_ID=231 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 -B ros_preview_probe.py --output "$root/result_v2" > "$root/run_v2.log" 2>&1
