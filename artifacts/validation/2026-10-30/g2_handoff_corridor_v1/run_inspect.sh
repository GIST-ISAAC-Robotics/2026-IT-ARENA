#!/usr/bin/env bash
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_handoff_corridor_v1
test ! -e "$base/inspect1.log"
python3 -B "$base/inspect_closed.py" > "$base/inspect1.log" 2>&1
