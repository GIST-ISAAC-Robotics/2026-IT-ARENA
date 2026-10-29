#!/usr/bin/env bash
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local' || exit 89
source /opt/ros/jazzy/setup.bash || exit 91
source install/local_setup.bash || exit 92
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/g2_entry_left_v1
echo "output=$PWD/$base/run1"
/usr/bin/python3 -B "$base/run.py" --prepare-only --output "$PWD/$base/run1" > "$base/run1_prepare.log" 2>&1
echo $? > "$base/run1_prepare.exit"
