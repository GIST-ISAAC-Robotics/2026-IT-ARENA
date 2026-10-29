#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
trial_dir='artifacts/validation/2026-10-29/g2_handoff_v1'
if [ -e "$trial_dir/prototype2" ] || [ -e "$trial_dir/tests2.log" ]; then exit 73; fi
python3 -B "$trial_dir/test_handoff.py" > "$trial_dir/tests2.log" 2>&1
python3 -B "$trial_dir/probe_handoff.py" --output prototype2 > "$trial_dir/prototype2.log" 2>&1
