#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/setup.bash
trial_dir='artifacts/validation/2026-10-29/g2_tracking_preview_v1'
if [ -e "$trial_dir/prototype2" ] || [ -e "$trial_dir/tests_v2.log" ]; then
  exit 73
fi
set +e
python3 -B "$trial_dir/test_prediction.py" > "$trial_dir/tests_v2.log" 2>&1
test_status=$?
printf '%s\n' "$test_status" > "$trial_dir/tests_v2.exit"
if [ "$test_status" -ne 0 ]; then
  exit "$test_status"
fi
python3 -B "$trial_dir/run_prototype.py" > "$trial_dir/prototype2.log" 2>&1
trial_status=$?
printf '%s\n' "$trial_status" > "$trial_dir/prototype2.exit"
exit "$trial_status"
