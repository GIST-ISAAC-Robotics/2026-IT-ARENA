#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
base=artifacts/validation/2026-10-30/rear_return_from_pause_v1
test ! -e "$base/exit.txt"
trap 'printf "%s\n" "$?" > "$base/exit.txt"' EXIT
export PYTHONDONTWRITEBYTECODE=1
for profile in ideal combined; do
  python3 -B "$base/probe.py" --output "$base/$profile" --cases B_right_stopped_return --sensor-profile "$profile" --trace --rear-lane-keep > "$base/$profile.log" 2>&1
done
