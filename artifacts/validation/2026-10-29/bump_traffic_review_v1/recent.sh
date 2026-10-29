#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-29/bump_traffic_review_v1
test ! -e "$base/recent.log"
set +e
/usr/bin/python3 -B "$base/probe_recent_clearance.py" > "$base/recent.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/recent.exit"
exit "$result"
