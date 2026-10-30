#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_center_offsets_v1
test ! -e "$base/comparison1.log"
set +e
/usr/bin/python3 -B "$base/run.py" > "$base/comparison1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/comparison1.exit"
exit "$rc"
