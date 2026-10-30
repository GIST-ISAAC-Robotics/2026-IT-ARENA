#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-30/g2_presteer_v1
test ! -e "$base/closed1.log"
set +e
/usr/bin/python3 -B "$base/run_closed.py" > "$base/closed1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/closed1.exit"
exit "$rc"
