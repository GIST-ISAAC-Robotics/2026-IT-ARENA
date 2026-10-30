#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PYTHONPATH"
base=artifacts/validation/2026-10-30/g2_side_nonapproach_v1
test ! -e "$base/analysis2.log"
set +e
(
 set -e
 /usr/bin/python3 -B "$base/analyze_residual.py"
) > "$base/analysis2.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/analysis2.exit"
exit "$rc"
