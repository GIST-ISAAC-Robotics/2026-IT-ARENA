#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PYTHONPATH"
base=artifacts/validation/2026-10-30/g2_outward_replan_v1
test ! -e "$base/review1.log"
set +e
(
 set -e
 /usr/bin/python3 -B -m pytest -q "$base/test_candidate_v2.py"
 /usr/bin/python3 -B "$base/audit.py"
 /usr/bin/python3 -B "$base/shadow_v2.py"
) > "$base/review1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/review1.exit"
exit "$rc"
