#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PYTHONPATH"
base=artifacts/validation/2026-10-30/g2_presteer_v1
test ! -e "$base/shadow_retry1.log"
set +e
(
 set -e
 /usr/bin/python3 -B -m pytest -q "$base/test_candidate.py"
 /usr/bin/python3 -B "$base/shadow.py"
) > "$base/shadow_retry1.log" 2>&1
rc=$?
printf '%s\n' "$rc" > "$base/shadow_retry1.exit"
exit "$rc"
