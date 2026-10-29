#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
base=artifacts/validation/2026-10-29/bump_traffic_review_v1
test ! -e "$base/ros_diagnostic_v1"
test ! -e "$base/ros_diagnostic_v1.log"
set +e
/usr/bin/python3 -B artifacts/validation/2026-10-29/bump_traffic_review_v1/diagnose_ros.py --output "$base/ros_diagnostic_v1" --partial-observation-policy bounded_local_v4 --traffic-recovery-trial > "$base/ros_diagnostic_v1.log" 2>&1
result=$?
printf '%s\n' "$result" > "$base/ros_diagnostic_v1.exit"
tail -n 10 "$base/ros_diagnostic_v1.log"
exit "$result"
