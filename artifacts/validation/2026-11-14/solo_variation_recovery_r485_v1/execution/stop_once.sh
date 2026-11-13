#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
a='artifacts/validation/2026-11-14/solo_variation_recovery_r485_v1'
case_name="${1:?one registered variation required}"
case "$case_name" in start_left20_yaw_plus3) ;; *) exit 2;; esac
receipt="$a/cases/$case_name/execution_v1"
test ! -e "$receipt/stop_helper.exit"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
set +e
python3 "$a/stop_owned.py" "$case_name" > "$receipt/stop_helper.log" 2>&1
rc=$?
set -e
printf '%s\n' "$rc" > "$receipt/stop_helper.exit"
cat "$receipt/stop_helper.log"
exit "$rc"
