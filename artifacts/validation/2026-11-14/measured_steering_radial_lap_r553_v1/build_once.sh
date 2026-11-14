#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
a='artifacts/validation/2026-11-14/measured_steering_radial_lap_r553_v1'
test ! -e "$a/build_result.json"
test ! -e "$a/prepared.json"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export PYTHONDONTWRITEBYTECODE=1
python3 -B "$a/build_receipt.py" before
set +e
colcon --log-base "$a/build_log" build --base-paths src --packages-select arena_autonomy --symlink-install --event-handlers console_direct+ > "$a/build.log" 2>&1
rc=$?
set -e
python3 -B "$a/build_receipt.py" after "$rc"
tail -n 8 "$a/build.log"
exit "$rc"
