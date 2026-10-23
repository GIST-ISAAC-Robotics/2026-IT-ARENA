#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_clock_checks_v2
test ! -e "$evidence/checks_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/checks_exit.txt"; exit "$result"; }
trap finish EXIT
python3 -B scripts/validate_sensor_probe.py --cases sim_regression \
 --output artifacts/validation/2026-10-23/sensor_probe_ros_v5 > "$evidence/ros.log" 2>&1
