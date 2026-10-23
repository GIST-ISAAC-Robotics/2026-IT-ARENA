#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_clock_checks_v1
test ! -e "$evidence/checks_exit.txt"
python3 -B scripts/validate_sensor_probe.py --cases sim_valid sim_regression \
 --output artifacts/validation/2026-10-23/sensor_probe_ros_v4 > "$evidence/ros.log" 2>&1
printf '0\n' > "$evidence/checks_exit.txt"
