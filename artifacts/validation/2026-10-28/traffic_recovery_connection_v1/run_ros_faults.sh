#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONDONTWRITEBYTECODE=1
BASE="$PWD/artifacts/validation/2026-10-28/traffic_recovery_connection_v1"
OLD="$PWD/artifacts/validation/2026-10-28/traffic_residual_v1"
export PYTHONPATH="$BASE:$OLD:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
test ! -e "$BASE/build.log"
set +e
colcon build --packages-select arena_autonomy arena_bringup --symlink-install --event-handlers console_direct+ > "$BASE/build.log" 2>&1
code=$?
printf '%s\n' "$code" > "$BASE/build.exit"
if [ "$code" -ne 0 ]; then exit "$code"; fi
source install/local_setup.bash
/usr/bin/python3 -B scripts/validate_sensor_input_ros.py --output "$BASE/ros_faults_v1" --partial-observation-policy bounded_local_v4 --traffic-recovery-trial > "$BASE/ros_faults.log" 2>&1
code=$?
printf '%s\n' "$code" > "$BASE/ros_faults.exit"
exit "$code"
