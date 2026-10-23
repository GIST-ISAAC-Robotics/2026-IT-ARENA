#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
test ! -e artifacts/validation/2026-10-23/sensor_probe_ros_v8/delivery_tests.log
python3 -B -m pytest -q -p no:cacheprovider tests/test_sensor_probe_delivery.py \
 > artifacts/validation/2026-10-23/sensor_probe_ros_v8/delivery_tests.log 2>&1
python3 -B artifacts/validation/2026-10-23/sensor_probe_ros_v8/inspect_delivery.py
