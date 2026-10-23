#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_preflight_v1
test ! -e "$evidence/run_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/run_exit.txt"; exit "$result"; }
trap finish EXIT
sha256sum scripts/sensor_probe_preflight.py tests/test_sensor_probe_preflight.py > "$evidence/sources_before.txt"
python3 -B -m pytest -q -p no:cacheprovider tests/test_sensor_probe_preflight.py tests/test_rosbag_writer_compat.py \
 tests/test_sensor_recording_profile.py --junitxml="$evidence/junit.xml" > "$evidence/tests.log" 2>&1
python3 -B scripts/sensor_probe_preflight.py --capture "$evidence/uncreated_capture" --clock-mode system \
 --sensor-topic /scan --sensor-topic /imu/data --sensor-topic /drive_motor/encoder \
 --sensor-topic /camera/color/image_raw --sensor-topic /camera/color/camera_info \
 --output "$evidence/report.json" > "$evidence/run.log" 2>&1
test ! -e "$evidence/uncreated_capture"
sha256sum scripts/sensor_probe_preflight.py tests/test_sensor_probe_preflight.py > "$evidence/sources_after.txt"
cmp "$evidence/sources_before.txt" "$evidence/sources_after.txt"
