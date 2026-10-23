#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_formats_checks_v3
test ! -e "$evidence/checks_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/checks_exit.txt"; exit "$result"; }
trap finish EXIT
sha256sum scripts/validate_sensor_probe.py scripts/record_sensor_bag.py scripts/audit_sensor_probe.py \
 scripts/sensor_recording_profile.py scripts/rosbag_writer_compat.py tests/test_sensor_probe_formats.py > "$evidence/sources_before.txt"
python3 -B -m pytest -q -p no:cacheprovider tests/test_sensor_probe_formats.py tests/test_sensor_probe_audit.py \
 tests/test_sensor_probe_timing.py tests/test_sensor_recording_profile.py tests/test_rosbag_writer_compat.py \
 --junitxml="$evidence/junit.xml" > "$evidence/tests.log" 2>&1
python3 -B scripts/validate_sensor_probe.py --cases full_input \
 --output artifacts/validation/2026-10-23/sensor_probe_ros_v7 > "$evidence/ros.log" 2>&1
sha256sum scripts/validate_sensor_probe.py scripts/record_sensor_bag.py scripts/audit_sensor_probe.py \
 scripts/sensor_recording_profile.py scripts/rosbag_writer_compat.py tests/test_sensor_probe_formats.py > "$evidence/sources_after.txt"
cmp "$evidence/sources_before.txt" "$evidence/sources_after.txt"
python3 -B scripts/analyze_sensor_probe_timing.py \
 artifacts/validation/2026-10-23/sensor_probe_ros_v7/full_input/sensor_recording \
 --output "$evidence/timing.json" > "$evidence/timing.log" 2>&1
