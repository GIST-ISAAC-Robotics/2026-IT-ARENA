#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/writer_cleanup_fix_v1
test ! -e "$evidence/run_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/run_exit.txt"; exit "$result"; }
trap finish EXIT
python3 -B -m pytest -q -p no:cacheprovider tests/test_rosbag_writer_compat.py \
 tests/test_recording_guard.py tests/test_sensor_recording_profile.py tests/test_sensor_probe_audit.py \
 tests/test_sensor_probe_preflight.py tests/test_sensor_probe_timing.py tests/test_sensor_probe_bundle.py \
 tests/test_jetson_bundle_dependencies.py tests/test_follow_profile_bundle.py tests/test_bag_contract.py \
 tests/test_bag_reader_compat.py tests/test_sensor_probe_delivery.py \
 --junitxml="$evidence/junit.xml" > "$evidence/tests.log" 2>&1
python3 -B scripts/validate_sensor_probe_writer.py --output "$evidence/results" > "$evidence/run.log" 2>&1
