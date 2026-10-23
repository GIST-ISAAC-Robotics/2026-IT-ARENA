#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_bundle_v2
test ! -e "$evidence/checks_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/checks_exit.txt"; exit "$result"; }
trap finish EXIT
python3 -B -m pytest -q -p no:cacheprovider tests/test_sensor_recording_profile.py \
 tests/test_sensor_probe_audit.py tests/test_rosbag_writer_compat.py tests/test_sensor_probe_bundle.py \
 tests/test_bag_contract.py tests/test_recording_guard.py tests/test_bag_reader_compat.py \
 tests/test_jetson_bundle_dependencies.py tests/test_follow_profile_bundle.py \
 --junitxml="$evidence/junit.xml" > "$evidence/tests.log" 2>&1
python3 -B scripts/prepare_jetson_bundle.py --sensor-probe-tools-only \
 --archive build/sensor_probe_tools_20261023_v2.tar.gz --manifest "$evidence/manifest.json" > "$evidence/package.log" 2>&1
python3 -B scripts/verify_sensor_probe_bundle.py build/sensor_probe_tools_20261023_v2.tar.gz \
 "$evidence/manifest.json" --output "$evidence/verification.json" > "$evidence/verification.log" 2>&1
