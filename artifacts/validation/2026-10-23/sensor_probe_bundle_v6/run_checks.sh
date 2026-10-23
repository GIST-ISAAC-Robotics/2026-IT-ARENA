#!/usr/bin/env bash
set -eo pipefail
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_bundle_v6
test ! -e "$evidence/checks_exit.txt"
finish() { result=$?; trap - EXIT; printf '%s\n' "$result" > "$evidence/checks_exit.txt"; exit "$result"; }
trap finish EXIT
python3 -B -m pytest -q -p no:cacheprovider tests/test_sensor_recording_profile.py \
 tests/test_sensor_probe_audit.py tests/test_rosbag_writer_compat.py tests/test_sensor_probe_bundle.py \
 tests/test_bag_contract.py tests/test_recording_guard.py tests/test_bag_reader_compat.py \
 tests/test_jetson_bundle_dependencies.py tests/test_follow_profile_bundle.py \
 tests/test_sensor_probe_timing.py tests/test_sensor_probe_preflight.py tests/test_sensor_probe_delivery.py tests/test_sensor_probe_inputs.py tests/test_sensor_probe_session.py \
 --junitxml="$evidence/junit.xml" > "$evidence/tests.log" 2>&1
python3 -B scripts/prepare_jetson_bundle.py --sensor-probe-tools-only \
 --archive build/sensor_probe_tools_20261023_v6.tar.gz --manifest "$evidence/manifest.json" > "$evidence/package.log" 2>&1
python3 -B scripts/verify_sensor_probe_bundle.py build/sensor_probe_tools_20261023_v6.tar.gz \
 "$evidence/manifest.json" --output "$evidence/verification.json" > "$evidence/verification.log" 2>&1
python3 -B scripts/verify_sensor_probe_bundle.py build/sensor_probe_tools_20261023_v2.tar.gz \
 artifacts/validation/2026-10-23/sensor_probe_bundle_v2/manifest.json \
 --output "$evidence/legacy_verification.json" > "$evidence/legacy_verification.log" 2>&1
python3 -B scripts/verify_sensor_probe_bundle.py build/sensor_probe_tools_20261023_v3.tar.gz \
 artifacts/validation/2026-10-23/sensor_probe_bundle_v3/manifest.json \
 --output "$evidence/previous_verification.json" > "$evidence/previous_verification.log" 2>&1
python3 -B "$evidence/check_isolated.py" > "$evidence/isolation.log" 2>&1
