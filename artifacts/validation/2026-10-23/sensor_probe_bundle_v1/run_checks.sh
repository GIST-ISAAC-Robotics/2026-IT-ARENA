#!/usr/bin/env bash
set -eo pipefail
cd -- /mnt/c/Users/Jinhyeong/Documents/ChatGPT/'IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
evidence=artifacts/validation/2026-10-23/sensor_probe_bundle_v1
test ! -e "$evidence/checks_exit.txt"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_before.sha256"
tar --exclude=__pycache__ -czf "$evidence/source_snapshot.tar.gz" src scripts tests config
finish() {
  result=$?
  trap - EXIT
  find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$evidence/sources_after.sha256"
  diff -u "$evidence/sources_before.sha256" "$evidence/sources_after.sha256" > "$evidence/source_hash_diff.txt" || result=1
  printf '%s\n' "$result" > "$evidence/checks_exit.txt"
  exit "$result"
}
trap finish EXIT
python3 -B -m pytest -q -p no:cacheprovider tests/test_sensor_recording_profile.py \
 tests/test_sensor_probe_audit.py tests/test_rosbag_writer_compat.py tests/test_sensor_probe_bundle.py \
 tests/test_bag_contract.py tests/test_recording_guard.py tests/test_bag_reader_compat.py \
 tests/test_jetson_bundle_dependencies.py tests/test_follow_profile_bundle.py \
 --junitxml="$evidence/junit.xml" > "$evidence/tests.log" 2>&1
python3 -B scripts/validate_sensor_probe.py --cases system \
 --output artifacts/validation/2026-10-23/sensor_probe_ros_v6 > "$evidence/ros.log" 2>&1
python3 -B scripts/prepare_jetson_bundle.py --sensor-probe-tools-only \
 --archive build/sensor_probe_tools_20261023_v1.tar.gz --manifest "$evidence/manifest.json" > "$evidence/package.log" 2>&1
python3 -B scripts/verify_sensor_probe_bundle.py build/sensor_probe_tools_20261023_v1.tar.gz \
 "$evidence/manifest.json" --output "$evidence/verification.json" > "$evidence/verification.log" 2>&1
