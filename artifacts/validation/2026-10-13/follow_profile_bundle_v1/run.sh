#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/follow_profile_bundle_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_follow_profile_bundle.py tests/test_jetson_bundle_dependencies.py tests/test_hardware_capture_audit.py > "$base/tests.log" 2>&1
tail -10 "$base/tests.log"
python3 -B scripts/prepare_jetson_bundle.py --follow-profile-only --follow-trace artifacts/validation/2026-10-13/gapped_boundary_v3/side_left_s0/follow_decisions.jsonl --follow-trace artifacts/validation/2026-10-13/g1_candidate_regression_v1/initial_s0/follow_decisions.jsonl --archive build/jetson_g1_profile_20261013_v1.tar.gz --manifest "$base/manifest.json" > "$base/bundle.log" 2>&1
tail -3 "$base/bundle.log"
python3 -B scripts/verify_follow_profile_bundle.py build/jetson_g1_profile_20261013_v1.tar.gz "$base/manifest.json" --output "$base/verification.json" > "$base/verification.log" 2>&1
tail -3 "$base/verification.log"
