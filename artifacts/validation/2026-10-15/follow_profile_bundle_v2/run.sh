#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/follow_profile_bundle_v2
archive=build/jetson_g1_profile_20261015_v2.tar.gz
test ! -e "$base/regression.log"
python3 -B -m pytest -q tests/test_follow_occupancy_audit.py tests/test_follow_profile.py tests/test_follow_profile_bundle.py > "$base/regression.log" 2>&1
tail -4 "$base/regression.log"
python3 -B scripts/prepare_jetson_bundle.py --follow-profile-only --archive "$archive" --manifest "$base/manifest.json" \
  --follow-trace artifacts/validation/2026-10-13/g1_per_ray_v1/side_left_uniform_s0/follow_decisions.jsonl \
  --follow-trace artifacts/validation/2026-10-15/g1_stress_remaining_v1/first_curve_delay_s0/follow_decisions.jsonl > "$base/build.log" 2>&1
python3 -B scripts/verify_follow_profile_bundle.py "$archive" "$base/manifest.json" --output "$base/verification.json"
