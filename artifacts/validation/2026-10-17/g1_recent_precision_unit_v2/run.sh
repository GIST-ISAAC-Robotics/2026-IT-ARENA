#!/usr/bin/env bash
set -eo pipefail
ROOT='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
ARTIFACT="$ROOT/artifacts/validation/2026-10-17/g1_recent_precision_unit_v2"
cd "$ROOT"
mkdir -p "$ARTIFACT"
find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ARTIFACT/source_hashes_before.sha256"
tar -czf "$ARTIFACT/source_snapshot_before.tar.gz" src scripts tests config
finish() {
  result=$?
  find src scripts tests config -type f -print0 | sort -z | xargs -0 sha256sum > "$ARTIFACT/source_hashes_after.sha256"
  if diff -u "$ARTIFACT/source_hashes_before.sha256" "$ARTIFACT/source_hashes_after.sha256" > "$ARTIFACT/source_hash_diff.txt"; then echo SOURCE_UNCHANGED > "$ARTIFACT/source_hash_status.txt"; else echo SOURCE_CHANGED > "$ARTIFACT/source_hash_status.txt"; fi
  echo "EXIT_STATUS=$result" > "$ARTIFACT/runner_status.txt"
}
trap finish EXIT
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$ROOT/src/arena_autonomy:$ROOT/src/arena_vehicle_interface:$ROOT/scripts:$ROOT/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
python3 -B -m pytest -q \
  tests/test_observation_support.py \
  tests/test_observation_lateral.py \
  tests/test_observation_depth_equivalence.py \
  tests/test_observation_sweep_precision.py \
  tests/test_recent_observation_follow.py \
  tests/test_recent_observation_safety.py \
  tests/test_recent_observation_pipeline.py \
  tests/test_corner_follow.py > "$ARTIFACT/pytest.log" 2>&1
