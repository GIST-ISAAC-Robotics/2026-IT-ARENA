#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-23/lidar_point_review_v1'
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/tests"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$out/source_hashes_before.txt"
tar -czf "$out/source_snapshot.tar.gz" src/arena_autonomy/arena_autonomy/lidar_point_review.py src/arena_autonomy/arena_autonomy/lidar_motion.py tests/test_lidar_point_review.py tests/test_lidar_motion.py
python3 -m pytest -q -p no:cacheprovider tests/test_lidar_point_review.py tests/test_lidar_motion.py > "$out/tests.log" 2>&1
test_rc=$?
printf '%s\n' "$test_rc" > "$out/tests_exit.txt"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$out/source_hashes_after.txt"
diff -u "$out/source_hashes_before.txt" "$out/source_hashes_after.txt" > "$out/source_hash_diff.txt"
diff_rc=$?
if [ "$test_rc" -ne 0 ] || [ "$diff_rc" -ne 0 ]; then exit 1; fi
