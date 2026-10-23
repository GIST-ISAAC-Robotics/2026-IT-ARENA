#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-23/bump_geometry_v2'
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy:$PWD/tests"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$out/source_hashes_before.txt"
tar -czf "$out/source_snapshot.tar.gz" scripts/analyze_bump_geometry.py tests/test_bump_geometry.py src/arena_description/config/vehicle.yaml src/arena_description/config/b0183_c1.yaml src/arena_description/models/arena_car/model.sdf.xacro
python3 -m pytest -q tests/test_bump_geometry.py > "$out/tests.log" 2>&1
test_rc=$?
printf '%s\n' "$test_rc" > "$out/tests_exit.txt"
if [ "$test_rc" -eq 0 ]; then
  python3 scripts/analyze_bump_geometry.py --output "$out/report.json" > "$out/study.log" 2>&1
  study_rc=$?
else
  study_rc=125
fi
printf '%s\n' "$study_rc" > "$out/study_exit.txt"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$out/source_hashes_after.txt"
diff -u "$out/source_hashes_before.txt" "$out/source_hashes_after.txt" > "$out/source_hash_diff.txt"
diff_rc=$?
if [ "$test_rc" -ne 0 ] || [ "$study_rc" -ne 0 ] || [ "$diff_rc" -ne 0 ]; then exit 1; fi
