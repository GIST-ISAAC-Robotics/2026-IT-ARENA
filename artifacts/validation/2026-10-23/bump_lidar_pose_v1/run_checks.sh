#!/usr/bin/env bash
set -o pipefail
source /opt/ros/jazzy/setup.bash
set -u
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-23/bump_lidar_pose_v1'
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$out/source_hashes_before.txt"
tar -czf "$out/source_snapshot.tar.gz" scripts/validate_bump_lidar_pose.py scripts/analyze_bump_geometry.py scripts/lidar_shutdown.py scripts/validate_start_signal_render.py tests/test_bump_lidar_pose.py tests/test_bump_geometry.py src/arena_description/config/vehicle.yaml src/arena_description/config/b0183_c1.yaml src/arena_description/models/arena_car/model.sdf.xacro
python3 -m pytest -q -p no:cacheprovider tests/test_bump_lidar_pose.py tests/test_bump_geometry.py > "$out/tests.log" 2>&1
test_rc=$?
printf '%s\n' "$test_rc" > "$out/tests_exit.txt"
if [ "$test_rc" -eq 0 ]; then
  python3 scripts/validate_bump_lidar_pose.py --geometry-report artifacts/validation/2026-10-23/bump_geometry_v3/report.json --output "$out/result" --timeout 90 > "$out/run.log" 2>&1
  run_rc=$?
else
  run_rc=125
fi
printf '%s\n' "$run_rc" > "$out/run_exit.txt"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$out/source_hashes_after.txt"
diff -u "$out/source_hashes_before.txt" "$out/source_hashes_after.txt" > "$out/source_hash_diff.txt"
diff_rc=$?
if [ "$test_rc" -ne 0 ] || [ "$run_rc" -ne 0 ] || [ "$diff_rc" -ne 0 ]; then exit 1; fi
