#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-23/hardware_accel_v1'
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/scripts:$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy:$PWD/tests"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$out/source_hashes_before.txt"
tar -czf "$out/source_snapshot.tar.gz" scripts/audit_hardware_capture.py tests/test_hardware_capture_accel.py tests/test_hardware_capture_audit.py config/tests/hardware_capture_accel_v2.jsonl src/arena_vehicle_interface/arena_vehicle_interface/drive_feedback.py
python3 -m pytest -q -p no:cacheprovider tests/test_hardware_capture_audit.py tests/test_hardware_capture_accel.py > "$out/tests.log" 2>&1
test_rc=$?
printf '%s\n' "$test_rc" > "$out/tests_exit.txt"
if [ "$test_rc" -eq 0 ]; then
  python3 scripts/audit_hardware_capture.py config/tests/hardware_capture_accel_v2.jsonl --output "$out/report.json" > "$out/audit.log" 2>&1
  audit_rc=$?
else
  audit_rc=125
fi
printf '%s\n' "$audit_rc" > "$out/audit_exit.txt"
find src scripts tests config -type f ! -path '*/__pycache__/*' -print0 | sort -z | xargs -0 sha256sum > "$out/source_hashes_after.txt"
diff -u "$out/source_hashes_before.txt" "$out/source_hashes_after.txt" > "$out/source_hash_diff.txt"
diff_rc=$?
if [ "$test_rc" -ne 0 ] || [ "$audit_rc" -ne 0 ] || [ "$diff_rc" -ne 0 ]; then exit 1; fi
