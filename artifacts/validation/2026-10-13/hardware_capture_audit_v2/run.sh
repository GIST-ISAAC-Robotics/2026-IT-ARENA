#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/hardware_capture_audit_v2
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_hardware_capture_audit.py tests/test_motor_encoder_validation.py tests/test_actuation_contract.py tests/test_follow_profile_bundle.py > "$base/tests.log" 2>&1
tail -10 "$base/tests.log"
python3 -B "$base/validate_matrix.py" > "$base/matrix.log" 2>&1
tail -1 "$base/matrix.log"
python3 -B scripts/audit_hardware_capture.py config/tests/hardware_capture_nominal_v1.jsonl --output "$base/legacy_example_report.json" > "$base/legacy_example.log" 2>&1
tail -1 "$base/legacy_example.log"
