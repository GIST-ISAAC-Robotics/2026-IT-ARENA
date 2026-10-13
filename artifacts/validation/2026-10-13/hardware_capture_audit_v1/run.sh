#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-13/hardware_capture_audit_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_hardware_capture_audit.py tests/test_motor_encoder_validation.py tests/test_actuation_contract.py > "$base/tests.log" 2>&1
tail -10 "$base/tests.log"
python3 -B scripts/audit_hardware_capture.py config/tests/hardware_capture_nominal_v1.jsonl --output "$base/example_report.json" > "$base/example.log" 2>&1
tail -5 "$base/example.log"
