#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="$PWD/src/arena_vehicle_interface:$PWD/src/arena_autonomy:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
base=artifacts/validation/2026-10-15/actuation_conformance_v1
test ! -e "$base/tests.log"
python3 -B -m pytest -q tests/test_actuation_conformance.py tests/test_actuation_contract.py tests/test_motor_encoder_validation.py > "$base/tests.log" 2>&1
python3 -B scripts/actuation_conformance.py --output "$base/export" > "$base/export.log" 2>&1
tail -3 "$base/tests.log"
tail -1 "$base/export.log"
