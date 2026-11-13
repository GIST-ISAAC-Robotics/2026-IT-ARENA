#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/solo_current_mask_r463_v1'
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="$PWD/stage/src/arena_autonomy:$PWD/stage/src/arena_vehicle_interface"
set +e
python3 -B -m pytest -q -p no:cacheprovider test_current_structure_shadow.py > pure_tests_v1.log 2>&1
code=$?
printf '%s\n' "$code" > pure_tests_v1.exit
cat pure_tests_v1.log
if [ "$code" -ne 0 ]; then exit "$code"; fi
python3 -B verify_counterexamples.py > verification_v2.log 2>&1
code=$?
printf '%s\n' "$code" > verification_v2.exit
tail -c 7000 verification_v2.log
exit "$code"
