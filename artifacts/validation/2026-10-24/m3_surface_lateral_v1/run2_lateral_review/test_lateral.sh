#!/usr/bin/env bash
set -uo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out='artifacts/validation/2026-10-24/m3_surface_lateral_v1/run2_lateral_review'
set +e
PYTHONPATH=scripts:src/arena_autonomy:src/arena_vehicle_interface python3 -B -m pytest -q -p no:cacheprovider --junitxml="$out/junit.xml" tests/test_lateral_intent.py 2>&1 | tee "$out/pytest.log"
rc=${PIPESTATUS[0]}
printf 'pytest_exit_code=%s\n' "$rc" | tee -a "$out/pytest.log"
exit "$rc"