#!/usr/bin/env bash
set -eo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
artifact='artifacts/validation/2026-11-14/solo_finish_r481_v1/integration_v1'
set +e
python3 -B "$artifact/run_common.py" > "$artifact/tests.log" 2>&1
result=$?
set -e
printf '%s\n' "$result" > "$artifact/tests.exit"
cat "$artifact/tests.log"
exit "$result"
