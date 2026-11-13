#!/usr/bin/env bash
set -euo pipefail
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/boundary_transient_r464_v1'
export PYTHONDONTWRITEBYTECODE=1
set +e
python3 -B run_native_bundle.py > execution_v1/bundle_v3.log 2>&1
code=$?
printf '%s\n' "$code" > execution_v1/bundle_v3.exit
cat execution_v1/bundle_v3.log
exit "$code"
