#!/usr/bin/env bash
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-08/p21_slow_observation_v1
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/analyze.py" > "$A/analysis.log" 2>&1
code=$?
echo "$code" > "$A/analysis.exit"
exit "$code"
