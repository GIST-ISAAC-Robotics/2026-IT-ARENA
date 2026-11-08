#!/usr/bin/env bash
# 사후 보조 점검(판정 비변경). evaluate.py 결과가 먼저 있어야 한다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-09/p22_sensor_preview_v1
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/sensitivity.py" > "$A/sensitivity.log" 2>&1
code=$?
echo "$code" > "$A/sensitivity.exit"
exit "$code"
