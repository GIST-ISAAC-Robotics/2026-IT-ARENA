#!/usr/bin/env bash
# 1단계(센서 전용) 관측. 인자: r11 또는 r27
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-09/p22_sensor_preview_v1
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/observe_run.py" "$1" > "$A/observe_$1.log" 2>&1
code=$?
echo "$code" > "$A/observe_$1.exit"
exit "$code"
