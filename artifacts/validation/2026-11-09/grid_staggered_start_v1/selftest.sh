#!/usr/bin/env bash
# 평가기 코드 시험: 11/8 기준 실행 산출물을 읽기 전용으로 사용한다(새 주행 아님). 각본이 달라 scheduled_starts 실패가 기대값이다.
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." || exit 2
A=artifacts/validation/2026-11-09/grid_staggered_start_v1
B=artifacts/validation/2026-11-08/grid_pack_newcourse_v1
D=$B/five_vehicle_grid
bash artifacts/validation/2026-11-01/turn_preview_v1/environment.sh python3 -B "$A/assess.py" --selftest "$D/run1" "$D/surface_run1.json" "$D/postflight_integrity_run1.json" "$D/postflight_processes_run1.txt" "$B/geometry.json" "$A/selftest_baseline_assessment.json" > "$A/selftest.log" 2>&1
r=$?; printf '%s\n' "$r" > "$A/selftest.exit"; exit 0
